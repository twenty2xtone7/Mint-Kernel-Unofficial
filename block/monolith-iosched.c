/*
 * Monolith I/O scheduler
 *
 * Ultimate unified scheduler with self-tuning AI:
 *   - Auto-expiry: tracks read latency EMA, sets expiry = 4x observed
 *   - Adaptive starved: measures write interference on reads
 *   - Burst detection: instant read-priority mode on app launch
 *   - Auto-thinktime: median of recent idle periods
 *   - Self-calibrating via continuous feedback loop
 *
 * Copyright (C) 2022 monolith
 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/rbtree.h>
#include <linux/math64.h>

enum {
	MD_SR = 0, MD_SW = 1, MD_AR = 2, MD_AW = 3, MD_NR_QUEUES,
};

/* Base defaults (floor values the AI tunes upward from) */
static const int base_sr_expire	= HZ / 100;	/* 10ms floor */
static const int base_sw_expire	= HZ / 20;	/* 50ms */
static const int base_ar_expire	= HZ / 50;	/* 20ms */
static const int base_aw_expire	= HZ / 10;	/* 100ms */
static const int base_starved	= 12;
static const int base_ratio	= 6;
static const int base_bcount	= 3;
static const int base_fifo_batch	= 4;
static const int tune_interval	= 128;		/* retune every N dispatches */

/* ==================================================== */
/* Self-tuning data structures                          */
/* ==================================================== */
struct md_tuner {
	/* Read latency tracking */
	u64 rd_lat_ema;		/* ns, exp moving average */
	u64 rd_lat_peak;	/* ns, peak this window */
	u32 rd_lat_cnt;		/* samples this window */

	/* Write interference tracking */
	u64 rd_lat_nowrite;	/* read ema, no concurrent write */
	u64 rd_lat_withwrite;	/* read ema, with concurrent write */
	u64 rd_lat_w_ewma;	/* write ema for comparison */
	u32 interference_ratio;	/* withwrite / nowrite * 256 */

	/* Thinktime ring buffer */
	u32 think_ring[16];	/* jiffies */
	u8  think_idx;
	u8  think_count;

	/* Burst detection */
	u64 burst_deadline;	/* jiffies */
	u32 burst_reqs;		/* reads since burst started */
	u32 burst_seq;		/* burst ID */

	/* Dispatch counter */
	u32 ops;

	/* Computed adaptive values (jiffies / count) */
	int expire_sr, expire_sw, expire_ar, expire_aw;
	int starved;
	int thinktime;
	int fifo_batch;
};

struct md_data {
	struct list_head fifo[MD_NR_QUEUES];
	int fifo_expire[MD_NR_QUEUES];

	int writes_starved;
	int sync_ratio;
	int batch_count;
	int fifo_batch;

	unsigned int starved;
	int batch_remaining;
	int ratio_counter;
	int batch_counter;

	unsigned long last_dispatch;
	int last_dir;
	unsigned int seen_idle;

	/* Adaptive tuner */
	struct md_tuner tn;
	int front_merges;
	struct rb_root sort[MD_NR_QUEUES];
	struct request *next_rq[MD_NR_QUEUES];
};

/* ==================================================== */
/* Helpers                                               */
/* ==================================================== */
static int md_rq_idx(struct request *rq)
{
	int ddir = rq_data_dir(rq);
	return ddir == READ ? (rq_is_sync(rq) ? MD_SR : MD_AR)
			    : (rq_is_sync(rq) ? MD_SW : MD_AW);
}

static struct rb_root *md_rb_root(struct md_data *md, struct request *rq)
{
	return &md->sort[md_rq_idx(rq)];
}

static struct request *md_latter(struct request *rq)
{
	struct rb_node *n = rb_next(&rq->rb_node);
	return n ? rb_entry_rq(n) : NULL;
}

static void md_add_rq_rb(struct md_data *md, struct request *rq)
{
	elv_rb_add(md_rb_root(md, rq), rq);
}

static void md_del_rq_rb(struct md_data *md, struct request *rq)
{
	int idx = md_rq_idx(rq);
	if (md->next_rq[idx] == rq)
		md->next_rq[idx] = md_latter(rq);
	elv_rb_del(md_rb_root(md, rq), rq);
}

/* ==================================================== */
/* Self-tuning engine                                    */
/* ==================================================== */
static void md_retune(struct md_data *md)
{
	struct md_tuner *tn = &md->tn;

	/* Auto-expiry: sync read = 4x observed latency, floor to base */
	if (tn->rd_lat_ema) {
		int sr_ms = (int)div64_u64(tn->rd_lat_ema * 4, NSEC_PER_MSEC);
		sr_ms = clamp(sr_ms, jiffies_to_msecs(base_sr_expire), 80);
		tn->expire_sr = msecs_to_jiffies(sr_ms);

		/* Other expiries scale relative to SR */
		tn->expire_sw = msecs_to_jiffies(clamp(sr_ms * 5, 50, 200));
		tn->expire_ar = msecs_to_jiffies(clamp(sr_ms * 2, 20, 80));
		tn->expire_aw = msecs_to_jiffies(clamp(sr_ms * 10, 100, 400));

		md->fifo_expire[MD_SR] = tn->expire_sr;
		md->fifo_expire[MD_SW] = tn->expire_sw;
		md->fifo_expire[MD_AR] = tn->expire_ar;
		md->fifo_expire[MD_AW] = tn->expire_aw;
	}

	/* Adaptive starved: if writes don't hurt reads, starve less */
	if (tn->rd_lat_nowrite && tn->rd_lat_withwrite &&
	    tn->rd_lat_withwrite > tn->rd_lat_nowrite) {
		tn->interference_ratio = (u32)div64_u64(
			tn->rd_lat_withwrite * 256, tn->rd_lat_nowrite);
		/* ratio > 256 means writes hurt reads */
		if (tn->interference_ratio > 320)
			tn->starved = min(tn->starved + 1, 20);
		else if (tn->interference_ratio < 192)
			tn->starved = max(tn->starved - 1, 4);
	}
	md->writes_starved = tn->starved;

	/* Auto-thinktime: median of recent idle periods */
	if (tn->think_count >= 4) {
		u32 sorted[16];
		int i, j;
		memcpy(sorted, tn->think_ring, sizeof(tn->think_ring));
		/* Bubble sort 16 elements is fine once per 128 dispatches */
		for (i = 0; i < tn->think_count - 1; i++)
			for (j = i + 1; j < tn->think_count; j++)
				if (sorted[i] > sorted[j])
					swap(sorted[i], sorted[j]);
		tn->thinktime = sorted[tn->think_count / 2];
		tn->thinktime = clamp(tn->thinktime, 1U, 10U);
	}

	/* Auto fifo_batch: scale with device speed */
	if (tn->rd_lat_ema) {
		u64 lat_us = tn->rd_lat_ema / NSEC_PER_USEC;
		if (lat_us < 500)
			tn->fifo_batch = clamp(tn->fifo_batch + 1, 2, 16);
		else if (lat_us > 2000)
			tn->fifo_batch = clamp(tn->fifo_batch - 1, 1, 8);
	}
	md->fifo_batch = tn->fifo_batch;
}

/* Record thinktime sample */
static void md_sample_thinktime(struct md_data *md)
{
	struct md_tuner *tn = &md->tn;
	tn->think_ring[tn->think_idx] = tn->thinktime;
	tn->think_idx = (tn->think_idx + 1) % ARRAY_SIZE(tn->think_ring);
	if (tn->think_count < ARRAY_SIZE(tn->think_ring))
		tn->think_count++;
}

/* Record a read completion latency sample */
static void md_sample_latency(struct md_data *md, u64 lat_ns,
			      int writes_active)
{
	struct md_tuner *tn = &md->tn;

	/* Update EMA */
	if (tn->rd_lat_ema)
		tn->rd_lat_ema += (lat_ns - tn->rd_lat_ema) >> 3;
	else
		tn->rd_lat_ema = lat_ns;

	/* Track peak in window */
	if (lat_ns > tn->rd_lat_peak)
		tn->rd_lat_peak = lat_ns;

	/* Track write interference */
	if (writes_active) {
		if (tn->rd_lat_withwrite)
			tn->rd_lat_withwrite += (lat_ns - tn->rd_lat_withwrite) >> 2;
		else
			tn->rd_lat_withwrite = lat_ns;
	} else {
		if (tn->rd_lat_nowrite)
			tn->rd_lat_nowrite += (lat_ns - tn->rd_lat_nowrite) >> 2;
		else
			tn->rd_lat_nowrite = lat_ns;
	}

	tn->rd_lat_cnt++;
}

/* ==================================================== */
/* Core callbacks                                        */
/* ==================================================== */
static void md_add_request(struct request_queue *q, struct request *rq)
{
	struct md_data *md = q->elevator->elevator_data;
	int idx = md_rq_idx(rq);

	md_add_rq_rb(md, rq);
	rq->fifo_time = jiffies + md->fifo_expire[idx];
	list_add_tail(&rq->queuelist, &md->fifo[idx]);

	/* Burst detection: sync reads arriving close together */
	if (idx == MD_SR) {
		struct md_tuner *tn = &md->tn;
		if (!tn->burst_reqs || time_after(jiffies, tn->burst_deadline)) {
			tn->burst_seq++;
			if (tn->burst_reqs)
				tn->burst_deadline = jiffies + 4;
			tn->burst_reqs = 1;
		} else {
			tn->burst_reqs++;
			tn->burst_deadline = jiffies + 4;
		}
	}
}

static void md_remove_request(struct request_queue *q, struct request *rq)
{
	struct md_data *md = q->elevator->elevator_data;
	rq_fifo_clear(rq);
	md_del_rq_rb(md, rq);
}

static enum elv_merge
md_merge(struct request_queue *q, struct request **req, struct bio *bio)
{
	struct md_data *md = q->elevator->elevator_data;
	int idx = op_is_sync(bio->bi_opf)
		  ? (bio_data_dir(bio) == READ ? MD_SR : MD_SW)
		  : (bio_data_dir(bio) == READ ? MD_AR : MD_AW);
	struct request *__rq;

	if (md->front_merges) {
		__rq = elv_rb_find(&md->sort[idx], bio_end_sector(bio));
		if (__rq) {
			BUG_ON(bio_end_sector(bio) != blk_rq_pos(__rq));
			if (elv_bio_merge_ok(__rq, bio)) {
				*req = __rq;
				return ELEVATOR_FRONT_MERGE;
			}
		}
	}
	if (!list_empty(&md->fifo[idx])) {
		__rq = list_entry_rq(md->fifo[idx].prev);
		if (__rq && elv_bio_merge_ok(__rq, bio)) {
			*req = __rq;
			return ELEVATOR_BACK_MERGE;
		}
	}
	return ELEVATOR_NO_MERGE;
}

static void md_merged_request(struct request_queue *q,
			      struct request *req, enum elv_merge type)
{
	struct md_data *md = q->elevator->elevator_data;
	if (type == ELEVATOR_FRONT_MERGE) {
		elv_rb_del(&md->sort[md_rq_idx(req)], req);
		md_add_rq_rb(md, req);
	}
}

static void md_merged_requests(struct request_queue *q,
			       struct request *req, struct request *next)
{
	if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before((unsigned long)next->fifo_time,
				(unsigned long)req->fifo_time))
			list_move(&req->queuelist, &next->queuelist);
	}
	md_remove_request(q, next);
}

/* ==================================================== */
/* Thinktime                                             */
/* ==================================================== */
static void md_thinktime(struct md_data *md)
{
	if (md->last_dispatch &&
	    time_after(jiffies, md->last_dispatch + md->tn.thinktime)) {
		if (!md->seen_idle) {
			md->tn.thinktime = jiffies - md->last_dispatch;
			if (md->tn.thinktime > 1)
				md_sample_thinktime(md);
		}
		md->seen_idle = 1;
		md->starved = 0;
	}
}

/* ==================================================== */
/* Expiry flush                                          */
/* ==================================================== */
static int md_flush_expired(struct md_data *md, struct request_queue *q,
			    int idx)
{
	while (!list_empty(&md->fifo[idx])) {
		struct request *rq = rq_entry_fifo(md->fifo[idx].next);
		if (time_after_eq(jiffies, (unsigned long)rq->fifo_time)) {
			rq_fifo_clear(rq);
			md_del_rq_rb(md, rq);
			elv_dispatch_add_tail(q, rq);
			return 1;
		}
		break;
	}
	return 0;
}

/* ==================================================== */
/* Latency callback                                      */
/* ==================================================== */
static void md_completed_request(struct request_queue *q,
				 struct request *rq)
{
	struct md_data *md = q->elevator->elevator_data;
	u64 now, start, lat;
	int writes_active;

	if (rq_data_dir(rq) != READ || blk_rq_is_passthrough(rq))
		return;

	now = ktime_get_ns();
	start = rq_start_time_ns(rq);
	lat = start ? now - start : 0;

	writes_active = !list_empty(&md->fifo[MD_SW]) ||
			!list_empty(&md->fifo[MD_AW]);

	md_sample_latency(md, lat, writes_active);

	/* Retune periodically */
	if (++md->tn.rd_lat_cnt >= tune_interval) {
		md->tn.rd_lat_cnt = 0;
		md_retune(md);
	}
}

/* ==================================================== */
/* Dispatch                                              */
/* ==================================================== */
static int md_dispatch_requests(struct request_queue *q, int force)
{
	struct md_data *md = q->elevator->elevator_data;
	struct md_tuner *tn = &md->tn;
	struct request *rq;
	int i, in_burst;

	if (list_empty(&md->fifo[MD_SR]) && list_empty(&md->fifo[MD_SW]) &&
	    list_empty(&md->fifo[MD_AR]) && list_empty(&md->fifo[MD_AW]))
		return 0;

	md->tn.ops++;
	md_thinktime(md);
	in_burst = tn->burst_reqs >= 4 &&
		   !time_after(jiffies, tn->burst_deadline);

	/* Stage 1: Expiry flush */
	for (i = MD_SR; i <= MD_AW; i++)
		if (md_flush_expired(md, q, i))
			goto out;

	/* Stage 2: Burst mode — reads only */
	if (in_burst && !list_empty(&md->fifo[MD_SR])) {
		rq = rq_entry_fifo(md->fifo[MD_SR].next);
		goto dispatch_rq;
	}

	/* Stage 3: Sequential boost */
	if (md->seen_idle && md->last_dir >= 0 &&
	    !list_empty(&md->fifo[md->last_dir])) {
		md->seen_idle = 0;
		rq = rq_entry_fifo(md->fifo[md->last_dir].next);
		goto dispatch_rq;
	}
	md->seen_idle = 0;

	/* Stage 4: Anxiety interleaving */
	if (md->batch_counter < md->batch_count) {
		if (md->ratio_counter < md->sync_ratio) {
			if (md->starved < md->writes_starved ||
			    in_burst) {
				if (!list_empty(&md->fifo[MD_SR])) {
					rq = rq_entry_fifo(md->fifo[MD_SR].next);
					md->ratio_counter++;
					goto dispatch_rq;
				}
			}
			if (!list_empty(&md->fifo[MD_SW]) &&
			    !in_burst) {
				rq = rq_entry_fifo(md->fifo[MD_SW].next);
				if (!list_empty(&md->fifo[MD_SR]) &&
				    md->starved < md->writes_starved) {
					rq = rq_entry_fifo(md->fifo[MD_SR].next);
					md->ratio_counter++;
					goto dispatch_rq;
				}
				md->starved = 0;
				md->ratio_counter++;
				goto dispatch_rq;
			}
			md->ratio_counter = md->sync_ratio;
		}
		if (md->ratio_counter >= md->sync_ratio && !in_burst) {
			if (!list_empty(&md->fifo[MD_AR])) {
				rq = rq_entry_fifo(md->fifo[MD_AR].next);
				md->ratio_counter = 0;
				md->batch_counter++;
				goto dispatch_rq;
			}
			if (!list_empty(&md->fifo[MD_AW])) {
				if (!list_empty(&md->fifo[MD_SR]) &&
				    md->starved < md->writes_starved) {
					rq = rq_entry_fifo(md->fifo[MD_SR].next);
					md->ratio_counter = 0;
					md->batch_counter = 0;
					goto dispatch_rq;
				}
				md->ratio_counter = 0;
				md->batch_counter++;
				goto dispatch_rq;
			}
			md->ratio_counter = 0;
			md->batch_counter++;
		}
	}

	/* Stage 5: Priority fallback */
	if (in_burst || md->starved < md->writes_starved) {
		if (!list_empty(&md->fifo[MD_SR])) {
			rq = rq_entry_fifo(md->fifo[MD_SR].next);
			goto dispatch_rq;
		}
		if (!list_empty(&md->fifo[MD_AR])) {
			rq = rq_entry_fifo(md->fifo[MD_AR].next);
			goto dispatch_rq;
		}
	}
	if (!list_empty(&md->fifo[MD_SW])) {
		rq = rq_entry_fifo(md->fifo[MD_SW].next);
		md->starved = 0;
		goto dispatch_rq;
	}
	if (!list_empty(&md->fifo[MD_AW])) {
		rq = rq_entry_fifo(md->fifo[MD_AW].next);
		md->starved = 0;
		goto dispatch_rq;
	}
	if (!list_empty(&md->fifo[MD_SR])) {
		rq = rq_entry_fifo(md->fifo[MD_SR].next);
		goto dispatch_rq;
	}
	if (!list_empty(&md->fifo[MD_AR])) {
		rq = rq_entry_fifo(md->fifo[MD_AR].next);
		goto dispatch_rq;
	}
	return 0;

dispatch_rq:
	md->starved++;
	md->last_dispatch = jiffies;
	md->last_dir = md_rq_idx(rq);
	rq_fifo_clear(rq);
	md_del_rq_rb(md, rq);
	elv_dispatch_add_tail(q, rq);
out:
	if (md->batch_counter >= md->batch_count) {
		md->batch_counter = 0;
		md->ratio_counter = 0;
	}
	return 1;
}

/* ==================================================== */
/* Init / exit                                           */
/* ==================================================== */
static void md_exit_queue(struct elevator_queue *e)
{
	kfree(e->elevator_data);
}

static int md_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct md_data *md;
	struct elevator_queue *eq;
	int i;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	md = kzalloc_node(sizeof(*md), GFP_KERNEL, q->node);
	if (!md) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = md;

	for (i = 0; i < MD_NR_QUEUES; i++) {
		INIT_LIST_HEAD(&md->fifo[i]);
		md->sort[i] = RB_ROOT;
	}

	/* Apply base defaults */
	md->fifo_expire[MD_SR] = base_sr_expire;
	md->fifo_expire[MD_SW] = base_sw_expire;
	md->fifo_expire[MD_AR] = base_ar_expire;
	md->fifo_expire[MD_AW] = base_aw_expire;
	md->writes_starved = base_starved;
	md->sync_ratio = base_ratio;
	md->batch_count = base_bcount;
	md->fifo_batch = base_fifo_batch;
	md->last_dispatch = jiffies;
	md->last_dir = -1;
	md->front_merges = 1;

	/* Init self-tuning defaults */
	md->tn.expire_sr = base_sr_expire;
	md->tn.expire_sw = base_sw_expire;
	md->tn.expire_ar = base_ar_expire;
	md->tn.expire_aw = base_aw_expire;
	md->tn.starved = base_starved;
	md->tn.thinktime = 2;
	md->tn.fifo_batch = base_fifo_batch;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

/* ==================================================== */
/* sysfs                                                 */
/* ==================================================== */
static ssize_t md_var_show(int var, char *page)
{
	return sprintf(page, "%d\n", var);
}
static void md_var_store(int *var, const char *page)
{
	*var = simple_strtol((char *)page, NULL, 10);
}

#define SHOW_FN(__FUNC, __VAR, __CONV)					\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct md_data *md = e->elevator_data;				\
	int __data = __VAR;						\
	if (__CONV) __data = jiffies_to_msecs(__data);			\
	return md_var_show(__data, page);				\
}
SHOW_FN(md_sync_read_expire_show,	md->tn.expire_sr, 1);
SHOW_FN(md_sync_write_expire_show,	md->tn.expire_sw, 1);
SHOW_FN(md_async_read_expire_show,	md->tn.expire_ar, 1);
SHOW_FN(md_async_write_expire_show,	md->tn.expire_aw, 1);
SHOW_FN(md_writes_starved_show,		md->tn.starved, 0);
SHOW_FN(md_fifo_batch_show,		md->tn.fifo_batch, 0);
SHOW_FN(md_thinktime_show,		md->tn.thinktime, 0);
SHOW_FN(md_rd_lat_ema_show,		md->tn.rd_lat_ema / 1000, 0);
SHOW_FN(md_interference_show,		md->tn.interference_ratio, 0);
SHOW_FN(md_ops_show,			md->tn.ops, 0);
#undef SHOW_FN

#define STORE_FN(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page,	\
		      size_t count)					\
{									\
	struct md_data *md = e->elevator_data;				\
	int __data;							\
	md_var_store(&__data, (page));					\
	__data = clamp(__data, (MIN), (MAX));				\
	if (__CONV)							\
		*(__PTR) = msecs_to_jiffies(__data);			\
	else								\
		*(__PTR) = __data;					\
	return count;							\
}
STORE_FN(md_sync_read_expire_store,	&md->tn.expire_sr, 0, INT_MAX, 1);
STORE_FN(md_sync_write_expire_store,	&md->tn.expire_sw, 0, INT_MAX, 1);
STORE_FN(md_async_read_expire_store,	&md->tn.expire_ar, 0, INT_MAX, 1);
STORE_FN(md_async_write_expire_store,	&md->tn.expire_aw, 0, INT_MAX, 1);
STORE_FN(md_writes_starved_store,	&md->tn.starved, 0, INT_MAX, 0);
STORE_FN(md_fifo_batch_store,		&md->tn.fifo_batch, 0, INT_MAX, 0);
#undef SHOW_FN
#undef STORE_FN

#define MD_ATTR(name) \
	__ATTR(name, S_IRUGO | S_IWUSR, md_##name##_show, md_##name##_store)

static struct elv_fs_entry md_attrs[] = {
	MD_ATTR(sync_read_expire),
	MD_ATTR(sync_write_expire),
	MD_ATTR(async_read_expire),
	MD_ATTR(async_write_expire),
	MD_ATTR(writes_starved),
	MD_ATTR(fifo_batch),
	MD_ATTR(thinktime),
	__ATTR(rd_lat_ema, S_IRUGO, md_rd_lat_ema_show, NULL),
	__ATTR(interference, S_IRUGO, md_interference_show, NULL),
	__ATTR(ops, S_IRUGO, md_ops_show, NULL),
	__ATTR_NULL
};

static struct elevator_type iosched_md = {
	.ops.sq = {
		.elevator_merge_fn		= md_merge,
		.elevator_merged_fn		= md_merged_request,
		.elevator_merge_req_fn		= md_merged_requests,
		.elevator_dispatch_fn		= md_dispatch_requests,
		.elevator_add_req_fn		= md_add_request,
		.elevator_completed_req_fn	= md_completed_request,
		.elevator_former_req_fn		= elv_rb_former_request,
		.elevator_latter_req_fn		= elv_rb_latter_request,
		.elevator_init_fn		= md_init_queue,
		.elevator_exit_fn		= md_exit_queue,
	},

	.elevator_attrs = md_attrs,
	.elevator_name = "monolith",
	.elevator_owner = THIS_MODULE,
};

static int __init md_init(void)
{
	return elv_register(&iosched_md);
}

static void __exit md_exit(void)
{
	elv_unregister(&iosched_md);
}

module_init(md_init);
module_exit(md_exit);

MODULE_AUTHOR("monolith");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Monolith IO scheduler: unified scheduler with self-tuning AI");
MODULE_VERSION("2.0");
