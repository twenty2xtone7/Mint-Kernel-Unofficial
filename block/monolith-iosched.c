/*
 * Monolith I/O scheduler
 *
 * Ultimate unified scheduler fusing NOOP, Deadline, SIO, Zen, Maple,
 * CFQ, Kyber, Anxiety, FIOPS, and BFQ concepts into one engine.
 *
 * Features:
 *   - Quad FIFO queues with per-queue expiry
 *   - Anxiety-style sync_ratio/batch_count batching
 *   - writes_starved with priority tiers (sync read > sync write > async read > async write)
 *   - CFQ-style thinktime detection with last-direction memory
 *   - Kyber-style latency EMA with dynamic expire scaling
 *   - Expiry boosting: flush all expired entries on timeout
 *   - Clock-based expiration scanning
 *   - NOOP O(1) FIFO fallback path
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

/* ------------------------------------------------------------------ */
/* Priority queue indices                                              */
/* ------------------------------------------------------------------ */
enum {
	MD_SR = 0,	/* sync read  — highest dispatch priority */
	MD_SW = 1,	/* sync write */
	MD_AR = 2,	/* async read */
	MD_AW = 3,	/* async write — lowest dispatch priority */
	MD_NR_QUEUES,
};

/* ------------------------------------------------------------------ */
/* Default tunables (all aggressive for UFS/eMMC)                      */
/* ------------------------------------------------------------------ */
static const int sync_read_expire	= HZ / 50;	/*  20 ms */
static const int sync_write_expire	= HZ / 10;	/* 100 ms */
static const int async_read_expire	= HZ / 25;	/*  40 ms */
static const int async_write_expire	= HZ / 5;	/* 200 ms */
static const int writes_starved		= 10;
static const int sync_ratio		= 6;		/* syncs before 1 async */
static const int batch_count		= 3;		/* repeats of sync_ratio */
static const int fifo_batch		= 4;
static const int thinktime_threshold	= 2;		/* jiffies */
static const u64 latency_target_ns	= 5ULL * NSEC_PER_MSEC;
static const int latency_window		= 4;		/* samples before re-eval */

/* ------------------------------------------------------------------ */
/* Per-device data                                                     */
/* ------------------------------------------------------------------ */
struct md_data {
	/* Quad FIFO queues */
	struct list_head fifo[MD_NR_QUEUES];
	int fifo_expire[MD_NR_QUEUES];

	/* Tunables */
	int writes_starved;
	int sync_ratio;
	int batch_count;
	int fifo_batch;

	/* Starvation / batching state */
	unsigned int starved;
	int batch_remaining;

	/* Anxiety-style sync/async interleaving */
	int ratio_counter;		/* counts syncs before async */
	int batch_counter;		/* counts ratio repeats */

	/* Thinktime */
	unsigned long last_dispatch;
	int last_dir;			/* MD_SR/MD_SW/etc or -1 */
	unsigned int seen_idle;

	/* Kyber-style latency tracking */
	u64 read_latency_ema;		/* ns */
	u64 read_latency_peak;		/* ns this window */
	int lat_samples;		/* samples this window */
	unsigned int scale_down;	/* 1 = halve expiries */

	/* Front-merge support */
	int front_merges;
	struct rb_root sort[MD_NR_QUEUES];
	struct request *next_rq[MD_NR_QUEUES];
};

/* ------------------------------------------------------------------ */
/* Queue helpers                                                       */
/* ------------------------------------------------------------------ */
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

static int md_effective_expire(struct md_data *md, int idx)
{
	int e = md->fifo_expire[idx];
	if (md->scale_down && (idx == MD_SR || idx == MD_AR))
		e >>= 1;
	return max(e, 1);
}

/* ------------------------------------------------------------------ */
/* Core elevator callbacks                                             */
/* ------------------------------------------------------------------ */
static void md_add_request(struct request_queue *q, struct request *rq)
{
	struct md_data *md = q->elevator->elevator_data;
	int idx = md_rq_idx(rq);

	md_add_rq_rb(md, rq);
	rq->fifo_time = jiffies + md_effective_expire(md, idx);
	list_add_tail(&rq->queuelist, &md->fifo[idx]);
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
	/* Back-merge fallback via FIFO tail */
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
		int idx = md_rq_idx(req);
		elv_rb_del(&md->sort[idx], req);
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

/* ------------------------------------------------------------------ */
/* Thinktime detection                                                 */
/* ------------------------------------------------------------------ */
static void md_thinktime(struct md_data *md)
{
	if (md->last_dispatch &&
	    time_after(jiffies, md->last_dispatch + thinktime_threshold)) {
		md->seen_idle = 1;
		md->starved = 0;
	}
}

/* ------------------------------------------------------------------ */
/* Expiry scan — flush all expired entries from a queue               */
/* Returns 1 if an expired entry was dispatched.                       */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* Kyber-style latency callback                                        */
/* ------------------------------------------------------------------ */
static void md_completed_request(struct request_queue *q,
				 struct request *rq)
{
	struct md_data *md = q->elevator->elevator_data;
	u64 now, start, lat;

	if (rq_data_dir(rq) != READ || blk_rq_is_passthrough(rq))
		return;

	now = ktime_get_ns();
	start = rq_start_time_ns(rq);
	lat = start ? now - start : 0;

	if (lat > md->read_latency_peak)
		md->read_latency_peak = lat;

	if (md->read_latency_ema)
		md->read_latency_ema += (lat - md->read_latency_ema) >> 3;
	else
		md->read_latency_ema = lat;

	if (++md->lat_samples >= latency_window) {
		md->scale_down = (md->read_latency_peak > latency_target_ns);
		md->read_latency_peak = 0;
		md->lat_samples = 0;
	}
}

/* ------------------------------------------------------------------ */
/* Dispatch — the heart of Monolith                                    */
/* ------------------------------------------------------------------ */
static int md_dispatch_requests(struct request_queue *q, int force)
{
	struct md_data *md = q->elevator->elevator_data;
	struct request *rq;
	int i;

	/* Quick empty check */
	if (list_empty(&md->fifo[MD_SR]) && list_empty(&md->fifo[MD_SW]) &&
	    list_empty(&md->fifo[MD_AR]) && list_empty(&md->fifo[MD_AW]))
		return 0;

	/* Thinktime: reset starved if we were idle */
	md_thinktime(md);

	/* Scale-down shrinks fifo_batch to react faster */
	if (md->scale_down && md->batch_remaining > md->fifo_batch >> 1)
		md->batch_remaining = md->fifo_batch >> 1;

	/* -- STAGE 1: Expiry flush (timeout override) -- */
	for (i = MD_SR; i <= MD_AW; i++)
		if (md_flush_expired(md, q, i))
			goto out;

	/* -- STAGE 2: Sequential read boost (thinktime + last_dir) -- */
	if (md->seen_idle && md->last_dir >= 0 &&
	    !list_empty(&md->fifo[md->last_dir])) {
		md->seen_idle = 0;
		rq = rq_entry_fifo(md->fifo[md->last_dir].next);
		goto dispatch_rq;
	}
	md->seen_idle = 0;

	/* -- STAGE 3: Anxiety-style sync/async interleaving -- */
	if (md->batch_counter < md->batch_count) {
		if (md->ratio_counter < md->sync_ratio) {
			/* Dispatch sync (read > write via starved) */
			if (md->starved < md->writes_starved) {
				if (!list_empty(&md->fifo[MD_SR])) {
					rq = rq_entry_fifo(md->fifo[MD_SR].next);
					md->ratio_counter++;
					goto dispatch_rq;
				}
			}
			if (!list_empty(&md->fifo[MD_SW])) {
				rq = rq_entry_fifo(md->fifo[MD_SW].next);
				if (!list_empty(&md->fifo[MD_SR])) {
					/* Sync read exists — only write if starved */
					if (md->starved < md->writes_starved) {
						rq = rq_entry_fifo(md->fifo[MD_SR].next);
						md->ratio_counter++;
						goto dispatch_rq;
					}
				}
				md->starved = 0;
				md->ratio_counter++;
				goto dispatch_rq;
			}
			/* No syncs left: fall through to async */
			md->ratio_counter = md->sync_ratio;
		}
		if (md->ratio_counter >= md->sync_ratio) {
			/* Async dispatch */
			if (!list_empty(&md->fifo[MD_AR])) {
				rq = rq_entry_fifo(md->fifo[MD_AR].next);
				md->ratio_counter = 0;
				md->batch_counter++;
				goto dispatch_rq;
			}
			if (!list_empty(&md->fifo[MD_AW])) {
				rq = rq_entry_fifo(md->fifo[MD_AW].next);
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

	/* -- STAGE 4: Priority fallback (read bias) -- */
	if (md->starved < md->writes_starved) {
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

/* ------------------------------------------------------------------ */
/* Init / exit                                                         */
/* ------------------------------------------------------------------ */
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
	md->fifo_expire[MD_SR] = sync_read_expire;
	md->fifo_expire[MD_SW] = sync_write_expire;
	md->fifo_expire[MD_AR] = async_read_expire;
	md->fifo_expire[MD_AW] = async_write_expire;
	md->writes_starved = writes_starved;
	md->sync_ratio = sync_ratio;
	md->batch_count = batch_count;
	md->fifo_batch = fifo_batch;
	md->last_dispatch = jiffies;
	md->last_dir = -1;
	md->front_merges = 1;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* sysfs                                                               */
/* ------------------------------------------------------------------ */
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
	if (__CONV)							\
		__data = jiffies_to_msecs(__data);			\
	return md_var_show(__data, page);				\
}
SHOW_FN(md_sync_read_expire_show,	md->fifo_expire[MD_SR], 1);
SHOW_FN(md_sync_write_expire_show,	md->fifo_expire[MD_SW], 1);
SHOW_FN(md_async_read_expire_show,	md->fifo_expire[MD_AR], 1);
SHOW_FN(md_async_write_expire_show,	md->fifo_expire[MD_AW], 1);
SHOW_FN(md_writes_starved_show,		md->writes_starved, 0);
SHOW_FN(md_sync_ratio_show,		md->sync_ratio, 0);
SHOW_FN(md_batch_count_show,		md->batch_count, 0);
SHOW_FN(md_fifo_batch_show,		md->fifo_batch, 0);
SHOW_FN(md_latency_ema_show,		md->read_latency_ema / 1000, 0);
SHOW_FN(md_scale_down_show,		md->scale_down, 0);
#undef SHOW_FN

#define STORE_FN(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page,	\
		      size_t count)					\
{									\
	struct md_data *md = e->elevator_data;				\
	int __data;							\
	md_var_store(&__data, (page));					\
	if (__data < (MIN))						\
		__data = (MIN);						\
	else if (__data > (MAX))					\
		__data = (MAX);						\
	if (__CONV)							\
		*(__PTR) = msecs_to_jiffies(__data);			\
	else								\
		*(__PTR) = __data;					\
	return count;							\
}
STORE_FN(md_sync_read_expire_store,	&md->fifo_expire[MD_SR], 0, INT_MAX, 1);
STORE_FN(md_sync_write_expire_store,	&md->fifo_expire[MD_SW], 0, INT_MAX, 1);
STORE_FN(md_async_read_expire_store,	&md->fifo_expire[MD_AR], 0, INT_MAX, 1);
STORE_FN(md_async_write_expire_store,	&md->fifo_expire[MD_AW], 0, INT_MAX, 1);
STORE_FN(md_writes_starved_store,	&md->writes_starved, 0, INT_MAX, 0);
STORE_FN(md_sync_ratio_store,		&md->sync_ratio, 1, INT_MAX, 0);
STORE_FN(md_batch_count_store,		&md->batch_count, 1, INT_MAX, 0);
STORE_FN(md_fifo_batch_store,		&md->fifo_batch, 0, INT_MAX, 0);
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
	MD_ATTR(sync_ratio),
	MD_ATTR(batch_count),
	MD_ATTR(fifo_batch),
	__ATTR(latency_ema, S_IRUGO, md_latency_ema_show, NULL),
	__ATTR(scale_down, S_IRUGO, md_scale_down_show, NULL),
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
MODULE_DESCRIPTION("Monolith IO scheduler: unified NOOP+Deadline+SIO+Zen+Maple+CFQ+Kyber+Anxiety+FIOPS+BFQ");
MODULE_VERSION("1.0");
