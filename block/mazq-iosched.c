// SPDX-License-Identifier: GPL-2.0
/*
 * MazQ I/O Scheduler
 *
 * Lightweight hybrid: Maple 4-way FIFO + Anxiety batch dispatch + Zen FCFS,
 * with Aether-style EMA latency tracking, CFQ thinktime detection,
 * per-queue rbtrees for front-merges, and adaptive deadline tightening.
 *
 * Copyright (C) 2026
 */
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/ktime.h>

enum { ASYNC, SYNC };

/* ── Tunable defaults ── */
static const int sync_read_expire    = 10;
static const int sync_write_expire   = 25;
static const int async_read_expire   = 25;
static const int async_write_expire  = 50;
static const int fifo_batch          = 6;
static const int writes_starved      = 4;
static const int sync_ratio          = 6;
static const int batch_count         = 3;
static const int thinktime_jiffs     = 2;
static const int latency_target_ns   = 5000000;
static const int latency_samples_max = 4;
static const int async_cost_limit    = 4;

struct mazq_data {
	struct list_head fifo_list[2][2];
	struct rb_root sort_list[2][2];
	struct request *next_rq[2][2];

	int fifo_expire[2][2];
	int fifo_batch;
	int writes_starved;
	int sync_ratio;
	int batch_count;
	int batched;
	int starved;
	int async_cost;

	unsigned long last_dispatch;
	int last_dispatch_dir;
	int think_jiffs;
	int think_seen;

	u64 read_latency_ema;
	u64 read_latency_max;
	int latency_samples;
	unsigned int tight_mode;
	int latency_target;
	int latency_window;
};

static inline struct mazq_data *mazq_get_data(struct request_queue *q)
{
	return q->elevator->elevator_data;
}

static struct rb_root *mazq_rb_root(struct mazq_data *md,
				    struct request *rq)
{
	int s = rq_is_sync(rq);
	int d = rq_data_dir(rq);
	return &md->sort_list[s][d];
}

static inline struct request *mazq_latter(struct request *rq)
{
	struct rb_node *node = rb_next(&rq->rb_node);
	return node ? rb_entry_rq(node) : NULL;
}

static void mazq_add_rq_rb(struct mazq_data *md, struct request *rq)
{
	elv_rb_add(mazq_rb_root(md, rq), rq);
}

static inline void mazq_del_rq_rb(struct mazq_data *md, struct request *rq)
{
	int s = rq_is_sync(rq);
	int d = rq_data_dir(rq);
	if (md->next_rq[s][d] == rq)
		md->next_rq[s][d] = mazq_latter(rq);
	elv_rb_del(mazq_rb_root(md, rq), rq);
}

static void mazq_check_thinktime(struct mazq_data *md)
{
	if (md->last_dispatch &&
	    time_after(jiffies, md->last_dispatch + md->think_jiffs)) {
		md->think_seen = 1;
		md->starved = 0;
		md->batched = md->fifo_batch;
	}
}

static void mazq_check_ema(struct mazq_data *md, u64 lat_ns)
{
	md->read_latency_max = max(md->read_latency_max, lat_ns);

	if (md->read_latency_ema)
		md->read_latency_ema += (lat_ns - md->read_latency_ema) >> 3;
	else
		md->read_latency_ema = lat_ns;

	md->latency_samples++;
	if (md->latency_samples >= md->latency_window) {
		md->latency_samples = 0;
		if (md->read_latency_max > md->latency_target)
			md->tight_mode = 1;
		else if (md->read_latency_ema < (md->latency_target >> 1))
			md->tight_mode = 0;
		md->read_latency_max = 0;
	}
}

static enum elv_merge mazq_merge(struct request_queue *q,
				 struct request **req, struct bio *bio)
{
	struct mazq_data *md = q->elevator->elevator_data;
	int d = bio_data_dir(bio), s;
	struct request *__rq;

	for (s = 0; s < 2; s++) {
		__rq = elv_rb_find(&md->sort_list[s][d], bio_end_sector(bio));
		if (__rq) {
			if (bio_end_sector(bio) != blk_rq_pos(__rq))
				return ELEVATOR_NO_MERGE;
			if (elv_bio_merge_ok(__rq, bio)) {
				*req = __rq;
				return ELEVATOR_FRONT_MERGE;
			}
		}
	}
	return ELEVATOR_NO_MERGE;
}

static void mazq_merged_request(struct request_queue *q,
				struct request *req, enum elv_merge type)
{
	struct mazq_data *md = q->elevator->elevator_data;
	if (type == ELEVATOR_FRONT_MERGE) {
		elv_rb_del(mazq_rb_root(md, req), req);
		mazq_add_rq_rb(md, req);
	}
}

static void mazq_merged_requests(struct request_queue *q,
				 struct request *rq, struct request *next)
{
	if (!list_empty(&rq->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before((unsigned long)next->fifo_time,
				(unsigned long)rq->fifo_time)) {
			list_move(&rq->queuelist, &next->queuelist);
			rq->fifo_time = next->fifo_time;
		}
	}
	rq_fifo_clear(next);
}

static void mazq_add_request(struct request_queue *q, struct request *rq)
{
	struct mazq_data *md = mazq_get_data(q);
	int s = rq_is_sync(rq);
	int d = rq_data_dir(rq);

	mazq_add_rq_rb(md, rq);

	if (md->fifo_expire[s][d]) {
		rq->fifo_time = jiffies +
			msecs_to_jiffies(md->fifo_expire[s][d]);
		list_add_tail(&rq->queuelist, &md->fifo_list[s][d]);
	}

	if (!md->next_rq[s][d])
		md->next_rq[s][d] = mazq_latter(rq);
}

static void __maybe_unused mazq_remove_request(struct request_queue *q, struct request *rq)
{
	struct mazq_data *md = mazq_get_data(q);
	rq_fifo_clear(rq);
	mazq_del_rq_rb(md, rq);
}

static struct request *mazq_choose_expired(struct mazq_data *md)
{
	struct request *rq, *best = NULL;
	unsigned long best_time = ~0UL;
	int s, d;

	for (s = 0; s < 2; s++) {
		for (d = 0; d < 2; d++) {
			struct list_head *list = &md->fifo_list[s][d];
			if (list_empty(list))
				continue;
			rq = rq_entry_fifo(list->next);
			if (time_after_eq(jiffies,
				    (unsigned long)rq->fifo_time) &&
			    time_before((unsigned long)rq->fifo_time,
					best_time)) {
				best = rq;
				best_time = (unsigned long)rq->fifo_time;
			}
		}
	}
	return best;
}

static int mazq_flush_expired(struct mazq_data *md, struct request_queue *q)
{
	int d = 0;
	int s, dir;

	for (s = 0; s < 2; s++)
		for (dir = 0; dir < 2; dir++) {
			struct list_head *list = &md->fifo_list[s][dir];
			struct request *rq;

			while (!list_empty(list)) {
				rq = rq_entry_fifo(list->next);
				if (!time_after_eq(jiffies,
				    (unsigned long)rq->fifo_time))
					break;
				rq_fifo_clear(rq);
				mazq_del_rq_rb(md, rq);
				elv_dispatch_add_tail(q, rq);
				d++;
			}
		}
	return d;
}

static struct request *mazq_choose_request(struct mazq_data *md, int dir)
{
	/* Prefer the last dispatch direction after thinktime */
	if (md->think_seen) {
		int s, d;
		for (s = 0; s < 2; s++)
			for (d = 0; d < 2; d++)
				if (!list_empty(&md->fifo_list[s][d]) &&
				    d == md->last_dispatch_dir)
					return rq_entry_fifo(
						md->fifo_list[s][d].next);
		md->think_seen = 0;
	}

	if (!list_empty(&md->fifo_list[SYNC][dir]))
		return rq_entry_fifo(md->fifo_list[SYNC][dir].next);
	if (!list_empty(&md->fifo_list[ASYNC][dir]))
		return rq_entry_fifo(md->fifo_list[ASYNC][dir].next);
	if (!list_empty(&md->fifo_list[ASYNC][!dir]))
		return rq_entry_fifo(md->fifo_list[ASYNC][!dir].next);
	if (!list_empty(&md->fifo_list[SYNC][!dir]))
		return rq_entry_fifo(md->fifo_list[SYNC][!dir].next);

	return NULL;
}

static struct request *mazq_choose_sequential(struct mazq_data *md, int dir)
{
	struct request *rq = NULL;

	if (md->next_rq[SYNC][dir])
		rq = md->next_rq[SYNC][dir];
	else if (md->next_rq[ASYNC][dir])
		rq = md->next_rq[ASYNC][dir];

	if (rq)
		return rq;

	return NULL;
}

static void mazq_dispatch(struct mazq_data *md, struct request *rq)
{
	md->batched++;
	md->last_dispatch = jiffies;
	md->last_dispatch_dir = rq_data_dir(rq);
	rq_fifo_clear(rq);
	mazq_del_rq_rb(md, rq);
	elv_dispatch_add_tail(rq->q, rq);

	if (rq_data_dir(rq) == WRITE) {
		if (!list_empty(&md->fifo_list[SYNC][READ]) ||
		    !list_empty(&md->fifo_list[ASYNC][READ]))
			md->starved++;
		md->async_cost++;
	} else {
		md->starved = 0;
		md->async_cost -= min(md->async_cost, 1);
	}
}

static int mazq_dispatch_batch(struct request_queue *q)
{
	struct mazq_data *md = mazq_get_data(q);
	struct request *rq;
	int dispatched = 0;
	int dir;
	u8 i, j;

	/* If async cost limit hit, force reads */
	if (md->async_cost >= async_cost_limit)
		dir = READ;
	else
		dir = (md->starved >= md->writes_starved) ? WRITE : READ;

	for (i = 0; i < md->batch_count; i++) {
		for (j = 0; j < md->sync_ratio; j++) {
			rq = mazq_choose_request(md, dir);
			if (!rq) return dispatched;
			mazq_dispatch(md, rq);
			dispatched++;
		}
		if (!list_empty(&md->fifo_list[SYNC][!dir]) ||
		    !list_empty(&md->fifo_list[ASYNC][!dir])) {
			rq = mazq_choose_request(md, !dir);
			if (rq) {
				mazq_dispatch(md, rq);
				dispatched++;
			}
		}
	}
	return dispatched;
}

static int mazq_dispatch_drain(struct request_queue *q)
{
	struct mazq_data *md = mazq_get_data(q);
	struct request *rq;
	int d = 0;
	int s, dir;

	for (s = 0; s < 2; s++)
		for (dir = 0; dir < 2; dir++)
			while (!list_empty(&md->fifo_list[s][dir])) {
				rq = rq_entry_fifo(md->fifo_list[s][dir].next);
				rq_fifo_clear(rq);
				mazq_del_rq_rb(md, rq);
				elv_dispatch_add_tail(q, rq);
				d++;
			}
	return d;
}

static int mazq_dispatch_requests(struct request_queue *q, int force)
{
	struct mazq_data *md = mazq_get_data(q);
	struct request *rq;
	int ret;

	if (unlikely(force))
		return mazq_dispatch_drain(q);

	mazq_check_thinktime(md);

	/* When tight_mode is active, check expiry every batch */
	if (md->tight_mode) {
		int flushed = mazq_flush_expired(md, q);
		if (flushed) return flushed;
	}

	if (md->batched >= md->fifo_batch) {
		md->batched = 0;
		rq = mazq_choose_expired(md);
		if (rq) {
			mazq_dispatch(md, rq);
			return 1;
		}
	}

	/* Sequential hint: dispatch next_rq if within fifo_batch */
	if (!md->tight_mode && md->batched < md->fifo_batch / 2) {
		int d = (md->starved >= md->writes_starved) ? WRITE : READ;
		rq = mazq_choose_sequential(md, d);
		if (rq) {
			mazq_dispatch(md, rq);
			return 1;
		}
		rq = mazq_choose_sequential(md, !d);
		if (rq) {
			mazq_dispatch(md, rq);
			return 1;
		}
	}

	ret = mazq_dispatch_batch(q);
	if (ret == 0) {
		md->batched = md->fifo_batch;
		rq = mazq_choose_expired(md);
		if (rq) {
			mazq_dispatch(md, rq);
			return 1;
		}
	}
	return ret;
}

static void mazq_completed_req(struct request_queue *q, struct request *rq)
{
	struct mazq_data *md = mazq_get_data(q);
	u64 start_ns;

	if (!rq_is_sync(rq) || op_is_flush(rq->cmd_flags) ||
	    rq_data_dir(rq) == WRITE)
		return;
	start_ns = rq_start_time_ns(rq);
	if (!start_ns)
		return;
	{
		u64 lat = ktime_get_ns() - start_ns;
		mazq_check_ema(md, lat);
	}
}

static void __maybe_unused mazq_put_request(struct request *rq)
{
}

static int mazq_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct mazq_data *md;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	md = kzalloc_node(sizeof(*md), GFP_KERNEL, q->node);
	if (!md) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = md;

	INIT_LIST_HEAD(&md->fifo_list[SYNC][READ]);
	INIT_LIST_HEAD(&md->fifo_list[SYNC][WRITE]);
	INIT_LIST_HEAD(&md->fifo_list[ASYNC][READ]);
	INIT_LIST_HEAD(&md->fifo_list[ASYNC][WRITE]);

	md->sort_list[SYNC][READ]   = RB_ROOT;
	md->sort_list[SYNC][WRITE]  = RB_ROOT;
	md->sort_list[ASYNC][READ]  = RB_ROOT;
	md->sort_list[ASYNC][WRITE] = RB_ROOT;

	md->fifo_expire[SYNC][READ]   = sync_read_expire;
	md->fifo_expire[SYNC][WRITE]  = sync_write_expire;
	md->fifo_expire[ASYNC][READ]  = async_read_expire;
	md->fifo_expire[ASYNC][WRITE] = async_write_expire;
	md->fifo_batch       = fifo_batch;
	md->writes_starved   = writes_starved;
	md->sync_ratio       = sync_ratio;
	md->batch_count      = batch_count;
	md->think_jiffs      = thinktime_jiffs;
	md->latency_target   = latency_target_ns;
	md->latency_window   = latency_samples_max;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static void mazq_exit_queue(struct elevator_queue *e)
{
	kfree(e->elevator_data);
}

/* ── Sysfs ── */

static ssize_t mazq_var_show(int var, char *page)
{
	return snprintf(page, PAGE_SIZE, "%d\n", var);
}

static ssize_t __maybe_unused mazq_var_store(int *var, const char *page, size_t count)
{
	int ret = kstrtoint(page, 0, var);
	return ret ? ret : count;
}

static ssize_t mazq_ema_show(struct elevator_queue *e, char *page)
{
	struct mazq_data *md = e->elevator_data;
	return snprintf(page, PAGE_SIZE, "%llu\n", md->read_latency_ema);
}

static ssize_t mazq_tight_show(struct elevator_queue *e, char *page)
{
	struct mazq_data *md = e->elevator_data;
	return snprintf(page, PAGE_SIZE, "%u\n", md->tight_mode);
}

static ssize_t mazq_peak_show(struct elevator_queue *e, char *page)
{
	struct mazq_data *md = e->elevator_data;
	return snprintf(page, PAGE_SIZE, "%llu\n", md->read_latency_max);
}

#define SHOW_FUNC(__FUNC, __VAR) \
static ssize_t __FUNC(struct elevator_queue *e, char *page) \
{ \
	struct mazq_data *md = e->elevator_data; \
	return mazq_var_show(__VAR, page); \
}
SHOW_FUNC(mazq_sync_read_expire_show, md->fifo_expire[SYNC][READ]);
SHOW_FUNC(mazq_sync_write_expire_show, md->fifo_expire[SYNC][WRITE]);
SHOW_FUNC(mazq_async_read_expire_show, md->fifo_expire[ASYNC][READ]);
SHOW_FUNC(mazq_async_write_expire_show, md->fifo_expire[ASYNC][WRITE]);
SHOW_FUNC(mazq_fifo_batch_show, md->fifo_batch);
SHOW_FUNC(mazq_writes_starved_show, md->writes_starved);
SHOW_FUNC(mazq_sync_ratio_show, md->sync_ratio);
SHOW_FUNC(mazq_batch_count_show, md->batch_count);
SHOW_FUNC(mazq_latency_target_show, md->latency_target);
SHOW_FUNC(mazq_thinktime_show, md->think_jiffs);
SHOW_FUNC(mazq_latency_window_show, md->latency_window);
#undef SHOW_FUNC

#define STORE_FUNC(__FUNC, __PTR, MIN, MAX) \
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count) \
{ \
	struct mazq_data *md = e->elevator_data; \
	int val; \
	int ret = kstrtoint(page, 0, &val); \
	if (ret) return ret; \
	if (val < (MIN)) val = (MIN); \
	if (val > (MAX)) val = (MAX); \
	*(__PTR) = val; \
	return count; \
}
STORE_FUNC(mazq_sync_read_expire_store, &md->fifo_expire[SYNC][READ], 0, 10000);
STORE_FUNC(mazq_sync_write_expire_store, &md->fifo_expire[SYNC][WRITE], 0, 10000);
STORE_FUNC(mazq_async_read_expire_store, &md->fifo_expire[ASYNC][READ], 0, 10000);
STORE_FUNC(mazq_async_write_expire_store, &md->fifo_expire[ASYNC][WRITE], 0, 10000);
STORE_FUNC(mazq_fifo_batch_store, &md->fifo_batch, 1, 256);
STORE_FUNC(mazq_writes_starved_store, &md->writes_starved, 1, 100);
STORE_FUNC(mazq_sync_ratio_store, &md->sync_ratio, 1, 128);
STORE_FUNC(mazq_batch_count_store, &md->batch_count, 1, 32);
STORE_FUNC(mazq_latency_target_store, &md->latency_target, 100000, 100000000);
STORE_FUNC(mazq_thinktime_store, &md->think_jiffs, 1, 100);
STORE_FUNC(mazq_latency_window_store, &md->latency_window, 1, 64);
#undef STORE_FUNC

#define MAZQ_ATTR(name) \
	__ATTR(name, S_IRUGO | S_IWUSR, mazq_##name##_show, \
	       mazq_##name##_store)

static struct elv_fs_entry mazq_attrs[] = {
	MAZQ_ATTR(sync_read_expire),
	MAZQ_ATTR(sync_write_expire),
	MAZQ_ATTR(async_read_expire),
	MAZQ_ATTR(async_write_expire),
	MAZQ_ATTR(fifo_batch),
	MAZQ_ATTR(writes_starved),
	MAZQ_ATTR(sync_ratio),
	MAZQ_ATTR(batch_count),
	MAZQ_ATTR(latency_target),
	MAZQ_ATTR(thinktime),
	MAZQ_ATTR(latency_window),
	__ATTR(latency_ema, S_IRUGO, mazq_ema_show, NULL),
	__ATTR(tight_mode, S_IRUGO, mazq_tight_show, NULL),
	__ATTR(peak_latency, S_IRUGO, mazq_peak_show, NULL),
	__ATTR_NULL
};

static struct elevator_type iosched_mazq = {
	.ops.sq = {
		.elevator_merge_fn		= mazq_merge,
		.elevator_merged_fn		= mazq_merged_request,
		.elevator_merge_req_fn		= mazq_merged_requests,
		.elevator_dispatch_fn		= mazq_dispatch_requests,
		.elevator_add_req_fn		= mazq_add_request,
		.elevator_completed_req_fn	= mazq_completed_req,
		.elevator_former_req_fn		= elv_rb_former_request,
		.elevator_latter_req_fn		= elv_rb_latter_request,
		.elevator_init_fn		= mazq_init_queue,
		.elevator_exit_fn		= mazq_exit_queue,
	},
	.elevator_attrs = mazq_attrs,
	.elevator_name = "mazq",
	.elevator_owner = THIS_MODULE,
};

static int __init mazq_init(void)
{
	return elv_register(&iosched_mazq);
}

static void __exit mazq_exit(void)
{
	elv_unregister(&iosched_mazq);
}

module_init(mazq_init);
module_exit(mazq_exit);

MODULE_AUTHOR("mint-kranal");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MazQ I/O scheduler - enhanced with EMA, thinktime, rbtrees");
MODULE_VERSION("4.0");
