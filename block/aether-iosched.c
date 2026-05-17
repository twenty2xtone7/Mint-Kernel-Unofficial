/*
 * Aether I/O scheduler
 * Aggressive performance tuning inspired by Deadline, CFQ, and Kyber.
 * - Deadline: per-queue expiry with read bias
 * - CFQ: thinktime detection for foreground IO priority
 * - Kyber: adaptive latency tightening
 *
 * Copyright (C) 2002 Jens Axboe <axboe@kernel.dk>
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

/* Aggressive performance defaults for UFS/eMMC */
static const int read_expire = HZ / 50;		/* ~20ms at 300Hz */
static const int write_expire = HZ / 10;	/* ~100ms at 300Hz */
static const int writes_starved = 6;
static const int fifo_batch = 6;
static const int thinktime_threshold = 2;	/* jiffies (~7ms) */
static const u64 latency_target_ns = 5 * NSEC_PER_MSEC; /* 5ms */
static const int latency_sample_window = 4;	/* samples before re-evaluating */

struct aether_data {
	struct rb_root sort_list[2];
	struct list_head fifo_list[2];
	struct request *next_rq[2];
	unsigned int batching;
	unsigned int starved;

	int fifo_expire[2];
	int fifo_batch;
	int writes_starved;
	int front_merges;

	/* CFQ thinktime */
	unsigned long last_dispatch;	/* jiffies */
	unsigned int think_seen;	/* thinktime detected flag */

	/* Kyber latency tracking (EMA) */
	u64 read_latency_ema;		/* ns, exponential moving average */
	u64 read_latency_max;		/* ns, max in current window */
	int latency_samples;		/* count in current window */
	unsigned int tight_mode;	/* 1 = tighten deadlines */
};

static inline struct rb_root *
aether_rb_root(struct aether_data *hd, struct request *rq)
{
	return &hd->sort_list[rq_data_dir(rq)];
}

static inline struct request *
aether_latter_request(struct request *rq)
{
	struct rb_node *node = rb_next(&rq->rb_node);
	if (node)
		return rb_entry_rq(node);
	return NULL;
}

static void aether_add_rq_rb(struct aether_data *hd, struct request *rq)
{
	elv_rb_add(aether_rb_root(hd, rq), rq);
}

static inline void aether_del_rq_rb(struct aether_data *hd, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	if (hd->next_rq[data_dir] == rq)
		hd->next_rq[data_dir] = aether_latter_request(rq);
	elv_rb_del(aether_rb_root(hd, rq), rq);
}

static void aether_add_request(struct request_queue *q, struct request *rq)
{
	struct aether_data *hd = q->elevator->elevator_data;
	const int data_dir = rq_data_dir(rq);

	aether_add_rq_rb(hd, rq);

	rq->fifo_time = jiffies + hd->fifo_expire[data_dir];
	list_add_tail(&rq->queuelist, &hd->fifo_list[data_dir]);
}

static void aether_remove_request(struct request_queue *q, struct request *rq)
{
	struct aether_data *hd = q->elevator->elevator_data;
	rq_fifo_clear(rq);
	aether_del_rq_rb(hd, rq);
}

static enum elv_merge
aether_merge(struct request_queue *q, struct request **req, struct bio *bio)
{
	struct aether_data *hd = q->elevator->elevator_data;
	struct request *__rq;

	if (hd->front_merges) {
		sector_t sector = bio_end_sector(bio);
		__rq = elv_rb_find(&hd->sort_list[bio_data_dir(bio)], sector);
		if (__rq) {
			BUG_ON(sector != blk_rq_pos(__rq));
			if (elv_bio_merge_ok(__rq, bio)) {
				*req = __rq;
				return ELEVATOR_FRONT_MERGE;
			}
		}
	}
	return ELEVATOR_NO_MERGE;
}

static void aether_merged_request(struct request_queue *q,
				  struct request *req, enum elv_merge type)
{
	struct aether_data *hd = q->elevator->elevator_data;
	if (type == ELEVATOR_FRONT_MERGE) {
		elv_rb_del(aether_rb_root(hd, req), req);
		aether_add_rq_rb(hd, req);
	}
}

static void aether_merged_requests(struct request_queue *q,
				   struct request *req, struct request *next)
{
	if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before((unsigned long)next->fifo_time,
				(unsigned long)req->fifo_time)) {
			list_move(&req->queuelist, &next->queuelist);
			req->fifo_time = next->fifo_time;
		}
	}
	aether_remove_request(q, next);
}

static inline void aether_move_to_dispatch(struct aether_data *hd,
					   struct request *rq)
{
	struct request_queue *q = rq->q;
	aether_remove_request(q, rq);
	elv_dispatch_add_tail(q, rq);
}

static void aether_move_request(struct aether_data *hd, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	hd->next_rq[READ] = NULL;
	hd->next_rq[WRITE] = NULL;
	hd->next_rq[data_dir] = aether_latter_request(rq);
	hd->last_dispatch = jiffies;
	aether_move_to_dispatch(hd, rq);
}

static inline int aether_check_fifo(struct aether_data *hd, int ddir)
{
	struct request *rq = rq_entry_fifo(hd->fifo_list[ddir].next);
	if (time_after_eq(jiffies, (unsigned long)rq->fifo_time))
		return 1;
	return 0;
}

/* Kyber: track read latency with exponential moving average */
static void aether_completed_request(struct request_queue *q,
				     struct request *rq)
{
	struct aether_data *hd = q->elevator->elevator_data;
	u64 now, start, lat;

	if (rq_data_dir(rq) != READ || blk_rq_is_passthrough(rq))
		return;

	now = ktime_get_ns();
	start = rq_start_time_ns(rq);
	lat = start ? now - start : 0;

	if (lat > hd->read_latency_max)
		hd->read_latency_max = lat;

	if (hd->read_latency_ema)
		hd->read_latency_ema += (lat - hd->read_latency_ema) >> 3;
	else
		hd->read_latency_ema = lat;

	hd->latency_samples++;

	if (hd->latency_samples >= latency_sample_window) {
		hd->tight_mode = (hd->read_latency_max > latency_target_ns) ? 1 : 0;
		hd->read_latency_max = 0;
		hd->latency_samples = 0;
	}
}

static int aether_dispatch_requests(struct request_queue *q, int force)
{
	struct aether_data *hd = q->elevator->elevator_data;
	const int reads = !list_empty(&hd->fifo_list[READ]);
	const int writes = !list_empty(&hd->fifo_list[WRITE]);
	struct request *rq;
	int data_dir;

	if (!reads && !writes)
		return 0;

	/* Kyber: in tight mode, force expiry check by resetting batch */
	if (hd->tight_mode && hd->batching)
		hd->batching = hd->fifo_batch;

	/* CFQ thinktime: if queue was idle, reads jump ahead */
	if (hd->last_dispatch &&
	    time_after(jiffies, hd->last_dispatch + thinktime_threshold)) {
		hd->think_seen = 1;
		hd->starved = 0;
	}

	if (hd->next_rq[WRITE])
		rq = hd->next_rq[WRITE];
	else
		rq = hd->next_rq[READ];

	if (rq && hd->batching < hd->fifo_batch)
		goto dispatch_request;

	if (!reads) {
		if (writes)
			goto dispatch_writes;
		return 0;
	}

	BUG_ON(RB_EMPTY_ROOT(&hd->sort_list[READ]));

	/* Read bias: starve writes or thinktime seen → dispatch reads */
	if (hd->think_seen || hd->starved++ < hd->writes_starved || !writes) {
		hd->think_seen = 0;
		data_dir = READ;
		goto dispatch_find_request;
	}

	if (writes) {
dispatch_writes:
		BUG_ON(RB_EMPTY_ROOT(&hd->sort_list[WRITE]));
		hd->starved = 0;
		data_dir = WRITE;
		goto dispatch_find_request;
	}

	return 0;

dispatch_find_request:
	if (aether_check_fifo(hd, data_dir) || !hd->next_rq[data_dir])
		rq = rq_entry_fifo(hd->fifo_list[data_dir].next);
	else
		rq = hd->next_rq[data_dir];

	hd->batching = 0;

dispatch_request:
	hd->batching++;
	aether_move_request(hd, rq);
	return 1;
}

static void aether_exit_queue(struct elevator_queue *e)
{
	struct aether_data *hd = e->elevator_data;
	BUG_ON(!list_empty(&hd->fifo_list[READ]));
	BUG_ON(!list_empty(&hd->fifo_list[WRITE]));
	kfree(hd);
}

static int aether_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct aether_data *hd;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	hd = kzalloc_node(sizeof(*hd), GFP_KERNEL, q->node);
	if (!hd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = hd;

	INIT_LIST_HEAD(&hd->fifo_list[READ]);
	INIT_LIST_HEAD(&hd->fifo_list[WRITE]);
	hd->sort_list[READ] = RB_ROOT;
	hd->sort_list[WRITE] = RB_ROOT;
	hd->fifo_expire[READ] = read_expire;
	hd->fifo_expire[WRITE] = write_expire;
	hd->writes_starved = writes_starved;
	hd->front_merges = 1;
	hd->fifo_batch = fifo_batch;
	hd->last_dispatch = jiffies;
	hd->latency_samples = 0;
	hd->tight_mode = 0;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

/********** sysfs **********/

static ssize_t
aether_var_show(int var, char *page)
{
	return sprintf(page, "%d\n", var);
}

static void
aether_var_store(int *var, const char *page)
{
	char *p = (char *)page;
	*var = simple_strtol(p, &p, 10);
}

#define SHOW_FUNCTION(__FUNC, __VAR, __CONV)				\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct aether_data *hd = e->elevator_data;			\
	int __data = __VAR;						\
	if (__CONV)							\
		__data = jiffies_to_msecs(__data);			\
	return aether_var_show(__data, (page));			\
}
SHOW_FUNCTION(aether_read_expire_show, hd->fifo_expire[READ], 1);
SHOW_FUNCTION(aether_write_expire_show, hd->fifo_expire[WRITE], 1);
SHOW_FUNCTION(aether_writes_starved_show, hd->writes_starved, 0);
SHOW_FUNCTION(aether_front_merges_show, hd->front_merges, 0);
SHOW_FUNCTION(aether_fifo_batch_show, hd->fifo_batch, 0);
SHOW_FUNCTION(aether_latency_ema_show, hd->read_latency_ema / 1000, 0);
SHOW_FUNCTION(aether_tight_mode_show, hd->tight_mode, 0);
#undef SHOW_FUNCTION

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page,	\
		      size_t count)					\
{									\
	struct aether_data *hd = e->elevator_data;			\
	int __data;							\
	aether_var_store(&__data, (page));				\
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
STORE_FUNCTION(aether_read_expire_store, &hd->fifo_expire[READ], 0, INT_MAX, 1);
STORE_FUNCTION(aether_write_expire_store, &hd->fifo_expire[WRITE], 0, INT_MAX, 1);
STORE_FUNCTION(aether_writes_starved_store, &hd->writes_starved, 0, INT_MAX, 0);
STORE_FUNCTION(aether_front_merges_store, &hd->front_merges, 0, 1, 0);
STORE_FUNCTION(aether_fifo_batch_store, &hd->fifo_batch, 0, INT_MAX, 0);
#undef SHOW_FUNCTION
#undef STORE_FUNCTION

#define AETHER_ATTR(name) \
	__ATTR(name, S_IRUGO | S_IWUSR, aether_##name##_show, \
					aether_##name##_store)

static struct elv_fs_entry aether_attrs[] = {
	AETHER_ATTR(read_expire),
	AETHER_ATTR(write_expire),
	AETHER_ATTR(writes_starved),
	AETHER_ATTR(front_merges),
	AETHER_ATTR(fifo_batch),
	__ATTR(latency_ema, S_IRUGO, aether_latency_ema_show, NULL),
	__ATTR(tight_mode, S_IRUGO, aether_tight_mode_show, NULL),
	__ATTR_NULL
};

static struct elevator_type iosched_aether = {
	.ops.sq = {
		.elevator_merge_fn		= aether_merge,
		.elevator_merged_fn		= aether_merged_request,
		.elevator_merge_req_fn		= aether_merged_requests,
		.elevator_dispatch_fn		= aether_dispatch_requests,
		.elevator_add_req_fn		= aether_add_request,
		.elevator_completed_req_fn	= aether_completed_request,
		.elevator_former_req_fn		= elv_rb_former_request,
		.elevator_latter_req_fn		= elv_rb_latter_request,
		.elevator_init_fn		= aether_init_queue,
		.elevator_exit_fn		= aether_exit_queue,
	},

	.elevator_attrs = aether_attrs,
	.elevator_name = "aether",
	.elevator_owner = THIS_MODULE,
};

static int __init aether_init(void)
{
	return elv_register(&iosched_aether);
}

static void __exit aether_exit(void)
{
	elv_unregister(&iosched_aether);
}

module_init(aether_init);
module_exit(aether_exit);

MODULE_AUTHOR("aether");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Aether IO scheduler: Deadline + CFQ thinktime + Kyber latency");
MODULE_VERSION("2.0");
