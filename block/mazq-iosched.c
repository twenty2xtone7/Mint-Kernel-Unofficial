// SPDX-License-Identifier: GPL-2.0
/*
 * MazQ I/O Scheduler v5.0
 *
 * A lightweight hybrid scheduler combining the strengths of three proven
 * designs for optimal flash storage (eMMC/UFS) performance:
 *
 *  - Maple's 4-way FIFO (sync/async x read/write) with display-aware
 *    deadline scaling and write starvation prevention
 *  - Zen's FCFS simplicity and one-at-a-time dispatch for low latency
 *  - Deadline/CFQ's rbtree front-merges and sequential (next_rq) dispatch
 *    for maximum throughput on sequential workloads
 *
 * Dispatch priority: expired deadlines > sequential continuation > FIFO
 * Direction priority: reads over writes (with starvation prevention)
 * Sync priority: sync over async within each direction
 *
 * Copyright (C) 2026
 */
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/fb.h>

enum { ASYNC, SYNC };

/* ── Tunable defaults (milliseconds, converted to jiffies at init) ── */
static const int sync_read_expire_ms    = 100;   /* 100ms - aggressive for UI */
static const int sync_write_expire_ms   = 200;   /* 200ms */
static const int async_read_expire_ms   = 250;   /* 250ms */
static const int async_write_expire_ms  = 500;   /* 500ms - relaxed for background */
static const int def_fifo_batch         = 8;      /* check expiry every 8 dispatches */
static const int def_writes_starved     = 4;      /* reads before forcing a write */
static const int def_sleep_latency_mul  = 3;      /* relax deadlines 3x when screen off */

struct mazq_data {
	/* 4-way FIFO: [sync/async][read/write] */
	struct list_head fifo_list[2][2];

	/* Per-direction rbtrees for front-merge and sequential dispatch */
	struct rb_root sort_list[2];       /* [READ/WRITE] */
	struct request *next_rq[2];        /* [READ/WRITE] sequential cursor */

	/* Runtime state */
	unsigned int batched;              /* dispatches since last expiry check */
	unsigned int starved;              /* consecutive reads without a write */

	/* Tunables (jiffies for expiry, counts for others) */
	int fifo_expire[2][2];            /* [sync/async][read/write] in jiffies */
	int fifo_batch;
	int writes_starved;
	int sleep_latency_mul;

	/* Display state for adaptive expiry scaling */
	struct notifier_block fb_notifier;
	bool display_on;
};

static inline struct mazq_data *mazq_get_data(struct request_queue *q)
{
	return q->elevator->elevator_data;
}

/* ── Rbtree helpers ── */

static inline struct request *mazq_rb_latter(struct request *rq)
{
	struct rb_node *node = rb_next(&rq->rb_node);

	return node ? rb_entry_rq(node) : NULL;
}

static void mazq_add_rq_rb(struct mazq_data *md, struct request *rq)
{
	elv_rb_add(&md->sort_list[rq_data_dir(rq)], rq);
}

static void mazq_del_rq_rb(struct mazq_data *md, struct request *rq)
{
	int d = rq_data_dir(rq);

	if (md->next_rq[d] == rq)
		md->next_rq[d] = mazq_rb_latter(rq);
	elv_rb_del(&md->sort_list[d], rq);
}

/* ── Merge support (front-merge via rbtree) ── */

static enum elv_merge mazq_merge(struct request_queue *q,
				 struct request **req, struct bio *bio)
{
	struct mazq_data *md = mazq_get_data(q);
	struct request *__rq;

	__rq = elv_rb_find(&md->sort_list[bio_data_dir(bio)],
			   bio_end_sector(bio));
	if (__rq && elv_bio_merge_ok(__rq, bio)) {
		*req = __rq;
		return ELEVATOR_FRONT_MERGE;
	}
	return ELEVATOR_NO_MERGE;
}

static void mazq_merged_request(struct request_queue *q,
				struct request *req, enum elv_merge type)
{
	struct mazq_data *md = mazq_get_data(q);

	/* Front-merge changed the start sector; reposition in rbtree */
	if (type == ELEVATOR_FRONT_MERGE) {
		elv_rb_del(&md->sort_list[rq_data_dir(req)], req);
		mazq_add_rq_rb(md, req);
	}
}

static void mazq_merged_requests(struct request_queue *q,
				 struct request *rq, struct request *next)
{
	struct mazq_data *md = mazq_get_data(q);

	/* Inherit the earlier deadline */
	if (!list_empty(&rq->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before((unsigned long)next->fifo_time,
				(unsigned long)rq->fifo_time)) {
			list_move(&rq->queuelist, &next->queuelist);
			rq->fifo_time = next->fifo_time;
		}
	}

	/* Remove 'next' from ALL internal structures before block layer frees it */
	rq_fifo_clear(next);
	mazq_del_rq_rb(md, next);
}

/* ── Request addition ── */

static void mazq_add_request(struct request_queue *q, struct request *rq)
{
	struct mazq_data *md = mazq_get_data(q);
	int s = rq_is_sync(rq);
	int d = rq_data_dir(rq);
	int expire;

	/* Add to rbtree for merge and sequential support */
	mazq_add_rq_rb(md, rq);

	/* Add to FIFO with computed deadline */
	expire = md->fifo_expire[s][d];
	if (expire) {
		/* Relax deadlines when display is off to save power */
		if (!md->display_on)
			expire *= md->sleep_latency_mul;
		rq->fifo_time = jiffies + expire;
		list_add_tail(&rq->queuelist, &md->fifo_list[s][d]);
	}
}

/* ── Expired request selection ── */

static struct request *mazq_expired_request(struct mazq_data *md, int s, int d)
{
	struct list_head *list = &md->fifo_list[s][d];
	struct request *rq;

	if (list_empty(list))
		return NULL;

	rq = rq_entry_fifo(list->next);
	if (time_after_eq(jiffies, (unsigned long)rq->fifo_time))
		return rq;

	return NULL;
}

/*
 * Find the most urgent expired request across all 4 queues.
 * Priority: sync read > async read > sync write > async write
 */
static struct request *mazq_choose_expired(struct mazq_data *md)
{
	struct request *rq;

	rq = mazq_expired_request(md, SYNC, READ);
	if (rq) return rq;
	rq = mazq_expired_request(md, ASYNC, READ);
	if (rq) return rq;
	rq = mazq_expired_request(md, SYNC, WRITE);
	if (rq) return rq;
	return mazq_expired_request(md, ASYNC, WRITE);
}

/* ── FIFO selection (oldest request in preferred direction) ── */

static struct request *mazq_choose_fifo(struct mazq_data *md, int d)
{
	/* Within a direction, prefer sync (foreground) over async (background) */
	if (!list_empty(&md->fifo_list[SYNC][d]))
		return rq_entry_fifo(md->fifo_list[SYNC][d].next);
	if (!list_empty(&md->fifo_list[ASYNC][d]))
		return rq_entry_fifo(md->fifo_list[ASYNC][d].next);
	return NULL;
}

/* ── Core dispatch ── */

static void mazq_dispatch(struct mazq_data *md, struct request *rq)
{
	int d = rq_data_dir(rq);

	/* Advance sequential cursor BEFORE removing from tree */
	md->next_rq[d] = mazq_rb_latter(rq);

	rq_fifo_clear(rq);
	elv_rb_del(&md->sort_list[d], rq);
	elv_dispatch_add_tail(rq->q, rq);

	md->batched++;

	/* Track write starvation: increment when reading with writes pending */
	if (d == READ) {
		if (!list_empty(&md->fifo_list[SYNC][WRITE]) ||
		    !list_empty(&md->fifo_list[ASYNC][WRITE]))
			md->starved++;
	} else {
		md->starved = 0;
	}
}

static int mazq_dispatch_requests(struct request_queue *q, int force)
{
	struct mazq_data *md = mazq_get_data(q);
	struct request *rq;
	int dir;

	/* Preferred direction: READ unless writes are starving */
	dir = (md->starved >= md->writes_starved) ? WRITE : READ;

	/*
	 * When forced (queue drain), skip scheduling logic and
	 * just dispatch anything available via FIFO.
	 */
	if (unlikely(force))
		goto fifo;

	/*
	 * Step 1: After fifo_batch dispatches, enforce deadlines.
	 * This prevents any request from starving beyond its expiry.
	 */
	if (md->batched >= md->fifo_batch) {
		md->batched = 0;
		rq = mazq_choose_expired(md);
		if (rq)
			goto dispatch;
	}

	/*
	 * Step 2: Sequential dispatch via rbtree cursor.
	 * Dispatching the next request in sector order maximizes
	 * throughput for sequential workloads on flash storage.
	 */
	rq = md->next_rq[dir];
	if (!rq)
		rq = md->next_rq[!dir];
	if (rq)
		goto dispatch;

fifo:
	/*
	 * Step 3: FIFO dispatch. Pick the oldest request in the
	 * preferred direction, with sync priority over async.
	 */
	rq = mazq_choose_fifo(md, dir);
	if (!rq)
		rq = mazq_choose_fifo(md, !dir);
	if (!rq)
		return 0;

dispatch:
	mazq_dispatch(md, rq);
	return 1;
}

/* ── Display notifier for adaptive deadlines ── */

static int mazq_fb_notifier(struct notifier_block *self,
			    unsigned long event, void *data)
{
	struct mazq_data *md = container_of(self, struct mazq_data,
					    fb_notifier);
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			md->display_on = true;
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			md->display_on = false;
			break;
		}
	}
	return 0;
}

/* ── Init / exit ── */

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

	/* Initialize 4-way FIFO lists */
	INIT_LIST_HEAD(&md->fifo_list[SYNC][READ]);
	INIT_LIST_HEAD(&md->fifo_list[SYNC][WRITE]);
	INIT_LIST_HEAD(&md->fifo_list[ASYNC][READ]);
	INIT_LIST_HEAD(&md->fifo_list[ASYNC][WRITE]);

	/* Initialize per-direction rbtrees */
	md->sort_list[READ]  = RB_ROOT;
	md->sort_list[WRITE] = RB_ROOT;

	/* Deadlines (converted from ms to jiffies for HZ-independence) */
	md->fifo_expire[SYNC][READ]   = msecs_to_jiffies(sync_read_expire_ms);
	md->fifo_expire[SYNC][WRITE]  = msecs_to_jiffies(sync_write_expire_ms);
	md->fifo_expire[ASYNC][READ]  = msecs_to_jiffies(async_read_expire_ms);
	md->fifo_expire[ASYNC][WRITE] = msecs_to_jiffies(async_write_expire_ms);
	md->fifo_batch       = def_fifo_batch;
	md->writes_starved   = def_writes_starved;
	md->sleep_latency_mul = def_sleep_latency_mul;

	/* Display-aware scheduling (from Maple) */
	md->display_on = true;
	md->fb_notifier.notifier_call = mazq_fb_notifier;
	fb_register_client(&md->fb_notifier);

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static void mazq_exit_queue(struct elevator_queue *e)
{
	struct mazq_data *md = e->elevator_data;

	fb_unregister_client(&md->fb_notifier);
	kfree(md);
}

/* ── Sysfs (values shown/stored in milliseconds for user convenience) ── */

static ssize_t mazq_var_show(int var, char *page)
{
	return snprintf(page, PAGE_SIZE, "%d\n", var);
}

#define SHOW_FUNC(__FUNC, __VAR, __CONV)				\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct mazq_data *md = e->elevator_data;				\
	int __data = __VAR;						\
	if (__CONV)							\
		__data = jiffies_to_msecs(__data);			\
	return mazq_var_show(__data, page);				\
}
SHOW_FUNC(mazq_sync_read_expire_show, md->fifo_expire[SYNC][READ], 1);
SHOW_FUNC(mazq_sync_write_expire_show, md->fifo_expire[SYNC][WRITE], 1);
SHOW_FUNC(mazq_async_read_expire_show, md->fifo_expire[ASYNC][READ], 1);
SHOW_FUNC(mazq_async_write_expire_show, md->fifo_expire[ASYNC][WRITE], 1);
SHOW_FUNC(mazq_fifo_batch_show, md->fifo_batch, 0);
SHOW_FUNC(mazq_writes_starved_show, md->writes_starved, 0);
SHOW_FUNC(mazq_sleep_latency_mul_show, md->sleep_latency_mul, 0);
#undef SHOW_FUNC

#define STORE_FUNC(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page,	\
		      size_t count)					\
{									\
	struct mazq_data *md = e->elevator_data;				\
	int val;							\
	int ret = kstrtoint(page, 0, &val);				\
	if (ret)							\
		return ret;						\
	if (val < (MIN)) val = (MIN);					\
	if (val > (MAX)) val = (MAX);					\
	if (__CONV)							\
		*(__PTR) = msecs_to_jiffies(val);			\
	else								\
		*(__PTR) = val;						\
	return count;							\
}
STORE_FUNC(mazq_sync_read_expire_store, &md->fifo_expire[SYNC][READ], 0, 10000, 1);
STORE_FUNC(mazq_sync_write_expire_store, &md->fifo_expire[SYNC][WRITE], 0, 10000, 1);
STORE_FUNC(mazq_async_read_expire_store, &md->fifo_expire[ASYNC][READ], 0, 10000, 1);
STORE_FUNC(mazq_async_write_expire_store, &md->fifo_expire[ASYNC][WRITE], 0, 10000, 1);
STORE_FUNC(mazq_fifo_batch_store, &md->fifo_batch, 1, 256, 0);
STORE_FUNC(mazq_writes_starved_store, &md->writes_starved, 1, 100, 0);
STORE_FUNC(mazq_sleep_latency_mul_store, &md->sleep_latency_mul, 1, 10, 0);
#undef STORE_FUNC

#define MAZQ_ATTR(name)							\
	__ATTR(name, S_IRUGO | S_IWUSR, mazq_##name##_show,		\
	       mazq_##name##_store)

static struct elv_fs_entry mazq_attrs[] = {
	MAZQ_ATTR(sync_read_expire),
	MAZQ_ATTR(sync_write_expire),
	MAZQ_ATTR(async_read_expire),
	MAZQ_ATTR(async_write_expire),
	MAZQ_ATTR(fifo_batch),
	MAZQ_ATTR(writes_starved),
	MAZQ_ATTR(sleep_latency_mul),
	__ATTR_NULL
};

static struct elevator_type iosched_mazq = {
	.ops.sq = {
		.elevator_merge_fn		= mazq_merge,
		.elevator_merged_fn		= mazq_merged_request,
		.elevator_merge_req_fn		= mazq_merged_requests,
		.elevator_dispatch_fn		= mazq_dispatch_requests,
		.elevator_add_req_fn		= mazq_add_request,
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
MODULE_DESCRIPTION("MazQ I/O scheduler - Maple + Zen + deadline hybrid");
MODULE_VERSION("5.0");
