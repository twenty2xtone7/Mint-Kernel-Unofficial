/*
 * Monolith I/O scheduler
 *
 * Ultimate unified scheduler: Deadine + SIO + Zen + Maple + CFQ + Anxiety.
 * - Quad FIFO queues with per-queue expiry
 * - writes_starved read bias with sync > async priority tiers
 * - Anxiety-style sync_ratio/batch_count interleaving
 * - Thinktime detection with last-direction sequential boost
 * - Expiry flushing: drain all expired on timeout
 * - Front-merge via rb-trees
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

enum {
	MD_SR = 0,	/* sync read */
	MD_SW = 1,	/* sync write */
	MD_AR = 2,	/* async read */
	MD_AW = 3,	/* async write */
	MD_NR,
};

static const int md_expire[MD_NR] = {
	HZ / 100,	/* SR: 10ms */
	HZ / 20,	/* SW: 50ms */
	HZ / 50,	/* AR: 20ms */
	HZ / 10,	/* AW: 100ms */
};
static const int md_starved		= 12;
static const int md_sync_ratio		= 6;
static const int md_batch_count		= 3;
static const int md_fifo_batch		= 4;
static const int md_thinktime_jiffies	= 2;	/* jiffies */

struct md_data {
	struct list_head fifo[MD_NR];
	int fifo_expire[MD_NR];
	int writes_starved;
	int sync_ratio;
	int batch_count;
	int fifo_batch;

	unsigned int starved;
	int ratio_cnt;
	int batch_cnt;

	unsigned long last_dispatch;
	int last_dir;
	int seen_idle;

	int front_merges;
	struct rb_root sort[MD_NR];
	struct request *next_rq[MD_NR];
};

static int md_dir(struct request *rq)
{
	int d = rq_data_dir(rq);
	return d == READ ? (rq_is_sync(rq) ? MD_SR : MD_AR)
			 : (rq_is_sync(rq) ? MD_SW : MD_AW);
}

static struct rb_root *md_root(struct md_data *md, struct request *rq)
{
	return &md->sort[md_dir(rq)];
}

static struct request *md_next_rq(struct request *rq)
{
	struct rb_node *n = rb_next(&rq->rb_node);
	return n ? rb_entry_rq(n) : NULL;
}

static void md_rb_add(struct md_data *md, struct request *rq)
{
	elv_rb_add(md_root(md, rq), rq);
}

static void md_rb_del(struct md_data *md, struct request *rq)
{
	int d = md_dir(rq);
	if (md->next_rq[d] == rq)
		md->next_rq[d] = md_next_rq(rq);
	elv_rb_del(md_root(md, rq), rq);
}

static void md_add_request(struct request_queue *q, struct request *rq)
{
	struct md_data *md = q->elevator->elevator_data;
	int d = md_dir(rq);

	md_rb_add(md, rq);
	rq->fifo_time = jiffies + md->fifo_expire[d];
	list_add_tail(&rq->queuelist, &md->fifo[d]);
}

static void md_remove_request(struct request_queue *q, struct request *rq)
{
	struct md_data *md = q->elevator->elevator_data;
	rq_fifo_clear(rq);
	md_rb_del(md, rq);
}

static enum elv_merge
md_merge(struct request_queue *q, struct request **req, struct bio *bio)
{
	struct md_data *md = q->elevator->elevator_data;
	int d = op_is_sync(bio->bi_opf)
		? (bio_data_dir(bio) == READ ? MD_SR : MD_SW)
		: (bio_data_dir(bio) == READ ? MD_AR : MD_AW);
	struct request *__rq;

	if (md->front_merges) {
		__rq = elv_rb_find(&md->sort[d], bio_end_sector(bio));
		if (__rq) {
			BUG_ON(bio_end_sector(bio) != blk_rq_pos(__rq));
			if (elv_bio_merge_ok(__rq, bio)) {
				*req = __rq;
				return ELEVATOR_FRONT_MERGE;
			}
		}
	}
	if (!list_empty(&md->fifo[d])) {
		__rq = list_entry_rq(md->fifo[d].prev);
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
		elv_rb_del(&md->sort[md_dir(req)], req);
		md_rb_add(md, req);
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

static void md_thinktime(struct md_data *md)
{
	if (md->last_dispatch &&
	    time_after(jiffies, md->last_dispatch + md_thinktime_jiffies)) {
		md->seen_idle = 1;
		md->starved = 0;
	}
}

static int md_flush(struct md_data *md, struct request_queue *q, int d)
{
	while (!list_empty(&md->fifo[d])) {
		struct request *rq = rq_entry_fifo(md->fifo[d].next);
		if (time_after_eq(jiffies, (unsigned long)rq->fifo_time)) {
			rq_fifo_clear(rq);
			md_rb_del(md, rq);
			elv_dispatch_add_tail(q, rq);
			return 1;
		}
		break;
	}
	return 0;
}

static int md_dispatch_requests(struct request_queue *q, int force)
{
	struct md_data *md = q->elevator->elevator_data;
	struct request *rq;
	int i;

	if (list_empty(&md->fifo[MD_SR]) && list_empty(&md->fifo[MD_SW]) &&
	    list_empty(&md->fifo[MD_AR]) && list_empty(&md->fifo[MD_AW]))
		return 0;

	md_thinktime(md);

	/* 1. Expiry flush: any queue, reads first */
	for (i = MD_SR; i <= MD_AW; i++)
		if (md_flush(md, q, i))
			return 1;

	/* 2. Sequential boost: after idle, resume last direction */
	if (md->seen_idle && md->last_dir >= 0 &&
	    !list_empty(&md->fifo[md->last_dir])) {
		md->seen_idle = 0;
		rq = rq_entry_fifo(md->fifo[md->last_dir].next);
		goto done;
	}
	md->seen_idle = 0;

	/* 3. Anxiety interleaving: sync_ratio syncs then 1 async */
	if (md->batch_cnt < md->batch_count) {
		if (md->ratio_cnt < md->sync_ratio) {
			/* Sync: reads beat writes via starved */
			if (md->starved < md->writes_starved &&
			    !list_empty(&md->fifo[MD_SR])) {
				rq = rq_entry_fifo(md->fifo[MD_SR].next);
				md->ratio_cnt++;
				goto done;
			}
			if (!list_empty(&md->fifo[MD_SW])) {
				/* If reads exist, only write if starved */
				if (!list_empty(&md->fifo[MD_SR]) &&
				    md->starved < md->writes_starved) {
					rq = rq_entry_fifo(md->fifo[MD_SR].next);
					md->ratio_cnt++;
					goto done;
				}
				md->starved = 0;
				rq = rq_entry_fifo(md->fifo[MD_SW].next);
				md->ratio_cnt++;
				goto done;
			}
			md->ratio_cnt = md->sync_ratio;
		}
		if (md->ratio_cnt >= md->sync_ratio) {
			if (!list_empty(&md->fifo[MD_AR])) {
				rq = rq_entry_fifo(md->fifo[MD_AR].next);
				md->ratio_cnt = 0;
				md->batch_cnt++;
				goto done;
			}
			if (!list_empty(&md->fifo[MD_AW])) {
				if (!list_empty(&md->fifo[MD_SR]) &&
				    md->starved < md->writes_starved) {
					rq = rq_entry_fifo(md->fifo[MD_SR].next);
					md->ratio_cnt = 0;
					md->batch_cnt = 0;
					goto done;
				}
				rq = rq_entry_fifo(md->fifo[MD_AW].next);
				md->ratio_cnt = 0;
				md->batch_cnt++;
				goto done;
			}
			md->ratio_cnt = 0;
			md->batch_cnt++;
		}
	}

	/* 4. Priority fallback: reads > writes */
	if (md->starved < md->writes_starved) {
		if (!list_empty(&md->fifo[MD_SR])) {
			rq = rq_entry_fifo(md->fifo[MD_SR].next);
			goto done;
		}
		if (!list_empty(&md->fifo[MD_AR])) {
			rq = rq_entry_fifo(md->fifo[MD_AR].next);
			goto done;
		}
	}
	if (!list_empty(&md->fifo[MD_SW])) {
		rq = rq_entry_fifo(md->fifo[MD_SW].next);
		md->starved = 0;
		goto done;
	}
	if (!list_empty(&md->fifo[MD_AW])) {
		rq = rq_entry_fifo(md->fifo[MD_AW].next);
		md->starved = 0;
		goto done;
	}
	if (!list_empty(&md->fifo[MD_SR])) {
		rq = rq_entry_fifo(md->fifo[MD_SR].next);
		goto done;
	}
	if (!list_empty(&md->fifo[MD_AR])) {
		rq = rq_entry_fifo(md->fifo[MD_AR].next);
		goto done;
	}
	return 0;

done:
	md->starved++;
	md->last_dispatch = jiffies;
	md->last_dir = md_dir(rq);
	rq_fifo_clear(rq);
	md_rb_del(md, rq);
	elv_dispatch_add_tail(q, rq);

	if (md->batch_cnt >= md->batch_count) {
		md->batch_cnt = 0;
		md->ratio_cnt = 0;
	}
	return 1;
}

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

	for (i = 0; i < MD_NR; i++) {
		INIT_LIST_HEAD(&md->fifo[i]);
		md->sort[i] = RB_ROOT;
		md->fifo_expire[i] = md_expire[i];
	}
	md->writes_starved = md_starved;
	md->sync_ratio = md_sync_ratio;
	md->batch_count = md_batch_count;
	md->fifo_batch = md_fifo_batch;
	md->last_dispatch = jiffies;
	md->last_dir = -1;
	md->front_merges = 1;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

/********** sysfs **********/
static ssize_t md_show(int v, char *p) { return sprintf(p, "%d\n", v); }
static void md_store(int *v, const char *p) { *v = simple_strtol((char *)p, NULL, 10); }

#define S1(name, field, conv) \
static ssize_t md_##name##_show(struct elevator_queue *e, char *p) { \
	struct md_data *m = e->elevator_data; int d = field; \
	if (conv) { d = jiffies_to_msecs(d); } return md_show(d, p); }
#define S2(name, ptr, mn, mx, conv) \
static ssize_t md_##name##_store(struct elevator_queue *e, const char *pg, size_t c) { \
	struct md_data *m = e->elevator_data; int d; md_store(&d, pg); \
	d = clamp(d, mn, mx); \
	if (conv) *(ptr) = msecs_to_jiffies(d); else *(ptr) = d; return c; }

S1(read_expire, m->fifo_expire[MD_SR], 1)  S2(read_expire, &m->fifo_expire[MD_SR], 0, INT_MAX, 1)
S1(write_expire, m->fifo_expire[MD_SW], 1) S2(write_expire, &m->fifo_expire[MD_SW], 0, INT_MAX, 1)
S1(ar_expire, m->fifo_expire[MD_AR], 1)    S2(ar_expire, &m->fifo_expire[MD_AR], 0, INT_MAX, 1)
S1(aw_expire, m->fifo_expire[MD_AW], 1)    S2(aw_expire, &m->fifo_expire[MD_AW], 0, INT_MAX, 1)
S1(starved, m->writes_starved, 0)          S2(starved, &m->writes_starved, 0, INT_MAX, 0)
S1(fifo_batch, m->fifo_batch, 0)           S2(fifo_batch, &m->fifo_batch, 0, INT_MAX, 0)
S1(sync_ratio, m->sync_ratio, 0)            S2(sync_ratio, &m->sync_ratio, 1, INT_MAX, 0)
S1(batch_count, m->batch_count, 0)          S2(batch_count, &m->batch_count, 1, INT_MAX, 0)

#define MD_ATTR(name) __ATTR(name, 0644, md_##name##_show, md_##name##_store)

static struct elv_fs_entry md_attrs[] = {
	MD_ATTR(read_expire),
	MD_ATTR(write_expire),
	MD_ATTR(ar_expire),
	MD_ATTR(aw_expire),
	MD_ATTR(starved),
	MD_ATTR(fifo_batch),
	MD_ATTR(sync_ratio),
	MD_ATTR(batch_count),
	__ATTR_NULL
};

#undef S1
#undef S2
#undef MD_ATTR

static struct elevator_type iosched_md = {
	.ops.sq = {
		.elevator_merge_fn		= md_merge,
		.elevator_merged_fn		= md_merged_request,
		.elevator_merge_req_fn		= md_merged_requests,
		.elevator_dispatch_fn		= md_dispatch_requests,
		.elevator_add_req_fn		= md_add_request,
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
MODULE_DESCRIPTION("Monolith IO scheduler: unified NOOP+Deadline+SIO+Zen+Maple+CFQ+Anxiety");
MODULE_VERSION("2.0");
