// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023-2024 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

/**
 * DOC: SBalance description
 *
 * This is a simple IRQ balancer that polls every X number of milliseconds and
 * moves IRQs from the most interrupt-heavy CPU to the least interrupt-heavy
 * CPUs until the heaviest CPU is no longer the heaviest. IRQs are only moved
 * from one source CPU to any number of destination CPUs per balance run.
 * Balancing is skipped if the gap between the most interrupt-heavy CPU and the
 * least interrupt-heavy CPU is below the configured threshold of interrupts.
 *
 * The heaviest IRQs are targeted for migration in order to reduce the number of
 * IRQs to migrate. If moving an IRQ would reduce overall balance, then it won't
 * be migrated.
 *
 * The most interrupt-heavy CPU is calculated by scaling the number of new
 * interrupts on that CPU to the CPU's current capacity. This way, interrupt
 * heaviness takes into account factors such as thermal pressure and time spent
 * processing interrupts rather than just the sheer number of them. This also
 * makes SBalance aware of CPU asymmetry, where different CPUs can have
 * different performance capacities and be proportionally balanced.
 */

#define pr_fmt(fmt) "sbalance: " fmt

#include <linux/freezer.h>
#include <linux/irq.h>
#include <linux/list_sort.h>
#include <linux/sysctl.h>
#include "../sched/sched.h"
#include "internals.h"

/* timer_setup_on_stack is not available in this kernel */
#define timer_setup_on_stack(timer, callback, flags)			\
	__setup_timer_on_stack((timer), (TIMER_FUNC_TYPE)(callback),	\
			       (TIMER_DATA_TYPE)(timer), (flags))

/* Perform IRQ balancing every POLL_MS milliseconds */
static int sbalance_poll_ms = CONFIG_IRQ_SBALANCE_POLL_MSEC;

/*
 * There needs to be a difference of at least this many new interrupts between
 * the heaviest and least-heavy CPUs during the last polling window in order for
 * balancing to occur. This is to avoid balancing when the system is quiet.
 *
 * This threshold is compared to the _scaled_ interrupt counts per CPU; i.e.,
 * the number of interrupts scaled to the CPU's capacity.
 */
static int sbalance_threshold = CONFIG_IRQ_SBALANCE_THRESH;
static unsigned int sbalance_balanced;
static unsigned int sbalance_idle_runs;

struct bal_irq {
	struct list_head node;
	struct list_head move_node;
	struct rcu_head rcu;
	struct irq_desc *desc;
	unsigned long delta_nr;
	unsigned long old_nr;
	int prev_cpu;
};

struct bal_domain {
	struct list_head movable_irqs;
	unsigned long intrs;
	unsigned long old_total;
	int cpu;
};

static LIST_HEAD(bal_irq_list);
static DEFINE_SPINLOCK(bal_irq_lock);
static DEFINE_PER_CPU(struct bal_domain, balance_data);
static DEFINE_PER_CPU(unsigned long, cpu_cap);
static cpumask_t cpu_exclude_mask __read_mostly;

void sbalance_desc_add(struct irq_desc *desc)
{
	struct bal_irq *bi;

	bi = kmalloc(sizeof(*bi), GFP_KERNEL);
	if (WARN_ON(!bi))
		return;

	*bi = (typeof(*bi)){ .desc = desc };
	spin_lock(&bal_irq_lock);
	list_add_tail_rcu(&bi->node, &bal_irq_list);
	spin_unlock(&bal_irq_lock);
}

void sbalance_desc_del(struct irq_desc *desc)
{
	struct bal_irq *bi;

	spin_lock(&bal_irq_lock);
	list_for_each_entry(bi, &bal_irq_list, node) {
		if (bi->desc == desc) {
			list_del_rcu(&bi->node);
			kfree_rcu(bi, rcu);
			break;
		}
	}
	spin_unlock(&bal_irq_lock);
}

static int bal_irq_move_node_cmp(void *priv, struct list_head *lhs_p, struct list_head *rhs_p)
{
	const struct bal_irq *lhs = list_entry(lhs_p, typeof(*lhs), move_node);
	const struct bal_irq *rhs = list_entry(rhs_p, typeof(*rhs), move_node);

	return rhs->delta_nr - lhs->delta_nr;
}

/* Returns false if this IRQ should be totally ignored for this balancing run */
static bool update_irq_data(struct bal_irq *bi, int *cpu)
{
	struct irq_desc *desc = bi->desc;
	unsigned int nr;

	/*
	 * Get the CPU which currently has this IRQ affined. Due to hardware and
	 * irqchip driver quirks, a previously set affinity may not match the
	 * actual affinity of the IRQ. Therefore, we check the last CPU that the
	 * IRQ fired upon in order to determine its actual affinity.
	 */
	*cpu = READ_ONCE(desc->last_cpu);
	if (*cpu >= nr_cpu_ids)
		return false;

	/*
	 * Calculate the number of new interrupts from this IRQ. It is assumed
	 * that the IRQ has been running on the same CPU since the last
	 * balancing run. This might not hold true if the IRQ was moved by
	 * someone else since the last balancing run, or if the CPU this IRQ was
	 * previously running on has since gone offline.
	 */
	nr = *per_cpu_ptr(desc->kstat_irqs, *cpu);
	if (nr <= bi->old_nr) {
		bi->old_nr = nr;
		return false;
	}

	/* Calculate the number of new interrupts on this CPU from this IRQ */
	bi->delta_nr = nr - bi->old_nr;
	bi->old_nr = nr;
	return true;
}

static int move_irq_to_cpu(struct bal_irq *bi, int cpu)
{
	struct irq_desc *desc = bi->desc;
	int prev_cpu, ret;

	/* Set the affinity if it wasn't changed since we looked at it */
	raw_spin_lock_irq(&desc->lock);
	prev_cpu = cpumask_first(desc->irq_common_data.affinity);
	if (prev_cpu == bi->prev_cpu) {
		ret = irq_set_affinity_locked(&desc->irq_data, cpumask_of(cpu),
					      false);
	} else {
		bi->prev_cpu = prev_cpu;
		ret = -EINVAL;
	}
	raw_spin_unlock_irq(&desc->lock);

	if (!ret) {
		/* Update the old interrupt count using the new CPU */
		bi->old_nr = *per_cpu_ptr(desc->kstat_irqs, cpu);
		pr_debug("Moved IRQ%d (CPU%d -> CPU%d)\n",
			 irq_desc_get_irq(desc), prev_cpu, cpu);
	}
	return ret;
}

static unsigned int scale_intrs(unsigned int intrs, int cpu)
{
	/* Scale the number of interrupts to this CPU's current capacity */
	return intrs * SCHED_CAPACITY_SCALE / per_cpu(cpu_cap, cpu);
}

/* Returns true if IRQ balancing should stop.
 * When min_bd is non-NULL, we already know the minimum is in or past
 * that domain — used to avoid full rescans in the migration loop.
 */
static bool find_min_bd(const cpumask_t *mask, unsigned int max_intrs,
			struct bal_domain **min_bd,
			struct bal_domain *skip_until)
{
	unsigned int intrs, min_intrs = UINT_MAX;
	bool past_skip = (skip_until == NULL);
	struct bal_domain *bd;
	int cpu;

	for_each_cpu(cpu, mask) {
		bd = per_cpu_ptr(&balance_data, cpu);

		if (!past_skip) {
			if (bd == skip_until)
				past_skip = true;
			continue;
		}

		intrs = scale_intrs(bd->intrs, bd->cpu);

		if (intrs > max_intrs)
			return true;

		if (cpumask_test_cpu(cpu, &cpu_exclude_mask))
			continue;

		if (intrs < min_intrs) {
			min_intrs = intrs;
			*min_bd = bd;
		}
	}

	if (min_intrs == UINT_MAX)
		return true;

	return max_intrs - min_intrs < sbalance_threshold;
}

static void balance_irqs(void)
{
	static cpumask_t cpus;
	struct bal_domain *bd, *max_bd, *min_bd;
	unsigned int intrs, max_intrs;
	bool moved_irq = false;
	struct bal_irq *bi;
	int cpu;

	cpus_read_lock();
	rcu_read_lock();

	cpumask_copy(&cpus, cpu_active_mask);
	if (unlikely(cpumask_weight(&cpus) <= 1))
		goto unlock;

	/*
	 * Single pass: collect per-CPU capacity, interrupt counts, and
	 * identify the heaviest CPU with movable IRQs.
	 */
	max_intrs = 0;
	max_bd = NULL;
	for_each_cpu(cpu, &cpus) {
		per_cpu(cpu_cap, cpu) = cpu_rq(cpu)->cpu_capacity;

		bd = per_cpu_ptr(&balance_data, cpu);
		bd->intrs = kstat_cpu_irqs_sum(cpu) - bd->old_total;
		bd->old_total += bd->intrs;

		if (!bd->intrs)
			continue;

		if (cpumask_test_cpu(cpu, &cpu_exclude_mask))
			continue;

		intrs = scale_intrs(bd->intrs, bd->cpu);
		if (intrs > max_intrs) {
			max_intrs = intrs;
			max_bd = bd;
		}
	}

	if (!max_bd)
		goto unlock;

	list_for_each_entry_rcu(bi, &bal_irq_list, node) {
		if (!__irq_can_set_affinity(bi->desc))
			continue;

		if (!update_irq_data(bi, &cpu))
			continue;

		if (cpu != bi->prev_cpu) {
			bi->prev_cpu = cpu;
			continue;
		}

		bd = per_cpu_ptr(&balance_data, cpu);
		list_add_tail(&bi->move_node, &bd->movable_irqs);
	}

	/*
	 * If the heaviest CPU has no movable IRQs, scan down until we
	 * find one that does. Precompute a sorted list of CPUs by scaled
	 * interrupt count to avoid rescanning.
	 */
	if (list_empty(&max_bd->movable_irqs)) {
		struct bal_domain *fallback_bd = NULL;
		unsigned int fallback_intrs = 0;

		for_each_cpu(cpu, &cpus) {
			bd = per_cpu_ptr(&balance_data, cpu);
			if (bd == max_bd)
				continue;
			if (list_empty(&bd->movable_irqs))
				continue;
			if (cpumask_test_cpu(cpu, &cpu_exclude_mask))
				continue;
			intrs = scale_intrs(bd->intrs, bd->cpu);
			if (!intrs)
				continue;
			if (intrs > fallback_intrs) {
				fallback_intrs = intrs;
				fallback_bd = bd;
			}
		}

		if (!fallback_bd) {
			sbalance_idle_runs++;
			goto unlock;
		}
		max_intrs = fallback_intrs;
		max_bd = fallback_bd;
	}

	/* Find the CPU with the lowest relative interrupt count */
	if (find_min_bd(&cpus, max_intrs, &min_bd, NULL))
		goto unlock;

	/* Sort movable IRQs in descending order of number of new interrupts */
	list_sort(NULL, &max_bd->movable_irqs, bal_irq_move_node_cmp);

	/* Push IRQs away from the heaviest CPU to the least-heavy CPUs */
	list_for_each_entry(bi, &max_bd->movable_irqs, move_node) {
		intrs = scale_intrs(min_bd->intrs + bi->delta_nr, min_bd->cpu);
		if (intrs >= max_intrs)
			continue;

		if (move_irq_to_cpu(bi, min_bd->cpu))
			continue;

		moved_irq = true;

		min_bd->intrs += bi->delta_nr;
		max_bd->intrs -= min(bi->delta_nr, max_bd->intrs);
		max_intrs = scale_intrs(max_bd->intrs, max_bd->cpu);

		/*
		 * Resume the min search from the current min_bd — CPUs
		 * before it are already heavier, so we skip them.
		 */
		if (find_min_bd(&cpus, max_intrs, &min_bd, min_bd))
			break;
	}

	if (!moved_irq)
		goto unlock;
	sbalance_balanced++;
	sbalance_idle_runs = 0;
unlock:
	rcu_read_unlock();
	cpus_read_unlock();

	for_each_possible_cpu(cpu) {
		bd = per_cpu_ptr(&balance_data, cpu);
		INIT_LIST_HEAD(&bd->movable_irqs);
		bd->intrs = 0;
	}
}

struct process_timer {
	struct timer_list timer;
	struct task_struct *task;
};

static void process_timeout(struct timer_list *t)
{
	struct process_timer *timeout = from_timer(timeout, t, timer);

	wake_up_process(timeout->task);
}

static void sbalance_wait(long poll_jiffies)
{
	struct process_timer timer;

	/*
	 * Open code freezable_schedule_timeout_interruptible() in order to
	 * make the timer deferrable, so that it doesn't kick CPUs out of idle.
	 */
	freezer_do_not_count();
	__set_current_state(TASK_IDLE);
	timer.task = current;
	timer_setup_on_stack(&timer.timer, process_timeout, TIMER_DEFERRABLE);
	timer.timer.expires = jiffies + poll_jiffies;
	add_timer(&timer.timer);
	schedule();
	del_singleshot_timer_sync(&timer.timer);
	destroy_timer_on_stack(&timer.timer);
	freezer_count();
}

static int __noreturn sbalance_thread(void *data)
{
	struct bal_domain *bd;
	int cpu;

	/* Parse the list of CPUs to exclude, if any */
	if (cpulist_parse(CONFIG_SBALANCE_EXCLUDE_CPUS, &cpu_exclude_mask))
		cpu_exclude_mask = CPU_MASK_NONE;

	/* Initialize the data used for balancing */
	for_each_possible_cpu(cpu) {
		bd = per_cpu_ptr(&balance_data, cpu);
		INIT_LIST_HEAD(&bd->movable_irqs);
		bd->cpu = cpu;
	}

	set_freezable();
	while (1) {
		int ms = sbalance_poll_ms;

		/*
		 * Adaptive backoff: when the system is balanced and no
		 * IRQs have been moved for several runs, scale the poll
		 * interval up to 4x to reduce CPU wakeups. Resets to the
		 * configured minimum as soon as balancing is needed.
		 */
		if (sbalance_idle_runs > 3 && ms < 10000)
			ms = min(ms * 2, 10000);

		sbalance_wait(msecs_to_jiffies(ms));
		balance_irqs();
	}
}

static struct ctl_table sbalance_sysctl_table[] = {
	{
		.procname	= "poll_ms",
		.data		= &sbalance_poll_ms,
		.maxlen		= sizeof(sbalance_poll_ms),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "threshold",
		.data		= &sbalance_threshold,
		.maxlen		= sizeof(sbalance_threshold),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "balanced",
		.data		= &sbalance_balanced,
		.maxlen		= sizeof(sbalance_balanced),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{ }
};

static int __init sbalance_init(void)
{
	pr_info("starting sbalance thread\n");
	BUG_ON(IS_ERR(kthread_run(sbalance_thread, NULL, "sbalanced")));
	pr_info("sbalance thread started\n");

	register_sysctl("kernel/sbalance", sbalance_sysctl_table);
	return 0;
}
late_initcall(sbalance_init);
