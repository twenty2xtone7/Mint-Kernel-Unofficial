// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

static inline void lsub_positive(unsigned long *val, unsigned long dmin)
{
	if (*val > dmin)
		*val -= dmin;
	else
		*val = 0;
}

static inline bool available_idle_cpu(int cpu)
{
	return idle_cpu(cpu);
}

static inline unsigned long cass_task_util_est(struct task_struct *p)
{
#ifdef CONFIG_SCHED_WALT
	if (likely(!walt_disabled && sysctl_sched_use_walt_task_util))
		return (p->ravg.demand /
			(walt_ravg_window >> SCHED_CAPACITY_SHIFT));
#endif
	return max_t(unsigned long, READ_ONCE(p->se.avg.util_avg),
		     max_t(unsigned long, READ_ONCE(p->se.avg.util_est.ewma),
			   READ_ONCE(p->se.avg.util_est.enqueued)));
}

/**
 * DOC: Capacity Aware Superset Scheduler (CASS) description
 *
 * The Capacity Aware Superset Scheduler (CASS) optimizes runqueue selection of
 * CFS tasks. By using CPU capacity as a basis for comparing the relative
 * utilization between different CPUs, CASS fairly balances load across CPUs of
 * varying capacities. This results in improved multi-core performance,
 * especially when CPUs are overutilized because CASS doesn't clip a CPU's
 * utilization when it eclipses the CPU's capacity.
 *
 * As a superset of capacity aware scheduling, CASS implements a hierarchy of
 * criteria to determine the better CPU to wake a task upon between CPUs that
 * have the same relative utilization. This way, single-core performance,
 * latency, and cache affinity are all optimized where possible.
 *
 * CASS doesn't feature explicit energy awareness but its basic load balancing
 * principle results in decreased overall energy, often better than what is
 * possible with explicit energy awareness. By fairly balancing load based on
 * relative utilization, all CPUs are kept at their lowest P-state necessary to
 * satisfy the overall load at any given moment.
 */

struct cass_cpu_cand {
	int cpu;
	unsigned int exit_lat;
	unsigned long cap;
	unsigned long util;
};

static __always_inline
unsigned long cass_cpu_util(int cpu, bool sync)
{
	struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
	unsigned long util = READ_ONCE(cfs_rq->avg.util_avg);

	if (sync && cpu == smp_processor_id())
		lsub_positive(&util, task_util(current));

	if (sched_feat(UTIL_EST))
		util = max_t(unsigned long, util,
			     READ_ONCE(cfs_rq->avg.util_est.enqueued));

	return util;
}

static __always_inline
void cass_fill_cand(struct cass_cpu_cand *cand, int cpu,
		    struct task_struct *p, unsigned long p_util,
		    bool prefer_sync, int prev_cpu)
{
	struct cpuidle_state *state;
	bool idle;

	idle = (prefer_sync && cpu == smp_processor_id()) ||
	       available_idle_cpu(cpu);

	cand->cpu = cpu;
	cand->exit_lat = idle ? 1 : 0;
	if (idle) {
		state = idle_get_state(cpu_rq(cpu));
		if (state)
			cand->exit_lat += state->exit_latency;
	}
	cand->util = cass_cpu_util(cpu, prefer_sync);
	if (cpu != task_cpu(p))
		cand->util += p_util;
	cand->cap = capacity_of(cpu);
	cand->util = cand->util * SCHED_CAPACITY_SCALE / cand->cap;
}

/* Returns true if @a is a better CPU than @b */
static __always_inline
int cass_cpu_better(const struct cass_cpu_cand *a,
		    const struct cass_cpu_cand *b,
		    int prev_cpu, bool sync)
{
	/* Prefer the CPU with lower relative utilization */
	if (a->util != b->util)
		return b->util > a->util;

	/* Prefer the current CPU for sync wakes */
	if (sync) {
		if (a->cpu == smp_processor_id())
			return 1;
		if (b->cpu == smp_processor_id())
			return 0;
	}

	/* Prefer the CPU with higher capacity */
	if (a->cap != b->cap)
		return a->cap > b->cap;

	/* Prefer the CPU with lower idle exit latency */
	if (a->exit_lat != b->exit_lat)
		return b->exit_lat > a->exit_lat;

	/* Prefer the previous CPU */
	if (a->cpu == prev_cpu)
		return 1;
	if (b->cpu == prev_cpu)
		return 0;

	/* Prefer the CPU that shares a cache with the previous CPU */
	return cpus_share_cache(a->cpu, prev_cpu) >
	       cpus_share_cache(b->cpu, prev_cpu);
}

static int cass_best_cpu(struct task_struct *p, int prev_cpu, bool sync)
{
	struct cass_cpu_cand cands[2], *best = cands, *curr;
	unsigned long p_util;
	bool has_idle = false;
	int cidx = 1, cpu;

	p_util = clamp(cass_task_util_est(p),
		       uclamp_eff_value(p, UCLAMP_MIN),
		       uclamp_eff_value(p, UCLAMP_MAX));

	/* Fast path: idle prev_cpu is always the best */
	if (likely(cpumask_test_cpu(prev_cpu, p->cpus_ptr) &&
		   available_idle_cpu(prev_cpu)))
		return prev_cpu;

	/* Seed best with a worst-case sentinel so every CPU is compared */
	best->cpu = prev_cpu;
	best->util = ULONG_MAX;
	best->cap = 0;
	best->exit_lat = UINT_MAX;

	for_each_cpu_and(cpu, p->cpus_ptr, cpu_active_mask) {
		curr = &cands[cidx];

		/* Skip non-idle CPUs once we found an idle one */
		if (has_idle && !(sync && cpu == smp_processor_id()) &&
		    !available_idle_cpu(cpu))
			continue;

		cass_fill_cand(curr, cpu, p, p_util, sync, prev_cpu);

		if (cass_cpu_better(curr, best, prev_cpu, sync)) {
			best = curr;
			cidx ^= 1;
			if (available_idle_cpu(cpu) ||
			    (sync && cpu == smp_processor_id()))
				has_idle = true;
		}
	}

	return best->cpu;
}

static int cass_select_task_rq_fair(struct task_struct *p, int prev_cpu,
				    int sd_flag, int wake_flags,
				    int sibling_count_hint)
{
	bool sync;

	if (sd_flag & SD_BALANCE_EXEC)
		return prev_cpu;

	if (unlikely(!cpumask_intersects(p->cpus_ptr, cpu_active_mask)))
		return cpumask_first(p->cpus_ptr);

	if (!(sd_flag & SD_BALANCE_FORK))
		sync_entity_load_avg(&p->se);

	sync = (wake_flags & WF_SYNC) && !(current->flags & PF_EXITING);
	return cass_best_cpu(p, prev_cpu, sync);
}

static struct ctl_table cass_ctl_table[] = {
	{
		.procname	= "cass_active",
		.data		= NULL,
		.maxlen		= 0,
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{ }
};

static int __init cass_init(void)
{
	static int cass_active = 1;

	cass_ctl_table[0].data = &cass_active;
	cass_ctl_table[0].maxlen = sizeof(cass_active);
	register_sysctl("kernel", cass_ctl_table);
	pr_alert("CASS scheduler active\n");
	return 0;
}
core_initcall(cass_init);
