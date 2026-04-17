// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */
#include <linux/sched.h>
#include <linux/sysctl.h>
#include <linux/reciprocal_div.h>
#include <linux/topology.h>
#include <linux/cpufreq.h>
#include "sched.h"
#include <linux/prefer_silver.h>

int sysctl_prefer_silver = 1;
int sysctl_heavy_task_thresh = 50;
int sysctl_cpu_util_thresh = 85;
int sysctl_silver_trigger_freq = 1503000;

bool prefer_silver_check_freq(int cpu)
{
	unsigned int freq = cpufreq_quick_get(cpu);
	return freq < sysctl_silver_trigger_freq;
}

static inline unsigned long __scale_demand(u64 demand)
{
	unsigned int divisor = sched_ravg_window >> SCHED_CAPACITY_SHIFT;

	if (likely(divisor > 0))
		return (unsigned long)div64_u64(demand, divisor);

	return 0;
}

static inline unsigned long ps_task_util(struct task_struct *p)
{
#ifdef CONFIG_SCHED_WALT
	if (likely(sched_ravg_window > 0)) {
		u64 demand = p->ravg.demand;
		return (unsigned long)div64_u64(demand << SCHED_CAPACITY_SHIFT, sched_ravg_window);
	}
#endif
	return task_util(p);
}

unsigned long ps_cpu_util(int cpu)
{
#ifdef CONFIG_SCHED_WALT
	if (likely(sched_ravg_window > 0)) {
		u64 walt_cpu_util = cpu_rq(cpu)->walt_stats.cumulative_runnable_avg_scaled;

		walt_cpu_util <<= SCHED_CAPACITY_SHIFT;
		do_div(walt_cpu_util, sched_ravg_window);

		return min_t(unsigned long, (unsigned long)walt_cpu_util,
			     capacity_orig_of(cpu));
	}
#endif
	return cpu_util(cpu);
}

bool prefer_silver_check_task_util(struct task_struct *p)
{
	unsigned long thresh_load, thresh;
	struct reciprocal_value spc_rdiv = reciprocal_value(100);

	thresh_load = capacity_orig_of(task_cpu(p)) * sysctl_heavy_task_thresh;
	thresh = reciprocal_divide(thresh_load, spc_rdiv);

	if (ps_task_util(p) < thresh || __scale_demand(p->ravg.sum) < thresh)
		return true;

	return false;
}

bool prefer_silver_check_cpu_util(int cpu)
{
	return (capacity_orig_of(cpu) * sysctl_cpu_util_thresh) >
		(ps_cpu_util(cpu) * 100);
}

int find_best_silver_cpu(struct task_struct *p)
{
	int i, best_cpu = -1;
	unsigned long min_util = ULONG_MAX;

	for_each_cpu(i, &p->cpus_allowed) {
		unsigned long cur_util;

		if (cpu_topology[i].package_id != 0)
			continue;

		if (!prefer_silver_check_freq(i))
			continue;

		if (!prefer_silver_check_cpu_util(i))
			continue;

		cur_util = ps_cpu_util(i);
		if (cur_util < min_util) {
			min_util = cur_util;
			best_cpu = i;
		}
	}
	return best_cpu;
}
