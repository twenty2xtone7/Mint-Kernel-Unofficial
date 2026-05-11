// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#include <linux/sched.h>
#include <linux/sysctl.h>
#include <linux/topology.h>
#include <linux/cpufreq.h>
#include "sched.h"
#include <linux/prefer_silver.h>

int sysctl_prefer_silver = 1;
int sysctl_heavy_task_thresh = 50;
int sysctl_cpu_util_thresh = 85;
int sysctl_silver_trigger_freq = 1503000;

#define PREFER_SILVER_DEBUG
#ifdef PREFER_SILVER_DEBUG
#define ps_debug(fmt, ...) pr_info("[PREFER_SILVER] " fmt, ##__VA_ARGS__)
#else
#define ps_debug(fmt, ...) no_printk(fmt, ##__VA_ARGS__)
#endif

bool prefer_silver_check_freq(int cpu)
{
	unsigned int freq = cpufreq_quick_get(cpu);
	ps_debug("check_freq: cpu=%d freq=%u threshold=%d ret=%d",
		 cpu, freq, sysctl_silver_trigger_freq,
		 freq < sysctl_silver_trigger_freq);
	return freq < sysctl_silver_trigger_freq;
}

static inline unsigned long ps_task_util(struct task_struct *p)
{
	return task_util(p);
}

unsigned long ps_cpu_util(int cpu)
{
	return cpu_util(cpu);
}

 
bool prefer_silver_check_task_util(struct task_struct *p)
{
	unsigned long thresh;
	bool ret;

	thresh = capacity_orig_of(task_cpu(p)) *
		 sysctl_heavy_task_thresh / 100;

	ret = ps_task_util(p) * 100 < thresh * 107;
	ps_debug("check_task_util: pid=%d util=%lu thresh=%lu ret=%d",
		 p->pid, ps_task_util(p), thresh, ret);
	return ret;
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

	ps_debug("find_best_silver_cpu: pid=%d prefer_silver=%d",
		 p->pid, sysctl_prefer_silver);

	for_each_cpu(i, p->cpus_ptr) {
		unsigned long cur_util;

		if (cpu_topology[i].cluster_id != 0)
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

	ps_debug("find_best_silver_cpu: result best_cpu=%d", best_cpu);
	return best_cpu;
}

static int __init prefer_silver_init(void)
{
	pr_info("[PREFER_SILVER] init: sysctl_prefer_silver=%d heavy_task_thresh=%d cpu_util_thresh=%d trigger_freq=%d",
		sysctl_prefer_silver, sysctl_heavy_task_thresh,
		sysctl_cpu_util_thresh, sysctl_silver_trigger_freq);
	return 0;
}
late_initcall(prefer_silver_init);