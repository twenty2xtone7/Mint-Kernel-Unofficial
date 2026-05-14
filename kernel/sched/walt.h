/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __WALT_H
#define __WALT_H

#define MAX_MARGIN_LEVELS 5

struct rq_flags;

#ifdef CONFIG_SCHED_WALT

void walt_update_task_ravg(struct task_struct *p, struct rq *rq, int event,
		u64 wallclock, u64 irqtime);
void walt_inc_cumulative_runnable_avg(struct rq *rq, struct task_struct *p);
void walt_dec_cumulative_runnable_avg(struct rq *rq, struct task_struct *p);

void walt_fixup_busy_time(struct task_struct *p, int new_cpu);
void walt_init_new_task_load(struct task_struct *p);
void walt_mark_task_starting(struct task_struct *p);
void walt_set_window_start(struct rq *rq, struct rq_flags *rf);
void walt_migrate_sync_cpu(int cpu);
u64 walt_ktime_clock(void);
void walt_account_irqtime(int cpu, struct task_struct *curr, u64 delta,
                                  u64 wallclock);

u64 walt_irqload(int cpu);
int walt_cpu_high_irqload(int cpu);

#else /* CONFIG_SCHED_WALT */

static inline void walt_update_task_ravg(struct task_struct *p, struct rq *rq,
		int event, u64 wallclock, u64 irqtime) { }
static inline void walt_inc_cumulative_runnable_avg(struct rq *rq, struct task_struct *p) { }
static inline void walt_dec_cumulative_runnable_avg(struct rq *rq, struct task_struct *p) { }
static inline void walt_fixup_busy_time(struct task_struct *p, int new_cpu) { }
static inline void walt_init_new_task_load(struct task_struct *p) { }
static inline void walt_mark_task_starting(struct task_struct *p) { }
static inline void walt_set_window_start(struct rq *rq, struct rq_flags *rf) { }
static inline void walt_migrate_sync_cpu(int cpu) { }
static inline u64 walt_ktime_clock(void) { return 0; }

#define walt_cpu_high_irqload(cpu) false

#endif /* CONFIG_SCHED_WALT */

#if defined(CONFIG_CFS_BANDWIDTH) && defined(CONFIG_SCHED_WALT)
void walt_inc_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p);
void walt_dec_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p);
#else
static inline void walt_inc_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p) { }
static inline void walt_dec_cfs_cumulative_runnable_avg(struct cfs_rq *rq,
		struct task_struct *p) { }
#endif

extern bool walt_disabled;

extern unsigned int sysctl_sched_user_hint;
extern unsigned int sysctl_sched_window_stats_policy;
extern unsigned int sysctl_sched_group_upmigrate_pct;
extern unsigned int sysctl_sched_group_downmigrate_pct;
extern int sysctl_sched_boost;
extern unsigned int sysctl_sched_conservative_pl;
extern unsigned int sysctl_sched_many_wakeup_threshold;
extern unsigned int sysctl_sched_walt_rotate_big_tasks;
extern unsigned int sysctl_sched_min_task_util_for_boost;
extern unsigned int sysctl_sched_min_task_util_for_colocation;
extern unsigned int sysctl_sched_asym_cap_sibling_freq_match_pct;
extern unsigned int sysctl_sched_coloc_downmigrate_ns;
extern unsigned int sysctl_sched_task_unfilter_period;
extern unsigned int sysctl_sched_busy_hyst_enable_cpus;
extern unsigned int sysctl_sched_busy_hyst;
extern unsigned int sysctl_sched_coloc_busy_hyst_enable_cpus;
extern unsigned int sysctl_sched_coloc_busy_hyst;
extern unsigned int sysctl_sched_coloc_busy_hyst_max_ms;
extern unsigned int sysctl_sched_ravg_window_nr_ticks;
extern unsigned int sysctl_sched_dynamic_ravg_window_enable;
extern unsigned int sysctl_sched_prefer_spread;
extern unsigned int sysctl_walt_rtg_cfs_boost_prio;
extern unsigned int sysctl_walt_low_latency_task_threshold;
extern unsigned int sysctl_sched_capacity_margin_up[MAX_MARGIN_LEVELS];
extern unsigned int sysctl_sched_capacity_margin_down[MAX_MARGIN_LEVELS];
extern unsigned int sysctl_sched_force_lb_enable;
extern unsigned int sysctl_memcg_stat_show_subtree;
extern int sched_user_hint_max;
extern int min_cfs_boost_prio;
extern int max_cfs_boost_prio;

int walt_proc_user_hint_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);
int walt_proc_group_thresholds_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);
int sched_boost_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);
int proc_douintvec_minmax_schedhyst(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);
int sched_ravg_window_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);
int sched_updown_migrate_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos);

#endif
