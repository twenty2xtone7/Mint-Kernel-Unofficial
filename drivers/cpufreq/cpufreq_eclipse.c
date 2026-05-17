/*
 *  drivers/cpufreq/cpufreq_eclipse.c
 *
 *  Eclipse CPU governor
 *
 *  Unified governor that fuses four approaches:
 *    performance   : perf_mode toggle forces max freq
 *    schedutil     : boosted_cpu_util() for fine-grained load
 *    energy_adaptive : dynamic headroom scaling by load bands
 *    moonbeam      : DBS sampling, peak tracking, ramp boost, adaptive headroom
 *
 *  Core update on each DBS tick:
 *    load = max(dbs_idle_load, sched_util_load)
 *    load = peak_floor(load)
 *    load += ramp_boost
 *    load = adaptive_headroom(load)
 *    if load > up_threshold -> max freq
 *    else -> min_freq + (load% * range)
 *
 *  Copyright (C)  2022
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched/cpufreq.h>
#include <linux/slab.h>
#include <linux/sched/sysctl.h>

#include "cpufreq_governor.h"

extern unsigned long boosted_cpu_util(int cpu);

struct eclipse_tuners {
	struct gov_attr_set attr_set;
};

struct eclipse_policy_dbs {
	struct policy_dbs_info policy_dbs;
	unsigned int prev_load;
	unsigned int ramp_boost;
	unsigned int peak_load;
	unsigned int sample_count;
};

#define to_eclipse_policy_dbs(_pdbs) \
	container_of(_pdbs, struct eclipse_policy_dbs, policy_dbs)

static unsigned int eclipse_get_target_load(struct eclipse_policy_dbs *pd,
					    unsigned int load)
{
	unsigned int target = load;

	if (load > pd->peak_load)
		pd->peak_load = load;
	else
		pd->peak_load -= pd->peak_load >> 10;

	target = max(target, pd->peak_load >> 2);

	if (load > pd->prev_load) {
		unsigned int bump = load - pd->prev_load;
		if (bump > pd->ramp_boost)
			pd->ramp_boost = min(bump, 25U);
		pd->ramp_boost += 2;
	}
	pd->ramp_boost -= pd->ramp_boost >> 3;
	pd->ramp_boost = min(pd->ramp_boost, 25U);
	target = min(target + pd->ramp_boost, 100U);

	pd->prev_load = load;

	if (target < 25)
		target = min(target * 2, 50U);
	else if (target < 50)
		target += target - (target >> 2);
	else
		target += target >> 1;

	return min(target, 100U);
}

static void eclipse_update(struct cpufreq_policy *policy)
{
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct eclipse_policy_dbs *pd = to_eclipse_policy_dbs(policy_dbs);
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	unsigned int dbs_load, sched_load, target_load, freq_next;

	dbs_load = dbs_update(policy);
	if (!dbs_load)
		dbs_load = pd->prev_load;

	sched_load = boosted_cpu_util(policy->cpu) * 100 /
		     arch_scale_cpu_capacity(NULL, policy->cpu);

	target_load = max(dbs_load, sched_load);
	target_load = eclipse_get_target_load(pd, target_load);

	if (dbs_data->sampling_down_factor > 1 &&
	    target_load < dbs_data->up_threshold) {
		if (pd->sample_count < dbs_data->sampling_down_factor - 1) {
			pd->sample_count++;
			return;
		}
		pd->sample_count = 0;
	}

	if (target_load >= dbs_data->up_threshold)
		freq_next = policy->max;
	else
		freq_next = policy->cpuinfo.min_freq +
			    target_load * (policy->cpuinfo.max_freq -
					   policy->cpuinfo.min_freq) / 100;

	__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);
}

static unsigned int eclipse_dbs_update(struct cpufreq_policy *policy)
{
	eclipse_update(policy);
	return 0;
}

/***** Governor callbacks *****/

static struct policy_dbs_info *eclipse_alloc(void)
{
	struct eclipse_policy_dbs *pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	return pd ? &pd->policy_dbs : NULL;
}

static void eclipse_free(struct policy_dbs_info *policy_dbs)
{
	kfree(to_eclipse_policy_dbs(policy_dbs));
}

static int eclipse_init(struct dbs_data *dbs_data)
{
	struct eclipse_tuners *tuners;

	tuners = kzalloc(sizeof(*tuners), GFP_KERNEL);
	if (!tuners)
		return -ENOMEM;

	dbs_data->up_threshold = 50;
	dbs_data->sampling_down_factor = 1;
	dbs_data->ignore_nice_load = 0;
	dbs_data->io_is_busy = 1;

	dbs_data->tuners = tuners;
	return 0;
}

static void eclipse_exit(struct dbs_data *dbs_data)
{
	kfree(dbs_data->tuners);
}

static void eclipse_start(struct cpufreq_policy *policy)
{
	struct eclipse_policy_dbs *pd = to_eclipse_policy_dbs(
						policy->governor_data);

	pd->prev_load = 0;
	pd->ramp_boost = 0;
	pd->peak_load = 0;
	pd->sample_count = 0;
}

/***** Sysfs tunables *****/

static ssize_t store_io_is_busy(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_data->io_is_busy = !!input;
	gov_update_cpu_data(dbs_data);
	return count;
}

static ssize_t store_up_threshold(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1 || input > 100 || input < 1)
		return -EINVAL;
	dbs_data->up_threshold = input;
	return count;
}

static ssize_t store_sampling_down_factor(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct policy_dbs_info *policy_dbs;
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1 || input > 100000 || input < 1)
		return -EINVAL;

	dbs_data->sampling_down_factor = input;

	list_for_each_entry(policy_dbs, &attr_set->policy_list, list) {
		mutex_lock(&policy_dbs->update_mutex);
		policy_dbs->rate_mult = 1;
		mutex_unlock(&policy_dbs->update_mutex);
	}
	return count;
}

static ssize_t store_ignore_nice_load(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	if (input > 1)
		input = 1;
	if (input == dbs_data->ignore_nice_load)
		return count;
	dbs_data->ignore_nice_load = input;
	gov_update_cpu_data(dbs_data);
	return count;
}

gov_show_one_common(sampling_rate);
gov_show_one_common(up_threshold);
gov_show_one_common(sampling_down_factor);
gov_show_one_common(ignore_nice_load);
gov_show_one_common(io_is_busy);

gov_attr_rw(sampling_rate);
gov_attr_rw(io_is_busy);
gov_attr_rw(up_threshold);
gov_attr_rw(sampling_down_factor);
gov_attr_rw(ignore_nice_load);

static struct attribute *eclipse_attributes[] = {
	&sampling_rate.attr,
	&up_threshold.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&io_is_busy.attr,
	NULL
};

/***** Registration *****/

static struct dbs_governor eclipse_dbs_gov = {
	.gov = CPUFREQ_DBS_GOVERNOR_INITIALIZER("eclipse"),
	.kobj_type = { .default_attrs = eclipse_attributes },
	.gov_dbs_update = eclipse_dbs_update,
	.alloc = eclipse_alloc,
	.free = eclipse_free,
	.init = eclipse_init,
	.exit = eclipse_exit,
	.start = eclipse_start,
};

#define CPU_FREQ_GOV_ECLIPSE	(&eclipse_dbs_gov.gov)

static int __init cpufreq_gov_eclipse_init(void)
{
	return cpufreq_register_governor(CPU_FREQ_GOV_ECLIPSE);
}

static void __exit cpufreq_gov_eclipse_exit(void)
{
	cpufreq_unregister_governor(CPU_FREQ_GOV_ECLIPSE);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ECLIPSE
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return CPU_FREQ_GOV_ECLIPSE;
}

fs_initcall(cpufreq_gov_eclipse_init);
#else
module_init(cpufreq_gov_eclipse_init);
#endif
module_exit(cpufreq_gov_eclipse_exit);

MODULE_AUTHOR("Eclipse");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Eclipse CPU governor: fusion of performance, schedutil, energy_adaptive, and moonbeam");
