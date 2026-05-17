/*
 * Swift TCP congestion control
 *
 * Adaptive algorithm that detects link quality and tunes itself:
 *
 *   Stable link (low loss, low jitter):
 *     - Aggressive probe gain, tight Westwood window, fast Cubic growth
 *     - Maximizes throughput on fast/reliable networks (WiFi, Ethernet)
 *
 *   Unstable link (lossy, high jitter):
 *     - Smooth bandwidth, wide Westwood window, gentle pacing
 *     - Preserves throughput on cellular, congested WiFi
 *
 *   Mode transitions are smoothed with hysteresis to avoid oscillation.
 *
 * Core: BBR state machine + Westwood ACK-rate bandwidth + Cubic growth,
 *       with link-quality-adaptive parameters throughout.
 *
 * Copyright (C) 2022 swift
 */

#include <linux/module.h>
#include <net/tcp.h>
#include <linux/inet_diag.h>

#define SWIFT_SCALE	8
#define SWIFT_UNIT	(1 << SWIFT_SCALE)
#define SWIFT_BW_SCALE	24
#define SWIFT_BW_UNIT	(1 << SWIFT_BW_SCALE)
#define SWIFT_RTT_MIN_US	55000

enum swift_state {
	SWIFT_STARTUP,
	SWIFT_DRAIN,
	SWIFT_PROBE_BW,
	SWIFT_PROBE_RTT,
};

struct swift {
	u32	bw_est;
	u32	bw_westwood;
	u32	min_rtt_us;
	u32	last_bw;

	u16	state:3,
		probe_idx:3,
		rounds_since_bw:5,
		full_bw_cnt:2,
		full_bw_reached:1,
		has_seen_rtt:1,
		probe_rtt_round_done:1,
		idle_restart:1,
		loss_in_round:1;

	u32	rtt_cnt;
	u32	next_round_delivered;
	u32	full_bw;
	u32	prior_cwnd;
	u32	probe_rtt_done_stamp;

	u32	epoch_start;
	u32	last_max_cwnd;
	u16	cube_cnt;

	u32	ww_win_start;
	u32	ww_acked;
	u32	ww_rtt;
	u32	ww_rtt_min;

	/* Link quality tracking */
	u32	lq_loss_start;
	u32	lq_delivered;
	u32	lq_lost;
	u16	adapt_level;	/* 0=stable .. 3=unstable */
};

/* Per-mode pacing gain tables */
static const u32 swift_pacing_gain_stable[] = {
	SWIFT_UNIT * 125 / 100,
	SWIFT_UNIT * 75 / 100,
	SWIFT_UNIT, SWIFT_UNIT, SWIFT_UNIT,
	SWIFT_UNIT, SWIFT_UNIT, SWIFT_UNIT,
};

static const u32 swift_pacing_gain_normal[] = {
	SWIFT_UNIT * 115 / 100,
	SWIFT_UNIT * 85 / 100,
	SWIFT_UNIT, SWIFT_UNIT,
	SWIFT_UNIT, SWIFT_UNIT,
	SWIFT_UNIT, SWIFT_UNIT,
};

static const u32 swift_pacing_gain_unstable[] = {
	SWIFT_UNIT * 110 / 100,
	SWIFT_UNIT * 90 / 100,
	SWIFT_UNIT, SWIFT_UNIT,
	SWIFT_UNIT, SWIFT_UNIT,
	SWIFT_UNIT, SWIFT_UNIT,
};

static const u32 swift_pacing_gain_bad[] = {
	SWIFT_UNIT * 105 / 100,
	SWIFT_UNIT * 95 / 100,
	SWIFT_UNIT, SWIFT_UNIT,
	SWIFT_UNIT, SWIFT_UNIT,
	SWIFT_UNIT, SWIFT_UNIT,
};

#define SWIFT_CYCLE_LEN		8
#define SWIFT_CYCLE_RAND	7

static const u32 swift_high_gain	= SWIFT_UNIT * 2885 / 1000 + 1;
static const u32 swift_drain_gain	= SWIFT_UNIT * 1000 / 2885;
static const u32 swift_cwnd_gain	= SWIFT_UNIT * 2;

static const u32 swift_min_tso_rate	= 1200000;
static const u32 swift_min_rtt_win_sec	= 10;
static const u32 swift_probe_rtt_ms	= 200;

/* Loss rate thresholds per adapt level */
static const u32 swift_lq_up_thresh[]	= { 2, 5, 10, 20 };  /* permille up */
static const u32 swift_lq_down_thresh[]	= { 1, 3, 5, 10 };  /* permille down */

static const u32 *swift_pacing_gain[] = {
	swift_pacing_gain_stable,
	swift_pacing_gain_normal,
	swift_pacing_gain_unstable,
	swift_pacing_gain_bad,
};

static u32 ww_filter(u32 prev, u32 sample)
{
	return prev ? ((7 * prev) + sample) >> 3 : sample;
}

static u32 bw_to_pkts(u32 bw)
{
	return bw >> SWIFT_BW_SCALE;
}

static u32 swift_cubic_root(u64 x)
{
	u32 l = 0, r = 256, m;
	while (l < r) {
		m = (l + r + 1) >> 1;
		if ((u64)m * m * m <= x)
			l = m;
		else
			r = m - 1;
	}
	return l;
}

/* Evaluate link quality and update adapt_level */
static void swift_eval_link_quality(struct sock *sk)
{
	struct swift *f = inet_csk_ca(sk);
	u32 losses;
	u32 rate;

	losses = f->lq_lost;
	f->lq_lost = 0;
	f->lq_delivered = 0;

	if (!f->lq_delivered)
		return;

	rate = losses * 1000 / f->lq_delivered; /* permille */

	if (rate > swift_lq_up_thresh[f->adapt_level]) {
		if (f->adapt_level < 3)
			f->adapt_level++;
	} else if (rate < swift_lq_down_thresh[f->adapt_level]) {
		if (f->adapt_level > 0)
			f->adapt_level--;
	}
}

static void swift_init(struct sock *sk)
{
	struct swift *f = inet_csk_ca(sk);

	f->state = SWIFT_STARTUP;
	f->probe_idx = prandom_u32_max(SWIFT_CYCLE_LEN - SWIFT_CYCLE_RAND);
	f->full_bw = 0;
	f->full_bw_cnt = 0;
	f->full_bw_reached = 0;
	f->rounds_since_bw = 0;
	f->rtt_cnt = 0;
	f->next_round_delivered = 0;
	f->prior_cwnd = 0;
	f->has_seen_rtt = 0;
	f->idle_restart = 0;
	f->loss_in_round = 0;
	f->probe_rtt_round_done = 0;
	f->probe_rtt_done_stamp = 0;
	f->min_rtt_us = 0;
	f->bw_est = 0;
	f->bw_westwood = 0;
	f->last_bw = 0;
	f->epoch_start = 0;
	f->last_max_cwnd = 0;
	f->cube_cnt = 0;
	f->ww_win_start = tcp_jiffies32;
	f->ww_acked = 0;
	f->ww_rtt = ~0U / 1000;
	f->ww_rtt_min = ~0U / 1000;
	f->adapt_level = 1;
	f->lq_delivered = 0;
	f->lq_lost = 0;
	f->lq_loss_start = tcp_jiffies32;
}

static void swift_westwood_update(struct sock *sk, u32 bytes, u32 rtt)
{
	struct swift *f = inet_csk_ca(sk);
	u32 delta = tcp_jiffies32 - f->ww_win_start;
	/* Window scales with adapt_level: tighter on stable, wider on unstable */
	u32 win = max(rtt, SWIFT_RTT_MIN_US) << (1 + f->adapt_level);

	f->ww_acked += bytes;
	f->ww_rtt = rtt;
	f->ww_rtt_min = min(f->ww_rtt_min, rtt);

	if (delta > max_t(u32, usecs_to_jiffies(rtt),
			  usecs_to_jiffies(win))) {
		u32 bw = (f->ww_acked * USEC_PER_SEC) / jiffies_to_usecs(delta);
		/* More smoothing on unstable links */
		if (f->adapt_level >= 2)
			f->bw_westwood = ww_filter(f->bw_westwood, bw);
		else
			f->bw_westwood = bw;
		f->ww_acked = 0;
		f->ww_win_start = tcp_jiffies32;
	}
}

static void swift_pkts_acked(struct sock *sk, const struct ack_sample *sample)
{
	struct swift *f = inet_csk_ca(sk);

	if (sample->rtt_us <= 0)
		return;

	if (!f->min_rtt_us)
		f->min_rtt_us = sample->rtt_us;
	else if (sample->rtt_us < f->min_rtt_us)
		f->min_rtt_us -= (f->min_rtt_us - sample->rtt_us) >> 2;

	if (!f->has_seen_rtt) {
		f->has_seen_rtt = 1;
		f->ww_rtt = sample->rtt_us;
		f->ww_rtt_min = sample->rtt_us;
	}

	swift_westwood_update(sk, sample->pkts_acked *
			      tcp_sk(sk)->mss_cache, sample->rtt_us);
}

static u32 swift_cubic_cnt(struct swift *f, u32 cwnd)
{
	u32 t, offs, K, target, cnt;

	if (!f->epoch_start) {
		f->epoch_start = tcp_jiffies32;
		f->last_max_cwnd = max(f->last_max_cwnd, cwnd);
	}

	t = tcp_jiffies32 - f->epoch_start;
	K = swift_cubic_root((u64)max(f->last_max_cwnd, cwnd) * 256);

	if (t < K)	offs = K - t;
	else		offs = t - K;

	target = (offs * offs * offs) >> 10;
	target = (t < K) ? max(f->last_max_cwnd, cwnd) - target
			 : max(f->last_max_cwnd, cwnd) + target;

	if (target > cwnd)
		cnt = cwnd / (target - cwnd);
	else
		cnt = 100 * cwnd;

	return clamp(cnt, 2U, 20U);
}

static u32 swift_bw_ssthresh(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct swift *f = inet_csk_ca(sk);
	u32 bw_bytes = f->bw_westwood;

	if (f->bw_est)
		bw_bytes = max(bw_bytes, bw_to_pkts(f->bw_est) * tp->mss_cache);

	if (bw_bytes && f->ww_rtt_min != ~0U / 1000)
		return max_t(u32, (bw_bytes * f->ww_rtt_min) /
			     USEC_PER_SEC / tp->mss_cache, 2);
	return max(tp->snd_cwnd >> 1, 2U);
}

static u32 swift_ssthresh(struct sock *sk)
{
	struct swift *f = inet_csk_ca(sk);
	f->last_max_cwnd = tcp_sk(sk)->snd_cwnd;
	f->epoch_start = 0;
	return swift_bw_ssthresh(sk);
}

static u32 swift_undo_cwnd(struct sock *sk)
{
	struct swift *f = inet_csk_ca(sk);
	return max(tcp_sk(sk)->snd_cwnd, f->prior_cwnd);
}

static void swift_main(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct swift *f = inet_csk_ca(sk);
	const u32 *gain_table;
	u32 bw, rate, cwnd;
	int round_start = 0;

	if (rs->delivered < 0 || rs->interval_us <= 0)
		return;

	/* Track loss for link quality */
	f->lq_delivered += rs->delivered;
	f->lq_lost += rs->losses;

	/* Evaluate link quality every 4 RTTs */
	if (f->rtt_cnt > 0 && (f->rtt_cnt % 4) == 0) {
		f->rtt_cnt = 0;
		swift_eval_link_quality(sk);
	}

	bw = (u32)div64_u64((u64)rs->delivered * tp->mss_cache * USEC_PER_SEC,
			    rs->interval_us);
	bw <<= (SWIFT_BW_SCALE - 10);

	if (before(rs->prior_delivered, f->next_round_delivered)) {
		f->next_round_delivered = tp->delivered;
		f->rtt_cnt++;
		round_start = 1;
	}

	/* Smooth bandwidth: less smoothing on stable links */
	if (round_start) {
		if (bw > f->bw_est) {
			f->bw_est = bw;
		} else if (f->adapt_level >= 2) {
			f->bw_est = (f->bw_est * 7 + bw) >> 3;
		} else {
			f->bw_est = (f->bw_est * 3 + bw) >> 2;
		}
		f->rounds_since_bw = 0;
	} else {
		f->rounds_since_bw++;
	}

	bw = max(f->bw_est, f->bw_westwood);
	cwnd = tp->snd_cwnd;

	switch (f->state) {
	case SWIFT_STARTUP:
		if (round_start) {
			f->last_max_cwnd = max(f->last_max_cwnd, cwnd);
			if (bw <= f->full_bw)
				f->full_bw_cnt++;
			else
				f->full_bw_cnt = 0;
			f->full_bw = max(f->full_bw, bw);
			if (f->full_bw_cnt >= 3)
				f->state = SWIFT_DRAIN;
		}
		if (!tcp_in_slow_start(tp))
			tp->snd_ssthresh = swift_bw_ssthresh(sk);
		rate = swift_high_gain;
		cwnd = min(cwnd + 1U,
			   (u32)(bw * f->min_rtt_us / USEC_PER_SEC *
				 swift_high_gain / SWIFT_UNIT));
		break;

	case SWIFT_DRAIN:
		rate = swift_drain_gain;
		cwnd = bw * f->min_rtt_us / USEC_PER_SEC *
		       swift_cwnd_gain / SWIFT_UNIT;
		if (round_start && cwnd <= tp->snd_cwnd)
			f->state = SWIFT_PROBE_BW;
		break;

	case SWIFT_PROBE_BW:
		if (round_start) {
			f->last_bw = bw;
			f->probe_idx = (f->probe_idx + 1) & (SWIFT_CYCLE_LEN - 1);
		}
		gain_table = swift_pacing_gain[f->adapt_level];
		rate = gain_table[f->probe_idx];

		if (tcp_in_slow_start(tp))
			tp->snd_ssthresh = swift_bw_ssthresh(sk);
		if (!tcp_in_slow_start(tp)) {
			f->cube_cnt = swift_cubic_cnt(f, cwnd);
			tcp_cong_avoid_ai(tp, f->cube_cnt, 1);
		}
		cwnd = tp->snd_cwnd;

		if (f->rtt_cnt > 0 && !f->probe_rtt_round_done &&
		    f->min_rtt_us &&
		    tcp_jiffies32 - f->probe_rtt_done_stamp >
		    swift_min_rtt_win_sec * HZ) {
			f->state = SWIFT_PROBE_RTT;
			f->probe_rtt_round_done = 0;
			f->prior_cwnd = cwnd;
			tp->snd_cwnd = min(cwnd, 4U);
		}
		break;

	case SWIFT_PROBE_RTT:
		rate = SWIFT_UNIT;
		cwnd = min(cwnd, 4U);
		if (round_start)
			f->probe_rtt_round_done = 1;
		if (f->probe_rtt_round_done &&
		    tcp_jiffies32 - f->probe_rtt_done_stamp >=
		    msecs_to_jiffies(swift_probe_rtt_ms)) {
			f->probe_rtt_done_stamp = tcp_jiffies32;
			f->probe_rtt_round_done = 0;
			tp->snd_cwnd = max(cwnd, f->prior_cwnd);
			f->state = f->full_bw_reached ?
				   SWIFT_PROBE_BW : SWIFT_STARTUP;
		}
		break;
	}

	if (f->min_rtt_us && bw) {
		cwnd = min(cwnd, (u32)(bw * f->min_rtt_us / USEC_PER_SEC *
			   swift_cwnd_gain / SWIFT_UNIT));
	}

	tp->snd_cwnd = max(cwnd, 2U);

	if (rate) {
		u64 pr = (u64)bw * rate * tp->mss_cache;
		pr >>= SWIFT_BW_SCALE + SWIFT_SCALE;
		sk->sk_pacing_rate = min_t(u64, pr, sk->sk_max_pacing_rate);
	}
}

static void swift_set_state(struct sock *sk, u8 new_state)
{
	struct swift *f = inet_csk_ca(sk);

	switch (new_state) {
	case TCP_CA_Loss:
		f->prior_cwnd = tcp_sk(sk)->snd_cwnd;
		f->epoch_start = 0;
		f->loss_in_round = 1;
		tcp_sk(sk)->snd_ssthresh = swift_bw_ssthresh(sk);
		tcp_sk(sk)->snd_cwnd = max(tcp_sk(sk)->snd_ssthresh,
					   tcp_sk(sk)->snd_cwnd >> 1);
		break;
	case TCP_CA_Recovery:
		break;
	case TCP_CA_Open:
		f->loss_in_round = 0;
		break;
	default:
		break;
	}
}

static void swift_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct swift *f = inet_csk_ca(sk);

	switch (event) {
	case CA_EVENT_TX_START:
		f->idle_restart = 1;
		f->ww_win_start = tcp_jiffies32;
		f->ww_acked = 0;
		break;
	case CA_EVENT_LOSS:
		tcp_sk(sk)->snd_ssthresh = swift_bw_ssthresh(sk);
		f->ww_rtt_min = f->ww_rtt;
		break;
	case CA_EVENT_CWND_RESTART:
		f->epoch_start = 0;
		break;
	default:
		break;
	}
}

static struct tcp_congestion_ops tcp_swift __read_mostly = {
	.init		= swift_init,
	.ssthresh	= swift_ssthresh,
	.undo_cwnd	= swift_undo_cwnd,
	.cong_control	= swift_main,
	.pkts_acked	= swift_pkts_acked,
	.set_state	= swift_set_state,
	.cwnd_event	= swift_cwnd_event,
	.owner		= THIS_MODULE,
	.name		= "swift",
};

static int __init swift_register(void)
{
	BUILD_BUG_ON(sizeof(struct swift) > ICSK_CA_PRIV_SIZE);
	return tcp_register_congestion_control(&tcp_swift);
}

static void __exit swift_unregister(void)
{
	tcp_unregister_congestion_control(&tcp_swift);
}

module_init(swift_register);
module_exit(swift_unregister);

MODULE_AUTHOR("swift");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Swift TCP: link-quality-adaptive congestion control");
MODULE_VERSION("1.0");
