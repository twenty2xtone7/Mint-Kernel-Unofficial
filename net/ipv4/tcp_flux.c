/*
 * Flux TCP congestion control
 *
 * Unified algorithm fusing BBR + Westwood + Cubic for maximum throughput
 * even on lossy links (cellular, WiFi):
 *
 *   BBR:      state machine (STARTUP/DRAIN/PROBE_BW/PROBE_RTT),
 *             delivery-rate bandwidth estimation, min-RTT tracking, pacing
 *   Westwood: ACK-rate bandwidth filter, loss-response ssthresh = bw * rtt_min,
 *             preserves window on non-congestion losses
 *   Cubic:    aggressive cubic-function window growth during congestion avoidance
 *
 * Designed for unreliable links: packet loss does not trigger window halving.
 * Instead, the window is set to (bw_estimate * min_rtt) / MSS, preserving
 * throughput on lossy cellular/WiFi connections.
 *
 * Copyright (C) 2022 flux
 */
#include <linux/module.h>
#include <net/tcp.h>
#include <linux/inet_diag.h>

#define FLUX_SCALE	8
#define FLUX_UNIT	(1 << FLUX_SCALE)
#define FLUX_BW_SCALE	24
#define FLUX_BW_UNIT	(1 << FLUX_BW_SCALE)
#define FLUX_RTT_MIN_US	55000	/* 55ms minimum RTT window for Westwood */

enum flux_state {
	FLUX_STARTUP,
	FLUX_DRAIN,
	FLUX_PROBE_BW,
	FLUX_PROBE_RTT,
};

struct flux {
	u32	bw_est;		/* bw in FLUX_BW_UNIT (pkts/uS << 24) */
	u32	bw_westwood;	/* Westwood-filtered bandwidth (bytes/sec) */
	u32	min_rtt_us;	/* minimum RTT seen (usec) */
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
};

/* Pacing gain cycle: probe, drain, cruise */
static const u32 flux_pacing_gain[] = {
	FLUX_UNIT * 115 / 100,
	FLUX_UNIT * 85 / 100,
	FLUX_UNIT, FLUX_UNIT,
	FLUX_UNIT, FLUX_UNIT,
	FLUX_UNIT, FLUX_UNIT,
};
#define FLUX_CYCLE_LEN		ARRAY_SIZE(flux_pacing_gain)
#define FLUX_CYCLE_RAND		7

static const u32 flux_high_gain		= FLUX_UNIT * 2885 / 1000 + 1;
static const u32 flux_drain_gain	= FLUX_UNIT * 1000 / 2885;
static const u32 flux_cwnd_gain		= FLUX_UNIT * 2;

/* Lossy-link tuning: tolerate more loss before reacting */
static const u32 flux_loss_thresh	= 3;	/* losses before tightening */
static const u32 flux_min_tso_rate	= 1200000;
static const u32 flux_min_rtt_win_sec	= 10;
static const u32 flux_probe_rtt_ms	= 200;

/* Westwood filter: exponential weighted moving average (alpha = 1/8) */
static u32 ww_filter(u32 prev, u32 sample)
{
	return prev ? ((7 * prev) + sample) >> 3 : sample;
}

static u32 bw_to_pkts(u32 bw)
{
	return bw >> FLUX_BW_SCALE;
}

/* Simplified cubic root using binary search */
static u32 flux_cubic_root(u64 x)
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

static void flux_init(struct sock *sk)
{
	struct flux *f = inet_csk_ca(sk);

	f->state = FLUX_STARTUP;
	f->probe_idx = prandom_u32_max(FLUX_CYCLE_LEN - FLUX_CYCLE_RAND);
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
}

/* Westwood bandwidth from ACK rate */
static void flux_westwood_update(struct sock *sk, u32 bytes, u32 rtt)
{
	struct flux *f = inet_csk_ca(sk);
	u32 delta = tcp_jiffies32 - f->ww_win_start;
	u32 win = max(rtt, FLUX_RTT_MIN_US) << 2;

	f->ww_acked += bytes;
	f->ww_rtt = rtt;
	f->ww_rtt_min = min(f->ww_rtt_min, rtt);

	if (delta > max_t(u32, usecs_to_jiffies(rtt),
			  usecs_to_jiffies(win))) {
		u32 bw = (f->ww_acked * USEC_PER_SEC) / jiffies_to_usecs(delta);
		f->bw_westwood = ww_filter(f->bw_westwood, bw);
		f->ww_acked = 0;
		f->ww_win_start = tcp_jiffies32;
	}
}

static void flux_pkts_acked(struct sock *sk, const struct ack_sample *sample)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct flux *f = inet_csk_ca(sk);

	if (sample->rtt_us <= 0)
		return;

	/* Filter min_rtt: only lower it by at most 20% per sample to avoid noise */
	if (!f->min_rtt_us)
		f->min_rtt_us = sample->rtt_us;
	else if (sample->rtt_us < f->min_rtt_us)
		f->min_rtt_us -= (f->min_rtt_us - sample->rtt_us) >> 2;

	if (!f->has_seen_rtt) {
		f->has_seen_rtt = 1;
		f->ww_rtt = sample->rtt_us;
		f->ww_rtt_min = sample->rtt_us;
	}

	flux_westwood_update(sk, sample->pkts_acked * tp->mss_cache,
			     sample->rtt_us);
}

/* Cubic-style increment cnt for tcp_cong_avoid_ai */
static u32 flux_cubic_cnt(struct flux *f, u32 cwnd)
{
	u32 t, offs, K, target, cnt;

	if (!f->epoch_start) {
		f->epoch_start = tcp_jiffies32;
		f->last_max_cwnd = max(f->last_max_cwnd, cwnd);
	}

	t = tcp_jiffies32 - f->epoch_start;
	K = flux_cubic_root((u64)max(f->last_max_cwnd, cwnd) * 256);

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

/* Westwood loss response: ssthresh = bw * rtt_min / MSS */
static u32 flux_bw_ssthresh(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct flux *f = inet_csk_ca(sk);
	u32 bw_bytes = f->bw_westwood;

	if (f->bw_est)
		bw_bytes = max(bw_bytes, bw_to_pkts(f->bw_est) * tp->mss_cache);

	if (bw_bytes && f->ww_rtt_min != ~0U / 1000)
		return max_t(u32, (bw_bytes * f->ww_rtt_min) /
			     USEC_PER_SEC / tp->mss_cache, 2);
	return max(tp->snd_cwnd >> 1, 2U);
}

static u32 flux_ssthresh(struct sock *sk)
{
	struct flux *f = inet_csk_ca(sk);
	f->last_max_cwnd = tcp_sk(sk)->snd_cwnd;
	f->epoch_start = 0;
	return flux_bw_ssthresh(sk);
}

static u32 flux_undo_cwnd(struct sock *sk)
{
	struct flux *f = inet_csk_ca(sk);
	return max(tcp_sk(sk)->snd_cwnd, f->prior_cwnd);
}

/* Core algorithm: called per ACK */
static void flux_main(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct flux *f = inet_csk_ca(sk);
	u32 bw, rate, cwnd;
	int round_start = 0;

	if (rs->delivered < 0 || rs->interval_us <= 0)
		return;

	/* Delivery-rate bandwidth (BBR-style) */
	bw = (u32)div64_u64((u64)rs->delivered * tp->mss_cache * USEC_PER_SEC,
			    rs->interval_us);
	bw <<= (FLUX_BW_SCALE - 10);

	/* Round boundary detection */
	if (before(rs->prior_delivered, f->next_round_delivered)) {
		f->next_round_delivered = tp->delivered;
		f->rtt_cnt++;
		round_start = 1;
	}

	/* Smooth bandwidth with EWMA on round boundaries */
	if (round_start) {
		if (bw > f->bw_est) {
			f->bw_est = bw;
		} else {
			f->bw_est = (f->bw_est * 7 + bw) >> 3;
		}
		f->rounds_since_bw = 0;
	} else {
		f->rounds_since_bw++;
	}

	bw = max(f->bw_est, f->bw_westwood);

	cwnd = tp->snd_cwnd;

	switch (f->state) {
	case FLUX_STARTUP:
		if (round_start) {
			f->last_max_cwnd = max(f->last_max_cwnd, cwnd);
			if (bw <= f->full_bw)
				f->full_bw_cnt++;
			else
				f->full_bw_cnt = 0;
			f->full_bw = max(f->full_bw, bw);
			if (f->full_bw_cnt >= 3) {
				f->state = FLUX_DRAIN;
			}
		}
		if (!tcp_in_slow_start(tp))
			tp->snd_ssthresh = flux_bw_ssthresh(sk);
		rate = flux_high_gain;
		cwnd = min(cwnd + 1U,
			   (u32)(bw * f->min_rtt_us / USEC_PER_SEC *
				 flux_high_gain / FLUX_UNIT));
		break;

	case FLUX_DRAIN:
		rate = flux_drain_gain;
		cwnd = bw * f->min_rtt_us / USEC_PER_SEC *
		       flux_cwnd_gain / FLUX_UNIT;
		if (round_start && cwnd <= tp->snd_cwnd)
			f->state = FLUX_PROBE_BW;
		break;

	case FLUX_PROBE_BW:
		if (round_start) {
			f->last_bw = bw;
			f->probe_idx = (f->probe_idx + 1) &
				       (FLUX_CYCLE_LEN - 1);
		}
		rate = flux_pacing_gain[f->probe_idx];

		if (tcp_in_slow_start(tp))
			tp->snd_ssthresh = flux_bw_ssthresh(sk);
		if (!tcp_in_slow_start(tp)) {
			f->cube_cnt = flux_cubic_cnt(f, cwnd);
			tcp_cong_avoid_ai(tp, f->cube_cnt, 1);
		}
		cwnd = tp->snd_cwnd;

		if (f->rtt_cnt > 0 && !f->probe_rtt_round_done &&
		    f->min_rtt_us &&
		    tcp_jiffies32 - f->probe_rtt_done_stamp >
		    flux_min_rtt_win_sec * HZ) {
			f->state = FLUX_PROBE_RTT;
			f->probe_rtt_round_done = 0;
			f->prior_cwnd = cwnd;
			tp->snd_cwnd = min(cwnd, 4U);
		}
		break;

	case FLUX_PROBE_RTT:
		rate = FLUX_UNIT;
		cwnd = min(cwnd, 4U);
		if (round_start)
			f->probe_rtt_round_done = 1;
		if (f->probe_rtt_round_done &&
		    tcp_jiffies32 - f->probe_rtt_done_stamp >=
		    msecs_to_jiffies(flux_probe_rtt_ms)) {
			f->probe_rtt_done_stamp = tcp_jiffies32;
			f->probe_rtt_round_done = 0;
			tp->snd_cwnd = max(cwnd, f->prior_cwnd);
			f->state = f->full_bw_reached ?
				   FLUX_PROBE_BW : FLUX_STARTUP;
		}
		break;
	}

	/* BDP cap */
	if (f->min_rtt_us && bw) {
		cwnd = min(cwnd, (u32)(bw * f->min_rtt_us / USEC_PER_SEC *
			   flux_cwnd_gain / FLUX_UNIT));
	}

	tp->snd_cwnd = max(cwnd, 2U);

	/* Pacing */
	if (rate) {
		u64 pr = (u64)bw * rate * tp->mss_cache;
		pr >>= FLUX_BW_SCALE + FLUX_SCALE;
		sk->sk_pacing_rate = min_t(u64, pr, sk->sk_max_pacing_rate);
	}
}

static void flux_set_state(struct sock *sk, u8 new_state)
{
	struct flux *f = inet_csk_ca(sk);

	switch (new_state) {
	case TCP_CA_Loss:
		f->prior_cwnd = tcp_sk(sk)->snd_cwnd;
		f->epoch_start = 0;
		f->loss_in_round = 1;

		tcp_sk(sk)->snd_ssthresh = flux_bw_ssthresh(sk);

		/* Packet conservation: maintain at least ssthresh */
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

static void flux_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct flux *f = inet_csk_ca(sk);

	switch (event) {
	case CA_EVENT_TX_START:
		f->idle_restart = 1;
		f->ww_win_start = tcp_jiffies32;
		f->ww_acked = 0;
		break;
	case CA_EVENT_LOSS:
		tcp_sk(sk)->snd_ssthresh = flux_bw_ssthresh(sk);
		f->ww_rtt_min = f->ww_rtt;
		break;
	case CA_EVENT_CWND_RESTART:
		f->epoch_start = 0;
		break;
	default:
		break;
	}
}

static struct tcp_congestion_ops tcp_flux __read_mostly = {
	.init		= flux_init,
	.ssthresh	= flux_ssthresh,
	.undo_cwnd	= flux_undo_cwnd,
	.cong_control	= flux_main,
	.pkts_acked	= flux_pkts_acked,
	.set_state	= flux_set_state,
	.cwnd_event	= flux_cwnd_event,
	.owner		= THIS_MODULE,
	.name		= "flux",
};

static int __init flux_register(void)
{
	BUILD_BUG_ON(sizeof(struct flux) > ICSK_CA_PRIV_SIZE);
	return tcp_register_congestion_control(&tcp_flux);
}

static void __exit flux_unregister(void)
{
	tcp_unregister_congestion_control(&tcp_flux);
}

module_init(flux_register);
module_exit(flux_unregister);

MODULE_AUTHOR("flux");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Flux TCP: unified BBR + Westwood + Cubic congestion control");
MODULE_VERSION("1.0");
