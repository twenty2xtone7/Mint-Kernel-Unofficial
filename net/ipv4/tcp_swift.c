// SPDX-License-Identifier: GPL-2.0
/*
 * Swift TCP congestion control v3.0
 *
 * Adaptive BBR-style state machine with Westwood bandwidth estimation,
 * Cubic congestion avoidance, and ML bandit parameter tuning.
 *
 * Core design:
 *   - BBR state machine: STARTUP → DRAIN → PROBE_BW ⟷ PROBE_RTT
 *   - Westwood: windowed max bandwidth filter for robust BDP estimation
 *   - Cubic: growth curve in PROBE_BW for safe above-BDP exploration
 *   - ML bandit: ε-greedy selection between 4 tuning profiles based on
 *     a reward function = delivered²/(elapsed * rtt), favoring throughput
 *     and low latency simultaneously
 *
 * Link quality adaptation:
 *   - Monitors loss rate over ML evaluation windows
 *   - 4 adaptation levels from aggressive (stable link) to conservative
 *     (lossy/cellular link)
 *   - Each level selects between bandit arms with different probe gains,
 *     BW smoothing, and Westwood filter widths
 *
 * Copyright (C) 2026 swift
 */

#include <linux/module.h>
#include <net/tcp.h>
#include <linux/inet_diag.h>
#include <linux/win_minmax.h>

/* Fixed-point scaling for pacing gain (8-bit fraction) */
#define SWIFT_SCALE		8
#define SWIFT_UNIT		(1 << SWIFT_SCALE)

/*
 * Bandwidth stored as pkts/usec in 24-bit fixed-point.
 * bw = delivered_pkts * BW_UNIT / interval_us
 * This gives bw in units of "packets per microsecond << 24".
 * To recover raw pkts/sec: raw = (u64)bw * USEC_PER_SEC >> BW_SCALE
 */
#define SWIFT_BW_SCALE		24
#define SWIFT_BW_UNIT		(1 << SWIFT_BW_SCALE)

enum swift_state {
	SWIFT_STARTUP,
	SWIFT_DRAIN,
	SWIFT_PROBE_BW,
	SWIFT_PROBE_RTT,
};

/* Per-arm tuning profile for the ML bandit */
struct swift_arm {
	u16 probe_boost;	/* probe gain scalar: 1000 = 1.0x, 1200 = 1.2x */
	u8  smooth_shift;	/* EWMA decay for BW smoothing (higher = smoother) */
	u8  ww_win;		/* Westwood windowed-max filter width in RTTs */
};

struct swift {
	/* Bandwidth estimation (pkts/usec << BW_SCALE) */
	u32	bw_hi;			/* Smoothed peak BW */

	/* RTT tracking (BBR-style windowed minimum) */
	u32	min_rtt_us;		/* Current windowed min RTT */
	u32	min_rtt_stamp;		/* tcp_jiffies32 when min_rtt was set */

	/* BBR state machine */
	u32	rtt_cnt;		/* Round trip counter */
	u32	next_round_delivered;	/* Delivered count at next round boundary */
	u32	full_bw;		/* Peak BW seen during STARTUP */
	u32	prior_cwnd;		/* Saved cwnd before PROBE_RTT */
	u32	probe_rtt_done_stamp;	/* When PROBE_RTT started */

	/* Cubic state */
	u32	epoch_start;		/* tcp_jiffies32 at cubic epoch start */
	u32	last_max_cwnd;		/* cwnd at last loss (cubic target) */

	/* Westwood windowed max BW filter */
	u32	next_rtt_delivered_ww;
	u32	ww_rtt_cnt;
	struct	minmax bw_ww;		/* Windowed max BW */

	/* Link quality tracking */
	u32	lq_delivered;		/* Packets delivered in eval window */
	u32	lq_lost;		/* Packets lost in eval window */

	/* ML bandit state */
	u16	ml_reward_acc;		/* Accumulated delivered for reward */
	u16	ml_arm_reward[4];	/* Smoothed reward per arm */

	/* State flags - no bitfields to avoid size bugs */
	u8	state;
	u8	probe_idx;		/* Current position in probe cycle */
	u8	full_bw_cnt;		/* Rounds without BW growth in STARTUP */
	u8	full_bw_reached;	/* Has STARTUP completed? */
	u8	probe_rtt_round_done;
	u8	adapt_level;		/* Link quality: 0=stable, 3=very lossy */
	u8	ml_arm;			/* Current bandit arm */
	u8	ml_best_arm;		/* Best performing arm */
	u8	ml_epsilon;		/* Exploration rate (0-100) */
};

/* Arm profiles: [probe_boost, smooth_shift, ww_window_rtts] */
static const struct swift_arm swift_arms[4] = {
	{ 1300, 2, 6  },	/* arm 0: Sprint - aggressive probing, fast reaction */
	{ 1150, 3, 8  },	/* arm 1: Cruise - balanced throughput/stability */
	{ 1075, 3, 10 },	/* arm 2: Endure - conservative, stable */
	{ 1025, 4, 12 },	/* arm 3: Survive - ultra-conservative, lossy links */
};

/*
 * BBR-style 8-slot pacing gain cycle for PROBE_BW.
 * Slot 0: probe above BDP to discover new bandwidth
 * Slot 1: drain any queue built during probe
 * Slots 2-7: cruise at 1.0x estimated BW
 */
static const u16 swift_pacing_gain[] = {
	SWIFT_UNIT * 125 / 100,		/* 1.25x: probe */
	SWIFT_UNIT * 75 / 100,		/* 0.75x: drain */
	SWIFT_UNIT, SWIFT_UNIT,		/* 1.0x: cruise */
	SWIFT_UNIT, SWIFT_UNIT,
	SWIFT_UNIT, SWIFT_UNIT,
};

#define SWIFT_CYCLE_LEN		8
#define SWIFT_ML_INTERVAL	8	/* ML evaluation every N rounds */

/* BBR-like gains */
static const u32 swift_startup_pacing = SWIFT_UNIT * 277 / 100;	/* ~2.77x */
static const u32 swift_drain_pacing   = SWIFT_UNIT * 100 / 277;	/* ~0.36x */
static const u32 swift_cwnd_gain      = SWIFT_UNIT * 2;		/* 2x BDP */
static const u32 swift_startup_cwnd   = SWIFT_UNIT * 277 / 100;	/* 2.77x BDP */

static const u32 swift_min_tso_rate    = 1200000;
static const u32 swift_min_rtt_win_sec = 10;	/* RTT filter window (seconds) */
static const u32 swift_probe_rtt_ms    = 200;	/* PROBE_RTT duration */

/* Loss rate thresholds per adapt_level (per-mille of delivered) */
static const u32 swift_lq_up_thresh[]  = { 2, 5, 10, 20 };
static const u32 swift_lq_down_thresh[] = { 1, 3, 5, 10 };


/* ── Helpers ── */

/*
 * Compute BDP in packets from bandwidth and RTT.
 *   bw:   pkts/usec << BW_SCALE
 *   gain: SWIFT_UNIT-scaled multiplier
 * Returns BDP in packets (plain u32).
 */
static u32 swift_bdp(u32 bw, u32 min_rtt_us, u32 gain)
{
	u64 bdp;

	if (!min_rtt_us || !bw)
		return TCP_INIT_CWND;

	/* bdp = bw * rtt_us >> BW_SCALE gives packets (bw is pkts/usec << BW_SCALE) */
	bdp = (u64)bw * min_rtt_us >> SWIFT_BW_SCALE;
	/* Apply gain */
	bdp = bdp * gain >> SWIFT_SCALE;
	return max_t(u32, bdp, 2);
}

/*
 * Convert BW estimate to pacing rate in bytes/sec.
 *   bw:   pkts/usec << BW_SCALE
 *   gain: SWIFT_UNIT-scaled multiplier
 */
static u32 swift_pacing_rate(u32 bw, u32 mss, u32 gain, u64 max_rate)
{
	u64 rate;

	/* rate = bw * mss * gain * USEC_PER_SEC >> (BW_SCALE + SCALE) */
	rate = (u64)bw * mss;
	rate *= gain;
	rate *= USEC_PER_SEC;
	rate >>= SWIFT_BW_SCALE + SWIFT_SCALE;
	rate = max_t(u64, rate, swift_min_tso_rate);
	return min_t(u64, rate, max_rate);
}

/* Get best available bandwidth estimate */
static u32 swift_max_bw(const struct swift *f)
{
	return max(f->bw_hi, (u32)minmax_get(&f->bw_ww));
}

/* Cube root via Newton-Raphson (replaces broken binary search capped at 256) */
static u32 swift_cbrt(u64 x)
{
	u64 r;
	int i;

	if (x <= 1)
		return (u32)x;

	r = 1ULL << ((fls64(x) + 2) / 3);
	for (i = 0; i < 6; i++)
		r = (2 * r + div64_u64(x, r * r)) / 3;
	while (r * r * r > x)
		r--;
	return (u32)r;
}


/* ── ML Bandit ── */

static void swift_ml_select_arm(struct swift *f)
{
	u8 best = 0;
	u16 best_val = 0;
	int i;

	for (i = 0; i < 4; i++) {
		if (f->ml_arm_reward[i] > best_val) {
			best_val = f->ml_arm_reward[i];
			best = i;
		}
	}
	f->ml_best_arm = best;

	/* ε-greedy: explore with probability epsilon/100 */
	if (prandom_u32_max(100) < f->ml_epsilon)
		f->ml_arm = prandom_u32_max(4);
	else
		f->ml_arm = best;

	/* Decay exploration toward 5% floor */
	if (f->ml_epsilon > 5)
		f->ml_epsilon--;
}

static void swift_ml_update_reward(struct swift *f, u32 delivered,
				   u32 elapsed_us, u32 rtt_us)
{
	u32 reward;

	if (!elapsed_us || !rtt_us)
		return;

	/* reward = delivered²*1000 / (elapsed * rtt) — favors throughput & low RTT */
	reward = (u32)div64_u64((u64)delivered * delivered * 1000,
				(u64)elapsed_us * rtt_us);
	reward = min_t(u32, reward, 65535);

	/* Exponential smoothing (weight 3:1 old:new) */
	if (f->ml_arm_reward[f->ml_arm])
		f->ml_arm_reward[f->ml_arm] =
			((u32)f->ml_arm_reward[f->ml_arm] * 3 + reward) >> 2;
	else
		f->ml_arm_reward[f->ml_arm] = reward;
}


/* ── Link Quality ── */

static void swift_eval_link_quality(struct swift *f)
{
	u32 loss_rate;

	if (!f->lq_delivered)
		return;

	loss_rate = f->lq_lost * 1000 / f->lq_delivered;
	f->lq_lost = 0;
	f->lq_delivered = 0;

	if (loss_rate > swift_lq_up_thresh[f->adapt_level]) {
		if (f->adapt_level < 3)
			f->adapt_level++;
	} else if (loss_rate < swift_lq_down_thresh[f->adapt_level]) {
		if (f->adapt_level > 0)
			f->adapt_level--;
	}
}


/* ── Cubic Growth ── */

/*
 * Compute cubic cwnd target for congestion avoidance in PROBE_BW.
 * Uses standard Cubic curve: C*(t-K)³ + Wmax
 * where K = cbrt(Wmax * β / C), t = time since epoch.
 */
static u32 swift_cubic_target(struct swift *f, u32 cwnd)
{
	u32 t, K, offs, target;

	if (!f->epoch_start) {
		f->epoch_start = tcp_jiffies32;
		f->last_max_cwnd = max(f->last_max_cwnd, cwnd);
	}

	t = (tcp_jiffies32 - f->epoch_start) * (USEC_PER_SEC / HZ);
	K = swift_cbrt((u64)(f->last_max_cwnd) * USEC_PER_SEC / HZ / 10);

	if (t < K)
		offs = K - t;
	else
		offs = t - K;

	target = (u32)div64_u64((u64)offs * offs * offs * 10,
				(u64)USEC_PER_SEC * USEC_PER_SEC / HZ);

	if (t < K)
		target = f->last_max_cwnd > target ?
			 f->last_max_cwnd - target : 0;
	else
		target = f->last_max_cwnd + target;

	return max_t(u32, target, 2);
}


/* ── Core Algorithm ── */

static void swift_init(struct sock *sk)
{
	struct swift *f = inet_csk_ca(sk);
	int i;

	memset(f, 0, sizeof(*f));
	f->state = SWIFT_STARTUP;
	f->probe_idx = prandom_u32_max(SWIFT_CYCLE_LEN);
	f->adapt_level = 1;
	f->ml_epsilon = 20;
	f->min_rtt_stamp = tcp_jiffies32;
	minmax_reset(&f->bw_ww, 0, 0);

	for (i = 0; i < 4; i++)
		f->ml_arm_reward[i] = 0;
}

static void swift_pkts_acked(struct sock *sk, const struct ack_sample *sample)
{
	struct swift *f = inet_csk_ca(sk);
	bool filter_expired;

	if (sample->rtt_us <= 0)
		return;

	/*
	 * BBR-style windowed minimum RTT:
	 * Accept new sample if it's lower than current min, or if the
	 * filter window has expired (forcing a refresh).
	 */
	filter_expired = (tcp_jiffies32 - f->min_rtt_stamp >
			  swift_min_rtt_win_sec * HZ);

	if (!f->min_rtt_us || sample->rtt_us < f->min_rtt_us || filter_expired) {
		f->min_rtt_us = sample->rtt_us;
		f->min_rtt_stamp = tcp_jiffies32;
	}
}

static void swift_update_bw(struct swift *f, const struct sock *sk,
			    const struct rate_sample *rs)
{
	const struct tcp_sock *tp = tcp_sk(sk);
	const struct swift_arm *arm = &swift_arms[f->ml_arm];
	u32 bw;

	if (rs->delivered <= 0 || rs->interval_us <= 0)
		return;

	/*
	 * Compute BW in pkts/usec << BW_SCALE.
	 * This matches BBR's unit system: bw = delivered * BW_UNIT / interval_us
	 */
	bw = (u32)div64_u64((u64)rs->delivered * SWIFT_BW_UNIT,
			    (u64)rs->interval_us);

	/* Westwood: windowed max filter over N RTTs */
	if (!before(tp->delivered, f->next_rtt_delivered_ww)) {
		f->next_rtt_delivered_ww = tp->delivered;
		f->ww_rtt_cnt++;
	}
	minmax_running_max(&f->bw_ww, arm->ww_win, f->ww_rtt_cnt, bw);

	/* EWMA: instant-up, smooth-down (per-arm decay rate) */
	if (bw >= f->bw_hi) {
		f->bw_hi = bw;
	} else {
		u32 mask = (1U << arm->smooth_shift) - 1;
		f->bw_hi = ((u64)f->bw_hi * mask + bw) >> arm->smooth_shift;
	}
}

static void swift_check_full_bw(struct swift *f)
{
	u32 bw_thresh;

	if (f->full_bw_reached)
		return;

	bw_thresh = (u64)f->full_bw * 5 >> 2;	/* 1.25x previous peak */
	if (swift_max_bw(f) >= bw_thresh) {
		f->full_bw = swift_max_bw(f);
		f->full_bw_cnt = 0;
		return;
	}

	/* BW has plateaued — count rounds without growth */
	f->full_bw_cnt++;
	if (f->full_bw_cnt >= 3)
		f->full_bw_reached = 1;
}

static u32 swift_ssthresh(struct sock *sk)
{
	struct swift *f = inet_csk_ca(sk);

	f->last_max_cwnd = tcp_sk(sk)->snd_cwnd;
	f->epoch_start = 0;
	return max(swift_bdp(swift_max_bw(f), f->min_rtt_us, SWIFT_UNIT), 2U);
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
	const struct swift_arm *arm;
	u32 bw, pacing_g, cwnd_g;
	int round_start = 0;

	if (rs->delivered < 0 || rs->interval_us <= 0)
		return;

	/* Accumulate link quality stats */
	f->lq_delivered += rs->delivered;
	f->lq_lost += rs->losses;

	/* Update bandwidth estimate */
	swift_update_bw(f, sk, rs);

	/*
	 * Detect new round trip.
	 * A new round starts when we receive an ACK for a packet that was
	 * sent AFTER the round boundary marker (next_round_delivered).
	 *
	 * CRITICAL FIX: Original had before() [inverted], which triggered
	 * round_start on every ACK within the round instead of once per RTT.
	 */
	if (!before(rs->prior_delivered, f->next_round_delivered)) {
		f->next_round_delivered = tp->delivered;
		f->rtt_cnt++;
		round_start = 1;
	}

	/* Periodically evaluate link quality and run ML bandit */
	if (round_start && (f->rtt_cnt % SWIFT_ML_INTERVAL) == 0) {
		swift_eval_link_quality(f);

		/* Update reward for current arm before selecting new one */
		if (f->min_rtt_us && f->ml_reward_acc) {
			u32 elapsed = jiffies_to_usecs(
				tcp_jiffies32 - f->epoch_start);
			swift_ml_update_reward(f, f->ml_reward_acc,
					       elapsed, f->min_rtt_us);
		}
		f->ml_reward_acc = 0;
		swift_ml_select_arm(f);
	}

	/* Accumulate delivered bytes for reward computation */
	f->ml_reward_acc += rs->delivered;

	arm = &swift_arms[f->ml_arm];
	bw = swift_max_bw(f);

	/* ── State machine ── */
	switch (f->state) {
	case SWIFT_STARTUP:
		/*
		 * Ramp up aggressively to fill the pipe.
		 * CRITICAL FIX: full_bw_cnt was a 2-bit bitfield (max 3)
		 * but checked against >= 5, so STARTUP never exited.
		 * Now uses a full u8.
		 */
		if (round_start)
			swift_check_full_bw(f);

		if (f->full_bw_reached) {
			f->state = SWIFT_DRAIN;
			break;
		}

		pacing_g = swift_startup_pacing * arm->probe_boost / 1000;
		cwnd_g = swift_startup_cwnd;
		break;

	case SWIFT_DRAIN:
		/* Drain excess queue built during STARTUP */
		pacing_g = swift_drain_pacing;
		cwnd_g = swift_cwnd_gain;

		if (round_start &&
		    tp->packets_out <=
		    swift_bdp(bw, f->min_rtt_us, SWIFT_UNIT))
			f->state = SWIFT_PROBE_BW;
		break;

	case SWIFT_PROBE_BW:
		/* Cycle through probe/drain/cruise phases */
		if (round_start)
			f->probe_idx = (f->probe_idx + 1) &
				       (SWIFT_CYCLE_LEN - 1);

		/*
		 * CRITICAL FIX: Original always read gain[0] instead of
		 * gain[probe_idx]. The entire probe/drain cycle was dead.
		 */
		pacing_g = swift_pacing_gain[f->probe_idx];

		/* Scale the probe slot (0) by the arm's boost factor */
		if (f->probe_idx == 0)
			pacing_g = pacing_g * arm->probe_boost / 1000;

		cwnd_g = swift_cwnd_gain;

		/* Cubic growth: slowly grow cwnd above BDP */
		if (!tcp_in_slow_start(tp)) {
			u32 target = swift_cubic_target(f, tp->snd_cwnd);

			if (target > tp->snd_cwnd)
				tp->snd_cwnd++;
		} else {
			tp->snd_ssthresh = swift_bdp(bw, f->min_rtt_us,
						     SWIFT_UNIT);
		}

		/* Enter PROBE_RTT when min_rtt filter has expired */
		if (tcp_jiffies32 - f->min_rtt_stamp >
		    swift_min_rtt_win_sec * HZ) {
			f->state = SWIFT_PROBE_RTT;
			f->probe_rtt_round_done = 0;
			f->prior_cwnd = tp->snd_cwnd;
			f->probe_rtt_done_stamp = tcp_jiffies32;
		}
		break;

	case SWIFT_PROBE_RTT:
		/* Reduce cwnd to minimum to measure true path RTT */
		pacing_g = SWIFT_UNIT;
		cwnd_g = SWIFT_UNIT;	/* 1x BDP = minimal window */

		tp->snd_cwnd = max(swift_bdp(bw, f->min_rtt_us,
					     SWIFT_UNIT), 4U);

		if (round_start)
			f->probe_rtt_round_done = 1;

		if (f->probe_rtt_round_done &&
		    tcp_jiffies32 - f->probe_rtt_done_stamp >=
		    msecs_to_jiffies(swift_probe_rtt_ms)) {
			/* Exit PROBE_RTT: restore cwnd, reset RTT stamp */
			f->min_rtt_stamp = tcp_jiffies32;
			tp->snd_cwnd = max(tp->snd_cwnd, f->prior_cwnd);
			f->state = f->full_bw_reached ?
				   SWIFT_PROBE_BW : SWIFT_STARTUP;
		}
		break;
	}

	/* Set cwnd = BDP * cwnd_gain, clamped to minimum 2 */
	if (f->state != SWIFT_PROBE_RTT) {
		u32 cwnd = swift_bdp(bw, f->min_rtt_us, cwnd_g);

		tp->snd_cwnd = max_t(u32, cwnd, 2);
	}

	/* Set pacing rate = BW * pacing_gain * MSS */
	sk->sk_pacing_rate = swift_pacing_rate(bw, tp->mss_cache,
					       pacing_g,
					       sk->sk_max_pacing_rate);
}

static void swift_set_state(struct sock *sk, u8 new_state)
{
	struct swift *f = inet_csk_ca(sk);

	switch (new_state) {
	case TCP_CA_Loss:
		f->prior_cwnd = tcp_sk(sk)->snd_cwnd;
		f->epoch_start = 0;
		tcp_sk(sk)->snd_ssthresh = swift_ssthresh(sk);
		break;
	case TCP_CA_Recovery:
		f->prior_cwnd = tcp_sk(sk)->snd_cwnd;
		tcp_sk(sk)->snd_ssthresh = swift_ssthresh(sk);
		break;
	default:
		break;
	}
}

static void swift_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct swift *f = inet_csk_ca(sk);

	if (event == CA_EVENT_CWND_RESTART)
		f->epoch_start = 0;
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
MODULE_DESCRIPTION("Swift TCP: ML-enhanced adaptive congestion control");
MODULE_VERSION("3.0");
