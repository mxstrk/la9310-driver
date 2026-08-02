// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM

// rfnm_agc: continuous in-kernel RX LEVELING for RFNM daughterboards.
//
// Measures the peak from the live ADC ring in the iqflood DDR region (same reader pattern as
// rfnm_qec) and actuates ONLY through the generic dgb layer: gain steps ride the NORMAL
// rx_ch_set apply path via rfnm_dgb_rx_set_gain() (the dgb drivers keep that path
// delta-optimized, so a gain-only restep touches just the gain stages). There is
// deliberately zero daughterboard-specific code here: any dgb driver with a delta-clean
// rx_ch_set gets AGC for free - rfnm_lime today, yucca etc. later.
//
// One loop, gated per channel on rx_ch->agc == RFNM_AGC_DEFAULT: classic two-rate step AGC
// on the user gain scale. Attack immediately on peak over threshold, decay slowly after a
// hold time below the low threshold. The applied gain is written back into the channel
// structs by the dgb driver, so GET_RX_CH_LIST reports it live.
//
// The analog DC null this module used to run was MOVED to rfnm_qec (design rule:
// DC correction does not belong in the AGC) - a one-shot measured DCOFF trim
// per RX apply plus a drift alarm, replacing the always-on servo whose decode breakage
// left the DC un-nulled for every local session while the PGA redistribution amplified it
// into the ADC rail. The rfnm_lime DC guard now caps the PGA whenever no fresh trim exists.
//
// Every gain step bumps rfnm_rx_gain_epoch so rfnm_qec restarts its settle window (an LNA step
// moves the IQ imbalance mid-estimate).

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/rfnm-shared.h>
#include <linux/rfnm-api.h>
#include <linux/rfnm-vspa.h>

#define AGC_CPU			3	// same reasoning as rfnm_qec: ride CPU 3 in the USB thread's idle time
#define AGC_ADC_SLOT_CNT	4096
#define AGC_SLOT_STRIDE		4096
#define AGC_PAYLOAD_OFF		256
#define AGC_WRITER_GUARD	4
#define AGC_MAX_SLOT_SAMPLES	960	// bufdesc payload capacity, samples

extern struct rfnm_dgb *rfnm_dgb_get(int dgb_id);
extern int rfnm_dgb_rx_set_gain(int dgb_id, int ch_id, int gain_db);
extern uint32_t rfnm_local_rx_fmt;
extern int rfnm_stream_is_native_rx(void);
extern atomic_t rfnm_rx_gain_epoch;

static int period_ms = 50;
module_param(period_ms, int, 0644);
MODULE_PARM_DESC(period_ms, "measurement wake period");
static int slots_per_wake = 16;
module_param(slots_per_wake, int, 0644);
MODULE_PARM_DESC(slots_per_wake, "max ADC ring slots sampled per wake");
static int settle_ms = 50;
module_param(settle_ms, int, 0644);
MODULE_PARM_DESC(settle_ms, "blanking after our own gain step");
static int stream_settle_ms = 300;
module_param(stream_settle_ms, int, 0644);
MODULE_PARM_DESC(stream_settle_ms, "blanking after a stream (re)start");
static int att_counts = 1450;
module_param(att_counts, int, 0644);
MODULE_PARM_DESC(att_counts, "attack threshold, peak 12-bit counts (1450 = -3 dBFS)");
static int dec_counts = 260;
module_param(dec_counts, int, 0644);
MODULE_PARM_DESC(dec_counts, "decay threshold, peak 12-bit counts (260 = -18 dBFS)");
static int hold_ms = 500;
module_param(hold_ms, int, 0644);
MODULE_PARM_DESC(hold_ms, "time below decay threshold before stepping gain up");
static int up_db = 2;
module_param(up_db, int, 0644);
MODULE_PARM_DESC(up_db, "decay step size, dB");
static int agc_master = 1;
module_param(agc_master, int, 0644);
MODULE_PARM_DESC(agc_master, "0 = freeze the loop");
static int verbose = 1;
module_param(verbose, int, 0644);

struct agc_state {
	u8 *ring;
	volatile struct rfnm_la9310_status *status;
	struct task_struct *task;

	int dgb_id;
	int ch_id;

	u32 last_slot;
	u32 last_rx_buf_id;
	int was_idle;
	unsigned long blank_until;
	int64_t last_freq;

	// window accumulator (12-bit-count domain)
	int peak;

	unsigned long below_since;
	int below_valid;

	u32 stat_slots, stat_steps_down, stat_steps_up, stat_idle;
};

static struct agc_state *agc;

// find the RX channel this module can control: adc 0 (the only lane the ring reader parses)
static int agc_find_target(struct agc_state *st) {
	int i, q;

	for(i = 0; i < 2; i++) {
		struct rfnm_dgb *dgb_dt = rfnm_dgb_get(i);
		if(!dgb_dt) {
			continue;
		}
		for(q = 0; q < dgb_dt->rx_ch_cnt; q++) {
			if(dgb_dt->rx_ch[q] && dgb_dt->rx_ch[q]->adc_id == 0) {
				st->dgb_id = i;
				st->ch_id = q;
				return 0;
			}
		}
	}
	return -ENODEV;
}

// decode one ring slot into the window stats. Format-generic: sub sizes come from the
// descriptors; packed12 payloads carry one direct 12-bit value in bits[15:4] of every u16
// (I/Q interleaved), CS16 payloads are plain s16 pairs at 12-bit << 4 scale.
static int agc_slot_stats(struct agc_state *st, const u8 *slot) {
	const struct rfnm_bufdesc_rx_sub *sub = (const struct rfnm_bufdesc_rx_sub *)slot;
	u32 nsamp = 0;
	u32 nvals, k;
	int s, peak = st->peak;

	for(s = 0; s < RFNM_RX_BUF_OUT_SUB_CNT; s++) {
		u32 size = sub[s].size;
		if(!size) {
			break;
		}
		if(RFNM_RX_SUBDESC_ID(sub[s].adc_id) != 0 || size > 256 || (size % 64)) {
			return 0;
		}
		nsamp += size;
	}
	if(!nsamp || nsamp > AGC_MAX_SLOT_SAMPLES) {
		return 0;
	}

	if(rfnm_local_rx_fmt == RFNM_PACKET_FMT_CS16) {
		const s16 *p = (const s16 *)(slot + AGC_PAYLOAD_OFF);
		nvals = nsamp * 2;
		for(k = 0; k < nvals; k += 2) {
			int vi = p[k] >> 4;
			int vq = p[k + 1] >> 4;
			if(vi < 0) {
				vi = -vi;
			}
			if(vq < 0) {
				vq = -vq;
			}
			if(vi > peak) {
				peak = vi;
			}
			if(vq > peak) {
				peak = vq;
			}
		}
	} else {
		// packed12 is 3 bytes per TWO 12-bit values (one complex sample: I in bits
		// [23:12], Q in bits [11:0] of each little-endian 3-byte group - the same
		// layout rfnm_local_fill_cs16/unpack12to16 decode). The old u16-stride read
		// ("one value in bits[15:4] of every u16") never matched this packing: it
		// mixed adjacent samples' bits into garbage values whose peak pins at ~2047
		// - the "detector pinned at 2048, gain rails to -24" defect. Only sessions
		// with a local CS16 client (the original validation rig) took the correct
		// branch, which is why the defect looked intermittent.
		const u8 *p = (const u8 *)(slot + AGC_PAYLOAD_OFF);
		u32 groups = nsamp / 2;	// two 12-bit values per 3-byte group
		nvals = groups * 2;
		for(k = 0; k < groups; k++) {
			u32 lp = p[0] | (p[1] << 8) | ((u32)p[2] << 16);
			int vi = (s16)(((lp >> 12) & 0xFFF) << 4) >> 4;	// sign-extend 12 bits
			int vq = (s16)((lp & 0xFFF) << 4) >> 4;
			p += 3;
			if(vi < 0) {
				vi = -vi;
			}
			if(vq < 0) {
				vq = -vq;
			}
			if(vi > peak) {
				peak = vi;
			}
			if(vq > peak) {
				peak = vq;
			}
		}
	}

	st->peak = peak;
	st->stat_slots++;
	return 1;
}

static void agc_level_step(struct agc_state *st, struct rfnm_api_rx_ch *rx_ch) {
	int gain = rx_ch->gain;
	int peak = st->peak;

	if(peak >= att_counts) {
		// coarse proportional attack: 6 dB per power of two of overshoot, 12 dB when pinned at the rail
		int down = 6;
		if(peak >= 2040) {
			down = 12;
		} else {
			int o = peak / (att_counts ? att_counts : 1);
			while(o > 1) {
				down += 6;
				o >>= 1;
			}
		}
		if(!rfnm_dgb_rx_set_gain(st->dgb_id, st->ch_id, gain - down)) {
			atomic_inc(&rfnm_rx_gain_epoch);
			st->stat_steps_down++;
			st->blank_until = jiffies + msecs_to_jiffies(settle_ms);
			if(verbose) {
				printk("rfnm_agc: attack peak=%d gain %d -> %d\n", peak, gain, rx_ch->gain);
			}
		}
		st->below_valid = 0;
		return;
	}

	if(peak && peak < dec_counts) {
		if(!st->below_valid) {
			st->below_valid = 1;
			st->below_since = jiffies;
		} else if(time_after(jiffies, st->below_since + msecs_to_jiffies(hold_ms))) {
			if(gain < rx_ch->gain_range.max && !rfnm_dgb_rx_set_gain(st->dgb_id, st->ch_id, gain + up_db)) {
				atomic_inc(&rfnm_rx_gain_epoch);
				st->stat_steps_up++;
				st->blank_until = jiffies + msecs_to_jiffies(settle_ms);
				if(verbose) {
					printk("rfnm_agc: decay peak=%d gain %d -> %d\n", peak, gain, rx_ch->gain);
				}
			}
			st->below_valid = 0;
		}
	} else {
		st->below_valid = 0;
	}
}

static void agc_poll(struct agc_state *st) {
	struct rfnm_dgb *dgb_dt;
	struct rfnm_api_rx_ch *rx_ch;
	u32 cur, target, avail, k;

	// Same contract rfnm_qec learned on 2026-07-03 (repeated-open wedge): the iqflood ADC
	// ring is only in the layout this reader understands when the stream is packed12 native
	// full-rate. Under any other state - a local client flips the fmt to CS16, and every
	// DCS/decimated mode changes the payload layout - the decode is garbage: the peak pins
	// at the 12-bit rail (reproduced on a local RTSA
	// session: attack peak=2048 storm, gain railed to -24) and BOTH loops actuate on noise.
	// Freeze them outside the understood state; leveling for local sessions is client-side.
	if(rfnm_local_rx_fmt != RFNM_PACKET_FMT_PACKED12 || !rfnm_stream_is_native_rx()) {
		return;
	}

	if(agc_find_target(st)) {
		return;
	}
	dgb_dt = rfnm_dgb_get(st->dgb_id);
	rx_ch = dgb_dt->rx_ch[st->ch_id];

	// retune guard (frequency sweeps): a slow step-AGC must never react across a freq
	// change - blank the loop so per-dwell content from the previous frequency can't
	// drive gain at the new one
	if(rx_ch->freq != st->last_freq) {
		st->last_freq = rx_ch->freq;
		st->blank_until = jiffies + msecs_to_jiffies(stream_settle_ms);
		st->below_valid = 0;
	}

	cur = st->status->rx_buf_id;
	if(cur >= AGC_ADC_SLOT_CNT) {
		st->was_idle = 1;
		return;
	}
	if(cur == st->last_rx_buf_id) {
		st->stat_idle++;
		st->was_idle = 1;
		return;
	}
	st->last_rx_buf_id = cur;

	if(st->was_idle) {
		st->was_idle = 0;
		st->blank_until = jiffies + msecs_to_jiffies(stream_settle_ms);
		st->below_valid = 0;
	}
	if(time_before(jiffies, st->blank_until)) {
		st->last_slot = (cur + AGC_ADC_SLOT_CNT - AGC_WRITER_GUARD) % AGC_ADC_SLOT_CNT;
		return;
	}

	// fresh window every wake: leveling reacts to the newest slots only
	st->peak = 0;

	target = (cur + AGC_ADC_SLOT_CNT - AGC_WRITER_GUARD) % AGC_ADC_SLOT_CNT;
	avail = (target + AGC_ADC_SLOT_CNT - st->last_slot) % AGC_ADC_SLOT_CNT;
	if(avail > (u32)slots_per_wake) {
		st->last_slot = (target + AGC_ADC_SLOT_CNT - slots_per_wake) % AGC_ADC_SLOT_CNT;
		avail = slots_per_wake;
	}
	for(k = 0; k < avail; k++) {
		const u8 *slot = st->ring + ((u64)((st->last_slot + 1 + k) % AGC_ADC_SLOT_CNT) * AGC_SLOT_STRIDE);
		agc_slot_stats(st, slot);
	}
	st->last_slot = target;

	if(!st->peak) {
		return;
	}

	if(rx_ch->agc == RFNM_AGC_DEFAULT && rx_ch->enable == RFNM_CH_RF_ON) {
		agc_level_step(st, rx_ch);
	} else {
		st->below_valid = 0;
	}

	if(verbose >= 2 && (st->stat_slots % 2048) == 0) {
		printk("rfnm_agc: slots %u down %u up %u idle %u\n",
			st->stat_slots, st->stat_steps_down, st->stat_steps_up, st->stat_idle);
	}
}

static int agc_thread_fn(void *arg) {
	struct agc_state *st = arg;

	while(!kthread_should_stop()) {
		if(agc_master) {
			agc_poll(st);
		}
		msleep_interruptible(period_ms);
	}
	return 0;
}

static int __init agc_init(void) {
	struct agc_state *st;
	int ret;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if(!st) {
		return -ENOMEM;
	}

	// same mapping rationale as rfnm_qec: LA9310 inbound PCIe DMA is not cache-coherent on
	// i.MX8MP, WC gives uncached reads with no invalidation dance
	st->ring = memremap(RFNM_IQFLOOD_ADC_MEMADDR, (u64)AGC_ADC_SLOT_CNT * AGC_SLOT_STRIDE, MEMREMAP_WC);
	st->status = (volatile struct rfnm_la9310_status *)memremap(RFNM_IQFLOOD_STATUS_MEMADDR, SZ_4K, MEMREMAP_WC);
	if(!st->ring || !st->status) {
		printk("rfnm_agc: mapping failed\n");
		ret = -ENOMEM;
		goto fail;
	}

	st->last_rx_buf_id = ~0U;
	st->was_idle = 1;
	st->last_slot = st->status->rx_buf_id % AGC_ADC_SLOT_CNT;

	if(agc_find_target(st)) {
		printk("rfnm_agc: no dgb with rx fast paths yet (will keep looking)\n");
	}

	st->task = kthread_create_on_cpu(agc_thread_fn, st, AGC_CPU, "rfnm_agc/%u");
	if(IS_ERR(st->task)) {
		ret = PTR_ERR(st->task);
		goto fail;
	}
	agc = st;
	wake_up_process(st->task);
	printk("rfnm_agc: loaded (cpu %d, period %d ms, att %d dec %d counts, leveling only - DC null lives in rfnm_qec)\n",
		AGC_CPU, period_ms, att_counts, dec_counts);
	return 0;

fail:
	if(st->ring) {
		memunmap(st->ring);
	}
	if(st->status) {
		memunmap((void *)st->status);
	}
	kfree(st);
	return ret;
}

static void __exit agc_exit(void) {
	struct agc_state *st = agc;

	kthread_stop(st->task);
	// last gain stays applied - same convention as rfnm_qec
	memunmap(st->ring);
	memunmap((void *)st->status);
	kfree(st);
	agc = NULL;
	printk("rfnm_agc: unloaded (last gain left in place)\n");
}

module_init(agc_init);
module_exit(agc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RFNM continuous RX leveling AGC");
