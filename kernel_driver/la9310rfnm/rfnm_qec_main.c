// rfnm_qec: continuous in-kernel RX QEC calibration for the RFNM Lime LA9310.
//
// Slowly estimates RX IQ imbalance (gain eps, phase phi, fractional IQ delay tau) and DC offset
// from the live ADC ring in the iqflood DDR region, then updates the txiqcomp_x32chf_5t corrector
// coefficients (structTXIQCompParams2 in VSPA DMEM) through the VSPA's host DMA channel 0 over
// PCIe BAR0. The VSPA reads the new coefficients on its next buffer, so updates are glitchless
// (ISM 7.1.2: the host has its own DMA command buffer; firmware uses channels 1-5/0xb-0xd only).
//
// Estimator (validated against FFT ground truth on FM-band captures, 2026-06-12):
//   a(m) = E[x(n+m) x*(n)] m=0..3,  c(m) = E[x(n) x(n+m)] m=0..2, central moments, then
//   Re c(m) = eps * Re a(m)
//   Im c(m) = phi * Re a(m) + tau * Im a'(m)      (a' = central difference over lag)
// Only Re a may multiply eps/phi (mirror-term algebra) - using complex a leaks eps into phi/tau.
//
// Maps everything it needs itself and can be loaded/unloaded at runtime. Only accumulates in
// full-rate mode (sub size 256) on adc 0 (Lime). Since the DC reform this module
// also OWNS the analog DC null (design rule: DC correction does not belong in the AGC): a
// one-shot measured DCOFF trim through the generic dgb layer on every RX apply - see the
// dc_null block below. That is the single coupling to the dgb layer.

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
#include <la9310_pci.h>	// rfnm_la9310_mmio_fenced() - the only coupling to la9310shiva: vspa_regs MMIO must stop while the LA9310 PCIe link is down (hard reprobe)
#include "rfnm_qec.h"

// la9310rfnm pins its SCHED_FIFO stream threads to CPUs 1/2/3 (RX/TX/USB) and CPU 0 takes all
// IRQs (dwc3 alone is ~130k/s while streaming) - keep it lean, and CPU 2 (TX) is reserved for
// upcoming work. This normal-prio thread rides CPU 3 in the USB thread's idle time (~10% busy
// during RX streaming) and yields to it completely under load - acceptable for a slow loop.
// A system-workqueue delayed_work starved to ~1 wake per 2 s under load; never use that here.
#define QEC_CPU			3

#define QEC_ADC_SLOT_CNT	4096
#define QEC_SLOT_STRIDE		4096
#define QEC_PAYLOAD_OFF		256
#define QEC_WRITER_GUARD	4		// stay this many slots behind rx_buf_id (in-flight VSPA DMA)

void kernel_neon_begin(void);
void kernel_neon_end(void);

#define QEC_BAR0_PHYS		0x18000000UL
#define QEC_VSPA_REGS_PHYS	(QEC_BAR0_PHYS + 0x1000000UL)
#define QEC_STAGE_PHYS		(RFNM_IQFLOOD_MEMADDR + 0x3000000UL + 0x800UL)	// fastswap staging region, own offset
#define QEC_STAGE_AXI		(0xC3000000UL + 0x800UL)			// same memory, LA9310 view
// DMEM addresses move when the fw layout changes (the linker reorders .rfnm_rx_iqcomp members) -
// always check vspa-nm after a fw build. qec_verify_layout() cross-checks them at init against the
// fw-owned circsize fields.
// DMEM addresses of the host-written coefficient structs. These MOVE whenever the
// fw DMEM layout changes - ALWAYS re-check with vspa-nm after a firmware change
// (fw ccf285b+txqec: rx 0x2600, rx_dec 0x2580; TX QEC structs at txn 0x2480, txs 0x2400).
#define QEC_PARAMS2_DMEM	0x2600		// iq_comp_params2_rx (full-rate path)
#define QEC_PARAMS2_DEC_DMEM	0x2580		// iq_comp_params2_rx_dec (decimated paths)
#define QEC_PARAMS2_TXN_DMEM	0x2480		// iq_comp_params2_txn (native TX ring window)
#define QEC_PARAMS2_TXS_DMEM	0x2400		// iq_comp_params2_txs (interp staging window)
// The circsize fields are fw-owned but RUNTIME-VARIABLE, not build constants: the fw rewrites
// inpCircBuffBase/Size per active path (iqmod_rx.c / iqmod_tx.c). Each struct's fingerprint is
// therefore SET-membership over every value it can legitimately hold - the sets are disjoint
// across the four structs, so the address check still discriminates. rx_dec holds
// 4992 = sizeof(rfnm_rx_out) from image init (every (re)boot of either eld) until a decimated
// RX session rewrites it: 832 = per-channel QEC pad (416*2), 512 = casc_deep_out, 256 = deep
// pad; identical sets in the stock and fdx lineages. The old exact-match on 832 (minted from a
// mid-session read) parked the corrector dead from module load until the first decimated RX
// session on EVERY boot, retry-flooding dmesg at 2 s.
static const u32 qec_cs_main[] = { 4608 };			// input_buffer ring, halfwords
static const u32 qec_cs_dec[] = { 4992, 832, 512, 256 };	// rfnm_rx_out init / QEC pads

#define REG_DMA_DMEM		(0x0B0 / 4)
#define REG_DMA_AXI		(0x0B4 / 4)
#define REG_DMA_BCNT		(0x0B8 / 4)
#define REG_DMA_XFRCTRL		(0x0BC / 4)
#define REG_DMA_COMPSTAT	(0x0C8 / 4)
#define REG_DMA_XFRERR		(0x0CC / 4)
#define REG_DMA_CFGERR		(0x0D0 / 4)

static int period_ms = 100;
module_param(period_ms, int, 0644);
MODULE_PARM_DESC(period_ms, "accumulation wake period");
static int slots_per_wake = 32;
module_param(slots_per_wake, int, 0644);
MODULE_PARM_DESC(slots_per_wake, "max ADC ring slots sampled per wake");
static int solve_ms = 5000;
module_param(solve_ms, int, 0644);
MODULE_PARM_DESC(solve_ms, "model update interval");
static int min_samples = 4000000;
module_param(min_samples, int, 0644);
MODULE_PARM_DESC(min_samples, "min accumulated samples per solve");
static int settle_ms = 500;
module_param(settle_ms, int, 0644);
MODULE_PARM_DESC(settle_ms, "discard window after stream restart (LO/gain settling)");
static int use_neon = 1;
module_param(use_neon, int, 0644);
MODULE_PARM_DESC(use_neon, "0 = scalar slot processing");
static int beta_q8 = 64;
module_param(beta_q8, int, 0644);
MODULE_PARM_DESC(beta_q8, "model damping per solve, Q8 (64 = 0.25)");
static int qec_enable = 1;
module_param(qec_enable, int, 0644);
MODULE_PARM_DESC(qec_enable, "0 = freeze (no estimation, no writes)");
static int verbose = 1;
module_param(verbose, int, 0644);

// ---- one-shot analog DC null (this module owns DC correction, 2026-07-21 reform) ----
// Replaces the rfnm_agc DC servo. Every RX apply (rfnm_rx_apply_epoch) re-arms the null:
// after settle it measures the lane means from the same accumulator the estimator uses
// and, outside the deadband, steps the ONE public correction - the wire rfic_dc_i/q
// logical codes (+-126) - through rfnm_dgb_rx_set_dc(), which rides the normal apply
// exactly like a client would (2026-07-21 API ruling: no side-band trim doors; how the
// board fans the correction over its silicon is its driver's business). The slope is
// LEARNED from our own steps (it varies with LO, clock plan and the board-internal gain
// split); converges in 2-6 measured steps, then goes quiet; a later solve seeing the
// mean drift out (temperature) re-arms it. The code mirror is what WE last applied - GET
// echoes it, and a client write that diverges it just costs the loop an extra iteration.
extern struct rfnm_dgb *rfnm_dgb_get(int dgb_id);
extern int rfnm_dgb_rx_set_dc(int dgb_id, int ch_id, int dc_i, int dc_q);	/* the ONE wire-field
					 * correction (rfic_dc_i/q, logical +-126) via the normal apply -
					 * how the board realizes it is its driver's business */
static int dc_null_enable = 1;
module_param(dc_null_enable, int, 0644);
MODULE_PARM_DESC(dc_null_enable, "one-shot analog DCOFF null on RX apply (0 = never touch the trim)");
static int dc_deadband = 24;
module_param(dc_deadband, int, 0644);
MODULE_PARM_DESC(dc_deadband, "null done below this many 12-bit counts (raised to half a code at the live PGA)");
static int dc_renull_counts = 80;
module_param(dc_renull_counts, int, 0644);
MODULE_PARM_DESC(dc_renull_counts, "re-arm the null when a solve sees the mean past this (temperature drift alarm)");
static int dc_null_min_samples = 100000;
module_param(dc_null_min_samples, int, 0644);
MODULE_PARM_DESC(dc_null_min_samples, "samples per null measurement (~0.5 s at the default wake cadence)");
// Stage 2: when scalar stepping ends with residual above the deadband (codes pinned,
// iterations spent, or stepping without effect - the board's internal knob fan-out can be
// cross-coupled between lanes in a clock-plan-dependent way), measure the LOGICAL 2x2
// response of the wire correction itself - probe each logical lane by QEC_PROBE codes
// from the current point, solve - chip-agnostic forever, no stored constants anywhere.
#define QEC_PROBE	16
static int dc_2x2_enable = 1;
module_param(dc_2x2_enable, int, 0644);
MODULE_PARM_DESC(dc_2x2_enable, "measured 2x2 stage when scalar stepping stalls (0 = scalar only)");

// ---- TX QEC coefficient interface (host-driven cross-board cal) -----------------
// The TX cal loop runs on a host: one board transmits, a second board measures, the host
// solves and writes coefficients HERE (q20 fixed point, floats * 2^20). Writing 1 to
// tx_write pushes {eps, phi, dc} into BOTH TX structs (native ring + interp staging)
// after the same circsize layout verification the RX path uses.
static int tx_eps_q20;
module_param(tx_eps_q20, int, 0644);
static int tx_phi_q20;
module_param(tx_phi_q20, int, 0644);
static int tx_dcre_q20;
module_param(tx_dcre_q20, int, 0644);
static int tx_dcim_q20;
module_param(tx_dcim_q20, int, 0644);
static int tx_write_status = -1;	// 0 ok, <0 errno of the last attempt
module_param(tx_write_status, int, 0444);
static const u32 qec_cs_txn[] = { 4096, 1536 };	// ddr_read_buffer: 8*256*2 at init, 3*DMA_TXR_size*2 once TX arms
static const u32 qec_cs_txs[] = { 2048 };	// 4-chunk AXIQ staging window

// dh1/dtau of the 5-tap LS fractional-delay design, Q20 (precomputed by qec_analyze2.design_taps)
static const s32 qec_d1_q20[5] = { 353371, -1057859, -16402, 1077529, -356918 };

struct qec_acc {
	u64 n;
	s64 si, sq;
	s64 pii[4], pqq[4], piq[4], pqi[4];
	u64 cnt[4];
};

struct qec_state {
	void __iomem *vspa_regs;
	void __iomem *stage;
	u8 *ring;				// memremap'd ADC ring
	volatile struct rfnm_la9310_status *status;
	struct task_struct *task;
	struct qec_acc acc;
	s16 *samp;				// decode scratch, QEC_DIRECT_VALS values
	u32 last_slot;
	u32 last_rx_buf_id;
	unsigned long last_solve;
	unsigned long settle_until;
	int was_idle;
	int corr_dead;				// corrector layout verify failed: no DMA writes, estimation
						// telemetry-only; the analog DCOFF null keeps running
	int last_gain_epoch;
	int last_apply_epoch;
	// one-shot analog null state (all codes LOGICAL: the wire rfic_dc domain, +-126)
	int null_armed, null_iters, null_frozen;
	s64 null_floor;			// dominant |mean| the last arm proved achievable: the drift
					// alarm re-arms only past floor + dc_renull_counts, else an
					// out-of-authority residual re-arms every solve forever
					// (observed as a 4 s probe churn)
	int trim_i, trim_q;		// mirror of the logical codes we last applied (GET echoes them;
					// a client write can diverge it - the measured loop re-converges)
	int dgb_id, ch_id;
	// measured-2x2 stage state (0 = off; 2/3 = waiting probe col I/Q; 4+ = solving)
	int p_stage, p_solves;
	int p_ei, p_eq;			// logical codes at stage entry (restore-on-worse target)
	int p_s1, p_s2;			// signed probe steps actually applied
	s64 p_mi, p_mq;			// means at the previous stage step (J deltas)
	s64 p_m0;			// dominant |mean| at stage entry
	s64 ji[2], jq[2];		// logical J, counts per code: [dI, dQ] for lane I / lane Q
	// model, Q20 (dc in normalized full-scale units)
	s32 m_eps, m_phi, m_tau, m_dcre, m_dcim;
	u32 stat_slots, stat_solves, stat_dma_err, stat_idle;
	u64 stat_proc_ns, stat_proc_slots;
};

static struct qec_state *qec;

// integer Q20 -> IEEE754 single bits (truncating; no kernel FP)
static u32 q20_to_f32(s32 v) {
	u32 s = 0, a, msb, e, m;
	if (v == 0) {
		return 0;
	}
	if (v < 0) {
		s = 1U << 31;
		a = -v;
	} else {
		a = v;
	}
	msb = fls(a) - 1;
	e = msb - 20 + 127;
	if (msb >= 23) {
		m = (a >> (msb - 23)) & 0x7FFFFF;
	} else {
		m = (a << (23 - msb)) & 0x7FFFFF;
	}
	return s | (e << 23) | m;
}

static int qec_dma_xfer(struct qec_state *st, u32 dmem, u32 axi, u32 bytes, u32 ctrl) {
	u32 w;

	// vspa_regs is the LA9310 VSPA block through the PCIe window - refuse while the link is down
	// (hard reprobe): a read/write then never completes and hard-hangs the SoC
	if (rfnm_la9310_mmio_fenced()) {
		st->stat_dma_err++;
		return -EBUSY;
	}
	// The VSPA registry loader shares channel 0 with us; a qec DMA racing a
	// parked-core kernel swap corrupts the image load (the fast-swap EIO class).
	// Stand off while the loader owns the engine; the boot_gen watch re-verifies after.
	{
		extern atomic_t rfnm_vspa_loader_busy;

		if (atomic_read(&rfnm_vspa_loader_busy)) {
			st->stat_dma_err++;
			return -EBUSY;
		}
	}

	iowrite32(1, st->vspa_regs + 4 * REG_DMA_COMPSTAT);		// W1C stale status
	iowrite32(1, st->vspa_regs + 4 * REG_DMA_XFRERR);
	iowrite32(1, st->vspa_regs + 4 * REG_DMA_CFGERR);
	iowrite32(dmem, st->vspa_regs + 4 * REG_DMA_DMEM);
	iowrite32(axi, st->vspa_regs + 4 * REG_DMA_AXI);
	iowrite32(bytes, st->vspa_regs + 4 * REG_DMA_BCNT);
	iowrite32(ctrl, st->vspa_regs + 4 * REG_DMA_XFRCTRL);		// channel 0

	for (w = 0; w < 1000; w++) {
		if (ioread32(st->vspa_regs + 4 * REG_DMA_COMPSTAT) & 1) {
			iowrite32(1, st->vspa_regs + 4 * REG_DMA_COMPSTAT);
			return 0;
		}
		udelay(1);
	}
	st->stat_dma_err++;
	return -ETIMEDOUT;
}

static int qec_write_struct(struct qec_state *st, u32 dmem, u32 stage_off, s32 tau_q20) {
	u32 ftaps_q20[12] = { 0 };
	u32 i;
	s32 h1_q20[5] = { 0 };
	int k;

	// h1 = (1+eps)*delta(k-2) + tau*d1 ; f4=1 (sec(phi)-1 < 1e-6 at our phi), f1=1, D=2
	for (k = 0; k < 5; k++) {
		h1_q20[k] = (s32)(((s64)tau_q20 * qec_d1_q20[k]) >> 20);
	}
	h1_q20[2] += (1 << 20) + st->m_eps;

	ftaps_q20[1] = -st->m_phi;					// f2 = -tan(phi) ~ -phi
	ftaps_q20[6] = 1 << 20;						// h2(2) = f1 = 1
	for (k = 0; k < 5; k++) {
		ftaps_q20[3 + 2 * (4 - k)] = h1_q20[k];			// h'1(k), descending k
	}

	for (i = 0; i < 12; i++) {
		iowrite32(q20_to_f32((s32)ftaps_q20[i]), st->stage + stage_off + 4 * i);
	}
	iowrite32(5, st->stage + stage_off + 4 * 12);			// IQImb_delay = 2*D+1
	iowrite32(q20_to_f32(st->m_dcre), st->stage + stage_off + 4 * 13);
	iowrite32(q20_to_f32(st->m_dcim), st->stage + stage_off + 4 * 14);
	wmb();
	return qec_dma_xfer(st, dmem, (u32)QEC_STAGE_AXI + stage_off, 60, 0x000);
}

static int qec_dma_write_params(struct qec_state *st) {
	int ret;

	ret = qec_write_struct(st, QEC_PARAMS2_DMEM, 0, st->m_tau);
	if (ret) {
		return ret;
	}
	// decimated paths: tau is per-sample, so it halves at 2x. 4x reuses the 2x taps (tau is
	// ~3e-3 samples here, the residual stays below -60 dB either way)
	return qec_write_struct(st, QEC_PARAMS2_DEC_DMEM, 0x100, st->m_tau / 2);
}

static int qec_circsize_member(u32 got, const u32 *set, int n) {
	int i;

	for (i = 0; i < n; i++) {
		if (got == set[i]) {
			return 1;
		}
	}
	return 0;
}

// the .rfnm_rx_iqcomp member order is linker-decided: confirm each DMEM address by its fw-owned
// circsize before ever writing coefficients there (set-membership - the field is runtime-variable)
static int qec_verify_layout(struct qec_state *st) {
	static const struct { u32 dmem; const u32 *cs; int cs_n; } chk[2] = {
		{ QEC_PARAMS2_DMEM, qec_cs_main, ARRAY_SIZE(qec_cs_main) },
		{ QEC_PARAMS2_DEC_DMEM, qec_cs_dec, ARRAY_SIZE(qec_cs_dec) },
	};
	int i, ret;

	for (i = 0; i < 2; i++) {
		u32 got;

		ret = qec_dma_xfer(st, chk[i].dmem, (u32)QEC_STAGE_AXI + 0x200, 68, 0x600);
		if (ret) {
			return ret;
		}
		rmb();
		got = ioread32(st->stage + 0x200 + 4 * 16);
		if (!qec_circsize_member(got, chk[i].cs, chk[i].cs_n)) {
			// ratelimited: the revive path retries this every 2 s while dead
			printk_ratelimited("rfnm_qec: layout check failed: DMEM 0x%x circsize %u not in the legit set (first %u) - fw layout changed, update QEC_PARAMS2_*_DMEM\n",
				chk[i].dmem, got, chk[i].cs[0]);
			return -EINVAL;
		}
	}
	return 0;
}

static struct qec_state *qec_st_global;

// flat TX correction: h1 = (1+eps)*delta(k-2), f2 = -phi, dc; same struct layout and
// staging DMA as the RX writer, offsets 0x300/0x340 (clear of the RX 0/0x100 and the
// verify scratch 0x200)
static int qec_tx_write_struct(struct qec_state *st, u32 dmem, u32 stage_off) {
	u32 i;

	for (i = 0; i < 12; i++) {
		iowrite32(0, st->stage + stage_off + 4 * i);
	}
	iowrite32(q20_to_f32(-tx_phi_q20), st->stage + stage_off + 4 * 1);		// f2 = -tan(phi) ~ -phi
	iowrite32(q20_to_f32(1 << 20), st->stage + stage_off + 4 * 6);			// h2(2) = f1 = 1
	iowrite32(q20_to_f32((1 << 20) + tx_eps_q20), st->stage + stage_off + 4 * 7);	// h'1(2) = f4 = 1+eps
	iowrite32(5, st->stage + stage_off + 4 * 12);					// IQImb_delay = 2*D+1, D=2
	iowrite32(q20_to_f32(tx_dcre_q20), st->stage + stage_off + 4 * 13);
	iowrite32(q20_to_f32(tx_dcim_q20), st->stage + stage_off + 4 * 14);
	wmb();
	return qec_dma_xfer(st, dmem, (u32)QEC_STAGE_AXI + stage_off, 60, 0x000);
}

static int qec_tx_verify_and_write(struct qec_state *st) {
	static const struct { u32 dmem; const u32 *cs; int cs_n; u32 off; } t[2] = {
		{ QEC_PARAMS2_TXN_DMEM, qec_cs_txn, ARRAY_SIZE(qec_cs_txn), 0x300 },
		{ QEC_PARAMS2_TXS_DMEM, qec_cs_txs, ARRAY_SIZE(qec_cs_txs), 0x340 },
	};
	int i, ret;

	for (i = 0; i < 2; i++) {
		u32 got;

		ret = qec_dma_xfer(st, t[i].dmem, (u32)QEC_STAGE_AXI + 0x200, 68, 0x600);
		if (ret) {
			return ret;
		}
		rmb();
		got = ioread32(st->stage + 0x200 + 4 * 16);
		if (!qec_circsize_member(got, t[i].cs, t[i].cs_n)) {
			printk("rfnm_qec: TX layout check failed: DMEM 0x%x circsize %u not in the legit set (first %u)\n",
				t[i].dmem, got, t[i].cs[0]);
			return -EINVAL;
		}
	}
	for (i = 0; i < 2; i++) {
		ret = qec_tx_write_struct(st, t[i].dmem, t[i].off);
		if (ret) {
			return ret;
		}
	}
	printk("rfnm_qec: TX coeffs written (eps %d phi %d dc %d/%d q20)\n",
		tx_eps_q20, tx_phi_q20, tx_dcre_q20, tx_dcim_q20);
	return 0;
}

static int tx_write_set(const char *val, const struct kernel_param *kp) {
	if (!qec_st_global) {
		tx_write_status = -ENODEV;
		return 0;
	}
	tx_write_status = qec_tx_verify_and_write(qec_st_global);
	return 0;
}
static const struct kernel_param_ops tx_write_ops = { .set = tx_write_set };
module_param_cb(tx_write, &tx_write_ops, NULL, 0200);

static void qec_slot_scalar(const u16 *p, s16 *v, struct qec_sums *out) {
	s64 i0, q0, im, qm;
	int k, m;

	memset(out, 0, sizeof(*out));
	for (k = 0; k < QEC_DIRECT_VALS; k++) {
		v[k] = (s16)(p[k] & 0xFFF0) >> 4;
	}
	for (k = 0; k < QEC_DIRECT_VALS; k += 2) {
		out->si += v[k];
		out->sq += v[k + 1];
	}
	for (m = 0; m < 4; m++) {
		for (k = 0; k < QEC_DIRECT_VALS / 2 - m; k++) {
			i0 = v[2 * k];
			q0 = v[2 * k + 1];
			im = v[2 * (k + m)];
			qm = v[2 * (k + m) + 1];
			out->pii[m] += i0 * im;
			out->pqq[m] += q0 * qm;
			out->piq[m] += i0 * qm;
			out->pqi[m] += q0 * im;
		}
	}
}

static void qec_fold_sums(struct qec_state *st, const struct qec_sums *s) {
	int m;

	st->acc.n += QEC_DIRECT_VALS / 2;
	st->acc.si += s->si;
	st->acc.sq += s->sq;
	for (m = 0; m < 4; m++) {
		st->acc.pii[m] += s->pii[m];
		st->acc.pqq[m] += s->pqq[m];
		st->acc.piq[m] += s->piq[m];
		st->acc.pqi[m] += s->pqi[m];
		st->acc.cnt[m] += QEC_DIRECT_VALS / 2 - m;
	}
	st->stat_slots++;
}

static int qec_slot_valid(const u8 *slot) {
	const struct rfnm_bufdesc_rx_sub *sub = (const struct rfnm_bufdesc_rx_sub *)slot;
	int s;
	// full-rate layout only: 3 subs of 256 samples, adc 0 (Lime single RX channel).
	// adc_id carries flag bits above the id byte since the windowed-era fw - mask with
	// RFNM_RX_SUBDESC_ID like rfnm_agc does (the raw !=0 word compare silently rejected
	// EVERY slot on fw 3bea8971, which is how this module sat dead on the board unnoticed)
	for (s = 0; s < 3; s++) {
		if (RFNM_RX_SUBDESC_ID(sub[s].adc_id) != 0 || sub[s].size != 256) {
			return 0;
		}
	}
	return 1;
}

// num/den in Q20 without overflowing the s64 shift: take what headroom num has, shift den for the rest
static s64 qec_div_q20(s64 num, s64 den) {
	int head = 62 - fls64((u64)(num < 0 ? -num : num) | 1);
	int k1 = head < 20 ? head : 20;

	den >>= 20 - k1;
	if (den == 0) {
		den = 1;
	}
	return div64_s64(num << k1, den);
}

static void qec_solve(struct qec_state *st) {
	struct qec_acc *a = &st->acc;

	if (verbose >= 2) {
		printk("rfnm_qec: raw n=%llu si=%lld sq=%lld pii0=%lld pqq0=%lld piq1=%lld pqi1=%lld cnt0=%llu\n",
			a->n, a->si, a->sq, a->pii[0], a->pqq[0], a->piq[1], a->pqi[1], a->cnt[0]);
	}
	s64 ra[4], ia[4], rc[3], ic[3], iap[3], ra2[3];
	s64 mi_q8, mq_q8;
	s64 s_rr = 0, s_rc = 0, s_rr2 = 0, s_ri = 0, s_ii = 0, s_rb = 0, s_ib = 0;
	s64 det, e_q20 = 0, p_q20 = 0, t_q20 = 0;
	int m, sh;

	mi_q8 = div64_s64(a->si << 8, (s64)a->n);
	mq_q8 = div64_s64(a->sq << 8, (s64)a->n);

	for (m = 0; m < 4; m++) {
		// central moments, Q8 per-sample (samples ~2^11, sums/cnt ~2^22, <<8 ~2^30: s64-safe)
		s64 eii = div64_s64(a->pii[m] << 8, (s64)a->cnt[m]) - ((mi_q8 * mi_q8) >> 8);
		s64 eqq = div64_s64(a->pqq[m] << 8, (s64)a->cnt[m]) - ((mq_q8 * mq_q8) >> 8);
		s64 eiq = div64_s64(a->piq[m] << 8, (s64)a->cnt[m]) - ((mi_q8 * mq_q8) >> 8);
		s64 eqi = div64_s64(a->pqi[m] << 8, (s64)a->cnt[m]) - ((mi_q8 * mq_q8) >> 8);
		ra[m] = eii + eqq;
		ia[m] = eiq - eqi;
		if (m < 3) {
			rc[m] = eii - eqq;
			ic[m] = eiq + eqi;
		}
	}
	iap[0] = ia[1];
	iap[1] = ia[2] >> 1;
	iap[2] = (ia[3] - ia[1]) >> 1;
	for (m = 0; m < 3; m++) {
		ra2[m] = ra[m];
	}

	// eps from ra/rc normalized to <2^18: 3-term dot products stay <2^37.6, division headroom handled below.
	// phi/tau Cramer from ra/iap/ic normalized to <2^14: det and numerators are 4-term degree-4 -> <2^60.
	// (the previous <2^24 normalization let s_rc<<20 and det overflow s64 right at FM-band magnitudes)
	for (sh = 0; sh < 40; sh++) {
		s64 mx = 0;
		for (m = 0; m < 3; m++) {
			mx = max3(mx, abs(ra[m]), abs(rc[m]));
		}
		if (mx < (1LL << 18)) {
			break;
		}
		for (m = 0; m < 3; m++) {
			ra[m] >>= 1;
			rc[m] >>= 1;
		}
	}
	for (sh = 0; sh < 40; sh++) {
		s64 mx = 0;
		for (m = 0; m < 3; m++) {
			mx = max3(mx, abs(ra2[m]), max(abs(iap[m]), abs(ic[m])));
		}
		if (mx < (1LL << 14)) {
			break;
		}
		for (m = 0; m < 3; m++) {
			ra2[m] >>= 1;
			iap[m] >>= 1;
			ic[m] >>= 1;
		}
	}

	for (m = 0; m < 3; m++) {
		s_rr += ra[m] * ra[m];
		s_rc += ra[m] * rc[m];
		s_rr2 += ra2[m] * ra2[m];
		s_ri += ra2[m] * iap[m];
		s_ii += iap[m] * iap[m];
		s_rb += ra2[m] * ic[m];
		s_ib += iap[m] * ic[m];
	}
	if (s_rr == 0) {
		goto out;
	}
	e_q20 = qec_div_q20(s_rc, s_rr);

	det = s_rr2 * s_ii - s_ri * s_ri;
	if (det > 0) {
		p_q20 = qec_div_q20(s_rb * s_ii - s_ib * s_ri, det);
		t_q20 = qec_div_q20(s_ib * s_rr2 - s_rb * s_ri, det);
	}

	// damped model update; dc measured in 12-bit counts, normalized full scale = 2048
	st->m_eps += (s32)((e_q20 * beta_q8) >> 8);
	st->m_phi += (s32)((p_q20 * beta_q8) >> 8);
	st->m_tau += (s32)((t_q20 * beta_q8) >> 8);
	st->m_dcre -= (s32)((div64_s64(a->si << 20, (s64)a->n * 2048) * beta_q8) >> 8);
	st->m_dcim -= (s32)((div64_s64(a->sq << 20, (s64)a->n * 2048) * beta_q8) >> 8);
	st->stat_solves++;

	// analog drift alarm: the null went quiet but the mean walked out (temperature, slow
	// LO pulling) - re-arm the one-shot instead of letting the digital corrector absorb an
	// offset the ADC is physically exposed to
	if(dc_null_enable && !st->null_armed && !st->null_frozen) {
		s64 mi_c = mi_q8 >> 8, mq_c = mq_q8 >> 8;
		s64 dom = mi_c < 0 ? -mi_c : mi_c;

		if((mq_c < 0 ? -mq_c : mq_c) > dom) {
			dom = mq_c < 0 ? -mq_c : mq_c;
		}
		if(dom > st->null_floor + dc_renull_counts) {
			st->null_armed = 1;
			st->null_iters = 0;
			if(verbose) {
				printk("rfnm_qec: dc drift re-null, mean=(%lld,%lld) counts (floor %lld)\n", mi_c, mq_c, st->null_floor);
			}
		}
	}

	if (verbose) {
		printk("rfnm_qec: n=%llu est(uQ20) eps=%lld phi=%lld tau=%lld | model eps=%d phi=%d tau=%d dc=(%d,%d) | slots %u solves %u dmaerr %u idle %u | %llu ns/slot (%s)\n",
			a->n, e_q20, p_q20, t_q20, st->m_eps, st->m_phi, st->m_tau, st->m_dcre, st->m_dcim,
			st->stat_slots, st->stat_solves, st->stat_dma_err, st->stat_idle,
			st->stat_proc_slots ? st->stat_proc_ns / st->stat_proc_slots : 0, use_neon ? "neon" : "scalar");
		st->stat_proc_ns = 0;
		st->stat_proc_slots = 0;
	}
out:
	memset(a, 0, sizeof(*a));
}

// find the RX channel the null can act on: adc 0 (the only lane the ring reader parses)
static int qec_find_target(struct qec_state *st) {
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

// one measured step of the DCOFF null, on the same accumulated lane means the estimator
// uses. Resets the accumulation whether or not it wrote: a step invalidates the window,
// and the estimator must never solve across the pre-null DC either.
// every null write rides the NORMAL apply (wire-api ruling) - which bumps
// rfnm_rx_apply_epoch, which re-arms the null. Absorb our own bump after every write,
// or each step triggers a fresh arm forever: iterations never accumulate, the walk runs
// to the clamp and stands there (an observed 2450-count runaway). A client
// apply racing into the same window is absorbed too - acceptable: the drift alarm
// catches any residual a missed re-arm would have.
static int qec_set_dc(struct qec_state *st, int dc_i, int dc_q) {
	extern atomic_t rfnm_rx_apply_epoch;
	int ret = rfnm_dgb_rx_set_dc(st->dgb_id, st->ch_id, dc_i, dc_q);

	st->last_apply_epoch = atomic_read(&rfnm_rx_apply_epoch);
	return ret;
}

// enter the measured-2x2 stage: the scalar loop stalled with residual above the deadband
// (codes pinned, iterations spent, or steps not moving the mean - the board's internal
// fan-out of the wire correction can be cross-coupled in a clock-plan-dependent way).
// Baseline = the CURRENT window's means at the CURRENT codes (no zeroing: the all-zero
// pair is skip-write by wire-api contract, and a known reference point is all J needs).
// The first probe fires here; its column is measured next window.
static int qec_dc_probe_enter(struct qec_state *st, s64 mi, s64 mq, s64 dom) {
	int s1;

	if(!dc_2x2_enable || st->null_frozen) {
		return 0;
	}
	st->p_ei = st->trim_i;
	st->p_eq = st->trim_q;
	st->p_m0 = dom;
	st->p_mi = mi;
	st->p_mq = mq;
	s1 = st->trim_i + QEC_PROBE <= 126 ? QEC_PROBE : -QEC_PROBE;
	if(qec_set_dc(st, st->trim_i + s1, st->trim_q)) {
		st->null_frozen = 1;
		return 0;
	}
	st->trim_i += s1;
	st->p_s1 = s1;
	st->p_stage = 2;
	st->p_solves = 0;
	return 1;
}

// one window of the 2x2 stage: stages 2/3 finish the live identification of the LOGICAL
// J (counts per code, signed probes folded in); stages 4+ Newton-solve with it.
static void qec_dc_probe_step(struct qec_state *st, s64 mi, s64 mq, int dead) {
	s64 dom = mi < 0 ? -mi : mi;
	s64 a, b, c, d, det, x, y;
	int ni, nq, s2;

	if((mq < 0 ? -mq : mq) > dom) {
		dom = mq < 0 ? -mq : mq;
	}

	if(st->p_stage == 2) {				// lane-I probe column measured
		st->ji[0] = div64_s64(mi - st->p_mi, st->p_s1);
		st->ji[1] = div64_s64(mq - st->p_mq, st->p_s1);
		st->p_mi = mi;
		st->p_mq = mq;
		s2 = st->trim_q + QEC_PROBE <= 126 ? QEC_PROBE : -QEC_PROBE;
		if(qec_set_dc(st, st->trim_i, st->trim_q + s2)) {
			st->null_frozen = 1;
			goto disarm;
		}
		st->trim_q += s2;
		st->p_s2 = s2;
		st->p_stage = 3;
		return;
	}
	if(st->p_stage == 3) {				// lane-Q probe column measured: J complete
		st->jq[0] = div64_s64(mi - st->p_mi, st->p_s2);
		st->jq[1] = div64_s64(mq - st->p_mq, st->p_s2);
		st->p_stage = 4;
		if(verbose) {
			printk("rfnm_qec: dc 2x2 J = [%lld %lld; %lld %lld] counts/code\n",
				st->ji[0], st->jq[0], st->ji[1], st->jq[1]);
		}
	}

	if(dom < dead) {
		if(verbose) {
			printk("rfnm_qec: dc 2x2 converged, mean=(%lld,%lld) codes=(%d,%d)\n", mi, mq, st->trim_i, st->trim_q);
		}
		st->p_stage = 0;
		st->null_armed = 0;
		st->null_floor = dom;
		return;
	}
	a = st->ji[0];
	c = st->ji[1];
	b = st->jq[0];
	d = st->jq[1];
	det = a * d - b * c;
	if(st->p_solves >= 3) {
		goto giveup;
	}
	if((det < 0 ? -det : det) < 4) {
		// rank-deficient session (the lottery often leaves ONE useful logical axis:
		// measured J=[0 0; -18 0]). Least-squares on the healthier column nulls
		// whatever that axis can reach instead of abandoning a correctable residual.
		s64 ni2 = a * a + c * c;
		s64 nq2 = b * b + d * d;

		if(ni2 >= nq2 && ni2 >= 4) {
			x = div64_s64(-(mi * a + mq * c), ni2);
			y = 0;
		} else if(nq2 >= 4) {
			x = 0;
			y = div64_s64(-(mi * b + mq * d), nq2);
		} else {
			goto giveup;	// no axis reaches anything this session
		}
	} else {
		x = div64_s64(-mi * d + mq * b, det);
		y = div64_s64(-mq * a + mi * c, det);
	}
	ni = st->trim_i + (int)x;
	nq = st->trim_q + (int)y;
	ni = ni > 126 ? 126 : (ni < -126 ? -126 : ni);
	nq = nq > 126 ? 126 : (nq < -126 ? -126 : nq);
	if(!ni && !nq) {
		ni = 1;				// the all-zero pair is skip-write by contract
	}
	if(ni == st->trim_i && nq == st->trim_q) {
		goto giveup;			// clamped: logical authority spent
	}
	if(qec_set_dc(st, ni, nq)) {
		st->null_frozen = 1;
		goto disarm;
	}
	st->trim_i = ni;
	st->trim_q = nq;
	st->p_solves++;
	if(verbose) {
		printk("rfnm_qec: dc 2x2 solve %d mean=(%lld,%lld) codes=(%d,%d)\n", st->p_solves, mi, mq, ni, nq);
	}
	return;

giveup:
	if((dom > st->p_m0 || !st->p_solves) && (st->trim_i != st->p_ei || st->trim_q != st->p_eq)) {
		// worse than entry, or nothing ever solved (degenerate J): put the entry codes
		// back - the probe offsets are transient by contract, never standing state
		// (observed creep 63->111 by +16/arm on hardware, one give-up at a time)
		int ri = st->p_ei, rq = st->p_eq;

		if(!ri && !rq) {
			ri = 1;			// skip-write pair: 1 code (~9 counts) off true zero
		}
		if(!qec_set_dc(st, ri, rq)) {
			st->trim_i = ri;
			st->trim_q = rq;
			st->null_floor = st->p_m0;
		} else {
			st->null_frozen = 1;
			st->null_floor = dom;
		}
	} else {
		st->null_floor = dom;
	}
	if(verbose) {
		printk("rfnm_qec: dc 2x2 stopped, mean=(%lld,%lld) codes=(%d,%d) solves=%d\n",
			mi, mq, st->trim_i, st->trim_q, st->p_solves);
	}
disarm:
	st->p_stage = 0;
	st->null_armed = 0;
}

static void qec_dc_null_step(struct qec_state *st) {
	s64 mi = div64_s64(st->acc.si, (s64)st->acc.n);
	s64 mq = div64_s64(st->acc.sq, (s64)st->acc.n);
	s64 dom;
	int dead = dc_deadband;

	dom = mi < 0 ? -mi : mi;
	if((mq < 0 ? -mq : mq) > dom) {
		dom = mq < 0 ? -mq : mq;
	}

	if(st->p_stage) {
		qec_dc_probe_step(st, mi, mq, dead);
		goto out;
	}

	if(mi < dead && mi > -dead && mq < dead && mq > -dead) {
		st->null_armed = 0;	// clean (or the standing correction already covers this LO)
		st->null_floor = dom;
		goto out;
	}
	if(qec_find_target(st)) {
		goto out;	// no dgb yet: stay armed, retry on the next window
	}

	// the measured 2x2 is the ONLY stage: the per-session
	// bring-up lottery means NO region of the code space may assume an aligned per-lane
	// response - the scalar optimization anti-corrected a crossed primary region straight
	// into the ADC rail (2047-count, measured) on its last outing. Probes are +-QEC_PROBE
	// bounded, every solve is computed from the live measured J, and give-ups restore the
	// entry codes: nothing here can walk blind.
	if(qec_dc_probe_enter(st, mi, mq, dom)) {
		goto out;
	}
	st->null_armed = 0;
	st->null_floor = dom;
out:
	memset(&st->acc, 0, sizeof(st->acc));
	st->settle_until = jiffies + msecs_to_jiffies(50);
}

static void qec_poll(struct qec_state *st) {
	struct qec_sums sums;
	u32 cur, target, avail, n;
	ktime_t t0;
	int neon = use_neon;
	{
		/* QEC may only sample when the ring is in the exact state it understands:
		 * packed12 fmt AND the native full-rate stream (no DCS mode, no decimation).
		 * The fmt check alone was not enough: a killed local client leaves the VSPA
		 * streaming its DCS/decim mode, QEC misparsed that ring and wrote runaway
		 * corrections into the running stream (the 2026-07-03 repeated-open wedge). */
		extern uint32_t rfnm_local_rx_fmt;
		extern int rfnm_stream_is_native_rx(void);
		if (rfnm_local_rx_fmt != 0 /* RFNM_PACKET_FMT_PACKED12 */ || !rfnm_stream_is_native_rx()) {
			return;
		}
	}

	{
		// an rfnm_agc gain step moves the IQ imbalance mid-estimate: drop the in-flight
		// accumulation and re-run the settle window, keeping the converged model
		extern atomic_t rfnm_rx_gain_epoch;
		int ge = atomic_read(&rfnm_rx_gain_epoch);

		if (ge != st->last_gain_epoch) {
			st->last_gain_epoch = ge;
			st->settle_until = jiffies + msecs_to_jiffies(settle_ms);
			memset(&st->acc, 0, sizeof(st->acc));
			st->p_stage = 0;	// a gain step moves the board's internal split: J is stale
			st->null_floor = 0;
		}
	}

	{
		// every RX apply re-arms the one-shot null and restarts settle: retunes MOVE the
		// analog DC (P0 map 2026-07-21: 110..640 counts across 1.5-3.5 GHz at PGA 0), and
		// the first post-arm measurement inside the deadband simply disarms it silently
		extern atomic_t rfnm_rx_apply_epoch;
		int ae = atomic_read(&rfnm_rx_apply_epoch);

		if (ae != st->last_apply_epoch) {
			st->last_apply_epoch = ae;
			st->settle_until = jiffies + msecs_to_jiffies(settle_ms);
			memset(&st->acc, 0, sizeof(st->acc));
			st->null_armed = 1;
			st->null_iters = 0;
			st->p_stage = 0;	// retunes move the plant: re-probe if needed
			st->null_floor = 0;
		}
	}


	cur = st->status->rx_buf_id;
	if (verbose >= 2) {
		static u32 dbg_cnt;

		if (++dbg_cnt % 20 == 0) {
			printk("rfnm_qec: poll cur=%u last=%u idle=%d settle=%d n=%llu slots=%u\n", cur, st->last_rx_buf_id,
				st->was_idle, (int)time_before(jiffies, st->settle_until), st->acc.n, st->stat_slots);
			if (verbose >= 3 && cur < QEC_ADC_SLOT_CNT) {
				const u32 *d = (const u32 *)(st->ring + (u64)((cur + QEC_ADC_SLOT_CNT - QEC_WRITER_GUARD) % QEC_ADC_SLOT_CNT) * QEC_SLOT_STRIDE);
				printk("rfnm_qec: slotdesc %08x %08x %08x %08x | %08x %08x %08x %08x | %08x %08x %08x %08x\n",
					d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10], d[11]);
			}
		}
	}
	if (cur >= QEC_ADC_SLOT_CNT) {
		st->was_idle = 1;				// status not sane (VSPA down / rebooting)
		return;
	}
	if (cur == st->last_rx_buf_id) {
		st->stat_idle++;				// stream idle: hold model, accumulate nothing
		st->was_idle = 1;
		return;
	}
	st->last_rx_buf_id = cur;

	// stream (re)start: skip the LO/gain settling window; accumulated stats from before the idle
	// gap stay (they were taken while streaming and the model is a slow hardware property)
	// re-assert the model immediately on stream restart: a VSPA reboot (fw reload or DCS-retune
	// hard reset) brings DMEM params back to fw identity, and short captures start consuming data
	// well before the settle window ends - the write is cheap and safe at any time
	if (st->was_idle) {
		st->was_idle = 0;
		st->settle_until = jiffies + msecs_to_jiffies(settle_ms);
		if (!st->corr_dead) {
			qec_dma_write_params(st);
		}
	}
	if (time_before(jiffies, st->settle_until)) {
		st->last_slot = (cur + QEC_ADC_SLOT_CNT - QEC_WRITER_GUARD) % QEC_ADC_SLOT_CNT;
		return;
	}

	target = (cur + QEC_ADC_SLOT_CNT - QEC_WRITER_GUARD) % QEC_ADC_SLOT_CNT;
	avail = (target + QEC_ADC_SLOT_CNT - st->last_slot) % QEC_ADC_SLOT_CNT;
	if (avail > (u32)slots_per_wake) {
		st->last_slot = (target + QEC_ADC_SLOT_CNT - slots_per_wake) % QEC_ADC_SLOT_CNT;
		avail = slots_per_wake;
	}
	t0 = ktime_get();
	if (neon) {
		kernel_neon_begin();
	}
	for (n = 0; n < avail; n++) {
		const u8 *slot = st->ring + ((u64)((st->last_slot + 1 + n) % QEC_ADC_SLOT_CNT) * QEC_SLOT_STRIDE);
		if (qec_slot_valid(slot)) {
			if (neon) {
				rfnm_qec_slot_neon((const u16 *)(slot + QEC_PAYLOAD_OFF), st->samp, &sums);
			} else {
				qec_slot_scalar((const u16 *)(slot + QEC_PAYLOAD_OFF), st->samp, &sums);
			}
			qec_fold_sums(st, &sums);
		}
	}
	if (neon) {
		kernel_neon_end();
	}
	st->stat_proc_ns += ktime_to_ns(ktime_sub(ktime_get(), t0));
	st->stat_proc_slots += avail;
	st->last_slot = target;

	// the null runs first and owns the accumulator until it disarms: the estimator must
	// never solve across the pre-null DC or a trim step
	if (dc_null_enable && st->null_armed && !st->null_frozen) {
		if (st->acc.n >= (u64)dc_null_min_samples) {
			qec_dc_null_step(st);
		}
		return;
	}

	if (time_after(jiffies, st->last_solve + msecs_to_jiffies(solve_ms)) && st->acc.n >= (u64)min_samples) {
		qec_solve(st);
		if (!st->corr_dead) {
			qec_dma_write_params(st);
		}
		st->last_solve = jiffies;
	}
}

static int qec_thread_fn(void *arg) {
	struct qec_state *st = arg;
	unsigned long next_revive_try = jiffies;
	extern atomic_t rfnm_vspa_boot_gen;
	int last_boot_gen = atomic_read(&rfnm_vspa_boot_gen);

	while (!kthread_should_stop()) {
		// A VSPA kernel (re)boot - registry fast swap or hard reset - means
		// the corrector layout/trims belong to a dead image; drop to corr_dead and
		// let the verify-retry below re-establish against the new one.
		if (atomic_read(&rfnm_vspa_boot_gen) != last_boot_gen) {
			last_boot_gen = atomic_read(&rfnm_vspa_boot_gen);
			if (!st->corr_dead) {
				st->corr_dead = 1;
				printk("rfnm_qec: vspa kernel (re)booted (gen %d) - corrector re-verifying\n", last_boot_gen);
			}
		}
		// Probe-once death demoted to a retry. The insmod-time verify races the
		// plan-dependent fw geometry (parked boot image, or a different decim depth),
		// so a dead corrector now self-heals as soon as the live layout matches (e.g.
		// the first decim-1 session). Telemetry-only until then; the analog DC null
		// never depended on this. Retires the rfnm_prime boot crutch in load_drivers.
		if (st->corr_dead && time_after_eq(jiffies, next_revive_try)) {
			next_revive_try = jiffies + msecs_to_jiffies(2000);
			if (!qec_verify_layout(st) && !qec_dma_write_params(st)) {
				st->corr_dead = 0;
				printk("rfnm_qec: corrector REVIVED (layout verify passed on retry)\n");
			}
		}
		if (qec_enable) {
			qec_poll(st);
		}
		msleep_interruptible(period_ms);
	}
	return 0;
}

static int __init qec_init(void) {
	struct qec_state *st;
	int ret;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st) {
		return -ENOMEM;
	}
	st->samp = kmalloc(QEC_DIRECT_VALS * sizeof(s16), GFP_KERNEL);
	if (!st->samp) {
		kfree(st);
		return -ENOMEM;
	}

	st->vspa_regs = ioremap(QEC_VSPA_REGS_PHYS, SZ_4K);
	st->stage = ioremap(QEC_STAGE_PHYS, SZ_4K);
	qec_st_global = st;
	// LA9310 PCIe inbound DMA is not cache-coherent on i.MX8MP: cached (WB) reads of the ring mix
	// stale and fresh lines and skew the statistics. WC = uncached reads, no invalidation dance.
	st->ring = memremap(RFNM_IQFLOOD_ADC_MEMADDR, (u64)QEC_ADC_SLOT_CNT * QEC_SLOT_STRIDE, MEMREMAP_WC);
	st->status = (volatile struct rfnm_la9310_status *)memremap(RFNM_IQFLOOD_STATUS_MEMADDR, SZ_4K, MEMREMAP_WC);
	if (!st->vspa_regs || !st->stage || !st->ring || !st->status) {
		printk("rfnm_qec: mapping failed\n");
		ret = -ENOMEM;
		goto fail;
	}

	qec = st;
	st->last_solve = jiffies;
	st->last_rx_buf_id = ~0U;
	st->was_idle = 1;
	st->last_slot = st->status->rx_buf_id % QEC_ADC_SLOT_CNT;

	// A corrector-layout mismatch (fw DMEM moved or a ring resized) must NOT kill the
	// module: the analog DCOFF null is SPI-only and load-bearing (the DC guard trusts it
	// to exist). Degrade to corrector-dead - estimation becomes telemetry, no DMA writes -
	// and say so loudly. (2026-07-21: exactly this had silently kept rfnm_qec off the
	// board since the windowed-era fw resized rfnm_rx_out 4992 -> 832.)
	ret = qec_verify_layout(st);
	if (ret) {
		printk("rfnm_qec: CORRECTOR DEAD (layout verify failed) - analog DC null still active, no VSPA writes\n");
		st->corr_dead = 1;
	} else {
		// establish known corrector state: identity at D=2 (model zero)
		ret = qec_dma_write_params(st);
		if (ret) {
			printk("rfnm_qec: initial params write failed (VSPA down?)\n");
			goto fail_unmap;
		}
	}

	st->task = kthread_create_on_cpu(qec_thread_fn, st, QEC_CPU, "rfnm_qec/%u");
	if (IS_ERR(st->task)) {
		ret = PTR_ERR(st->task);
		goto fail_unmap;
	}
	wake_up_process(st->task);
	printk("rfnm_qec: loaded (params2 @0x%x, cpu %d, period %d ms, solve %d ms, beta %d/256, %s)\n",
		QEC_PARAMS2_DMEM, QEC_CPU, period_ms, solve_ms, beta_q8, use_neon ? "neon" : "scalar");
	return 0;

fail_unmap:
fail:
	if (st->vspa_regs) {
		iounmap(st->vspa_regs);
	}
	if (st->stage) {
		iounmap(st->stage);
	}
	if (st->ring) {
		memunmap(st->ring);
	}
	if (st->status) {
		memunmap((void *)st->status);
	}
	kfree(st->samp);
	kfree(st);
	qec = NULL;
	return ret;
}

static void __exit qec_exit(void) {
	struct qec_state *st = qec;

	kthread_stop(st->task);
	// last written correction stays active in the VSPA - intentional
	iounmap(st->vspa_regs);
	iounmap(st->stage);
	memunmap(st->ring);
	memunmap((void *)st->status);
	kfree(st->samp);
	kfree(st);
	qec = NULL;
	printk("rfnm_qec: unloaded (correction left in place)\n");
}

module_init(qec_init);
module_exit(qec_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("RFNM Lime continuous RX QEC calibration");
