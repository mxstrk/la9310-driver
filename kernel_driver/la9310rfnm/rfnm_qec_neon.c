// NEON path for rfnm_qec slot processing. Built without -mgeneral-regs-only (see Makefile);
// every entry point must be wrapped in kernel_neon_begin()/kernel_neon_end() by the caller.
//
// Two passes per slot:
//   1. decode: 1152 u16 -> s16 samples ((s16)(u16 & 0xFFF0) >> 4), 16-byte loads. This is also
//      where the win on the WC (uncached) ring mapping comes from - wide loads instead of u16 reads.
//   2. lag MACs from the cached scratch buffer: vld2 de-interleaves I/Q, vmull+vpadal accumulate
//      straight into s64 lanes (s32 lane accumulation would overflow at 576 samples x 2^22).

#include <arm_neon.h>
#include "rfnm_qec.h"

void rfnm_qec_slot_neon(const unsigned short *payload, short *scratch, struct qec_sums *out) {
	const unsigned short *p = payload;
	const int nsamp = QEC_DIRECT_VALS / 2;
	const uint16x8_t mask = vdupq_n_u16(0xFFF0);
	int32x4_t si32 = vdupq_n_s32(0), sq32 = vdupq_n_s32(0);
	int64x2_t aii[4], aqq[4], aiq[4], aqi[4];
	int k, m, kt;

	for (k = 0; k < QEC_DIRECT_VALS; k += 16) {
		uint16x8_t a = vandq_u16(vld1q_u16(p + k), mask);
		uint16x8_t b = vandq_u16(vld1q_u16(p + k + 8), mask);
		vst1q_s16(scratch + k, vshrq_n_s16(vreinterpretq_s16_u16(a), 4));
		vst1q_s16(scratch + k + 8, vshrq_n_s16(vreinterpretq_s16_u16(b), 4));
	}

	for (m = 0; m < 4; m++) {
		aii[m] = vdupq_n_s64(0);
		aqq[m] = vdupq_n_s64(0);
		aiq[m] = vdupq_n_s64(0);
		aqi[m] = vdupq_n_s64(0);
	}

	// blocks of 8 complex samples; k+7+3 <= 575 keeps every lag's shifted load in bounds
	for (k = 0; k + 8 + 3 <= nsamp; k += 8) {
		int16x8x2_t base = vld2q_s16(scratch + 2 * k);

		si32 = vpadalq_s16(si32, base.val[0]);
		sq32 = vpadalq_s16(sq32, base.val[1]);

		for (m = 0; m < 4; m++) {
			int16x8x2_t lag = vld2q_s16(scratch + 2 * (k + m));

			aii[m] = vpadalq_s32(aii[m], vmull_s16(vget_low_s16(base.val[0]), vget_low_s16(lag.val[0])));
			aii[m] = vpadalq_s32(aii[m], vmull_s16(vget_high_s16(base.val[0]), vget_high_s16(lag.val[0])));
			aqq[m] = vpadalq_s32(aqq[m], vmull_s16(vget_low_s16(base.val[1]), vget_low_s16(lag.val[1])));
			aqq[m] = vpadalq_s32(aqq[m], vmull_s16(vget_high_s16(base.val[1]), vget_high_s16(lag.val[1])));
			aiq[m] = vpadalq_s32(aiq[m], vmull_s16(vget_low_s16(base.val[0]), vget_low_s16(lag.val[1])));
			aiq[m] = vpadalq_s32(aiq[m], vmull_s16(vget_high_s16(base.val[0]), vget_high_s16(lag.val[1])));
			aqi[m] = vpadalq_s32(aqi[m], vmull_s16(vget_low_s16(base.val[1]), vget_low_s16(lag.val[0])));
			aqi[m] = vpadalq_s32(aqi[m], vmull_s16(vget_high_s16(base.val[1]), vget_high_s16(lag.val[0])));
		}
	}

	kt = k;							// first sample index not covered by NEON blocks

	out->si = vaddvq_s32(si32);
	out->sq = vaddvq_s32(sq32);
	for (m = 0; m < 4; m++) {
		out->pii[m] = vaddvq_s64(aii[m]);
		out->pqq[m] = vaddvq_s64(aqq[m]);
		out->piq[m] = vaddvq_s64(aiq[m]);
		out->pqi[m] = vaddvq_s64(aqi[m]);
	}

	// scalar tails: samples past the uniform block bound (means), then per-lag ends
	for (k = kt; k < nsamp; k++) {
		out->si += scratch[2 * k];
		out->sq += scratch[2 * k + 1];
	}
	for (m = 0; m < 4; m++) {
		for (k = kt; k < nsamp - m; k++) {
			long long i0 = scratch[2 * k], q0 = scratch[2 * k + 1];
			long long im = scratch[2 * (k + m)], qm = scratch[2 * (k + m) + 1];

			out->pii[m] += i0 * im;
			out->pqq[m] += q0 * qm;
			out->piq[m] += i0 * qm;
			out->pqi[m] += q0 * im;
		}
	}
}
