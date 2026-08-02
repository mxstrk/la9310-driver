#ifndef RFNM_QEC_H_
#define RFNM_QEC_H_

// plain C types only: rfnm_qec_neon.c includes <arm_neon.h> (compiler stdint), which conflicts
// with <linux/types.h> int64_t/uint64_t - so this header must not pull kernel type headers

#define QEC_DIRECT_VALS		1152		// comp16_12b: first 576 complex samples live in bits[15:4] of 1152 u16s

// per-slot partial sums; same semantics as the scalar path (lags 0..3, products of centered-later samples)
struct qec_sums {
	long long si, sq;
	long long pii[4], pqq[4], piq[4], pqi[4];
};

// NEON decode+accumulate of one slot payload; caller must hold kernel_neon_begin()
void rfnm_qec_slot_neon(const unsigned short *payload, short *scratch, struct qec_sums *out);

#endif
