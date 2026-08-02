#ifndef RFNM_LIME_MCU_H_
#define RFNM_LIME_MCU_H_

struct lms7002m_context;

// runs the LMS7002M on-chip MCU RX LPF calibration (TuneRxFilter port). Caller holds the
// rfnm_lime apply mutex and invalidates the fast-tune caches afterwards.
int rfnm_lime_mcu_rx_lpf_cal(struct lms7002m_context *lms, uint32_t bw_hz, uint32_t ref_hz);

#endif
