// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM

#ifndef RFNM_FE_YUCCA0_H_
#define RFNM_FE_YUCCA0_H_

void yucca0_disable_all_pa(struct rfnm_dgb * dgb_dt);
void yucca0_disable_all_lna(struct rfnm_dgb * dgb_dt);

void yucca0_tx_on(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch *tx_ch, int gain);
void yucca0_tx_off(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch *tx_ch);
void yucca0_rx_ant_b_on(struct rfnm_dgb *dgb_dt, int gain);
void yucca0_rx_ant_b_off(struct rfnm_dgb *dgb_dt);
void yucca0_rx_ant_a_on(struct rfnm_dgb *dgb_dt, int gain);
void yucca0_rx_ant_a_off(struct rfnm_dgb *dgb_dt);


#define YUCCA0_PA_GAIN_THRESHOLD 20
#define YUCCA0_LNA_GAIN_THRESHOLD 20


#define RFNM_YUCCA0_NUM_LATCHES_1 1
#define RFNM_YUCCA0_NUM_LATCHES_2 1
#define RFNM_YUCCA0_NUM_LATCHES_3 1
#define RFNM_YUCCA0_NUM_LATCHES_4 0
#define RFNM_YUCCA0_NUM_LATCHES_5 0
#define RFNM_YUCCA0_NUM_LATCHES_6 0

#define RFNM_YUCCA0_NUM_LATCHES ((int[]){0, RFNM_YUCCA0_NUM_LATCHES_1, RFNM_YUCCA0_NUM_LATCHES_2, RFNM_YUCCA0_NUM_LATCHES_3, RFNM_YUCCA0_NUM_LATCHES_4, RFNM_YUCCA0_NUM_LATCHES_5, RFNM_YUCCA0_NUM_LATCHES_6})

#define RFNM_LATCH1 (1 << 28)
#define RFNM_LATCH2 (2 << 28)
#define RFNM_LATCH3 (3 << 28)

#define RFNM_LATCH_SEQ1 (1 << 24)

#define RFNM_LATCH_Q0 (0 << 16)
#define RFNM_LATCH_Q1 (1 << 16)
#define RFNM_LATCH_Q2 (2 << 16)
#define RFNM_LATCH_Q3 (3 << 16)
#define RFNM_LATCH_Q4 (4 << 16)
#define RFNM_LATCH_Q5 (5 << 16)
#define RFNM_LATCH_Q6 (6 << 16)
#define RFNM_LATCH_Q7 (7 << 16)

//#define RFNM_LO_LIME0_ANT 1
//#define RFNM_LO_LIME0_FA 2
//#define RFNM_LO_LIME0_TX 3
//#define RFNM_LO_LIME0_END 0, 0, 0, 0, 0

#define RFNM_YUCCA0_TX_TLO (RFNM_LATCH1 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q5)
#define RFNM_YUCCA0_PA_EN (RFNM_LATCH1 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q6)
#define RFNM_YUCCA0_TX_TLI (RFNM_LATCH1 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q7)

#define RFNM_YUCCA0_ANT_A_LNA_ENB (RFNM_LATCH2 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q1)
#define RFNM_YUCCA0_ANT_A_LNA_BYP (RFNM_LATCH2 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q3)
#define RFNM_YUCCA0_ANT_B_LNA_BYP (RFNM_LATCH2 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q5)
#define RFNM_YUCCA0_ANT_B_LNA_ENB (RFNM_LATCH2 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q7)

#define RFNM_YUCCA0_MT_TRX (RFNM_LATCH3 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q0)
#define RFNM_YUCCA0_FF_ANT_B (RFNM_LATCH3 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q1)
#define RFNM_YUCCA0_FF_ANT_T (RFNM_LATCH3 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q2)
#define RFNM_YUCCA0_FF_ANT_A (RFNM_LATCH3 | RFNM_LATCH_SEQ1 | RFNM_LATCH_Q5)

#endif /* RFNM_FE_YUCCA0_H_ */