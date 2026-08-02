// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM


#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/rfnm-shared.h>

#include "rfnm_fe_yucca0.h"
#include "rfnm_fe_generic.h"


void yucca0_disable_all_pa(struct rfnm_dgb * dgb_dt) {
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_PA_EN, 0);	
}

void yucca0_disable_all_lna(struct rfnm_dgb * dgb_dt) {
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_A_LNA_BYP, 1);	
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_B_LNA_BYP, 1);		
}

void yucca0_tx_on(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch *tx_ch, int pa_en) {
	pr_debug("%s\n", __func__);

	if (tx_ch->path == RFNM_PATH_SMA_A) {
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_T, 1);
	} else {
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_T, 0);
	}

	if (pa_en) {
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_A, 1);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_B, 0);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_TX_TLI, 0);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_TX_TLO, 1);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_MT_TRX, 0);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_PA_EN, 1);
	} else {
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_PA_EN, 0);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_A, 1);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_B, 0);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_TX_TLI, 1);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_TX_TLO, 0);
		rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_MT_TRX, 0);
	}
}

void yucca0_tx_off(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch *tx_ch) {
	pr_debug("%s\n", __func__);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_PA_EN, 0);		
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_A, 0);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_B, 1);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_MT_TRX, 1);
}

void yucca0_rx_ant_a_on(struct rfnm_dgb *dgb_dt, int lna_en) {
	pr_debug("%s\n", __func__);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_A, 0);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_A_LNA_ENB, 0);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_A_LNA_BYP, lna_en ? 0 : 1);		
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_MT_TRX, 1);
}

void yucca0_rx_ant_b_on(struct rfnm_dgb *dgb_dt, int lna_en) {
	pr_debug("%s\n", __func__);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_B, 1);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_B_LNA_BYP, lna_en ? 0 : 1);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_B_LNA_ENB, 0);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_MT_TRX, 1);
}

void yucca0_rx_ant_a_off(struct rfnm_dgb *dgb_dt) {
	pr_debug("%s\n", __func__);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_MT_TRX, 0);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_A_LNA_ENB, 1);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_A, 1);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_A_LNA_BYP, 0);
}

void yucca0_rx_ant_b_off(struct rfnm_dgb *dgb_dt) {
	pr_debug("%s\n", __func__);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_FF_ANT_B, 0);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_B_LNA_ENB, 1);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_ANT_B_LNA_BYP, 0);
	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_MT_TRX, 0);
}
