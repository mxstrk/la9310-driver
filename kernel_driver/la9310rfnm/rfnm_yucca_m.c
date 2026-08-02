// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM

// RFNM Yucca daughterboard: one Diora MT3812 transceiver (SPI strap) serving 2 RX + 1 TX lanes,
// n78-ish FE (per-antenna LNA w/ bypass, TX PA, FF antenna switches, MT TRX_SW on an FE latch).
//
// The MT3812 contract implemented here is the silicon-proven one, NOT first-guess datasheet
// reading - the key rules, all proven against running silicon:
//   - state machine: PLL mount completion, unmount-before-remount idempotency, retune = walk
//     to STANDBY
//   - MT3812-UG-2.3: SetTrxPll freq word = LO in 500 Hz steps, vco_sel 7 = auto, gain word =
//     RF[6:4] + BB[3:0], SetGain legal in ACTIVE, TXRX_SW 0=TX 1=RX in TDD
//
// One TRX PLL serves every lane: all enabled channels share the LO and the channel being
// applied owns it (a batch retune converges - each lane re-applies at the new frequency).
// TX is not yet hardware-proven on this board (the RX side was proven as the Blue receiver flow);
// FDD_TX and the TDD pin flip are UG-correct but flagged for hardware validation.

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <la9310_base.h>
#include <asm/cacheflush.h>

#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/list.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <linux/device.h>
#include <linux/usb/composite.h>

#include <linux/delay.h>
#include <linux/math64.h>

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include <linux/rfnm-shared.h>
#include <linux/rfnm-gpio.h>

#include "rfnm_fe_generic.h"
#include "rfnm_fe_yucca0.h"

#include <mt3812_api.h>
#include <mt3812_cmd.h>
#include <mt_fw.h>
#include <mt3812_drv_priv.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

// Tuning envelope = the advertised MT3812 range (2496-5000), silicon-proven 2026-07-12:
// with the per-LO band map below, a 22-point apply ladder locked at every LO from 2300 to
// 5200 (rule: "prove, then widen" - proven, widened). RF-quality outside
// the n78 FE band (LNA/PA response, gain cal validity in LB/HB) is characterization debt,
// not a tuning limit.
#define YUCCA_FREQ_MIN MHZ_TO_HZ(2496)
#define YUCCA_FREQ_MAX MHZ_TO_HZ(5000)

// UG 6.4.1 documents BAND_AUTO (chip picks LB/MB/HB from the last SetTrxPll LO) but this
// board's chip fw REJECTS it (SetPath rtc 5, probed 2026-07-12) - map the band from the LO
// driver-side instead. Optimization ranges per the restudy: LB ~2.5-3.0, MB 3.3-3.8
// (proven), HB ~4.2-5.0; LOs in the gaps get the nearest band and live or die by the
// silicon ladder.
static band_t yucca_band_for(u64 freq) {
	if(freq < MHZ_TO_HZ(3150)) {
		return BAND_LB;
	}
	if(freq <= MHZ_TO_HZ(4000)) {
		return BAND_MB;
	}
	return BAND_HB;
}

// RX gain = the raw 7-bit TRX_SetGain word, exposed directly (the old (rf<<4)|bb split as a
// user unit was noise). Decodes as RF[6:4] ~5 dB/step, BB[3:0] ~1.9 dB/step, NON-monotonic
// sawtooth; useful span 0..77. Yucca-FE characterization owed.
#define YUCCA_RX_GAIN_MAX 77
// TX gain word is 5 bits acting on TX RF; placeholder linear map power_dBm -> word
// ("power - (-10)", 0..31), calibration owed.
#define YUCCA_TX_POWER_MIN (-10)
#define YUCCA_TX_POWER_MAX 21

struct rfnm_yucca_priv {
	struct mt3812_dev *mt;
	// mirror of the mounted chip state; the state WALKS always follow the chip's own
	// GetSysStatus report, the mirror only decides whether an apply is structural
	uint64_t lo_hz;		// 0 = TRX PLL off
	int path;		// path_t bitmask actually mounted (PATH_NONE = unmounted)
	rxbw_t rxbw;
	txbw_t txbw;
	activation_mode_t act;
	int rx_gain_code[2];	// last gain word written per RX lane, -1 = unknown
	int tx_gain_code;
};

// everything that talks to the chip or the FE latches serializes here (chlist apply work,
// rfnm_agc gain resteps via rfnm_dgb_rx_set_gain, the mt_status debug hook)
static DEFINE_MUTEX(rfnm_yucca_apply_lock);

static struct rfnm_dgb *rfnm_yucca_dbg_dgb;	// for the mt_status module-param hook

// rxbw_t/txbw_t are total complex signal bandwidths (UG: LPF cutoff = half signal bw)
static rxbw_t yucca_rxbw_from_mhz(int mhz) {
	static const int bw[] = { 20, 25, 30, 40, 50, 60, 70, 80, 90, 100, 200, 400 };
	int i;
	if(mhz <= 0) {
		return RXBW_20M;
	}
	for(i = 0; i < ARRAY_SIZE(bw); i++) {
		if(mhz <= bw[i]) {
			return (rxbw_t)i;
		}
	}
	return RXBW_400M;
}

static txbw_t yucca_txbw_from_mhz(int mhz) {
	static const int bw[] = { 50, 100, 200, 400 };
	int i;
	if(mhz <= 0) {
		return TXBW_50M;
	}
	for(i = 0; i < ARRAY_SIZE(bw); i++) {
		if(mhz <= bw[i]) {
			return (txbw_t)i;
		}
	}
	return TXBW_400M;
}

static int yucca_rx_gain_code(int gain) {
	return clamp_t(int, gain, 0, YUCCA_RX_GAIN_MAX);
}

static int yucca_tx_gain_code(int power) {
	return clamp_t(int, power - YUCCA_TX_POWER_MIN, 0, 31);
}

static u16 yucca_mt_sys_state(struct rfnm_yucca_priv *priv, sys_state_t *state) {
	struct in_api_param in_p;
	struct out_api_param out_p;
	u16 rtc;

	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	rtc = mt3812_api_APP_GetSysStatus(priv->mt, &in_p, &out_p);
	if(!rtc) {
		*state = out_p.app_getsysstatus.sys_state;
	}
	return rtc;
}

// Mount the TRX PLL from STANDBY. A bare SetTrxPll returns rtc 0 on R2.7 but leaves SetPath
// refused - the mount is only complete after the REG_CalTrxPll lock loop + the AccPLL MMD
// program (proven on silicon). freq word = LO in 500 Hz steps, 24 bits; vco_sel 7 =
// firmware auto-select.
static u16 yucca_mt_pll_mount(struct rfnm_yucca_priv *priv, uint64_t lo_hz) {
	struct in_api_param in_p;
	struct out_api_param out_p;
	u32 freq = (u32)div_u64(lo_hz, 500);
	u16 rtc;

	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	in_p.trx_settrxpll.mode = ON;
	in_p.trx_settrxpll.vco_sel = 7;
	in_p.trx_settrxpll.freq_msw = (freq >> 16) & 0xff;
	in_p.trx_settrxpll.freq_lsw = freq & 0xffff;
	rtc = mt3812_api_TRX_SetTrxPll(priv->mt, &in_p, &out_p);
	if(rtc) {
		return rtc;
	}

	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	rtc = mt3812_api_REG_CalTrxPll(priv->mt, &in_p, &out_p);
	if(rtc) {
		return rtc;
	}
	if(out_p.reg_caltrxpll.trxpll_unlock) {
		return RTC_PLL_UNLOCKED;
	}

	rtc = mt3812_api_AccPLL(priv->mt, freq);
	if(rtc) {
		return rtc;
	}

	priv->lo_hz = lo_hz;
	return 0;
}

// Walk the TRX state machine down to STANDBY by the chip's own state report:
// ACTIVE -SetActive(OFF)-> PREPARED -SetPath(NONE)-> TRXPLLON -SetTrxPll(OFF)-> STANDBY.
// The PATH_NONE unmount must carry the LIVE band/bw read back via GetPath - a zeros-unmount
// is refused.
static u16 yucca_mt_to_standby(struct rfnm_yucca_priv *priv) {
	struct in_api_param in_p;
	struct out_api_param out_p;
	sys_state_t state;
	u16 rtc;
	int tries;

	for(tries = 0; tries < 5; tries++) {
		rtc = yucca_mt_sys_state(priv, &state);
		if(rtc) {
			return rtc;
		}

		switch(state) {
		case SS_STANDBY:
			priv->lo_hz = 0;
			priv->path = PATH_NONE;
			priv->act = ACT_OFF;
			return 0;
		case SS_ACTIVE:
			memset(&in_p, 0, sizeof(in_p));
			memset(&out_p, 0, sizeof(out_p));
			in_p.trx_setactive.mode = ACT_OFF;
			in_p.trx_setactive.dpd_rx = RX_NONE;
			rtc = mt3812_api_TRX_SetActive(priv->mt, &in_p, &out_p);
			break;
		case SS_PREPARED:
			memset(&in_p, 0, sizeof(in_p));
			memset(&out_p, 0, sizeof(out_p));
			rtc = mt3812_api_TRX_GetPath(priv->mt, &in_p, &out_p);
			if(rtc) {
				return rtc;
			}
			memset(&in_p, 0, sizeof(in_p));
			in_p.trx_setpath.path = PATH_NONE;
			in_p.trx_setpath.band = out_p.trx_getpath.band;
			in_p.trx_setpath.rssi_mode = out_p.trx_getpath.rssi_mode;
			in_p.trx_setpath.dpd_mode = out_p.trx_getpath.dpd_mode;
			in_p.trx_setpath.rxbw = out_p.trx_getpath.rxbw;
			in_p.trx_setpath.txbw = out_p.trx_getpath.txbw;
			memset(&out_p, 0, sizeof(out_p));
			rtc = mt3812_api_TRX_SetPath(priv->mt, &in_p, &out_p);
			break;
		case SS_TRXPLLON:
			memset(&in_p, 0, sizeof(in_p));
			memset(&out_p, 0, sizeof(out_p));
			in_p.trx_settrxpll.mode = OFF;
			rtc = mt3812_api_TRX_SetTrxPll(priv->mt, &in_p, &out_p);
			break;
		default:
			// BBLOOP/CALPLLON/RFLOOP are never entered by this driver; IDLE = the
			// pre-firmware Radio Controller state - nothing to walk, needs a reprobe
			pr_err("RFNM: Yucca cannot walk to STANDBY from chip state %d\n", state);
			return RTC_INTERNAL_ERROR;
		}
		if(rtc) {
			return rtc;
		}
	}
	return TRC_TIMEOUT;
}

// TRX_SetGain: manual=1, hash_mode=0, gain word applied raw. channel selects the chip's
// stored parameter set (CH_RX=1 loaded in ACTIVE, CH_TX=0 in TRANSMIT/DPD). Legal in ACTIVE
// (UG fig. 6 self-loop) - this is the AGC/gain-only fast path, so it must stay delta-skipped.
static u16 yucca_mt_set_rx_gain(struct rfnm_yucca_priv *priv, int lane, int code) {
	struct in_api_param in_p;
	struct out_api_param out_p;
	u16 rtc;

	if(priv->rx_gain_code[lane] == code) {
		return 0;
	}
	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	in_p.trx_setgain.path = lane ? PATH_RX2 : PATH_RX1;
	in_p.trx_setgain.manual = 1;
	in_p.trx_setgain.channel = CH_RX;
	in_p.trx_setgain.hash_mode = 0;
	in_p.trx_setgain.gain = code & 0x7f;
	rtc = mt3812_api_TRX_SetGain(priv->mt, &in_p, &out_p);
	if(!rtc) {
		priv->rx_gain_code[lane] = code;
	}
	return rtc;
}

static u16 yucca_mt_set_tx_gain(struct rfnm_yucca_priv *priv, int code) {
	struct in_api_param in_p;
	struct out_api_param out_p;
	u16 rtc;

	if(priv->tx_gain_code == code) {
		return 0;
	}
	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	in_p.trx_setgain.path = PATH_TX1;	// mixing RX and TX paths in one SetGain is prohibited
	in_p.trx_setgain.manual = 1;
	in_p.trx_setgain.channel = CH_TX;
	in_p.trx_setgain.hash_mode = 0;
	in_p.trx_setgain.gain = code & 0x1f;
	rtc = mt3812_api_TRX_SetGain(priv->mt, &in_p, &out_p);
	if(!rtc) {
		priv->tx_gain_code = code;
	}
	return rtc;
}

static u16 yucca_mt_activate(struct rfnm_yucca_priv *priv, activation_mode_t mode) {
	struct in_api_param in_p;
	struct out_api_param out_p;
	u16 rtc;

	if(priv->act == mode) {
		return 0;
	}
	// ACTIVE is only documented as entered from PREPARED: a mode-to-mode change steps
	// through ACT_OFF (unreachable today - act only changes with path, behind a full
	// restructure that lands here with act == ACT_OFF - but keep the helper honest)
	if(priv->act != ACT_OFF && mode != ACT_OFF) {
		memset(&in_p, 0, sizeof(in_p));
		memset(&out_p, 0, sizeof(out_p));
		in_p.trx_setactive.mode = ACT_OFF;
		in_p.trx_setactive.dpd_rx = RX_NONE;
		rtc = mt3812_api_TRX_SetActive(priv->mt, &in_p, &out_p);
		if(rtc) {
			return rtc;
		}
		priv->act = ACT_OFF;
	}
	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	in_p.trx_setactive.mode = mode;
	in_p.trx_setactive.dpd_rx = RX_NONE;
	rtc = mt3812_api_TRX_SetActive(priv->mt, &in_p, &out_p);
	if(!rtc) {
		priv->act = mode;
	}
	return rtc;
}

// The whole-chip target an apply must converge to. Built from the channel being applied
// (wanted values) plus the APPLIED state of every other channel (rx_s/tx_s), because a
// batch apply reaches us one channel at a time with the others' wanted values not yet live.
struct yucca_target {
	uint64_t lo_hz;
	int path;		// path_t bitmask union of enabled lanes
	rxbw_t rxbw;
	txbw_t txbw;
	activation_mode_t act;
};

static struct rfnm_api_rx_ch *yucca_eff_rx(struct rfnm_dgb *dgb_dt, struct rfnm_api_rx_ch *rx_new, int ch_id) {
	if(rx_new && rx_new->dgb_ch_id == ch_id) {
		return rx_new;
	}
	return dgb_dt->rx_s[ch_id];
}

static struct rfnm_api_tx_ch *yucca_eff_tx(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch *tx_new) {
	return tx_new ? tx_new : dgb_dt->tx_s[0];
}

static void yucca_compute_target(struct rfnm_dgb *dgb_dt, struct rfnm_api_rx_ch *rx_new, struct rfnm_api_tx_ch *tx_new, struct yucca_target *t) {
	struct rfnm_api_rx_ch *rx0 = yucca_eff_rx(dgb_dt, rx_new, 0);
	struct rfnm_api_rx_ch *rx1 = yucca_eff_rx(dgb_dt, rx_new, 1);
	struct rfnm_api_tx_ch *tx = yucca_eff_tx(dgb_dt, tx_new);
	int rxbw_mhz = 0;

	memset(t, 0, sizeof(*t));

	if(rx0 && rx0->enable != RFNM_CH_RF_OFF) {
		t->path |= PATH_RX1;
		t->lo_hz = rx0->freq;
		rxbw_mhz = max_t(int, rxbw_mhz, rx0->rfic_lpf_bw);
	}
	if(rx1 && rx1->enable != RFNM_CH_RF_OFF) {
		t->path |= PATH_RX2;
		t->lo_hz = rx1->freq;
		rxbw_mhz = max_t(int, rxbw_mhz, rx1->rfic_lpf_bw);
	}
	if(tx && tx->enable != RFNM_CH_RF_OFF) {
		t->path |= PATH_TX1;
		t->lo_hz = tx->freq;
		t->txbw = yucca_txbw_from_mhz(tx->rfic_lpf_bw);
	}
	// single TRX PLL: the channel being applied owns the LO
	if(rx_new && rx_new->enable != RFNM_CH_RF_OFF) {
		t->lo_hz = rx_new->freq;
	}
	if(tx_new && tx_new->enable != RFNM_CH_RF_OFF) {
		t->lo_hz = tx_new->freq;
	}
	t->rxbw = yucca_rxbw_from_mhz(rxbw_mhz);

	if((t->path & PATH_TX1) && (t->path & PATH_RX1_RX2)) {
		// TX and RX both mounted: activate under the TXRX_SW pin (FE latch MT_TRX,
		// 0 = TRANSMITTING, 1 = RECEIVING). For RF_ON_TDD pairs the M4 flips the fe_tdd
		// profiles on the tick grid; for plain RF_ON both, the last-applied direction
		// owns the pin. ACT_TDD is UG-correct but not yet hardware-proven on this board.
		t->act = ACT_TDD;
	} else if(t->path & PATH_TX1) {
		t->act = ACT_FDD_TX;
	} else if(t->path != PATH_NONE) {
		t->act = ACT_FDD_RX;
	}
}

static bool yucca_needs_restructure(struct rfnm_yucca_priv *priv, const struct yucca_target *t) {
	if(t->path != priv->path || t->act != priv->act) {
		return true;
	}
	if(t->path == PATH_NONE) {
		return false;
	}
	return t->lo_hz != priv->lo_hz || t->rxbw != priv->rxbw || t->txbw != priv->txbw;
}

// Full remount to the target: STANDBY -> PLL -> SetPath -> SetRxBw -> gains -> SetActive
// (the silicon-proven order; SetRxBw uploads the per-bw RC cal AFTER the path mounts it).
// Any structural change pays the full ladder - retunes need it anyway (UG: frequency change
// requires SetTrxPll(OFF) from STANDBY) and path/bw/act changes are session-setup events.
static u16 yucca_restructure(struct rfnm_dgb *dgb_dt, const struct yucca_target *t, struct rfnm_api_rx_ch *rx_new, struct rfnm_api_tx_ch *tx_new) {
	struct rfnm_yucca_priv *priv = dgb_dt->priv_drv;
	struct in_api_param in_p;
	struct out_api_param out_p;
	u16 rtc;

	rtc = yucca_mt_to_standby(priv);
	if(rtc) {
		return rtc;
	}
	// the walk unmounted everything; don't trust stale per-lane gain state either
	priv->rx_gain_code[0] = -1;
	priv->rx_gain_code[1] = -1;
	priv->tx_gain_code = -1;

	if(t->path == PATH_NONE) {
		return 0;
	}

	rtc = yucca_mt_pll_mount(priv, t->lo_hz);
	if(rtc) {
		return rtc;
	}

	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	in_p.trx_setpath.path = t->path;
	in_p.trx_setpath.band = yucca_band_for(t->lo_hz);
	in_p.trx_setpath.rssi_mode = RX_NONE;
	in_p.trx_setpath.dpd_mode = DPD_NONE;
	in_p.trx_setpath.rxbw = t->rxbw;
	in_p.trx_setpath.txbw = t->txbw;
	rtc = mt3812_api_TRX_SetPath(priv->mt, &in_p, &out_p);
	if(rtc) {
		return rtc;
	}
	priv->path = t->path;
	priv->rxbw = t->rxbw;
	priv->txbw = t->txbw;

	if(t->path & PATH_RX1_RX2) {
		memset(&in_p, 0, sizeof(in_p));
		memset(&out_p, 0, sizeof(out_p));
		in_p.trx_setrxbw.bw = t->rxbw;
		rtc = mt3812_api_TRX_SetRxBw(priv->mt, &in_p, &out_p);
		if(rtc) {
			return rtc;
		}
	}

	if(t->path & PATH_RX1) {
		struct rfnm_api_rx_ch *rx0 = yucca_eff_rx(dgb_dt, rx_new, 0);
		rtc = yucca_mt_set_rx_gain(priv, 0, yucca_rx_gain_code(rx0->gain));
		if(rtc) {
			return rtc;
		}
	}
	if(t->path & PATH_RX2) {
		struct rfnm_api_rx_ch *rx1 = yucca_eff_rx(dgb_dt, rx_new, 1);
		rtc = yucca_mt_set_rx_gain(priv, 1, yucca_rx_gain_code(rx1->gain));
		if(rtc) {
			return rtc;
		}
	}
	if(t->path & PATH_TX1) {
		struct rfnm_api_tx_ch *tx = yucca_eff_tx(dgb_dt, tx_new);
		rtc = yucca_mt_set_tx_gain(priv, yucca_tx_gain_code(tx->power));
		if(rtc) {
			return rtc;
		}
	}

	return yucca_mt_activate(priv, t->act);
}

// after a chip refusal mid-apply: park the FE (no PA bias, no LNA), walk the chip back to
// STANDBY and force the next apply to rebuild from scratch - never leave half-mounted state
static void yucca_fail_safe(struct rfnm_dgb *dgb_dt) {
	struct rfnm_yucca_priv *priv = dgb_dt->priv_drv;
	u16 rtc;

	yucca0_disable_all_pa(dgb_dt);
	yucca0_disable_all_lna(dgb_dt);
	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);

	rtc = yucca_mt_to_standby(priv);
	if(rtc) {
		pr_err("RFNM: Yucca fail-safe walk to STANDBY failed, rtc %d\n", rtc);
	}
	priv->lo_hz = 0;
	priv->path = PATH_NONE;
	priv->act = ACT_OFF;
	priv->rx_gain_code[0] = -1;
	priv->rx_gain_code[1] = -1;
	priv->tx_gain_code = -1;
}

void rfnm_tx_ch_get(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch * tx_ch) {
	printk("inside rfnm_tx_ch_get\n");
}
void rfnm_rx_ch_get(struct rfnm_dgb *dgb_dt, struct rfnm_api_rx_ch * rx_ch) {
	printk("inside rfnm_rx_ch_get\n");
}

int rfnm_tx_ch_set(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch * tx_ch) {
	struct rfnm_yucca_priv *priv = dgb_dt->priv_drv;
	rfnm_api_failcode ecode = RFNM_API_OK;
	struct yucca_target t;
	u16 rtc;
	int i;

	mutex_lock(&rfnm_yucca_apply_lock);

	// validate against the driver's own envelope, not the struct's freq_min/max - the
	// chlist apply memcpys the whole client struct, so those fields are client-supplied
	if(tx_ch->enable != RFNM_CH_RF_OFF && (tx_ch->freq < YUCCA_FREQ_MIN || tx_ch->freq > YUCCA_FREQ_MAX)) {
		pr_err("RFNM: Yucca TX freq %llu Hz outside the proven MB envelope\n", tx_ch->freq);
		ecode = RFNM_API_TUNE_FAIL;
		goto fail_param;
	}

	yucca_compute_target(dgb_dt, NULL, tx_ch, &t);

	if(tx_ch->enable == RFNM_CH_RF_OFF) {
		// RF_OFF must actually stop radiating: PA off + feeds parked first, then the chip
		// drops the TX lane (or walks to STANDBY if nothing is left enabled)
		yucca0_tx_off(dgb_dt, tx_ch);
		rfnm_fe_load_latches(dgb_dt);
		rfnm_fe_trigger_latches(dgb_dt);

		if(yucca_needs_restructure(priv, &t)) {
			rtc = yucca_restructure(dgb_dt, &t, NULL, tx_ch);
			if(rtc) {
				pr_err("RFNM: Yucca TX off restructure failed, rtc %d\n", rtc);
				ecode = RFNM_API_TUNE_FAIL;
				goto fail;
			}
		}
		memcpy(dgb_dt->tx_s[0], dgb_dt->tx_ch[0], sizeof(struct rfnm_api_tx_ch));
		mutex_unlock(&rfnm_yucca_apply_lock);
		return 0;
	}

	if(yucca_needs_restructure(priv, &t)) {
		rtc = yucca_restructure(dgb_dt, &t, NULL, tx_ch);
		if(rtc) {
			pr_err("RFNM: Yucca TX mount failed at %llu Hz, rtc %d\n", tx_ch->freq, rtc);
			ecode = RFNM_API_TUNE_FAIL;
			goto fail;
		}
	} else {
		rtc = yucca_mt_set_tx_gain(priv, yucca_tx_gain_code(tx_ch->power));
		if(rtc) {
			pr_err("RFNM: Yucca TX gain failed, rtc %d\n", rtc);
			ecode = RFNM_API_GAIN_FAIL;
			goto fail;
		}
	}

	// FE last: the PA engages only once the chip is mounted and active
	yucca0_tx_on(dgb_dt, tx_ch, tx_ch->power >= YUCCA0_PA_GAIN_THRESHOLD);
	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);

	if(tx_ch->enable == RFNM_CH_RF_ON_TDD) {
		memcpy(&dgb_dt->fe_tdd[RFNM_TX], &dgb_dt->fe, sizeof(struct fe_s));
		for(i = 0; i < 2; i++) {
			if(dgb_dt->rx_s[i] && dgb_dt->rx_s[i]->enable == RFNM_CH_RF_ON_TDD) {
				rfnm_dgb_en_tdd(dgb_dt, tx_ch, dgb_dt->rx_s[i]);
				break;
			}
		}
	}

	memcpy(dgb_dt->tx_s[0], dgb_dt->tx_ch[0], sizeof(struct rfnm_api_tx_ch));
	mutex_unlock(&rfnm_yucca_apply_lock);
	return 0;

fail:
	yucca_fail_safe(dgb_dt);
fail_param:
	mutex_unlock(&rfnm_yucca_apply_lock);
	return -ecode;
}

int rfnm_rx_ch_set(struct rfnm_dgb *dgb_dt, struct rfnm_api_rx_ch * rx_ch) {
	struct rfnm_yucca_priv *priv = dgb_dt->priv_drv;
	rfnm_api_failcode ecode = RFNM_API_OK;
	struct yucca_target t;
	int ch = rx_ch->dgb_ch_id;
	u16 rtc;

	mutex_lock(&rfnm_yucca_apply_lock);

	if(rx_ch->enable != RFNM_CH_RF_OFF && (rx_ch->freq < YUCCA_FREQ_MIN || rx_ch->freq > YUCCA_FREQ_MAX)) {
		pr_err("RFNM: Yucca RX%d freq %llu Hz outside the proven MB envelope\n", ch, rx_ch->freq);
		ecode = RFNM_API_TUNE_FAIL;
		goto fail_param;
	}
	// rfic_dc_i/q not wired: the MT3812 DC trim is a per-gain coarse/fine LUT upload
	// (TRX_SetRxDc), not a single code pair - needs characterization before it can be honest

	yucca_compute_target(dgb_dt, rx_ch, NULL, &t);

	if(rx_ch->enable == RFNM_CH_RF_OFF) {
		if(ch == 0) {
			yucca0_rx_ant_a_off(dgb_dt);
		} else {
			yucca0_rx_ant_b_off(dgb_dt);
		}
		if(t.path == PATH_NONE) {
			yucca0_disable_all_pa(dgb_dt);
			yucca0_disable_all_lna(dgb_dt);
		}
		rfnm_fe_load_latches(dgb_dt);
		rfnm_fe_trigger_latches(dgb_dt);

		if(yucca_needs_restructure(priv, &t)) {
			rtc = yucca_restructure(dgb_dt, &t, rx_ch, NULL);
			if(rtc) {
				pr_err("RFNM: Yucca RX%d off restructure failed, rtc %d\n", ch, rtc);
				ecode = RFNM_API_TUNE_FAIL;
				goto fail;
			}
		}
		memcpy(dgb_dt->rx_s[ch], dgb_dt->rx_ch[ch], sizeof(struct rfnm_api_rx_ch));
		mutex_unlock(&rfnm_yucca_apply_lock);
		return 0;
	}

	if(yucca_needs_restructure(priv, &t)) {
		rtc = yucca_restructure(dgb_dt, &t, rx_ch, NULL);
		if(rtc) {
			pr_err("RFNM: Yucca RX%d mount failed at %llu Hz, rtc %d\n", ch, rx_ch->freq, rtc);
			ecode = RFNM_API_TUNE_FAIL;
			goto fail;
		}
	} else {
		// gain-only restep (this is the rfnm_agc path): just the chip gain word, no
		// side effects beyond a possible LNA bypass flip at the threshold below
		rtc = yucca_mt_set_rx_gain(priv, ch, yucca_rx_gain_code(rx_ch->gain));
		if(rtc) {
			pr_err("RFNM: Yucca RX%d gain failed, rtc %d\n", ch, rtc);
			ecode = RFNM_API_GAIN_FAIL;
			goto fail;
		}
	}

	if(ch == 0) {
		yucca0_rx_ant_a_on(dgb_dt, rx_ch->gain >= YUCCA0_LNA_GAIN_THRESHOLD);
	} else {
		yucca0_rx_ant_b_on(dgb_dt, rx_ch->gain >= YUCCA0_LNA_GAIN_THRESHOLD);
	}
	// park the PA only when TX is genuinely off or when building the RX TDD half-profile,
	// never on a gain restep during concurrent TX (lime's delta-aware parking lesson)
	if(!dgb_dt->tx_s[0] || dgb_dt->tx_s[0]->enable == RFNM_CH_RF_OFF || rx_ch->enable == RFNM_CH_RF_ON_TDD) {
		yucca0_disable_all_pa(dgb_dt);
	}
	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);

	if(rx_ch->enable == RFNM_CH_RF_ON_TDD) {
		memcpy(&dgb_dt->fe_tdd[RFNM_RX], &dgb_dt->fe, sizeof(struct fe_s));
		if(dgb_dt->tx_s[0] && dgb_dt->tx_s[0]->enable == RFNM_CH_RF_ON_TDD) {
			rfnm_dgb_en_tdd(dgb_dt, dgb_dt->tx_s[0], rx_ch);
		}
	}

	memcpy(dgb_dt->rx_s[ch], dgb_dt->rx_ch[ch], sizeof(struct rfnm_api_rx_ch));
	mutex_unlock(&rfnm_yucca_apply_lock);
	return 0;

fail:
	yucca_fail_safe(dgb_dt);
fail_param:
	mutex_unlock(&rfnm_yucca_apply_lock);
	return -ecode;
}

// debug: echo 1 > /sys/module/rfnm_yucca/parameters/mt_status dumps the chip's own
// view (sys state, PLL, path, gains) plus the driver mirror to dmesg - read-only verbs.
// This is the "Blue ground truth" instrument the mt3812 restudy asked for (§6 next-exp 1).
static int rfnm_yucca_mt_status(const char *val, const struct kernel_param *kp) {
	struct rfnm_dgb *dgb_dt;
	struct rfnm_yucca_priv *priv;
	struct in_api_param in_p;
	struct out_api_param out_p;
	u16 rtc;

	mutex_lock(&rfnm_yucca_apply_lock);
	dgb_dt = rfnm_yucca_dbg_dgb;
	if(!dgb_dt) {
		mutex_unlock(&rfnm_yucca_apply_lock);
		return -ENODEV;
	}
	priv = dgb_dt->priv_drv;

	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	rtc = mt3812_api_APP_GetSysStatus(priv->mt, &in_p, &out_p);
	if(rtc) {
		printk("RFNM: Yucca GetSysStatus failed, rtc %d\n", rtc);
	} else {
		printk("RFNM: Yucca sys_state %d, trxpll unlock %d vtune lo/hi %d/%d\n",
			out_p.app_getsysstatus.sys_state, out_p.app_getsysstatus.trxpll_unlock,
			out_p.app_getsysstatus.trxpll_vtune_det_lo, out_p.app_getsysstatus.trxpll_vtune_det_hi);
	}

	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	rtc = mt3812_api_TRX_GetTrxPll(priv->mt, &in_p, &out_p);
	if(!rtc) {
		uint64_t lo = ((uint64_t)(((u32)out_p.trx_gettrxpll.freq_msw << 16) | out_p.trx_gettrxpll.freq_lsw)) * 500;
		printk("RFNM: Yucca trxpll mode %d vco_sel %d lo %llu Hz\n", out_p.trx_gettrxpll.mode, out_p.trx_gettrxpll.vco_sel, lo);
	}

	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	rtc = mt3812_api_TRX_GetPath(priv->mt, &in_p, &out_p);
	if(!rtc) {
		printk("RFNM: Yucca path %d band %d rxbw %d txbw %d\n", out_p.trx_getpath.path,
			out_p.trx_getpath.band, out_p.trx_getpath.rxbw, out_p.trx_getpath.txbw);
	}

	memset(&in_p, 0, sizeof(in_p));
	memset(&out_p, 0, sizeof(out_p));
	rtc = mt3812_api_TRX_GetGain(priv->mt, &in_p, &out_p);
	if(!rtc) {
		printk("RFNM: Yucca gains rx1 0x%02x rx2 0x%02x tx1 0x%02x (raw %04x/%04x/%04x)\n",
			out_p.trx_getgain.rx1_gain, out_p.trx_getgain.rx2_gain, out_p.trx_getgain.tx1_gain,
			out_p.trx_getgain.rx1_gain_raw, out_p.trx_getgain.rx2_gain_raw, out_p.trx_getgain.tx1_gain_raw);
	}

	printk("RFNM: Yucca mirror lo %llu Hz path %d act %d rxbw %d txbw %d gains %d/%d/%d\n",
		priv->lo_hz, priv->path, priv->act, priv->rxbw, priv->txbw,
		priv->rx_gain_code[0], priv->rx_gain_code[1], priv->tx_gain_code);

	mutex_unlock(&rfnm_yucca_apply_lock);
	return 0;
}
static const struct kernel_param_ops rfnm_yucca_mt_status_ops = {
	.set = rfnm_yucca_mt_status,
};
module_param_cb(mt_status, &rfnm_yucca_mt_status_ops, NULL, 0200);

static int rfnm_yucca_probe(struct spi_device *spi)
{
	struct rfnm_bootconfig *cfg;
	cfg = memremap(RFNM_BOOTCONFIG_PHYADDR, SZ_4M, MEMREMAP_WB);

	struct spi_master *spi_master = spi->master;
	int dgb_id = spi_master->bus_num - 1;

	if(	cfg->daughterboard_present[dgb_id] != RFNM_DAUGHTERBOARD_PRESENT ||
		cfg->daughterboard_eeprom[dgb_id].board_id != RFNM_DAUGHTERBOARD_YUCCA) {
		memunmap(cfg);
		return -ENODEV;
	}
	memunmap(cfg);

	printk("RFNM: Loading Yucca driver for daughterboard at slot %d\n", dgb_id);

	struct device *dev = &spi->dev;
	struct rfnm_dgb *dgb_dt;
	struct rfnm_yucca_priv *priv;
	struct rfnm_api_tx_ch *tx_ch, *tx_s;
	struct rfnm_api_rx_ch *rx_ch[2], *rx_s[2];

	dgb_dt = devm_kzalloc(dev, sizeof(struct rfnm_dgb), GFP_KERNEL);
	priv = devm_kzalloc(dev, sizeof(struct rfnm_yucca_priv), GFP_KERNEL);
	if(!dgb_dt || !priv) {
		return -ENOMEM;
	}

	dgb_dt->dgb_id = dgb_id;

	dgb_dt->rx_ch_set = rfnm_rx_ch_set;
	dgb_dt->rx_ch_get = rfnm_rx_ch_get;
	dgb_dt->tx_ch_set = rfnm_tx_ch_set;
	dgb_dt->tx_ch_get = rfnm_tx_ch_get;

	// chip reset sequence: rails up, then NRST released with the TRX latch strapped LOW
	// (on this board low = SPI control interface; the strap role ends at NRST release and
	// the same latch becomes the runtime TXRX_SW selector)

	rfnm_gpio_output(dgb_id, RFNM_DGB_GPIO5_16);
	rfnm_gpio_output(dgb_id, RFNM_DGB_GPIO3_22);
	rfnm_gpio_output(dgb_id, RFNM_DGB_GPIO4_5);
	rfnm_gpio_output(dgb_id, RFNM_DGB_GPIO4_8);

	rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO3_22); // VDD_EN
	rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_8); // VCORE_EN
	rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO5_16); // MPM_EN
	msleep(1);

	rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO3_22);
	rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_8);
	msleep(1);

	rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_5); // MT_NRST

	msleep(1);

	rfnm_fe_generic_init(dgb_dt, RFNM_YUCCA0_NUM_LATCHES);

	rfnm_fe_srb(dgb_dt, RFNM_YUCCA0_MT_TRX, 0);
	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);

	msleep(1);

	rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_5);

	spi_set_drvdata(spi, dgb_dt);
	spi->max_speed_hz = 1000000;
	spi->bits_per_word = 16;
	spi->mode = SPI_MODE_0;

	int ret = spi_setup(spi);
	if (ret) {
		dev_err(dev, "SPI setup failed: %d\n", ret);
		return ret;
	}

	// zeroed init_params on purpose: the SDK's GPIO plumbing stays inert (reset is done
	// above, outside the SDK), then the handle is patched with the real SPI transport
	struct mt3812_init_params mt_ip = {0};
	mt3812_handle_t mt_h = mt3812_drv_open(&mt_ip);
	if(!mt_h) {
		pr_err("RFNM: Yucca mt3812_drv_open failed\n");
		return -ENOMEM;
	}
	struct mt3812_dev *mt_dev = (struct mt3812_dev*) mt_h;
	mt_dev->cmd_rw_device_h = spi;
	mt_dev->magic_word = 0xBA55D00D;

	priv->mt = mt_dev;
	priv->rx_gain_code[0] = -1;
	priv->rx_gain_code[1] = -1;
	priv->tx_gain_code = -1;
	dgb_dt->priv_drv = priv;

	if (	mt3812_firmware_load(mt_h,  fw_prog_mem_size_R2_7_001, &fw_prog_mem_R2_7_001[0],
			fw_init_mem_size_R2_7_001, &fw_init_mem_R2_7_001[0])) {

		pr_err("RFNM: Yucca firmware load failed\n");
		goto fail_mt;
	}

	struct in_api_param in_p;
	struct out_api_param out_p;

	// CalResistorCalib (zeroed params) makes the chip PERFORM the calibration; the plain
	// CalResistor(rcal) variant force-writes a value and skips it
	memset(&in_p, 0, sizeof(struct in_api_param));
	memset(&out_p, 0, sizeof(struct out_api_param));
	if(mt3812_api_APP_CalResistorCalib(mt_h, &in_p, &out_p)) {
		pr_err("RFNM: Yucca resistor calib failed\n");
		goto fail_mt;
	}

	memset(&in_p, 0, sizeof(struct in_api_param));
	memset(&out_p, 0, sizeof(struct out_api_param));
	if(mt3812_api_APP_CalRegulator(mt_h, &in_p, &out_p)) {
		pr_err("RFNM: Yucca regulator calib failed\n");
		goto fail_mt;
	}

	// post-cal the chip must sit in STANDBY (the tune-from state); the radio itself stays
	// down until the first channel apply - no hardcoded probe bring-up
	sys_state_t state;
	if(yucca_mt_sys_state(priv, &state)) {
		pr_err("RFNM: Yucca GetSysStatus failed after cal\n");
		goto fail_mt;
	}
	if(state != SS_STANDBY) {
		pr_warn("RFNM: Yucca chip reports state %d after cal (expected STANDBY)\n", state);
	}

	yucca0_disable_all_pa(dgb_dt);
	yucca0_disable_all_lna(dgb_dt);
	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);

	printk("RFNM: Yucca daughterboard initialized\n");

	tx_ch = devm_kzalloc(dev, sizeof(struct rfnm_api_tx_ch), GFP_KERNEL);
	tx_s = devm_kzalloc(dev, sizeof(struct rfnm_api_tx_ch), GFP_KERNEL);

	rx_ch[0] = devm_kzalloc(dev, sizeof(struct rfnm_api_rx_ch), GFP_KERNEL);
	rx_ch[1] = devm_kzalloc(dev, sizeof(struct rfnm_api_rx_ch), GFP_KERNEL);
	rx_s[0] = devm_kzalloc(dev, sizeof(struct rfnm_api_rx_ch), GFP_KERNEL);
	rx_s[1] = devm_kzalloc(dev, sizeof(struct rfnm_api_rx_ch), GFP_KERNEL);

	if(!(tx_ch && tx_s && rx_ch[0] && rx_ch[1] && rx_s[0] && rx_s[1])) {
		goto fail_mt;
	}

	// registered for every slot (the validation Yucca sat in slot 2); note the daughterboard
	// core currently accepts TX channels from slot 0 only - the driver is ready for when
	// that framework gate lifts
	tx_ch->freq_min = YUCCA_FREQ_MIN;
	tx_ch->freq_max = YUCCA_FREQ_MAX;
	tx_ch->path_preferred = RFNM_PATH_SMA_B;
	tx_ch->path_possible[0] = RFNM_PATH_SMA_B;
	tx_ch->path_possible[1] = RFNM_PATH_SMA_A;
	tx_ch->path_possible[2] = RFNM_PATH_NULL;
	tx_ch->dac_id = 0;
	tx_ch->power_range.min = YUCCA_TX_POWER_MIN;
	tx_ch->power_range.max = YUCCA_TX_POWER_MAX;
	rfnm_dgb_reg_tx_ch(dgb_dt, tx_ch, tx_s);

	rx_ch[0]->freq_min = YUCCA_FREQ_MIN;
	rx_ch[0]->freq_max = YUCCA_FREQ_MAX;
	rx_ch[0]->path_preferred = RFNM_PATH_SMA_A;
	rx_ch[0]->path_possible[0] = RFNM_PATH_SMA_A;
	rx_ch[0]->path_possible[1] = RFNM_PATH_NULL;
	rx_ch[0]->gain_range.min = 0;
	rx_ch[0]->gain_range.max = YUCCA_RX_GAIN_MAX;
	rx_ch[0]->adc_id = 1;

	rx_ch[1]->freq_min = YUCCA_FREQ_MIN;
	rx_ch[1]->freq_max = YUCCA_FREQ_MAX;
	rx_ch[1]->path_preferred = RFNM_PATH_SMA_B;
	rx_ch[1]->path_possible[0] = RFNM_PATH_SMA_B;
	rx_ch[1]->path_possible[1] = RFNM_PATH_NULL;
	rx_ch[1]->gain_range.min = 0;
	rx_ch[1]->gain_range.max = YUCCA_RX_GAIN_MAX;
	rx_ch[1]->adc_id = 0;

	// dgb ch 0 (ANT A) -> chip lane RX1, ch 1 (ANT B) -> RX2 assumed by registration order;
	// if hardware testing shows the antennas crossed, swap the lane pick in yucca_mt_set_rx_gain
	// callers and yucca_compute_target, not the FE helpers
	rfnm_dgb_reg_rx_ch(dgb_dt, rx_ch[0], rx_s[0]);
	rfnm_dgb_reg_rx_ch(dgb_dt, rx_ch[1], rx_s[1]);

	dgb_dt->dac_ifs = 0xf;
	dgb_dt->dac_iqswap[0] = 1;
	dgb_dt->dac_iqswap[1] = 0;
	// Hardware truth (lime-TX LO-leak A/B + the 3440.64 MHz clock-harmonic beacon,
	// both cross-checked against the proven lime RX): ANT B / local adc 0 = spectrally
	// CORRECT; ANT A / local adc 1 = spectrally MIRRORED (hardware I/Q crossing on that
	// lane). The GPOUT lane-swap these values drive is INERT for slot-1 lanes (flipping
	// [1] 1->0 changed nothing on the air), same as the lime slot-0 note - the real fix
	// is per-ADC conjugation in the shared unpack path (la9310_rfnm fill/pack), owed.
	dgb_dt->adc_iqswap[0] = 0;
	dgb_dt->adc_iqswap[1] = 0;
	rfnm_dgb_reg(dgb_dt);

	rfnm_yucca_dbg_dgb = dgb_dt;
	return 0;

fail_mt:
	// a half-initialized probe used to return 0 here: the driver core kept the binding with
	// no channels registered, and a later unbind ran rfnm_dgb_unreg on a never-registered dgb
	mt3812_drv_close(mt_h);
	return -ENODEV;
}

static void rfnm_yucca_remove(struct spi_device *spi)
{
	struct rfnm_dgb *dgb_dt = spi_get_drvdata(spi);
	struct rfnm_yucca_priv *priv = dgb_dt->priv_drv;

	// under the apply lock so the mt_status hook can't race the handle teardown
	mutex_lock(&rfnm_yucca_apply_lock);
	if(rfnm_yucca_dbg_dgb == dgb_dt) {
		rfnm_yucca_dbg_dgb = NULL;
	}
	mutex_unlock(&rfnm_yucca_apply_lock);

	rfnm_dgb_unreg(dgb_dt);
	// dgb_dt and priv are devm-allocated (kfree here was a double free); the mt handle is not
	mt3812_drv_close((mt3812_handle_t)priv->mt);

	// power the chip down in reverse of the probe: into reset, then rails off
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_5); // MT_NRST
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO3_22); // VDD_EN
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_8); // VCORE_EN
}

static const struct spi_device_id rfnm_yucca_ids[] = {
	{ "rfnm,daughterboard" },
	{},
};
MODULE_DEVICE_TABLE(spi, rfnm_yucca_ids);


static const struct of_device_id rfnm_yucca_match[] = {
	{ .compatible = "rfnm,daughterboard" },
	{},
};
MODULE_DEVICE_TABLE(of, rfnm_yucca_match);

static struct spi_driver rfnm_yucca_spi_driver = {
	.driver = {
		.name = "rfnm_yucca",
		.of_match_table = rfnm_yucca_match,
	},
	.probe = rfnm_yucca_probe,
	.remove = rfnm_yucca_remove,
	.id_table = rfnm_yucca_ids,
};

module_spi_driver(rfnm_yucca_spi_driver);


MODULE_PARM_DESC(device, "RFNM Yucca Daughterboard Driver");

MODULE_LICENSE("GPL");
