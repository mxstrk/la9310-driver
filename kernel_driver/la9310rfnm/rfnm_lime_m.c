// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/dma-mapping.h>
#include <la9310_base.h>
#include <asm/cacheflush.h>

#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>


#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/list.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>


#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/usb/composite.h>
#include <linux/err.h>

#include <linux/delay.h>

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/math64.h>

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "limesuiteng/embedded/loglevel.h"

#include "../../LimeSuiteNG/embedded/lms7002m/csr_data.h"
#include "../../LimeSuiteNG/embedded/lms7002m/spi.h"

#include "rfnm_lime0_regs.h"

#define LMS_REF_FREQ (51200000)

/* Kill switch for the whole fast-tune rework: set to 0 to force the stock LimeSuiteNG full
 * VCO search on every tune and disable all the last-value delta skips. Default on. */
#define RFNM_LIME_SX_FAST_TUNE 1

/* Set to 1 to print a per-apply timing breakdown (fe/sx/rb/lpf/tail + sx mode) to dmesg
 * and re-enable the tuned-LO SPI readback + "Actual LO freq" prints. */
#define RFNM_LIME_SX_TIMING 0

#include <linux/rfnm-shared.h>
#include <linux/rfnm-gpio.h>

#include "rfnm_fe_generic.h"
#include "rfnm_fe_lime0.h"
#include "rfnm_lime_mcu.h"


static int lms_spi16_transact(const uint32_t* mosi, uint32_t* miso, uint32_t count, void* userData) {
	struct spi_device *spi = (struct spi_device *)userData;
	struct spi_transfer xfer;
	uint32_t txbuf;
	uint32_t rxbuf;
	size_t i;
	int ret;

	for (i=0; i < count; i++) {
		memset(&xfer, 0, sizeof(xfer));
		txbuf = mosi[i];
		xfer.tx_buf = &txbuf;
		xfer.len = 4;
		if (!(txbuf & (1 << 31)) && miso)
			xfer.rx_buf = &rxbuf;

		ret = spi_sync_transfer(spi, &xfer, 1);

		if (ret < 0) {
			printk("spi_sync_transfer failed: %d\n", ret);
		}

		// if this is a read, signified by the most significant bit being cleared,
		// we need the data in the received buffer. If the miso output buffer is
		// not NULL, stick the data in there.
		if (!(txbuf & (1 << 31)) && miso) {
			miso[i] = rxbuf;
		}
	}

#if 0

    {
        size_t  max_words = count;
        size_t  word_len  = 11; /* "0x%08X " → up to 10 chars + NUL */
        char   *tx_str    = kmalloc(max_words * word_len, GFP_KERNEL);
        char   *p;
        size_t  len;

        if (!tx_str) {
            printk(KERN_ERR "lms_spi16_transact: out of memory for TX dump\n");
        } else {
            /* build the TX string */
            p = tx_str;
            for (i = 0; i < count; i++) {
                len = sprintf(p, "0x%08X ", mosi[i]);
                p += len;
            }
            /* single printk of the entire TX buffer */
            printk(KERN_DEBUG "SPI TX data: %s\n", tx_str);
            kfree(tx_str);
        }
    }

    if (miso) {
        size_t  max_words = count;
        size_t  word_len  = 11;
        char   *rx_str    = kmalloc(max_words * word_len, GFP_KERNEL);
        char   *p;
        size_t  len;

        if (!rx_str) {
            printk(KERN_ERR "lms_spi16_transact: out of memory for RX dump\n");
        } else {
            p = rx_str;
            for (i = 0; i < count; i++) {
                len = sprintf(p, "0x%08X ", miso[i]);
                p += len;
            }
            printk(KERN_DEBUG "SPI RX data: %s\n", rx_str);
            kfree(rx_str);
        }
    }

#endif

	return 0;
}

int parse_lime_iq_lpf(int mhz) {
	if(mhz >= 100) {
		return 100;
	} else if(mhz >= 90) {
		return 90;
	} else if(mhz >= 80) {
		return 80;
	} else if(mhz >= 70) {
		return 70;
	} else if(mhz >= 60) {
		return 60;
	} else if(mhz >= 50) {
		return 50;
	} else if(mhz >= 40) {
		return 40;
	} else if(mhz >= 30) {
		return 30;
	} else if(mhz >= 20) {
		return 20;
	} else if(mhz >= 15) {
		return 15;
	} else if(mhz >= 10) {
		return 10;
	} else if(mhz >= 5) {
		return 5;
	} else if(mhz >= 3) {
		return 3;
	} else if(mhz >= 1) {
		return 1;
	} else {
		return 100;
	}
}



void lime0_set_iq_tx_lpf_bandwidth(struct rfnm_dgb *dgb_dt, int txbw) {
	struct lms7002m_context *lms;
	lms = dgb_dt->priv_drv;

	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
#if 0
	// LMS7002M_set_mac_ch(lms, LMS_CHA);
	switch(txbw) {
		case 100:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,9);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 52);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 193);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,25);
			break;
		case 90:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,7);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 40);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 193);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,25);
			break;

		case 80:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,7);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 29);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 193);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,25);
			break;

		case 70:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,6);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 18);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 193);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,24);
			break;
		case 60:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,5);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 7);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 193);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,24);
			break;
		case 50:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,4);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 0);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 193);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,29);
			break;
		case 40:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,33);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 97);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 255);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,20);
			break;
		case 30:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,34);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 97);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 204);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,26);
			break;
		case 20:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,33);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 97);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 120);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,25);
			break;
		case 15:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,22);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 97);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 77);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,24);
			break;
		case 10:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,22);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 97);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 35);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,24);
			break;
		case 5:
			lms7002m_spi_modify_csr(lms, LMS7002M_CG_IAMP_TBB,22);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFH_TBB, 97);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFLAD_TBB, 0);
			lms7002m_spi_modify_csr(lms, LMS7002M_RCAL_LPFS5_TBB, 76);
			lms7002m_spi_modify_csr(lms, LMS7002M_CCAL_LPFLAD_TBB,31);
			break;
	}
#endif
#if 0
	// LMS7002M_regs_spi_write(lms, 0x0108);
	// LMS7002M_regs_spi_write(lms, 0x0109);
	// LMS7002M_regs_spi_write(lms, 0x010a);

	// lms->regs->reg_0x010b_value = 1;
	// LMS7002M_regs_spi_write(lms, 0x010b);
	lms7002m_spi_modify_csr(lms, LMS7002M_R5_LPF_BYP_TBB, 1);


	//self->regs->reg_0x0105_pd_lpfh_tbb = 1;
	lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFH_TBB, 1);
   // self->regs->reg_0x0105_pd_lpflad_tbb = 1;
	lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFLAD_TBB , 1);
    //self->regs->reg_0x0105_pd_lpfs5_tbb = 1;
	lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFS5_TBB , 1);
    //self->regs->reg_0x010a_bypladder_tbb = 1;
	lms7002m_spi_modify_csr(lms, LMS7002M_BYPLADDER_TBB, 1); 
#endif

#if 0
	 if(txbw < 50) {
	// 	LMS7002M_tbb_set_path(lms, LMS_CHA, LMS7002M_TBB_HBF);
	// 	LMS7002M_tbb_set_test_in(lms, LMS_CHA, LMS7002M_TBB_TSTIN_LBF);

	printk("ERROR: For some reason, tx filters below 50 MHz don't work. Don't use them. ");

		//self->regs->reg_0x0105_pd_lpfh_tbb = 0;
		lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFH_TBB, 0);

		lms7002m_spi_modify_csr(lms, LMS7002M_TSTIN_TBB, 1);
	 } else {
	// 	LMS7002M_tbb_set_path(lms, LMS_CHA, LMS7002M_TBB_LBF);

		//self->regs->reg_0x010a_bypladder_tbb = 0;
        lms7002m_spi_modify_csr(lms, LMS7002M_BYPLADDER_TBB, 0); 
		//self->regs->reg_0x0105_pd_lpflad_tbb = 0;
        lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFLAD_TBB , 0);
		//self->regs->reg_0x0105_pd_lpfs5_tbb = 0;
		lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFS5_TBB , 0);


	// 	LMS7002M_tbb_set_test_in(lms, LMS_CHA, LMS7002M_TBB_TSTIN_HBF);
		lms7002m_spi_modify_csr(lms, LMS7002M_TSTIN_TBB, 2);
	 }
#else

if (txbw < 50) {
    // **Low-band path** (<50 MHz)
    lms7002m_spi_modify_csr(lms, LMS7002M_BYPLADDER_TBB,    0);  // go through ladder
    lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFLAD_TBB,    0);  // power up ladder
    lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFS5_TBB,     0);  // power up real-pole
    lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFH_TBB,      1);  // keep HPF off
    lms7002m_spi_modify_csr(lms, LMS7002M_R5_LPF_BYP_TBB,   0);  // engage real-pole
    lms7002m_spi_modify_csr(lms, LMS7002M_TSTIN_TBB,        2);  // route test into low-band filter
}
else {
    // **High-band path** (≥50 MHz)
    lms7002m_spi_modify_csr(lms, LMS7002M_BYPLADDER_TBB,    1);  // bypass ladder
    lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFH_TBB,      0);  // power up HPF
    lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFLAD_TBB,    1);  // keep ladder off
    lms7002m_spi_modify_csr(lms, LMS7002M_PD_LPFS5_TBB,     1);  // keep real-pole off
    lms7002m_spi_modify_csr(lms, LMS7002M_R5_LPF_BYP_TBB,   1);  // buffer real-pole
    lms7002m_spi_modify_csr(lms, LMS7002M_TSTIN_TBB,        1);  // route test into high-band filter
}


//lms7002m_spi_modify_csr(lms, LMS7002M_EN_LOOPB_TXPAD_TRF, 0);



#endif
	 
}

void rfnm_lime_set_bias_t(struct rfnm_dgb *dgb_dt, enum rfnm_bias_tee bias_tee) {
	if(bias_tee == RFNM_BIAS_TEE_ON) {
		rfnm_fe_srb(dgb_dt, RFNM_LIME0_ANT_BIAS, 0);
	} else {
		rfnm_fe_srb(dgb_dt, RFNM_LIME0_ANT_BIAS, 1);
	}
}

#if RFNM_LIME_SX_FAST_TUNE
static int rx_last_path_rfe = -1;	/* last LNA path programmed via set_path_rfe; -1 = unknown (force after probe) */
static int32_t rx_last_lna_db = S32_MIN;	/* last raw decibel value written via set_rfelna_db */
#endif
static int rx_last_pga_db = S32_MIN;	/* last PGA dB written via set_rbbpga_db */

/* RX DC guard (2026-07-21 reform): the PGA redistribution may only amplify the analog DC
 * while a live DCOFF null holds it down - the DC is LO- and temperature-dependent (P0 map:
 * 110..640 12-bit counts across 1.5-3.5 GHz at PGA 0), so freshness means a trim written
 * this boot at an LO within dc_lo_window_mhz of the current tune. The cap keeps
 * resid * 10^(pga/20) <= dc_budget_counts; with the defaults an untrimmed chip is capped
 * at PGA +3 (worst DC ~988 counts, the +32752 stream rail is unreachable) and a fresh
 * trim runs the full +12 (residual allowance 160 counts -> <=636). Trim slope for scale:
 * ~ -9 counts/code at PGA 0 (P0 2026-07-21; the 07-08 "-34" was the same slope observed
 * at max PGA). */
static int dc_budget_counts = 1024;
module_param(dc_budget_counts, int, 0644);
MODULE_PARM_DESC(dc_budget_counts, "max 12-bit counts of DC the PGA may amplify to (1024 = -6 dBFS)");
static int dc_untrimmed_counts = 700;
module_param(dc_untrimmed_counts, int, 0644);
MODULE_PARM_DESC(dc_untrimmed_counts, "worst-case un-nulled DC, 12-bit counts (P0 LO map 2026-07-21)");
static int dc_trimmed_counts = 160;
module_param(dc_trimmed_counts, int, 0644);
MODULE_PARM_DESC(dc_trimmed_counts, "post-null residual allowance, 12-bit counts");
static int dc_lo_window_mhz = 300;
module_param(dc_lo_window_mhz, int, 0644);
MODULE_PARM_DESC(dc_lo_window_mhz, "DCOFF trim validity window around the trim LO (P0: +-300 MHz holds residual ~<=100 counts)");
static int64_t rx_dc_trim_hz = -1;	/* LO at the last nonzero DCOFF write; -1 = never trimmed this boot */

extern atomic_t rfnm_rx_apply_epoch;	/* la9310rfnm: rfnm_qec re-arms its one-shot null on our bump */

/* one lock around every path that talks LMS SPI / FE latches: the chlist apply work, the
 * sysfs store path, and the rfnm_agc fast hooks all reach here concurrently, and interleaved
 * MAC/register sequences corrupt each other */
static DEFINE_MUTEX(rfnm_lime_apply_lock);

void lime0_set_rx_gain(struct rfnm_dgb *dgb_dt, struct rfnm_api_rx_ch * rx_ch) {
	struct lms7002m_context *lms;
	lms = dgb_dt->priv_drv;
	int dbm = rx_ch->gain;
	int32_t lna_raw;
	int pga_db;

	// The ANT port is ONE switch shared by both directions. While this
	// dgb's TX is RF-live the TX apply owns it (lime0_ant_tx) - repointing it here
	// for RX gain staging made plain RF_ON duplex a per-session draw (whichever
	// apply wrote the latch last won the port; a TX re-apply "fixing" it was the
	// OAI shim's accidental dodge). With TX live, RX on a shared-port DB receives
	// via switch leakage and the attenuator stages are out of circuit - gain lives
	// on LNA/PGA alone. TX RF_OFF hands the port back on the next RX apply (the
	// apply pipeline runs the TX chlist first, so a combined off+rx apply lands
	// with the port back at the RX selection).
	// Plain RF_ON only. Under RF_ON_TDD the M7 flips the port per pattern
	// edge and the RX half-profile snapshot (fe_tdd[RX], below at the RF_ON_TDD
	// block) must carry a real RX ANT selection - skipping the staging here handed
	// the M7 a TX-owned "RX" profile on every AGC restep / re-apply.
	if(dgb_dt->tx_s[0] && dgb_dt->tx_s[0]->enable == RFNM_CH_RF_ON) {
		if(dbm < 0) {
			dbm = 0;	// no attenuator stages available while TX owns the port
		}
	} else if(rx_ch->path == RFNM_PATH_TERMINATED) {
		// terminated input (receiver-only floor): the ANT attenuator switches ARE the
		// termination, so gain stays on LNA/PGA alone (mirror of the TX-owned case)
		lime0_rx_terminated(dgb_dt);
		if(dbm < 0) {
			dbm = 0;
		}
	} else if(rx_ch->path == RFNM_PATH_EMBED_ANT) {
		lime0_ant_embed(dgb_dt);
		//printk("embed\n");
	} else {
		//printk("not embed\n");
		if(dbm < -12) {
			// enable 24 dB attenuator
			dbm += 24;
			lime0_ant_attn_24(dgb_dt);
		} else if(dbm < 0) {
			// enable 12 dB attenuator
			dbm += 12;
			lime0_ant_attn_12(dgb_dt);
		} else {
			lime0_ant_through(dgb_dt);
		}
	}

	if(dbm >= 0) {
		if(dbm > 30) {
			dbm = 30;
		}
		lna_raw = dbm * 65536;
	} else {
		lna_raw = 0;
	}

#if RFNM_LIME_SX_FAST_TUNE
	if(lna_raw != rx_last_lna_db) {
		struct lms7002m_decibel lms_db = { lna_raw };
		lms7002m_set_rfelna_db(lms, lms_db, LMS7002M_CHANNEL_AB);
		rx_last_lna_db = lna_raw;
	}
#else
	{
		struct lms7002m_decibel lms_db = { lna_raw };
		lms7002m_set_rfelna_db(lms, lms_db, LMS7002M_CHANNEL_AB);
	}
#endif

	/* baseband gain redistribution (2026-07-08 study): with PGA fixed at 0 dB the LA9310 ADC
	 * floor is ~half the total noise power at low front gain (measured 3.6 vs 3.35 counts rms
	 * at gain 0), which is what made the DCS divide-bit / high-rate configs visibly noisier.
	 * Ramp the PGA up as the front gain comes down so the analog floor stays ~10 dB above the
	 * ADC floor at every setting. Costs delivered-level slope (~0.5 dB/dB for gain 0..24). */
	int chip_lb = rx_ch->path == RFNM_PATH_LOOPBACK && dgb_dt->tx_s[0] &&
			dgb_dt->tx_s[0]->enable == RFNM_CH_RF_ON &&
			dgb_dt->tx_s[0]->path != RFNM_PATH_LOOPBACK;
	if(chip_lb) {
		// chip-LB: set_path_rfe(LB*) parks the LNA this mapping feeds, so the
		// redistribution rule below runs the knob BACKWARDS (more requested gain =
		// less PGA = less of the only live stage). In LB the requested gain drives
		// the PGA directly; the coarse rung is the LB buffer, dBm-engine scaled at
		// path selection. DC guard below still caps.
		pga_db = dbm / 2;
	} else {
		pga_db = 12 - dbm / 2;
	}
	if(pga_db < 0) {
		pga_db = 0;
	}
	if(pga_db > 12) {
		pga_db = 12;
	}

	/* DC guard: the redistribution's old precondition ("relies on the rfnm_agc DC null")
	 * as enforced arithmetic instead of a comment - cap the PGA so the predicted un-nulled
	 * DC can never reach the budget, let alone the rail (observed face: Q parked at +32752,
	 * 98-100% clip, at every gain <= 0). Freshness relaxes the cap; rfnm_qec's one-shot
	 * null (native sessions) or a client rfic_dc write (local sessions) establishes it. */
	{
		static const u32 pga_gain_q16[13] = { 65536, 73533, 82505, 92572, 103868, 116541,
			130762, 146717, 164619, 184706, 207243, 232531, 260904 };	/* 10^(p/20) */
		int64_t freq = rx_ch->freq;
		int64_t dist = freq > rx_dc_trim_hz ? freq - rx_dc_trim_hz : rx_dc_trim_hz - freq;
		int fresh = rx_dc_trim_hz >= 0 && dist <= (int64_t)dc_lo_window_mhz * 1000000;
		int resid = fresh ? dc_trimmed_counts : dc_untrimmed_counts;
		// Do NOT relax this for the LB path: tried 2026-07-22 (post-decim capture DC
		// "measured tiny") - the guard's counts are RAW 12-bit ADC units and the un-nulled
		// chain really does rail: PGA 10 pegged the Q lane to a constant (perfectly
		// symmetric spectrum, images equal power) until the PGA came back down. The full
		// LB PGA range is earned only by a real DC null establishing freshness above.

		while(pga_db > 0 && (int64_t)resid * pga_gain_q16[pga_db] > ((int64_t)dc_budget_counts << 16)) {
			pga_db--;
		}
	}

	if(pga_db != rx_last_pga_db) {
		struct lms7002m_decibel pga_lms_db = { pga_db * 65536 };
		lms7002m_set_rbbpga_db(lms, pga_lms_db, LMS7002M_CHANNEL_AB);
		rx_last_pga_db = pga_db;
	}
}

void rfnm_tx_ch_get(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch * tx_ch) {
	printk("inside rfnm_tx_ch_get\n");
}
void rfnm_rx_ch_get(struct rfnm_dgb *dgb_dt, struct rfnm_api_rx_ch * rx_ch) {
	printk("inside rfnm_rx_ch_get\n");
}

#define LMS_TO_DECIBEL(R) \
    (struct lms7002m_decibel) \
    { \
        ((int32_t)((R) * (1 << 16))) \
    }

/* ---- fast SX tune: cached CSW anchors + comparator verify ----
 *
 * A full lms7002m_set_frequency_sx() binary-searches the VCO cap bank (CSW_VCO) with 20+
 * comparator settles of 50-100us each; measured ~3.6ms per tune. The only non-deterministic
 * outputs of that search are SEL_VCO and CSW_VCO - INT/FRAC SDM, DIV_LOCH and EN_DIV2 are
 * pure arithmetic. We remember (vco_freq, csw) anchors from every full search, per VCO and
 * per SX (SXR/SXT), and predict CSW for new frequencies by interpolating in the 1/f^2 domain
 * (the cap bank is linear in C and f = 1/(2pi*sqrt(LC)), so CSW is ~linear in 1/f^2). The
 * predicted CSW is verified with one comparator read and walked by single codes if needed;
 * any failure falls back to the full search, which refreshes the anchors. Predicted-and-locked
 * values are NOT fed back as anchors: only a full search finds the lock-window center, and
 * temperature drift must not walk the anchors toward a window edge.
 *
 * VCO band table and SDM math duplicated from LimeSuiteNG lms7002m.c set_frequency_sx /
 * write_sx_registers, which are not usable piecemeal from there (static/monolithic). */

#define RFNM_LIME_SX_ANCHORS 16
#define RFNM_LIME_SX_SETTLE_US 60
#define RFNM_LIME_SX_WALK_MAX 10
#define RFNM_LIME_SX_EXTRAP_KHZ 100000	/* max extrapolation beyond the anchor span */
#define RFNM_LIME_SX_LONE_KHZ 30000	/* max distance to a lone anchor */
#define RFNM_LIME_SX_MERGE_KHZ 8000	/* full-search results this close refresh an anchor instead of adding one */

static uint64_t sx_last_hz[2];	/* last LO actually programmed into SXR/SXT; 0 = unknown */
static char sx_last_mode = '-';	/* k=skipped f=fast w=fast+walk F=full X=full after failed fast */

#if RFNM_LIME_SX_FAST_TUNE

struct rfnm_lime_sx_anchor {
	uint32_t vco_khz;
	uint8_t csw;
};

struct rfnm_lime_sx_cand {
	uint64_t vco_hz;
	uint8_t div_loch;
	bool ok;
};

static struct rfnm_lime_sx_anchor sx_anchors[2][3][RFNM_LIME_SX_ANCHORS];	/* [is_tx][vco], sorted by vco_khz */
static int sx_anchor_cnt[2][3];

static const uint64_t rfnm_lime_vco_min[3] = { 3800000000ULL, 4961000000ULL, 6306000000ULL };
static const uint64_t rfnm_lime_vco_max[3] = { 5222000000ULL, 6754000000ULL, 7714000000ULL };

static void rfnm_lime_sx_candidates(uint64_t lo_hz, struct rfnm_lime_sx_cand cand[3]) {
	int8_t dl;
	int i;

	memset(cand, 0, sizeof(struct rfnm_lime_sx_cand) * 3);

	// div_loch value 7 is not allowed; first (largest) in-band VCO freq wins, like set_frequency_sx
	for(dl = 6; dl >= 0; dl--) {
		uint64_t vf = ((uint64_t)1 << (dl + 1)) * lo_hz;
		for(i = 0; i < 3; i++) {
			if(!cand[i].ok && vf >= rfnm_lime_vco_min[i] && vf <= rfnm_lime_vco_max[i]) {
				cand[i].ok = true;
				cand[i].div_loch = dl;
				cand[i].vco_hz = vf;
			}
		}
	}
}

static uint64_t rfnm_lime_sx_xdom(uint32_t vco_khz) {
	return div64_u64(1000000000000000000ULL, (uint64_t)vco_khz * vco_khz);
}

static int rfnm_lime_sx_predict(int is_tx, int vco, uint32_t vco_khz) {
	struct rfnm_lime_sx_anchor *a = sx_anchors[is_tx][vco];
	int n = sx_anchor_cnt[is_tx][vco];
	uint64_t x, x0, x1;
	int64_t csw;
	int i1;

	if(n == 0) {
		return -1;
	}

	if(n == 1) {
		uint32_t d = a[0].vco_khz > vco_khz ? a[0].vco_khz - vco_khz : vco_khz - a[0].vco_khz;
		return d <= RFNM_LIME_SX_LONE_KHZ ? a[0].csw : -1;
	}

	if(vco_khz + RFNM_LIME_SX_EXTRAP_KHZ < a[0].vco_khz || vco_khz > a[n - 1].vco_khz + RFNM_LIME_SX_EXTRAP_KHZ) {
		return -1;
	}

	i1 = 1;
	while(i1 < n - 1 && a[i1].vco_khz < vco_khz) {
		i1++;
	}

	x = rfnm_lime_sx_xdom(vco_khz);
	x0 = rfnm_lime_sx_xdom(a[i1 - 1].vco_khz);
	x1 = rfnm_lime_sx_xdom(a[i1].vco_khz);
	if(x0 == x1) {
		return a[i1].csw;
	}

	csw = a[i1 - 1].csw + div64_s64(((int64_t)a[i1].csw - a[i1 - 1].csw) * ((int64_t)x - (int64_t)x0), (int64_t)x1 - (int64_t)x0);
	return csw < 0 ? 0 : (csw > 255 ? 255 : csw);
}

static void rfnm_lime_sx_learn(int is_tx, int vco, uint32_t vco_khz, uint8_t csw) {
	struct rfnm_lime_sx_anchor *a = sx_anchors[is_tx][vco];
	int n = sx_anchor_cnt[is_tx][vco];
	int i, victim = -1;
	uint32_t best_d = ~0U;

	for(i = 0; i < n; i++) {
		uint32_t d = a[i].vco_khz > vco_khz ? a[i].vco_khz - vco_khz : vco_khz - a[i].vco_khz;
		if(d < best_d) {
			best_d = d;
			victim = i;
		}
	}

	if(victim >= 0 && (best_d <= RFNM_LIME_SX_MERGE_KHZ || n == RFNM_LIME_SX_ANCHORS)) {
		memmove(&a[victim], &a[victim + 1], (n - victim - 1) * sizeof(*a));
		n--;
	}

	for(i = 0; i < n && a[i].vco_khz < vco_khz; i++) {
	}
	memmove(&a[i + 1], &a[i], (n - i) * sizeof(*a));
	a[i].vco_khz = vco_khz;
	a[i].csw = csw;
	sx_anchor_cnt[is_tx][vco] = n + 1;
}

static int rfnm_lime_fast_sx(struct lms7002m_context *lms, bool is_tx, const struct rfnm_lime_sx_cand *c, int sel_vco, int csw) {
	const uint64_t thr_hz = 5500000000ULL;	// VCO frequency threshold to enable the extra /2, as in write_sx_registers
	uint32_t refclk = lms7002m_get_reference_clock(lms);
	uint64_t divider;
	uint16_t integer, reg011c;
	uint32_t frac;
	int steps, dir = 0, cmphl;

	if(!refclk) {
		return -1;
	}

	divider = (uint64_t)refclk << (c->vco_hz > thr_hz ? 1 : 0);
	integer = div64_u64(c->vco_hz, divider);
	frac = div64_u64((c->vco_hz - (uint64_t)integer * divider) << 20, divider);
	integer -= 4;

	lms7002m_set_active_channel(lms, is_tx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);

	// 0x011C composed in one RMW: EN_DIV2_DIVPROG[10], EN_INTONLY_SDM[9]=0, PD_VCO_COMP[2]=0, PD_VCO[1]=0;
	// all other bits (RESET_N, SPDUP/BYPLDO/COARSEPLL/CURLIM, EN_SDM_CLK, the divider/CP/SDM PDs, EN_G) preserved
	reg011c = lms7002m_spi_read(lms, 0x011C);
	reg011c &= ~((1 << 10) | (1 << 9) | (1 << 2) | (1 << 1));
	if(c->vco_hz > thr_hz) {
		reg011c |= 1 << 10;
	}
	lms7002m_spi_write(lms, 0x011C, reg011c);

	lms7002m_spi_write(lms, 0x011D, frac & 0xFFFF);	// FRAC_SDM[15:0] is the full register - plain write, no RMW
	lms7002m_spi_modify(lms, 0x011E, 13, 0, ((uint16_t)integer << 4) | (frac >> 16));	// INT_SDM[13:4] + FRAC_SDM[19:16]
	lms7002m_spi_modify_csr(lms, LMS7002M_DIV_LOCH, c->div_loch);
	lms7002m_spi_modify(lms, 0x0121, 10, 1, ((uint16_t)csw << 2) | sel_vco);	// CSW_VCO[10:3] + SEL_VCO[2:1] in one op

	for(steps = 0; steps <= RFNM_LIME_SX_WALK_MAX; steps++) {
		usleep_range(RFNM_LIME_SX_SETTLE_US, RFNM_LIME_SX_SETTLE_US + 30);
		cmphl = lms7002m_spi_read_bits(lms, LMS7002M_VCO_CMPHO.address, 13, 12);
		if(cmphl == 2) {
			lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
			sx_last_mode = steps ? 'w' : 'f';
			return 0;
		}
		// cmphl semantics from tune_vco: bit0 set -> CSW too high, 0 -> too low; a direction flip means no window between codes
		if(cmphl & 0x1) {
			if(dir > 0 || csw == 0) {
				break;
			}
			dir = -1;
			csw--;
		} else {
			if(dir < 0 || csw == 255) {
				break;
			}
			dir = 1;
			csw++;
		}
		lms7002m_spi_modify_csr(lms, LMS7002M_CSW_VCO, csw);
	}

	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
	return -1;
}

#endif /* RFNM_LIME_SX_FAST_TUNE */

static bool rfnm_lime_sx_needs_tune(bool is_tx, uint64_t freq_hz) {
#if RFNM_LIME_SX_FAST_TUNE
	if(sx_last_hz[is_tx ? 1 : 0] == freq_hz) {
		sx_last_mode = 'k';
		return false;
	}
#endif
	return true;
}

/* any real RX-side SX retune invalidates the gain-stage skip caches: a proven
 * outlier (one unclipped gain -24 capture) pointed at rx_last_pga_db trusting a chip
 * that a reclock had moved on from - after a retune the next apply rewrites LNA/PGA
 * unconditionally (a few CSR writes, dwarfed by PLL settle) */
static void rfnm_lime_rx_stage_cache_drop(void) {
#if RFNM_LIME_SX_FAST_TUNE
	rx_last_path_rfe = -1;
	rx_last_lna_db = S32_MIN;
#endif
	rx_last_pga_db = S32_MIN;
}

static lime_Result rfnm_lime_tune_sx(struct lms7002m_context *lms, bool is_tx, uint64_t freq_hz) {
	int t = is_tx ? 1 : 0;
	lime_Result ret;
#if RFNM_LIME_SX_FAST_TUNE
	struct rfnm_lime_sx_cand cand[3];
	static const uint8_t pref[3] = { 2, 0, 1 };	// VCOH, VCOL, VCOM - same preference as set_frequency_sx
	int i, sel = -1, csw = -1;

	rfnm_lime_sx_candidates(freq_hz, cand);

	for(i = 0; i < 3 && csw < 0; i++) {
		int v = pref[i];
		if(cand[v].ok) {
			csw = rfnm_lime_sx_predict(t, v, div_u64(cand[v].vco_hz, 1000));
			if(csw >= 0) {
				sel = v;
			}
		}
	}

	if(csw >= 0 && !rfnm_lime_fast_sx(lms, is_tx, &cand[sel], sel, csw)) {
		sx_last_hz[t] = freq_hz;
		if(!is_tx) {
			rfnm_lime_rx_stage_cache_drop();
		}
		return lime_Result_Success;
	}

	sx_last_mode = csw >= 0 ? 'X' : 'F';
#else
	sx_last_mode = 'F';
#endif
	ret = lms7002m_set_frequency_sx(lms, is_tx, freq_hz);
	if(ret == lime_Result_Success) {
		sx_last_hz[t] = freq_hz;
		if(!is_tx) {
			rfnm_lime_rx_stage_cache_drop();
		}
#if RFNM_LIME_SX_FAST_TUNE
		lms7002m_set_active_channel(lms, is_tx ? LMS7002M_CHANNEL_SXT : LMS7002M_CHANNEL_SXR);
		sel = lms7002m_spi_read_csr(lms, LMS7002M_SEL_VCO);
		csw = lms7002m_spi_read_csr(lms, LMS7002M_CSW_VCO);
		lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
		if(sel >= 0 && sel < 3 && cand[sel].ok) {
			rfnm_lime_sx_learn(t, sel, div_u64(cand[sel].vco_hz, 1000), csw);
		}
#endif
	} else {
		sx_last_hz[t] = 0;
	}
	return ret;
}

/* TX LO-leak null engage (defined with the tx_dc knob block below) */
static void rfnm_lime_write_analog_dc(struct lms7002m_context *lms, uint16_t addr, int16_t value);
static void rfnm_lime_rx_dc_dac_set_locked(struct lms7002m_context *lms, int dc_i, int dc_q);
static int tx_dc_i, tx_dc_q;
static int tx_dc_engage = 1;
module_param(tx_dc_engage, int, 0644);
MODULE_PARM_DESC(tx_dc_engage, "engage the TX DC trim (LO-leak null) at every TX RF_ON apply (default 1; 0 = legacy never-engaged)");

/* DEBUG DOOR (TX cal model): force the switched FE / TXPAD independently of
 * the s-ladder, for per-element isolation. Values land on the NEXT tx apply (trigger one
 * by toggling any tx knob). -1 = door closed (normal policy). */
static int tx_fe_force = -1;	/* bit0 PA1, bit1 PA2, bit2 T24, bit3 T12, bit4 T6 */
static int tx_trf_force = -1;	/* raw TXPAD pad code 0..31 written to LOSS_MAIN+LOSS_LIN */
module_param(tx_fe_force, int, 0644);
MODULE_PARM_DESC(tx_fe_force, "debug: force TX FE element mask (PA1/PA2/T24/T12/T6), -1=off");
module_param(tx_trf_force, int, 0644);
MODULE_PARM_DESC(tx_trf_force, "debug: force raw TXPAD loss code 0..31, -1=off (ladder residue trim)");

static void rfnm_lime_set_txpad_code(struct lms7002m_context *lms, int code) {
	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
	lms7002m_spi_modify_csr(lms, LMS7002M_LOSS_MAIN_TXPAD_TRF, code & 31);
	lms7002m_spi_modify_csr(lms, LMS7002M_LOSS_LIN_TXPAD_TRF, code & 31);
}

/* TX dBm engine: with a per-board map loaded from
 * /lib/firmware/rfnm/tx_dbm_<serial>.map, tx_ch->power IS the requested dBm at the SMA
 * (contract: producer digital drive at the -6 dBFS reference). The map is measured
 * per-composition-state (latch mask x raw TXPAD code, compression and interactions baked
 * in); the engine picks the nearest state
 * with linear interpolation between the two bracketing map frequencies and programs the
 * latches + TXPAD directly. No map file -> legacy s-ladder semantics unchanged. */
struct rfnm_txdbm_rec {
	uint32_t freq_khz;
	uint8_t mask;
	uint8_t code;
	int16_t cdbm;
} __attribute__((__packed__));
static struct rfnm_txdbm_rec *txdbm_map;
static int txdbm_n;
static int16_t txdbm_amin, txdbm_amax;
static int txdbm_last_pred_cdbm = INT_MIN;	/* chip-LB capture-gain scaling reads this */

static void rfnm_lime_txdbm_load(struct device *dev, const uint8_t *serial) {
	const struct firmware *fw;
	char path[64];
	int i;
	snprintf(path, sizeof(path), "rfnm/tx_dbm_%.8s.map", serial);
	if(request_firmware_direct(&fw, path, dev)) {
		printk("RFNM: no TX dBm map (%s) - legacy raw power semantics\n", path);
		return;
	}
	if(fw->size < 16 || memcmp(fw->data, "RFTX", 4) || fw->data[4] != 1) {
		printk("RFNM: TX dBm map %s: bad header\n", path);
		release_firmware(fw);
		return;
	}
	txdbm_n = fw->data[6] | (fw->data[7] << 8);
	txdbm_amin = (int16_t)(fw->data[8] | (fw->data[9] << 8));
	txdbm_amax = (int16_t)(fw->data[10] | (fw->data[11] << 8));
	if(fw->size < 16 + (size_t)txdbm_n * sizeof(struct rfnm_txdbm_rec)) {
		printk("RFNM: TX dBm map %s: truncated\n", path);
		txdbm_n = 0;
		release_firmware(fw);
		return;
	}
	txdbm_map = kmemdup(fw->data + 16, txdbm_n * sizeof(struct rfnm_txdbm_rec), GFP_KERNEL);
	if(!txdbm_map) {
		txdbm_n = 0;
	}
	release_firmware(fw);
	i = txdbm_n ? 1 : 0;
	if(i) {
		printk("RFNM: TX dBm map %s: %d states, %d..%d dBm - tx power IS dBm on this board\n",
			path, txdbm_n, txdbm_amin / 100, txdbm_amax / 100);
	}
}

/* nearest map value for (mask, code) at freq_khz, linear across the two bracketing map
 * frequencies; returns INT_MIN when the state is absent at the bracket */
static int rfnm_lime_txdbm_at(uint32_t f0, uint32_t f1, uint32_t fq, uint8_t mask, uint8_t code) {
	int i, v0 = INT_MIN, v1 = INT_MIN;
	for(i = 0; i < txdbm_n; i++) {
		if(txdbm_map[i].mask != mask || txdbm_map[i].code != code) {
			continue;
		}
		if(txdbm_map[i].freq_khz == f0) {
			v0 = txdbm_map[i].cdbm;
		}
		if(txdbm_map[i].freq_khz == f1) {
			v1 = txdbm_map[i].cdbm;
		}
	}
	if(v0 == INT_MIN || v1 == INT_MIN) {
		return INT_MIN;
	}
	if(f1 == f0) {
		return v0;
	}
	return v0 + (int)((int64_t)(v1 - v0) * (fq - f0) / (f1 - f0));
}

static int rfnm_lime_txdbm_pick(uint64_t freq_hz, int req_cdbm, int *out_mask, int *out_code, int *out_pred) {
	uint32_t fq = (uint32_t)(freq_hz / 1000);
	uint32_t f0 = 0, f1 = 0xFFFFFFFF;
	int i, best = INT_MIN, best_err = INT_MAX;
	if(!txdbm_n) {
		return -1;
	}
	for(i = 0; i < txdbm_n; i++) {	/* bracketing map frequencies (clamped at the ends) */
		uint32_t f = txdbm_map[i].freq_khz;
		if(f <= fq && f > f0) {
			f0 = f;
		}
		if(f >= fq && f < f1) {
			f1 = f;
		}
	}
	if(f0 == 0) {
		f0 = f1;
	}
	if(f1 == 0xFFFFFFFF) {
		f1 = f0;
	}
	if(f0 == 0 || f1 == 0xFFFFFFFF) {
		return -1;
	}
	for(i = 0; i < txdbm_n; i++) {
		int v, err, pas, better;
		if(txdbm_map[i].freq_khz != f0) {
			continue;
		}
		v = rfnm_lime_txdbm_at(f0, f1, fq, txdbm_map[i].mask, txdbm_map[i].code);
		if(v == INT_MIN) {
			continue;
		}
		err = v > req_cdbm ? v - req_cdbm : req_cdbm - v;
		pas = (txdbm_map[i].mask & 1) + ((txdbm_map[i].mask >> 1) & 1);
		/* closest wins; within 0.25 dB prefer fewer PA stages (headroom rule) */
		better = 0;
		if(best == INT_MIN || err + 25 < best_err) {
			better = 1;
		} else if(err <= best_err + 25 &&
			pas < ((*out_mask & 1) + ((*out_mask >> 1) & 1))) {
			better = 1;
		}
		if(better) {
			best = v;
			best_err = err;
			*out_mask = txdbm_map[i].mask;
			*out_code = txdbm_map[i].code;
			*out_pred = v;
		}
	}
	return best == INT_MIN ? -1 : 0;
}

int rfnm_tx_ch_set(struct rfnm_dgb *dgb_dt, struct rfnm_api_tx_ch * tx_ch) {
	rfnm_api_failcode ecode = RFNM_API_OK;
	lime_Result ret;
	uint64_t freq = tx_ch->freq / (MHZ_TO_HZ(1));
	struct lms7002m_context *lms;
	lms = dgb_dt->priv_drv;

	mutex_lock(&rfnm_lime_apply_lock);

	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);

	if(tx_ch->enable == RFNM_CH_RF_OFF) {
		// RF_OFF must actually stop radiating: with the TRF left biased, the DAC's frozen
		// constant upconverts to a full-power carrier at the TX LO after the stream stops
		lms7002m_enable_channel(lms, true, LMS7002M_CHANNEL_A, false);
		// EN_G gates the whole SXT block; force a (fast) retune on the next RF_ON rather than trusting the parked VCO to re-lock
		sx_last_hz[1] = 0;
		memcpy(dgb_dt->tx_s[0], dgb_dt->tx_ch[0], sizeof(struct rfnm_api_tx_ch));
		mutex_unlock(&rfnm_lime_apply_lock);
		return 0;
	}

	// need to enable those modules before other writes
	lms7002m_enable_channel(lms, true, LMS7002M_CHANNEL_A, true);

	// Engage the TX LO-leak null on every RF_ON. The DC trim DAC + comparator
	// were powered down from boot (0x05C0=0x00BF) and no TX cal exists anywhere in
	// the lifecycle - the un-nulled node then drifts during continuous TX (the leak)
	// until the growing carrier swamps every DC-blind correlator (the LO-leak "decay";
	// an earlier "thermal drift" story was this instrument delusion). Trim values ride
	// the live-walkable tx_dc_i/q params; the bias enables are idempotent.
	if(tx_dc_engage) {
		lms7002m_spi_modify_csr(lms, LMS7002M_DCMODE, 1);
		lms7002m_spi_modify_csr(lms, LMS7002M_PD_DCDAC_TXA, 0);
		lms7002m_spi_modify_csr(lms, LMS7002M_PD_DCCMP_TXA, 0);
		rfnm_lime_write_analog_dc(lms, 0x05C3, tx_dc_i);
		rfnm_lime_write_analog_dc(lms, 0x05C4, tx_dc_q);
	}



	// this might fail because some tx paths are disabled when run in rx mode?
	if(rfnm_lime_sx_needs_tune(true, tx_ch->freq)) {
		ret = rfnm_lime_tune_sx(lms, true, tx_ch->freq);
		if(ret) {
			printk("Tuning failed\n");
			ecode = RFNM_API_TUNE_FAIL;
			goto fail;
		}
#if RFNM_LIME_SX_TIMING
		printk("%d - Actual TX LO freq %llu Hz (sx_mode=%c)\n", ret, lms7002m_get_frequency_sx(lms, true), sx_last_mode);
#endif
	}

	if(freq < 2350) {
		lime0_tx_band(dgb_dt, RFNM_LIME0_TX_BAND_LOW);
		lms7002m_set_band_trf(lms, 2);
	} else {
		lime0_tx_band(dgb_dt, RFNM_LIME0_TX_BAND_HIGH);
		lms7002m_set_band_trf(lms, 1);
	}

	//struct lms7002m_decibel ldb = {LMS_TO_DECIBEL(52)};

	

	// Power double-spend defect: the factory pwr tables (May 2025) were minted with the FE ladder
	// alone at full LMS drive - the trfpad line below rode along commented out. A later change uncommented
	// it with the RAW power, double-spending the knob (TXPAD loss 52-power stacked on the ladder)
	// and sinking live output (52-power) dB under every table row. Convention restored: the FE
	// ladder (PA1 > 0, PA2 > 20, pads walk) owns the knob; the TXPAD sits at loss 0 (the mint
	// state - calib_pwr.tbl rows are physically true at the anchor settings) and absorbs only the
	// ladder's residual overshoot, so the between-anchor settings interpolate in ~1 dB steps.
	// Retro-note: earlier "dead TL1 bypass" measurements were this defect (power <= 0 sat 52 dB down);
	// the leg measures healthy in the affected unit's own factory rows.
	{
		int em, ec, ep;
		if(tx_fe_force >= 0) {
			lime0_tx_force(dgb_dt, tx_fe_force);
			if(tx_trf_force >= 0) {
				rfnm_lime_set_txpad_code(lms, tx_trf_force);
			}
		} else if(txdbm_n && !rfnm_lime_txdbm_pick(tx_ch->freq, tx_ch->power * 100, &em, &ec, &ep)) {
			lime0_tx_force(dgb_dt, em);
			rfnm_lime_set_txpad_code(lms, ec);
			txdbm_last_pred_cdbm = ep;
			printk("RFNM: txdbm req %d dBm @ %llu kHz -> mask %d code %d (predicted %d.%02d dBm)\n",
				tx_ch->power, tx_ch->freq / 1000, em, ec, ep / 100, abs(ep % 100));
		} else {
			int fe_residue_cdb = lime0_tx_power(dgb_dt, freq, tx_ch->power);
			if(tx_trf_force >= 0) {
				rfnm_lime_set_txpad_code(lms, tx_trf_force);
			} else {
				int trf_db = 52;
				if(fe_residue_cdb < 0) {
					trf_db = 52 + (fe_residue_cdb - 50) / 100;
				}
				lms7002m_set_trfpad_db(lms, LMS_TO_DECIBEL(trf_db), LMS7002M_CHANNEL_A);
			}
		}
	}


	lime0_tx_lpf(dgb_dt, freq);

	lms7002m_set_tx_lpf(lms, MHZ_TO_HZ(tx_ch->rfic_lpf_bw));
	//lime0_set_iq_tx_lpf_bandwidth(dgb_dt, parse_lime_iq_lpf(tx_ch->rfic_lpf_bw));



	if (tx_ch->rfic_lpf_bw <= 40) {
		lms7002m_spi_modify_csr(lms, LMS7002M_TSTIN_TBB, 2);
	}
	else {
		lms7002m_spi_modify_csr(lms, LMS7002M_TSTIN_TBB, 1);
	}
	lms7002m_spi_modify_csr(lms, LMS7002M_LOOPB_TBB, 0);
	



	//// lms7002m_set_tx_lpf(lms, MHZ_TO_HZ(tx_ch->rfic_lpf_bw));
	//lms7002m_set_tx_lpf(lms, MHZ_TO_HZ(tx_ch->rfic_lpf_bw));

	if(tx_ch->path != RFNM_PATH_LOOPBACK) {
		lime0_ant_tx(dgb_dt);
		lime0_disable_all_lna(dgb_dt);
	} else {
		lime0_loopback(dgb_dt);
	}

	rfnm_lime_set_bias_t(dgb_dt, tx_ch->bias_tee);

	rfnm_fe_load_order(dgb_dt, RFNM_LO_LIME0_FA, RFNM_LO_LIME0_ANT, RFNM_LO_LIME0_TX, RFNM_LO_LIME0_END);

	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);

	if(tx_ch->enable == RFNM_CH_RF_ON_TDD) {
		memcpy(&dgb_dt->fe_tdd[RFNM_TX], &dgb_dt->fe, sizeof(struct fe_s));
		if(dgb_dt->rx_s[0]->enable == RFNM_CH_RF_ON_TDD) {
			rfnm_dgb_en_tdd(dgb_dt, tx_ch, dgb_dt->rx_s[0]);
		}
	}

	memcpy(dgb_dt->tx_s[0], dgb_dt->tx_ch[0], sizeof(struct rfnm_api_tx_ch));

	mutex_unlock(&rfnm_lime_apply_lock);
	return 0;

fail:
	mutex_unlock(&rfnm_lime_apply_lock);
	return -ecode;

}

int rfnm_rx_ch_set(struct rfnm_dgb *dgb_dt, struct rfnm_api_rx_ch * rx_ch) {
	lime_Result ret;
	rfnm_api_failcode ecode = RFNM_API_OK;
	struct lms7002m_context *lms;
	ktime_t t0, t_fe, t_sx, t_rb, t_lpf, t_end;
	int want_path;
	lms = dgb_dt->priv_drv;
	uint64_t freq = rx_ch->freq / (MHZ_TO_HZ(1));

	t0 = ktime_get();

	mutex_lock(&rfnm_lime_apply_lock);

	// CHIP-INTERNAL LOOPBACK (TX QEC monitor): RX path LOOPBACK while
	// the TX is RF-live means "observe my own transmitter WITHOUT touching it" - the
	// LMS RFE LB input taps the TXPAD output (non-diverting, proven on hardware: a
	// cross-board witness keeps hearing the TX with the tap open). In this mode the RX
	// apply must not write ANY FE latch (the TX owns the fabric): LMS-side work only.
	// With TX off (or TX itself in LOOPBACK) the legacy FE loopback pairing stands
	// (the factory-cal path: diverting, PAs parked).
	int tx_live = dgb_dt->tx_s[0] && dgb_dt->tx_s[0]->enable == RFNM_CH_RF_ON &&
		dgb_dt->tx_s[0]->path != RFNM_PATH_LOOPBACK;
	int chip_lb = (rx_ch->path == RFNM_PATH_LOOPBACK) && tx_live;

	if(!chip_lb) {
	if(rx_ch->fm_notch == RFNM_FM_NOTCH_AUTO) {
		lime0_fm_notch(dgb_dt, 1);
	} else if(rx_ch->fm_notch == RFNM_FM_NOTCH_ON) {
		lime0_fm_notch(dgb_dt, 1);
	} else if(rx_ch->fm_notch == RFNM_FM_NOTCH_OFF) {
		lime0_fm_notch(dgb_dt, 0);
	}


	/*if(freq <= 2) {
		lime0_filter_0_70(dgb_dt);
	} else if(freq <= 12) {
		lime0_filter_2_12(dgb_dt);
	} else */
	if(freq <= 30) {
		//lime0_filter_12_30(dgb_dt);
		lime0_filter_0_70(dgb_dt);
	} else if(freq <= 60) {
		lime0_filter_30_60(dgb_dt);
	} else if(freq <= 120) {
		lime0_filter_60_120(dgb_dt);
		if(rx_ch->fm_notch == RFNM_FM_NOTCH_AUTO) {
			lime0_fm_notch(dgb_dt, 0);
		}
	} else if(freq <= 250) {
		lime0_filter_120_250(dgb_dt);
		if(freq <= 150 && rx_ch->fm_notch == RFNM_FM_NOTCH_AUTO) {
			lime0_fm_notch(dgb_dt, 0);
		}
	} else if(freq <= 480) {
		lime0_filter_250_480(dgb_dt);
	} else if(freq <= 1000) {
		lime0_filter_480_1000(dgb_dt);
	} else {
		if(freq >= 1166 && freq <= 1229) {
			lime0_filter_1166_1229(dgb_dt);
		} else if(freq >= 1574 && freq <= 1605) {
			lime0_filter_1574_1605(dgb_dt);
		} else if(freq >= 1805 && freq <= 2250) { // 2250? test
			lime0_filter_1805_2200(dgb_dt);
		} else if(freq >= 2250 && freq <= 2700) {
			lime0_filter_2300_2690(dgb_dt);
		} else {
			lime0_filter_950_4000(dgb_dt);
		}
	}

	}

	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);

	if(chip_lb) {
		// LB index follows the TX band (band_trf 2 below 2350 MHz pairs LB2, band 1
		// above pairs LB1); set_path_rfe(LB*) powers the loopback buffer, opens the
		// input switches, parks the LNA and sets EN_LOOPB_TXPAD_TRF in one call.
		want_path = (dgb_dt->tx_s[0]->freq / MHZ_TO_HZ(1)) < 2350 ?
			LMS7002M_PATH_RFE_LB2 : LMS7002M_PATH_RFE_LB1;
	} else if(freq <= 60) {
		want_path = LMS7002M_PATH_RFE_LNAL;
	} else if(freq <= 1400) {
		want_path = LMS7002M_PATH_RFE_LNAW;
		rfnm_fe_srb(dgb_dt, RFNM_LIME0_LRIM, 0);
	} else {
		want_path = LMS7002M_PATH_RFE_LNAH;
		rfnm_fe_srb(dgb_dt, RFNM_LIME0_LRIM, 1);
	}

#if RFNM_LIME_SX_FAST_TUNE
	// set_path_rfe is 18 serialized SPI transactions (~150-200us) and depends only on the band
	if(want_path != rx_last_path_rfe) {
		lms7002m_set_path_rfe(lms, want_path);
		rx_last_path_rfe = want_path;
	}
#else
	lms7002m_set_path_rfe(lms, want_path);
#endif

	if(chip_lb) {
		// capture level management: scale the loopback buffer gain from the dBm
		// engine's last prediction so the tap lands ~-26 dBFS peak at PGA 0 (the
		// requested rx gain rides the PGA on top of this, +0..12 dB - see
		// lime0_set_rx_gain; never rails, never starves - the stream-collapse
		// guard). Rungs measured on hardware (peak dBFS at PGA 0): dbm+10/g0
		// -33, +5/g1 -33, -5/g3 -26 - the old ladder ran one rung too cold
		// everywhere and the monitor view starved. Steps are coarse and nonuniform
		// (g0->1 ~ +7 dB, g1->3 ~ +21 dB).
		int g = 6;
		if(txdbm_last_pred_cdbm != INT_MIN) {
			if(txdbm_last_pred_cdbm >= 1000) { g = 1; }
			else if(txdbm_last_pred_cdbm >= 0) { g = 3; }
			else if(txdbm_last_pred_cdbm >= -1000) { g = 6; }
			else if(txdbm_last_pred_cdbm >= -2000) { g = 10; }
			else { g = 13; }
		}
		lms7002m_spi_modify_csr(lms, LMS7002M_G_RXLOOPB_RFE, g);
	}

	if(freq < 30) {
		freq = 30;
	}

	t_fe = ktime_get();

	if(rfnm_lime_sx_needs_tune(false, rx_ch->freq)) {
		ret = rfnm_lime_tune_sx(lms, false, rx_ch->freq);
		if(ret) {
			printk("Tuning failed!\n");
			ecode = RFNM_API_TUNE_FAIL;
			goto fail;
		}

		t_sx = ktime_get();

#if RFNM_LIME_SX_TIMING
		printk("%d - Actual RX LO freq %llu Hz\n", ret, lms7002m_get_frequency_sx(lms, false));
#endif
	} else {
		t_sx = ktime_get();
	}

	t_rb = ktime_get();

#if RFNM_LIME_SX_FAST_TUNE
	if(rx_ch->rfic_lpf_bw != dgb_dt->rx_s[0]->rfic_lpf_bw) {
		lms7002m_set_rx_lpf(lms, MHZ_TO_HZ(rx_ch->rfic_lpf_bw));
	}
#else
	lms7002m_set_rx_lpf(lms, MHZ_TO_HZ(rx_ch->rfic_lpf_bw));
#endif

	if(rx_ch->rfic_dc_q || rx_ch->rfic_dc_i) {
		/* The ONE public RFIC IQ correction (wire api, logical codes +-126) fans out over
		 * this chip's two analog knobs - the silicon inventory is invisible above this
		 * line, like the LNA/PGA gain split. Per lane: |code| <= 63 lands on DCOFF 0x010E
		 * and the excess spills into the DC-cal DACs 0x05C7/8 (saturating). DCOFF carries
		 * the board I/Q lane crossing (the chip's Q field serves board I, plant-verified
		 * ~-9 counts/code at PGA 0); the spill goes chip-native - its lane coupling is
		 * clock-plan-dependent (measured fully-crossed at native, diagonal at 61.44M), so
		 * no static mapping is correct and the measured loops above the wire api close
		 * whatever residual the approximation leaves. Every nonzero apply rewrites BOTH
		 * knobs deterministically (a debug-door DAC poke does not survive a client apply;
		 * the all-zero pair still skip-writes = leaves standing state alone). */
		int li = rx_ch->rfic_dc_i;
		int lq = rx_ch->rfic_dc_q;
		int di, dq;
		int16_t ci, cq;

		if(li > 126 || li < -126 || lq > 126 || lq < -126) {
			printk("rfic_dc out of range [-126 126], clamping\n");
			li = li > 126 ? 126 : (li < -126 ? -126 : li);
			lq = lq > 126 ? 126 : (lq < -126 ? -126 : lq);
		}
		di = li > 63 ? 63 : (li < -63 ? -63 : li);
		dq = lq > 63 ? 63 : (lq < -63 ? -63 : lq);

		/* Knob priority (forensically established): the DC-cal trim
		 * DACs are the PRIMARY knob - they acted in every session the geometry study and
		 * the dead-session discriminator threw at them - while DCOFF 0x010E is the
		 * mechanism the per-session bring-up lottery randomly deadens (+50 DAC codes =
		 * -872 counts in the same live session where +50 DCOFF codes moved NOTHING).
		 * So: |code| <= 63 lands on the DAC pairs of BOTH channels (the board digitizes
		 * one rail from EACH LMS channel and the session picks which - the AB rule that
		 * already governs the PGA), and the spill goes to DCOFF A+B as best-effort.
		 * The measured loops above the wire own the per-session lane mapping. */
		rfnm_lime_rx_dc_dac_set_locked(lms, di, dq);
		ci = li - di;
		cq = lq - dq;
		if(cq < 0) {
			cq = -cq;
			cq |= (1 << 6);
		}
		if(ci < 0) {
			ci = -ci;
			ci |= (1 << 6);
		}
		lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
		lms7002m_set_dc_offset(lms, false, cq, ci);
		lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_B);
		lms7002m_set_dc_offset(lms, false, cq, ci);
		lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
		rx_dc_trim_hz = rx_ch->freq;	// DC guard freshness
	}

	t_lpf = ktime_get();

	lime0_set_rx_gain(dgb_dt, rx_ch);

	// delta-aware TX-chain parking: only park the PA when TX is actually off, or when
	// building the RX TDD half-profile (which must never carry PA bias). The old
	// unconditional kill yanked the PA mid-transmission on any RX re-apply during
	// concurrent TX+RX - including the rfnm_agc gain resteps that ride this path.
	if(rx_ch->path == RFNM_PATH_LOOPBACK) {
		if(!tx_live) {
			lime0_disable_all_pa(dgb_dt);	// legacy FE-loopback pairing (factory cal)
		}
		// tx_live: chip-internal loopback - the TX fabric is not ours to touch
	} else if(!dgb_dt->tx_s[0] || dgb_dt->tx_s[0]->enable == RFNM_CH_RF_OFF || rx_ch->enable == RFNM_CH_RF_ON_TDD) {
		lime0_tx_power(dgb_dt, 1000, -100);
		lime0_disable_all_pa(dgb_dt);
	}

	if(!chip_lb) {
		rfnm_lime_set_bias_t(dgb_dt, rx_ch->bias_tee);

		rfnm_fe_load_order(dgb_dt, RFNM_LO_LIME0_TX, RFNM_LO_LIME0_ANT, RFNM_LO_LIME0_FA, RFNM_LO_LIME0_END);

		rfnm_fe_load_latches(dgb_dt);
		rfnm_fe_trigger_latches(dgb_dt);
	}


	if(rx_ch->enable == RFNM_CH_RF_ON_TDD) {
		memcpy(&dgb_dt->fe_tdd[RFNM_RX], &dgb_dt->fe, sizeof(struct fe_s));
		if(dgb_dt->tx_s[0] && dgb_dt->tx_s[0]->enable == RFNM_CH_RF_ON_TDD) {
			rfnm_dgb_en_tdd(dgb_dt, dgb_dt->tx_s[0], rx_ch);
		}
	}

	memcpy(dgb_dt->rx_s[0], dgb_dt->rx_ch[0], sizeof(struct rfnm_api_rx_ch));

	t_end = ktime_get();
#if RFNM_LIME_SX_TIMING
	printk("rfnm_lime: rx_set timing fe=%lld sx=%lld rb=%lld lpf=%lld tail=%lld total=%lld us mode=%c\n",
		ktime_us_delta(t_fe, t0), ktime_us_delta(t_sx, t_fe), ktime_us_delta(t_rb, t_sx),
		ktime_us_delta(t_lpf, t_rb), ktime_us_delta(t_end, t_lpf), ktime_us_delta(t_end, t0), sx_last_mode);
#else
	(void)t0; (void)t_fe; (void)t_sx; (void)t_rb; (void)t_lpf; (void)t_end;
#endif

	mutex_unlock(&rfnm_lime_apply_lock);
	atomic_inc(&rfnm_rx_apply_epoch);	// rfnm_qec re-arms its one-shot null + settle
	return 0;

fail:
	mutex_unlock(&rfnm_lime_apply_lock);
	return -ecode;
}

static void rfnm_lime_log_callback(int level, const char* message, void* handle) {
	(void) handle;

	if(level <= lime_LogLevel_Error) { //lime_LogLevel_Verbose
		printk(message);
	}
	
}

static lime_Result rfnm_lime_cgen_frequency_changed(void* handle) {
	(void) handle;
	return lime_Result_Success;
}

/* TX LO-leak trim via the LMS7002M analog DC-cal DACs (DC_TXAI/DC_TXAQ at 0x5C3/0x5C4).
 * The TX baseband path is AC-coupled, so digital DC injection never reaches the carrier;
 * these DACs are the only knob that nulls TX LO leakage. Interface mirrors rfnm_qec:
 * set tx_dc_i/tx_dc_q, then echo 1 > tx_dc_write to apply. */
static struct lms7002m_context *rfnm_lime_txcal_lms;
module_param(tx_dc_i, int, 0644);
module_param(tx_dc_q, int, 0644);

/* duplicated from LimeSuiteNG calibrations.c lms7002m_write_analog_dc, which is static there */
static void rfnm_lime_write_analog_dc(struct lms7002m_context *lms, uint16_t addr, int16_t value) {
	const uint16_t mask = addr < 0x05C7 ? 0x03FF : 0x003F;
	int16_t regValue = 0;
	if(value < 0) {
		regValue |= (mask + 1);
		regValue |= (abs(value + mask) & mask);
	} else {
		regValue |= (abs(value + mask + 1) & mask);
	}
	lms7002m_spi_write(lms, addr, regValue);
	lms7002m_spi_write(lms, addr, regValue | 0x8000);
}

static int rfnm_lime_tx_dc_apply(const char *val, const struct kernel_param *kp) {
	if(!rfnm_lime_txcal_lms) {
		return -ENODEV;
	}
	if(tx_dc_i < -1024 || tx_dc_i > 1023 || tx_dc_q < -1024 || tx_dc_q > 1023) {
		return -EINVAL;
	}
	lms7002m_spi_modify_csr(rfnm_lime_txcal_lms, LMS7002M_DCMODE, 1);
	lms7002m_spi_modify_csr(rfnm_lime_txcal_lms, LMS7002M_PD_DCDAC_TXA, 0);
	lms7002m_spi_modify_csr(rfnm_lime_txcal_lms, LMS7002M_PD_DCCMP_TXA, 0);
	rfnm_lime_write_analog_dc(rfnm_lime_txcal_lms, 0x05C3, tx_dc_i);
	rfnm_lime_write_analog_dc(rfnm_lime_txcal_lms, 0x05C4, tx_dc_q);
	return 0;
}
static const struct kernel_param_ops rfnm_lime_tx_dc_write_ops = {
	.set = rfnm_lime_tx_dc_apply,
};
module_param_cb(tx_dc_write, &rfnm_lime_tx_dc_write_ops, NULL, 0200);

/* RX DC-cal DACs (DC_RXAI/DC_RXAQ at 0x05C7/0x05C8, +-63) - the second analog RX DC
 * knob, stacking with DCOFF 0x010E (which sits at 61/63 codes at 2450 MHz and runs
 * ~50 counts short of the 3.2-3.5 GHz plant). The DCMODE/DCCMP SAR is disconnected
 * (by design decision) but manual DAC writes inject fine: measured +32 codes on DC_RXAI =
 * ~+1031 observed-I counts at PGA 12 WITH ~50% cross-lane coupling - which is why the
 * consumer (rfnm_qec's null stage 2) measures the 2x2 knob->lane response live instead
 * of trusting any per-lane assumption. Chip-native lane order here, no board crossing:
 * the measured response absorbs the mapping. */
static void rfnm_lime_rx_dc_dac_set_locked(struct lms7002m_context *lms, int dc_i, int dc_q) {
	/* caller holds rfnm_lime_apply_lock. BOTH channels' DAC pairs (absolute registers,
	 * not MAC-banked): the session pairing draws rails from either LMS channel. */
	lms7002m_spi_modify_csr(lms, LMS7002M_DCMODE, 1);
	lms7002m_spi_modify_csr(lms, LMS7002M_PD_DCDAC_RXA, 0);
	lms7002m_spi_modify_csr(lms, LMS7002M_PD_DCDAC_RXB, 0);
	rfnm_lime_write_analog_dc(lms, 0x05C7, dc_i);
	rfnm_lime_write_analog_dc(lms, 0x05C8, dc_q);
	rfnm_lime_write_analog_dc(lms, 0x05C9, dc_i);
	rfnm_lime_write_analog_dc(lms, 0x05CA, dc_q);
}

/* debug door: set rx_dc_dac_i/rx_dc_dac_q, then echo 1 > rx_dc_dac_write. Debug only -
 * any client apply carrying nonzero rfic_dc rewrites both knobs deterministically. */
static int rx_dc_dac_i, rx_dc_dac_q;
module_param(rx_dc_dac_i, int, 0644);
module_param(rx_dc_dac_q, int, 0644);
static int rfnm_lime_rx_dc_dac_apply(const char *val, const struct kernel_param *kp) {
	if(!rfnm_lime_txcal_lms) {
		return -ENODEV;
	}
	if(rx_dc_dac_i < -63 || rx_dc_dac_i > 63 || rx_dc_dac_q < -63 || rx_dc_dac_q > 63) {
		return -EINVAL;
	}
	mutex_lock(&rfnm_lime_apply_lock);
	lms7002m_set_active_channel(rfnm_lime_txcal_lms, LMS7002M_CHANNEL_A);
	rfnm_lime_rx_dc_dac_set_locked(rfnm_lime_txcal_lms, rx_dc_dac_i, rx_dc_dac_q);
	mutex_unlock(&rfnm_lime_apply_lock);
	return 0;
}
static const struct kernel_param_ops rfnm_lime_rx_dc_dac_write_ops = {
	.set = rfnm_lime_rx_dc_dac_apply,
};
module_param_cb(rx_dc_dac_write, &rfnm_lime_rx_dc_dac_write_ops, NULL, 0200);

/* r8 forensics (the duplex-TX-dead hunt): echo 1 > lms_dump prints the LMS7002M
 * register file to dmesg. RAW SPI reads only - never read_csr with side effects and
 * never the analog-DC readback (it MUTATES, r5 trap). Under the apply lock; channel
 * file dumped at MAC=A, the SX file at SXR then SXT, MAC restored to A after. */
static void rfnm_lime_dump_range(struct lms7002m_context *lms, const char *tag, uint16_t a, uint16_t b) {
	char line[100];
	int n = 0;
	uint16_t addr;

	for(addr = a; addr <= b; addr++) {
		n += scnprintf(line + n, sizeof(line) - n, "%04x=%04x ", addr, lms7002m_spi_read(lms, addr));
		if(n > 78) {
			printk("lms[%s]: %s\n", tag, line);
			n = 0;
		}
	}
	if(n) {
		printk("lms[%s]: %s\n", tag, line);
	}
}
static int rfnm_lime_lms_dump(const char *val, const struct kernel_param *kp) {
	struct lms7002m_context *lms = rfnm_lime_txcal_lms;

	if(!lms) {
		return -ENODEV;
	}
	mutex_lock(&rfnm_lime_apply_lock);
	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
	rfnm_lime_dump_range(lms, "cfg", 0x0020, 0x002F);
	rfnm_lime_dump_range(lms, "afe", 0x0081, 0x00A6);
	rfnm_lime_dump_range(lms, "chA", 0x0100, 0x012B);
	rfnm_lime_dump_range(lms, "dc", 0x05C0, 0x05CC);
	rfnm_lime_dump_range(lms, "tsp", 0x0200, 0x020C);
	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_SXR);
	rfnm_lime_dump_range(lms, "sxr", 0x011C, 0x0124);
	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_SXT);
	rfnm_lime_dump_range(lms, "sxt", 0x011C, 0x0124);
	lms7002m_set_active_channel(lms, LMS7002M_CHANNEL_A);
	mutex_unlock(&rfnm_lime_apply_lock);
	return 0;
}
static const struct kernel_param_ops rfnm_lime_lms_dump_ops = {
	.set = rfnm_lime_lms_dump,
};
module_param_cb(lms_dump, &rfnm_lime_lms_dump_ops, NULL, 0200);

/* RBB LPF corner calibration via the LMS7002M on-chip MCU (rfnm_lime_mcu.c):
 * echo <bw_mhz> > rx_lpf_cal runs TuneRxFilter for that RF bandwidth, then invalidates
 * every fast-tune cache and re-applies the RX channel (the cal retunes CGEN/SX
 * internally - trusting the parked caches after it would skip a needed LO retune).
 * Result in rx_lpf_cal_status: 0 ok, -errno otherwise. Manual trigger only for now;
 * per-bw persistence policy comes after the corner proof. */
static struct rfnm_dgb *rfnm_lime_cal_dgb;
static int rx_lpf_cal_status = -1;
module_param(rx_lpf_cal_status, int, 0444);

static int rfnm_lime_rx_lpf_cal_run(const char *val, const struct kernel_param *kp) {
	unsigned int bw_mhz;
	int ret = kstrtouint(val, 0, &bw_mhz);

	if(ret) {
		return ret;
	}
	if(!rfnm_lime_txcal_lms || !rfnm_lime_cal_dgb) {
		rx_lpf_cal_status = -ENODEV;
		return 0;
	}

	mutex_lock(&rfnm_lime_apply_lock);
	ret = rfnm_lime_mcu_rx_lpf_cal(rfnm_lime_txcal_lms, bw_mhz * 1000000u, LMS_REF_FREQ);
	// the MCU cal owns the whole chip while it runs: every last-value skip is now stale
	sx_last_hz[0] = 0;
	sx_last_hz[1] = 0;
#if RFNM_LIME_SX_FAST_TUNE
	rx_last_path_rfe = -1;
	rx_last_lna_db = S32_MIN;
#endif
	rx_last_pga_db = S32_MIN;
	mutex_unlock(&rfnm_lime_apply_lock);

	if(!ret) {
		int r = rfnm_rx_ch_set(rfnm_lime_cal_dgb, rfnm_lime_cal_dgb->rx_ch[0]);
		if(r) {
			printk("rfnm_lime: rx re-apply after lpf cal failed: %d\n", r);
			ret = -EIO;
		}
	}
	rx_lpf_cal_status = ret;
	return 0;
}
static const struct kernel_param_ops rfnm_lime_rx_lpf_cal_ops = {
	.set = rfnm_lime_rx_lpf_cal_run,
};
module_param_cb(rx_lpf_cal, &rfnm_lime_rx_lpf_cal_ops, NULL, 0200);

/* debug access to LMS7002M SPI registers: echo $((addr<<16 | val)) > lms_poke;
 * echo addr > lms_peek prints the register to dmesg */
static int rfnm_lime_lms_poke(const char *val, const struct kernel_param *kp) {
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);
	if(ret) {
		return ret;
	}
	if(!rfnm_lime_txcal_lms) {
		return -ENODEV;
	}
	lms7002m_spi_write(rfnm_lime_txcal_lms, (v >> 16) & 0x7FFF, v & 0xFFFF);
	return 0;
}
static const struct kernel_param_ops rfnm_lime_lms_poke_ops = {
	.set = rfnm_lime_lms_poke,
};
module_param_cb(lms_poke, &rfnm_lime_lms_poke_ops, NULL, 0200);

static int rfnm_lime_lms_peek(const char *val, const struct kernel_param *kp) {
	unsigned int addr;
	int ret = kstrtouint(val, 0, &addr);
	if(ret) {
		return ret;
	}
	if(!rfnm_lime_txcal_lms) {
		return -ENODEV;
	}
	printk("rfnm_lime: lms reg 0x%04x = 0x%04x\n", addr & 0x7FFF, lms7002m_spi_read(rfnm_lime_txcal_lms, addr & 0x7FFF));
	return 0;
}
static const struct kernel_param_ops rfnm_lime_lms_peek_ops = {
	.set = rfnm_lime_lms_peek,
};
module_param_cb(lms_peek, &rfnm_lime_lms_peek_ops, NULL, 0200);

/* TEMP debug knob (revert): force the FE ANT latch (latch 1) to an exact
 * bit pattern and reclock it immediately - physical-truth sweep for the dead TX path.
 * echo 0xNN > fe_l1_force */
static int rfnm_lime_fe_l1_force(const char *val, const struct kernel_param *kp) {
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);
	struct rfnm_dgb *dgb_dt = rfnm_lime_cal_dgb;

	if(ret) {
		return ret;
	}
	if(!dgb_dt) {
		return -ENODEV;
	}
	mutex_lock(&rfnm_lime_apply_lock);
	dgb_dt->fe.latch_val[0] = v & 0xFF;
	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);
	mutex_unlock(&rfnm_lime_apply_lock);
	printk("rfnm_lime: FE latch1 forced to 0x%02x\n", v & 0xFF);
	return 0;
}
static const struct kernel_param_ops rfnm_lime_fe_l1_force_ops = {
	.set = rfnm_lime_fe_l1_force,
};
module_param_cb(fe_l1_force, &rfnm_lime_fe_l1_force_ops, NULL, 0200);

/* TEMP debug knob (revert): same for the TX/PA/attn latch (latch 3, 16 bit) */
static int rfnm_lime_fe_l3_force(const char *val, const struct kernel_param *kp) {
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);
	struct rfnm_dgb *dgb_dt = rfnm_lime_cal_dgb;

	if(ret) {
		return ret;
	}
	if(!dgb_dt) {
		return -ENODEV;
	}
	mutex_lock(&rfnm_lime_apply_lock);
	dgb_dt->fe.latch_val[2] = v & 0xFFFF;
	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);
	mutex_unlock(&rfnm_lime_apply_lock);
	printk("rfnm_lime: FE latch3 forced to 0x%04x\n", v & 0xFFFF);
	return 0;
}
static const struct kernel_param_ops rfnm_lime_fe_l3_force_ops = {
	.set = rfnm_lime_fe_l3_force,
};
module_param_cb(fe_l3_force, &rfnm_lime_fe_l3_force_ops, NULL, 0200);

static int rfnm_lime_probe(struct spi_device *spi)
{
	struct rfnm_bootconfig *cfg;
	struct rfnm_eeprom_data *eeprom_data;
	cfg = memremap(RFNM_BOOTCONFIG_PHYADDR, SZ_4M, MEMREMAP_WB);

	struct spi_master *spi_master;
	spi_master = spi->master;
	int dgb_id = spi_master->bus_num - 1;

	if(	cfg->daughterboard_present[dgb_id] != RFNM_DAUGHTERBOARD_PRESENT ||
		cfg->daughterboard_eeprom[dgb_id].board_id != RFNM_DAUGHTERBOARD_LIME) {
		memunmap(cfg);
		return -ENODEV;
	}

	printk("RFNM: Loading Lime driver for daughterboard at slot %d\n", dgb_id);

	rfnm_gpio_output(dgb_id, RFNM_DGB_GPIO3_22); // 1.4V LMS
	rfnm_gpio_output(dgb_id, RFNM_DGB_GPIO4_8); // 1.25V LMS

	rfnm_gpio_output(dgb_id, RFNM_GPIO4_RB_3V3); 
	//rfnm_gpio_output(dgb_id, RFNM_GPIO4_RB_1V8_SHARED); 
	// 


	rfnm_gpio_clear(dgb_id, RFNM_GPIO4_RB_3V3); 
	rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO3_22); // 1.4V LMS
	rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_8); // 1.25V LMS
	//rfnm_gpio_clear(dgb_id, RFNM_GPIO4_RB_1V8_SHARED); 

	msleep(100);

	rfnm_gpio_set(dgb_id, RFNM_GPIO4_RB_3V3); 
	rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO3_22); // 1.4V LMS
	rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_8); // 1.25V LMS
	//rfnm_gpio_set(dgb_id, RFNM_GPIO4_RB_1V8_SHARED); 

	msleep(100);

 	struct device *dev = &spi->dev;
	struct rfnm_dgb *dgb_dt;

	const struct spi_device_id *id = spi_get_device_id(spi);
	int i, ret;

	dgb_dt = devm_kzalloc(dev, sizeof(struct rfnm_dgb), GFP_KERNEL);
	if(!dgb_dt) {
		return -ENOMEM;
	}

	dgb_dt->dgb_id = dgb_id;

	dgb_dt->rx_ch_set = rfnm_rx_ch_set;
	dgb_dt->rx_ch_get = rfnm_rx_ch_get;
	dgb_dt->tx_ch_set = rfnm_tx_ch_set;
	dgb_dt->tx_ch_get = rfnm_tx_ch_get;
	spi_set_drvdata(spi, dgb_dt);

	spi->max_speed_hz = 10000000;
	spi->bits_per_word = 32;
	spi->mode = 0;

	rfnm_fe_generic_init(dgb_dt, RFNM_LIME0_NUM_LATCHES);

	lms7002m_hooks lms_hooks;

	lms_hooks.log = rfnm_lime_log_callback;
	lms_hooks.spi16_transact = lms_spi16_transact;
	lms_hooks.spi16_userData = spi;
	lms_hooks.on_cgen_frequency_changed = rfnm_lime_cgen_frequency_changed;

	struct lms7002m_context *lms;
	lms = lms7002m_create(&lms_hooks);
	if (lms == NULL) {
		return -1;
	}
	dgb_dt->priv_drv = lms;

	const uint32_t reset_cmd = ((0x8000 | 0x0020) << 16) | 0x0000;
	int spi_result = spi_write(spi, &reset_cmd, 4);
	const uint32_t release_reset_cmd = ((0x8000 | 0x0020) << 16) | 0xFFFD;
	spi_result = spi_write(spi, &release_reset_cmd, 4);
	// Both the new and old LimeSuite drivers do this on a reset. ??
	const uint32_t write_mimo_ch_b_cmd = ((0x8000 | 0x002E) << 16) | 0x0000;
	spi_result = spi_write(spi, &write_mimo_ch_b_cmd, 4);

	// set 4-wire spi before reading back
	// TODO: is this really necessary?
	const uint32_t set_4wire_spi_cmd = ((0x8000 | 0x0021) << 16) | 0x0E9F;
	spi_result = spi_write(spi, &set_4wire_spi_cmd, 4);

	//read info register
	const uint32_t read_lms_info_reg_cmd = 0x002f << 16;
	uint32_t read_lms_info_reg_value;
	spi_result = lms_spi16_transact(&read_lms_info_reg_cmd, &read_lms_info_reg_value, 1, spi);
	printk("ver 0x%x, rev 0x%x, mask 0x%x\n",
		(read_lms_info_reg_value >> 11) & 0x1f,
		(read_lms_info_reg_value >> 6) & 0x1f,
		(read_lms_info_reg_value >> 0) & 0x3f);

	if(!((read_lms_info_reg_value >> 6) & 0x1f)) {
		printk("LMS Not Found!");
		lms7002m_destroy(lms);
		return -1;
	}

	// An attempt at doing a bulk write of all initial registers was not
	// successful, despite the LMS7002M datasheet saying that: "multiple
	// read/write is possible by repeating the instruction/data sequence while
	// keeping SEN low". As such, we'll do individual SPI transfers for now. It
	// seems the SEN toggle between some or all writes is important.
	for(i = 0; i < RFNM_LMS_REGS_CNT_ALL; i++) {
		uint32_t spi_cmd = ((0x8000 | rfnm_lms_init_regs_all[i][0]) << 16) | rfnm_lms_init_regs_all[i][1];
		spi_write(spi, &spi_cmd, 4);
	}

	//turn the clocks on
	lime_Result lms_ret = lms7002m_set_reference_clock(lms, LMS_REF_FREQ);
	if (ret != 0) {
		printk("provided reference clock invalid %d\n", ret);
		return -1;
	}

	// rx path is always enabled
	lms7002m_enable_channel(lms, false, LMS7002M_CHANNEL_A, true);

	// TODO: do we need to do something like this still?
	// tune tx PLL to a high frequency out of the way to avoid spurs... meh.
	rfnm_lime_tune_sx(lms, true, MHZ_TO_HZ(3000));
	// then disable some of it
	lms7002m_enable_channel(lms, true, LMS7002M_CHANNEL_A, false);
	// SXT is gated by EN_G while TX is off; make the first real TX apply tune instead of skipping
	sx_last_hz[1] = 0;


	//lime0_tx_power(dgb_dt, 1000, 100000);
//	lime0_disable_all_pa(dgb_dt);
/*
	rfnm_fe_srb(dgb_dt, RFNM_LIME0_TL1O, 0);



	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);


	rfnm_fe_srb(dgb_dt, RFNM_LIME0_TL1O, 1);

	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);
*/

	rfnm_lime_set_bias_t(dgb_dt, RFNM_BIAS_TEE_OFF);
	rfnm_fe_load_latches(dgb_dt);
	rfnm_fe_trigger_latches(dgb_dt);

	printk("RFNM: Lime daughterboard initialized\n");

	struct rfnm_api_tx_ch *tx_ch, *tx_s;
	struct rfnm_api_rx_ch *rx_ch, *rx_s;

	if (dgb_id == 0) {
		tx_ch = devm_kzalloc(dev, sizeof(struct rfnm_api_tx_ch), GFP_KERNEL);
		tx_s = devm_kzalloc(dev, sizeof(struct rfnm_api_tx_ch), GFP_KERNEL);
		if (!tx_ch || !tx_s) {
			return -ENOMEM;
		}
	}
	rx_ch = devm_kzalloc(dev, sizeof(struct rfnm_api_rx_ch), GFP_KERNEL);
	rx_s = devm_kzalloc(dev, sizeof(struct rfnm_api_rx_ch), GFP_KERNEL);

	if(!rx_ch || !rx_s) {
		return -ENOMEM;
	}

	if (dgb_id == 0) {
		tx_ch->freq_max = MHZ_TO_HZ(3500);
		tx_ch->freq_min = MHZ_TO_HZ(10);
		tx_ch->path_preferred = RFNM_PATH_SMA_A;
		tx_ch->path_possible[0] = RFNM_PATH_SMA_A;
		tx_ch->path_possible[1] = RFNM_PATH_NULL;
		rfnm_lime_txdbm_load(&spi->dev, cfg->daughterboard_eeprom[dgb_id].serial_number);
		if(txdbm_n) {
			/* map present: the power knob IS requested dBm at the SMA (by
			 * contract); range = the map's honest envelope with margin */
			tx_ch->power_range.min = txdbm_amin / 100;
			tx_ch->power_range.max = txdbm_amax / 100;
		} else {
			tx_ch->power_range.min = -60;
			// Raw-power mapping: the FE ladder tops out at power 40 (PA1+PA2, TXPAD at loss 0 = the
			// factory-mint drive); 41..52 air identically flat (positive residue has nothing to
			// add). Max stays 52 so legacy consumers (p50 raw-power probes) validate instead of
			// getting the whole apply rejected.
			tx_ch->power_range.max = 52;
		}
		tx_ch->dac_id = 0;
		rfnm_dgb_reg_tx_ch(dgb_dt, tx_ch, tx_s);
	}

	rx_ch->freq_max = MHZ_TO_HZ(3500);
	rx_ch->freq_min = MHZ_TO_HZ(10);
	rx_ch->path_preferred = RFNM_PATH_SMA_A;
	rx_ch->path_possible[0] = RFNM_PATH_SMA_A;
	rx_ch->path_possible[1] = RFNM_PATH_EMBED_ANT;
	rx_ch->path_possible[2] = RFNM_PATH_TERMINATED;
	rx_ch->path_possible[3] = RFNM_PATH_NULL;
	rx_ch->gain_range.min = -24;
	rx_ch->gain_range.max = 30;
	rx_ch->adc_id = 0;
	rfnm_dgb_reg_rx_ch(dgb_dt, rx_ch, rx_s);

	dgb_dt->dac_ifs = 0x7;
	dgb_dt->dac_iqswap[0] = 0;
	dgb_dt->dac_iqswap[1] = 0;
	dgb_dt->adc_iqswap[0] = 0;   /* the GPOUT lane-swap this declares is inert for Lime slot 0
	                              * (verified live 2026-07-03: value 0 or 1, spectrum unchanged);
	                              * the conjugate is corrected in the la9310rfnm data path via
	                              * rfnm_sw_iqswap below (applies to CS16 local + packed-12 USB) */
	dgb_dt->adc_iqswap[1] = 0;
	rfnm_dgb_reg(dgb_dt);


	if(dgb_id == 0) {
		rfnm_lime_txcal_lms = lms;
		rfnm_lime_cal_dgb = dgb_dt;
	}

	return 0;
}

static void rfnm_lime_remove(struct spi_device *spi)
{
	struct rfnm_dgb *dgb_dt;
	dgb_dt = spi_get_drvdata(spi);

	if(dgb_dt->priv_drv == rfnm_lime_txcal_lms) {
		rfnm_lime_txcal_lms = NULL;
	}

	lms7002m_destroy(dgb_dt->priv_drv);

	rfnm_dgb_unreg(dgb_dt);

	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO3_22); // 1.4V LMS
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_8); // 1.25V LMS
}

static const struct spi_device_id rfnm_lime_ids[] = {
	{ "rfnm,daughterboard" },
	{},
};
MODULE_DEVICE_TABLE(spi, rfnm_lime_ids);


static const struct of_device_id rfnm_lime_match[] = {
	{ .compatible = "rfnm,daughterboard" },
	{},
};
MODULE_DEVICE_TABLE(of, rfnm_lime_match);

static struct spi_driver rfnm_lime_spi_driver = {
	.driver = {
		.name = "rfnm_lime",
		.of_match_table = rfnm_lime_match,
	},
	.probe = rfnm_lime_probe,
	.remove = rfnm_lime_remove,
	.id_table = rfnm_lime_ids,
};

module_spi_driver(rfnm_lime_spi_driver);


MODULE_PARM_DESC(device, "RFNM Lime Daughterboard Driver");

MODULE_LICENSE("GPL");
