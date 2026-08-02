// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM
//
// LMS7002M on-chip MCU control for calibration procedures. The chip carries an 8051 MCU
// that runs Lime's calibration image (DC/IQ/filter tuning, single image ID 0x05); the host
// only uploads the 16 KB program into MCU SRAM over SPI, passes parameters, triggers a
// procedure and polls its status - the whole RSSI/NCO/CGEN dance happens on-chip.
// Protocol ported from LimeSuiteNG MCU_BD.cpp (chip registers 0x0000-0x0006), which is not
// usable piecemeal from there (C++/host-side). Timing mirrors the desktop flow with margin:
// the desktop's per-transaction USB latency is an implicit settle that our ~10 MHz direct
// SPI does not have, so the parameter-latch procedures get explicit sleeps.
//
// Caller contract (rfnm_lime_m.c): hold the apply mutex for the whole call - while 0x0006=1
// the MCU owns the chip and any concurrent host SPI corrupts the procedure - and invalidate
// every fast-tune cache afterwards (the cal retunes CGEN/SX internally).

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/errno.h>

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "../../LimeSuiteNG/embedded/lms7002m/spi.h"

#include "rfnm_lime_mcu.h"
#include "rfnm_lime_mcu_fw.h"

// poll 0x0001 until the MCU posts a status (0xFF = still running/idle marker), then hand
// SPI control back to the host. Returns status & 0x7F (0 = success) or -ETIMEDOUT.
static int rfnm_lime_mcu_wait(struct lms7002m_context *lms, uint32_t timeout_ms) {
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	uint16_t v;

	usleep_range(50, 100);
	for(;;) {
		v = lms7002m_spi_read(lms, 0x0001) & 0xFF;
		if(v != 0xFF) {
			break;
		}
		if(time_after(jiffies, deadline)) {
			lms7002m_spi_write(lms, 0x0006, 0);
			return -ETIMEDOUT;
		}
		usleep_range(1000, 1500);
	}
	lms7002m_spi_write(lms, 0x0006, 0);
	return v & 0x7F;
}

static void rfnm_lime_mcu_run(struct lms7002m_context *lms, uint8_t id) {
	uint16_t r2;

	lms7002m_spi_write(lms, 0x0006, 1);	// SPI switch: MCU owns the chip
	lms7002m_spi_write(lms, 0x0000, id);
	r2 = lms7002m_spi_read(lms, 0x0002);
	lms7002m_spi_write(lms, 0x0002, r2 & ~0x08);	// pulse interrupt 6
	lms7002m_spi_write(lms, 0x0002, r2 | 0x08);
	lms7002m_spi_write(lms, 0x0002, r2 & ~0x08);
	lms7002m_spi_read(lms, 0x0002);	// MCU stalls until any SPI activity happens
	usleep_range(10, 30);
}

static int rfnm_lime_mcu_read_id(struct lms7002m_context *lms) {
	rfnm_lime_mcu_run(lms, 255);
	return rfnm_lime_mcu_wait(lms, 50);
}

// upload the calibration image into MCU SRAM (skipped when it already answers with the
// image ID). ~256 chunks of 64 bytes through the reg-0x0004 FIFO, gated on 0x0003 bit0
// (write buffer empty); bit6 = programmed.
static int rfnm_lime_mcu_program(struct lms7002m_context *lms) {
	int i, j, w;

	if(rfnm_lime_mcu_read_id(lms) == RFNM_LIME_MCU_CAL_IMAGE_ID) {
		return 0;
	}

	lms7002m_spi_write(lms, 0x0002, 0);	// reset MCU
	lms7002m_spi_write(lms, 0x0002, 2);	// prog mode = SRAM

	for(i = 0; i < RFNM_LIME_MCU_PROGRAM_SIZE; i += 64) {
		for(w = 0; w < 1000; w++) {
			if(lms7002m_spi_read(lms, 0x0003) & 0x01) {
				break;
			}
			usleep_range(20, 50);
		}
		if(w == 1000) {
			printk("rfnm_lime: MCU program FIFO stuck at byte %d\n", i);
			return -ETIMEDOUT;
		}
		for(j = 0; j < 64; j++) {
			lms7002m_spi_write(lms, 0x0004, rfnm_lime_mcu_cal_fw[i + j]);
		}
	}

	for(w = 0; w < 1000; w++) {
		if(lms7002m_spi_read(lms, 0x0003) & 0x40) {
			break;
		}
		usleep_range(50, 100);
	}
	if(w == 1000) {
		printk("rfnm_lime: MCU 'programmed' flag never set\n");
		return -ETIMEDOUT;
	}

	if(rfnm_lime_mcu_read_id(lms) != RFNM_LIME_MCU_CAL_IMAGE_ID) {
		printk("rfnm_lime: MCU image ID readback mismatch after upload\n");
		return -EIO;
	}
	return 0;
}

// pass a frequency parameter (integer MHz byte + fractional kHz as 16 bits, MSB-first
// latch order like the desktop) and latch it with its procedure (4 = ref clock, 3 = bw)
static void rfnm_lime_mcu_set_freq_param(struct lms7002m_context *lms, int is_ref_clk, uint32_t hz) {
	uint16_t r2 = lms7002m_spi_read(lms, 0x0002);
	uint32_t frac_khz = (hz / 1000) % 1000;
	uint8_t in[3] = { (uint8_t)(hz / 1000000), (uint8_t)(frac_khz >> 8), (uint8_t)(frac_khz & 0xFF) };
	int i;

	for(i = 0; i < 3; i++) {
		lms7002m_spi_write(lms, 0x0000, in[2 - i]);
		lms7002m_spi_write(lms, 0x0002, r2 | 0x04);	// pulse interrupt 7
		lms7002m_spi_write(lms, 0x0002, r2 & ~0x04);
		usleep_range(20, 50);
	}
	rfnm_lime_mcu_run(lms, is_ref_clk ? 4 : 3);
	msleep(2);	// the desktop never waits here; give the latch procedure explicit margin
}

// TuneRxFilter port: MCU procedure 5 tunes the RBB LPF RC codes for bw_hz (RF, double-sided)
// against ref_hz. Results land in 0x0112/0x0114-0x0118/0x011A. MCU status: 0 = ok; notable
// codes: 2 CGEN tune failed, 5 loopback signal weak, 8 bw out of range, 9 invalid TIA gain.
int rfnm_lime_mcu_rx_lpf_cal(struct lms7002m_context *lms, uint32_t bw_hz, uint32_t ref_hz) {
	static const uint16_t result_regs[] = { 0x0112, 0x0114, 0x0115, 0x0116, 0x0117, 0x0118, 0x011A };
	int ret, i;

	if(bw_hz < 1400000 || bw_hz > 130000000) {
		return -EINVAL;
	}

	ret = rfnm_lime_mcu_program(lms);
	if(ret) {
		return ret;
	}

	rfnm_lime_mcu_set_freq_param(lms, 1, ref_hz);
	rfnm_lime_mcu_set_freq_param(lms, 0, bw_hz);
	rfnm_lime_mcu_run(lms, 5);
	ret = rfnm_lime_mcu_wait(lms, 2000);
	if(ret < 0) {
		printk("rfnm_lime: rx lpf cal timed out\n");
		return ret;
	}
	if(ret != 0) {
		printk("rfnm_lime: rx lpf cal MCU status %d\n", ret);
		return -EIO;
	}

	for(i = 0; i < ARRAY_SIZE(result_regs); i++) {
		printk("rfnm_lime: rx lpf cal reg 0x%04x = 0x%04x\n", result_regs[i], lms7002m_spi_read(lms, result_regs[i]));
	}
	return 0;
}
