// SPDX-License-Identifier: MPL-2.0
// Copyright (C) 2026 RFNM

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/dma-mapping.h>
#define __RFIC

#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/rfnm-vspa.h>	/* struct rfnm_la9310_status: the anchor-publish wait */

#include <la9310_base.h>
#include <la9310_vspa_registry.h>
//#include "rfnm.h"
//#include "rfnm_callback.h"
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
#include <linux/pm_qos.h>
#include <linux/cpuidle.h>
#include <linux/cpu.h>

// v3 phase B: cpu latency QoS held while any stream is configured. cpu-pd-wait
// idle (1.5 ms exit latency) starves the VSPA's inbound PCIe DDR reads mid-window;
// a 0-latency request bars it while streaming. Serialized: concurrent rx+tx apply
// work items both transition streams, and the earlier unserialized add/remove of
// shared request objects was exactly what oopsed (plist corruption read as a
// framework fault). Process context only.
static DEFINE_MUTEX(rfnm_cpu_qos_lock);
static struct pm_qos_request rfnm_cpu_qos_req;
static bool rfnm_cpu_qos_held;

static void rfnm_cpu_qos_update(int streaming) {
	mutex_lock(&rfnm_cpu_qos_lock);
	if(streaming && !rfnm_cpu_qos_held) {
		cpu_latency_qos_add_request(&rfnm_cpu_qos_req, 0);
		rfnm_cpu_qos_held = true;
	} else if(!streaming && rfnm_cpu_qos_held) {
		cpu_latency_qos_remove_request(&rfnm_cpu_qos_req);
		rfnm_cpu_qos_held = false;
	}
	mutex_unlock(&rfnm_cpu_qos_lock);
}

// The RT command ring producer (option B, rt-cores review 07-18): absolute-tick
// verbs into the M4 rtc engine, which stays sole owner of every comparator arm.
// Ring home = the TCM gap the retired txn ring used, NEW magic; producer code
// lives with the debugfs test cluster below.
#include "rfnm_rtc_ring.h"
// SET_TXN wire kind 4 = the sticky session anchor (librfnm schedule_ctl kind 4).
// Kernel-side only - it never entered the M4 ring; the retired txn header
// carried it and this is its surviving home.
#define RFNM_TXN_ANCHOR		( 4 )
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
#include <linux/i2c.h>
#include <linux/timekeeping.h>
#include <linux/io.h>


//#include "rfnm_types.h"
#include <linux/rfnm-shared.h>
#include <linux/rfnm-gpio.h>

#include "la9310_rfnm.h"
#include "rfnm_lalib.h"

static struct rf_host_stats rfic_stats = {0};
static struct rfdevice *rfdev = NULL;


#define htole32(x) (x)
#define le32toh(x) (x)

#define iowrite32le(v, p) writel_relaxed(htole32((v)), (void __iomem *)(p))
#define ioread32le(p) le32toh(readl_relaxed((void __iomem *)(p)))

// rfdev->hif_p / rfdev->ccsr_p are permanent ioremaps of the LA9310 PCIe window (TCML + CCSR at phys
// 0x18000000+) that bypass the PCI layer entirely. While the link is down (hard reprobe) a CPU access
// through them hard-hangs the SoC - gate every access on the shiva-owned MMIO fence: reads return 0,
// writes are dropped. rf_send_swcmd additionally fails fast with -EBUSY at entry.
#define iowr32(r,v,p) do { if(!rfnm_la9310_mmio_fenced()) { iowrite32le((v),(p)); } } while (0)
#define iord32(r,p) ( rfnm_la9310_mmio_fenced() ? 0 : ioread32le((p)) )

struct i2c_client *si5510_i2c_client;
struct device *si5510_i2c_dev;

static uint64_t selected_dcs_freq;

// v5 apply-timing handle:
// cumulative counts of real (non-deduped) stream sends and dcs reclocks. The
// daughterboard apply works diff these around their stream() call to learn whether
// THIS apply broke a timeline - the word dedup in __rfnm_la9310_stream_send is exactly
// why a frequency-only apply reports no break.
uint32_t rfnm_stream_send_cnt;
EXPORT_SYMBOL_GPL(rfnm_stream_send_cnt);
uint32_t rfnm_dcs_reclock_cnt;
EXPORT_SYMBOL_GPL(rfnm_dcs_reclock_cnt);
int rfnm_la9310_stream_send(uint32_t t);

//echo 61440 > /sys/class/i2c-dev/i2c-0/device/0-0058/rfnm_set_dcs_freq
//root@imx8mp-rfnm:~# ~/librfnm/build/utils/rfnm_loopback/rfnm_loopback

/*


#define iowr32(r,v,p) ( r->endian == MOD_BE ? iowrite32be((v), (p)) : iowrite32le ((v),(p)) )
#define iord32(r, p) ( r->endian == MOD_BE ? ioread32be((p)) : ioread32le((p)) )

#define iowr16(r,v,p) ( r->endian == MOD_BE ? iowrite16be((v), (p)) : iowrite16le ((v),(p)) )
#define iord16(r, p) ( r->endian == MOD_BE ? ioread16be((p)) : ioread16le((p)) )

*/

/* #define iowr32(r,v,p) iowrite32be((v), (p)) */
/* #define iord32(r, p) ioread32be((p)) */
/* static inline void iowr32(struct rfdevice *rf, uint32_t val, uint32_t *addr) */
/* { */
/*     if(rf->endian == MOD_BE) */
/*         iowrite32be((val), (addr)); */
/*     else */
/*         iowrite32le((val), (addr)); */
/* } */
/* static inline uint32_t iord32(struct rfdevice *rf, uint32_t *addr) */
/* { */
/*     if(rf->endian == MOD_BE) */
/*         return ioread32be((addr)); */
/*     else */
/*         return ioread32le((addr)); */
/* } */

/*
 * MPIC MSI IRQs to trigger host->modem interrupts
 */

#define SWCMD_SIZE 0x124
#define SWCMD_DATA_SIZE 0x104
#define SWCMD_CMD_SIZE 0x1C

static int la93xx_raise_msgunit_irq(struct rfdevice *rfdev,
			  int msg_unit_idx, int bit_num)
{
	struct la9310_msg_unit *msg_unit = &( ( (struct la9310_msg_unit *)( rfdev->ccsr_p + ( MSG_UNIT_OFFSET ) ) )[msg_unit_idx] );

	iowr32(rfdev, bit_num, &msg_unit->msiir);

	return 0;
}
/* Dummy function till we have proper err2str function in la9310 code */
static inline char *rf_err2str(uint32_t r)
{
    return "";
}

static void rf_raise_modem_irq(struct rfdevice *rfdev)
{
    la93xx_raise_msgunit_irq(rfdev, LA9310_IRQ_MUX_MSG_UNIT, LA9310_RF_SW_CMD_MSG_UNIT_BIT);

}

static void rf_free_cmd(struct rfdevice *rfdev, rf_sw_cmd_desc_t *sw_cmd)
{
	rf_sw_cmd_desc_t *remote_cmd;

	remote_cmd = &((struct rf_host_if *)(rfdev->rfic_p))->rf_mdata.host_swcmd;
//	iowr32(rfdev, RF_SW_CMD_STATUS_FREE, &remote_cmd->status);
	iowr32(rfdev, RF_SW_CMD_STATUS_FREE, &remote_cmd->status);
	sw_cmd->status = RF_SW_CMD_STATUS_FREE;
}

static void rf_swcmd_get_data( struct rfdevice *rfdev, struct rf_sw_cmd_desc *sw_cmd,
		       int data_size)
{
	struct rf_sw_cmd_desc *remote_cmd;
	u32 *data_r, *data_l, size_wrds, i;

	remote_cmd = &((struct rf_host_if *)(rfdev->rfic_p))->rf_mdata.host_swcmd;
	data_r = (u32 *) &remote_cmd->data[0];
	data_l = (u32 *) &sw_cmd->data[0];
	size_wrds = data_size >> 2;
	for (i = 0; i < size_wrds; i++) {
		data_l[i] = iord32(rfdev, data_r);

		//printk("%d: 0x%x, 0x%x\n", i, data_l[i], *data_r);
		data_r++;
	}
}


long long current_timestamp(void) {
    return ktime_get_ns() / (1000LL * 1000);
}


/*
 * This function will use shared memory to send RFIC SW command
 */
static int rf_send_swcmd(struct rfdevice *rfdev, struct rf_sw_cmd_desc *sw_cmd,
		  int data_size)
{
	int ret = 0;
	struct rf_sw_cmd_desc *remote_cmd;
	int cmd_size, i;
	struct rf_host_stats *stats = &rfic_stats;
	//struct gul_dev *gul_dev = NULL;
	u32 *cmd_wrd_l, *cmd_wrd_r;
    //printk("rfdev : rfic_p : %p, ccsr_p : %p, endian : %d\n", rfdev->rfic_p, rfdev->ccsr_p, rfdev->endian);

	// LA9310 PCIe link down (hard reprobe in progress): the mailbox is unreachable, fail like a busy
	// mailbox - the callers already handle the swcmd busy/timeout path
	if (rfnm_la9310_mmio_fenced()) {
		stats->sw_cmds_desc_busy++;
		return -EBUSY;
	}

	remote_cmd = &((struct rf_host_if *)(rfdev->rfic_p))->rf_mdata.host_swcmd;

	if (iord32(rfdev, &remote_cmd->status) != RF_SW_CMD_STATUS_FREE) {
		if (iord32(rfdev, &remote_cmd->status) == RF_SW_CMD_STATUS_DONE) {
			/* This can happen when command processing takes more
			 * time than timeout configured in this driver. We can
			 * process successfully in this case as DONE from MODEM
			 * means FREE for this driver.
			 */
			//printk( "RFIC: timeout is insufficient for command : %d\n",iord32(rfdev, &remote_cmd->cmd));
		} else {
			//printk( "RFIC: mdata host swcmd busy [%d]\n", iord32(rfdev, &remote_cmd->status));
			stats->sw_cmds_desc_busy++;
			ret = -EBUSY;
			goto busy_out;
		}
	}

#ifdef GEUL
	sw_cmd->timeout = RFIC_REMOTE_CMD_TIMEOUT;
#endif
	sw_cmd->status  = RF_SW_CMD_STATUS_POSTED;
	sw_cmd->flags = RF_SW_CMD_REMOTE;

	cmd_size = SWCMD_CMD_SIZE + data_size;
	/*cmd_size is needed in words*/
	cmd_size = cmd_size / 4;
	cmd_wrd_l = (u32 *) sw_cmd;
	cmd_wrd_r = (u32 *) remote_cmd;
	//printk( "cmd %d, cmdsize %d\ncmd dump", sw_cmd->cmd, cmd_size);
	/*Modem E200 cores are big-endian, thus changing endianness.
	 *Copying local command to remote command is sub-optimal, but because
	 *of different endianness it is better to handle change of endianess via
	 *a copy so that code is less error prone. Moreover there is no real
	 *time or low latency requirement of this interface, so a copy here is
	 *fine
	 */
	for (i = 0; i < cmd_size; i++) {
		iowr32(rfdev, *cmd_wrd_l, cmd_wrd_r);
		//printk( "cmd_wrd_l 0x%x, cmd_wrd_r 0x%x\n", *cmd_wrd_l, *cmd_wrd_r);
		cmd_wrd_l++;
		cmd_wrd_r++;
	}
	dma_wmb();
	rf_raise_modem_irq(rfdev);
	dma_wmb();
	/* Wait Modem RF driver to complete processing */
#if 1

    long long start = current_timestamp();
    long long endwait = start + 100;

    while (start < endwait && !rfnm_la9310_mmio_fenced() &&
           (iord32(rfdev, &remote_cmd->status) != RF_SW_CMD_STATUS_DONE)) {
        start = current_timestamp();
	}
	if ((iord32(rfdev, &remote_cmd->status)  != RF_SW_CMD_STATUS_DONE)) {
		//printk( "swcmd 0x%x timed out\n", sw_cmd->cmd);
		stats->sw_cmds_timed_out++;
		ret = -EBUSY;
		goto out;
	}
	//printk( "swcmd 0x%x done\n", sw_cmd->cmd);
	stats->sw_cmds_tx++;

	ret = iord32(rfdev, &remote_cmd->result);
	if (ret != RF_SW_CMD_RESULT_OK) {
		//printk( "%s: CMD response error. result[%d:%s]\n", __func__, ret, rf_err2str(ret));
	}
    rf_swcmd_get_data(rfdev, sw_cmd, data_size);

out:
	iowr32(rfdev, RF_SW_CMD_STATUS_FREE, &remote_cmd->status);
#endif
busy_out:

	return ret;
}

static void print_stream_params_from_t(uint32_t t)
{
    uint32_t tx           = (t >> 0)  & 0x1;
    uint32_t rx[4];
    rx[0]               = (t >> 1)  & 0x1;
    rx[1]               = (t >> 2)  & 0x1;
    rx[2]               = (t >> 3)  & 0x1;
    rx[3]               = (t >> 4)  & 0x1;
    uint32_t rx_decimation = ((t >> 5) & 0x3) | (((t >> 11) & 0x3) << 2);
    uint32_t tx_upsampling = ((t >> 7) & 0x3) | (((t >> 13) & 0x3) << 2);
    uint32_t rx_dcs        = (t >> 9)  & 0x1;
    uint32_t tx_dcs        = (t >> 10) & 0x1;

    printk("stream: -> applied tx <%u> dcs %u upsampling %u; "
           "rx: <%u %u %u %u> dcs %u decimation %u\n",
           tx, tx_dcs, tx_upsampling,
           rx[0], rx[1], rx[2], rx[3],
           rx_dcs, rx_decimation);
}


// THE SESSION (v2 charter P1): every piece of client-lifetime timing state in ONE
// object, so ownership is a fact of the type instead of a convention spread over
// file-scope globals (a proven stale-ownership defect class). Lifetime = SM reset to SM reset:
// rfnm_la9310_quiesce IS the declared ownership boundary - a client that armed the
// pattern or pushed ring entries and died must not gate the next owner's stream
// (failure signature: a plain 7.68M client reading 24 Msps of gated flood until
// reboot). The anchor is the one STANDING field (P2): it is phase INTENT - a
// residue, not a tick - so it cannot go stale and cannot harm an heir; it lives
// until replaced by the next anchor_at.
struct rfnm_session {
	uint32_t stream_word;		// live applied stream word (0 = quiesced)
	uint32_t anchor_tick;		// standing phase intent (kind-4); only its residue matters
	int anchor_valid;
	uint32_t tdd_period_chunks;	// pattern as last commanded = the anchor granule source
	bool tdd_armed;			// pattern reached the M4 (quiesce must disarm)
};
static struct rfnm_session rfnm_session;

// True iff the applied stream word is the one layout rfnm_qec understands:
// some RX enabled, no DCS mode, no decimation (native 122.88 full rate).
int rfnm_stream_is_native_rx(void) {
	uint32_t t = rfnm_session.stream_word;

	return ((t >> 1) & 0xf) && !((t >> 9) & 0x1) && !(((t >> 5) & 0x3) | (((t >> 11) & 0x3) << 2));
}
EXPORT_SYMBOL_GPL(rfnm_stream_is_native_rx);

// phytimer phase 2a: the LA9310-M4 owns the RX allow-gate comparators - it closes the
// gates before the stream teardown, mints T0 on-chip (us-scale guard), ships the full
// 32-bit T0 to the VSPA inside the stream mailbox, and arms the gates at exactly T0
// after the VSPA ack (vRficProcessIqDump in la93xx_freertos). A late arm fails the
// swcmd loudly, so the caller's error path retries with a fresh command. The phase 1
// kernel-side minting/arming (BAR0 pokes) is gone with it.

static uint32_t rfnm_la9310_phytimer_now(void);
extern uint32_t rfnm_sched_anchor_step_ticks;	// la9310rfnm-owned (v3 bundle), used by the projection
static void rfnm_la9310_stream_reapply(void);
static void rfnm_anchor_reapply_workfn(struct work_struct *work) {
	rfnm_la9310_stream_reapply();
}
static DECLARE_WORK(rfnm_anchor_reapply_work, rfnm_anchor_reapply_workfn);

// One lock for every stream-word sender (apply / quiesce / regate).
// The senders sleep (swcmd + the epoch-publish wait) and all run in process context, but
// they used to interleave freely: a regate work item could read the word mid-apply, or
// cross a concurrent quiesce and heal a torn-down session. Under this lock the live word
// is settled truth wherever a sender reads it.
static DEFINE_MUTEX(rfnm_stream_send_lock);

// windowed-session mode (rt-cores step 2; the engine lives with the vwin block
// below; declared here for the injection at the one stream chokepoint). Windowed
// intent is PER-SESSION: the RT command ring armed (debugfs verb / windowed
// client) when the TX apply lands. The old rfnm_tx_windowed module param - a
// global mode knob that windowized EVERY session and silently darkened plain TX
// wherever a boot script re-asserted it - is deleted.
static int rfnm_rtc_armed(void);
static int rfnm_rtc_arm(void);
static int rfnm_rtc_disarm(void);
static void rfnm_vwin_reset(void);

// The LA9310's AXIQ/DMA-IF comes out of EVERY chip reset in a state
// where the TX start ritual's ptr_rst handshake cannot complete behind the closed
// gate; one completed stream cycle on ANY lane mints the
// block-level state it needs (proven: an RX micro-cycle is sufficient, a bare completed
// PRST is not). Before the FIRST TX-carrying word of a phy time generation, run one
// internal RX micro-cycle: rx0-on at the plan's dcs bits, a short dwell, all-off.
// The phy generation is the authority (every reset path that mints a gen already
// bumps it or tick promises break), so the invariant cannot rot with new reset
// paths. Fail-open: a failed init send falls through to the client's word - worst
// case is the pre-fix behavior (honest session fault, reopen recovers).
static uint32_t rfnm_streamed_gen;
static int rfnm_streamed_gen_valid;
extern uint32_t rfnm_phy_gen_now(void);
static int __rfnm_la9310_stream_send(uint32_t t, int force);

static void rfnm_gen_tx_init_cycle(uint32_t t) {
	// rx0 on, dcs bits copied from the client's word so the internal cycle runs on
	// the exact current plan (no reclock logic can trigger); no decim, no ups
	uint32_t rx_word = (t & ((1u << 9) | (1u << 10))) | (1u << 1);
	int ret;

	printk("RFNM: gen-init RX micro-cycle before first TX arm of phy gen %u\n",
			rfnm_phy_gen_now());
	ret = __rfnm_la9310_stream_send(rx_word, 0);
	if(ret) {
		printk("RFNM: gen-init rx word failed (%d) - falling through to the TX apply\n", ret);
		return;
	}
	// 300 ms: the apply budget is ~1 s client-side and a reclock eats ~0.4 s + ~0.2 s of
	// publish waits (500 ms dwell tipped code-4 on the margin); 60/500 ms immunized 9/10 /
	// 12/13 (the residual tail = the separate DCS-100 DAC-drain defect,
	// which this init cannot reach). Empirical bound inside the apply budget.
	msleep(300);
	ret = __rfnm_la9310_stream_send(0, 0);
	if(ret) {
		printk("RFNM: gen-init close word failed (%d) - falling through to the TX apply\n", ret);
	}
}

// force=1 skips the word-unchanged early-out (the regate resends the SAME word for a
// fresh T0). It replaces the old "zero the global, then resend" regate idiom, which left
// rfnm_session.stream_word reading 0 for the whole send (up to 50 ms) - any concurrent
// reader in that window judged the session torn down. Callers hold rfnm_stream_send_lock.
static int __rfnm_la9310_stream_send(uint32_t t, int force) {
	// windowed session (rt-cores step 2): inject the mode bit BEFORE the dedupe
	// compare so the stored word, regate resends and the A2 config latch all see
	// one consistent word. TX-carrying words only. Intent = ring armed at apply
	// time (a windowed test harness arms via the debugfs verb before its apply); the
	// quiesce/SM-reset word disarms, so intent can never leak across sessions.
	if(rfnm_rtc_armed() && (t & 0x1)) {
		t |= (1u << 16);
	}
	extern volatile struct rfnm_la9310_status *rfnm_la9310_status;
	uint32_t rx_epoch_pre = 0, tx_epoch_pre = 0;
	int wait_rx, wait_tx;
	int ret;

	if(!force && t == rfnm_session.stream_word) {
		return 0;
	}

	// gen-init injection point: TX-carrying word, no channel-on word has run on
	// this phy generation, and no session is nominally live (a live word implies
	// the gen-mismatch was already healed by the restart reapply - log if not).
	if((t & 0x1) &&
	   (!rfnm_streamed_gen_valid || rfnm_streamed_gen != rfnm_phy_gen_now())) {
		if(rfnm_session.stream_word & 0x1E) {
			printk("RFNM: gen-init SKIPPED (live rx word %x on a fresh gen - restart reapply owns this)\n",
					rfnm_session.stream_word);
		} else {
			rfnm_gen_tx_init_cycle(t);
		}
	}

	// Contract: apply() returning must GUARANTEE the timing anchors are host-visible.
	// The VSPA processes the stream word asynchronously (this swcmd completes when the
	// M4 accepts it), so a client reading get_rx/tx_timing right after apply RACED the
	// publication and got the PREVIOUS stream's anchor - the first session after boot
	// got the boot block (epoch 0, r_shift 0) and a void timeline; every session read
	// its predecessor's epoch. The fw publishes the anchors synchronously inside its
	// apply handler now (LA9310_cal 37c8dc9); the wait below closes the contract
	// end-to-end. Epochs only bump for enabled directions, so only those are waited
	// on; bounded and fail-open with a loud log.
	wait_rx = ((t >> 1) & 0xf) != 0;
	wait_tx = (t & 0x1) != 0;
	// A2 scoped applies: a regate that resends the LIVE word verbatim is TX-PRESERVING
	// fw-side (the M4 keeps the running chain; t0/epoch never move), so a tx-epoch bump
	// is not coming - this wait burned its full 50 ms on every heal. The TX anchor IS
	// host-visible, unchanged. Fresh or changed words keep the full anchor-publish contract.
	if(force && t == rfnm_session.stream_word) {
		wait_tx = 0;
	}
	if(rfnm_la9310_status) {
		rx_epoch_pre = rfnm_la9310_status->rx_epoch;
		tx_epoch_pre = rfnm_la9310_status->tx_epoch;
	}

	// The anchor-publish wait was fail-open AND single-shot - a word landing
	// in a fw bring-up window could stay unprocessed forever, with the dedup pinning
	// the divergence. One bounded re-send (safe: the regate path resends the same word
	// for a fresh T0 by design) covers the transient; still fail-open after that, loudly.
	int attempt;
	int published = 0;
	for(attempt = 0; attempt < 2 && !published; attempt++) {
		struct sw_cmddata_dump_iq * data;
		struct rf_sw_cmd_desc sw_cmd = {0};
		sw_cmd.cmd = RF_SW_CMD_DUMP_IQ_DATA;
		data = (struct sw_cmddata_dump_iq *)&sw_cmd.data[0];
		data->addr = t;
		if(rfnm_session.anchor_valid) {
			// v2 charter P2: ship the phase INTENT; the M4 - the only true clock -
			// resolves congruence at the mint. No projection, no staleness window: a
			// phase cannot age (the r10 K1 heuristics died with the absolute-tick
			// field). Granule = the TDD period when commanded, else the word's own
			// anchor step (384 << larger enabled dcs keeps BOTH converter grids on
			// duplex words - the duplex-grid rule).
			uint32_t g;
			if(rfnm_session.tdd_period_chunks) {
				g = rfnm_session.tdd_period_chunks * (384u << ((t >> 9) & 0x1));
			} else {
				uint32_t d = (t >> 9) & 0x1;
				if((t & 0x1) && ((t >> 10) & 0x1) > d) {
					d = (t >> 10) & 0x1;
				}
				g = 384u << d;
			}
			data->anchor_phase = rfnm_session.anchor_tick;
			data->anchor_granule = g;
			printk_ratelimited("rfnm: anchor intent phase %u granule %u\n",
					rfnm_session.anchor_tick % g, g);
		}

		ret = rf_send_swcmd(rfdev, &sw_cmd, sizeof(struct sw_cmddata_dump_iq));

		rf_free_cmd(rfdev, &sw_cmd);
		if(ret) {
			return ret;
		}

		published = 1;
		if(rfnm_la9310_status && (wait_rx || wait_tx)) {
			unsigned long tmo = jiffies + msecs_to_jiffies(50);
			while(time_before(jiffies, tmo)) {
				if((!wait_rx || rfnm_la9310_status->rx_epoch != rx_epoch_pre) &&
				   (!wait_tx || rfnm_la9310_status->tx_epoch != tx_epoch_pre)) {
					break;
				}
				usleep_range(200, 400);
			}
			if((wait_rx && rfnm_la9310_status->rx_epoch == rx_epoch_pre) ||
			   (wait_tx && rfnm_la9310_status->tx_epoch == tx_epoch_pre)) {
				published = 0;
				printk_ratelimited("rfnm: stream anchor publish wait timed out (word %x rx %d tx %d, attempt %d)%s\n",
						t, wait_rx, wait_tx, attempt + 1, attempt == 0 ? " - re-sending once" : "");
			}
		}
	}

	print_stream_params_from_t(t);
	//printk("stream: sent %x last was %x \n", t, rfnm_session.stream_word);

	// windowed lifecycle: ring armed only AFTER the apply completed (the M4
	// warmup minted the latch and force-closed C11; arming earlier would close
	// the gate across the enable pulse = the stuck-queue dead-latch birth). Teardown
	// words return C11 to the legacy path and drop queued windows.
	if(t & (1u << 16)) {
		rfnm_rtc_arm();		/* -EBUSY on re-applies = already armed, fine */
	} else {
		rfnm_vwin_reset();
		// disarm only when a windowed session ends/hands over (the previously
		// applied word carried bit 16) or a plain TX session actively claims the
		// pump (TX-carrying word). A bare quiesce word must NOT wipe a pre-armed
		// intent: every client OPEN quiesces (word 0) before its first apply, so
		// arm-then-open-then-apply is a legal windowed-client sequence.
		if((rfnm_session.stream_word & (1u << 16)) || (t & 0x1)) {
			rfnm_rtc_disarm();
		}
	}

	rfnm_session.stream_word = t;
	rfnm_stream_send_cnt++;
	// Any channel-on word that reached the fw counts as this generation's cycle
	// (proven: even faulted/partial cycles mint the block state)
	if(t & 0x1F) {
		rfnm_streamed_gen = rfnm_phy_gen_now();
		rfnm_streamed_gen_valid = 1;
	}

	return 0;
}

int rfnm_la9310_stream_send(uint32_t t) {
	int ret;

	mutex_lock(&rfnm_stream_send_lock);
	ret = __rfnm_la9310_stream_send(t, 0);
	mutex_unlock(&rfnm_stream_send_lock);
	return ret;
}

// After an UNCOMMANDED phytimer restart (fw rebooted under a live session) the
// stored word no longer matches fw state, and the same-word dedup would pin that
// divergence until a rate change. Re-apply the live word once, from process context.
// TX-only words are NOT re-applied (re-minting tx_t0 mid-session = measured TA chaos,
// same rule as the regate reapply); those sessions surface TIME_RESET and rebuild.
extern void (*rfnm_phy_restart_notify_cb)(void);
static uint32_t rfnm_phy_restart_stash;
static void rfnm_phy_restart_workfn(struct work_struct *work) {
	uint32_t t;
	int ret;

	mutex_lock(&rfnm_stream_send_lock);
	t = rfnm_phy_restart_stash;
	rfnm_phy_restart_stash = 0;
	if(t && ((t >> 1) & 0xF)) {
		ret = __rfnm_la9310_stream_send(t, 1);
		printk("rfnm_lalib: stream word %x re-applied after uncommanded phytimer restart: %d\n", t, ret);
	}
	mutex_unlock(&rfnm_stream_send_lock);
}
static DECLARE_WORK(rfnm_phy_restart_work, rfnm_phy_restart_workfn);

// The host<->M4 VSPA mailbox ownership verb (see la9310_rfnm.c cb decl)
extern int (*rfnm_vspa_handoff_cb)(int on);
static int rfnm_lalib_vspa_handoff(int on) {
	struct rf_sw_cmd_desc sw_cmd = {0};
	int ret;

	sw_cmd.cmd = RF_SW_CMD_VSPA_MBOX_HANDOFF;
	((uint32_t *)&sw_cmd.data[0])[0] = !!on;
	ret = rf_send_swcmd(rfdev, &sw_cmd, sizeof(uint32_t));
	rf_free_cmd(rfdev, &sw_cmd);
	return ret;
}
static void rfnm_lalib_phy_restart_notify(void) {
	// timer (softirq) context - no sleeping: stash the live word, kill the dedup
	// baseline so no same-word apply early-outs against unknown fw state, and resend
	// from a work. Racing a concurrent sender only ever forces one extra resend.
	uint32_t t = rfnm_session.stream_word;

	if(t) {
		rfnm_phy_restart_stash = t;
		rfnm_session.stream_word = 0;
		schedule_work(&rfnm_phy_restart_work);
	}
}

extern uint64_t rfnm_tx_slot_rate_hz;
void rfnm_tx_flush_staging(int scrub_ring);
static uint8_t rfnm_tx_was_on;

// v3 bundle: the anchor congruence step (ticks) for the CURRENTLY APPLIED stream
// word - anchors mint on multiples of 384 << max(rx_dcs, tx_dcs) (the r4 M4 rule).
// Published in dev_status ext so clients stop re-deriving it from cached hwinfo
// (the r4b anchor_at_probe stale-cache folklore). DEFINED in la9310rfnm (module
// load order: it loads first); this module computes it at stream apply.
extern uint32_t rfnm_sched_anchor_step_ticks;

// DCS overclock ceiling: 160 MHz is the certified silicon limit, 200 MHz validated on hardware (2026-06-10)
#define RFNM_LA9310_DCS_MAX_HZ 200000000ULL
// DCS floor: the LA9310's platform clocks derive from the DCS input; a 61.44 MHz retune froze
// PCIe link training after the hard reset and hung the i.MX8MP bus (power-cycle required,
// 2026-06-12). 81.92 MHz is the lowest hardware-validated point. Rates that would need a lower
// DCS must be served by deeper VSPA decimation instead of a slower clock.
#define RFNM_LA9310_DCS_MIN_HZ 100000000ULL	// 81.92 validated; 100 MHz keeps margin above the hang floor

// Planning result for one (rate, mode) request: the DCS target plus the chain-config knobs.
struct rfnm_la9310_dcs_plan {
	uint64_t dcs_freq;
	uint8_t rx_dcs;
	uint8_t rx_decimation;
	uint8_t tx_dcs;
	uint8_t tx_upsampling;
	uint32_t caps;		// VSPA kernel capabilities this plan requires
};

// Pure DCS/chain planner - the ladder math only, no hardware access, no bookkeeping. Split out of
// rfnm_la9310_stream so SET_SAMP_RATE can validate a rate at set time with exactly the
// rules the stream path will apply later.
static int rfnm_la9310_plan(uint64_t user_dcs_hz, uint8_t tx, uint8_t *rx, struct rfnm_la9310_dcs_plan *plan) {

	uint64_t suggested_dcs_freq = 0;
	uint64_t max_la9310_freq = RFNM_LA9310_DCS_MAX_HZ;
	uint64_t min_la9310_freq = RFNM_LA9310_DCS_MIN_HZ;
	uint8_t tx_dcs = 0;
	uint8_t tx_upsampling = 0;	// 0..8 = 1x..256x (4 bits in the config word: 7-8 + 13-14)
	uint8_t rx_dcs = 0;
	uint8_t rx_decimation = 0;

	if(!tx) {
		// unified RX rate ladder: maximum VSPA decimation THE MACHINERY CAN SERVICE.
		// Pure-FIR configs (dcs=0, full-rate input) are budget-clean up to 8x (decim 3):
		// the fw gets 3.84 us per 768-sample chunk at DCS 200 MHz and the deep-cascade
		// service fits through 3 stages (verified on-air against strong carriers; 16x+
		// at full input rate periodically overruns the deadline and paints sub-rate spur
		// combs around strong carriers). Deeper rungs take the rx_dcs divide-by-2 in
		// front of the cascade: the halved VSPA input rate doubles the per-chunk budget,
		// and DCS = rate << (k+1) keeps the whole 2^n zoom family on one DCS (200 MHz -
		// zero si5510 retunes/hard resets while zooming). Try deepest k first; at each k
		// the dcs=1 variant (higher DCS, more budget) wins over dcs=0 (capped at k <= 3).
		int k;
		int pass;
		// pass 0 applies the per-depth VSPA input-rate ceilings (measured against a
		// continuously-keyed strong carrier: 64x+ at 100 MHz input still combs at
		// -2..-11 dBc; at 50 MHz input the budget quadruples vs the original config).
		// pass 1 drops the deep ceiling for rates with no lower-DCS alternative
		// (240k-class: 256x at DCS 122.88 MHz is the only in-window config).
		for(pass = 0; pass < 2 && !suggested_dcs_freq; pass++) {
			for(k = 8; k >= 0; k--) {
				uint64_t f = user_dcs_hz << (k + 1);
				uint64_t dcs_ceiling = (k >= 6 && pass == 0) ? 100000000ULL : max_la9310_freq;
				if(f <= dcs_ceiling && f >= min_la9310_freq && k >= 3) {
					suggested_dcs_freq = f;
					rx_dcs = 1;
					rx_decimation = k;
					break;
				}
				f = user_dcs_hz << k;
				if(k <= 3 && f <= max_la9310_freq && f >= min_la9310_freq) {
					suggested_dcs_freq = f;
					rx_dcs = 0;
					rx_decimation = k;
					break;
				}
			}
		}
	} else if(tx) {
		if(rx[0] || rx[1] || rx[2] || rx[3]) {
			// FDX matched rung (offered whenever a REGISTERED VSPA kernel
			// advertises FDX_CONCURRENT - the flag-file/fdx_plans era is dead; the
			// apply below swaps the right kernel in): TX X2-interp + RX X2-decim
			// share one BLESSED DCS at user*4 - the rule is "decimate RX to the same
			// freq you send". 25M duplex then rides the proven-healthy auto-plan
			// TX shape (DCS 100, dcs1+ups1). Blessed clocks only: unvalidated DCS
			// frequencies are independently broken (verified per-frequency).
			if(rfnm_vspa_registry_has(RFNM_VSPA_CAP_FDX_CONCURRENT) &&
					(user_dcs_hz * 4 == 100000000ULL || user_dcs_hz * 4 == 122880000ULL)) {
				plan->dcs_freq = user_dcs_hz * 4;
				plan->rx_dcs = 1;
				plan->rx_decimation = 1;
				plan->tx_dcs = 1;
				plan->tx_upsampling = 1;
				plan->caps = RFNM_VSPA_CAP_STREAMING_BASE | RFNM_VSPA_CAP_FDX_CONCURRENT;
				return 0;
			}
			// concurrent TX+RX: both sides share one DCS at user_rate x 2 (hw divide-by-2
			// each way, no decimation/upsampling). Valid user rates are therefore
			// [min/2, max/2] = [50M, 100M] - 61.44M is the standard point. Anything else
			// is rejected HERE, with the real numbers, before the ladder can hand an
			// out-of-range DCS to the hard-reset reclock path. (The old code silently
			// mangled the rate - user_dcs_hz = 30720000*4, a units bug that turned any
			// >50M request into a 245.76 MHz DCS target and a guaranteed reclock failure.)
			// Lower concurrent rates need matched TX-ZOH + RX-decim at one DCS plus the
			// TRX DMEM rebalance (the ZOH staging buffer is rfnm_rx_out, busy in
			// concurrent mode) - future work.
			suggested_dcs_freq = user_dcs_hz * 2;
			rx_dcs = 1;
			rx_decimation = 0;
			tx_dcs = 1;
			tx_upsampling = 0;

			if(suggested_dcs_freq < min_la9310_freq || suggested_dcs_freq > max_la9310_freq) {
				printk("RFNM: concurrent TX+RX at %llu Hz needs DCS %llu, outside the validated window [%llu, %llu]\n",
						(unsigned long long)user_dcs_hz, (unsigned long long)suggested_dcs_freq,
						(unsigned long long)min_la9310_freq, (unsigned long long)max_la9310_freq);
				return -EINVAL;
			}
			
		} else {
			// TX-only rate ladder: host rate x 2^(u+1) must land in the DCS window;
			// the hardware divide-by-2 (tx_dcs) sets the DAC rate, fw ZOH covers the
			// remaining 2^u (u <= 3). Prefer the LOWEST u (no ZOH images). Adds the
			// min-frequency check the old code lacked - 30.72M used to retune the DCS
			// to 61.44 MHz, below the 81.92 MHz validated hang floor.
			int u;
			for(u = 0; u <= 8; u++) {
				uint64_t f = user_dcs_hz << (u + 1);
				// deep TX upsampling mirrors RX deep decimation: 2^u X2-interpolator
				// stages, u <= 8 (256x = 240 kSPS host rate at DCS 122.88, zero retunes)
				if(f <= max_la9310_freq && f >= min_la9310_freq) {
					suggested_dcs_freq = f;
					tx_dcs = 1;
					tx_upsampling = u;
					break;
				}
			}
			if(!suggested_dcs_freq && user_dcs_hz <= max_la9310_freq && user_dcs_hz >= min_la9310_freq) {
				suggested_dcs_freq = user_dcs_hz;
				tx_dcs = 0;
				tx_upsampling = 0;
			}
		}
		
	} else {
		printk("Error, suggested_dcs_freq not being set!\n");
	}

	if(!suggested_dcs_freq) {
		// No valid DCS configuration for this rate/mode. Reject cleanly instead of
		// reprobing the LA9310 with a garbage clock (which fails and can hang PCIe).
		printk("RFNM: no valid DCS configuration for sample rate %llu Hz\n", (unsigned long long)user_dcs_hz);
		return -EINVAL;
	}

	plan->dcs_freq = suggested_dcs_freq;
	plan->rx_dcs = rx_dcs;
	plan->rx_decimation = rx_decimation;
	plan->tx_dcs = tx_dcs;
	plan->tx_upsampling = tx_upsampling;
	plan->caps = RFNM_VSPA_CAP_STREAMING_BASE;
	return 0;
}

// SET_SAMP_RATE-time validation: stamp NOT_SUPPORTED at set time instead of OK for a
// rate whose plan cannot exist or whose DCS target the si5510 cannot synthesize exactly. Channel
// enables are unknown at set time, so this validates the RX-only plan - the shared ladder window
// spans a full octave, so a shift into it always exists for any in-range rate; what this really
// screens is kHz-alignment of the DCS target. The stream path revalidates the mode-specific plan
// before any reclock anyway (concurrent TX+RX has a narrower window and still rejects at stream
// time, same as before).
int rfnm_la9310_samp_rate_ok(uint64_t user_hz) {
	uint8_t rx[4] = { 1, 0, 0, 0 };
	struct rfnm_la9310_dcs_plan plan;

	if(rfnm_la9310_plan(user_hz, 0, rx, &plan)) {
		return 0;
	}

	return rfnm_si5510_dcs_freq_supported(plan.dcs_freq);
}
EXPORT_SYMBOL_GPL(rfnm_la9310_samp_rate_ok);

int rfnm_la9310_stream(uint64_t user_dcs_hz, uint8_t tx, uint8_t *rx) {

	struct rfnm_la9310_dcs_plan plan;
	uint64_t suggested_dcs_freq;
	uint32_t current_dcs_freq;
	uint8_t tx_dcs;
	uint8_t tx_upsampling;
	uint8_t rx_dcs;
	uint8_t rx_decimation;
	int plan_ret;

	current_dcs_freq = rfnm_si5510_get_dcs_freq(si5510_i2c_client);
	// The si5510 driver's cache is the only bookkeeping tying this module to the real
	// clock. Take it verbatim - including 0, which means "the last program/settle attempt failed,
	// hardware state unknown" - so the reclock below always runs instead of "unchanged"-skipping
	// on stale local state (the rate/3 wedge stayed wedged until reboot precisely because this
	// skip trusted bookkeeping over hardware).
	selected_dcs_freq = current_dcs_freq;

	plan_ret = rfnm_la9310_plan(user_dcs_hz, tx, rx, &plan);
	if(plan_ret) {
		return plan_ret;
	}
	suggested_dcs_freq = plan.dcs_freq;
	rx_dcs = plan.rx_dcs;
	rx_decimation = plan.rx_decimation;
	tx_dcs = plan.tx_dcs;
	tx_upsampling = plan.tx_upsampling;

/*
	if(user_dcs_hz == 30720000 && tx && (rx[0] || rx[1] || rx[2] || rx[3])) {
		printk("Hack, force a config that works for concurrent tx/rx\n");
		suggested_dcs_freq = user_dcs_hz * 2;
		rx_dcs = 0;
		rx_decimation = 1;
		tx_dcs = 1;
		tx_upsampling = 0;
	}*/


	// stream config word layout (must match fw _main.c AND the M4 rfic_cmd.c, which hard-parses
	// rx_dcs at bit 9 / tx_dcs at bit 10 to set the ADC clock divider - moving those bits makes
	// every dcs mode silently run the ADC at 2x): bit 0 tx, bits 1-4 rx[0..3],
	// bits 5-6 rx_decimation[1:0], bits 7-8 tx_upsampling, bit 9 rx_dcs, bit 10 tx_dcs,
	// bits 11-12 rx_decimation[3:2] (deep modes)
	uint32_t t = ((tx & 0x1) << 0) | ((rx[0] & 0x1) << 1) | ((rx[1] & 0x1) << 2) | ((rx[2] & 0x1) << 3) | ((rx[3] & 0x1) << 4)
				| ((rx_decimation & 0x3) << 5)
				| ((tx_upsampling & 0x3) << 7)
				| ((rx_dcs & 0x1) << 9)
				| ((tx_dcs & 0x1) << 10)
				| (((rx_decimation >> 2) & 0x3) << 11)
				| (((tx_upsampling >> 2) & 0x3) << 13);	// tx_upsampling[3:2]: deep TX, same split trick as rx decim[3:2]


	if(suggested_dcs_freq != selected_dcs_freq) {
		int ret;

		if(suggested_dcs_freq < RFNM_LA9310_DCS_MIN_HZ || suggested_dcs_freq > RFNM_LA9310_DCS_MAX_HZ) {
			// belt and braces: a reclock is a full LA9310 hard reset - never run it
			// with a target outside the validated window, whatever branch computed it
			printk("stream: -> BUG: refusing reclock to out-of-range DCS %llu kHz\n", (unsigned long long)(suggested_dcs_freq / 1000));
			return -EINVAL;
		}

		if(!rfnm_si5510_dcs_freq_supported(suggested_dcs_freq)) {
			// Validate against the si5510's synthesizable set BEFORE the hard reset -
			// an unsupported target used to be discovered only after rfnm_si5510_reset_la9310 had
			// already unpowered the LA9310, leaving it dead in reset with its PCIe endpoint gone.
			printk("stream: -> refusing reclock to DCS %llu Hz: not synthesizable by the si5510\n", (unsigned long long)suggested_dcs_freq);
			return -EINVAL;
		}

		// Pick the VSPA kernel for this plan's capabilities BEFORE the reset -
		// the reprobe on the far side boots the selection. Refuse cleanly if no
		// registered kernel provides them (never reset toward an impossible plan).
		ret = rfnm_vspa_select_for(plan.caps);
		if(ret) {
			printk("stream: -> refusing reclock: no VSPA kernel provides caps %02x\n", plan.caps);
			return ret;
		}

		printk("stream: -> reclocking dcs frequency from %llu kHz to %llu kHz\n", (unsigned long long)(selected_dcs_freq / 1000), (unsigned long long)(suggested_dcs_freq / 1000));
		ret = rfnm_hard_reset_la9310(suggested_dcs_freq);
		if(ret) {
			printk("stream: -> failed to reclock dcs frequency to %llu kHz: %d\n", (unsigned long long)(suggested_dcs_freq / 1000), ret);
			return ret;
		}

		selected_dcs_freq = suggested_dcs_freq;
		rfnm_session.stream_word = 0;
		rfnm_dcs_reclock_cnt++;
	} else if(!rfnm_vspa_running_has(plan.caps)) {
		// Same DCS, different kernel need: the quiesce parks
		// the cache-less core, and a PARKED core boots the new image on write+GO in
		// ~ms (no core reset). If the fw refuses to park (-EBUSY), fall back to the
		// proven chip reset at the same DCS. Either way the fresh fw has no session
		// state: the dedup baseline dies and the word send below re-arms it.
		int ret;
		extern int rfnm_vspa_kernel_swap(uint32_t required_caps);

		printk("stream: -> vspa kernel swap on unchanged dcs %llu kHz (caps %02x)\n",
				(unsigned long long)(selected_dcs_freq / 1000), plan.caps);
		ret = rfnm_vspa_kernel_swap(plan.caps);
		if(ret) {
			printk("stream: -> parked swap unavailable (%d) - falling back to chip reset\n", ret);
			ret = rfnm_vspa_select_for(plan.caps);
			if(!ret) {
				ret = rfnm_hard_reset_la9310(plan.dcs_freq);
			}
		}
		if(ret) {
			printk("stream: -> vspa kernel change failed: %d\n", ret);
			return ret;
		}
		rfnm_session.stream_word = 0;
	} else {
		printk("stream: -> unchanged dcs frequency of %llu kHz \n", (unsigned long long)(selected_dcs_freq / 1000));
	}

	// tail extrapolation in the TX consumer needs the exact DAC drain rate in
	// 256-sample slots/s = the host-side TX sample rate / 256
	rfnm_tx_slot_rate_hz = tx ? div_u64(user_dcs_hz, 256) : 0;

	// v3 bundle: publish the anchor congruence step for this word (384 << max dcs)
	rfnm_sched_anchor_step_ticks = 384u << ((rx_dcs || tx_dcs) ? 1 : 0);

	// stream start = clean slate: stale packets from a previous stream must never
	// leak into the new one's cc space (rapid restarts burned ~97 gaps re-syncing).
	// stream STOP = silence the air: the fw free-runs the DAC ring forever, so
	// resident bursts would otherwise replay every ring lap until the next stream
	// start (~147 stale re-airs/s measured) - scrub so it airs zeros instead.
	if(tx != rfnm_tx_was_on) {
		// OFF-edge (fw quiescing): full flush incl. ring scrub - silence the air.
		// ON-edge = THE ARM WINDOW: staging flush + state resets ONLY, never a ring
		// write (dark-TX class 07-17: a writeback flood here starves the VSPA's
		// DDR reads and wedges the arming pump into zeros-forever; the ring is
		// already clean from the previous stop/close/SM-reset flush).
		rfnm_tx_flush_staging(!tx);
	}
	rfnm_tx_was_on = tx;

	// cpu-pd-wait starves the VSPA's inbound PCIe DDR reads (1.5 ms exit latency inside
	// a 66 us window - docs 5w), so hold a cpu-latency QoS while ANY stream is up. The
	// A.2-era crashes here were double add/remove races: concurrent rx+tx apply work
	// items shared request objects with no lock (plist_add oops), and the global-add
	// fault was the same corruption - not a framework defect. One request, one holder
	// flag, one mutex; applies run in process context (ioctl or workqueue) so the
	// mutex is safe.
	rfnm_cpu_qos_update(tx || rx[0] || rx[1] || rx[2] || rx[3]);

	if(t != rfnm_session.stream_word) {
		int ret;

		ret = rfnm_la9310_stream_send(t);
		if(ret) {
			printk("stream: -> failed to apply tx <%d> dcs %d upsampling %d; rx: <%d %d %d %d> dcs %d decimation %d: %d\n",
					tx, tx_dcs, tx_upsampling, rx[0], rx[1], rx[2], rx[3], rx_dcs, rx_decimation, ret);
			return ret;
		}
	}
	else {
		printk("stream: -> unchanged tx <%d> dcs %d upsampling %d; rx: <%d %d %d %d> dcs %d decimation %d\n", tx, tx_dcs, tx_upsampling, rx[0], rx[1], rx[2], rx[3], rx_dcs, rx_decimation);
	}

	//printk("stream -> %x\n", t);
	
	// simple code, but important concept: 
	// never trigger this stream command if RFNM_CH_STREAM_OFF is set
	// so that NXP can use the drivers without disabling this lib
	//if(t == rfnm_session.stream_word) {
	//	return 0;
	//}
	
	
	return 0;
	//return ret;
}

EXPORT_SYMBOL(rfnm_la9310_stream);

// Decode the programmed stream chain state out of the last stream command word so hwinfo
// can report the real converter rates: effective ADC rate = dcs_clk >> rx_dcs_div, and the
// user rate is that >> rx_decim_log2. All zero until a stream has programmed the chain.
void rfnm_la9310_get_clock_state(uint8_t *rx_dcs_div, uint8_t *tx_dcs_div, uint8_t *rx_decim_log2, uint8_t *tx_interp_log2) {
	uint32_t t = rfnm_session.stream_word;

	*rx_dcs_div = (t >> 9) & 0x1;
	*tx_dcs_div = (t >> 10) & 0x1;
	*rx_decim_log2 = ((t >> 5) & 0x3) | (((t >> 11) & 0x3) << 2);
	*tx_interp_log2 = ((t >> 7) & 0x3) | (((t >> 13) & 0x3) << 2);
}
EXPORT_SYMBOL(rfnm_la9310_get_clock_state);


int rfnm_la9310_tdd(uint32_t period_chunks, uint32_t duty_chunks);
int rfnm_la9310_txn(struct rfnm_dev_txn *t);


// P3/D3: the positional-viability oracle (consumed by the la9310rfnm TX path via
// rfnm_pos_tx_viable_cb). A TDD pattern WITHOUT an armed schedule ring paces the DAC
// drain to the duty and re-mints tx_epoch every window (measured) - positional TX
// is structurally impossible there, so it is refused.
int rfnm_la9310_pos_tx_viable(void) {
	if(!rfnm_session.tdd_armed) {
		return 1;
	}
	return 0;	// the schedule-ring escape hatch is deleted with the ring
}

extern int (*rfnm_pos_tx_viable_cb)(void);

void rfnm_la9310_quiesce(void) {
	// THE session ownership boundary: tear the dying session's state down in order
	// (the cleanup sends still read live fields), then the object is the next
	// owner's. The anchor intent deliberately survives (struct doc): a residue
	// cannot go stale, and the incoming owner's anchor_at lands BEFORE this
	// cleanup runs - clearing here recreates the stale-ownership defect.
	if(rfnm_session.stream_word != 0) {
		int ret = rfnm_la9310_stream_send(0);
		if(ret) {
			printk("stream: -> failed to quiesce LA9310 stream: %d\n", ret);
		}
		rfnm_session.stream_word = 0;
	}

	if(rfnm_session.tdd_armed) {
		if(rfnm_la9310_tdd(0, 0)) {
			printk("stream: -> failed to stop the TDD pattern on state reset\n");
		}
	}

}
EXPORT_SYMBOL_GPL(rfnm_la9310_quiesce);

// Autopsy: one line of register ground truth at park/re-park time (with no
// M4 console attached this is the only witness). Plain reads on the permanent
// ioremap: AXIQ SR0 RX nibbles (per lane: ENABLED/NOTEMPTY/ERRUNDER/ERROVER), SR1 TX
// enable, the five gate lines (comparator SC bit 31), and the WR channel's (ch11) DMA
// latches. NOTEMPTY on a parked lane = fill survived the park = the stale-fill face.
void rfnm_la9310_rx18_autopsy(const char *why) {
	uint32_t sr0, sr1, go, fifo, xrun, abrt, c11, g1, g2, g3, g4;

	if(!rfdev || !rfdev->ccsr_p || rfnm_la9310_mmio_fenced()) {
		return;
	}
	sr0 = readl(rfdev->ccsr_p + 0x1000500);
	sr1 = readl(rfdev->ccsr_p + 0x1000504);
	go = readl(rfdev->ccsr_p + 0x10000D8);
	fifo = readl(rfdev->ccsr_p + 0x10000DC);
	xrun = readl(rfdev->ccsr_p + 0x10000D4);
	abrt = readl(rfdev->ccsr_p + 0x10000C0);
	c11 = readl(rfdev->ccsr_p + 0x102005C);
	g1 = readl(rfdev->ccsr_p + 0x102000C);
	g2 = readl(rfdev->ccsr_p + 0x1020014);
	g3 = readl(rfdev->ccsr_p + 0x102001C);
	g4 = readl(rfdev->ccsr_p + 0x1020024);
	printk("stream: rx18 autopsy (%s): axiq sr0 %04x sr1tx %u | gates rx %u%u%u%u tx %u | ch11 go %u room %u xrun %u abort %u\n",
			why, sr0 & 0xFFFF, (sr1 >> 16) & 1,
			(g1 >> 31) & 1, (g2 >> 31) & 1, (g3 >> 31) & 1, (g4 >> 31) & 1, (c11 >> 31) & 1,
			(go >> 11) & 1, (fifo >> 11) & 1, (xrun >> 11) & 1, (abrt >> 11) & 1);
}

// Quiet arm-death detector - a stuck command queue on a READ channel is SILENT
// (commands stuck in the channel's DMA FIFO, nothing executing, no error latch:
// the register-proven famine/freeze ghost). Predicate per channel:
// FIFO_STAT bit LOW (no free entry = commands queued) AND XRUN bit LOW (nothing
// executing), sustained across consecutive polls - a healthy queued command shows
// XRUN within a beat, and a pended-behind-gate command sits IN the engine with
// XRUN high. Returns the mask of channels stuck RFNM_ARM_DEATH_POLLS polls in a
// row, then re-arms their hysteresis so the heal gets time to land. READ-ONLY by
// contract: GO_STAT must never be W1C'd from the host (ISM caution - it would eat
// VCPU go events) and DMA_STAT_ABORT reads self-clear; neither is touched.
// 40 polls at ~20 Hz = 2 s sustained: a real corpse persists forever (observed
// minutes-long), while gated session BRING-UP legitimately holds queued-not-yet-
// executing commands for hundreds of ms - K=3 false-positived there and one
// spurious regate seeds a park/heal storm (A/B tested). 2 s detection
// still beats the shim's 4 s famine threshold with margin.
#define RFNM_ARM_DEATH_POLLS 40
uint32_t rfnm_la9310_arm_death_check(void) {
	static uint8_t hits[16];
	uint32_t fifo, xrun, stuck, dead = 0;
	int n;

	if(!rfdev || !rfdev->ccsr_p || rfnm_la9310_mmio_fenced()) {
		return 0;
	}
	// Predicate refinement: watch ONLY the session's ACTIVE
	// channels. The observed false positive (FIRE ch2 with gates open, session
	// healthy, storm seeded by the heal) was an INACTIVE RX lane: a lane nobody
	// streams holds its stale queued commands forever (no GO trigger ever comes) -
	// FIFO=0 ^ XRUN=0 is that lane's legitimate steady state, not a corpse. Active
	// lanes: stream word bit0 = TX -> ch11 (LA9310_DMA_CHAN_AXIQ_TX), bits1-4 =
	// RX lanes A-D -> ch1-4 (LA9310_DMA_CHAN_AXIQ_RX1..4). aux/RSSI (ch5/6) are
	// fw-internal - never watched.
	{
		uint32_t t = READ_ONCE(rfnm_session.stream_word);
		uint32_t watch = 0;

		for(n = 0; n < 4; n++) {
			if(t & (1u << (1 + n))) {
				watch |= 1u << (n + 1);
			}
		}
		if(t & 1u) {
			watch |= 1u << 11;
		}
		fifo = readl(rfdev->ccsr_p + 0x10000DC);
		xrun = readl(rfdev->ccsr_p + 0x10000D4);
		stuck = ~fifo & ~xrun & watch;
	}
	for(n = 0; n < 16; n++) {
		if(stuck & (1u << n)) {
			if(++hits[n] >= RFNM_ARM_DEATH_POLLS) {
				dead |= 1u << n;
				hits[n] = 0;
			}
		} else {
			// Tuning forensics: a streak that CLEARS is the aliasing state the
			// K=40 threshold exists for - log its dwell so the predicate refinement
			// works from measured distributions, not folklore. Gate lines + GO word
			// discriminate the suspects (closed-span pend vs engine-residency-
			// without-XRUN vs read-channel FIFO_STAT semantics). Plain reads only.
			if(hits[n] >= 8) {
				uint32_t go = readl(rfdev->ccsr_p + 0x10000D8);
				uint32_t c11 = readl(rfdev->ccsr_p + 0x102005C);
				uint32_t g1 = readl(rfdev->ccsr_p + 0x102000C);
				uint32_t g2 = readl(rfdev->ccsr_p + 0x1020014);
				uint32_t g3 = readl(rfdev->ccsr_p + 0x102001C);
				uint32_t g4 = readl(rfdev->ccsr_p + 0x1020024);
				printk_ratelimited("stream: arm-death near-miss ch%d streak %u (%ums) fifo %04x xrun %04x go %04x gates rx %u%u%u%u tx %u\n",
						n, hits[n], hits[n] * 50, fifo & 0xFFFF, xrun & 0xFFFF, go & 0xFFFF,
						(g1 >> 31) & 1, (g2 >> 31) & 1, (g3 >> 31) & 1, (g4 >> 31) & 1, (c11 >> 31) & 1);
			}
			hits[n] = 0;
		}
	}
	if(dead) {
		uint32_t go = readl(rfdev->ccsr_p + 0x10000D8);
		uint32_t c11 = readl(rfdev->ccsr_p + 0x102005C);
		uint32_t g1 = readl(rfdev->ccsr_p + 0x102000C);
		uint32_t g2 = readl(rfdev->ccsr_p + 0x1020014);
		uint32_t g3 = readl(rfdev->ccsr_p + 0x102001C);
		uint32_t g4 = readl(rfdev->ccsr_p + 0x1020024);
		// fire forensics at printk (not pr_unflood - that path is level-suppressed
		// and the heals counter otherwise increments with zero witness)
		printk("stream: arm-death FIRE ch mask %04x fifo %04x xrun %04x go %04x gates rx %u%u%u%u tx %u hits[%u %u %u %u %u %u %u %u %u %u %u %u]\n",
				dead, fifo & 0xFFFF, xrun & 0xFFFF, go & 0xFFFF,
				(g1 >> 31) & 1, (g2 >> 31) & 1, (g3 >> 31) & 1, (g4 >> 31) & 1, (c11 >> 31) & 1,
				hits[0], hits[1], hits[2], hits[3], hits[4], hits[5],
				hits[6], hits[7], hits[8], hits[9], hits[10], hits[11]);
	}
	return dead;
}

// The CLIENT-COMMANDED stream re-apply - resend the current word for a
// fresh T0. Two callers, both client verbs: SET_TDD arming a pattern on a live stream,
// and anchor_at (sticky intent takes effect now). The autonomous heal machinery that
// shared this path (regate work, schedule-time snapshots, reparked retries) is
// deleted - git history holds it. A re-apply that lands
// parked is the park fault-predicate's problem, not ours.
static void rfnm_la9310_stream_reapply(void) {
	extern volatile struct rfnm_la9310_status *rfnm_la9310_status;
	uint32_t t, e_pre, e_post;
	int ret;

	mutex_lock(&rfnm_stream_send_lock);
	t = (uint32_t)rfnm_session.stream_word;
	if(t == 0 || !((t >> 1) & 0xF)) {
		// no live stream, or a TX-only word (a TX-only re-mint moves tx_t0 mid-session:
		// measured ms-scale TA chaos 2026-07-10) - the sticky intent applies at the
		// next mint instead
		mutex_unlock(&rfnm_stream_send_lock);
		return;
	}
	e_pre = rfnm_la9310_status ? rfnm_la9310_status->rx_epoch : 0;
	ret = __rfnm_la9310_stream_send(t, 1);
	if(ret) {
		// leave the global zeroed so the next apply of the same word resends
		// instead of early-outing against unknown fw state
		rfnm_session.stream_word = 0;
		printk_ratelimited("stream: re-apply FAILED: %d (word %x)\n", ret, t);
	} else {
		e_post = rfnm_la9310_status ? rfnm_la9310_status->rx_epoch : 0;
		printk_ratelimited("stream: re-apply landed (rx epoch %u -> %u, word %x)\n", e_pre, e_post, t);
	}
	mutex_unlock(&rfnm_stream_send_lock);
}





extern void (*rfnm_ptmr_regate_cb)(void);	// la9310rfnm's overrun-heal hook (callback avoids a circular module dep)
extern void (*rfnm_ptmr_regate_snap_cb)(void);	// schedule-time stream-word snapshot taker (same pattern)
extern uint32_t (*rfnm_ptmr_now_cb)(void);	// current-tick provider for timed TX placement (same pattern)

// Current phy timer tick via the spare comparator C21 capture (CAPTURE cmd to CnSC,
// captured count in CnV). C22 is the M4's t0-mint scratch channel - using it here
// would race the mint mid-apply, so C21 (spare per the comparator map) it is.
// MUST honor the link-down fence: this is a raw CCSR access on the permanent PCIe
// ioremap, and its hottest caller is the dev_status fill, which client polls drive at
// ~1 kHz THROUGH a DCS-reclock window - USB ep0 and LOCAL ioctl control paths are not
// serialized against the apply the way the TCP ctrl worker is, so the capture landed
// on the suspended RC and hard-hung the SoC (the 2026-07-07 campaign wedge, 3/3).
// While fenced, report the last captured tick: the LA9310 restarts its phytimer on the
// other side of the reclock anyway (new epoch/stream ack), and a frozen value keeps
// client-side unwrap state sane where a 0 would fake a wrap.
static uint32_t rfnm_la9310_phytimer_now(void) {
	static uint32_t last_now;

	if(rfnm_la9310_mmio_fenced()) {
		return last_now;
	}
	writel(0x20, rfdev->ccsr_p + 0x10200AC);
	last_now = readl(rfdev->ccsr_p + 0x10200B0);
	return last_now;
}

// The schedule request ring is DELETED (walker + kinds RX_WINDOW/
// TX_SLOT/FE - git history holds them). The one
// surviving SET_TXN kind is ANCHOR: the client's sticky pattern-phase intent
// (anchor_at) - byte-compatible on the wire, re-homed to a direct set + a
// client-commanded re-apply (scheduled: the USB ep0 completion path calls in from
// atomic context).
int rfnm_la9310_txn(struct rfnm_dev_txn *t) {
	extern int rfnm_phy_gen_session_ok(void);

	if(!rfnm_phy_gen_session_ok()) {
		return -ENODEV;	// no promises on a dead session/time generation
	}
	if(t->op != 1 || t->kind != RFNM_TXN_ANCHOR) {
		printk_ratelimited("rfnm: txn verb retired (op %u kind %u) - the schedule ring is deleted\n",
				t->op, t->kind);
		return -ENOSYS;
	}
	// v2 P2: standing phase intent - only the residue matters, so it cannot go
	// stale and lives until replaced. Carried by EVERY stream mint; the M4 mints
	// the first tick congruent to it.
	rfnm_session.anchor_tick = t->tick;
	rfnm_session.anchor_valid = !!(t->flags & 1);
	schedule_work(&rfnm_anchor_reapply_work);
	return 0;
}
EXPORT_SYMBOL_GPL(rfnm_la9310_txn);

// TX health snapshot for the pace detector's consumers and the tx_health debugfs. The
// AXIQ registers found during the stuck-queue investigation: SR1 = GPIN1 (host ccsr+0x1000504), CR3 =
// GPOUT7 (ccsr+0x100059C), C11 SC live level (ccsr+0x102005C). Plain reads, no side
// effects; fenced like every other MMIO in this file.
void rfnm_la9310_tx_health(struct rfnm_tx_health *h) {
	memset(h, 0xff, sizeof(*h));	// txn_* fields stay 0xff: the ring is deleted
	if(rfnm_la9310_mmio_fenced()) {
		return;
	}
	h->axiq_sr1 = readl(rfdev->ccsr_p + 0x1000504);
	h->axiq_cr3 = readl(rfdev->ccsr_p + 0x100059C);
	h->c11_sc = readl(rfdev->ccsr_p + 0x102005C);
}
EXPORT_SYMBOL_GPL(rfnm_la9310_tx_health);

// phytimer phase 2a debug trigger: echo a|b > /sys/kernel/debug/rfnm_switch_rf runs
// switch_rf() on the LA9310-M4 (timed FE flip via RFCTL_5 edge + rf_ctrl -> i.MX8MP M7).
// Bring-up instrument for the M-core TDD loop; the production trigger is the phase 2
// window/TDD scheduler.
#include <linux/debugfs.h>
static struct dentry *rfnm_switch_rf_dfs;
static ssize_t rfnm_switch_rf_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off) {
	char c = 0;
	uint32_t mode;
	struct rf_sw_cmd_desc sw_cmd = {0};
	int ret;

	if(len < 1 || copy_from_user(&c, ubuf, 1)) {
		return -EFAULT;
	}
	if(c == 'a') {
		mode = 0xAAAAAAAA;
	} else if(c == 'b') {
		mode = 0xBBBBBBBB;
	} else {
		return -EINVAL;
	}

	sw_cmd.cmd = RF_SW_CMD_SWITCH_RF;
	*(uint32_t *)&sw_cmd.data[0] = mode;
	ret = rf_send_swcmd(rfdev, &sw_cmd, sizeof(uint32_t));
	rf_free_cmd(rfdev, &sw_cmd);
	printk("rfnm: switch_rf(%c) -> %d\n", c, ret);
	return ret ? ret : len;
}
static const struct file_operations rfnm_switch_rf_fops = { .write = rfnm_switch_rf_write };

// phytimer phase 2a step 3 debug trigger: echo "<period_ticks> <duty_ticks>" >
// /sys/kernel/debug/rfnm_tdd starts the M4 TDD scheduler (gates + FE flips on one
// tick grid); "0 0" stops it. Production trigger = the phase 2 window/TDD API.
static struct dentry *rfnm_tdd_dfs;
// phytimer phase 2: configure the M4 TDD scheduler (both zero = stop). The pattern
// engages/disengages at a stream anchor, so a re-anchor is forced on success. Also
// the RFNM_SET_TDD control-verb backend (librfnm rx_tdd_configure).
int rfnm_la9310_tdd(uint32_t period_chunks, uint32_t duty_chunks) {
	extern int rfnm_phy_gen_session_ok(void);

	if(!rfnm_phy_gen_session_ok()) {
		return -ENODEV;	// no pattern promises on a dead time generation
	}
	struct rf_sw_cmd_desc sw_cmd = {0};
	int ret;

	sw_cmd.cmd = RF_SW_CMD_TDD;
	((uint32_t *)&sw_cmd.data[0])[0] = period_chunks;
	((uint32_t *)&sw_cmd.data[0])[1] = duty_chunks;
	ret = rf_send_swcmd(rfdev, &sw_cmd, 2 * sizeof(uint32_t));
	rf_free_cmd(rfdev, &sw_cmd);
	printk("rfnm: tdd period %u duty %u chunks -> %d\n", period_chunks, duty_chunks, ret);
	if(!ret) {
		// SM reset disarms a leftover pattern (see rfnm_la9310_quiesce)
		rfnm_session.tdd_armed = (period_chunks != 0 || duty_chunks != 0);
		rfnm_session.tdd_period_chunks = period_chunks;	// r10 K1: anchor projection granule
		rfnm_la9310_stream_reapply();	// client-commanded: the pattern reaches a live stream now
	}
	return ret;
}
EXPORT_SYMBOL_GPL(rfnm_la9310_tdd);

// v3 phase 0 debug triggers.
// echo "<src_slot> <nslots>" > /sys/kernel/debug/rfnm_tx_fill      - tone into the DAC ring
// echo "<lead_us> <len_ticks> <src_slot> <count> <gap_ticks>" > /sys/kernel/debug/rfnm_tx_window
//   - run <count> DFE-style TX windows: window k opens at now+lead + k*gap (absolute,
//     SAMPLE resolution), length len_ticks, payload from src_slot. Each swcmd blocks on
//     the M4 until its open edge fires, so the loop self-paces to the grid.
extern void rfnm_tx_fill_tone(uint32_t src, uint32_t nslots);
static struct dentry *rfnm_tx_fill_dfs;
static struct dentry *rfnm_tx_window_dfs;

static ssize_t rfnm_tx_fill_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off) {
	char kbuf[40] = {0};
	uint32_t src = 0, nslots = 0;

	if(len >= sizeof(kbuf) || copy_from_user(kbuf, ubuf, len)) {
		return -EFAULT;
	}
	if(sscanf(kbuf, "%u %u", &src, &nslots) != 2 || nslots == 0 || nslots > 16384) {
		return -EINVAL;
	}
	rfnm_tx_fill_tone(src, nslots);
	printk("rfnm: tx_fill_tone(%u, %u)\n", src, nslots);
	return len;
}
static const struct file_operations rfnm_tx_fill_fops = { .write = rfnm_tx_fill_write };

static ssize_t rfnm_tx_window_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off) {
	char kbuf[80] = {0};
	uint32_t lead_us = 0, len_ticks = 0, src = 0, count = 1, gap = 0;
	uint32_t tick0, k;
	int ret = 0;

	if(len >= sizeof(kbuf) || copy_from_user(kbuf, ubuf, len)) {
		return -EFAULT;
	}
	if(sscanf(kbuf, "%u %u %u %u %u", &lead_us, &len_ticks, &src, &count, &gap) < 3) {
		return -EINVAL;
	}
	if(count == 0 || count > 1000 || len_ticks == 0) {
		return -EINVAL;
	}

	tick0 = rfnm_la9310_phytimer_now() + (uint32_t)div_u64((u64)lead_us * 6144ull, 100);
	for(k = 0; k < count; k++) {
		struct rf_sw_cmd_desc sw_cmd = {0};
		int tries;

		sw_cmd.cmd = RF_SW_CMD_TX_WINDOW;
		((uint32_t *)&sw_cmd.data[0])[0] = tick0 + k * gap;
		((uint32_t *)&sw_cmd.data[0])[1] = len_ticks;
		((uint32_t *)&sw_cmd.data[0])[2] = src;
		// the M4 blocks in the previous window until its open edge fires: retry
		// the busy descriptor until it frees (that IS the pacing)
		for(tries = 0; tries < 5000; tries++) {
			ret = rf_send_swcmd(rfdev, &sw_cmd, 3 * sizeof(uint32_t));
			if(ret != -EBUSY) {
				break;
			}
			usleep_range(1000, 1500);
		}
		rf_free_cmd(rfdev, &sw_cmd);
		if(ret) {
			printk("rfnm: tx_window %u/%u tick %u -> %d\n", k + 1, count, tick0 + k * gap, ret);
			break;
		}
	}
	printk("rfnm: tx_window done (%u windows, len %u ticks, src %u, gap %u) -> %d\n",
			count, len_ticks, src, gap, ret);
	return ret ? ret : len;
}
static const struct file_operations rfnm_tx_window_fops = { .write = rfnm_tx_window_write };

static ssize_t rfnm_tdd_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off) {
	char kbuf[40] = {0};
	uint32_t period = 0, duty = 0;
	int ret;

	if(len >= sizeof(kbuf) || copy_from_user(kbuf, ubuf, len)) {
		return -EFAULT;
	}
	if(sscanf(kbuf, "%u %u", &period, &duty) != 2) {
		return -EINVAL;
	}
	ret = rfnm_la9310_tdd(period, duty);
	return ret ? ret : len;
}
static const struct file_operations rfnm_tdd_fops = { .write = rfnm_tdd_write };

// ==== RT command ring producer (rfnm_rtc_ring.h contract, option B) ====
// The M4 rtc engine owns every comparator arm; this side only posts absolute-tick
// verbs over PCIe + the MSG1 doorbell. Step 1: the debugfs test verb below is the
// only producer (no TX-path involvement); the windowed-session TX path becomes the
// real producer in step 2 (mode latch + descriptor sends).
static void __iomem *rfnm_rtc_p;
static DEFINE_SPINLOCK(rfnm_rtc_producer_lock);
static struct dentry *rfnm_rtc_dfs;

static void rfnm_rtc_doorbell(void) {
	la93xx_raise_msgunit_irq(rfdev, LA9310_IRQ_MUX_MSG_UNIT, RFNM_RTC_DOORBELL_MSG_UNIT_BIT);
}

// arm: producer-owned header fields only, magic LAST (the consumer zeroes its own
// counters + cons at the magic edge). -EBUSY when already armed.
// armed-query for the apply chokepoint: a single aligned ioread32 of the ring
// magic. Lockless by design - arm comes from the client BEFORE its apply,
// disarm from the quiesce word; a torn read is a caller sequencing bug, not ours.
static int rfnm_rtc_armed(void) {
	struct rfnm_rtc_ring __iomem *r = (struct rfnm_rtc_ring __iomem *)rfnm_rtc_p;

	if(!rfnm_rtc_p || rfnm_la9310_mmio_fenced()) {
		return 0;
	}
	return ioread32(&r->magic) == RFNM_RTC_RING_MAGIC;
}

static int rfnm_rtc_arm(void) {
	struct rfnm_rtc_ring __iomem *r = (struct rfnm_rtc_ring __iomem *)rfnm_rtc_p;
	unsigned long fl;

	if(!rfnm_rtc_p || rfnm_la9310_mmio_fenced()) {
		return -ENODEV;
	}
	spin_lock_irqsave(&rfnm_rtc_producer_lock, fl);
	if(ioread32(&r->magic) == RFNM_RTC_RING_MAGIC) {
		spin_unlock_irqrestore(&rfnm_rtc_producer_lock, fl);
		return -EBUSY;
	}
	iowrite32(0, &r->prod);
	wmb();
	iowrite32(RFNM_RTC_RING_MAGIC, &r->magic);
	spin_unlock_irqrestore(&rfnm_rtc_producer_lock, fl);
	rfnm_rtc_doorbell();
	return 0;
}

// disarm: the consumer returns C11 OPEN to the legacy path and logs its counters
static int rfnm_rtc_disarm(void) {
	struct rfnm_rtc_ring __iomem *r = (struct rfnm_rtc_ring __iomem *)rfnm_rtc_p;
	unsigned long fl;

	if(!rfnm_rtc_p || rfnm_la9310_mmio_fenced()) {
		return -ENODEV;
	}
	spin_lock_irqsave(&rfnm_rtc_producer_lock, fl);
	iowrite32(0, &r->magic);
	spin_unlock_irqrestore(&rfnm_rtc_producer_lock, fl);
	rfnm_rtc_doorbell();
	return 0;
}

// flush: drop everything queued, chain killed, session stays armed + gate closed
static int rfnm_rtc_flush(void) {
	struct rfnm_rtc_ring __iomem *r = (struct rfnm_rtc_ring __iomem *)rfnm_rtc_p;
	unsigned long fl;

	if(!rfnm_rtc_p || rfnm_la9310_mmio_fenced()) {
		return -ENODEV;
	}
	spin_lock_irqsave(&rfnm_rtc_producer_lock, fl);
	iowrite32(ioread32(&r->gen) + 1, &r->gen);
	spin_unlock_irqrestore(&rfnm_rtc_producer_lock, fl);
	rfnm_rtc_doorbell();
	return 0;
}

// push one verb; caller rings the doorbell (batchable). Honesty at the edge:
// ring-full and not-armed are errnos, nothing queues silently.
static int rfnm_rtc_push(uint32_t tick, uint32_t kind, uint32_t len_ticks) {
	struct rfnm_rtc_ring __iomem *r = (struct rfnm_rtc_ring __iomem *)rfnm_rtc_p;
	struct rfnm_rtc_cmd __iomem *e;
	uint32_t prod;
	unsigned long fl;

	if(!rfnm_rtc_p || rfnm_la9310_mmio_fenced()) {
		return -ENODEV;
	}
	spin_lock_irqsave(&rfnm_rtc_producer_lock, fl);
	if(ioread32(&r->magic) != RFNM_RTC_RING_MAGIC) {
		spin_unlock_irqrestore(&rfnm_rtc_producer_lock, fl);
		return -EPIPE;
	}
	prod = ioread32(&r->prod);
	if(prod - ioread32(&r->cons) >= RFNM_RTC_RING_ENTRIES) {
		spin_unlock_irqrestore(&rfnm_rtc_producer_lock, fl);
		return -ENOSPC;
	}
	e = &r->e[prod % RFNM_RTC_RING_ENTRIES];
	iowrite32(tick, (void __iomem *)e);		/* tick */
	iowrite32(kind, (void __iomem *)e + 4);		/* {kind, flags=0, type=0} lane, LE */
	iowrite32(len_ticks, (void __iomem *)e + 8);	/* len */
	iowrite32(0, (void __iomem *)e + 12);		/* bind */
	wmb();
	iowrite32(prod + 1, &r->prod);
	spin_unlock_irqrestore(&rfnm_rtc_producer_lock, fl);
	return 0;
}

// ==== windowed TX session (rt-cores step 2): the VSPA window-send engine ====
// Per assembled window the kernel issues posted writes only: ARM_TX_WINDOW into
// the rtc ring (gate edges, M4-owned) + MBOX_OPC_TX_WINDOW to the VSPA. The VSPA
// holds ONE window at a time (single prime + budget), so sends are PACED to land
// after the previous window's close - an hrtimer fires at that tick; everything
// else is fire-and-forget (the fw suppresses window acks in windowed sessions).
static int rfnm_vwin_go_param;
module_param_named(rfnm_vwin_go, rfnm_vwin_go_param, int, 0600);
MODULE_PARM_DESC(rfnm_vwin_go, "raise VSPA host_go after each windowed TX_WINDOW mbox (fallback if the mbox event alone does not wake the parked core)");

// VSPA host-interface registers, CCSR-relative (VSPA block at CCSR+0x1000000;
// mbox order MSB-then-LSB - the LSB write raises the valid/event, fastswap POC)
#define RFNM_VSPA_CONTROL	( 0x1000008 )
#define RFNM_VSPA_OUT_MSB	( 0x1000680 )
#define RFNM_VSPA_OUT_LSB	( 0x1000684 )
#define RFNM_VSPA_MBOX_STATUS	( 0x10006A0 )	/* bit0 = host-out mbox0 unconsumed */
// CONTROL write mask: never rewrite the loader bits (0-7, 16=vcpu_reset) and never
// reflect the W1C mbox event bits (20/21) back - a blind RMW would eat a pending
// mailbox event
#define RFNM_VSPA_CONTROL_KEEP	( ~(0x000100FFu | 0x00300000u) )

#define RFNM_VWIN_QLEN			( 512 )	/* matches the rtc ring depth; the lib's
					 * 40 ms schedule-ahead at the ~120 us window
					 * floor is ~333 in flight (64 measured qfull
					 * at 500 us x 80) */
#define RFNM_VWIN_SETUP_MARGIN_TICKS	( 6144 )	/* 100 us: GO-wake + rebuild+prime (fw 15-27 us measured) */
#define RFNM_VWIN_CLOSE_MARGIN_TICKS	( 64 )
struct rfnm_vwin {
	uint32_t open;		/* window open tick (gate edge, slot-aligned) */
	uint32_t close;		/* close tick (open + len_slots*tps - tps/4) */
	uint32_t src_slot;
	uint32_t len_slots;	/* content + the guaranteed-zero pad slot */
};
static struct rfnm_vwin rfnm_vwin_q[RFNM_VWIN_QLEN];
static uint32_t rfnm_vwin_prod, rfnm_vwin_cons;
static uint32_t rfnm_vwin_busy_until;	/* prev window's close + margin */
static uint32_t rfnm_vwin_busy_valid;
static struct hrtimer rfnm_vwin_timer;
static DEFINE_SPINLOCK(rfnm_vwin_lock);
static u64 rfnm_vwin_next_ns;		/* pump -> caller: re-fire delay (0 = none) */
static struct {
	uint32_t sent;		/* TX_WINDOW mboxes posted */
	uint32_t late;		/* dropped: send slot missed (deadline = open - margin) */
	uint32_t qfull;
	uint32_t mbox_busy;	/* send deferred: previous op unconsumed */
	uint32_t no_latch;	/* refused: fw tx_state bit5 (windowed) not latched */
	uint32_t rtc_err;	/* rtc ring push failed */
} rfnm_vwin_stats;

static inline u64 rfnm_ticks_to_ns(uint32_t ticks) {
	return div_u64((u64)ticks * 3125, 192);	/* 1/61.44 MHz = 3125/192 ns exact */
}

// Fenced VSPA host_go raise: wakes a parked (__builtin_done) core for one pass.
// Registered as rfnm_vspa_go_cb (the status-freshness consumer in la9310_rfnm rings
// it when the heartbeat block goes stale - a parked core has no ambient GO source in
// windowed-idle / TX-only shapes) and shared by the vwin fallback below.
static void rfnm_la9310_vspa_go(void)
{
	uint32_t c;

	if(!rfdev || !rfdev->ccsr_p || rfnm_la9310_mmio_fenced()) {
		return;
	}
	c = ioread32(rfdev->ccsr_p + RFNM_VSPA_CONTROL);
	iowrite32((c & RFNM_VSPA_CONTROL_KEEP) | 0x1u, rfdev->ccsr_p + RFNM_VSPA_CONTROL);
}

// send every due queue head; sets rfnm_vwin_next_ns when a future moment needs a
// re-fire (the caller starts/forwards the hrtimer - never self-started under the
// lock). Caller holds rfnm_vwin_lock. Posted writes only, no waits.
static void rfnm_vwin_pump(void)
{
	while(rfnm_vwin_cons != rfnm_vwin_prod) {
		struct rfnm_vwin *w = &rfnm_vwin_q[rfnm_vwin_cons % RFNM_VWIN_QLEN];
		uint32_t now;

		if(!rfdev || !rfdev->ccsr_p || rfnm_la9310_mmio_fenced()) {
			return;
		}
		now = rfnm_la9310_phytimer_now();
		if(rfnm_vwin_busy_valid && (int32_t)(rfnm_vwin_busy_until - now) > 0) {
			rfnm_vwin_next_ns = rfnm_ticks_to_ns(rfnm_vwin_busy_until - now);
			return;
		}
		if((int32_t)(now - (w->open - RFNM_VWIN_SETUP_MARGIN_TICKS)) > 0) {
			// the send slot is gone: the gate will cycle an unprimed window
			// (silence airs) - honest, loud, counted
			rfnm_vwin_stats.late++;
			rfnm_vwin_cons++;
			printk_ratelimited("RFNM: vwin late (open %u now %u) - window dropped\n", w->open, now);
			continue;
		}
		if(ioread32(rfdev->ccsr_p + RFNM_VSPA_MBOX_STATUS) & 0x1) {
			// previous op still unconsumed (us-class) - brief defer
			rfnm_vwin_stats.mbox_busy++;
			rfnm_vwin_next_ns = 20000;
			return;
		}
		iowrite32(0xAu << 24, rfdev->ccsr_p + RFNM_VSPA_OUT_MSB);	/* MBOX_OPC_TX_WINDOW */
		iowrite32((w->src_slot << 16) | w->len_slots, rfdev->ccsr_p + RFNM_VSPA_OUT_LSB);
		if(rfnm_vwin_go_param) {
			rfnm_la9310_vspa_go();
		}
		rfnm_vwin_busy_until = w->close + RFNM_VWIN_CLOSE_MARGIN_TICKS;
		rfnm_vwin_busy_valid = 1;
		rfnm_vwin_stats.sent++;
		rfnm_vwin_cons++;
	}
}

static enum hrtimer_restart rfnm_vwin_timer_fn(struct hrtimer *t)
{
	unsigned long fl;

	spin_lock_irqsave(&rfnm_vwin_lock, fl);
	rfnm_vwin_next_ns = 0;
	rfnm_vwin_pump();
	spin_unlock_irqrestore(&rfnm_vwin_lock, fl);
	if(rfnm_vwin_next_ns) {
		hrtimer_forward_now(t, ns_to_ktime(rfnm_vwin_next_ns));
		return HRTIMER_RESTART;
	}
	return HRTIMER_NORESTART;
}

// drop everything queued + forget pacing state (session teardown / re-arm)
static void rfnm_vwin_reset(void)
{
	unsigned long fl;

	hrtimer_cancel(&rfnm_vwin_timer);
	spin_lock_irqsave(&rfnm_vwin_lock, fl);
	rfnm_vwin_cons = rfnm_vwin_prod;
	rfnm_vwin_busy_valid = 0;
	rfnm_vwin_next_ns = 0;
	spin_unlock_irqrestore(&rfnm_vwin_lock, fl);
}

// the window-complete sink (assembly lives in la9310_rfnm.c; assigned to its
// rfnm_tx_window_cb at init - the usual dependency inversion): rtc gate edges +
// the paced VSPA send. Consume-thread process context.
static int rfnm_la9310_tx_window(uint32_t open_tick, uint32_t src_slot, uint32_t len_slots)
{
	extern volatile struct rfnm_la9310_status *rfnm_la9310_status;
	unsigned long fl;
	uint32_t tps, len_ticks;
	int ret;

	if(!rfdev || !rfdev->ccsr_p || rfnm_la9310_mmio_fenced() || !rfnm_la9310_status) {
		return -ENODEV;
	}
	if(!(rfnm_la9310_status->tx_state & 0x20)) {
		// the fw never latched the windowed apply - feeding windows against a
		// free-run pump is the stuck-queue latch killer; refuse loudly
		rfnm_vwin_stats.no_latch++;
		return -EPIPE;
	}
	tps = 128u << rfnm_la9310_status->tx_r_shift;
	// close 64 samples before the pad slot ends (the walker-era proven close:
	// last write complete, fifo holds pad zeros, no write pending at the edge)
	len_ticks = len_slots * tps - tps / 4;
	// queue capacity BEFORE the rtc push (proven: the reverse order
	// orphaned a gate entry per qfull drop - C11 cycled an unprimed window)
	spin_lock_irqsave(&rfnm_vwin_lock, fl);
	if(rfnm_vwin_prod - rfnm_vwin_cons >= RFNM_VWIN_QLEN) {
		spin_unlock_irqrestore(&rfnm_vwin_lock, fl);
		rfnm_vwin_stats.qfull++;
		return -ENOSPC;
	}
	spin_unlock_irqrestore(&rfnm_vwin_lock, fl);
	ret = rfnm_rtc_push(open_tick, RFNM_RTC_ARM_TX_WINDOW, len_ticks);
	if(ret) {
		rfnm_vwin_stats.rtc_err++;
		return ret;
	}
	rfnm_rtc_doorbell();
	spin_lock_irqsave(&rfnm_vwin_lock, fl);
	rfnm_vwin_q[rfnm_vwin_prod % RFNM_VWIN_QLEN] = (struct rfnm_vwin){
		.open = open_tick, .close = open_tick + len_ticks,
		.src_slot = src_slot, .len_slots = len_slots };
	rfnm_vwin_prod++;
	rfnm_vwin_next_ns = 0;
	rfnm_vwin_pump();
	if(rfnm_vwin_next_ns) {
		hrtimer_start(&rfnm_vwin_timer, ns_to_ktime(rfnm_vwin_next_ns), HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&rfnm_vwin_lock, fl);
	return 0;
}

static uint32_t rfnm_la9310_tx_windowed_q(void)
{
	// the applied stream word is the per-session truth: bit 16 is injected at the
	// chokepoint iff the ring was armed when THIS session's TX apply landed
	return (rfnm_session.stream_word >> 16) & 1u;
}

// step-1 debug verb (no TX path, no VSPA):
//   echo arm|disarm|flush > /sys/kernel/debug/rfnm_rtc
//   echo "win <lead_us> <len_ticks> [count] [period_ticks]" - push an ARM_TX_WINDOW train
//   cat /sys/kernel/debug/rfnm_rtc - ring telemetry + C11/alarm comparator peeks
static ssize_t rfnm_rtc_write(struct file *f, const char __user *ubuf, size_t len, loff_t *off) {
	char kbuf[96] = {0};
	int ret;

	if(len >= sizeof(kbuf) || copy_from_user(kbuf, ubuf, len)) {
		return -EFAULT;
	}
	if(!strncmp(kbuf, "arm", 3)) {
		ret = rfnm_rtc_arm();
		printk("rfnm: rtc arm -> %d\n", ret);
	} else if(!strncmp(kbuf, "disarm", 6)) {
		ret = rfnm_rtc_disarm();
		printk("rfnm: rtc disarm -> %d\n", ret);
	} else if(!strncmp(kbuf, "flush", 5)) {
		ret = rfnm_rtc_flush();
		printk("rfnm: rtc flush -> %d\n", ret);
	} else if(!strncmp(kbuf, "win ", 4)) {
		uint32_t lead_us = 0, len_ticks = 0, count = 1, period = 0, t0, k;

		if(sscanf(kbuf + 4, "%u %u %u %u", &lead_us, &len_ticks, &count, &period) < 2) {
			return -EINVAL;
		}
		if(!count || count > RFNM_RTC_RING_ENTRIES || (count > 1 && period <= len_ticks)) {
			return -EINVAL;
		}
		t0 = rfnm_la9310_phytimer_now() + (uint32_t)div_u64((u64)lead_us * 6144ull, 100);
		ret = 0;
		for(k = 0; k < count && !ret; k++) {
			ret = rfnm_rtc_push(t0 + k * period, RFNM_RTC_ARM_TX_WINDOW, len_ticks);
		}
		rfnm_rtc_doorbell();
		printk("rfnm: rtc win train %u/%u pushed (t0 %u len %u period %u) -> %d\n",
				k, count, t0, len_ticks, period, ret);
	} else {
		return -EINVAL;
	}
	return ret ? ret : len;
}

static ssize_t rfnm_rtc_read(struct file *f, char __user *ubuf, size_t len, loff_t *off) {
	extern volatile struct rfnm_la9310_status *rfnm_la9310_status;
	extern uint32_t rfnm_wa_windows, rfnm_wa_ragged, rfnm_wa_implicit, rfnm_wa_cbfail;
	struct rfnm_rtc_ring __iomem *r = (struct rfnm_rtc_ring __iomem *)rfnm_rtc_p;
	char kbuf[768];
	int n;

	if(!rfnm_rtc_p || !rfdev || !rfdev->ccsr_p || rfnm_la9310_mmio_fenced()) {
		return -ENODEV;
	}
	n = scnprintf(kbuf, sizeof(kbuf),
			"magic %08x gen %u prod %u cons %u\n"
			"exec %u miss %u rej %u refuse %u\n"
			"gen_seen %u disarms %u worst_margin %d\n"
			"last_open %u last_close %u now %u\n"
			"c11 csr %08x cnv %u\n"
			"alarm csr %08x cnv %u\n"
			"vwin sent %u late %u qfull %u mbox_busy %u no_latch %u rtc_err %u q %u\n"
			"wa win %u ragged %u implicit %u cbfail %u tx_state %02x\n",
			ioread32(&r->magic), ioread32(&r->gen), ioread32(&r->prod), ioread32(&r->cons),
			ioread32(&r->executed), ioread32(&r->missed), ioread32(&r->rejected), ioread32(&r->refused),
			ioread32(&r->gen_seen), ioread32(&r->disarms), (int32_t)ioread32(&r->worst_margin),
			ioread32(&r->last_open), ioread32(&r->last_close), rfnm_la9310_phytimer_now(),
			readl(rfdev->ccsr_p + 0x102005C), readl(rfdev->ccsr_p + 0x1020060),
			readl(rfdev->ccsr_p + 0x1020074), readl(rfdev->ccsr_p + 0x1020078),
			rfnm_vwin_stats.sent, rfnm_vwin_stats.late, rfnm_vwin_stats.qfull,
			rfnm_vwin_stats.mbox_busy, rfnm_vwin_stats.no_latch, rfnm_vwin_stats.rtc_err,
			rfnm_vwin_prod - rfnm_vwin_cons,
			rfnm_wa_windows, rfnm_wa_ragged, rfnm_wa_implicit, rfnm_wa_cbfail,
			rfnm_la9310_status ? rfnm_la9310_status->tx_state : 0xFF);
	return simple_read_from_buffer(ubuf, len, off, kbuf, n);
}
static const struct file_operations rfnm_rtc_fops = { .read = rfnm_rtc_read, .write = rfnm_rtc_write };

static __init int rfnm_lalib_init(void)
{
	rfdev = (struct rfdevice *)kzalloc(sizeof(struct rfdevice), GFP_KERNEL);

	selected_dcs_freq = 122880000;

	rfnm_pos_tx_viable_cb = rfnm_la9310_pos_tx_viable;
	rfnm_ptmr_now_cb = rfnm_la9310_phytimer_now;
	rfnm_phy_restart_notify_cb = rfnm_lalib_phy_restart_notify;
	rfnm_vspa_handoff_cb = rfnm_lalib_vspa_handoff;
	{
		extern void (*rfnm_rx18_autopsy_cb)(const char *why);
		extern uint32_t (*rfnm_arm_death_check_cb)(void);
		rfnm_rx18_autopsy_cb = rfnm_la9310_rx18_autopsy;
		rfnm_arm_death_check_cb = rfnm_la9310_arm_death_check;
	}
	{
		extern void (*rfnm_tx_health_cb)(struct rfnm_tx_health *h);
		rfnm_tx_health_cb = rfnm_la9310_tx_health;
	}

	rfnm_switch_rf_dfs = debugfs_create_file("rfnm_switch_rf", 0200, NULL, NULL, &rfnm_switch_rf_fops);
	rfnm_tdd_dfs = debugfs_create_file("rfnm_tdd", 0200, NULL, NULL, &rfnm_tdd_fops);
	rfnm_tx_fill_dfs = debugfs_create_file("rfnm_tx_fill", 0200, NULL, NULL, &rfnm_tx_fill_fops);
	rfnm_tx_window_dfs = debugfs_create_file("rfnm_tx_window", 0200, NULL, NULL, &rfnm_tx_window_fops);
	rfnm_rtc_dfs = debugfs_create_file("rfnm_rtc", 0600, NULL, NULL, &rfnm_rtc_fops);
	// the RT command ring window (the TCM gap between the M4's m_data and the HIF)
	rfnm_rtc_p = ioremap(TCML_START + RFNM_RTC_RING_HOST_OFF, sizeof(struct rfnm_rtc_ring));

	// windowed TX: the send engine's timer + the assembly sinks (dependency
	// inversion, same as the other cbs)
	hrtimer_init(&rfnm_vwin_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rfnm_vwin_timer.function = rfnm_vwin_timer_fn;
	{
		extern int (*rfnm_tx_window_cb)(uint32_t open_tick, uint32_t src_slot, uint32_t len_slots);
		extern uint32_t (*rfnm_tx_windowed_q_cb)(void);
		extern void (*rfnm_vspa_go_cb)(void);
		rfnm_tx_window_cb = rfnm_la9310_tx_window;
		rfnm_tx_windowed_q_cb = rfnm_la9310_tx_windowed_q;
		rfnm_vspa_go_cb = rfnm_la9310_vspa_go;
	}

	void *rfic_p = NULL;
	void __iomem *hif_p = ioremap(HIF_START, HIF_SIZE);
	rfdev->hif_p = hif_p;
	rfic_p = &(((struct la9310_hif *)hif_p)->rf_hif);
    rfdev->rfic_p = rfic_p;

    void __iomem *ccsr_p = ioremap(CCSR_START, CCSR_SIZE);
    rfdev->ccsr_p = ccsr_p;

	rfnm_session.stream_word = 0;
	
	si5510_i2c_dev = bus_find_device_by_name(&i2c_bus_type, NULL, "0-0058");
	if (!si5510_i2c_dev) {
		printk("Couldn't find i2c device\n");
	} else {
		si5510_i2c_client = i2c_verify_client(si5510_i2c_dev);
		if (!si5510_i2c_client) {
			printk("Couldn't find i2c client\n");
		}
	}

	rfnm_register_lalib_quiesce_cb(rfnm_la9310_quiesce);

	return 0;
}

static __exit void rfnm_lalib_exit(void) {
	// THE partial-reload oops class (root-caused): every cb this module registers
	// into la9310rfnm must be retracted before this text unloads - the phy64 timer
	// alone calls rfnm_ptmr_now_cb every second from a PREEMPT_RT timer kthread, and
	// an orphaned-session stack drives the other hooks at kHz rates; one unretracted
	// pointer = a call into freed text (Oops 86000007, kthread at a raw address -
	// reproduced 5x; the old "rmmod races the RX workqueue" story was this).
	// ptmr_now/pos_tx_viable/phy_restart were the missing three. Null everything
	// FIRST, then settle so in-flight callers complete before the text goes away.
	{
		extern uint32_t (*rfnm_ptmr_now_cb)(void);
		extern int (*rfnm_pos_tx_viable_cb)(void);
		rfnm_ptmr_now_cb = NULL;
		rfnm_pos_tx_viable_cb = NULL;
		rfnm_phy_restart_notify_cb = NULL;
		rfnm_vspa_handoff_cb = NULL;
	}
	cancel_work_sync(&rfnm_phy_restart_work);
	{
		extern int (*rfnm_tx_window_cb)(uint32_t open_tick, uint32_t src_slot, uint32_t len_slots);
		extern uint32_t (*rfnm_tx_windowed_q_cb)(void);
		extern void (*rfnm_vspa_go_cb)(void);
		rfnm_tx_window_cb = NULL;
		rfnm_tx_windowed_q_cb = NULL;
		rfnm_vspa_go_cb = NULL;
	}
	hrtimer_cancel(&rfnm_vwin_timer);
	debugfs_remove(rfnm_rtc_dfs);
	if(rfnm_rtc_p) {
		iounmap(rfnm_rtc_p);
		rfnm_rtc_p = NULL;
	}
	rfnm_register_lalib_quiesce_cb(NULL);
	{
		// a tx_health debugfs read after this module unloads must not call into
		// vanished text
		extern void (*rfnm_tx_health_cb)(struct rfnm_tx_health *h);
		rfnm_tx_health_cb = NULL;
	}
	{
		extern void (*rfnm_rx18_autopsy_cb)(const char *why);
		extern uint32_t (*rfnm_arm_death_check_cb)(void);
		rfnm_rx18_autopsy_cb = NULL;
		rfnm_arm_death_check_cb = NULL;
	}
	// settle: callers snapshot the pointer then call; anyone mid-call when the nulls
	// landed finishes inside this window (calls are µs-scale, the hottest is 1 Hz)
	msleep(20);

	// a held request whose storage vanishes with the module corrupts the QoS
	// plist later - drop it even if a stream was still configured (reload_some
	// unloads mid-session routinely)
	rfnm_cpu_qos_update(0);

	put_device(si5510_i2c_dev);
}


MODULE_PARM_DESC(device, "RFNM LA9310 Command Driver");
module_init(rfnm_lalib_init);
module_exit(rfnm_lalib_exit);
MODULE_LICENSE("GPL");
