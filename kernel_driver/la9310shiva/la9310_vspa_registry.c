/* SPDX-License-Identifier: GPL-2.0
 * RFNM VSPA kernel registry (registry-refactor/adm28): N first-class self-describing DSP
 * images, capability-selected per apply, parked-core ms swaps. Extracted from
 * la9310_main_vspa.c (registry-refactor cleanup) - vendor code stays vendor, RFNM logic
 * lives here; the shared module-internal surface is la9310_vspa_internal.h.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/rfnm-vspa.h>

#include "la9310_vspa.h"
#include "la9310_base.h"
#include "la9310_vspa_internal.h"
#include "la9310_vspa_registry.h"

/* ==== RFNM VSPA kernel registry (adm27) ====================================
 * N first-class DSP images, each self-describing via a stamped .rfnm_ident ELF
 * section. Userspace registers filenames once at boot (sysfs vspa_register);
 * the registry request_firmware's each, ABI-checks + integrity-checks it
 * (fail-closed per image), and caches the blob in kernel RAM so every later
 * load - probe after a hard reset, or the parked-core fast swap - is disk- and
 * NFS-independent and loads exactly the verified bytes. Selection is
 * capability-driven from the planner (rfnm_lalib); no flavor flags exist.
 */

#define RFNM_VSPA_REG_MAX	8

struct rfnm_vspa_reg_entry {
	char fwname[64];		/* request_firmware path, e.g. rfnm/vspa/apm-fdx.eld */
	struct rfnm_vspa_ident id;
	void *blob;
	u32 size;
};

static struct rfnm_vspa_reg_entry rfnm_vspa_reg[RFNM_VSPA_REG_MAX];
static int rfnm_vspa_reg_cnt;
static int rfnm_vspa_selected = -1;	/* index loaded at the next boot_image */
static int rfnm_vspa_running = -1;	/* index currently executing (-1 = parked/none) */
static DEFINE_MUTEX(rfnm_vspa_reg_lock);
atomic_t rfnm_vspa_boot_gen = ATOMIC_INIT(0);
EXPORT_SYMBOL_GPL(rfnm_vspa_boot_gen);
/* set while the loader owns the VSPA DMA engine (channel 0): rfnm_qec stands off -
 * its concurrent channel-0 DMAs corrupted live-core loads (the fast-swap EIO) */
atomic_t rfnm_vspa_loader_busy = ATOMIC_INIT(0);
EXPORT_SYMBOL_GPL(rfnm_vspa_loader_busy);

static u32 rfnm_vspa_wordsum(const u8 *d, u32 size)
{
	u32 csum = 0, ci;

	for (ci = 0; ci + 3 < size; ci += 4)
		csum += le32_to_cpu(*(const __le32 *)(d + ci));
	for (; ci < size; ci++)
		csum += d[ci];
	return csum;
}

/* minimal bounds-checked ELF32-LE section walk to find .rfnm_ident in a blob */
static const struct rfnm_vspa_ident *rfnm_vspa_find_ident(const u8 *d, u32 size)
{
	u32 shoff, shnum, shentsz, shstrndx, stroff, strsz, i;

	if (size < 0x34 || d[0] != 0x7f || d[1] != 'E' || d[2] != 'L' || d[3] != 'F')
		return NULL;
	if (d[4] != 1 || d[5] != 1)	/* ELF32, little-endian */
		return NULL;
	shoff = le32_to_cpu(*(const __le32 *)(d + 0x20));
	shentsz = le16_to_cpu(*(const __le16 *)(d + 0x2E));
	shnum = le16_to_cpu(*(const __le16 *)(d + 0x30));
	shstrndx = le16_to_cpu(*(const __le16 *)(d + 0x32));
	if (!shoff || shentsz < 0x28 || !shnum || shstrndx >= shnum)
		return NULL;
	if ((u64)shoff + (u64)shnum * shentsz > size)
		return NULL;
	stroff = le32_to_cpu(*(const __le32 *)(d + shoff + (u64)shstrndx * shentsz + 0x10));
	strsz = le32_to_cpu(*(const __le32 *)(d + shoff + (u64)shstrndx * shentsz + 0x14));
	if ((u64)stroff + strsz > size)
		return NULL;
	for (i = 0; i < shnum; i++) {
		u32 nameoff = le32_to_cpu(*(const __le32 *)(d + shoff + (u64)i * shentsz));
		u32 off = le32_to_cpu(*(const __le32 *)(d + shoff + (u64)i * shentsz + 0x10));
		u32 sz = le32_to_cpu(*(const __le32 *)(d + shoff + (u64)i * shentsz + 0x14));

		if (nameoff >= strsz)
			continue;
		if (strncmp((const char *)d + stroff + nameoff, RFNM_VSPA_IDENT_SECTION,
			    strsz - nameoff) != 0)
			continue;
		if (sz < sizeof(struct rfnm_vspa_ident) || (u64)off + sz > size)
			return NULL;
		return (const struct rfnm_vspa_ident *)(d + off);
	}
	return NULL;
}

int rfnm_vspa_registry_register(const char *fwname)
{
	struct la9310_dev *la9310_dev = get_la9310_dev_byname("nlm0");
	const struct firmware *fw;
	const struct rfnm_vspa_ident *id;
	struct rfnm_vspa_reg_entry *e;
	int rc, i;

	if (!la9310_dev)
		return -ENODEV;
	if (strlen(fwname) >= sizeof(e->fwname))
		return -ENAMETOOLONG;

	rc = request_firmware(&fw, fwname, la9310_dev->dev);
	if (rc) {
		dev_err(la9310_dev->dev, "vspa registry: %s: request failed (%d)\n", fwname, rc);
		return rc;
	}
	id = rfnm_vspa_find_ident(fw->data, fw->size);
	if (!id || le32_to_cpu(id->magic) != RFNM_VSPA_IDENT_MAGIC) {
		dev_err(la9310_dev->dev, "vspa registry: %s REFUSED: no .rfnm_ident section (unstamped image)\n", fwname);
		rc = -EINVAL;
		goto out;
	}
	if (le32_to_cpu(id->abi) != RFNM_VSPA_ABI_CURRENT) {
		dev_err(la9310_dev->dev, "vspa registry: %s REFUSED: abi %u != driver abi %u (deploy-together violation)\n",
			fwname, le32_to_cpu(id->abi), RFNM_VSPA_ABI_CURRENT);
		rc = -EINVAL;
		goto out;
	}
	if (le32_to_cpu(id->orig_size) > fw->size ||
	    rfnm_vspa_wordsum(fw->data, le32_to_cpu(id->orig_size)) != le32_to_cpu(id->payload_csum)) {
		dev_err(la9310_dev->dev, "vspa registry: %s REFUSED: payload csum mismatch (corrupt image)\n", fwname);
		rc = -EINVAL;
		goto out;
	}

	mutex_lock(&rfnm_vspa_reg_lock);
	for (i = 0; i < rfnm_vspa_reg_cnt; i++) {
		if (!strcmp(rfnm_vspa_reg[i].fwname, fwname))
			break;
	}
	if (i == rfnm_vspa_reg_cnt && rfnm_vspa_reg_cnt == RFNM_VSPA_REG_MAX) {
		mutex_unlock(&rfnm_vspa_reg_lock);
		rc = -ENOSPC;
		goto out;
	}
	e = &rfnm_vspa_reg[i];
	kvfree(e->blob);
	e->blob = kvmalloc(fw->size, GFP_KERNEL);
	if (!e->blob) {
		mutex_unlock(&rfnm_vspa_reg_lock);
		rc = -ENOMEM;
		goto out;
	}
	memcpy(e->blob, fw->data, fw->size);
	e->size = fw->size;
	e->id = *id;
	strscpy(e->fwname, fwname, sizeof(e->fwname));
	if (i == rfnm_vspa_reg_cnt) {
		/* publish protocol: entry fields land before the count that makes them
		 * visible to the lock-free readers (rfnm_vspa_reg_count + smp_rmb) */
		smp_wmb();
		WRITE_ONCE(rfnm_vspa_reg_cnt, rfnm_vspa_reg_cnt + 1);
	}
	mutex_unlock(&rfnm_vspa_reg_lock);

	dev_info(la9310_dev->dev, "vspa registry[%d]: %s abi %u caps %02x rank %u %.24s (%u bytes, csum %08x)\n",
		 i, e->id.name, le32_to_cpu(e->id.abi), le32_to_cpu(e->id.caps),
		 le32_to_cpu(e->id.rank), e->id.githash, e->size,
		 le32_to_cpu(e->id.payload_csum));
	rc = 0;
out:
	release_firmware(fw);
	return rc;
}
EXPORT_SYMBOL_GPL(rfnm_vspa_registry_register);

/* lock-free reader side of the publish protocol (entries are append-only) */
static int rfnm_vspa_reg_count(void)
{
	int n = READ_ONCE(rfnm_vspa_reg_cnt);

	smp_rmb();
	return n;
}

static int rfnm_vspa_provides(int i, u32 caps)
{
	return i >= 0 && i < rfnm_vspa_reg_count() &&
	       (le32_to_cpu(rfnm_vspa_reg[i].id.caps) & caps) == caps;
}

int rfnm_vspa_registry_has(u32 caps)
{
	int i;

	for (i = 0; i < rfnm_vspa_reg_count(); i++) {
		if (rfnm_vspa_provides(i, caps))
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(rfnm_vspa_registry_has);

int rfnm_vspa_running_has(u32 caps)
{
	return rfnm_vspa_provides(rfnm_vspa_running, caps);
}
EXPORT_SYMBOL_GPL(rfnm_vspa_running_has);

int rfnm_vspa_parked(void)
{
	return rfnm_vspa_running < 0;
}
EXPORT_SYMBOL_GPL(rfnm_vspa_parked);

/* live STATUS busy bit - the swap orchestration must NOT send a quiesce word to an
 * already-parked core: a parked (done) core never reads its inbox, so the M4 waits
 * forever for a reply and its pending read then EATS the next kernel's Boot
 * Complete (the round-14 class; the userspace tool works because it never sends
 * words at all) */
int rfnm_vspa_core_busy(void)
{
	struct la9310_dev *la9310_dev = get_la9310_dev_byname("nlm0");
	struct vspa_device *vspadev;

	if (!la9310_dev || !la9310_dev->vspa_priv)
		return 0;
	vspadev = (struct vspa_device *)la9310_dev->vspa_priv;
	return !!(vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET) & (1u << 8));
}
EXPORT_SYMBOL_GPL(rfnm_vspa_core_busy);

/* prefer the running image if sufficient (no gratuitous swaps); else the minimal
 * provider by caps popcount, tie-break by lower rank then registration order */
int rfnm_vspa_select_for(u32 caps)
{
	int i, best = -1;

	mutex_lock(&rfnm_vspa_reg_lock);
	if (rfnm_vspa_provides(rfnm_vspa_running, caps)) {
		rfnm_vspa_selected = rfnm_vspa_running;
		mutex_unlock(&rfnm_vspa_reg_lock);
		return 0;
	}
	for (i = 0; i < rfnm_vspa_reg_cnt; i++) {
		if (!rfnm_vspa_provides(i, caps))
			continue;
		if (best < 0 ||
		    hweight32(le32_to_cpu(rfnm_vspa_reg[i].id.caps)) <
		    hweight32(le32_to_cpu(rfnm_vspa_reg[best].id.caps)) ||
		    (hweight32(le32_to_cpu(rfnm_vspa_reg[i].id.caps)) ==
		     hweight32(le32_to_cpu(rfnm_vspa_reg[best].id.caps)) &&
		     le32_to_cpu(rfnm_vspa_reg[i].id.rank) <
		     le32_to_cpu(rfnm_vspa_reg[best].id.rank)))
			best = i;
	}
	rfnm_vspa_selected = best;
	mutex_unlock(&rfnm_vspa_reg_lock);
	if (best < 0) {
		pr_err("RFNM vspa registry: no registered kernel provides caps %02x (%d registered)\n",
		       caps, rfnm_vspa_reg_cnt);
		return -ENOENT;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(rfnm_vspa_select_for);

/* adm27c swap forensics: answer "where does it get stuck" from the failure itself.
 * (a) the fw's age heartbeat in the DDR status mirror: ticking = the core RUNS and
 * only the handshake was missed; frozen = truly dead. (b) full register snapshot
 * incl. DBG_RCSTATUS (BAR-visible run-control state - TAP-free eyes). (c) late
 * Boot Complete check: did 0xF1000000 land after startup() gave up? */
static void rfnm_vspa_swap_forensics(struct la9310_dev *la9310_dev)
{
	struct vspa_device *vspadev = (struct vspa_device *)la9310_dev->vspa_priv;
	volatile u32 *mirror;
	u32 age0 = 0, age1 = 0, mbox, msb = 0, lsb = 0;

	mirror = (volatile u32 *)memremap(RFNM_IQFLOOD_STATUS_MEMADDR, SZ_4K, MEMREMAP_WC);
	if (mirror)
		age0 = mirror[0];
	dev_err(la9310_dev->dev, "swap forensics t0: ctrl %08x stat %08x mbox %08x extgo %08x ippu %08x gostat %08x rcstat %08x age %u\n",
		vspa_reg_read(vspadev->regs + CONTROL_REG_OFFSET),
		vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET),
		vspa_reg_read(vspadev->regs + HOST_MBOX_STATUS_REG_OFFSET),
		vspa_reg_read(vspadev->regs + EXT_GO_STATUS_REG_OFFSET),
		vspa_reg_read(vspadev->regs + IPPU_STATUS_REG_OFFSET),
		vspa_reg_read(vspadev->regs + DMA_GO_STAT_REG_OFFSET),
		vspa_reg_read(vspadev->regs + DBG_RCSTATUS_REG_OFFSET), age0);
	msleep(500);
	if (mirror) {
		age1 = mirror[0];
		memunmap((void *)mirror);
	}
	mbox = vspa_reg_read(vspadev->regs + HOST_MBOX_STATUS_REG_OFFSET);
	if (mbox & MBOX_STATUS_IN_64_BIT) {
		msb = vspa_reg_read(vspadev->regs + HOST_IN_64_MSB_REG_OFFSET);
		lsb = vspa_reg_read(vspadev->regs + HOST_IN_64_LSB_REG_OFFSET);
	}
	dev_err(la9310_dev->dev, "swap forensics t+500ms: age %u -> %u (%s), mbox %08x%s%08x:%08x, rcstat %08x stat %08x\n",
		age0, age1, age0 == age1 ? "FROZEN - core dead" : "TICKING - core ALIVE, handshake missed",
		mbox, (mbox & MBOX_STATUS_IN_64_BIT) ? " LATE-MSG " : " no-msg ", msb, lsb,
		vspa_reg_read(vspadev->regs + DBG_RCSTATUS_REG_OFFSET),
		vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET));
}

/* common post-boot tail (stats once per probe, mailbox clears, VSPA_READY) */
void rfnm_vspa_boot_tail(struct la9310_dev *la9310_dev)
{
	struct vspa_device *vspadev = (struct vspa_device *)la9310_dev->vspa_priv;

	if (!vspadev->rfnm_stats_ready) {
		if (la9310_vspa_stats_init(la9310_dev) == 0)
			vspadev->rfnm_stats_ready = 1;
	}
	vspa_reg_write(vspadev->regs + HOST_FLAGS0_REG_OFFSET, 0xFFFFFFFFUL);
	vspa_reg_write(vspadev->regs + HOST_FLAGS1_REG_OFFSET, 0xFFFFFFFFUL);
	vspa_reg_write(vspadev->regs + STATUS_REG_OFFSET, 0xF000);
	dma_wmb();
	la9310_set_host_ready(la9310_dev, LA9310_HIF_STATUS_VSPA_READY);
}

/* Boot one cached image: halt VCPU, zeroise, load, replay the boot handshake.
 * This is both the first-load path and the parked-core fast swap (no PCIe
 * teardown, no chip reset, no phytimer restart). The parked-core contract makes
 * it safe: every host command is one GO -> done cycle, so a quiesced core holds
 * no live PC state; VCPU_RESET covers the mid-run case (fastswap POC, dsp skill). */
static int rfnm_vspa_boot_image_locked(struct la9310_dev *la9310_dev, int idx)
{
	struct vspa_device *vspadev = (struct vspa_device *)la9310_dev->vspa_priv;
	struct rfnm_vspa_reg_entry *e = &rfnm_vspa_reg[idx];
	u64 t0 = ktime_get_ns();
	int rc;

	/* owner's law (adm27b): the VSPA is a simple cache-less CPU - a PARKED core
	 * (STATUS busy=0, done-state) starts a new image cleanly on program write + GO,
	 * no core reset needed or wanted. Force-halting a RUNNING core (VCPU_RESET
	 * mid-burst) is what wedged the live-swap rounds (EIO x2, one SoC hang). The
	 * caller's quiesce word parks the fw; wait for the park here and REFUSE if it
	 * never comes (the caller falls back to the chip reset) - the old core keeps
	 * running untouched on refusal. */
	{
		int tries;

		for (tries = 0; tries < 250; tries++) {
			if (!(vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET) & (1u << 8)))
				break;
			usleep_range(2000, 3000);
		}
		if (vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET) & (1u << 8)) {
			dev_warn(la9310_dev->dev, "vspa registry: %s not booted - core did not park after quiesce (status %08x); old kernel keeps running\n",
				 e->id.name, vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET));
			return -EBUSY;
		}
	}

	/* round-12 mimic of the PROVEN userspace fastswap (works TODAY, 303 us, on this
	 * exact board state - receipted with ctrl 03000000 / stat 00000001 present, no
	 * core reset, no zeroise, no mailbox writes): park + section DMA + GO strobe,
	 * nothing else. GO reads 0 on the parked core (self-clearing strobe; the done
	 * state re-enters at entry per the execution model - the June A/B receipts).
	 * Kept beyond the tool's shape: the qec channel-0 exclusion (a proven race)
	 * and DMA COMP/XFR/CFG W1C (the tool clears ch0 per transfer; our loader
	 * otherwise polls a stale completion). Removed vs rounds 3-11 (all proven
	 * non-load-bearing or suspect by the tool's success): VCPU_RESET, zeroise,
	 * inbox drains, HOST_FLAGS writes, STATUS W1C. */
	atomic_set(&rfnm_vspa_loader_busy, 1);
	usleep_range(3000, 5000);
	vspa_reg_write(vspadev->regs + DMA_COMP_STAT_REG_OFFSET, 0xFFFFFFFF);
	vspa_reg_write(vspadev->regs + DMA_XFRERR_STAT_REG_OFFSET, 0xFFFFFFFF);
	vspa_reg_write(vspadev->regs + DMA_CFGERR_STAT_REG_OFFSET, 0xFFFFFFFF);
	vspadev->state = VSPA_STATE_LOADING;

	{
		/* NXP loader path (round 12/13 verdict: this load is GOOD - the image
		 * provably executes; round 13's from-overlay-window DMA was the hang) */
		struct la9310_mem_region_info *fwr =
			la9310_get_dma_region(la9310_dev, LA9310_VSPA_OVERLAY);
		char *vaddr;

		if (!fwr) {
			rc = -ENODEV;
			goto fail;
		}
		vaddr = PTR_ALIGN(fwr->vaddr, vspadev->hardware.axi_data_width);
		memcpy_toio(vaddr, e->blob, e->size);
		rc = la9310_load_vspa_image(la9310_dev, vaddr, e->size);
		if (rc < 0)
			goto fail;
	}
	/* adm27c symmetric snapshot: identical line on fresh-probe successes and swap
	 * failures - any bit differing between the two contexts is the blocker */
	dev_info(la9310_dev->dev, "pre-GO regs: ctrl %08x stat %08x mbox %08x rcstat %08x gdben %08x dvr %08x extgo %08x\n",
		 vspa_reg_read(vspadev->regs + CONTROL_REG_OFFSET),
		 vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET),
		 vspa_reg_read(vspadev->regs + HOST_MBOX_STATUS_REG_OFFSET),
		 vspa_reg_read(vspadev->regs + DBG_RCSTATUS_REG_OFFSET),
		 vspa_reg_read(vspadev->regs + DBG_GDBEN_REG_OFFSET),
		 vspa_reg_read(vspadev->regs + DBG_DVR_REG_OFFSET),
		 vspa_reg_read(vspadev->regs + EXT_GO_STATUS_REG_OFFSET));
	rc = startup(la9310_dev);
	if (rc) {
		rfnm_vspa_swap_forensics(la9310_dev);
		goto fail;
	}
	strscpy(vspadev->eld_filename, e->id.name, VSPA_MAX_ELD_FILENAME);
	rfnm_vspa_boot_tail(la9310_dev);
	rfnm_vspa_running = idx;
	atomic_inc(&rfnm_vspa_boot_gen);
	atomic_set(&rfnm_vspa_loader_busy, 0);
	dev_info(la9310_dev->dev, "vspa BOOTED kernel %s (caps %02x, %.16s, gen %d, %llu us)\n",
		 e->id.name, le32_to_cpu(e->id.caps), e->id.githash,
		 atomic_read(&rfnm_vspa_boot_gen),
		 (unsigned long long)((ktime_get_ns() - t0) / 1000));
	return 0;
fail:
	rfnm_vspa_running = -1;
	atomic_set(&rfnm_vspa_loader_busy, 0);
	dev_err(la9310_dev->dev, "vspa registry: boot of %s FAILED (%d: xfrerr %08x cfgerr %08x ctrl %08x stat %08x mbox %08x)\n",
		e->id.name, rc,
		vspa_reg_read(vspadev->regs + DMA_XFRERR_STAT_REG_OFFSET),
		vspa_reg_read(vspadev->regs + DMA_CFGERR_STAT_REG_OFFSET),
		vspa_reg_read(vspadev->regs + CONTROL_REG_OFFSET),
		vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET),
		vspa_reg_read(vspadev->regs + HOST_MBOX_STATUS_REG_OFFSET));
	return rc;
}

/* probe-time bring-up from the registry: selects a STREAMING_BASE image if none
 * selected yet (post-hard-reset reprobes keep the previous selection) */
int rfnm_vspa_probe_boot(struct la9310_dev *la9310_dev)
{
	int rc;

	mutex_lock(&rfnm_vspa_reg_lock);
	if (rfnm_vspa_selected < 0 || rfnm_vspa_selected >= rfnm_vspa_reg_cnt) {
		int i, best = -1;

		for (i = 0; i < rfnm_vspa_reg_cnt; i++) {
			if (rfnm_vspa_provides(i, RFNM_VSPA_CAP_STREAMING_BASE) &&
			    (best < 0 ||
			     hweight32(le32_to_cpu(rfnm_vspa_reg[i].id.caps)) <
			     hweight32(le32_to_cpu(rfnm_vspa_reg[best].id.caps))))
				best = i;
		}
		rfnm_vspa_selected = best;
	}
	if (rfnm_vspa_selected < 0) {
		mutex_unlock(&rfnm_vspa_reg_lock);
		return -ENOENT;
	}
	rfnm_vspa_running = -1;	/* fresh endpoint: nothing runs until boot succeeds */
	rc = rfnm_vspa_boot_image_locked(la9310_dev, rfnm_vspa_selected);
	mutex_unlock(&rfnm_vspa_reg_lock);
	return rc;
}

int rfnm_vspa_registry_snapshot(char *buf, size_t sz)
{
	int i, n = 0;

	mutex_lock(&rfnm_vspa_reg_lock);
	for (i = 0; i < rfnm_vspa_reg_cnt; i++) {
		n += scnprintf(buf + n, sz - n, "%d%s%s %s abi %u caps %02x rank %u %.24s %u bytes csum %08x\n",
			       i, i == rfnm_vspa_running ? " RUNNING" : "",
			       i == rfnm_vspa_selected ? " SELECTED" : "",
			       rfnm_vspa_reg[i].id.name,
			       le32_to_cpu(rfnm_vspa_reg[i].id.abi),
			       le32_to_cpu(rfnm_vspa_reg[i].id.caps),
			       le32_to_cpu(rfnm_vspa_reg[i].id.rank),
			       rfnm_vspa_reg[i].id.githash,
			       rfnm_vspa_reg[i].size,
			       le32_to_cpu(rfnm_vspa_reg[i].id.payload_csum));
	}
	if (!rfnm_vspa_reg_cnt)
		n += scnprintf(buf + n, sz - n, "(registry empty)\n");
	mutex_unlock(&rfnm_vspa_reg_lock);
	return n;
}
EXPORT_SYMBOL_GPL(rfnm_vspa_registry_snapshot);

/* Boot the SELECTED image now: halt VCPU, zeroise, load from the cached blob,
 * replay the boot handshake. This is both the first-load path and the parked-core
 * fast swap (no PCIe teardown, no chip reset, no phytimer restart, ~ms). The
 * caller (la9310rfnm swap orchestration) owns stream quiesce around it. */
int rfnm_vspa_boot_selected(void)
{
	struct la9310_dev *la9310_dev = get_la9310_dev_byname("nlm0");
	int rc;

	if (!la9310_dev || !la9310_dev->vspa_priv)
		return -ENODEV;
	mutex_lock(&rfnm_vspa_reg_lock);
	if (rfnm_vspa_selected < 0) {
		mutex_unlock(&rfnm_vspa_reg_lock);
		return -ENOENT;
	}
	rc = rfnm_vspa_boot_image_locked(la9310_dev, rfnm_vspa_selected);
	mutex_unlock(&rfnm_vspa_reg_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(rfnm_vspa_boot_selected);

