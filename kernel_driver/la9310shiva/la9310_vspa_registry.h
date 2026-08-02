/* SPDX-License-Identifier: GPL-2.0
 * RFNM VSPA kernel registry (adm27): N first-class DSP images, self-describing via a
 * .rfnm_ident ELF section, selected per-apply by capability need. Replaces the
 * /rfnm/fdx_kernel flag + apm.eld copy-assembly design (configuration by artifact
 * identity + attach-time handshake, never by side-channel flags).
 *
 * The .rfnm_ident struct is stamped into finished .eld files host-side by
 * the SDK eld_ident tool (appended section; loadable content byte-identical).
 * Layout is a frozen contract between that tool and this header.
 */
#ifndef __LA9310_VSPA_REGISTRY_H__
#define __LA9310_VSPA_REGISTRY_H__

#include <linux/types.h>

#define RFNM_VSPA_IDENT_MAGIC	0x52464E49u
#define RFNM_VSPA_IDENT_SECTION	".rfnm_ident"
/* ABI 1 = the stream-word / status-mirror / ring contract of the 3bea8971 (stock) and
 * 80a19769 (FDX) lineages. Bump on any breaking fw<->driver contract change; the
 * registry refuses images whose abi != RFNM_VSPA_ABI_CURRENT (fail-closed). */
#define RFNM_VSPA_ABI_CURRENT	1

/* capability bits (fw<->kernel taxonomy; planner requirements map onto these) */
#define RFNM_VSPA_CAP_STREAMING_BASE	(1u << 0)
#define RFNM_VSPA_CAP_DEEP_DECIM	(1u << 1)
#define RFNM_VSPA_CAP_FDX_CONCURRENT	(1u << 2)
#define RFNM_VSPA_CAP_CALIB		(1u << 3)

struct rfnm_vspa_ident {
	u32 magic;
	u32 abi;
	u32 caps;
	u32 rank;
	char name[32];
	char githash[24];
	char buildtime[24];
	u32 payload_csum;	/* 32-bit LE word-sum over the ORIGINAL (pre-stamp) file bytes */
	u32 orig_size;		/* size of the original file the csum covers */
} __packed;

/* shiva-exported registry surface (callers: rfnm_lalib planner, la9310rfnm swap
 * orchestration, rfnm_qec boot-generation watch) */
int rfnm_vspa_registry_register(const char *fwname);
int rfnm_vspa_registry_has(u32 caps);	/* any REGISTERED image provides all of caps */
int rfnm_vspa_running_has(u32 caps);	/* the RUNNING image provides all of caps */
int rfnm_vspa_select_for(u32 caps);	/* pick + mark selected (loaded at next boot_image) */
int rfnm_vspa_boot_selected(void);	/* halt+zeroise+load+startup the selected image NOW */
int rfnm_vspa_parked(void);		/* 1 = no image has ever booted on this endpoint */
int rfnm_vspa_core_busy(void);		/* live STATUS busy bit (1 = core running, 0 = parked) */
int rfnm_vspa_registry_snapshot(char *buf, size_t sz);	/* sysfs table */
extern atomic_t rfnm_vspa_boot_gen;	/* bumps on every VSPA image (re)boot; qec re-verifies on change */

#endif
