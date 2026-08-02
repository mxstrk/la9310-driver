/* SPDX-License-Identifier: GPL-2.0
 * Module-internal surface shared between the NXP VSPA driver (la9310_main_vspa.c)
 * and the RFNM kernel registry (la9310_vspa_registry.c). Nothing here is exported;
 * these are shiva-module-internal symbols only (registry-refactor extraction).
 */
#ifndef __LA9310_VSPA_INTERNAL_H__
#define __LA9310_VSPA_INTERNAL_H__

struct la9310_dev;
struct vspa_device;
struct vspa_dma_req;

#define CONTROL_REG_MASK	(~0x000100FF)
#define CONTROL_PDN_EN		(1<<31)
#define CONTROL_HOST_MSG_GO	(1<<20 | 1<<21 | 1<<22 | 1<<23)
#define CONTROL_VCPU_RESET	(1<<16)
#define CONTROL_DEBUG_MSG_GO	(1<<5)
#define CONTROL_IPPU_GO		(1<<1)
#define CONTROL_HOST_GO		(1<<0)

int startup(struct la9310_dev *la9310_dev);
int vspa_mem_initialization(struct vspa_device *vspadev);
int la9310_load_vspa_image(struct la9310_dev *la9310_dev, char *vaddr, int vspa_fw_size);
int dma_raw_transmit(struct vspa_device *vspadev, struct vspa_dma_req *dr);
int la9310_vspa_stats_init(struct la9310_dev *la9310_dev);

/* registry-side entry points the NXP probe path uses (module-internal) */
int rfnm_vspa_probe_boot(struct la9310_dev *la9310_dev);
void rfnm_vspa_boot_tail(struct la9310_dev *la9310_dev);

#endif
