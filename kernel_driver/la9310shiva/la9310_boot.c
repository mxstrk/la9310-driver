/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017, 2021-2023 NXP
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/version.h>

#include "la9310_pci.h"
#include "la9310_base.h"

int la9310_do_reset_handshake(struct la9310_dev *la9310_dev)
{
#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
	int rc = 0;
#else
	int rc = 0, retries = LA9310_HOST_BOOT_HSHAKE_RETRIES;

#endif
	struct la9310_ccsr_dcr *ccsr_dcr;
	u32 scratch_val;
	u32 *scratch_reg;

	ccsr_dcr = (struct la9310_ccsr_dcr *)
			(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr
			+ DCR_OFFSET);
	scratch_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_SCRATCH_REG];
	writel(LA9310_HOST_COMPLETE_CLOCK_CONFIG, scratch_reg);
	dma_wmb();

	dev_dbg(la9310_dev->dev,
		 "[Reset HS] Waiting for FreeRTOS to write %d\n",
		  LA9310_HOST_START_DRIVER_INIT);

#ifdef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
	set_current_state(TASK_INTERRUPTIBLE);
	/* Wait for FreeRTOS to complete reset hand shake */
	schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
	dma_rmb();
	scratch_val = readl(scratch_reg);

	while ((scratch_val != LA9310_HOST_START_DRIVER_INIT) && retries) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(
			LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
		retries--;
		dma_rmb();
		scratch_val = readl(scratch_reg);
	}
#else
	/*waiting for interrupt from la9310 to do handshake*/
	wait_for_completion_timeout(&ScratchRegisterHandshake,
				    msecs_to_jiffies
				    (LA9310_HOST_BOOT_HSHAKE_TIMEOUT));


	scratch_val = readl(scratch_reg);
#endif
	if (scratch_val != LA9310_HOST_START_DRIVER_INIT) {
		dev_err(la9310_dev->dev,
			"LA9310 Reset HSHAKE failed, scratch 0x%x\n",
			scratch_val);
		rc = -EINVAL;
	} else {
		dev_dbg(la9310_dev->dev,
			 "LA9310 Reset HSHAKE done, scratch 0x%x",
			 scratch_val);
	}

	return rc;
}

static int la9310_wait_for_boot_scratch(struct la9310_dev *la9310_dev, u32 *scratch_reg, u32 expected, u32 *scratch_val) {
	unsigned long timeout_ms = LA9310_HOST_BOOT_HSHAKE_TIMEOUT * (LA9310_HOST_BOOT_HSHAKE_RETRIES + 1);
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	u32 val;

	for (;;) {
		dma_rmb();
		val = readl(scratch_reg);
		if (val == expected) {
			*scratch_val = val;
			return 0;
		}

		if (time_after(jiffies, deadline)) {
			*scratch_val = val;
			return -ETIMEDOUT;
		}

		msleep(LA9310_HOST_BOOT_POLL_INTERVAL);
	}
}

int la9310_load_rtos_img(struct la9310_dev *la9310_dev)
{
	int rc = 0;
	struct la9310_mem_region_info *tcm_region;
	struct la9310_mem_region_info *dma_region;
	struct la9310_mem_region_info *ccsr_region;
	struct la9310_ccsr_dcr *ccsr_dcr;
	u32 scratch_val;
	u32 *scratch_reg;
	u8 * __iomem ccs_dcr_ptr;
	struct la9310_boot_header __iomem *boot_header;
	int size, fw_size;
	u32 ep_dma_offset, rsvd_ctrl = 0;
	dma_addr_t dma_addr;


#if NXP_ERRATUM_A008822
	u32 pcie_amba_err_offset;
#endif
	ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];
	ccs_dcr_ptr = ccsr_region->vaddr + DCR_OFFSET;
	ccsr_dcr = (struct la9310_ccsr_dcr *) ccs_dcr_ptr;

	tcm_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU];
#if NXP_ERRATUM_A008822
       /*Before writing boot header,write 0000_9401h at ccsr adrress
		0x34008d0h*/
	pcie_amba_err_offset = PCIE_RHOM_DBI_BASE +
				AMBA_ERROR_RESPONSE_DEFAULT_OFF;
	writel(AMBA_ERROR_RESPONSE_DEFAULT_VALUE, ccsr_region->vaddr
						+pcie_amba_err_offset);
#endif
	/* ROM code polls the contents of offset 0x0 from the TCM Data Memory
	 * base address, 0x2000_0000 for a valid Boot Header Preamble.
	 */
	boot_header = (struct la9310_boot_header *)((u8 *)tcm_region->vaddr +
		LA9310_EP_BOOT_HDR_OFFSET);
	la9310_dev->boot_header = boot_header;

	dma_region = la9310_get_dma_region(la9310_dev, LA9310_MEM_REGION_FW);
	size = dma_region->size;
	rc = la9310_dev_reserve_firmware(la9310_dev);
	if (rc)
		goto out;
	rc  = la9310_udev_load_firmware(la9310_dev, dma_region->vaddr,
				     size, FIRMWARE_RTOS);
	la9310_dev_free_firmware(la9310_dev);
	if (rc) {
		dev_err(la9310_dev->dev, "udev Firmware [%s] request failed\n",
			 FIRMWARE_RTOS);
		goto out;
	}

	fw_size =  la9310_dev->firmware_info.size;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
	dma_addr=dma_map_page_attrs(&la9310_dev->pdev->dev,
			virt_to_page(dma_region->vaddr),
			offset_in_page(dma_region->vaddr), fw_size,
			(enum dma_data_direction)DMA_TO_DEVICE, 0);
	dev_dbg(la9310_dev->dev,"### dma_region->vaddr %px dma_addr %px\n",dma_region->vaddr, (void*)dma_addr);

#else
	pci_map_single(la9310_dev->pdev, dma_region->vaddr,
			fw_size, PCI_DMA_TODEVICE);
#endif
 
	dev_dbg(la9310_dev->dev, "udev Firmware [%s] - Addr %px, size %d\n",
		FIRMWARE_RTOS, dma_region->vaddr, fw_size);

	dev_dbg(la9310_dev->dev, "BootHDR: bl_src_offset [0x%p]: 0x%llx\n",
		&boot_header->bl_src_offset, dma_region->phys_addr);

	dev_dbg(la9310_dev->dev, "BootHDR: bl_size [0x%p]: %d\n",
		&boot_header->bl_size, fw_size);

	dev_dbg(la9310_dev->dev, "BootHDR: preamble [0x%p]: 0x%x\n",
		&boot_header->preamble, PREAMBLE);

	/*Update Firmware pointers + size in boot header */
	ep_dma_offset = LA9310_EP_DMA_PHYS_OFFSET(dma_region->phys_addr);
	writel(ep_dma_offset, &boot_header->bl_src_offset);
	writel(fw_size, &boot_header->bl_size);
	writel(LA9310_EP_FREERTOS_LOAD_ADDR, &boot_header->bl_dest);
	writel(LA9310_EP_FREERTOS_LOAD_ADDR, &boot_header->bl_entry);

	/* Write reserved to tell bootrom to pull Image using memcpy instead of
	 * edma
	 */
	rsvd_ctrl = LA9310_BOOT_HDR_BYPASS_BOOT_PLUGIN;
#if !defined(BOOTROM_USE_EDMA)
	rsvd_ctrl |= LA9310_BOOT_HDR_BYPASS_BOOT_EDMA;
#endif
	writel(rsvd_ctrl, &boot_header->reserved);
	dma_wmb();
	writel(PREAMBLE, &boot_header->preamble);

	scratch_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_SCRATCH_REG];
	dev_dbg(la9310_dev->dev,
		"Waiting for FreeRTOS to write %d\n",
		LA9310_HOST_START_CLOCK_CONFIG);

	rc = la9310_wait_for_boot_scratch(la9310_dev, scratch_reg, LA9310_HOST_START_CLOCK_CONFIG, &scratch_val);
	if (rc) {
		dev_err(la9310_dev->dev, "LA9310 FreeRTOS boot failed: 0x%x\n",
			scratch_val);
		rc = -EINVAL;
		goto out;
	} else {
		dev_dbg(la9310_dev->dev,
			 "LA9310 FreeRTOS booted succesfully: 0x%x",
			 scratch_val);
	}

out:
	return rc;
}
