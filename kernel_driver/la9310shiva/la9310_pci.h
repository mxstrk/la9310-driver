/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017, 2021 NXP
 */

#ifndef _PCI_UTILITIES_H
#define _PCI_UTILITIES_H

#include <linux/device.h>
#include <linux/pci.h>
#include <linux/atomic.h>
#include "la9310_pci_def.h"

extern struct list_head pcidev_list;

/* RFNM: LA9310 MMIO fence - raised across the hard-reprobe link-down window (rfnm_la9310_hard_reprobe).
 * While it is set, the PCIe path to the LA9310 window (BAR0/CCSR/TCML, phys 0x18000000+) is dead: a CPU
 * read or write of that window never completes and hard-hangs the whole SoC (silent serial, power-cycle
 * only). Every LA9310 MMIO path must check this fence before touching the window and fail safe instead:
 * reads return 0, writes are dropped, commands return an error. Lock-free - safe from any context. */
extern atomic_t rfnm_la9310_mmio_fence;
extern atomic_t rfnm_la9310_mmio_fence_hits;

static inline bool rfnm_la9310_mmio_fenced(void) {
	if(likely(!atomic_read(&rfnm_la9310_mmio_fence))) {
		return false;
	}
	atomic_inc(&rfnm_la9310_mmio_fence_hits);
	return true;
}
#define PCIE_ABSERR             0x8d0
#define PCIE_ABSERR_SETTING     0x9401

#define GUL_MMAP_CCSR_OFFSET    0x8000000
#define GUL_MMAP_CCSR_SIZE      0x4000000

#define GUL_MMAP_DCSR_OFFSET    0xC000000
#define GUL_MMAP_DCSR_SIZE      0x100000

#define GUL_MMAP_PEBM_OFFSET    0x200000
#define GUL_MMAP_PEBM_SIZE      0x200000

#endif /* _PCI_UTILITIES_H */
