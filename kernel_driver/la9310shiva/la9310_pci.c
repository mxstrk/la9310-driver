/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2022 NXP
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
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/rfnm-shared.h>

#include "la9310_pci.h"
#include "la9310_vspa.h"
#include "la9310_base.h"
#include "la9310_wdog_ioctl.h"

static const char *driver_name = "la9310-shiva";
int scratch_buf_size;
uint64_t scratch_buf_phys_addr;

int dac_mask = 0x1;
EXPORT_SYMBOL(dac_mask);
int adc_mask = 0x4;
EXPORT_SYMBOL(adc_mask);

int adc_rate_mask = 0x4;
int dac_rate_mask = 0x1;

EXPORT_SYMBOL(adc_rate_mask);
EXPORT_SYMBOL(dac_rate_mask);

LIST_HEAD(pcidev_list);
static int la9310_dev_id_g;
static char *la9310_dev_name_prefix_g = "nlm";
static struct class *la9310_class;

/* RFNM: LA9310 MMIO fence (see la9310_pci.h). Owned here, raised across the hard-reprobe link-down
 * window, checked lock-free by every LA9310 MMIO path in la9310shiva and the la9310rfnm modules. */
atomic_t rfnm_la9310_mmio_fence = ATOMIC_INIT(0);
EXPORT_SYMBOL_GPL(rfnm_la9310_mmio_fence);
atomic_t rfnm_la9310_mmio_fence_hits = ATOMIC_INIT(0);
EXPORT_SYMBOL_GPL(rfnm_la9310_mmio_fence_hits);
module_param_named(mmio_fence, rfnm_la9310_mmio_fence.counter, int, 0444);
MODULE_PARM_DESC(mmio_fence, "LA9310 MMIO fence state (1 = PCIe link down, LA9310 MMIO blocked)");
module_param_named(mmio_fence_hits, rfnm_la9310_mmio_fence_hits.counter, int, 0444);
MODULE_PARM_DESC(mmio_fence_hits, "LA9310 MMIO accesses blocked by the link-down fence");

#define RFNM_IMX8MP_PCIE_OB1_LOWER_TARGET	0x33B00214
#define RFNM_LA9310_BAR0_FIX_BASE		0x1B400010
#define RFNM_LA9310_BAR0_FIX_LIMIT		0x1B401010
#define RFNM_LA9310_BAR0_TEMP_TARGET		0x10000000
#define RFNM_LA9310_BAR0_TARGET		0x18000000
#define RFNM_LA9310_BAR0_LIMIT		0x03FFFFFF

static void la9310_pcidev_remove(struct pci_dev *pdev);

static inline void __hexdump(unsigned long start, unsigned long end,
		unsigned long p, size_t sz, const unsigned char *c)
{
	while (start < end) {
		unsigned int pos = 0;
		char buf[64];
		int nl = 0;

		pos += sprintf(buf + pos, "%08lx: ", start);
		do {
			if ((start < p) || (start >= (p + sz)))
				pos += sprintf(buf + pos, "..");
			else
				pos += sprintf(buf + pos, "%02x", *(c++));
			if (!(++start & 15)) {
				buf[pos++] = '\n';
				nl = 1;
			} else {
				nl = 0;
			if (!(start & 1))
				buf[pos++] = ' ';
			if (!(start & 3))
				buf[pos++] = ' ';
			}
		} while (start & 15);
		if (!nl)
			buf[pos++] = '\n';
		buf[pos] = '\0';
		pr_debug("%s", buf);
	}
}

void la9310_hexdump(const void *ptr, size_t sz)
{
	unsigned long p = (unsigned long)ptr;
	unsigned long start = p & ~(unsigned long)15;
	unsigned long end = (p + sz + 15) & ~(unsigned long)15;
	const unsigned char *c = ptr;

	__hexdump(start, end, p, sz, c);
}
EXPORT_SYMBOL_GPL(la9310_hexdump);

struct la9310_global g_la9310_global[MAX_MODEM_INSTANCES];

static int
get_la9310_dev_id_pcidevname(struct device *dev)
{
	int i;

	for (i = 0; i < MAX_MODEM_INSTANCES; i++) {
		if (!strcmp(dev_name(dev), g_la9310_global[i].dev_name)) {
			pr_debug("device matched %s at Id %d\n",
				dev_name(dev), i);
			return i;
		}
	}
	if (la9310_dev_id_g >= MAX_MODEM_INSTANCES)
		return -1;
	return la9310_dev_id_g++;
}

struct la9310_dev *get_la9310_dev_byname(const char *name)
{
	struct list_head *ptr;
	struct la9310_dev *dev = NULL;

	list_for_each(ptr, &pcidev_list) {
		dev = list_entry(ptr, struct la9310_dev, list);
		if (!strcmp(dev->name, name))
			return dev;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(get_la9310_dev_byname);

void la9310_dev_reset_interrupt_capability(struct la9310_dev *la9310_dev)
{
	if (LA9310_CHK_FLG(la9310_dev->flags, LA9310_FLG_PCI_MSI_EN)) {
		pci_disable_msi(la9310_dev->pdev);
		LA9310_CLR_FLG(la9310_dev->flags, LA9310_FLG_PCI_MSI_EN);
	}
}

void enable_all_msi(struct la9310_dev *la9310_dev)
{
	u32 __iomem *pcie_vaddr, *pcie_msi_control;
	u32 val;

	pcie_vaddr = (u32 *)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR]
				.vaddr + PCIE_RHOM_DBI_BASE
					+ PCIE_MSI_BASE);

	val = ioread32(pcie_vaddr);
	dev_dbg(la9310_dev->dev, "MSI Capability: Control Reg. -> value = %x\n",
							val);
	pcie_msi_control = (u32 *)(la9310_dev->mem_regions
				   [LA9310_MEM_REGION_CCSR].vaddr +
				    PCIE_RHOM_DBI_BASE + PCIE_MSI_CONTROL);

	iowrite8(0xb6, pcie_msi_control);

	val = ioread32(pcie_vaddr);
	dev_dbg(la9310_dev->dev, "MSI Capability: Control Reg. -> value = %x\n"
		  , val);
}

/*
 * la9310_dev_set_interrupt_capability - set MSI or MSI-X if supported
 *
 * Attempt to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 */
int la9310_dev_set_interrupt_capability(struct la9310_dev *la9310_dev, int mode)
{
	
	int ret = 0, i = 0;

	pr_debug("la9310_dev_set_interrupt_capability\n");

	/* Check whether the device has MSIx cap */
	switch (mode) {
	case PCI_INT_MODE_MULTIPLE_MSI:
		pr_debug("PCI_INT_MODE_MULTIPLE_MSI\n");
		enable_all_msi(la9310_dev);
		ret = pci_alloc_irq_vectors_affinity(la9310_dev->pdev,
				   MIN_MSI_ITR_LINES,
				   LA9310_MSI_MAX_CNT,
				   PCI_IRQ_MSI, NULL);
		if (ret < LA9310_MSI_MAX_CNT) {
			dev_err(la9310_dev->dev,
				"Cannot complete request for multiple MSI");
			goto msi_error;
		} else {
			dev_dbg(la9310_dev->dev,
				 "%d MSI successfully created\n", ret);
		}
		LA9310_SET_FLG(la9310_dev->flags, LA9310_FLG_PCI_MSI_EN);
		for (i = 0; i < LA9310_MSI_MAX_CNT; i++) {
			la9310_dev->irq[i].msi_val = i;
			la9310_dev->irq[i].irq_val =
				pci_irq_vector(la9310_dev->pdev, i);
			la9310_dev->irq[i].free = LA931XA_MSI_IRQ_FREE;
		}
		la9310_dev->irq_count = LA9310_MSI_MAX_CNT;
		break;

	case PCI_INT_MODE_MSIX:
		pr_debug("PCI_INT_MODE_MSIX\n");
		/* TBD:XXX: LA9310 will support 8 MSIs, thus MSIx. this code
		 * need to be changed
		 */
		dev_err(la9310_dev->dev,
			"Unable to support MSIX for LA9310\n");
		__attribute__((__fallthrough__));
		/* Fall through */
	case PCI_INT_MODE_MSI:
		pr_debug("PCI_INT_MODE_MSI\n");
		if (!pci_enable_msi(la9310_dev->pdev)) {
			LA9310_SET_FLG(la9310_dev->flags, LA9310_FLG_PCI_MSI_EN);
			la9310_dev->irq[MSI_IRQ_MUX].irq_val =
				pci_irq_vector(la9310_dev->pdev, MSI_IRQ_MUX);
			la9310_dev->irq[MSI_IRQ_MUX].free = LA931XA_MSI_IRQ_FREE;
			la9310_dev->irq_count = 1;
		} else {
			dev_warn(la9310_dev->dev,
				 "Failed to init MSI, fall bk to legacy\n");
			goto msi_error;
		}
		__attribute__((__fallthrough__));
		/* Fall through */
	case PCI_INT_MODE_LEGACY:
		pr_debug("PCI_INT_MODE_LEGACY\n");
		la9310_dev->irq[MSI_IRQ_MUX].irq_val = la9310_dev->pdev->irq;
		la9310_dev->irq[MSI_IRQ_MUX].free = LA931XA_MSI_IRQ_FREE;
		la9310_dev->irq_count = 1;
		break;

	case PCI_INT_MODE_NONE:
		pr_debug("PCI_INT_MODE_NONE\n");
		break;
	}

	return 0;

msi_error:
	return -EINTR;
}

static void la9310_dev_free(struct la9310_dev *la9310_dev)
{
	list_del(&la9310_dev->list);

	pci_set_drvdata(la9310_dev->pdev, NULL);
	kfree(la9310_dev);
}

static int pcidev_tune_caps(struct pci_dev *pdev)
{
	struct pci_dev *parent;
	u16 pcaps, ecaps, ctl;
	int rc_sup, ep_sup;

	/* Find out supported and configured values for parent (root) */
	parent = pdev->bus->self;
	if (parent->bus->parent) {
		dev_dbg(&pdev->dev, "Parent not root\n");
		return -EINVAL;
	}

	if (!pci_is_pcie(parent) || !pci_is_pcie(pdev))
		return -EINVAL;

	pcie_capability_read_word(parent, PCI_EXP_DEVCAP, &pcaps);
	pcie_capability_read_word(pdev, PCI_EXP_DEVCAP, &ecaps);

	/* Find max payload supported by root, endpoint */
	rc_sup = pcaps & PCI_EXP_DEVCAP_PAYLOAD;
	ep_sup = ecaps & PCI_EXP_DEVCAP_PAYLOAD;
	dev_dbg(&pdev->dev, "max payload size    rc:%d ep:%d\n",
			128 * (1<<rc_sup), 128 * (1<<ep_sup));
	if (rc_sup > ep_sup)
		rc_sup = ep_sup;

	pcie_capability_clear_and_set_word(parent, PCI_EXP_DEVCTL,
					   PCI_EXP_DEVCTL_PAYLOAD, rc_sup << 5);

	pcie_capability_clear_and_set_word(pdev, PCI_EXP_DEVCTL,
					   PCI_EXP_DEVCTL_PAYLOAD, rc_sup << 5);

	pcie_capability_read_word(pdev, PCI_EXP_DEVCTL, &ctl);
	dev_dbg(&pdev->dev, "MAX payload size is %dB, MAX read size is %dB.\n",
		128 << ((ctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5),
		128 << ((ctl & PCI_EXP_DEVCTL_READRQ) >> 12));

	return 0;
}

static int la9310_fix_imx8mp_bar0_window(struct pci_dev *pdev) {
	void __iomem *ob1_lower_target;
	void __iomem *bar0_base;
	void __iomem *bar0_limit;
	u16 command;

	ob1_lower_target = ioremap(RFNM_IMX8MP_PCIE_OB1_LOWER_TARGET, sizeof(u32));
	bar0_base = ioremap(RFNM_LA9310_BAR0_FIX_BASE, sizeof(u32));
	bar0_limit = ioremap(RFNM_LA9310_BAR0_FIX_LIMIT, sizeof(u32));
	if(!ob1_lower_target || !bar0_base || !bar0_limit) {
		dev_err(&pdev->dev, "RFNM: failed to map LA9310 BAR0 window fix registers\n");
		if(ob1_lower_target) {
			iounmap(ob1_lower_target);
		}
		if(bar0_base) {
			iounmap(bar0_base);
		}
		if(bar0_limit) {
			iounmap(bar0_limit);
		}
		return -ENOMEM;
	}

	pci_read_config_word(pdev, PCI_COMMAND, &command);
	command |= PCI_COMMAND_MEMORY;
	pci_write_config_word(pdev, PCI_COMMAND, command);

	writel(RFNM_LA9310_BAR0_TEMP_TARGET, ob1_lower_target);
	readl(ob1_lower_target);
	writel(RFNM_LA9310_BAR0_LIMIT, bar0_limit);
	readl(bar0_limit);
	writel(RFNM_LA9310_BAR0_TARGET, bar0_base);
	readl(bar0_base);
	writel(RFNM_LA9310_BAR0_TARGET, ob1_lower_target);
	readl(ob1_lower_target);

	dev_dbg(&pdev->dev, "RFNM: applied LA9310 BAR0 window fix before probe\n");

	iounmap(bar0_limit);
	iounmap(bar0_base);
	iounmap(ob1_lower_target);
	return 0;
}

static struct la9310_dev *la9310_pci_priv_init(struct pci_dev *pdev)
{
	struct la9310_dev *la9310_dev = NULL;
	int i, rc = 0;

	la9310_dev = kzalloc(sizeof(struct la9310_dev), GFP_KERNEL);
	if (!la9310_dev)
		goto out;

	la9310_dev->dev = &pdev->dev;
	la9310_dev->pdev = pdev;

	i = get_la9310_dev_id_pcidevname(la9310_dev->dev);
	/* Not allowed to create it */
	if (i == -1) {
		dev_err(&pdev->dev,
			"exceeding max permitted (%d) la9310 devices!\n",
			MAX_MODEM_INSTANCES);
		kfree(la9310_dev);
		return NULL;
	}
	la9310_dev->id = i;

	sprintf(g_la9310_global[la9310_dev->id].dev_name, "%s",
		dev_name(la9310_dev->dev));

	dev_dbg(la9310_dev->dev, "Init - %s !\n", la9310_dev->name);

	sprintf(&la9310_dev->name[0], "%s%d", la9310_dev_name_prefix_g,
		la9310_dev->id);
	/* Get the BAR resources and remap them into the driver memory */
	for (i = 0; i < LA9310_MEM_REGION_BAR_END; i++) {
		/* Read the hardware address */
		la9310_dev->mem_regions[i].phys_addr = pci_resource_start(pdev,
									  i);
		la9310_dev->mem_regions[i].size = pci_resource_len(pdev, i);
		dev_dbg(la9310_dev->dev, "BAR:%d  addr:0x%llx len:0x%llx\n",
			 i, la9310_dev->mem_regions[i].phys_addr,
			 (u64)la9310_dev->mem_regions[i].size);
	}

	rc = la9310_map_mem_regions(la9310_dev);
	if (rc) {
		dev_err(la9310_dev->dev, "Failed to map mem regions, err %d\n",
			rc);
		goto out;
	}

#ifdef LA9310_FLG_PCI_8MSI_EN
	rc = la9310_dev_set_interrupt_capability(la9310_dev,
				PCI_INT_MODE_MULTIPLE_MSI);
#else
	rc = la9310_dev_set_interrupt_capability(la9310_dev,
				PCI_INT_MODE_MSI);
#endif
	if (rc < 0) {
		dev_dbg(la9310_dev->dev, "Cannot set the capability of device\n");
		goto out;
	}

	list_add_tail(&la9310_dev->list, &pcidev_list);

out:
	if (rc) {
		if (la9310_dev) {
			la9310_unmap_mem_regions(la9310_dev);
			kfree(la9310_dev);
		}
		la9310_dev = NULL;
	}

	return la9310_dev;
}

static int la9310_pcidev_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	int rc = 0;
	struct la9310_dev *la9310_dev = NULL;

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "failed to enable\n");
		goto err1;
	}

	rc = la9310_fix_imx8mp_bar0_window(pdev);
	if (rc) {
		goto err2;
	}

	rc = pci_request_regions(pdev, driver_name);
	if (rc) {
		dev_err(&pdev->dev, "failed to request pci regions\n");
		goto err2;
	}

	pci_set_master(pdev);

	rc = dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
	if (rc) {
		rc = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
		if (rc) {
			dev_err(&pdev->dev, "Could not set PCI Mask\n");
			goto err3;
		}
	}
	pcidev_tune_caps(pdev);

	/*Initialize la9310_dev from information obtained from pci_dev*/
	la9310_dev = la9310_pci_priv_init(pdev);
	if (!la9310_dev) {
		rc = -ENOMEM;
		dev_err(&pdev->dev, "la9310_pci_priv_init failed, err %d\n",
			rc);
		goto err4;
	}

	la9310_dev->class = la9310_class;

	rc = la9310_base_probe(la9310_dev);
	if (rc) {
		dev_err(la9310_dev->dev, "la9310_base_probe failed, err %d\n",
			rc);
		goto err5;
	}

	pci_set_drvdata(pdev, la9310_dev);

	return rc;

err5:
	la9310_dev_reset_interrupt_capability(la9310_dev);

err4:
	if (la9310_dev)
		la9310_dev_free(la9310_dev);

err3:
	pci_release_regions(pdev);

err2:
	pci_disable_device(pdev);

err1:
	return rc;
}

static void la9310_pcidev_remove(struct pci_dev *pdev)
{
	struct la9310_dev *la9310_dev = pci_get_drvdata(pdev);

	if (!la9310_dev)
		return;

	la9310_base_remove(la9310_dev);
	la9310_unmap_mem_regions(la9310_dev);
	la9310_dev_reset_interrupt_capability(la9310_dev);
	la9310_dev_free(la9310_dev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id la9310_pcidev_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_LA9310) },
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_LA9310_DISABLE_CIP)
	 },
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_LS1043A) },
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_LS1046A) },
	{ 0 },
};

static struct pci_driver la9310_pcidev_driver = {
	.name		= "LA9310",
	.id_table	= la9310_pcidev_ids,
	.probe		= la9310_pcidev_probe,
	.remove		= la9310_pcidev_remove
};

static DEFINE_MUTEX(la9310_hard_reprobe_lock);

static struct pci_dev *la9310_find_endpoint(void) {
	struct pci_dev *pdev;

	pdev = pci_get_device(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_LA9310_DISABLE_CIP, NULL);
	if (pdev) {
		return pdev;
	}

	return pci_get_device(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_LA9310, NULL);
}

static struct pci_dev *la9310_find_root_port(void) {
	struct pci_dev *endpoint;
	struct pci_dev *root_port;

	endpoint = la9310_find_endpoint();
	if (endpoint) {
		root_port = pci_dev_get(endpoint->bus->self);
		pci_dev_put(endpoint);
		if (root_port) {
			return root_port;
		}
	}

	return pci_get_domain_bus_and_slot(0, 0, PCI_DEVFN(0, 0));
}

static void la9310_rescan_all_buses(void) {
	struct pci_bus *bus = NULL;

	pci_lock_rescan_remove();
	while ((bus = pci_find_next_bus(bus)) != NULL) {
		pci_rescan_bus(bus);
	}
	pci_unlock_rescan_remove();
}

static int la9310_verify_reprobe(void) {
	struct pci_dev *endpoint;
	int ret;

	endpoint = la9310_find_endpoint();
	if (!endpoint) {
		return -ENODEV;
	}

	if (endpoint->driver != &la9310_pcidev_driver) {
		dev_err(&endpoint->dev, "RFNM: LA9310 endpoint is present but not bound to %s\n", la9310_pcidev_driver.name);
		ret = -ENODEV;
	} else {
		ret = 0;
	}

	pci_dev_put(endpoint);
	return ret;
}

static int la9310_rescan_and_verify(void) {
	int i, ret = -ENODEV;

	for (i = 0; i < 50; i++) {
		la9310_rescan_all_buses();
		ret = la9310_verify_reprobe();
		if (!ret) {
			return 0;
		}
		msleep(10);
	}

	return ret;
}

/* RFNM: quiesce the LA9310 MSI vectors before the endpoint teardown begins - disable_irq() waits for
 * running handlers, so no LA9310 IRQ handler can still be mid-MMIO once the fence-protected teardown
 * proceeds. The vectors are freed by the endpoint .remove and re-requested fresh by the re-probe
 * (irq_startup resets the disable depth), so no matching re-enable is needed here. */
static void la9310_quiesce_endpoint_irqs(void) {
	struct pci_dev *endpoint;
	struct la9310_dev *la9310_dev;
	int i;

	endpoint = la9310_find_endpoint();
	if (!endpoint) {
		return;
	}

	la9310_dev = pci_get_drvdata(endpoint);
	if (la9310_dev) {
		for (i = 0; i < la9310_dev->irq_count; i++) {
			disable_irq(la9310_dev->irq[i].irq_val);
		}
	}

	pci_dev_put(endpoint);
}

int rfnm_la9310_hard_reprobe(uint64_t dcs_freq) {
	struct pci_dev *root_port;
	int ret, resume_ret;
	bool rc_disabled = false;

	mutex_lock(&la9310_hard_reprobe_lock);

	root_port = la9310_find_root_port();
	if (!root_port) {
		ret = -ENODEV;
		goto out;
	}

	/* RFNM: raise the MMIO fence for the whole teardown + board reset + RC resume span. Concurrent
	 * contexts (FE GPIO latches via rfnm_gpio bank 6, rf_send_swcmd, the rfnm_qec kthread, IRQ
	 * handlers, sysfs/ioctl readers) must not touch the LA9310 window anywhere in this span - a CPU
	 * access to the window while the link is down never returns and hard-hangs the SoC. */
	atomic_set(&rfnm_la9310_mmio_fence, 1);
	smp_mb__after_atomic();
	la9310_quiesce_endpoint_irqs();

	dev_dbg(&root_port->dev, "RFNM: removing LA9310 PCIe root port for hard reprobe\n");
	pci_stop_and_remove_bus_device_locked(root_port);
	pci_dev_put(root_port);
	pr_debug("RFNM: LA9310 PCIe root port removed\n");

	pr_debug("RFNM: suspending PCIe RC before LA9310 reset\n");
	ret = rfnm_pcie_rc_set_disabled(true);
	if (ret) {
		pr_err("RFNM: failed to suspend PCIe RC before LA9310 reset: %d\n", ret);
		/* RC never went down - lift the fence for the recovery rescan */
		atomic_set(&rfnm_la9310_mmio_fence, 0);
		goto recover_rescan;
	}
	rc_disabled = true;

	pr_debug("RFNM: toggling LA9310 board reset controls\n");
	ret = rfnm_board_reset_la9310(dcs_freq);
	if (ret) {
		pr_err("RFNM: failed to reset LA9310 board controls: %d\n", ret);
		goto recover_resume;
	}

	pr_debug("RFNM: resuming PCIe RC after LA9310 reset\n");
	ret = rfnm_pcie_rc_set_disabled(false);
	if (ret) {
		pr_err("RFNM: failed to resume PCIe RC after LA9310 reset: %d\n", ret);
		goto recover_rescan;
	}
	rc_disabled = false;

	/* RFNM: link is back up - drop the fence before the rescan so the re-bound driver's probe MMIO is not starved */
	atomic_set(&rfnm_la9310_mmio_fence, 0);

	pr_debug("RFNM: rescanning PCIe and waiting for LA9310 driver bind\n");
	ret = la9310_rescan_and_verify();
	if (ret) {
		pr_err("RFNM: LA9310 reprobe failed: %d\n", ret);
	} else {
		pr_debug("RFNM: LA9310 hard reprobe complete\n");
	}

	goto out;

recover_resume:
	resume_ret = rfnm_pcie_rc_set_disabled(false);
	if (resume_ret) {
		pr_err("RFNM: failed to resume PCIe RC during recovery: %d\n", resume_ret);
	} else {
		atomic_set(&rfnm_la9310_mmio_fence, 0);
	}
	rc_disabled = false;

recover_rescan:
	if (rc_disabled) {
		resume_ret = rfnm_pcie_rc_set_disabled(false);
		if (resume_ret) {
			pr_err("RFNM: failed to resume PCIe RC during rescan recovery: %d\n", resume_ret);
		} else {
			atomic_set(&rfnm_la9310_mmio_fence, 0);
		}
	}
	if (atomic_read(&rfnm_la9310_mmio_fence)) {
		/* RC resume failed with the link down: leave the fence up permanently - the LA9310 is dead
		 * until reboot, but a fenced access returns poison instead of hard-hanging the SoC. */
		pr_err("RFNM: PCIe RC did not resume; LA9310 MMIO fence stays raised until reboot\n");
	}
	la9310_rescan_all_buses();

out:
	mutex_unlock(&la9310_hard_reprobe_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rfnm_la9310_hard_reprobe);

static int __init la9310_pcidev_init(void)
{
	int err = 0;

	pr_debug("NXP PCIe LA9310 Driver.\n");

	if (!(scratch_buf_size && scratch_buf_phys_addr)) {
		pr_err("ERR %s: Scratch buf values are not correct\n",
		       __func__);
		err = -EINVAL;
		goto out;
	}

	if ((scratch_buf_size <= LA9310_VSPA_FW_SIZE) ||
	    (scratch_buf_size > LA9310_MAX_SCRATCH_BUF_SIZE)) {
		pr_err("ERR %s: Scratch_buf_size is not correct\n", __func__);
		pr_err("size=0x%08x\n",scratch_buf_size);
		err = -EINVAL;
		goto out;
	}

	la9310_class = class_create(driver_name);
	//la9310_class = class_create(THIS_MODULE, driver_name);
	if (IS_ERR(la9310_class)) {
		pr_err("%s:%d Error in creating (%s) class\n",
			__func__, __LINE__, driver_name);
		return PTR_ERR(la9310_class);
	}

	err = la9310_subdrv_mod_init();
	if (err)
		goto out;

	err = pci_register_driver(&la9310_pcidev_driver);
	if (err) {
		pr_err("%s:%d pci_register_driver() failed!\n",
			__func__, __LINE__);
	}

out:
	return err;
}

static void __exit la9310_pcidev_exit(void)
{

	pci_unregister_driver(&la9310_pcidev_driver);
	la9310_subdrv_mod_exit();
	class_destroy(la9310_class);
	pr_err("Exit from NXP PCIe LA9310 driver\n");
}

module_init(la9310_pcidev_init);
module_exit(la9310_pcidev_exit);
module_param(scratch_buf_size, int, 0);
module_param(scratch_buf_phys_addr, ullong, 0);

module_param(adc_mask, int, 0400);
MODULE_PARM_DESC(adc_mask, "ADC channel enable mask - bit wise (MAX 0x4)");
module_param(adc_rate_mask, int, 0400);
MODULE_PARM_DESC(adc_rate_mask,
	"ADC Frequency mask for each channel (0 for Full Duplex, 1 for Half Duplex)");
module_param(dac_mask, int, 0400);
MODULE_PARM_DESC(dac_mask, "DAC channel enable mask - bit wise (MAX 0x2)");
module_param(dac_rate_mask, int, 0400);
MODULE_PARM_DESC(dac_rate_mask,
	"DAC Frequency for each channel (0 for Full Duplex, 1 for Half Duplex");


MODULE_PARM_DESC(max_raw_minors, "Maximum number of raw devices (1-65536)");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("PCIe LA9310 Driver");
MODULE_VERSION(LA9310_HOST_SW_VERSION);
