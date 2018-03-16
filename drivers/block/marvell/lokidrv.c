/*
 * Copyright (C) 2011 Marvell Semiconductor, Inc
 *
 * Licensed under the MIT License
 * Alternatively, you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * See COPYING-GPL.txt for the GPL license text.
 *
 * MIT License:
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/uio.h>
#include <asm/atomic.h>
#include <asm/bug.h>
#include <linux/mm.h>
#include <linux/bio.h>
#include <linux/hdreg.h>
#include <linux/blkdev.h>
#include <linux/genhd.h>
#include <asm/cacheflush.h>

#include "loki.h"
#include "event.h"
#include "dlist.h"
#include "registers.h"
#include "mv_common.h"
#include "mv_types.h"
#include "mv_util.h"
#include "lokidrv.h"
#include "wamklib.h"

#define	str(s)	#s
#define	xstr(s)	str(s)

// VM_RESERVED was removed in 3.8
#ifndef VM_RESERVED
#	define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
#endif

MODULE_LICENSE("Dual MIT/GPL");
MODULE_VERSION(xstr(WAM_DRV_VERSION_MAJOR) "." xstr(WAM_DRV_VERSION_MINOR)\
		"." xstr(WAM_DRV_VERSION_BUGFIX));

/*
 * Prototypes
 */
static int
mvloki_probe_one(struct pci_dev *pdev, const struct pci_device_id *id);
static void mvloki_detach_one(struct pci_dev *pdev);
static void setup_sram_access(nvram_hba_t *nvhba);
static void setup_nvsram_access(nvram_hba_t *nvhba);
static int nv_map_bars(nvram_hba_t *nvhba);
static int nvram_setup_dma(nvram_hba_t *nvhba, dram_access_mode_t mode);
static void nv_unmap_bars(nvram_hba_t *nvhba);
static void nvram_exit_dma(nvram_hba_t *nvhba);
static int mvloki_request_irq(nvram_hba_t *nvhba);
static void nv_mask_intr(nvram_hba_t *nvhba, u32_t intr_mask);
static void nv_unmask_intr(nvram_hba_t *nvhba, u32_t intr_mask);
static void nv_enable_irq(nvram_hba_t *nvhba);
static int nvram_ping_fw(nvram_hba_t *nvhba);
static int dma_alloc_pages(nvram_dma_t *dma_ctrl);
static void
dump_ecc_info(u32_t dev_id, nvram_dma_t *dma_ctrl, u32_t slot_id);
static int
nvram_enable_forced_ecc(nvram_hba_t *nvhba, u8_t ecc_byte);
static int
nvram_disable_forced_ecc(nvram_hba_t *nvhba);
static int
nvram_set_time(nvram_hba_t *nvhba);

static void dump_ecc_rec(u32_t id, ecc_err_info_t *ecc_rec);

static int cdev_create(nvram_hba_t *nvhba, int major, int minor);
static void cdev_destroy(nvram_hba_t *nvhba);
static int cdev_open(struct inode *, struct file *);
static int cdev_close(struct inode *, struct file *);
static long cdev_ioctl(struct file *nfile, unsigned int cmd, unsigned long arg);
static ssize_t cdev_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos);
static ssize_t cdev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos);
static int cdev_mmap(struct file *filp, struct vm_area_struct *vma);
static int cdev_aio_handler(struct file *filp, unsigned long arg);
static int aio_get_cmpl_id(nvram_hba_t *nvhba, unsigned long arg);
static int aio_wait_id(nvram_hba_t *nvhba, unsigned long arg);
static int cmd_init_ddr_prd(nvram_hba_t *nvhba, dma_request_t *req,
		u64_t offset, int rw);

static int wam_io_test(struct file *filp, unsigned long arg);
static u32_t wam_get_cmpl_cause(nvram_hba_t *nvhba);
static u32_t wam_get_intr_cause(nvram_hba_t *nvhba);

static int nv_kmem_cache_init(void);
static void nv_kmem_cache_destroy(void);
static void
dma_process_pending_requests(nvram_hba_t *nvhba);

static void nvram_irq_tasklet(unsigned long arg);
static void
wam_iostats_record(wam_iostats_t *stats, int rw, u32_t reqsize);

static void
wam_stats_init(wam_stats_t *stats);

static void
nvram_init_timer(nvram_hba_t *nvhba);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 11)
static int cdev_old_ioctl(struct inode *ninode, struct file *nfile,
		unsigned int cmd, unsigned long arg);
#endif

/*
 * PCI ID table for LokiPlus
 */
static struct pci_device_id mvloki_pci_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_MARVELL, PCI_DEVICE_ID_MARVELL_MV8180) },
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, mvloki_pci_table);

/*
 * PCI registration structure
 */
static struct pci_driver mvloki_pci_driver = {
	.name		= MV_DRIVER_NAME,
	.id_table	= mvloki_pci_table,
	.probe		= mvloki_probe_one,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	.remove		= mvloki_detach_one
#else
	.remove		= __devexit_p(mvloki_detach_one)
#endif
};


/*
 * Character device file operation handlers
 */
struct file_operations nv_cdev_fops = {
	.owner		= THIS_MODULE,

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11)
	.unlocked_ioctl	= cdev_ioctl,
#else
	.ioctl		= cdev_old_ioctl,
#endif
	.open		= cdev_open,
	.release	= cdev_close,
	.write		= cdev_write,
	.read		= cdev_read,
	.mmap		= cdev_mmap
};

/*
 * Block dev operation table
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
static int nv_bdev_ioctl(struct block_device *bdev, fmode_t mode,
		unsigned int cmd, unsigned long arg);
#else
static int nv_bdev_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
static blk_qc_t __nvram_make_request(struct request_queue *q, struct bio *bio);
#endif
static int nvram_make_request(struct request_queue *q, struct bio *bio);

static int nv_bdev_create(nvram_hba_t *nvhba);
static int nv_bdev_destroy(nvram_hba_t *nvhba);

static int nv_bdev_getgeo(struct block_device *bdev, struct hd_geometry *geo);

static struct block_device_operations nvram_block_ops = {
	.owner = THIS_MODULE,
	.ioctl = nv_bdev_ioctl,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 16)
	.getgeo = nv_bdev_getgeo,
#endif
};
#define	NV_BDEV_NR_MINORS	16
#define	NV_BDEV_SECTOR_SZ	512


/* global variables */
static nvram_hba_t	*nvram_hba[MV_MAX_HBA];
static u32_t		nvram_hba_cnt;
static int		nvram_major;
static int		nvram_major_b;
static struct class     *nvram_class;

mv_waitq_t		g_wam_alert_waitq;
spinlock_t		g_wam_alert_lock;
u32_t			g_wam_alert_dev_map;


#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)
typedef struct kmem_cache	mv_kmem_cache_t;
#else
typedef kmem_cache_t		mv_kmem_cache_t;
#endif

mv_kmem_cache_t			*nv_chained_req_cache;
mv_kmem_cache_t			*nv_cdev_req_cache;
mv_kmem_cache_t			*nv_dma_req_cache;

/* PCI-E Window control register offsets */
u32_t pcie_win_base [] = {
	PCIE_WIN0_CTRL,
	PCIE_WIN1_CTRL,
	PCIE_WIN2_CTRL,
};


/* DMA Engine Delivery Queue Write Pointers */
u32_t dma_dlvryq_wrptr[] = {
	DMA_A_DLVRY_Q_WR_PTR,
	DMA_B_DLVRY_Q_WR_PTR,
	DMA_C_DLVRY_Q_WR_PTR,
};

enum {
	REQ_REG0,
	REQ_REG1,
	REQ_REG2,
	REQ_REG3,
};

enum {
	RESP_REG0,
	RESP_REG1,
	RESP_REG2,
	RESP_REG3,
};

u32_t wam_msg_req_regs [] = {
	PCIE_A_2_CPU_MSG_0,
	PCIE_A_2_CPU_MSG_1,
	CPU_2_PCIE_A_MSG_0,
	CPU_2_PCIE_A_MSG_1,
};

u32_t wam_msg_resp_regs [] = {
	CPU_2_PCIE_A_MSG_0,
	CPU_2_PCIE_A_MSG_1,
	PCIE_A_2_CPU_MSG_0,
	PCIE_A_2_CPU_MSG_1,
};


static char *dma_state_strings[] = {
	"Not Initialized",
	"Initialized",
	"Needs Reset"
};

static char *memory_ifc_strings[] = {
	"Host Memory",
	"CS0",
	"CS1",
	"CS2",
	"CS3",
	"SRAM",
	"Unknown"
};

static char g_wam_drv_version[64];

int dma_cmd_queue_size = 32;
int ddr_cs_count = 4;
long ddr_size_shft_per_cs = 31;

/*
 * Module configurable parameters
 */
int intr_coalesce_count = 1;
module_param(intr_coalesce_count, int, 0644);
MODULE_PARM_DESC(intr_coalesce_count, "Nr of interrupts to coalesce");

int io_rescue_timer = 400;
module_param(io_rescue_timer, int, 0644);
MODULE_PARM_DESC(io_rescue_timer, "IO Hang rescue timer");

/*
 * Wrapper for snprintf
 *
 * When the given buffer size is not large enough, snprintf returns the size
 * of the buffer required for successful print
 *
 * This wrapper returns 0 if snprintf returns size larger than given size. This
 * will void corruption.
 */
static int
wm_snprintf(char *str, u32_t len, const char *fmt, ...)
{
	u32_t count;
	va_list ap;

	va_start(ap, fmt);
	count = vsnprintf(str, len, fmt, ap);
	va_end(ap);

	if (count >= len)
		return 0;

	return count;
}



/*
 * Driver entry point called by insmod
 */
int
nvram_init(void)
{
	int	ret = -1;
	dev_t	dev;

	wm_snprintf(g_wam_drv_version, 64, "%u.%u.%u-%u",
			WAM_DRV_VERSION_MAJOR,
			WAM_DRV_VERSION_MINOR,
			WAM_DRV_VERSION_BUGFIX,
			SF_REVISION);

	if (ddr_cs_count == 0) {
		mv_printk("DDR Info incorrect. Cannot continue. "
				"Nr. of CS: %d, Size per CS: %lx\n",
				ddr_cs_count, (1L << ddr_size_shft_per_cs));
		return -1;
	}

	nvram_class = class_create(THIS_MODULE, "mvloki");
	if (IS_ERR(nvram_class)) {
		ret = PTR_ERR(nvram_class);
		mv_printk("cannot create device class, err=%d\n", ret);
		return -1;
	}

	mv_waitq_init(&g_wam_alert_waitq);
	spin_lock_init(&g_wam_alert_lock);

	ret = nv_kmem_cache_init();
	if (ret != 0) {
		class_destroy(nvram_class);
		return -1;
	}

	ret = alloc_chrdev_region(&dev, 0, MV_MAX_HBA, MV_DRIVER_NAME);
	if (ret < 0) {
		mv_printk("cannot obtain major number. err: %d\n", ret);
		nv_kmem_cache_destroy();
		class_destroy(nvram_class);
		return -1;
	}

	nvram_major = MAJOR(dev);
	nvram_major_b = register_blkdev(0, MV_BLK_DRIVER_NAME);
	if (nvram_major_b <= 0) {
		mv_printk("cannot obtain blk major number. err: %d\n", ret);
		unregister_chrdev_region(dev, MV_MAX_HBA);
		nv_kmem_cache_destroy();
		class_destroy(nvram_class);
		return -1;
	}


	ret = pci_register_driver(&mvloki_pci_driver);
	if (ret < 0) {
		mv_printk("pci driver registration failed. err: %d\n", ret);
		unregister_blkdev(nvram_major_b, MV_BLK_DRIVER_NAME);
		unregister_chrdev_region(dev, MV_MAX_HBA);
		nv_kmem_cache_destroy();
		class_destroy(nvram_class);
		return ret;
	}

	mv_printk("Marvell WAM driver %s loaded. Initialized %u device(s)\n",
			g_wam_drv_version, nvram_hba_cnt);
	return 0;
}


/*
 * Driver cleanup routine called by rmmod
 */
void
nvram_exit(void)
{
	dev_t dev = MKDEV(nvram_major, 0);

	pci_unregister_driver(&mvloki_pci_driver);
	unregister_blkdev(nvram_major_b, MV_BLK_DRIVER_NAME);
	unregister_chrdev_region(dev, MV_MAX_HBA);

	nv_kmem_cache_destroy();
	class_destroy(nvram_class);
	mv_printk("Marvell WAM driver %s unloaded\n",
			g_wam_drv_version);
}

static inline int
wam_pci_restore_state(nvram_hba_t *nvhba)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
	pci_restore_state(nvhba->nv_pcidev);

	return (0);
#else
	return pci_restore_state(nvhba->nv_pcidev, nvhba->nv_pci_state);
#endif
}

static inline int
wam_pci_save_state(nvram_hba_t *nvhba)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
	int ret;

	ret = pci_save_state(nvhba->nv_pcidev);
	memcpy(nvhba->nv_pci_state, nvhba->nv_pcidev->saved_config_space,
		256);
	return ret;
#else
	return pci_save_state(nvhba->nv_pcidev, nvhba->nv_pci_state);
#endif
}




/*
 * PCI entry point
 *
 * @pdev:		PCI device structure
 * @id:			PCI ids of supported adapter
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int
#else
static int __devinit
#endif
mvloki_probe_one(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int		ret;
	nvram_hba_t	*nvhba;

	/*
	 * Announce PCI information
	 */
	mv_printk(" %#4.04x:%#4.04x:%#4.04x:%#4.04x: ",
			pdev->vendor, pdev->device, pdev->subsystem_vendor,
			pdev->subsystem_device);

	mv_printk("bus %d:slot %d:func %d\n",
			pdev->bus->number, PCI_SLOT(pdev->devfn),
			PCI_FUNC(pdev->devfn));

	nvhba = (nvram_hba_t *) kmalloc(sizeof(nvram_hba_t), GFP_KERNEL);
	if (nvhba == NULL) {
		mv_printk("nvhba struct allocation failed\n");
		return -ENOMEM;
	}

	memset(nvhba, 0, sizeof(*nvhba));

	init_waitqueue_head(&nvhba->nv_dma_ctrl.nv_broadcast_event);

	/* enabling the pci device */
	ret = pci_enable_device(pdev);
	if (ret) {
		goto init_err_1;
	}

	nvhba->nv_pcidev = pdev;

	/* map the bars into kernel address space */
	ret = nv_map_bars(nvhba);
	if (ret != 0) {
		goto init_err_2;
	}

	wam_stats_init(&nvhba->nv_stats);

	nvhba->nv_ddr_cs_cnt	= ddr_cs_count;
	nvhba->nv_ddr_size_shft	= ddr_size_shft_per_cs;
	nvhba->nv_ddr_size	= ddr_cs_count;
	nvhba->nv_ddr_size	<<= ddr_size_shft_per_cs;
	nvhba->nv_minor		= nvram_hba_cnt;
	nvhba->nv_max_ddr_offset = nvhba->nv_ddr_size - MV_DDR_OFFSET;
	nvhba->nv_pkg_ver_comp	= NV_PKG_VER_COMPATIBLE;

	sema_init(&nvhba->nv_msglock, 1);
	sema_init(&nvhba->nv_oplock, 1);

	wm_snprintf(nvhba->nv_name, MV_WAM_NAME_SIZE, "%s%u",
			MV_DRIVER_NAME, nvhba->nv_minor);

	mv_printk("DDR Size: %llu MB, Nr of CS: %d, Size per CS: %lu MB\n",
			nvhba->nv_ddr_size >> 20, ddr_cs_count,
			(1UL << ddr_size_shft_per_cs) >> 20);

	/* save the pci config space */
	wam_pci_save_state(nvhba);

	/* program the pci-e window to access scratchpad */
	setup_sram_access(nvhba);

	/* program the pci-e window to access nvsram */
	setup_nvsram_access(nvhba);

	/* allocate required pages */
	ret = dma_alloc_pages(&nvhba->nv_dma_ctrl);
	if (ret != 0) {
		mv_printk("Failed to allocate DMA pages\n");
		goto init_err_2;
	}
	nvhba->nv_shdw_cause = &nvhba->nv_dma_ctrl.nv_cmplq->cmpl_shdw_cause;
	memset(nvhba->nv_shdw_cause, 0, sizeof(wam_shdw_cause_t));

	nvhba->nv_get_cause = wam_get_intr_cause;

	nvram_set_time(nvhba);

	/*
	 * Initialize DMA by handshaking with firmware
	 */
	ret = nvram_setup_dma(nvhba, DRAM_WRITE_MODE);
	if (ret != 0) {
		mv_printk("Failed to setup DMA with fw\n");
	}

	tasklet_init(&nvhba->nv_irq_tasklet, nvram_irq_tasklet,
			(unsigned long) nvhba);
	/*
	 * Register IRQ
	 */
	ret = mvloki_request_irq(nvhba);
	if (ret != 0) {
		mv_printk("Failed to register IRQ\n");
		goto init_err_5;
	}

	nvram_init_timer(nvhba);

	/*
	 * Create char device
	 */
	ret = cdev_create(nvhba, nvram_major, nvhba->nv_minor);
	if (ret != 0) {
		mv_printk("Failed to create char device\n");
		goto init_err_6;
	}

	/*
	 * Create block device
	 */
	ret = nv_bdev_create(nvhba);
	if (ret != 0) {
		mv_printk("Failed to create block device\n");
		goto init_err_6;
	}

	/*
	 * Store nvhba in PCI softstate
	 */
	pci_set_drvdata(pdev, nvhba);

	nvram_hba[nvhba->nv_minor] = nvhba;
	nvram_hba_cnt++;

	mv_printk("Initialized WAM device %u\n", nvram_hba_cnt - 1);

	return 0;

	/* error path */
init_err_6:
	nvhba->nv_exit_flag = 1;
	del_timer_sync(&nvhba->nv_timer);
	free_irq(nvhba->nv_pcidev->irq, nvhba);

init_err_5:
	nvram_exit_dma(nvhba);
	nv_unmap_bars(nvhba);

init_err_2:
//	pci_disable_device(pdev);

init_err_1:
	kfree(nvhba);
	mv_printk("Failed initializing nvram device\n");
	return -ENODEV;
}


/*
 * PCI "un"plug entry point
 *
 * @pdev:		PCI device structure
 */
static void mvloki_detach_one(struct pci_dev *pdev)
{
	nvram_hba_t	*nvhba;

	nvhba = pci_get_drvdata(pdev);

	/* disable interrupt */
	nv_mask_intr(nvhba, 0xffffffff);

	if (nvhba->nv_irq_enabled)
		free_irq(nvhba->nv_pcidev->irq, nvhba);

#if defined(LOKI_USE_MSI) && defined(CONFIG_PCI_MSI)
	if (nvhba->nv_msi_enabled) {
		mv_printk("Disabling MSI mode, irq: %u\n",
				nvhba->nv_pcidev->irq);
		pci_disable_msi(nvhba->nv_pcidev);
	}
#endif
	cdev_destroy(nvhba);

	nv_bdev_destroy(nvhba);

	nvhba->nv_exit_flag = 1;
	del_timer_sync(&nvhba->nv_timer);

	tasklet_disable(&nvhba->nv_irq_tasklet);

	nvram_exit_dma(nvhba);

	nv_unmap_bars(nvhba);

	pci_set_drvdata(pdev, NULL);

	/* disabled for RHEL EL 4 */
//	pci_disable_device(pdev);

	memset(nvhba, 0, sizeof(nvram_hba_t));

	kfree(nvhba);

	return;
}


/*
 * Get the physical addresses of enabled BARs
 * @nvhba - hba handle
 */
static void
nv_get_bar_info(nvram_hba_t *nvhba)
{
	int	i;
	int	bar;
	unsigned long	start;
	unsigned int	len;

	for (i = 0; i < MV_MAX_BARS; i++) {
		bar = (i << 1);
		start = pci_resource_start(nvhba->nv_pcidev, bar);
		len = pci_resource_len(nvhba->nv_pcidev, bar);

		nvhba->nv_bar[i].bar_base = start;
		nvhba->nv_bar[i].bar_length = len;
		mv_printk("BAR %u: Base: %08llx, Length: 0x%x\n",
				i,
				nvhba->nv_bar[i].bar_base,
				nvhba->nv_bar[i].bar_length);
	}
}


/*
 * Check if the BARs are big enough to map necessary memory regions.
 *
 * @nvhba - hba handle
 * @return 0 if successful, -1 if any BAR is smaller
 */
static int
nv_check_bar_size(nvram_hba_t *nvhba)
{
	int	imem_bar = NV_CTRL_ACCESS_BAR;
	u32_t	reqd_size;

	/* check the size of bar (0) to access the register space */
	if (nvhba->nv_bar[0].bar_length < NV_REG_SPACE_SIZE) {
		mv_printk("BAR 0 too small - %x, Expected: %x\n",
				nvhba->nv_bar[0].bar_length, NV_REG_SPACE_SIZE);
		return -1;
	}

	/* check the size of bar to access internal memory like scratchpad,
	 * flash and event log space
	 */
	imem_bar = NV_CTRL_ACCESS_BAR;
	reqd_size = NV_SRAM_MEM_SIZE + NV_FLASH_ACCESS_SIZE +
			NV_NVSRAM_ACCESS_SIZE;

	if (nvhba->nv_bar[imem_bar].bar_length < reqd_size) {
		mv_printk("BAR %u too small (%x), Expected: %u\n",
				imem_bar, nvhba->nv_bar[imem_bar].bar_length,
				reqd_size);
		return -1;
	}

	return 0;
}

/*
 * Assign kernel virtual addresses for BARs
 *
 * @nvhba - hba handle
 * @return 0 if successful, -1 if any BAR is smaller or virtual address
 * mapping fails.
 */
static int
nv_map_bars(nvram_hba_t *nvhba)
{
	int	imem_bar = NV_CTRL_ACCESS_BAR;
	u32_t	reqd_size;
	void	*bar_vaddr;

	nv_get_bar_info(nvhba);

	if (nv_check_bar_size(nvhba) != 0)
		return -1;

	/* Map register space to KVA */
	nvhba->nv_regs = ioremap_nocache(nvhba->nv_bar[0].bar_base,
						NV_REG_SPACE_SIZE);
	if (nvhba->nv_regs == NULL) {
		mv_printk("could not map internal register space\n");
		return -1;
	}

	reqd_size = NV_SRAM_MEM_SIZE + NV_FLASH_ACCESS_SIZE +
			NV_NVSRAM_ACCESS_SIZE;

	/* Allocate kernel virtual addr to access internal memory regions */
	bar_vaddr = ioremap_nocache(nvhba->nv_bar[imem_bar].bar_base,
					reqd_size);
	if (bar_vaddr == NULL) {
		mv_printk("could not map bar %u to access internal memory\n",
				imem_bar);
		return -1;
	}

	nvhba->nv_sram_base = bar_vaddr;
	nvhba->nv_flash_base = bar_vaddr + NV_FLASH_OFFSET;
	nvhba->nv_log_base = bar_vaddr + NV_NVSRAM_OFFSET;

	return 0;
}

/*
 * Unmap the kernel virtual addresses assigned to each BAR
 * @nvhba - hba handle
 */
static void
nv_unmap_bars(nvram_hba_t *nvhba)
{
	if (nvhba->nv_sram_base != NULL) {
		pci_iounmap(nvhba->nv_pcidev, nvhba->nv_sram_base);
		nvhba->nv_sram_base = NULL;
	}

	if (nvhba->nv_regs != NULL) {
		pci_iounmap(nvhba->nv_pcidev, nvhba->nv_regs);
		nvhba->nv_regs = NULL;
	}
}


/*
 * Function to program a pci-e window with the given parameters
 *
 * @pcie_reg_base	- register base address of the pci-e port
 * @window		- the window to be programmed
 * @bar			- map to BAR 1 or BAR 2
 * @base_addr		- address within the BAR for this window
 * @size		- size of the window (power of 2, lowest size - 64KB)
 * @remap_offset	- address within the destination to be mapped
 * @mbus_id		- destination crossbar port id
 * @mbus_attr		- destination crossbar attribute
 */
static void
pcie_set_window(void *pcie_reg_base, int window, int bar,
		u32_t base_addr, u32_t size,
		u32_t remap_offset, u32_t mbus_id,
		u32_t mbus_attr)
{
	u32_t value;
	void *win_reg = pcie_reg_base + pcie_win_base[window];

	/* divide by 64K and subract 1 to make it a series of 1's */
	size >>= 16;
	size--;

	/* BAR 1 - 0, BAR 2 - 1 */
	bar >>= 1;

	value = ((size << PCIE_WCTRL_SIZE_SHFT) |
				(mbus_attr << PCIE_WCTRL_ATTR_SHFT) |
				(mbus_id << PCIE_WCTRL_TGT_SHFT) |
				(bar << PCIE_WCTRL_BAR_SHFT) |
				PCIE_WCTRL_WEN);

	/* set the window control */
	iowrite32(value, win_reg + PCIE_WIN_CTRL_OFF);

	/* set the base address for the window */
	value = (base_addr & 0xFFFF0000);
	iowrite32(value, win_reg + PCIE_WIN_BASE_OFF);

	/* Remap the address to given offset */
	value = (remap_offset & 0xFFFF0000) | 1;
	iowrite32(value, win_reg + PCIE_WIN_REMAP_OFF);
}

/*
 * Program pci-e window to access boot flash. This pcie window
 * is used to write or upgrade firmware on the boot flash.
 * This window needs to modified repeatedly with the desired offset
 * in the flash.
 *
 * @nvhba		- hba handle
 * @flash_offset	- offset of the flash to map (64K aligned)
 * @return 0 if successful, -EINVAL if offset is not 64K aligned
 */
static int
setup_flash_access(nvram_hba_t *nvhba, u32_t flash_offset)
{
	int bar;
	unsigned long paddr;
	unsigned int size;
	void *pcie_reg_base = nvhba->nv_regs + PCIE_A_REG_BASE;

	if (flash_offset & 0xFFFF)
		return -EINVAL;

	bar = NV_CTRL_ACCESS_BAR;
	paddr = nvhba->nv_bar[bar].bar_base + NV_FLASH_OFFSET;

	size = NV_FLASH_ACCESS_SIZE;
	pcie_set_window(pcie_reg_base, NV_PCIE_FLASH_WIN, bar,
			(u32_t)paddr, size, flash_offset,
			FLASH_MBUS_ID, FLASH_MBUS_ATTR);
	return 0;
}

/*
 * Program pci-e window to access internal scratchpad memory from
 * offset 0 and size NV_SRAM_MEM_SIZE
 *
 * @nvhba		- hba handle
 */
static void
setup_sram_access(nvram_hba_t *nvhba)
{
	int	bar;
	unsigned long	paddr;
	unsigned int	size;
	unsigned int	mbus_id;
	unsigned int	mbus_attr;
	void		*pcie_reg_base = nvhba->nv_regs + PCIE_A_REG_BASE;

	bar = NV_CTRL_ACCESS_BAR;
	paddr = nvhba->nv_bar[bar].bar_base;

	mbus_id = SCRPAD_MBUS_ID;
	mbus_attr = SCRPAD_MBUS_ATTR;

	/*
	 * Configure PCIE Window Control 0 register to access
	 * 512K of scratchpad memory
	 */
	size = NV_SRAM_MEM_SIZE;
	pcie_set_window(pcie_reg_base, NV_PCIE_SRAM_WIN, bar,
			(u32_t)paddr, size, 0, mbus_id, mbus_attr);
	return;
}


/*
 * Program pci-e window to access internal nvram memory from
 * offset 0 and size NV_NVSRAM_ACCESS_SIZE
 *
 * @nvhba		- hba handle
 */
static void
setup_nvsram_access(nvram_hba_t *nvhba)
{
	int	bar;
	unsigned long	paddr;
	unsigned int	size;
	unsigned int	mbus_id;
	unsigned int	mbus_attr;
	void		*pcie_reg_base = nvhba->nv_regs + PCIE_A_REG_BASE;

	bar = NV_CTRL_ACCESS_BAR;
	paddr = nvhba->nv_bar[bar].bar_base + NV_NVSRAM_OFFSET;

	mbus_id = NVSRAM_MBUS_ID;
	mbus_attr = NVSRAM_MBUS_ATTR;

	/*
	 * Configure PCIE Window Control 2 register to access
	 * 128K of on-board nvram
	 */
	size = NV_NVSRAM_ACCESS_SIZE;
	pcie_set_window(pcie_reg_base, NV_PCIE_NVSRAM_WIN, bar,
			(u32_t)paddr, size, 0, mbus_id, mbus_attr);
	return;
}


/*
 * routine to mask all interrupts
 * @nvhba		- hba handle
 * @intr_mask		- interrupts that require to be masked (-1 for all)
 */
static void
nv_mask_intr(nvram_hba_t *nvhba, u32_t intr_mask)
{
	iowrite32(0, nvhba->nv_regs + MAIN_INTR_MASK);
}


/*
 * routine to unmask interrupt. The driver handles only the doorbell
 * interrupt
 * @nvhba		- hba handle
 * @intr_mask		- interrupt mask to enable
 */
static void
nv_unmask_intr(nvram_hba_t *nvhba, u32_t intr_mask)
{
	unsigned int val;

	val = INTR_CPU_2_PCIEA_DRBL;
	iowrite32(val, nvhba->nv_regs + MAIN_INTR_MASK);
}


/*
 * function to read and write registers by user land programs
 * (for debug)
 * @nvhba	- hba handle
 * @reg_cfg	- object containing register address and values
 */
static void
reg_io(nvram_hba_t *nvhba, reg_io_t *reg_cfg)
{
	if (reg_cfg->rw) {
		iowrite32(reg_cfg->reg_val, nvhba->nv_regs + reg_cfg->reg_addr);
	} else {
		reg_cfg->reg_val = ioread32(nvhba->nv_regs + reg_cfg->reg_addr);
	}
	return;
}

/*
 * debug register print routine
 */
static void
dump_reg(void *regs, char *str, unsigned int reg_offset)
{
	unsigned int val;

	val = ioread32(regs + reg_offset);
	mv_printk("%s = %08X\n", str, val);
}

/*
 * debug dump routine
 */
static void
dump_registers(nvram_hba_t *nvhba)
{
	dump_reg(nvhba->nv_regs, "Intr cause", MAIN_INTR_CAUSE);
	dump_reg(nvhba->nv_regs, "Intr Mask EP -A", MAIN_INTR_MASK);
	dump_reg(nvhba->nv_regs, "DMA RD Ptr", DMA_DLVRY_Q_RD_PTR);
}

/*
 * routine to clear the firmware to host driver doorbell
 *
 * @nvhba	- hba handle
 * @cause	- cause to be cleared
 */
static void
clear_drbl(nvram_hba_t *nvhba, u32_t cause)
{
	iowrite32(cause, nvhba->nv_regs + CPU_2_PCIE_A_DRBL);
}

/*
 * Function to send a message to FW
 *
 * This routine sends a message to FW using message registers and waits until
 * the response is received. This routine makes sure that only one outstanding
 * message is sent.
 *
 * It holds a semaphore and hence cannot be invoked in ISR. Tasklets are ok.
 *
 * @nvhba	- hba handle
 * @msg		- msg info to send and hold the response back.
 * @return 0 if successful,
 * -ETIME       - if fw did not respond for a while
 * -EINVAL      - fw sent invalid handshake signature (BUG)
 */
static int
mv_send_msg(nvram_hba_t *nvhba, wam_fwmsg_t *msg)
{
	int	i;
	int	ret;
	int	retry = msg->msg_waitmsecs;
	u32_t	drbl;
	u32_t	drbl_msg;
	unsigned long end_jiffies;

	if (!in_atomic())
		down(&nvhba->nv_msglock);

	/* clear the fw to driver doorbell */
	clear_drbl(nvhba, FW_DRBL_HANDSHAKE_IRQ);

	for (i = 0; i < NV_MAX_MSG_DATA; i++) {
		iowrite32(msg->msg_data[i],
				nvhba->nv_regs + wam_msg_req_regs[i + 1]);
	}

	/* set the signature and msg code */
	drbl_msg = (FW_HOST_2_CPU_DRBL_SIGNATURE << 16);
	drbl_msg |= (msg->msg_code & 0xFFFF);
	iowrite32(drbl_msg, nvhba->nv_regs + wam_msg_req_regs[REQ_REG0]);

	/* ring the doorbell */
	iowrite32(FW_DRBL_HANDSHAKE_IRQ,
			nvhba->nv_regs + PCIE_A_2_CPU_DRBL);

	end_jiffies = jiffies + msecs_to_jiffies(msg->msg_waitmsecs);

	/* poll for response */
	do {
		if (!in_atomic())
			msleep(1);
		else
			mdelay(1);

		drbl = ioread32(nvhba->nv_regs + CPU_2_PCIE_A_DRBL);
		if (drbl & FW_DRBL_HANDSHAKE_IRQ) {
			break;
		}

		if (jiffies >= end_jiffies) {
			retry = 0;
			break;
		}

		retry--;
	} while (retry > 0);

	if (retry == 0) {
		ret = -ETIME;
		mv_printk("FW Cmd: 0x%02X timedout after %u msec.\n",
				msg->msg_code, msg->msg_waitmsecs);
		iowrite32(0, nvhba->nv_regs + wam_msg_req_regs[REQ_REG0]);
		iowrite32(0, nvhba->nv_regs + PCIE_A_2_CPU_DRBL);
		goto msg_fail;
	}

	/* clear the doorbell */
	clear_drbl(nvhba, FW_DRBL_HANDSHAKE_IRQ);

	/* read the response */
	drbl_msg = ioread32(nvhba->nv_regs + wam_msg_resp_regs[RESP_REG0]);
	if ((drbl_msg >> 16) != FW_CPU_2_HOST_DRBL_SIGNATURE) {
		mv_printk("Invalid signature from FW. Msg: %08x\n", drbl_msg);
		ret = -EINVAL;
		goto msg_fail;
	}

	msg->msg_rsp = (drbl_msg & 0xFFFF);
	for (i = 0; i < NV_MAX_MSG_DATA; i++) {
		msg->msg_data[i] = ioread32(nvhba->nv_regs +
				wam_msg_resp_regs[i + 1]);
	}

	ret = 0;

msg_fail:
	if (!in_atomic())
		up(&nvhba->nv_msglock);

	return ret;
}


/*
 * Allocate the shadow PRD table and completion required for the DMA engine
 *
 * This function allocates pages of fixed size (NV_DMA_CTRL_PAGE_SIZE) for
 * alignment requirement.
 *
 * Note: kmalloc returns physically contiguous memory in linux
 *
 * @dma_ctrl	- dma control handle
 * @return 0 if successful,
 * -EINVAL - if number of slots is too large
 * -ENOMEM - if memory allocation fails
 */
static int
dma_alloc_pages(nvram_dma_t *dma_ctrl)
{
	u32_t	prd_tbl_size;
	u32_t	cmplq_size;
	u32_t	nr_slots;
	void	*buffer;

	spin_lock_init(&dma_ctrl->nv_lock);

	nr_slots = 3 * dma_cmd_queue_size;
	prd_tbl_size = nr_slots * sizeof(cmd_prd_tbl_t);
	cmplq_size = sizeof(cmplq_t) + (nr_slots * sizeof(cmplq_entry_t));

	if ((prd_tbl_size > NV_DMA_CTRL_PAGE_SIZE) ||
			(cmplq_size > NV_DMA_CTRL_PAGE_SIZE)) {
		mv_printk("command queue size %u too large\n",
				nr_slots);
		return -EINVAL;
	}

	/* allocate the shadow prd table */
	buffer = dma_alloc_coherent(NULL, NV_DMA_CTRL_PAGE_SIZE,
			&dma_ctrl->nv_shadow_prd_pa, GFP_KERNEL);
	if (buffer == NULL) {
		mv_printk("Failed to allocate memory for PRD table\n");
		return -ENOMEM;
	}
	dma_ctrl->nv_shadow_prd = (cmd_prd_tbl_t *)buffer;

	/* allocate the completion queue */
	buffer = dma_alloc_coherent(NULL, NV_DMA_CTRL_PAGE_SIZE,
			&dma_ctrl->nv_cmplq_pa, GFP_KERNEL);
	if (buffer == NULL) {
		mv_printk("Failed to allocate completion queue\n");
		dma_free_coherent(NULL, NV_DMA_CTRL_PAGE_SIZE,
				dma_ctrl->nv_shadow_prd,
				dma_ctrl->nv_shadow_prd_pa);
		dma_ctrl->nv_shadow_prd = NULL;
		return -ENOMEM;
	}
	dma_ctrl->nv_cmplq = (cmplq_t *)buffer;

	return 0;
}


/*
 * Release the pages allocated for DMA engine
 * @dma_ctrl	- dma control handle
 */
static void
dma_free_pages(nvram_dma_t *dma_ctrl)
{
	if (dma_ctrl->nv_shadow_prd) {
		dma_free_coherent(NULL, NV_DMA_CTRL_PAGE_SIZE,
				dma_ctrl->nv_shadow_prd,
				dma_ctrl->nv_shadow_prd_pa);
		dma_ctrl->nv_shadow_prd = NULL;
	}

	if (dma_ctrl->nv_cmplq) {
		dma_free_coherent(NULL, NV_DMA_CTRL_PAGE_SIZE,
				dma_ctrl->nv_cmplq,
				dma_ctrl->nv_cmplq_pa);
		dma_ctrl->nv_cmplq = NULL;
	}
}


/*
 * Initialize the DMA control structure and initialize the ring pointers
 *
 * @dma_ctrl	- dma control handle
 */
static void
dma_init_state(nvram_dma_t *dma_ctrl)
{
	u32_t		nr_slots;

	/* nr of slots should be a multiple of 3 (3 data transfer engines) */
	nr_slots		= 3 * dma_cmd_queue_size;
	dma_ctrl->nv_nr_slots	= nr_slots;
	dma_ctrl->nv_queued_id	= 0;
	dma_ctrl->nv_cmpltd_id	= 0;
	dma_ctrl->nv_state	= DMA_NOT_INITIALIZED;

	if (dma_ctrl->nv_errcode == 0)
		dma_ctrl->nv_errcode = -ENODEV;

	/* initialize the ring pointers' starting position */
	dma_ctrl->nv_cmplq_wptr	= nr_slots - 1;
	dma_ctrl->nv_sw_dlvryq_wptr = nr_slots - 1;
	dma_ctrl->nv_hw_dlvryq_wptr = nr_slots - 1;

	dlist_init(&dma_ctrl->nv_request_list);
	dlist_init(&dma_ctrl->nv_posted_list);

	dma_ctrl->nv_cmplq->cmpl_wr_ptr	= nr_slots - 1;

	return;
}


static int
nvram_set_time(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;
	struct timespec	ts;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	getnstimeofday(&ts);

	msg.msg_code = DRBL_MSG_SET_TIME;
	msg.msg_data[0] = ts.tv_sec;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_TIME_SET) {
		if (msg.msg_rsp != DRBL_RSP_UNKNOWN_MSG) {
			mv_printk("Failed to set time: %04x\n",
					msg.msg_rsp);
			ret = msg.msg_rsp;
		}
	}

	return ret;
}


/*
 * Function to handshake with firmware and initialize the firmware DMA engine
 *
 * The required pages for DMA should be allocated before calling this function.
 * This routine may be called to reset the DMA engine after any ecc error.
 *
 * @nvhba	- hba handle
 * @rdonly	- DRAM_RDONLY_MODE if read-only mode is required or
 *		  DRAM_WRITE_MODE for read-write mode
 * @return 0 if successful or
 * -ENOMEM	- not enough memory
 * -ETIME	- if fw did not respond for a while
 * -EINVAL	- fw sent invalid handshake signature
 */
static int
nvram_setup_dma(nvram_hba_t *nvhba, dram_access_mode_t mode)
{
	int		ret = 0;
	u32_t		sram_offset;
	u64_t		prd_paddr;
	u64_t		cmplq_paddr;
	dma_addr_t	cfg_paddr;
	mv_dma_cfg_t	*dma_cfg;
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;
	wam_fwmsg_t	msg;

	dma_init_state(dma_ctrl);

	if (nvram_ping_fw(nvhba) != 0) {
		mv_printk("Dev %u Firmware failed to respond. Init failed\n",
				nvhba->nv_minor);
		return -ETIME;
	}

	prd_paddr = dma_ctrl->nv_shadow_prd_pa;
	cmplq_paddr = dma_ctrl->nv_cmplq_pa;

	/* allocate the handshake dma config page */
	dma_cfg = (mv_dma_cfg_t *) dma_alloc_coherent(NULL,
					NV_DMA_CTRL_PAGE_SIZE,
					&cfg_paddr, GFP_ATOMIC);
	if (dma_cfg == NULL) {
		mv_printk("Failed to allocate dma config page\n");
		return -ENOMEM;
	}

	dma_cfg->d_nr_slots	= dma_ctrl->nv_nr_slots;
	dma_cfg->d_prd_arr_low	= LSB_32(prd_paddr);
	dma_cfg->d_prd_arr_high	= MSB_32(prd_paddr);
	dma_cfg->d_cmplq_low	= LSB_32(cmplq_paddr);
	dma_cfg->d_cmplq_high	= MSB_32(cmplq_paddr);
	dma_cfg->d_src_prds	= MV_SRC_PRDS_PER_CMD;
	dma_cfg->d_dst_prds	= MV_DST_PRDS_PER_CMD;
	dma_cfg->d_intr_coal_cnt = intr_coalesce_count;

	if (mode == DRAM_RDONLY_MODE)
		dma_cfg->d_rdonly_magic = DRAM_RDONLY_MODE_MAGIC;
	else
		dma_cfg->d_rdonly_magic = 0;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	/*
	 * Initialize the msg with code and the physical address
	 * of the config structure
	 */
	msg.msg_code = DRBL_MSG_DMA_SETUP;
	msg.msg_data[0] = LSB_32(cfg_paddr);
	msg.msg_data[1] = MSB_32(cfg_paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		goto hndshk_fail;

	if (msg.msg_rsp != DRBL_RSP_DMA_SETUP_DONE) {
		mv_printk("dma init handshake failed. err_code: %04x\n",
				msg.msg_rsp);
		ret = msg.msg_rsp;
		goto hndshk_fail;
	}

	/* read the offset of command header table in scratchpad */
	sram_offset = msg.msg_data[0];
	dma_ctrl->nv_cmd_tbl = (cmd_hdr_t *)(nvhba->nv_sram_base +
			sram_offset);

	sram_offset = msg.msg_data[1];
	dma_ctrl->nv_sram_prd = (cmd_prd_tbl_t *)(nvhba->nv_sram_base +
			sram_offset);

	dma_ctrl->nv_state = DMA_INITIALIZED;
	dma_ctrl->nv_mode = mode;

	nvhba->nv_alert_count = 0;
	nvhba->nv_get_cause = wam_get_cmpl_cause;
	dma_ctrl->nv_errcode = 0;

	ret = 0;

hndshk_fail:
	dma_free_coherent(NULL, NV_DMA_CTRL_PAGE_SIZE, dma_cfg, cfg_paddr);
	return ret;
}


/*
 * Enable Write mode (typically after supercap calibration) to allow
 * firmware to backup on a power loss
 *
 * During Supercap calibration, after data is backed-up to SSD, DMA can
 * be initialized in READ ONLY mode. The read-only mode will allow data
 * to be read by the host while the supercap is being calibrated
 */
int
nvram_en_write_mode(nvram_hba_t *nvhba)
{
	int	ret;
	u16_t	msg_rsp;
	wam_fwmsg_t	msg;

	/* enable write mode (after a read-only mode) */
	memset(&msg, 0, sizeof(wam_fwmsg_t));
	msg.msg_code = DRBL_MSG_SET_OPTIONS;
	msg.msg_data[0] = EN_DRAM_WRITE_MODE;
	msg.msg_waitmsecs = 1000;

	/* set the dma-setup msg code */
	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	msg_rsp = msg.msg_rsp;
	if (msg_rsp != DRBL_RSP_SET_OPTIONS_DONE) {
		mv_printk("set options failed. err_code: %04x\n",
				msg_rsp);
		return -1;
	}

	return 0;
}


/*
 * Stop the FW DMA engine.
 * DMA should be reinitialized using nvram_setup_dma if required again.
 *
 * @nvhba - hba handle
 * @errcode - error code to return for all IOs
 * @return 0 if successful,
 * fw did not acknowledge otherwise.
 */
static int
dma_stop(nvram_hba_t *nvhba, int errcode)
{
	int		ret = 0;
	dlist_t		*link;
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;
	dma_request_t	*dma_req;
	dma_done_t	dma_done;
	wam_fwmsg_t	msg;

	dma_ctrl->nv_state = DMA_NOT_INITIALIZED;
	dma_ctrl->nv_errcode = errcode;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_DMA_STOP;
	msg.msg_waitmsecs = 1000;

	/* set the dma-setup msg code */
	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_DMA_STOP_ACK) {
		mv_printk("dma stop notification failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	nvhba->nv_get_cause = wam_get_intr_cause;

	spin_lock_bh(&dma_ctrl->nv_lock);

	/*
	 * walk through the request lists and return the IOs as failed
	 */
	link = dlist_rm_front(&dma_ctrl->nv_posted_list);
	while (link != NULL) {
		dma_req = MV_CONTAINER_OF(link, dma_request_t, dma_link);
		dma_req->dma_status = dma_ctrl->nv_errcode;

		dma_done = dma_req->dma_done;
		if (dma_done != NULL)
			dma_done(dma_req);

		link = dlist_rm_front(&dma_ctrl->nv_posted_list);
	}

	link = dlist_rm_front(&dma_ctrl->nv_request_list);
	while (link != NULL) {
		dma_req = MV_CONTAINER_OF(link, dma_request_t, dma_link);
		dma_req->dma_status = dma_ctrl->nv_errcode;

		dma_done = dma_req->dma_done;
		if (dma_done != NULL)
			dma_done(dma_req);

		link = dlist_rm_front(&dma_ctrl->nv_request_list);
	}

	spin_unlock_bh(&dma_ctrl->nv_lock);

	return 0;
}

static u32_t
dma_dequeue_requests(nvram_dma_t *dma_ctrl, dlist_t *new_list)
{
	u32_t		count = 0;
	dlist_t		*link;

	dlist_init(new_list);

	/*
	 * move commands from posted lists to new list
	 */
	link = dlist_rm_front(&dma_ctrl->nv_posted_list);
	while (link != NULL) {
		count++;
		dlist_add_back(new_list, link);
		link = dlist_rm_front(&dma_ctrl->nv_posted_list);
	}

	/*
	 * move commands from request list to new list
	 */
	link = dlist_rm_front(&dma_ctrl->nv_request_list);
	while (link != NULL) {
		count++;
		dlist_add_back(new_list, link);
		link = dlist_rm_front(&dma_ctrl->nv_request_list);
	}

	return count;
}


/*
 * Reset the fw DMA state machine
 *
 * If DMA is already initialized and an ECC error is encountered, this
 * function is used to reset the DMA with the previous configured
 * parameters.
 *
 * This function should be invoked with the lock held
 *
 * @nvhba	- hba handle
 * @return 0 if successful, error otherwise
 */
static int
dma_reset(nvram_hba_t *nvhba)
{
	int		ret;
	u32_t		req_count;
	u32_t		sram_offset;
	dlist_t		new_list;
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;
	wam_fwmsg_t	msg;


	req_count = dma_dequeue_requests(dma_ctrl, &new_list);

	dma_init_state(dma_ctrl);

	dma_ctrl->nv_queued_id = req_count;

	mv_printk("Dev: %u Resetting DMA. Rescheduling %u requests\n",
			nvhba->nv_minor, req_count);

	dlist_replace(&new_list, &dma_ctrl->nv_request_list);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_DMA_RESET;
	msg.msg_waitmsecs = 1000;

	/* send the dma-reset msg code */
	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return -1;

	if (msg.msg_rsp != DRBL_RSP_DMA_RESET_DONE) {
		mv_printk("dma reset handshake failed. err_code: %04x\n",
				msg.msg_rsp);
		ret = msg.msg_rsp;
		return -1;
	}

	/* read the offset of command header table in scratchpad */
	sram_offset = msg.msg_data[0];
	dma_ctrl->nv_cmd_tbl = (cmd_hdr_t *)(nvhba->nv_sram_base + sram_offset);

	sram_offset = msg.msg_data[1];
	dma_ctrl->nv_sram_prd = (cmd_prd_tbl_t *)(nvhba->nv_sram_base +
			sram_offset);

	dma_ctrl->nv_state = DMA_INITIALIZED;
	nvhba->nv_alert_count = 0;
	dma_ctrl->nv_errcode = 0;

	return 0;
}


/*
 * DMA cleanup routine
 *
 * @nvhba - hba handle
 */
static void
nvram_exit_dma(nvram_hba_t *nvhba)
{
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;

	if (dma_ctrl->nv_state == DMA_NOT_INITIALIZED ||
			dma_stop(nvhba, -ENODEV) == 0) {
		/* free the pages only after the acknowledgement from fw */
		dma_free_pages(&nvhba->nv_dma_ctrl);
	} else {
		mv_printk("not freeing pages to avoid possible corruption\n");
	}
}


/*
 * Flush the contents of the DDR to SSD
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - if fw did not complete the backup.
 */
static int
nvram_backup(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	ret = dma_stop(nvhba, -EBUSY);
	if (ret != 0) {
		mv_printk("Cannot stop DMA. Backup not initiated\n");
		return ret;
	}

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_DDR_BACKUP;
	msg.msg_waitmsecs = 80000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_DDR_BACKUP_DONE) {
		mv_printk("Backup failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	nvram_setup_dma(nvhba, nvhba->nv_dma_ctrl.nv_mode);

	return 0;
}


/*
 * Restore the contents of the DDR from SSD
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - fw did not complete the restore.
 */
static int
nvram_restore(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_DDR_RESTORE;
	msg.msg_waitmsecs = 46000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_DDR_RESTORE_DONE) {
		mv_printk("ssd restore failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


/*
 * Format the SSD
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - if fw timed out
 */
static int
nvram_ssd_format(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SSD_FORMAT;
	msg.msg_waitmsecs = 15000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_SSD_FORMAT_DONE) {
		mv_printk("ssd format failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


/*
 * Enable SSD Trim option
 */
static int
nvram_en_ssd_trim(nvram_hba_t *nvhba)
{
	int	ret;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SET_OPTIONS;
	msg.msg_data[0] = EN_SSD_TRIM;
	msg.msg_waitmsecs = 1000;

	/* set the dma-setup msg code */
	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_SET_OPTIONS_DONE) {
		mv_printk("set options failed. err_code: %04x\n",
				msg.msg_rsp);
		return -1;
	}

	return 0;
}

/*
 * Trim 8GB of SSD
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - fw did not trim the SSD.
 */
static int
nvram_ssd_trim(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	nvram_en_ssd_trim(nvhba);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SSD_TRIM;
	msg.msg_waitmsecs = 25000;

	/* set the dma-setup msg code */
	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_SSD_TRIM_DONE) {
		mv_printk("ssd trim failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


/*
 * Enable forced ECC for all DDR writes.
 *
 * This command will enable forced ECC for all DDR writes. The DDR controller
 * sets the given byte as the ECC byte instead of generating the ECC byte
 * based on the 64 bit data. Any memory location which is written after this
 * command and read back, will generate trigger an ECC error (unless the
 * given ECC byte matches with the actual ECC that would have been generated
 * for the 64 bit data).
 *
 * @nvhba - hba handle
 * @ecc_byte - ECC byte to be stored for all writes
 * @return 0 if successful,
 * non-zero - if fw timed out
 */
static int
nvram_enable_forced_ecc(nvram_hba_t *nvhba, u8_t ecc_byte)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_EN_FORCE_ECC;
	msg.msg_data[0] = ecc_byte;
	msg.msg_waitmsecs = 100;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_FORCE_ECC_ON) {
		mv_printk("Enable forced ECC failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


/*
 * Disable forced ECC.
 *
 * This command will disable forced ECC if it was previously enabled.
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - if fw timed out
 */
static int
nvram_disable_forced_ecc(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_DIS_FORCE_ECC;
	msg.msg_waitmsecs = 100;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_FORCE_ECC_OFF) {
		mv_printk("Disable forced ECC failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


static void
dev_busy_wait(nvram_hba_t *nvhba)
{
	while (jiffies < nvhba->nv_op_expires)
		msleep(1);
}

/*
 * Reset firmware (ioctl)
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_reset_fw(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	/* protection against multiple resets */
	ret = down_interruptible(&nvhba->nv_oplock);
	if (ret != 0)
		return -EINTR;

	/* wait for any pending operations to complete */
	dev_busy_wait(nvhba);

	dma_stop(nvhba, -EBUSY);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_RESET_CPU;
	msg.msg_waitmsecs = 3000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		/* reset FW forcefully */
		iowrite32(CPU_RESET_BIT, nvhba->nv_regs + CPU_CTRL_STAT_REG);
		udelay(50);
		iowrite32(0, nvhba->nv_regs + CPU_CTRL_STAT_REG);
	}

	/* wait for firmware to boot up */
	msleep(1500);

	nvram_set_time(nvhba);

	nvram_setup_dma(nvhba, nvhba->nv_dma_ctrl.nv_mode);

	up(&nvhba->nv_oplock);

	return 0;
}

static int
wam_pci_restore_device(nvram_hba_t *nvhba)
{
	int ret;
	struct pci_dev *pdev = nvhba->nv_pcidev;

	/* enable the pci for the device */
	if (pci_enable_device(pdev) != 0) {
		mv_printk("hardreset: failed to enable device\n");
		return -1;
	}

	/* pci-e window to access scratchpad */
	setup_sram_access(nvhba);

	/* pci-e window to access nvsram */
	setup_nvsram_access(nvhba);

	nvram_set_time(nvhba);

	/* initialize DMA by handshaking with firmware */
	ret = nvram_setup_dma(nvhba, nvhba->nv_dma_ctrl.nv_mode);
	if (ret != 0) {
		mv_printk("hardreset: failed to setup DMA\n");
	}

	/* reset the interrupt */
	ret = mvloki_request_irq(nvhba);
	if (ret != 0) {
		mv_printk("hardreset: request_irq failed\n");
	}

	return ret;
}


/*
 * Hard reset fw command
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_hard_reset_msg(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_HARD_RESET;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_HARD_RESET_ACK) {
		mv_printk("hard reset cmd failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


/*
 * Hard Reset (ioctl)
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 */
static int
nvram_hardreset(nvram_hba_t *nvhba)
{
	int ret;

	/* protection against multiple resets */
	ret = down_interruptible(&nvhba->nv_oplock);
	if (ret != 0)
		return -EINTR;

	/* wait for any pending operations to complete */
	dev_busy_wait(nvhba);

	/* tear down IO path */
	dma_stop(nvhba, -EBUSY);

	if (nvhba->nv_irq_enabled) {
		free_irq(nvhba->nv_pcidev->irq, nvhba);
	}
#if defined(LOKI_USE_MSI) && defined(CONFIG_PCI_MSI)
	if (nvhba->nv_msi_enabled) {
		pci_disable_msi(nvhba->nv_pcidev);
	}
#endif

	ret = nvram_hard_reset_msg(nvhba);
	if (ret != 0) {
		/* force hard reset if fw failed to respond */
		iowrite32(6, nvhba->nv_regs + CPU_RSTOUT_MASK);
		iowrite32(1, nvhba->nv_regs + CPU_SOFT_RESET);
	}

	/* wait for device to come up */
	msleep(2000);

	/* restore pci cfg space */
	wam_pci_restore_state(nvhba);

	/* initialize device */
	wam_pci_restore_device(nvhba);

	up(&nvhba->nv_oplock);

	return 0;
}


/*
 * Get Super-Cap Analog to Digital Converter registers (IOCTL interface)
 *
 * @nvhba - hba handle
 * @arg - user address pointing to reg_io_t structure.
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_read_scap_reg(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	reg_io_t	reg_cfg;
	wam_fwmsg_t	msg;

	/* read/write register */
	if (copy_from_user(&reg_cfg, (void __user *)arg,
				sizeof(reg_io_t))) {
		return -EACCES;
	}

	if (reg_cfg.reg_addr > 7) {
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_READ_SCAP_REG;
	msg.msg_data[0] = reg_cfg.reg_addr;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_READ_SCAP_REG_DONE) {
		mv_printk("Reading scap register failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	reg_cfg.reg_val = msg.msg_data[0];

	if (copy_to_user((void __user *)arg, &reg_cfg,
				sizeof(reg_io_t))) {
		return -EACCES;
	}

	return 0;
}


/*
 * Get Super-Cap voltage
 *
 * @nvhba - hba handle
 * @arg - user space reference to scap_vol_t to return voltage info
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_scap_vol(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	scap_vol_t	vol;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_SCAP_VOL;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_GET_SCAP_VOL_DONE) {
		mv_printk("Getting scap capacitance failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	vol.sc_cur_vol = msg.msg_data[0];
	vol.sc_exp_vol = msg.msg_data[1];

	if (copy_to_user((void __user *)arg, &vol,
				sizeof(scap_vol_t))) {
		return -EACCES;
	}

	return 0;
}

static int
nvram_get_info_priv(nvram_hba_t *nvhba, wam_info_t **info)
{
	int		ret = 0;
	u8_t		*buffer;
	unsigned long	paddr;
	wam_fwmsg_t	msg;

	buffer = (u8_t *) kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buffer == NULL)
		return -ENOMEM;

	memset(buffer, 0, sizeof(wam_info_t));

	paddr = __pa(buffer);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_INFO;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		kfree(buffer);
		return ret;
	}

	if (msg.msg_rsp != DRBL_RSP_GET_INFO_DONE) {
		mv_printk("Get info command failed. err_code: %04x\n",
				msg.msg_rsp);
		kfree(buffer);
		return msg.msg_rsp;
	}

	*info = (wam_info_t *)buffer;

	return 0;
}

/*
 * Get Super-Cap calibration result
 *
 * @nvhba - hba handle
 * @arg - user space reference to scap_vol_t to return voltage info
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_info(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	wam_info_t	*info = NULL;

	ret = nvram_get_info_priv(nvhba, &info);

	if (ret != 0) {
		return (ret);
	}

	if (copy_to_user((void __user *)arg, info,
				sizeof(wam_info_t))) {
		ret = -EACCES;
	}

	kfree(info);

	return ret;
}


/*
 * Get FW Debug info
 *
 * @nvhba - hba handle
 * @arg - user space reference to wam_dma_dbg_t
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_dbg_info(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u8_t		*buffer;
	unsigned long	paddr;
	wam_fwmsg_t	msg;

	buffer = (u8_t *) kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buffer == NULL)
		return -ENOMEM;

	memset(buffer, 0, PAGE_SIZE);

	paddr = __pa(buffer);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_DMA_DBG_INFO;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		kfree(buffer);
		return ret;
	}

	if (msg.msg_rsp != DRBL_RSP_DMA_DBG_INFO_COPIED) {
		mv_printk("Get dbg info command failed. err_code: %04x\n",
				msg.msg_rsp);
		kfree(buffer);
		return msg.msg_rsp;
	}

	if (copy_to_user((void __user *)arg, buffer,
				sizeof(wam_dma_dbg_t))) {
		kfree(buffer);
		return -EACCES;
	}

	kfree(buffer);
	return 0;
}

/*
 * Start Supercap Calibration
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_scap_calib_start(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u32_t		type;
	wam_fwmsg_t	msg;

	ret = down_interruptible(&nvhba->nv_oplock);
	if (ret != 0)
		return -EINTR;

	if (nvhba->nv_scap_calib) {
		ret = -EBUSY;
		goto calib_exit;
	}

	if (nvram_ping_fw(nvhba) != 0) {
		ret = -EBUSY;
		goto calib_exit;
	}

	type = arg;
	if (!type) {
		/* tear down IO path in case of offline check */
		ret = dma_stop(nvhba, -EBUSY);
		if (ret != 0) {
			mv_printk("dma stop failed! ret: %d\n", ret);
			goto calib_exit;
		}

		/* to gaurd against fw reset command - allow 60 sec wait */
		nvhba->nv_op_expires = jiffies + msecs_to_jiffies(60000);
	} else {
		/* set online health check bit */
		type = (1UL << 31);
	}

	nvhba->nv_calib_type = type;
	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SCAP_CALIB_START;
	msg.msg_data[0] = type;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		goto calib_exit;

	if (msg.msg_rsp != DRBL_RSP_SCAP_CALIB_STARTED) {
		mv_printk("scap calibration start failed. err_code: %04x\n",
				msg.msg_rsp);
		if (!type)
			nvram_setup_dma(nvhba, nvhba->nv_dma_ctrl.nv_mode);

		ret = msg.msg_rsp;
		goto calib_exit;
	}

	nvhba->nv_scap_calib = 1;

	ret = 0;

calib_exit:
	up(&nvhba->nv_oplock);
	return ret;
}

/*
 * Enable supercap discharge circuit
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_scap_discharge_on(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SCAP_DISCHARGE_ON;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_SCAP_DISCHARGE_ON) {
		mv_printk("enabling supercap discharge failed "
				"with code: %04x\n", msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


/*
 * Stop supercap discharging
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_scap_discharge_off(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SCAP_DISCHARGE_OFF;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_SCAP_DISCHARGE_OFF) {
		mv_printk("disabling supercap discharge failed "
				"with code: %04x\n", msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}
/*
 * Get Firmware version (ioctl)
 *
 * @nvhba - hba handle
 * @arg - user space reference to a buffer
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_fw_ver(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	char		*buf;
	unsigned long	buf_paddr;
	wam_fwmsg_t	msg;

	buf = (char *) kmalloc(4 << 10, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	buf_paddr = __pa(buf);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_FW_VERSION;
	msg.msg_data[0] = LSB_32(buf_paddr);
	msg.msg_data[1] = MSB_32(buf_paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		kfree(buf);
		return ret;
	}

	if (msg.msg_rsp != DRBL_RSP_FW_VERSION_DONE) {
		mv_printk("Getting firmware version failed. err_code: %04x\n",
				msg.msg_rsp);
		kfree(buf);
		return msg.msg_rsp;
	}

	if (copy_to_user((void __user *)arg, buf,
				256)) {
		kfree(buf);
		return -EACCES;
	}

	kfree(buf);
	return 0;
}


/*
 * Get nvram backup stats
 *
 * @nvhba - hba handle
 * @arg - user space reference to a struct of type backup_stats_t
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_backup_stats(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	backup_stats_t	*stats;
	unsigned long	stats_paddr;
	wam_fwmsg_t	msg;

	stats = (backup_stats_t *) kmalloc(4 << 10, GFP_KERNEL);
	if (stats == NULL)
		return -ENOMEM;

	stats_paddr = __pa(stats);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_BACKUP_STATS;
	msg.msg_data[0] = LSB_32(stats_paddr);
	msg.msg_data[1] = MSB_32(stats_paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		kfree(stats);
		return ret;
	}

	if (msg.msg_rsp != DRBL_RSP_GET_BACKUP_STATS_DONE) {
		mv_printk("Getting backup stats failed. err_code: %04x\n",
				msg.msg_rsp);
		kfree(stats);
		return msg.msg_rsp;
	}

	if (copy_to_user((void __user *)arg, stats,
				sizeof(backup_stats_t))) {
		kfree(stats);
		return -EACCES;
	}

	kfree(stats);
	return 0;
}


/*
 * Is NVRAM data valid?
 *
 * This API returns whether the backup & restore was successful
 *
 * @nvhba - hba handle
 * @arg - user space reference (u64_t) to return status in the following format
 *		63:32 - 0 - Data Invalid, 1 - Data Valid
 *		31:16 - backup error if non-zero
 *		15:0 - restore error if non-zero
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_is_data_valid(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u64_t		status;
	u32_t		err_info = -1;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_IS_DATA_VALID;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	err_info = msg.msg_data[0];

	if (msg.msg_rsp != DRBL_RSP_DATA_IS_VALID) {
		mv_printk("Data is corrupt. err_info: %08x\n", err_info);
		status = 0;
	} else {
		status = 1;
	}

	status <<= 32;
	status |= err_info;

	if (copy_to_user((void __user *)arg, &status,
				sizeof(u64_t))) {
		return -EACCES;
	}

	return 0;
}

/*
 * Get dram blocks which had errors during backup
 *
 * @nvhba - hba handle
 * @arg - user space reference to a struct of type backup_stats_t
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_err_blocks(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u32_t		nr_errs = 0;
	err_block_t	*eblks;
	unsigned long	paddr;
	wam_fwmsg_t	msg;

	eblks = (err_block_t *)kmalloc(32 << 10, GFP_KERNEL);
	if (eblks == NULL)
		return -ENOMEM;

	paddr = __pa(eblks);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_ERR_BLOCKS;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		kfree(eblks);
		return ret;
	}

	if (msg.msg_rsp != DRBL_RSP_GET_ERR_BLOCKS_DONE) {
		mv_printk("Getting err blocks failed. err_code: %04x\n",
				msg.msg_rsp);
		kfree(eblks);
		return msg.msg_rsp;
	}

	nr_errs = msg.msg_data[0];
	memset(&eblks[nr_errs], 0, sizeof(err_block_t));

	if (copy_to_user((void __user *)arg, eblks,
				sizeof(err_block_t) * (nr_errs + 1))) {
		kfree(eblks);
		return -EACCES;
	}

	kfree(eblks);
	return 0;
}


/*
 * Read supercap charger PMBus command output.
 *
 * @nvhba - hba handle
 * @arg - user reference to scap_chrgr_cmd_t.
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_scap_chrgr_read(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u32_t		val;
	wam_fwmsg_t	msg;
	scap_chrgr_cmd_t	chrgr_cmd;

	if (copy_from_user(&chrgr_cmd, (void __user *)arg,
				sizeof(scap_chrgr_cmd_t))) {
		return -EACCES;
	}

	val = chrgr_cmd.nr_bytes;
	val <<= 8;
	val |= chrgr_cmd.cmd;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_CHARGER_READ;
	msg.msg_data[0] = val;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_CHARGER_ACCESS_DONE) {
		mv_printk("Charger read command failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	chrgr_cmd.data = msg.msg_data[0];
	if (copy_to_user((void __user *)arg, &chrgr_cmd,
				sizeof(scap_chrgr_cmd_t))) {
		return -EACCES;
	}

	return 0;
}

/*
 * Set supercap charging voltage
 *
 * @nvhba - hba handle
 * @arg - charging voltage in millivolts
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_scap_chrgr_set_vol(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_CHARGER_SET_VOL;
	msg.msg_data[0] = (u32_t) arg;
	msg.msg_waitmsecs = 30000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_CHARGER_ACCESS_DONE) {
		mv_printk("Charger set vol command failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}

/*
 * Set DDR voltage
 *
 * @nvhba - hba handle
 * @arg - one of the four DDR voltage settings (0 to 3)
 *		0: 1.75 V
 *		1: 1.62 V
 *		2: 1.98 V
 *		3: 1.80 V
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_ddr_sel_vol(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	/* value cannot be > 3 */
	arg &= 0x3;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SEL_DDR_VOL;
	msg.msg_data[0] = (u32_t) arg;
	msg.msg_waitmsecs = 2000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_DDR_VOLSEL_DONE) {
		mv_printk("Charger set vol command failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


/*
 * Set supercap charger parameters using PMBus command.
 *
 * @nvhba - hba handle
 * @arg - user reference to scap_chrgr_cmd_t.
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_scap_chrgr_set(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u32_t		val;
	scap_chrgr_cmd_t	chrgr_cmd;
	wam_fwmsg_t	msg;

	if (copy_from_user(&chrgr_cmd, (void __user *)arg,
				sizeof(scap_chrgr_cmd_t))) {
		return -EACCES;
	}

	val = chrgr_cmd.data;
	val <<= 16;
	val |= chrgr_cmd.nr_bytes;
	val <<= 8;
	val |= chrgr_cmd.cmd;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_CHARGER_SET;
	msg.msg_data[0] = val;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_CHARGER_ACCESS_DONE) {
		mv_printk("Charger set command failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}

/*
 * Get Event Log start offset and rd/wr pointers
 *
 * @nvhba - hba handle
 * @arg - user space reference to a struct of type event_log_t
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_event_log_info(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u32_t		val;
	event_log_t	eventlog;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_EVNTLOG_PTRS;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_GET_EVNTLOG_PTRS_DONE) {
		mv_printk("getting event log pointers failed "
				"with msg code: %04x\n", msg.msg_rsp);
		return msg.msg_rsp;
	}

	val = msg.msg_data[0];
	eventlog.e_wr_ptr = (val >> 16) & 0xFFFF;
	eventlog.e_rd_ptr = val & 0xFFFF;
	eventlog.e_nvsram_offset = msg.msg_data[1];
	eventlog.e_max_events = msg.msg_data[2];

	if (copy_to_user((void __user *)arg, &eventlog,
				sizeof(event_log_t))) {
		return -EACCES;
	}

	return 0;
}


/*
 * Update event log read pointer
 *
 * The driver needs to update the event log read pointer after reading event
 * log entries
 *
 * @nvhba - hba handle
 * @rd_ptr - value of the read pointer to be set.
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_set_event_log_rdptr(nvram_hba_t *nvhba, u32_t rd_ptr)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SET_EVNTLOG_RDPTR;
	msg.msg_data[0] = rd_ptr;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_EVNTLOG_RDPTR_UPDATED) {
		mv_printk("setting event log rd_ptr failed "
				"with msg code: %04x\n", msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}


/*
 * Is NVRAM ready for writes?
 *
 * @nvhba - hba handle
 * @arg - user space reference to u64_t to return the status
 * The status will be updated in the following format
 *	63:32	- 0 - not ready / 1 - write ready
 *	31:24	- supercap charge percentage
 *	23:16	- supercap status (refer mv_common.h)
 *	15:8	- ssd dom status
 *	7:0	- dram post status
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_is_write_ready(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u32_t		val;
	u64_t		status;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_DMA_WRITE_READY;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp == DRBL_RSP_DMA_WRITE_READY) {
		val = 1;
	} else {
		val = 0;
	}

	status = val;
	status <<= 32;
	status |= msg.msg_data[0];

	if (copy_to_user((void __user *)arg, &status,
				sizeof(u64_t))) {
		return -EACCES;
	}

	return 0;
}



/*
 * clear the backup stats to latch stats from next time. The backup stats
 * record is not overwritten in case of incomplete backup and it requires
 * the driver to clear the record.
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_clear_backup_stats(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_CLEAR_BACKUP_STATS;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_BACKUP_STATS_CLEARED) {
		mv_printk("clearing backup stats record failed "
				"with code: %04x\n", msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}

/*
 * Get SSD Identify page
 *
 * @nvhba - hba handle
 * @arg - user space reference to buffer of size SSD_IDENTIFY_PAGE_SIZE
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_ssd_identify(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u64_t		paddr;
	char		*buf;
	wam_fwmsg_t	msg;

	buf = kmalloc(4 << 10, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	paddr = __pa(buf);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SSD_IDENTIFY;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		kfree(buf);
		return ret;
	}

	if (msg.msg_rsp != DRBL_RSP_SSD_IDENTIFY_DONE) {
		mv_printk("SSD identify command failed "
				"with code: %04x\n", msg.msg_rsp);
		kfree(buf);
		return -1;
	}

	if (copy_to_user((void __user *)arg, buf,
				SSD_IDENTIFY_PAGE_SIZE) != 0) {
		kfree(buf);
		return -EACCES;
	}

	kfree(buf);
	return 0;
}


/*
 * Update SSD firmware (ioctl)
 *
 * @nvhba - hba handle
 * @arg - reference to file_info_t structure containing a buffer and file size
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_ssd_fw_update(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u64_t		paddr;
	file_info_t	finfo;
	u8_t		*fw_page;
	dma_addr_t	fw_page_hdl;
	wam_fwmsg_t	msg;

	fw_page = dma_alloc_coherent(NULL, SSD_FW_UPDATE_PAGE_SIZE,
			&fw_page_hdl, GFP_KERNEL);
	if (fw_page == NULL)
		return -ENOMEM;

	memset(fw_page, 0, SSD_FW_UPDATE_PAGE_SIZE);

	memset(&finfo, 0, sizeof(file_info_t));

	if (copy_from_user(&finfo, (void __user *)arg, sizeof(file_info_t))
			!= 0) {
		mv_printk("copy from user failed\n");
		ret = -EACCES;
		goto fw_update_exit;
	}

	if (copy_from_user(fw_page, (void __user *)finfo.file_buf,
				finfo.file_size) != 0) {
		mv_printk("file buffer copy from user failed\n");
		ret = -EACCES;
		goto fw_update_exit;
	}

	paddr = (u64_t)fw_page_hdl;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SSD_FW_UPDATE;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_data[2] = finfo.file_size;
	msg.msg_waitmsecs = 40000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		goto fw_update_exit;

	if (msg.msg_rsp != DRBL_RSP_SSD_FW_UPDATE_DONE) {
		mv_printk("SSD fw update command failed "
				"with code: %04x\n", msg.msg_rsp);
		ret = -1;
		goto fw_update_exit;
	}

	ret = 0;

fw_update_exit:
	dma_free_coherent(NULL, SSD_FW_UPDATE_PAGE_SIZE, fw_page, fw_page_hdl);
	return ret;
}

/*
 * Get crash dump stack and registers (ioctl)
 *
 * @nvhba - hba handle
 * @arg - user space reference to a buffer
 * @return 0 if successful, 1 if no crash dump
 * negative - fw did not respond
 */
static int
nvram_get_fw_crashdump(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	unsigned int	*buf;
	unsigned long	buf_paddr;
	coredump_t	cb;
	u32_t		len;
	wam_fwmsg_t	msg;

	if (copy_from_user(&cb, (void __user *)arg,
				sizeof(coredump_t))) {
		return -EACCES;
	}

	buf = (unsigned int *) kmalloc((8 << 10), GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	memset(buf, 0, (8 << 10));
	memset(cb.core_buf, 0, cb.corebuf_size);
	len = (cb.corebuf_size > (8 << 10)) ? (8<<10):(cb.corebuf_size);

	buf_paddr = __pa(buf);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_FW_CRASHDUMP;
	msg.msg_data[0] = LSB_32(buf_paddr);
	msg.msg_data[1] = MSB_32(buf_paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		kfree(buf);
		return ret;
	}

	if (msg.msg_rsp != DRBL_RSP_FW_CRASHDUMP_DONE) {
		mv_printk("Get Firmware crash dump failed. err_code: %04x\n",
				msg.msg_rsp);
		kfree(buf);
		return msg.msg_rsp;
	}

	if (msg.msg_rsp == DRBL_RSP_FW_CRASHDUMP_EMPTY) {
		kfree(buf);
		return 1;
	}

	if (copy_to_user((void __user *)cb.core_buf, buf, len)) {
		kfree(buf);
		return -EACCES;
	}

	kfree(buf);
	return 0;
}


/*
 * Simulate Firmware Crash(ioctl)
 *
 * @nvhba - hba handle
 * @return 0 if successful,
 * non-zero - wrong fw response code
 */
static int
nvram_trigger_coredump(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_TRIGGER_FW_CRASH;
	msg.msg_waitmsecs = 40000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return 0;//ret;

	if (msg.msg_rsp != DRBL_RSP_FW_CRASH_TRIGGERED) {
		mv_printk("simulate coredump failed "
				"with code: %04x\n", msg.msg_rsp);
		return -1;
	}

	return 0;

}


/*
 * Clean Firmware Crash Record(ioctl)
 *
 * @nvhba - hba handle
 * @return 0 if successful, 1 if no crash dump
 * negative - wrong fw response code
 */
static int
nvram_clean_coredump(nvram_hba_t *nvhba)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_CLEAR_FW_CRASHDUMP;
	msg.msg_waitmsecs = 40000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_FW_CRASHDUMP_CLEARED) {
		mv_printk("clean coredump failed "
				"with code: %04x\n", msg.msg_rsp);
		return -1;
	}

	if (msg.msg_rsp == DRBL_RSP_FW_CRASHDUMP_EMPTY) {
		return 1;
	}

	return 0;

}


/*
 * Read device EEPROM (ioctl)
 *
 * @nvhba - hba handle
 * @arg - reference to file_info_t structure containing a buffer and file size
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_eeprom_read(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u64_t		paddr;
	file_info_t	finfo;
	u8_t		*buffer;
	wam_fwmsg_t	msg;

	buffer = (u8_t *) kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buffer == NULL)
		return -ENOMEM;

	memset(buffer, 0, MV_EEPROM_SIZE);

	memset(&finfo, 0, sizeof(file_info_t));

	if (copy_from_user(&finfo, (void __user *)arg, sizeof(file_info_t))
			!= 0) {
		mv_printk("copy from user failed\n");
		ret = -EACCES;
		goto read_exit;
	}

	paddr = __pa(buffer);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_READ_EEPROM;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_waitmsecs = 3000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		goto read_exit;

	if (msg.msg_rsp != DRBL_RSP_EEPROM_READ_DONE) {
		mv_printk("EEPROM read command failed "
				"with code: %04x\n", msg.msg_rsp);
		ret = -1;
		goto read_exit;
	}

	if (copy_to_user((void __user *)finfo.file_buf, buffer,
				MV_EEPROM_SIZE) != 0) {
		mv_printk("eeprom copy to user failed\n");
		ret = -EACCES;
		goto read_exit;
	}

	ret = 0;

read_exit:
	kfree(buffer);
	return ret;
}


/*
 * Update device EEPROM (ioctl)
 *
 * @nvhba - hba handle
 * @arg - reference to file_info_t structure containing a buffer and file size
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_eeprom_update(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u64_t		paddr;
	file_info_t	finfo;
	u8_t		*buffer;
	wam_fwmsg_t	msg;

	buffer = (u8_t *) kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buffer == NULL)
		return -ENOMEM;

	memset(buffer, 0, MV_EEPROM_SIZE);

	memset(&finfo, 0, sizeof(file_info_t));

	if (copy_from_user(&finfo, (void __user *)arg, sizeof(file_info_t))
			!= 0) {
		mv_printk("copy from user failed\n");
		ret = -EACCES;
		goto update_exit;
	}

	if (copy_from_user(buffer, (void __user *)finfo.file_buf,
				finfo.file_size) != 0) {
		mv_printk("file buffer copy from user failed\n");
		ret = -EACCES;
		goto update_exit;
	}

	paddr = __pa(buffer);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_UPDATE_EEPROM;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_data[2] = finfo.file_size;
	msg.msg_waitmsecs = 3000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		goto update_exit;

	if (msg.msg_rsp != DRBL_RSP_EEPROM_UPDATE_DONE) {
		mv_printk("EEPROM update command failed "
				"with code: %04x\n", msg.msg_rsp);
		ret = -1;
		goto update_exit;
	}

	ret = 0;

update_exit:
	kfree(buffer);
	return ret;
}

/*
 * Get thermal log
 *
 * @nvhba - hba handle
 * @arg - user space reference to a struct of type thermal_log_t
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_read_thermal_log(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u32_t		nr_recs = 0;
	void		*buffer;
	unsigned long	paddr;
	wam_fwmsg_t	msg;
	thermal_log_t	tlog;

	memset(&tlog, 0, sizeof(thermal_log_t));

	if (copy_from_user(&tlog, (void __user *)arg,
				sizeof(thermal_log_t)) != 0)
		return -EACCES;

	buffer = kmalloc(32 << 10, GFP_KERNEL);
	if (buffer == NULL)
		return -ENOMEM;

	paddr = __pa(buffer);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_READ_THERMAL_LOG;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0) {
		kfree(buffer);
		return ret;
	}

	if (msg.msg_rsp != DRBL_RSP_THERMAL_LOG_COPIED) {
		mv_printk("Reading thermal log failed. err_code: %04x\n",
				msg.msg_rsp);
		kfree(buffer);
		return msg.msg_rsp;
	}

	nr_recs = msg.msg_data[0];
	tlog.nr_records = nr_recs;

	if (copy_to_user((void __user *)tlog.temp_records, buffer,
				sizeof(wam_temprec_t) * nr_recs)) {
		kfree(buffer);
		return -EACCES;
	}

	if (copy_to_user((void __user *)arg, &tlog, sizeof(thermal_log_t))) {
		kfree(buffer);
		return -EACCES;
	}

	kfree(buffer);
	return 0;
}

/*
 * Clear thermal log
 *
 * @nvhba - hba handle
 * @arg - number of records to clear
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_clear_thermal_log(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_CLEAR_THERMAL_LOG;
	msg.msg_data[0] = (u32_t)arg;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_THERMAL_LOG_CLEARED) {
		mv_printk("Clearing thermal log failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}

/*
 * Set wam attribute
 *
 * @nvhba - hba handle
 * @arg - user space reference to attr_pair_t
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_set_attr_ioc(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	wam_fwmsg_t	msg;
	attr_pair_t	attrpair;

	memset(&attrpair, 0, sizeof(attr_pair_t));
	if (copy_from_user(&attrpair, (void __user *)arg,
				sizeof(attr_pair_t)) != 0)
		return -EACCES;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_SET_ATTR;
	msg.msg_data[0] = attrpair.w_attr;
	msg.msg_data[1] = attrpair.w_value;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_SET_ATTR_DONE) {
		mv_printk("set attr failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	return 0;
}

/*
 * Get wam attribute
 *
 * @nvhba - hba handle
 * @arg - user space reference to attr_pair_t
 * @return 0 if successful,
 * non-zero - fw did not respond
 */
static int
nvram_get_attr_ioc(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	wam_fwmsg_t	msg;
	attr_pair_t	attrpair;

	memset(&attrpair, 0, sizeof(attr_pair_t));
	if (copy_from_user(&attrpair, (void __user *)arg,
				sizeof(attr_pair_t)) != 0)
		return -EACCES;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_ATTR;
	msg.msg_data[0] = attrpair.w_attr;
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		return ret;

	if (msg.msg_rsp != DRBL_RSP_GET_ATTR_DONE) {
		mv_printk("get attr failed. err_code: %04x\n",
				msg.msg_rsp);
		return msg.msg_rsp;
	}

	attrpair.w_value = msg.msg_data[0];
	if (copy_to_user((void __user *)arg, &attrpair,
				sizeof(attr_pair_t))) {
		return -EACCES;
	}

	return 0;
}

#define is_barrier_char(req)	((req)->dma_barrier_req ? 'B' : 'N')

static u32_t
dump_dma_queue(char *buffer, u32_t size, dlist_t *list, char *queue_str)
{
	u32_t		len;
	u32_t		nr_reqs;
	dlist_t		*next, *prev;
	dma_request_t	*head_req, *tail_req;

	next = DLIST_NEXT(list);
	prev = DLIST_PREV(list);
	if (next == list) {
		len = wm_snprintf(buffer, size, "%s\t\t: EMPTY\n", queue_str);
		return len;
	}

	head_req = MV_CONTAINER_OF(next, dma_request_t, dma_link);
	tail_req = MV_CONTAINER_OF(prev, dma_request_t, dma_link);
	len = wm_snprintf(buffer, size, "%s\t\t: %llu [%c]",
			queue_str,
			head_req->dma_id, is_barrier_char(head_req));

	if (head_req == tail_req) {
		len += wm_snprintf(buffer + len, size - len, "\n");
	} else {
		nr_reqs = tail_req->dma_id - head_req->dma_id + 1;
		len += wm_snprintf(buffer + len, size - len, " to %llu [%c] "
				"Nr Reqs: %u\n",
				tail_req->dma_id, is_barrier_char(tail_req),
				nr_reqs);
	}

	return len;
}

static u32_t
dump_engine_status(nvram_hba_t *nvhba, char *buffer,
		u32_t size, u32_t unit_base)
{
	u32_t   len = 0;
	void	*regs = nvhba->nv_regs + unit_base;

	len += wm_snprintf(buffer + len, size - len, "0x%05x\t\t\t0x%08x\t"
			"%u\t\t%u\t\t%u\n", unit_base,
			ioread32(regs + XOR_CTRL_STATUS),
			ioread32(regs + XOR_DLVRY_Q_WR_PTR),
			ioread32(regs + XOR_DLVRY_Q_RD_PTR),
			ioread32(regs + XOR_CMPL_Q_WR_PTR));
	return len;
}


static u32_t
dump_all_engines(nvram_hba_t *nvhba, char *buffer, u32_t size)
{
	u32_t   len = 0;

	len += wm_snprintf(buffer + len, size - len, "\n");
	len += wm_snprintf(buffer + len, size - len,
			"XOR Unit\t\tControl\t\tWrPtr\t\tRdPtr\t\tCmplPtr\n");

	len += dump_engine_status(nvhba, buffer + len, size - len, 0x61000);
	len += dump_engine_status(nvhba, buffer + len, size - len, 0);
	len += dump_engine_status(nvhba, buffer + len, size - len, 0x70000);
	len += dump_engine_status(nvhba, buffer + len, size - len, 0x60000);

	return len;
}


static u32_t
dump_geniostats(wam_geniostats_t *stats, char *buffer, u32_t size, char *iotype)
{
	u32_t   len = 0;

	len += wm_snprintf(buffer + len, size - len, "%s\t\t%-16llu%-24llu"
			"%-16u%-16u\n", iotype, stats->st_ios,
			stats->st_bytes >> 10,
			(stats->st_min_reqsize == (u64_t)(u32_t)(-1) ?
			0 : stats->st_min_reqsize), stats->st_max_reqsize);

	return len;
}

static u32_t
dump_iostats_hdr(char *buffer, u32_t size)
{
	u32_t	len = 0;

	len += wm_snprintf(buffer + len, size - len, "\n");

	len += wm_snprintf(buffer + len, size - len, "\t\t\t%-16s%-24s"
			"%-16s%-16s\n", "Ops", "Transferred (in KB)",
			"Min Reqsize", "Max Reqsize");

	return len;
}


static u32_t
dump_iostats(wam_iostats_t *stats, char *buffer, u32_t size, char *device)
{
	u32_t	len = 0;
	char	iotype[256];

	wm_snprintf(iotype, 256, "%s-%s", device, "reads");
	len += dump_geniostats(&stats->st_rd_stats, buffer + len, size - len,
			iotype);

	wm_snprintf(iotype, 256, "%s-%s", device, "writes");
	len += dump_geniostats(&stats->st_wr_stats, buffer + len, size - len,
			iotype);
	return len;
}

static u32_t
nvram_dump_stats(nvram_hba_t *nvhba, char *buffer, u32_t size)
{
	u32_t		len = 0;
	wam_stats_t	*stats = &nvhba->nv_stats;
	u64_t		cmpls_per_tsklt;

	len += wm_snprintf(buffer + len, size - len, "\n%-24s: %llu\n",
			"Interrupts", stats->st_intrs);
	len += wm_snprintf(buffer + len, size - len, "%-24s: %llu\n",
			"Tasklet Runs", stats->st_tsklt_runs);
	len += wm_snprintf(buffer + len, size - len, "%-24s: %u\n",
			"Rescues", stats->st_io_rescues);

	cmpls_per_tsklt = nvhba->nv_dma_ctrl.nv_cmpltd_id;
	cmpls_per_tsklt *= 100;
	cmpls_per_tsklt /= stats->st_tsklt_runs;

	len += wm_snprintf(buffer + len, size - len, "%-24s: %u.%02u\n",
			"Cmpls Per Tasklet", (u32_t)cmpls_per_tsklt / 100,
			(u32_t)cmpls_per_tsklt % 100);

	len += dump_iostats_hdr(buffer + len, size - len);

	len += dump_iostats(&stats->st_cdev_stats, buffer + len, size - len,
			"char");
	len += dump_iostats(&stats->st_bdev_stats, buffer + len, size - len,
			"block");
	return len;
}

/*
 * Dump DMA Pointers and Stats
 * @nvhba - hba handle
 * @buffer - a buffer which is atleast 4K large to dump all the information
 */
static int
dump_dma_state(nvram_hba_t *nvhba, char *buffer)
{
	char		*state_buf = buffer;
	char		*tmp_buf;
	u32_t		size = MV_DMA_DUMP_PAGE_SIZE;
	u32_t		len;
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;

	tmp_buf = state_buf;
	len = wm_snprintf(tmp_buf, size, "Driver Version\t\t: %s\n",
			g_wam_drv_version);
	tmp_buf += len;
	size -= len;

	len = wm_snprintf(tmp_buf, size, "DMA State\t\t: %s\n",
			dma_state_strings[dma_ctrl->nv_state]);
	tmp_buf += len;
	size -= len;

	len = wm_snprintf(tmp_buf, size, "DMA Queued ID\t\t: %llu\n"
			"DMA Completed ID\t: %llu\n",
			dma_ctrl->nv_queued_id,
			dma_ctrl->nv_cmpltd_id);
	tmp_buf += len;
	size -= len;

	spin_lock_bh(&dma_ctrl->nv_lock);

	len = dump_dma_queue(tmp_buf, size, &dma_ctrl->nv_request_list,
			"Request Queue");
	tmp_buf += len;
	size -= len;

	len = dump_dma_queue(tmp_buf, size, &dma_ctrl->nv_posted_list,
			"Posted Queue");
	tmp_buf += len;
	size -= len;

	len = wm_snprintf(tmp_buf, size, "Dlvry Ptr\t\t: %u\n"
			"Driver Cmpl Ptr\t\t: %u\n"
			"Device Cmpl Ptr\t\t: %u\n",
			dma_ctrl->nv_sw_dlvryq_wptr,
			dma_ctrl->nv_cmplq_wptr,
			dma_ctrl->nv_cmplq->cmpl_wr_ptr);
	tmp_buf += len;
	size -= len;

	len = dump_all_engines(nvhba, tmp_buf, size);
	tmp_buf += len;
	size -= len;

	len = nvram_dump_stats(nvhba, tmp_buf, size);
	tmp_buf += len;
	size -= len;

	spin_unlock_bh(&dma_ctrl->nv_lock);

	/* terminate the string */
	tmp_buf[0] = 0;

	return 0;
}

/*
 * Dump DMA Pointers (ioctl interface)
 * @nvhba - hba handle
 * @arg - user space reference to a buffer which is atleast 4K large
 */
static int
nvram_dump_state(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret;
	char		*state_buf;
	u32_t		size = MV_DMA_DUMP_PAGE_SIZE;

	state_buf = (char *)kmalloc(size, GFP_KERNEL);
	if (state_buf == NULL)
		return -ENOMEM;

	dump_dma_state(nvhba, state_buf);

	ret = copy_to_user((void __user *)arg, state_buf,
				MV_DMA_DUMP_PAGE_SIZE);

	if (ret != 0)
		ret = -EACCES;

	kfree(state_buf);
	return ret;
}

/*
 * Log DMA Pointers (print to kernel buffer)
 * @nvhba - hba handle
 */
static int
log_dma_state(nvram_hba_t *nvhba)
{
	char		*state_buf;
	u32_t		size = MV_DMA_DUMP_PAGE_SIZE;

	state_buf = (char *)kmalloc(size, GFP_KERNEL);
	if (state_buf == NULL)
		return -ENOMEM;

	dump_dma_state(nvhba, state_buf);

	mv_printk("%s\n", state_buf);

	kfree(state_buf);
	return 0;
}


/*
 * Ping the firmware and see if it responds
 * @nvhba - hba handle
 * @return - 0 if fw responded, -1 if firmware did not respond
 */
static int
nvram_ping_fw(nvram_hba_t *nvhba)
{
	int ret;
	u32_t version;
	u32_t fw_msg_version;
	wam_fwmsg_t	msg;

	version = (FW_MSG_VER_MAJOR << 16) | FW_MSG_VER_MINOR;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_PING;
	msg.msg_data[0] = version;
	msg.msg_data[2] = 0xFFFFFFFF;
	msg.msg_waitmsecs = 1000;

	/* set the ping msg code */
	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0 || msg.msg_rsp != DRBL_RSP_HELLO)
		return -1;

	fw_msg_version = msg.msg_data[0];
	if (fw_msg_version != 0xFFFFFFFF) {
		if (fw_msg_version != version)
			nvhba->nv_pkg_ver_comp = NV_PKG_VER_MISMATCHED;
		else
			nvhba->nv_pkg_ver_comp = NV_PKG_VER_COMPATIBLE;
	} else {
		nvhba->nv_pkg_ver_comp = NV_PKG_VER_COMPATIBLE;
	}

	return 0;
}



/*
 * Process DMA completions and invoke the callback routines. This function
 * is called by the hba thread whenever it is signalled.
 *
 * @dma_ctrl	- dma control handle of an HBA
 */
static void
dma_process_completion(nvram_dma_t *dma_ctrl, u32_t dev_id)
{
	int		i;
	int		diff;
	u32_t		status;
	u32_t		slot_idx;
	u32_t		cmpltd_slot;
	cmplq_entry_t	*cmpl_ent;
	dma_done_t	dma_done;
	dma_request_t	*dma_req;
	dlist_t		*link;

	/* slot completed by the firmware */
	cmpltd_slot = dma_ctrl->nv_cmplq->cmpl_wr_ptr;

	diff = cmpltd_slot - dma_ctrl->nv_cmplq_wptr;
	if (diff < 0) {
		/* queue wrap around case */
		diff += dma_ctrl->nv_nr_slots;
	}

	slot_idx = dma_ctrl->nv_cmplq_wptr;
	for (i = 0; i < diff; i++) {
		slot_idx++;
		if (slot_idx >= dma_ctrl->nv_nr_slots) {
			slot_idx = 0;
		}

		cmpl_ent = &(dma_ctrl->nv_cmplq->cmpl_ents[slot_idx]);
		status = cmpl_ent->cmd_status;

		/* pick the request from the circular list */
		link = dlist_rm_front(&dma_ctrl->nv_posted_list);

		/* assert the following conditions */
		if (link == NULL ||
				!(status & DMA_CMD_COMPLETE) ||
				(cmpl_ent->cmd_slot != slot_idx)) {
			mv_printk("Assert Condition!. Link: %p, Status: %08x, "
					"Prev Cmpltd Slot: %u, "
					"Cmpltd Slot: %u\n",
					link, status,
					cmpltd_slot,
					dma_ctrl->nv_cmplq_wptr);
			return;
		}

		/* clear the completion queue entry */
		memset(cmpl_ent, 0, sizeof(cmplq_entry_t));

		status &= ~DMA_CMD_COMPLETE;
		dma_req = MV_CONTAINER_OF(link, dma_request_t, dma_link);
		dma_req->dma_status = status;

		/* if ecc error, the dma engine will need to be reset */
		if (status != 0) {
			dma_ctrl->nv_state = DMA_NEED_RESET;
			dump_ecc_info(dev_id, dma_ctrl, slot_idx);
		}

		dma_done = dma_req->dma_done;
		if (dma_done != NULL)
			dma_done(dma_req);
		else
			printk("done cb NULL for req %p\n", dma_req);
	}

	/* update the pointers */
	dma_ctrl->nv_cmpltd_id += i;
	dma_ctrl->nv_cmplq_wptr = slot_idx;

	if (diff) {
		wake_up_all(&dma_ctrl->nv_broadcast_event);
	}
}

static int
get_ecc_info(nvram_hba_t *nvhba, ecc_err_info_t *record)
{
	int		ret = 0;
	u64_t		paddr;
	u8_t		*buffer;
	wam_fwmsg_t	msg;

	buffer = (u8_t *) kmalloc(PAGE_SIZE, GFP_ATOMIC);
	if (buffer == NULL)
		return -ENOMEM;

	memset(buffer, 0, sizeof(ecc_err_info_t));
	paddr = __pa(buffer);

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_GET_ECC_INFO;
	msg.msg_data[0] = LSB_32(paddr);
	msg.msg_data[1] = MSB_32(paddr);
	msg.msg_waitmsecs = 1000;

	ret = mv_send_msg(nvhba, &msg);
	if (ret != 0)
		goto read_exit;

	if (msg.msg_rsp != DRBL_RSP_ECC_INFO_COPIED) {
		mv_printk("Get ECC error command failed "
				"with code: %04x\n", msg.msg_rsp);
		ret = -1;
		goto read_exit;
	}

	memcpy(record, buffer, sizeof(ecc_err_info_t));
	ret = 0;

read_exit:
	kfree(buffer);
	return ret;
}

/*
 * DMA completion soft IRQ handler
 *
 * @nvhba - hba handle
 * @return - nr of completions handled
 */
static u32_t
dma_softirq_handler(nvram_hba_t *nvhba)
{
	u32_t		wr_ptr;
	u64_t		cmpl_id;
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;
	volatile u32_t	*cmplq_wr_ptr;

	spin_lock_bh(&dma_ctrl->nv_lock);

	cmpl_id = dma_ctrl->nv_cmpltd_id;
	cmplq_wr_ptr = &dma_ctrl->nv_cmplq->cmpl_wr_ptr;

	do {
		wr_ptr = *cmplq_wr_ptr;
		dma_process_completion(dma_ctrl, nvhba->nv_minor);

		mb();

		if (dma_ctrl->nv_state != DMA_INITIALIZED)
			break;

		/* loop if the wr_ptr is updated by the firmware */
	} while (wr_ptr != *cmplq_wr_ptr);

	cmpl_id = dma_ctrl->nv_cmpltd_id - cmpl_id;

	if (dma_ctrl->nv_state == DMA_NEED_RESET) {
		dma_reset(nvhba);
		dma_process_pending_requests(nvhba);
	}

	spin_unlock_bh(&dma_ctrl->nv_lock);

	return (unsigned int)cmpl_id;
}

static u32_t
wam_get_cmpl_cause(nvram_hba_t *nvhba)
{
	if (nvhba->nv_alert_count == nvhba->nv_shdw_cause->fw_alert_counter) {
		return FW_DRBL_DMA_CMPLT;
	} else {
		nvhba->nv_alert_count++;
		return  nvhba->nv_shdw_cause->fw_shdw_intr_cause;
	}
}

static u32_t
wam_get_intr_cause(nvram_hba_t *nvhba)
{
	return (ioread32(nvhba->nv_regs + CPU_2_PCIE_A_DRBL));
}

static int
nv_error_inj(nvram_hba_t *nvhba, u32_t err)
{
	wam_fwmsg_t	msg;

	memset(&msg, 0, sizeof(wam_fwmsg_t));

	msg.msg_code = DRBL_MSG_INJECT_ERR;
	msg.msg_data[0] = err;
	msg.msg_waitmsecs = 1000;

	return mv_send_msg(nvhba, &msg);
}

static int
nv_get_drv_version(unsigned long arg)
{
	wam_drv_version_t drv;

	memset(&drv, 0, sizeof(wam_drv_version_t));

	drv.w_drv_major = WAM_DRV_VERSION_MAJOR;
	drv.w_drv_minor = WAM_DRV_VERSION_MINOR;
	drv.w_drv_bugfix = WAM_DRV_VERSION_BUGFIX;

	if (copy_to_user((void __user *)arg, &drv,
				sizeof(wam_drv_version_t))) {
		return -EACCES;
	}

	return 0;
}


static void
nv_clear_dev_alert(nvram_hba_t *nvhba, u32_t alerts)
{
	unsigned long flags;

	spin_lock_irqsave(&g_wam_alert_lock, flags);

	nvhba->nv_alerts ^= alerts;

	if ((nvhba->nv_pending_alerts | nvhba->nv_alerts) == 0)
		g_wam_alert_dev_map &= ~(1 << nvhba->nv_minor);

	spin_unlock_irqrestore(&g_wam_alert_lock, flags);
}

static u32_t
nv_get_dev_alerts(nvram_hba_t *nvhba)
{
	unsigned long flags;

	spin_lock_irqsave(&g_wam_alert_lock, flags);

	nvhba->nv_alerts |= nvhba->nv_pending_alerts;
	nvhba->nv_pending_alerts = 0;

	spin_unlock_irqrestore(&g_wam_alert_lock, flags);

	return nvhba->nv_alerts;
}

static int
nv_wait_on_alerts(unsigned long arg)
{
	int ret = 0;
	int i;
	u32_t alerts = 0;
	wam_alert_info_t info;

	ret = copy_from_user(&info, (void __user *)arg,
			sizeof(wam_alert_info_t));
	if (ret != 0)
		return -EACCES;

	if (g_wam_alert_dev_map == 0) {
		switch ((int)info.w_alert_wait_msec) {
		case 0:
			break;

		case -1:
			/* wait infinitely */
			mv_waitq_wait(&g_wam_alert_waitq);
			break;

		default:
			/* wait for the given timeout period */
			ret = mv_waitq_timedwait(&g_wam_alert_waitq,
					info.w_alert_wait_msec);
			break;
		}
	}

	if (g_wam_alert_dev_map == 0) {
		if (ret == 0)
			ret = -ETIMEDOUT;

		if (ret == -ETIMEDOUT)
			ret = -ETIME;
		else
			ret = -EINTR;

		return ret;
	} else {
		for (i = 0; i < nvram_hba_cnt; i++) {
			alerts = nv_get_dev_alerts(nvram_hba[i]);
			if (alerts)
				break;
		}
	}


	info.w_dev_map = g_wam_alert_dev_map;
	info.w_alert_cause = alerts;

	ret = copy_to_user((void __user *)arg, &info,
			sizeof(wam_alert_info_t));

	return ret;
}

static void
nv_report_alerts(nvram_hba_t *nvhba, u32_t alerts)
{
	unsigned long flags;

	spin_lock_irqsave(&g_wam_alert_lock, flags);

	g_wam_alert_dev_map |= (1 << nvhba->nv_minor);
	nvhba->nv_pending_alerts |= alerts;

	spin_unlock_irqrestore(&g_wam_alert_lock, flags);

	mv_waitq_signal(&g_wam_alert_waitq);
}

static void
doorbell_handler(nvram_hba_t *nvhba)
{
	u32_t		cause;
	u32_t		handled = 0;
	ecc_err_info_t	ecc_rec;
	u32_t		alerts = 0;
	u32_t		mode;
	u32_t		power_cycle = 0;


	cause = nvhba->nv_get_cause(nvhba);
	if (cause == 0xFFFFFFFF) {
		mv_printk("dev[%u] disconnected!\n", nvhba->nv_minor);
		return;
	}

	if (cause & FW_DRBL_DMA_CMPLT) {
		clear_drbl(nvhba, FW_DRBL_DMA_CMPLT);
		dma_softirq_handler(nvhba);
	}

	if (cause & FW_DRBL_ECC_ERROR) {
		if (get_ecc_info(nvhba, &ecc_rec) == 0)
			dump_ecc_rec(nvhba->nv_minor, &ecc_rec);
		handled |= FW_DRBL_ECC_ERROR;
		alerts |= NV_ALRT_ECC_ERROR;
	}

	if (cause & FW_DRBL_CPU_CRASH) {
		mv_printk("Firmware crashed!\n");
		mdelay(100);
		nvram_setup_dma(nvhba, nvhba->nv_dma_ctrl.nv_mode);
		handled |= FW_DRBL_CPU_CRASH;
		alerts |= NV_ALRT_FW_CRASH;
	}

	if (cause & FW_DRBL_DEV_WR_READY) {
		mv_printk("Device Write Ready\n");
		handled |= FW_DRBL_DEV_WR_READY;
	}

	if (cause & FW_DRBL_SCAP_DISCONNECTED) {
		mv_printk("CRITICAL: Supercap is disconnected!\n");
		handled |= FW_DRBL_SCAP_DISCONNECTED;
		alerts |= NV_ALRT_COMP_FAIL;
	}

	if (cause & FW_DRBL_SCAP_VOLTAGE_LOW) {
		mv_printk("WARNING: Supercap voltage reached unsafe level!\n");
		handled |= FW_DRBL_SCAP_VOLTAGE_LOW;
		alerts |= NV_ALRT_SCAP_VOL_LOW;
	}

	if (cause & FW_DRBL_SSD_DISCONNECTED) {
		mv_printk("CRITICAL: SSD is disconnected!\n");
		handled |= FW_DRBL_SSD_DISCONNECTED;
		alerts |= NV_ALRT_COMP_FAIL;
		power_cycle++;
	}

	if (cause & FW_DRBL_SSD_FAILURE) {
		mv_printk("CRITICAL: SSD Failure reported!\n");
		handled |= FW_DRBL_SSD_FAILURE;
		alerts |= NV_ALRT_COMP_FAIL;
		power_cycle++;
	}

	if (cause & FW_DRBL_SSD_EOL) {
		mv_printk("WARNING: SSD reaching its end of life!\n");
		handled |= FW_DRBL_SSD_EOL;
		alerts |= NV_ALRT_COMP_FAIL;
	}

	if (cause & FW_DRBL_DATA_BACKUP_DONE) {
		mv_printk("Data backup complete. Setting up read-only mode\n");
		clear_drbl(nvhba, FW_DRBL_DATA_BACKUP_DONE);

		if (nvhba->nv_scap_calib)
			mode = DRAM_RDONLY_MODE;
		else
			mode = nvhba->nv_dma_ctrl.nv_mode;

		nvram_setup_dma(nvhba, mode);
	}

	if (cause & FW_DRBL_SCAP_CALIB_DONE) {
		mv_printk("Supercap calibration completed\n");
		nvhba->nv_scap_calib = 0;
		handled |= FW_DRBL_SCAP_CALIB_DONE;
	}

	if (cause & FW_DRBL_HANDSHAKE_READY) {
		mv_printk("Firmware ready to handshake\n");
		handled |= FW_DRBL_HANDSHAKE_READY;
	}

	if (cause & FW_DRBL_SCAP_DEGRADED) {
		mv_printk("[Dev: %u] supercap degraded\n", nvhba->nv_minor);
		handled |= FW_DRBL_SCAP_DEGRADED;
		alerts |= NV_ALRT_SCAP_DEGRADED;
	}

	if (cause & FW_DRBL_SCAP_GOOD) {
		mv_printk("[Dev: %u] supercap health is now good\n", nvhba->nv_minor);
		handled |= FW_DRBL_SCAP_GOOD;
		alerts |= NV_ALRT_SCAP_GOOD;
	}

	if (cause & FW_DRBL_MAX_TEMP_ALERT) {
		mv_printk("[Dev: %u] Thermal Alert\n", nvhba->nv_minor);
		handled |= FW_DRBL_MAX_TEMP_ALERT;
		alerts |= NV_ALRT_MAX_TEMP;
	}

	if (cause & FW_DRBL_MEMORY_TEST_FAILED) {
		mv_printk("CRITICAL: Power On Memory Test Failed!\n");
		handled |= FW_DRBL_MEMORY_TEST_FAILED;
		alerts |= NV_ALRT_COMP_FAIL;
		power_cycle++;
	}

	if (alerts) {
		nv_report_alerts(nvhba, alerts);
	}

	if (power_cycle) {
		mv_printk("Please try powering off the system for 10 mins\n");
	}

	clear_drbl(nvhba, handled);
}


#ifdef CONFIG_ENHANCED_PERF_MODE
/*
 * Copy PRD Table from host memory to device scratchpad memory.
 *
 * @sram_prd - cmd prd table location in scratchpad
 * @host_prd - cmd prd table location in host memory
 */
static void
copy_prd_tbl(cmd_prd_tbl_t *sram_prd, cmd_prd_tbl_t *host_prd)
{
	prd_t *dst_prd = (prd_t *)sram_prd;
	prd_t *src_prd = (prd_t *)host_prd;
	u32_t nr_prds = sizeof(*host_prd) / sizeof(prd_t);
	u32_t i;

	for (i = 0; i < nr_prds; i++) {
		if (src_prd[i].prd_byte_cnt != 0)
			memcpy_toio(&dst_prd[i], &src_prd[i], sizeof(prd_t));
	}
}
#endif

static void
dump_prd(prd_t *prd, int nr_prds)
{
	int i;

	for (i = 0; i < nr_prds; i++)
		mv_printk("[%d] %08x %08x %u %u\n", i,
				prd[i].prd_addr_high, prd[i].prd_addr_low,
				prd[i].prd_xbar_ifc, prd[i].prd_byte_cnt);
}


/*
 * Setup the given DMA request at the next available slot in the
 * ring structure.
 *
 * @dma_ctrl	- dma control handle
 * @dma_req	- dma request
 * @regs	- register base address
 * @return 0 if successful or
 * DMA_QUEUE_FULL - if no slot is available in the ring
 * DMA_STATE_ERROR - DMA engine encountered ecc error. cannot queue new
 * request until the engine is reset.
 */
static int
dma_setup_request(nvram_dma_t *dma_ctrl, dma_request_t *dma_req, void *regs)
{
	u32_t		cmd_idx;
#ifdef CONFIG_ENHANCED_PERF_MODE
	u32_t		eng_id;
	u32_t		eng_slot;
#endif

	if (dma_ctrl->nv_state != DMA_INITIALIZED)
		return DMA_STATE_ERROR;

	cmd_idx = dma_ctrl->nv_sw_dlvryq_wptr + 1;
	if (cmd_idx > dma_ctrl->nv_nr_slots) {
		mv_printk("next cmd_idx [%u] > dma_ctrl->nv_sw_dlvryq_wptr "
				"[%u]\n", cmd_idx, dma_ctrl->nv_nr_slots);
		BUG();
	}

	if (cmd_idx == dma_ctrl->nv_nr_slots) {
		cmd_idx = 0;
	}

	if (cmd_idx == dma_ctrl->nv_cmplq_wptr) {
		/* queue full */
		return DMA_QUEUE_FULL;
	}

	if ((dma_req->dma_id - dma_ctrl->nv_cmpltd_id) >=
			(dma_ctrl->nv_nr_slots - 2)) {
		return DMA_QUEUE_FULL;
	}

	/* unlink from request queue and add it to posted request queue */
	dlist_rm(&dma_req->dma_link);
	dlist_add_back(&dma_ctrl->nv_posted_list, &dma_req->dma_link);

	dma_req->dma_ticks = get_clock_ticks();

	/* program the byte count for the command */
	iowrite32(dma_req->dma_byte_count,
			&dma_ctrl->nv_cmd_tbl[cmd_idx].cmd_byte_count);

	/* update the pointers */
	dma_ctrl->nv_sw_dlvryq_wptr = cmd_idx;

#ifdef CONFIG_ENHANCED_PERF_MODE
	/* copy directly to scratchpad prd table */
	copy_prd_tbl(&dma_ctrl->nv_sram_prd[cmd_idx], &dma_req->dma_prd_tbl);

	mmiowb();

	eng_id = cmd_idx % 3;
	eng_slot = cmd_idx / 3;

	iowrite32(eng_slot, regs + dma_dlvryq_wrptr[eng_id]);
#else
	/* copy the command prd table to the shadow prd table */
	memcpy(&dma_ctrl->nv_shadow_prd[cmd_idx], &dma_req->dma_prd_tbl,
			sizeof(cmd_prd_tbl_t));
#endif

	return 0;
}

#ifndef CONFIG_ENHANCED_PERF_MODE
/*
 * Post the commands to the hardware. This function is invoked after
 * configuring one or more commands to minimize the pci-e writes
 *
 * @dma_ctrl	- dma control handle
 * @regs	- register base address
 */
static void
dma_post_to_hw(nvram_dma_t *dma_ctrl, void *regs)
{
	if (dma_ctrl->nv_sw_dlvryq_wptr == dma_ctrl->nv_hw_dlvryq_wptr)
		return;

	dma_ctrl->nv_hw_dlvryq_wptr = dma_ctrl->nv_sw_dlvryq_wptr;
	iowrite32(dma_ctrl->nv_sw_dlvryq_wptr, regs + DMA_DLVRY_Q_WR_PTR);
}
#endif

/*
 * Function to print a DMA request (for debug)
 */
void
dma_dump_request(dma_request_t *req)
{
	prd_t	*prd = (prd_t *)&req->dma_prd_tbl;

	mv_printk("[%6u KB] [next: %p] [done: %p]\n",
			req->dma_byte_count >> 10,
			req->dma_next,
			req->dma_done);

	dump_prd(prd, MV_PRDS_PER_CMD);
}

static void
dump_ecc_rec(u32_t dev_id, ecc_err_info_t *ecc_rec)
{
	mv_printk("ECC ERROR: Dev: %u, Type: %s, Addr: %s-0x%08x, Exp: %02x "
			"Found: %02x, Data: 0x%08x_%08x\n", dev_id,
			(ecc_rec->ecc_type) ? "Double" : "Single",
			memory_ifc_strings[ecc_rec->ecc_ifc],
			ecc_rec->ecc_addr_low, ecc_rec->ecc_calc_val,
			ecc_rec->ecc_rcvd_val, ecc_rec->ecc_data_low,
			ecc_rec->ecc_data_high);
#if 0
	mv_printk("ECC Error encountered in WAM device %u\n", dev_id);
	mv_printk("Error Type\t\t: %s\n", (ecc_rec->ecc_type) ?
			"Double" : "Single");
	mv_printk("Address low\t\t: %08x\n", ecc_rec->ecc_addr_low);
	mv_printk("XBar Interface\t\t: %s\n",
			memory_ifc_strings[ecc_rec->ecc_ifc]);
	mv_printk("Calculated ECC val\t: %02x\n", ecc_rec->ecc_calc_val);
	mv_printk("Received ECC val\t\t: %02x\n", ecc_rec->ecc_rcvd_val);
	mv_printk("Data low 32\t\t: %08x\n", ecc_rec->ecc_data_low);
	mv_printk("Data high 32\t\t: %08x\n", ecc_rec->ecc_data_high);
#endif

}

/*
 * Dump ECC Error Info record for the given slot
 */
static void
dump_ecc_info(u32_t dev_id, nvram_dma_t *dma_ctrl, u32_t slot_id)
{
	u32_t err_idx;
	ecc_err_info_t *ecc_rec;

	err_idx = slot_id % 3;
	ecc_rec = &dma_ctrl->nv_cmplq->cmpl_ecc_info[err_idx];

	dump_ecc_rec(dev_id, ecc_rec);
}


/*
 * Walk the request queue and post dma requests to hw.
 *
 * @nvhba - hba handle
 */
static void
dma_process_pending_requests(nvram_hba_t *nvhba)
{
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;
	dma_request_t	*dma_req;
	dlist_t		*next;
	dlist_t		*node;
	int		ret;
	u32_t		count = 0;

	next = DLIST_NEXT(&dma_ctrl->nv_request_list);
	while (next != &dma_ctrl->nv_request_list) {
		node = next;
		next = DLIST_NEXT(node);

		dma_req = MV_CONTAINER_OF(node, dma_request_t, dma_link);

		/*
		 * In case of a barrier request, wait for posted requests
		 * to complete.
		 */
		if (dma_req->dma_barrier_req &&
				!dlist_empty(&dma_ctrl->nv_posted_list))
			break;

		ret = dma_setup_request(dma_ctrl, dma_req, nvhba->nv_regs);
		if (ret != 0) {
			if (ret == DMA_QUEUE_FULL) {
				break;
			} else {
				return;
			}
		}
		count++;
	}

#ifndef CONFIG_ENHANCED_PERF_MODE
	if (count != 0)
		dma_post_to_hw(dma_ctrl, nvhba->nv_regs);
#endif
}


static void
dma_process_requests(nvram_hba_t *nvhba)
{
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;

	spin_lock_bh(&dma_ctrl->nv_lock);

	dma_process_pending_requests(nvhba);

	spin_unlock_bh(&dma_ctrl->nv_lock);
}

static void
nv_enable_irq(nvram_hba_t *nvhba)
{
	u32_t val = 0;

	/* enable doorbell mask */
	val |= FW_DRBL_DMA_CMPLT;
	val |= FW_DRBL_ECC_ERROR;
	val |= FW_DRBL_DEV_WR_READY;
	val |= FW_DRBL_SCAP_DISCONNECTED;
	val |= FW_DRBL_SCAP_VOLTAGE_LOW;
	val |= FW_DRBL_SSD_DISCONNECTED;
	val |= FW_DRBL_SSD_FAILURE;
	val |= FW_DRBL_SSD_EOL;
	val |= FW_DRBL_SCAP_CALIB_DONE;
	val |= FW_DRBL_DATA_BACKUP_DONE;
	val |= FW_DRBL_HANDSHAKE_READY;
	val |= FW_DRBL_SCAP_DEGRADED;
	val |= FW_DRBL_SCAP_GOOD;
	val |= FW_DRBL_MAX_TEMP_ALERT;

	nvhba->nv_drbl_mask = val;
	iowrite32(val, nvhba->nv_regs + CPU_2_PCIE_A_DRBL_MASK);

	nv_unmask_intr(nvhba, INTR_CPU_2_PCIEA_DRBL);
}

void
ddr_irq_handler(nvram_hba_t *nvhba)
{
	u32_t cause;
	u32_t val;

	cause = ioread32(nvhba->nv_regs + DDR_INTR_CAUSE);
	mv_printk("############# ECC Error in device: %u ###############\n",
			nvhba->nv_minor);

	val = ioread32(nvhba->nv_regs + DDR_ERR_ADDR);
	mv_printk("Error Type\t\t\t: %s\n", (val & 1) ? "Double" : "Single");
	mv_printk("Error Chip-select\t\t: %01x\n", (val & 6) >> 1);
	mv_printk("Error Addr\t\t\t: %08x\n", val & ~7);

	val = ioread32(nvhba->nv_regs + DDR_ERR_DATA_HIGH);
	mv_printk("Data High\t\t\t: %08x\n", val);

	val = ioread32(nvhba->nv_regs + DDR_ERR_DATA_LOW);
	mv_printk("Data Low\t\t\t: %08x\n", val);

	val = ioread32(nvhba->nv_regs + DDR_RCVD_ECC);
	mv_printk("Received ECC\t\t: %02x\n", val);

	val = ioread32(nvhba->nv_regs + DDR_CALC_ECC);
	mv_printk("Calculated ECC\t\t: %02x\n", val);

	val = ioread32(nvhba->nv_regs + DDR_ECC_SBIT_CNTR);
	mv_printk("Single Bit ECC Count\t: %u\n", val);

	val = ioread32(nvhba->nv_regs + DDR_ECC_DBIT_CNTR);
	mv_printk("Double Bit ECC Count\t: %u\n", val);

	mv_printk("#####################################################\n");

	iowrite32(0, nvhba->nv_regs + DDR_INTR_CAUSE);
}


static void
nvram_timer(unsigned long arg)
{
	nvram_hba_t	*nvhba = (nvram_hba_t *)arg;
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;

	if (dma_ctrl->nv_queued_id != dma_ctrl->nv_cmpltd_id) {
		if (dma_ctrl->nv_cmpltd_id == nvhba->nv_prev_cmpltd_id) {
			/* hang situation */
			nvhba->nv_stats.st_io_rescues++;
			tasklet_schedule(&nvhba->nv_irq_tasklet);
		}
	}

	nvhba->nv_prev_cmpltd_id = dma_ctrl->nv_cmpltd_id;
	if (!nvhba->nv_exit_flag) {
		nvhba->nv_timer.expires += nvhba->nv_timer_delay;
		add_timer(&nvhba->nv_timer);
	}
}


static void
nvram_init_timer(nvram_hba_t *nvhba)
{
	init_timer(&nvhba->nv_timer);

	nvhba->nv_timer_delay = msecs_to_jiffies(io_rescue_timer);
	nvhba->nv_timer.expires = jiffies + nvhba->nv_timer_delay;
	nvhba->nv_timer.function = nvram_timer;
	nvhba->nv_timer.data = (unsigned long)nvhba;

	add_timer(&nvhba->nv_timer);
}


/*
 * nvram hba irq tasklet
 *
 * @arg - nvhba handle
 */
static void
nvram_irq_tasklet(unsigned long arg)
{
	nvram_hba_t	*nvhba = (nvram_hba_t *)arg;

	doorbell_handler(nvhba);
	nv_unmask_intr(nvhba, INTR_CPU_2_PCIEA_DRBL);

	dma_process_requests(nvhba);

	nvhba->nv_stats.st_tsklt_runs++;
}


/*
 * Function to queue a DMA request
 *
 * @nvhba	- hba handle
 * @req		- initialized dma request
 * @io_id	- reference to return IO id (if not null)
 */
static int
nvram_queue_cmd(nvram_hba_t *nvhba, dma_request_t *req, u64_t *io_id)
{
	nvram_dma_t	*dma_ctrl = &nvhba->nv_dma_ctrl;
	int		req_count = 0;

	if (dma_ctrl->nv_state != DMA_INITIALIZED)
		return (dma_ctrl->nv_errcode);

	spin_lock_bh(&dma_ctrl->nv_lock);

	while (req) {
		req_count++;
		dma_ctrl->nv_queued_id++;
		req->dma_id = dma_ctrl->nv_queued_id;
		dlist_add_back(&dma_ctrl->nv_request_list, &req->dma_link);

		req = req->dma_next;
	}

	if (io_id)
		*io_id = dma_ctrl->nv_queued_id;

	dma_process_pending_requests(nvhba);

	spin_unlock_bh(&dma_ctrl->nv_lock);

	return 0;
}


/*
 * IRQ handler registered with the OS
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
static irqreturn_t
loki_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t
loki_irq_handler(int irq, void *dev_id)
#endif
{
	nvram_hba_t *nvhba = (nvram_hba_t *)dev_id;

	nv_mask_intr(nvhba, INTR_CPU_2_PCIEA_DRBL);
	tasklet_schedule(&nvhba->nv_irq_tasklet);

	nvhba->nv_stats.st_intrs++;

	return IRQ_HANDLED;
}

/*
 * Register IRQ handler for the HBA
 *
 * @nvhba - hba handle
 * @return 0 if successful, failed otherwise
 */
static int
mvloki_request_irq(nvram_hba_t *nvhba)
{
	int ret;
	unsigned int irq;
	unsigned int irq_flag;

	nvhba->nv_irq_enabled = 0;
	nvhba->nv_msi_enabled = 0;

#if defined(LOKI_USE_MSI) && defined(CONFIG_PCI_MSI)
	ret = pci_enable_msi(nvhba->nv_pcidev);
	if (ret != 0) {
		mv_printk("Failed to enable MSI... Continuing with INTx\n");
	} else {
		nvhba->nv_msi_enabled = 1;
		irq = nvhba->nv_pcidev->irq;
		mv_printk("Using MSI interrupt mode, irq assigned: %u\n", irq);
	}
#endif

	irq = nvhba->nv_pcidev->irq;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	irq_flag = SA_SHIRQ;
#else
	irq_flag = IRQF_SHARED;
#endif

	ret = request_irq(irq, loki_irq_handler, irq_flag,
			nvhba->nv_name, nvhba);
	if (ret != 0) {
		mv_printk("IRQ registration failed. Err: %d\n", ret);
		return ret;
	}

	nvhba->nv_irq_enabled = 1;

	nv_enable_irq(nvhba);

	return 0;
}


/*
 * Create a character device to access the nvram memory region
 *
 * @nvhba	- hba handle
 * @major	- driver major number
 * @minor	- minor number
 * @return 0 if successful, -1 if device creation fails
 */
int
cdev_create(nvram_hba_t *nvhba, int major, int minor)
{
	int ret;
	int dev = MKDEV(major, minor);

	cdev_init(&nvhba->nv_cdev, &nv_cdev_fops);
	nvhba->nv_cdev.owner = THIS_MODULE;
	nvhba->nv_cdev.ops = &nv_cdev_fops;

	device_create(nvram_class, NULL, dev, NULL, "mvwam%d", minor);

	ret = cdev_add(&nvhba->nv_cdev, dev, 1);
	if (ret != 0) {
		mv_printk("char dev create failed. err: %d\n", ret);
		return -1;
	}

	return 0;
}


/*
 * Destroy the character device
 * @nvhba - hba handle
 */
void
cdev_destroy(nvram_hba_t *nvhba)
{
	device_destroy(nvram_class, MKDEV(nvhba->nv_major, nvhba->nv_minor));
	cdev_del(&nvhba->nv_cdev);
}


/*
 * Get the HBA handle from the inode minor number
 * @ninode	- device file inode
 * @return	- hba handle
 */
static nvram_hba_t *
get_nvhba(struct inode *ninode)
{
	int minor;

	minor = iminor(ninode);
	if (minor > nvram_hba_cnt) {
		return NULL;
	}

	return nvram_hba[minor];
}


/*
 * Character device open system call
 */
static int
cdev_open(struct inode *inode, struct file *filp)
{
	nvram_hba_t *nvhba;

	nvhba = get_nvhba(inode);
	if (nvhba == NULL)
		return -EINVAL;

	filp->private_data = nvhba;

	return 0;
}

/*
 * Character device close system call
 */
static int
cdev_close(struct inode *inode, struct file *filp)
{
	return 0;
}


/*
 * Character device ioctl system call
 */
static long cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int		ret = 0;
	u32_t		tmp;
	reg_io_t	reg_cfg;
	nvram_hba_t	*nvhba;

	/* check the magic */
	if (_IOC_TYPE(cmd) != NVRAM_IOC_MAGIC) {
		return -ENOTTY;
	}

	/* get the hba handle from the inode */
	nvhba = get_nvhba(file_inode(filp));
	if (nvhba == NULL) {
		return -EINVAL;
	}

	switch (cmd) {
	case NVRAM_IOC_DUMP_REG:
		dump_registers(nvhba);
		ret = 0;
		break;

	case NVRAM_IOC_GET_NR_DEVICES:
		ret = copy_to_user((void __user *)arg, &nvram_hba_cnt,
				sizeof(u32_t));
		break;

	case NVRAM_IOC_REG_IO:
		/* read/write register */
		if (copy_from_user(&reg_cfg, (void __user *)arg,
					sizeof(reg_io_t))) {
			return -EACCES;
		}

		if (reg_cfg.reg_addr > ((1 << 20) -1)) {
			return -EINVAL;
		}

		reg_io(nvhba, &reg_cfg);
		if (reg_cfg.rw == 0) {
			if (copy_to_user((void __user *)arg, &reg_cfg,
						sizeof(reg_io_t))) {
				return -EACCES;
			}
		}
		break;

	case NVRAM_IOC_GET_HBA_COUNT:
		/* return the nr of hbas found */
		if (copy_to_user((void __user *)arg, &nvram_hba_cnt,
					sizeof(u32_t))) {
			return -EACCES;
		}
		break;

	case NVRAM_IOC_REMAP_FLASH:
		/* program the pci-e window to access boot flash */
		ret = setup_flash_access(nvhba, (u32_t)arg);
		break;

	case NVRAM_IOC_START_BACKUP:
		/* flush contents of ddr to ssd */
		ret = nvram_backup(nvhba);
		break;

	case NVRAM_IOC_START_RESTORE:
		/* manual restore from SSD to DDR */
		ret = nvram_restore(nvhba);
		break;

	case NVRAM_IOC_FORMAT_SSD:
		/* format ssd */
		ret = nvram_ssd_format(nvhba);
		break;

	case NVRAM_IOC_SSD_TRIM:
		/* SSD Trim */
		ret = nvram_ssd_trim(nvhba);
		break;

	case NVRAM_IOC_EN_FORCED_ECC:
		/* enable forced ECC */
		ret = nvram_enable_forced_ecc(nvhba, (u8_t)arg);
		break;

	case NVRAM_IOC_DIS_FORCED_ECC:
		/* disable forced ECC */
		ret = nvram_disable_forced_ecc(nvhba);
		break;

	case NVRAM_IOC_READ_SCAP_REG:
		/* read supercap reg */
		ret = nvram_read_scap_reg(nvhba, arg);
		break;

	case NVRAM_IOC_AIO:
		/* Async IO */
		ret = cdev_aio_handler(filp, arg);
		break;

	case NVRAM_IOC_AIO_CMPL_ID:
		/* Async IO */
		ret = aio_get_cmpl_id(nvhba, arg);
		break;

	case NVRAM_IOC_AIO_WAIT_ID:
		/* Async IO */
		ret = aio_wait_id(nvhba, arg);
		break;

	case NVRAM_IOC_GET_SCAP_VOL:
		/* read supercap voltage */
		ret = nvram_get_scap_vol(nvhba, arg);
		break;

	case NVRAM_IOC_DMA_DUMP:
		/* dump DMA state */
		ret = nvram_dump_state(nvhba, arg);
		break;

	case NVRAM_IOC_PING_FW:
		/* Ping Firmware */
		ret = nvram_ping_fw(nvhba);
		break;

	case NVRAM_IOC_FW_VERSION:
		/* get firmware version */
		ret = nvram_get_fw_ver(nvhba, arg);
		break;

	case NVRAM_IOC_GET_BACKUP_STATS:
		/* get backup stats record */
		ret = nvram_get_backup_stats(nvhba, arg);
		break;

	case NVRAM_IOC_CLEAR_BACKUP_STATS:
		/* clear backup stats */
		ret = nvram_clear_backup_stats(nvhba);
		break;

	case NVRAM_IOC_SCAP_DISCHARGE_ON:
		/* enable supercap discharge */
		ret = nvram_scap_discharge_on(nvhba);
		break;

	case NVRAM_IOC_SCAP_DISCHARGE_OFF:
		/* stop supercap discharge */
		ret = nvram_scap_discharge_off(nvhba);
		break;

	case NVRAM_IOC_WRITE_READY:
		/* is it safe to write to DRAM */
		ret = nvram_is_write_ready(nvhba, arg);
		break;

	case NVRAM_IOC_IS_DATA_VALID:
		/* was backup / restore successful ? */
		ret = nvram_is_data_valid(nvhba, arg);
		break;

	case NVRAM_IOC_GET_EVNTLOG_PTRS:
		/* get event log info */
		ret = nvram_get_event_log_info(nvhba, arg);
		break;

	case NVRAM_IOC_SET_EVNTLOG_RDPTR:
		/* move the read pointer for reading events */
		ret = nvram_set_event_log_rdptr(nvhba, (u32_t)arg);
		break;

	case NVRAM_IOC_GET_ERR_BLOCKS:
		/* get error blocks list */
		ret = nvram_get_err_blocks(nvhba, arg);
		break;

	case NVRAM_IOC_SSD_IDENTIFY:
		/* read SSD identify page */
		ret = nvram_get_ssd_identify(nvhba, arg);
		break;

	case NVRAM_IOC_SSD_FW_UPDATE:
		/* update SSD firmware */
		ret = nvram_ssd_fw_update(nvhba, arg);
		break;

	case NVRAM_IOC_READ_EEPROM:
		/* read device EEPROM contents */
		ret = nvram_eeprom_read(nvhba, arg);
		break;

	case NVRAM_IOC_UPDATE_EEPROM:
		/* update device eeprom */
		ret = nvram_eeprom_update(nvhba, arg);
		break;

	case NVRAM_IOC_TRIGGER_COREDUMP:
		/* simulate firmware crash */
		ret = nvram_trigger_coredump(nvhba);
		break;

	case NVRAM_IOC_CLEAN_COREDUMP:
		/* clean firmware crash */
		ret = nvram_clean_coredump(nvhba);
		break;

	case NVRAM_IOC_GET_COREDUMP:
		/* get firmware crash */
		ret = nvram_get_fw_crashdump(nvhba, arg);
		break;

	case NVRAM_IOC_GET_INFO:
		/* get device info */
		ret = nvram_get_info(nvhba, arg);
		break;

	case NVRAM_IOC_SCAP_CALIB_START:
		/* start supercap calibration */
		ret = nvram_scap_calib_start(nvhba, arg);
		break;

	case NVRAM_IOC_RESET_FW:
		/* reset firmware */
		ret = nvram_reset_fw(nvhba);
		break;

	case NVRAM_IOC_HARDRESET:
		/* hard reset */
		ret = nvram_hardreset(nvhba);
		break;

	case NVRAM_IOC_GET_DBG_INFO:
		/* get device dbg info */
		ret = nvram_get_dbg_info(nvhba, arg);
		break;

	case NVRAM_IOC_WAM_IOTEST:
		/* kernel io api test */
		ret = wam_io_test(filp, arg);
		break;

	case NVRAM_IOC_CHARGER_READ:
		ret = nvram_scap_chrgr_read(nvhba, arg);
		break;

	case NVRAM_IOC_CHARGER_SET:
		ret = nvram_scap_chrgr_set(nvhba, arg);
		break;

	case NVRAM_IOC_CHARGER_SET_VOL:
		ret = nvram_scap_chrgr_set_vol(nvhba, arg);
		break;

	case NVRAM_IOC_DDR_SEL_VOL:
		ret = nvram_ddr_sel_vol(nvhba, arg);
		break;

	case NVRAM_IOC_WAIT_ON_ALERTS:
		ret = nv_wait_on_alerts(arg);
		break;

	case NVRAM_IOC_GET_DEV_ALERT:
		tmp = nv_get_dev_alerts(nvhba);
		ret = copy_to_user((void __user *)arg, &tmp, sizeof(u32_t));
		break;

	case NVRAM_IOC_CLEAR_DEV_ALERT:
		nv_clear_dev_alert(nvhba, (u32_t)arg);
		ret = 0;
		break;

	case NVRAM_IOC_INJ_ERR:
		ret = nv_error_inj(nvhba, (u32_t)arg);
		break;

	case NVRAM_IOC_DRV_VERSION:
		ret = nv_get_drv_version(arg);
		break;

	case NVRAM_IOC_SET_ATTR:
		ret = nvram_set_attr_ioc(nvhba, arg);
		break;

	case NVRAM_IOC_GET_ATTR:
		ret = nvram_get_attr_ioc(nvhba, arg);
		break;

	case NVRAM_IOC_READ_TLOG:
		ret = nvram_read_thermal_log(nvhba, arg);
		break;

	case NVRAM_IOC_CLEAR_TLOG:
		ret = nvram_clear_thermal_log(nvhba, arg);
		break;

	default:
		ret = -ENOTTY;
	}

	return ret;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 11)
static int
cdev_old_ioctl(struct inode *ninode, struct file *nfile,
		unsigned int cmd, unsigned long arg)
{
	return cdev_ioctl(nfile, cmd, arg);
}
#endif

static void
nv_page_cleanup(cdev_request_t *creq, int rw)
{
	u32_t i;
	struct page **pages;

	pages = creq->cr_page_list;

	for (i = 0; i < creq->cr_nr_pages; i++) {
		if (rw == DMA_READ)
			SetPageDirty(pages[i]);

		put_page(pages[i]);
	}

	if (creq->cr_nr_pages > 1)
		kfree(pages);
}

#define	CDEV_SIGNATURE 0x5050505020100812UL

/*
 * Callback function to cleanup the allocated resources. This
 * function is used when the user program is signalled and no
 * longer waiting.
 */
static void
cdev_free_request(dma_request_t *req)
{
	cdev_request_t *creq;

	creq = container_of(req, cdev_request_t, cr_dma_req);

	if (creq->cr_sign != CDEV_SIGNATURE) {
		printk("Signature not correct!\n");
		BUG();
	}

	nv_page_cleanup(creq, DMA_WRITE);
	kmem_cache_free(nv_cdev_req_cache, creq);
}

/*
 * DMA completion callback function
 */
static void
cdev_io_done(dma_request_t *req)
{
	cdev_request_t *creq;

	creq = container_of(req, cdev_request_t, cr_dma_req);

	/* signal the sleeping thread */
	mv_waitq_signal(&creq->dma_waitq);
	return;
}

/*
 * Block dev entry operations
 */

static int
nv_bdev_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	long	cylinder_sz;
	nvram_hba_t *nvhba;

	if (bdev == NULL || geo == NULL)
		return -EINVAL;

	/* do not return geometry info for partitions */
	if (bdev->bd_part != NULL) {
		return -1;
	}

	nvhba = bdev->bd_disk->private_data;
	if (nvhba == NULL)
		return -ENODEV;

	geo->heads = 1 << 6;
	geo->sectors = 1 << 5;
	cylinder_sz = 1 << (6 + 5);
	geo->cylinders = ((get_capacity(bdev->bd_disk) + cylinder_sz - 1)
				>> (6 + 5));
	geo->start = 0;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
static int nv_bdev_ioctl(struct block_device *bdev, fmode_t mode,
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct hd_geometry geo;

	switch (cmd) {
	case HDIO_GETGEO:
		ret = nv_bdev_getgeo(bdev, &geo);
		if (ret == 0) {
			if (copy_to_user((void __user*)arg,
						&geo, sizeof(geo)))
				ret = -EFAULT;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;

}
#else
static int
nv_bdev_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct hd_geometry geo;
	struct block_device *bdev = inode->i_bdev;

	switch (cmd) {
	case HDIO_GETGEO:
		ret = nv_bdev_getgeo(bdev, &geo);
		if (ret == 0) {
			if (copy_to_user((void __user*)arg,
					&geo, sizeof(geo)))
				ret = -EFAULT;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
#endif

static int
nv_bdev_create(nvram_hba_t *nvhba)
{
	struct gendisk	*gd;
	char disk_name[16];

	/*
	 * Create block device
	 */
	gd = alloc_disk(NV_BDEV_NR_MINORS);
	if (gd == NULL) {
		mv_printk("Failed to create block device\n");
		return -1;
	}

	gd->major	= nvram_major_b;
	gd->minors	= NV_BDEV_NR_MINORS;
	gd->first_minor	= nvhba->nv_minor * NV_BDEV_NR_MINORS;

	set_capacity(gd, (nvhba->nv_ddr_size - MV_DDR_OFFSET) / NV_BDEV_SECTOR_SZ);

	sprintf(disk_name, "%s%d", MV_BLK_DRIVER_NAME, nvhba->nv_minor);
	strcpy(gd->disk_name, disk_name);

	gd->fops = &nvram_block_ops;
	gd->private_data = nvhba;

	nvhba->gdisk = gd;

	/*
	 * Initialize block device queue
	 */
	gd->queue = blk_alloc_queue(GFP_KERNEL);
	if (gd->queue == NULL) {
		mv_printk("Failed to alloc block dev queue\n");
		put_disk(gd);
		return -1;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
	blk_queue_make_request(gd->queue, nvram_make_request);
#else
	blk_queue_make_request(gd->queue, __nvram_make_request);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	blk_queue_hardsect_size(gd->queue, NV_BDEV_SECTOR_SZ);
#endif

	gd->queue->queuedata = nvhba;

	add_disk(gd);

	return 0;
}


static int
nv_bdev_destroy(nvram_hba_t *nvhba)
{
	struct gendisk *gd;

	gd = nvhba->gdisk;

	if (gd != NULL) {
		if (gd->queue) {
#if	LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 25)
			blk_put_queue(gd->queue);
#else
			blk_cleanup_queue(gd->queue);
#endif

		}
		del_gendisk(gd);
		put_disk(gd);
	}
	return 0;
}


/*
 * Initialize the PRDs pointing to nvram memory region
 *
 * @nvhba	- hba handle
 * @req		- dma request
 * @offset	- nvram memory offset
 * @rw		- DMA_READ or DMA_WRITE
 * @return 0 if successful,
 * -ENOMEM	- not enough PRDs to accommodate.
 */
static int
cmd_init_ddr_prd(nvram_hba_t *nvhba, dma_request_t *req,
		u64_t offset, int rw)
{
	prd_t	*prd;
	u32_t	nr_prds;
	u32_t	length = req->dma_byte_count;
	u32_t	start_chip_slct, end_chip_slct;
	u32_t	byte_cnt;
	u32_t	nr_chip_slct;
	u32_t	cs_offset;
	u32_t	i;
	u32_t	size_per_cs;

	offset += MV_DDR_OFFSET;

	if (rw == DMA_READ) {
		prd = req->dma_prd_tbl.c_src_prds;
		nr_prds = MV_SRC_PRDS_PER_CMD;
	} else {
		prd = req->dma_prd_tbl.c_dst_prds;
		nr_prds = MV_DST_PRDS_PER_CMD;
	}

	size_per_cs = (1UL << nvhba->nv_ddr_size_shft);
	start_chip_slct	= offset >> nvhba->nv_ddr_size_shft;
	end_chip_slct	= (offset + length + size_per_cs - 1) >>
				nvhba->nv_ddr_size_shft;

	nr_chip_slct = end_chip_slct - start_chip_slct;

	mv_printk_dbg("Number of chip selects to use: %u\n", nr_chip_slct);

	if (nr_chip_slct > nr_prds) {
		mv_printk("not enough space. required: %u "
				"space available for %u\n",
				nr_chip_slct, nr_prds);
		return -ENOMEM;
	}

	cs_offset = offset & (size_per_cs - 1);

	start_chip_slct += IFC_DDR_CS0;
        i = 0;
        while (length) {
                prd[i].prd_addr_high	= 0;
                prd[i].prd_addr_low	= cs_offset;
		prd[i].prd_xbar_ifc	= start_chip_slct;

                byte_cnt = MV_MIN(MV_MAX_PRD_BUF_SIZE, length);
                if (byte_cnt > (size_per_cs - cs_offset)) {
                        byte_cnt = size_per_cs - cs_offset;
                        start_chip_slct++;
                        cs_offset = 0;
                } else {
                        cs_offset += byte_cnt;
                }

                prd[i].prd_byte_cnt = byte_cnt;
                length -= byte_cnt;
                i++;

                if (i > nr_prds) {
                        mv_printk("not enough space for ddr prd!, avail: %u, "
                                        "bytes left: %u\n", nr_prds, length);
                        return -ENOMEM;
                }

                if (start_chip_slct > IFC_DDR_CS3) {
                        mv_printk("DDR Address out of range %012llx!\n",
                                        offset);
                        BUG();
                }
        }

        /* set the byte count of remaining prds to 0 */
        while (i < nr_prds) {
                prd[i].prd_byte_cnt = 0;
                i++;
        }

        return 0;
}

static inline u32_t
nv_calc_nr_pages(unsigned long base_addr, size_t len)
{
	unsigned long start, end;

	start = (base_addr >> PAGE_SHIFT);
	end = ((base_addr + len + PAGE_SIZE - 1) >> PAGE_SHIFT);

	return (end - start);
}

static u32_t
nv_get_page_count(struct iovec *iov, u32_t iov_cnt)
{
	unsigned int i;
	unsigned int page_cnt = 0;

	for (i = 0; i < iov_cnt; i++) {
		if (iov[i].iov_len == 0) {
			return 0;
		}

		page_cnt += nv_calc_nr_pages((unsigned long)iov[i].iov_base,
						iov[i].iov_len);
	}

	return page_cnt;
}

static int
cdev_alloc_req(cdev_request_t  **creq, struct iovec *iov, u32_t iov_cnt)
{
	u32_t	nr_pages;
	cdev_request_t *req;

	nr_pages = nv_get_page_count(iov, iov_cnt);
	if (nr_pages == 0)
		return -EINVAL;

	req = kmem_cache_alloc(nv_cdev_req_cache, GFP_KERNEL);
	if (req == NULL)
		return -ENOMEM;

	memset(req, 0, sizeof(cdev_request_t));

	if (nr_pages > 1) {
		req->cr_page_list = kmalloc(nr_pages *
				sizeof(struct page *), GFP_KERNEL);
		if (req->cr_page_list == NULL) {
			kmem_cache_free(nv_cdev_req_cache, req);
			*creq = NULL;
			return -ENOMEM;
		}
	} else {
		req->cr_page_list = &(req->cr_single_page);
	}

	req->cr_sign = CDEV_SIGNATURE;
	req->cr_nr_pages = nr_pages;
	*creq = req;

	return 0;
}

int
nv_pin_user_pages(struct page **page_list, struct iovec *iov, int rw,
			u32_t *nr_pages)
{
	int ret = 0;

	*nr_pages = nv_calc_nr_pages((unsigned long)iov->iov_base,
					iov->iov_len);

	/* pin down the user pages */
	ret = get_user_pages((unsigned long)iov->iov_base,
			     *nr_pages, rw == DMA_READ, page_list, NULL);

	if (ret < *nr_pages)
		return -1;

	return 0;
}

enum {
	NV_HOST_PRD,
	NV_DEVICE_PRD
};

static void
nv_get_req_prd(dma_request_t *req, int rw, int type,
		prd_t **prd, u32_t *nr_prds)
{
	if (rw == DMA_READ) {
		if (type == NV_HOST_PRD) {
			*prd = req->dma_prd_tbl.c_dst_prds;
			*nr_prds = MV_DST_PRDS_PER_CMD;
		} else {
			*prd = req->dma_prd_tbl.c_src_prds;
			*nr_prds = MV_SRC_PRDS_PER_CMD;
		}
	} else {
		if (type == NV_HOST_PRD) {
			*prd = req->dma_prd_tbl.c_src_prds;
			*nr_prds = MV_SRC_PRDS_PER_CMD;
		} else {
			*prd = req->dma_prd_tbl.c_dst_prds;
			*nr_prds = MV_DST_PRDS_PER_CMD;
		}
	}
}

typedef struct prd_merge_cntxt {
	int	prd_idx;
	u32_t	nr_prds;
	u32_t	prev_len;
	u32_t	byte_count;
	unsigned long prev_paddr;
	prd_t	*prd;
} prd_merge_cntxt_t;

/*
 * Initialize the PRD entry with a physical memory region info
 *
 * @cntxt - the prd merge context
 * @paddr - 64 bit physical address
 * @mem_size - size of the buffer
 */
static int
host_prd_try_merge(prd_merge_cntxt_t* cntxt, unsigned long paddr, u32_t size)
{
	unsigned long prev_paddr = 0;
	prd_t *prd = cntxt->prd;
	u32_t len = 0;
	u32_t bytes_left = size;
	u32_t i;
	u64_t tmp;

	i = cntxt->prd_idx;
	if ((cntxt->prev_paddr + cntxt->prev_len) == paddr) {
		bytes_left = cntxt->prev_len + size;
		paddr = cntxt->prev_paddr;
	} else {
		i++;
	}

	while (bytes_left) {
		if (i >= cntxt->nr_prds)
			return -1;

		if (bytes_left > MV_MAX_PRD_BUF_SIZE) {
			len = MV_MAX_PRD_BUF_SIZE;
			if ((i + 1) >= cntxt->nr_prds)
				return -1;
		} else {
			len = bytes_left;
		}

		tmp = 1;
		tmp <<= 32;
		tmp -= LSB_32(paddr);

		if ((u64_t)len > tmp) {
			/* break the buffer if it crosses 4GB boundary */
			len = tmp;
		}

		prd[i].prd_byte_cnt	= len;
		prd[i].prd_addr_low	= LSB_32(paddr);
		prd[i].prd_addr_high	= MSB_32(paddr);
		prd[i].prd_xbar_ifc	= IFC_HOST_MEM;

		prev_paddr = paddr;
		paddr += len;
		bytes_left -= len;

		i++;
	}

	cntxt->prd_idx = i - 1;
	cntxt->prev_paddr = prev_paddr;
	cntxt->prev_len = len;
	cntxt->byte_count += size;

	return 0;
}


static void
nv_free_chained_req(dma_request_t *req)
{
	dma_request_t *next;
	chained_request_t *ch_req = req->dma_private;
	int i = 0;

	while (req) {
		next = req->dma_next;
		kmem_cache_free(nv_dma_req_cache, req);
		req = next;
		i++;
	}

	kmem_cache_free(nv_chained_req_cache, ch_req);
}

static void
nv_split_io_cb(dma_request_t *req)
{
	chained_request_t *ch_req;
	dma_done_t dma_done;

	ch_req = (chained_request_t *)req->dma_private;
	ch_req->ch_nr_done++;


	if (ch_req->ch_status == 0 && req->dma_status != 0)
		ch_req->ch_status = req->dma_status;

	if (ch_req->ch_nr_reqs != ch_req->ch_nr_done)
		return;

	/* all IOs in the chain have completed */
	req = ch_req->ch_dma_req;
	req->dma_private = ch_req->ch_private;
	req->dma_done = ch_req->ch_done;
	req->dma_status = ch_req->ch_status;

	dma_done = req->dma_done;

	/* removing the first request out. will be freed by done cb */
	ch_req->ch_dma_req = req->dma_next;
	req->dma_next = NULL;

	nv_free_chained_req(ch_req->ch_dma_req);
	dma_done(req);
}

static chained_request_t *
nv_alloc_chained_req(dma_request_t *dma_req)
{
	chained_request_t *ch_req;

	ch_req = kmem_cache_alloc(nv_chained_req_cache, GFP_KERNEL);
	if (ch_req == NULL)
		return NULL;

	memset(ch_req, 0, sizeof(chained_request_t));
	ch_req->ch_done = dma_req->dma_done;
	ch_req->ch_private = dma_req->dma_private;
	ch_req->ch_dma_req = dma_req;
	ch_req->ch_nr_reqs = 1;

	dma_req->dma_done = nv_split_io_cb;
	dma_req->dma_private = ch_req;

	return ch_req;
}

static int
nv_split_io(nvram_hba_t *nvhba, dma_request_t **req, int rw,
	       u64_t *offset, prd_merge_cntxt_t *cntxt)
{
	chained_request_t	*ch_req;
	dma_request_t		*dma_req;
	int			ret;

	dma_req = *req;
	if (dma_req->dma_done == nv_split_io_cb) {
		ch_req = dma_req->dma_private;
	} else {
		ch_req = nv_alloc_chained_req(dma_req);
		if (ch_req == NULL)
			return -ENOMEM;
	}

	dma_req->dma_byte_count = cntxt->byte_count;
	ret = cmd_init_ddr_prd(nvhba, dma_req, *offset, rw);
	if (ret != 0)
		return -EINVAL;

	(*offset) += cntxt->byte_count;

	dma_req->dma_next = kmem_cache_alloc(nv_dma_req_cache, GFP_KERNEL);
	if (dma_req->dma_next == NULL)
		return -ENOMEM;

	dma_req = dma_req->dma_next;
	memset(dma_req, 0, sizeof(*dma_req));
	memset(cntxt, 0, sizeof(prd_merge_cntxt_t));

	nv_get_req_prd(dma_req, rw, NV_HOST_PRD,
			&cntxt->prd, &cntxt->nr_prds);

	cntxt->prd_idx = -1;
	dma_req->dma_status = 0xFF;
	dma_req->dma_done = nv_split_io_cb;
	dma_req->dma_private = ch_req;

	*req = dma_req;
	ch_req->ch_nr_reqs++;

	return 0;
}

static void
nv_update_req_cb(dma_request_t *req)
{
	if (req->dma_done == cdev_io_done) {
		req->dma_done = cdev_free_request;
	} else {
		chained_request_t *ch_req;
		ch_req = (chained_request_t *)req->dma_private;
		ch_req->ch_done = cdev_free_request;
	}
}

/*
 * Character device I/O handler
 *
 * This function creates the dma request, initializes the required fields,
 * initializes the PRD tables of host buffer and nvram memory region, and
 * queues the request to the dma thread.
 *
 * @filp	- file pointer
 * @iov		- iovec of user buffers
 * @iov_cnt	- nr. of iovs
 * @rw		- DMA_READ or DMA_WRITE
 * @f_pos	- nvram memory region offset (0 based offset)
 * @count	- byte count
 * @return	- nr of bytes transferred
 */
static ssize_t
cdev_io_handler(struct file *filp, struct iovec *iov, u32_t iov_cnt,
		int rw, loff_t *f_pos, ssize_t count, int sync,
		int barrier, u64_t *io_id)
{
	ssize_t		ret = 0;
	nvram_hba_t	*nvhba = (nvram_hba_t *)filp->private_data;
	cdev_request_t	*creq;
	dma_request_t	*dma_req;
	struct page     **page_list;
	u32_t		page_idx;
	u32_t		nr_pages;
	unsigned long	paddr;
	unsigned long	buf_addr;
	u32_t		buf_len;
	u32_t		valid_bytes;
	int		status;
	prd_merge_cntxt_t cntxt;
	u64_t		offset;
	int		i, j;
//	unsigned long	flags;
	int		nr_splits;

	if (nvhba->nv_pkg_ver_comp != NV_PKG_VER_COMPATIBLE) {
		mv_printk("Driver/FW version mismatched, I/O Disabled.\n");
		return -EINVAL;
	}

	offset = *f_pos;
	if (offset >= nvhba->nv_max_ddr_offset)
		return -EINVAL;

	if ((offset + count) > nvhba->nv_max_ddr_offset) {
		mv_printk_dbg("offset[%llx] + count[%lx] > Max Offset[%llx]\n",
				offset, count, nvhba->nv_max_ddr_offset);
		return -ESPIPE;
	}

	ret = cdev_alloc_req(&creq, iov, iov_cnt);
	if (ret != 0)
		return ret;

	mv_waitq_init(&creq->dma_waitq);

	dma_req = &creq->cr_dma_req;
	dma_req->dma_private	= nvhba;
	dma_req->dma_status	= 0xFF;
	dma_req->dma_barrier_req = barrier;

	if (sync)
		dma_req->dma_done = cdev_io_done;
	else
		dma_req->dma_done = cdev_free_request;

	page_list = creq->cr_page_list;

	memset(&cntxt, 0, sizeof(prd_merge_cntxt_t));
	nv_get_req_prd(dma_req, rw, NV_HOST_PRD,
			&cntxt.prd, &cntxt.nr_prds);

	cntxt.prd_idx = -1;

	page_idx = 0;
	nr_splits = 0;
	for (i = 0; i < iov_cnt; i++) {
		nr_pages = 0;
		ret = nv_pin_user_pages(&page_list[page_idx], &iov[i], rw,
				&nr_pages);
		if (ret < 0) {
			page_idx += nr_pages;
			goto pin_err;
		}

		buf_addr = (unsigned long)iov[i].iov_base;
		buf_len = iov[i].iov_len;

		paddr = page_to_pfn(page_list[page_idx]);
		paddr <<= PAGE_SHIFT;
		paddr += (buf_addr & ~PAGE_MASK);

		if (nr_pages == 1) {
			valid_bytes = buf_len;
		} else {
			valid_bytes = PAGE_SIZE - (buf_addr & ~PAGE_MASK);
		}

		j = 0;
		do {
			buf_len -= valid_bytes;
			ret = host_prd_try_merge(&cntxt, paddr, valid_bytes);
			if (ret != 0) {
				/* max prd limit reached. need to split io */
				ret = nv_split_io(nvhba, &dma_req, rw,
						&offset, &cntxt);
				if (ret != 0) {
					page_idx += nr_pages;
					goto pin_err;
				}
				nr_splits++;
				host_prd_try_merge(&cntxt, paddr, valid_bytes);
			}
			j++;
			if (j >= nr_pages)
				break;

			paddr = page_to_pfn(page_list[page_idx + j]);
			paddr <<= PAGE_SHIFT;
			valid_bytes = MV_MIN(buf_len, PAGE_SIZE);
		} while (j < nr_pages);

		page_idx += nr_pages;
	}

	cntxt.prd_idx++;

	if ((cntxt.nr_prds - cntxt.prd_idx) != 0)
		memset(cntxt.prd + cntxt.prd_idx, 0,
			sizeof(prd_t) * (cntxt.nr_prds - cntxt.prd_idx));

	dma_req->dma_byte_count = cntxt.byte_count;
	ret = cmd_init_ddr_prd(nvhba, dma_req, offset, rw);
	if (ret != 0)
		goto pin_err;

	if ((ret = nvram_queue_cmd(nvhba, &creq->cr_dma_req, io_id)) != 0) {
		goto pin_err;
	}

	wam_iostats_record(&nvhba->nv_stats.st_cdev_stats, rw, count);

	if (sync == 0)
		return count;

	/* wait for completion */
	ret = mv_waitq_timedwait(&creq->dma_waitq, 15000);
	status = creq->cr_dma_req.dma_status;

	if (ret != 0) {
		/* received signal. swap the callback */
		nv_update_req_cb(&creq->cr_dma_req);

		udelay(10);

		if (ret == -ETIMEDOUT) {
			mv_printk("IO Timed out. dev: %u, "
					"status: %03x\n",
					nvhba->nv_minor,
					creq->cr_dma_req.dma_status);
			ret = -ETIME;
		} else {
			mv_printk("IO interrupted. dev: %u, "
					"status: %03x\n",
					nvhba->nv_minor,
					creq->cr_dma_req.dma_status);
			ret = -EINTR;
		}

		log_dma_state(nvhba);
		return ret;
	} else {
		nv_page_cleanup(creq, rw);
		kmem_cache_free(nv_cdev_req_cache, creq);
	}

	if (status != 0) {
		mv_printk("Dev: %u DMA Failed: %d\n",
			       nvhba->nv_minor, status);
		ret = 0;
	} else {
		*f_pos += count;
		ret = count;
	}

	return ret;

pin_err:
	creq->cr_nr_pages = page_idx;
	nv_page_cleanup(creq, rw);

	dma_req = creq->cr_dma_req.dma_next;
	if (dma_req != NULL) {
		nv_free_chained_req(dma_req);
	}
	kmem_cache_free(nv_cdev_req_cache, creq);

	return ret;
}

static ssize_t
cdev_read(struct file *filp, char __user *buf, size_t count,
			loff_t *f_pos)
{
	struct iovec iov;

	iov.iov_base = (void *)buf;
	iov.iov_len = count;

	return cdev_io_handler(filp, &iov, 1, DMA_READ, f_pos, count,
			NV_SYNC_IO, 0, NULL);
}

static ssize_t
cdev_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *f_pos)
{
	struct iovec iov;

	iov.iov_base = (void *)buf;
	iov.iov_len = count;

	return cdev_io_handler(filp, &iov, 1, DMA_WRITE, f_pos, count,
			NV_SYNC_IO, 0, NULL);
}

static void
cdev_vma_open(struct vm_area_struct *vma)
{
	return;
}

static void
cdev_vma_close(struct vm_area_struct *vma)
{
	return;
}

struct vm_operations_struct cdev_vm_ops = {
	.open	= cdev_vma_open,
	.close	= cdev_vma_close,
};

static int
cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int	ret;
	u32_t	size;
	u32_t	offset;
	u32_t	bar_offset;
	unsigned long	paddr;
	nvram_hba_t	*nvhba = (nvram_hba_t *)filp->private_data;

	size = vma->vm_end - vma->vm_start;
	offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset >= NV_MMAP_REG_OFFSET &&
			(offset + size) <= NV_MMAP_FLASH_OFFSET) {

		/* mmap request to map device register space */
		paddr = nvhba->nv_bar[0].bar_base + offset;

	} else if (offset >= NV_MMAP_FLASH_OFFSET &&
			(offset + size) <= NV_MMAP_SRAM_OFFSET) {

		/* mmap request to map boot flash */
		bar_offset = offset - NV_MMAP_FLASH_OFFSET;
		paddr = nvhba->nv_bar[NV_CTRL_ACCESS_BAR].bar_base +
				NV_FLASH_OFFSET + bar_offset;
	} else if (offset >= NV_MMAP_SRAM_OFFSET &&
			(offset + size) <= NV_MAX_MMAP_OFFSET) {
		/* mmap sram area */
		bar_offset = offset - NV_MMAP_SRAM_OFFSET;
		paddr = nvhba->nv_bar[NV_CTRL_ACCESS_BAR].bar_base +
				bar_offset;
	} else {
		/* not a well known offset */
		mv_printk("requested offset %08x + size %08x out of range "
				"%x\n", offset, size, NV_MAX_MMAP_OFFSET);
		return -EINVAL;
	}

	vma->vm_ops = &cdev_vm_ops;
	vma->vm_flags |= VM_IO;
	vma->vm_flags |= VM_RESERVED;
	vma->vm_private_data = filp->private_data;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 12)
	ret = io_remap_page_range(vma, vma->vm_start, paddr, size,
			vma->vm_page_prot);
#else
	ret = io_remap_pfn_range(vma, vma->vm_start, (paddr >> PAGE_SHIFT),
			size, vma->vm_page_prot);
#endif

	if (ret != 0)
		mv_printk("io_remap_page_range failed with err %d\n", ret);

	return ret;
}

static int
cdev_aio_handler(struct file *filp, unsigned long arg)
{
	int		ret;
	ssize_t		ret_size;
	nv_aiocb_t	aiocb;
	struct iovec	*iov;

	ret = copy_from_user(&aiocb, (void __user *)arg, sizeof(nv_aiocb_t));
	if (ret != 0)
		return -EACCES;

	iov = (struct iovec *) kmalloc(sizeof(struct iovec) * aiocb.io_vcnt,
			GFP_KERNEL);
	if (iov == NULL)
		return -ENOMEM;

	ret = copy_from_user(iov, aiocb.io_vec,
			sizeof(struct iovec) * aiocb.io_vcnt);
	if (ret != 0) {
		kfree(iov);
		return -EACCES;
	}

	ret_size = cdev_io_handler(filp, iov, aiocb.io_vcnt, aiocb.io_op,
			&aiocb.io_offset, aiocb.io_size, aiocb.io_sync,
			aiocb.io_type, &aiocb.io_id);

	if (ret_size == aiocb.io_size) {
		if(copy_to_user((void __user *)arg, &aiocb,
					sizeof(nv_aiocb_t))) {
			mv_printk("copy to user failed.. ID: %llu\n",
					aiocb.io_id);
		}
		ret = 0;
	} else {
		ret = -ENOMEM;
	}

	kfree(iov);
	return ret;
}


static int
aio_get_cmpl_id(nvram_hba_t *nvhba, unsigned long arg)
{
	int		ret = 0;
	u64_t		cmpltd_id;

	cmpltd_id = nvhba->nv_dma_ctrl.nv_cmpltd_id;

	if(copy_to_user((void __user *)arg, &cmpltd_id, sizeof(u64_t))) {
		ret = -EACCES;
	}

	return 0;
}

static int
aio_wait_id(nvram_hba_t *nvhba, unsigned long arg)
{
	int	ret;
	int	retry = 0;
	u64_t	id;
	long	timeout = msecs_to_jiffies(1000);

	if (copy_from_user(&id, (void __user *)arg, sizeof(u64_t))) {
		return -EACCES;
	}

	/* using broadcast method as the first step to implement wait on
	 * io completion. Need to fine tune this to per IO based wait queue */
	while (1) {
		ret = wait_event_interruptible_timeout(
				nvhba->nv_dma_ctrl.nv_broadcast_event,
				(id <= nvhba->nv_dma_ctrl.nv_cmpltd_id),
				timeout);

		ret = (!ret ? -ETIMEDOUT : ((ret == -ERESTARTSYS) ? -EINTR : 0));
		if((ret != -ETIMEDOUT) || (id <= nvhba->nv_dma_ctrl.nv_cmpltd_id)) {
			return ret;
		}

		mv_printk_dbg("compare id wake up[%d]: wait_id(%llu),"
				"compled_id(%llu), queued_id(%llu)\n",
				ret, id, nvhba->nv_dma_ctrl.nv_cmpltd_id,
				nvhba->nv_dma_ctrl.nv_queued_id);

		retry++;
		if(retry == 3) {
			break;
		}

	}

	return ret;
}

static inline mv_kmem_cache_t *
mv_kmem_cache_create(const char *name, size_t size, size_t align,
		unsigned long flags)
{
	mv_kmem_cache_t *memcache;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	memcache = kmem_cache_create(name, size, align, flags, NULL, NULL);
#else
	memcache = kmem_cache_create(name, size, align, flags, NULL);
#endif
	if (memcache == NULL) {
		mv_printk("kmem_cache creation failed for %s, size: %zu, "
				"align: %zu, flags: %08lx\n",
				name, size, align, flags);
	}

	return memcache;
}


static int
nv_kmem_cache_init(void)
{
	nv_cdev_req_cache = mv_kmem_cache_create("mvwam_cdev_req",
				sizeof(cdev_request_t), 32, 0);
	if (nv_cdev_req_cache == NULL)
		return -ENOMEM;

	nv_dma_req_cache = mv_kmem_cache_create("mvwam_dma_req",
				sizeof(dma_request_t), 32, 0);
	if (nv_dma_req_cache == NULL) {
		nv_kmem_cache_destroy();
		return -ENOMEM;
	}

	nv_chained_req_cache = mv_kmem_cache_create("mvwam_chained_req",
				sizeof(chained_request_t), 32, 0);
	if (nv_chained_req_cache == NULL) {
		nv_kmem_cache_destroy();
		return -ENOMEM;
	}

	return 0;
}

static void
nv_kmem_cache_destroy(void)
{
	if (nv_chained_req_cache)
		kmem_cache_destroy(nv_chained_req_cache);

	if (nv_dma_req_cache)
		kmem_cache_destroy(nv_dma_req_cache);

	if (nv_cdev_req_cache)
		kmem_cache_destroy(nv_cdev_req_cache);

	nv_chained_req_cache = NULL;
	nv_dma_req_cache = NULL;
	nv_cdev_req_cache = NULL;
}


//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
//#define nvram_endio(bio, size, status)	bio_endio(bio, status)
//#else
//#define nvram_endio(bio, size, status)	bio_endio(bio, size, status)
//#endif

#define nvram_endio(bio, size, status)	bio_endio(bio)

static void
nv_blk_req_done(dma_request_t *dma_req)
{
	struct bio *bio;

	bio = dma_req->dma_private;
	nvram_endio(bio, bio->bi_iter.bi_size, dma_req->dma_status);

	kmem_cache_free(nv_dma_req_cache, dma_req);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
static blk_qc_t __nvram_make_request(struct request_queue *q, struct bio *bio)
{
	int ret;

	ret = nvram_make_request(q, bio);
	if (ret != 0)
		nvram_endio(bio, bio->bi_iter.bi_size, ret);
	return ret;
}
#endif

/*
 * Entry of block I/O
 *
 * This function will construct prepare and submit block I/O to the queue
 *
 * @q		- request queue from bio layer
 * @bio		- block io request
 *
 * @return 0 any way, extra info will be returned in nvram_endio
 */
static int
nvram_make_request(struct request_queue *q, struct bio *bio)
{
	int	ret = 0;
//	int	segno;
	struct  bvec_iter iter;
	int	nr_splits = 0;
	int	rw;
	u32_t	valid_bytes;
	u32_t	count;
	u64_t	offset;
	unsigned long	paddr;
//	struct bio_vec	*bvec;
	struct bio_vec	bvec;
	dma_request_t	*dma_req;
	dma_request_t	*dma_req_head;
	nvram_hba_t	*nvhba;
	prd_merge_cntxt_t cntxt;

	nvhba = (nvram_hba_t *)bio->bi_bdev->bd_disk->private_data;

	if(bio->bi_vcnt <= 0) {
		mv_printk_dbg("bi_vcnt = %d, size: %u calling endio()\n",
				bio->bi_vcnt, bio->bi_iter.bi_size);
		nvram_endio(bio, bio->bi_iter.bi_size, 0);
		return 0;
	}

	offset = bio->bi_iter.bi_sector;
	offset *= NV_BDEV_SECTOR_SZ;

	count = bio_sectors(bio);
	count *= NV_BDEV_SECTOR_SZ;

	if ((offset >= nvhba->nv_max_ddr_offset) ||
			((offset + count) > nvhba->nv_max_ddr_offset)) {
		mv_printk_dbg("offset: %llx + count: %lx >= max_ddr_offset: "
				"%llx\n", offset, nvhba->nv_max_ddr_offset);
		return -ESPIPE;
	}

	if(count == 0) {
		mv_printk("IO Size = 0\n");
		return -EINVAL;
	}

	rw = (bio_data_dir(bio) ? DMA_WRITE : DMA_READ);

//	if ((offset + count) > nvhba->nv_max_ddr_offset)
//		count = nvhba->nv_max_ddr_offset - offset;

	dma_req = kmem_cache_alloc(nv_dma_req_cache, GFP_KERNEL);
	if (dma_req == NULL)
		return -ENOMEM;

	memset(dma_req, 0, sizeof(*dma_req));
	dma_req->dma_next = NULL;
	dma_req->dma_private = bio;
	dma_req->dma_status = 0xFF;
	dma_req->dma_barrier_req = 0;
	dma_req->dma_done = nv_blk_req_done;

	dma_req_head = dma_req;

	memset(&cntxt, 0, sizeof(prd_merge_cntxt_t));
	nv_get_req_prd(dma_req, rw, NV_HOST_PRD,
			&cntxt.prd, &cntxt.nr_prds);

	cntxt.prd_idx = -1;
//	bio_for_each_segment(bvec, bio, segno) {
//		paddr = bvec_to_phys(bvec);
//		valid_bytes = bvec->bv_len;
	bio_for_each_segment(bvec, bio, iter) {
		paddr = bvec_to_phys(&bvec);
		valid_bytes = bvec.bv_len;

		ret = host_prd_try_merge(&cntxt, paddr, valid_bytes);
		if (ret != 0) {
			/* max prd limit reached. need to split io */
			ret = nv_split_io(nvhba, &dma_req, rw,
					&offset, &cntxt);
			if (ret != 0) {
				goto prep_err;
			}
			nr_splits++;
			host_prd_try_merge(&cntxt, paddr, valid_bytes);
		}
	}

	cntxt.prd_idx++;
	if ((cntxt.nr_prds - cntxt.prd_idx) != 0) {
		memset(cntxt.prd + cntxt.prd_idx, 0,
			sizeof(prd_t) * (cntxt.nr_prds - cntxt.prd_idx));
	}

	dma_req->dma_byte_count = cntxt.byte_count;
	ret = cmd_init_ddr_prd(nvhba, dma_req, offset, rw);
	if (ret != 0)
		goto prep_err;

	if ((ret = nvram_queue_cmd(nvhba, dma_req_head, NULL)) != 0) {
		goto prep_err;
	}

	wam_iostats_record(&nvhba->nv_stats.st_bdev_stats, rw, count);

	return 0;

prep_err:
	if (nr_splits)
		nv_free_chained_req(dma_req_head);
	else
		kmem_cache_free(nv_dma_req_cache, dma_req_head);

	nvram_endio(bio, bio->bi_iter.bi_size, ret);
	return 0;
}


static void
wam_io_done(dma_request_t *dma_req)
{
	wam_iocntxt_t *wam_io;

	wam_io = container_of(dma_req, wam_iocntxt_t, w_dma_req);

	wam_io->w_io_done(wam_io->w_dma_cntxt, dma_req->dma_status);
	kfree(wam_io);
}

/*
 * Kernel IO API to NVRAM
 *
 * @dev_id - wam device id (0 to n-1, n =  nr of devices)
 * @rw - DMA_READ or DMA_WRITE
 * @offset - device offset in bytes
 * @count - number of bytes to write
 * @buffer - buffer address that needs to be written to device.
 * Note: the buffer should be allocated using kmalloc API (the
 * driver uses __pa() to get the physical address to do the DMA).
 * @done - callback function which takes user provided cookie and
 * a status.
 * @cookie - private reference from user which is returned in done
 * callback
 * @return 0 if successful,
 * -ENOMEM - not enough memory
 * -EINVAL - invalid device id
 * -ESPIPE - invalid offset or length
 */
static int
wam_async_io(int dev_id, int rw, u64_t offset, u32_t count,
		char *buffer, void (*done)(void *, wam_errno_t), void *cookie)
{
	int	ret = 0;
	nvram_hba_t	*nvhba;
	dma_request_t	*dma_req;
	wam_iocntxt_t	*wam_io;
	unsigned long	paddr;
	prd_merge_cntxt_t cntxt;

	if (dev_id < 0 || dev_id >= nvram_hba_cnt || count == 0)
		return -EINVAL;

	nvhba = nvram_hba[dev_id];

	if (offset >= nvhba->nv_max_ddr_offset)
		return -ESPIPE;

	if ((offset + count) > nvhba->nv_max_ddr_offset) {
		mv_printk_dbg("offset[%llx] + count[%lx] > Max Offset[%llx]\n",
				offset, count, nvhba->nv_max_ddr_offset);
		return -ESPIPE;
	}

	wam_io = (wam_iocntxt_t *)kmalloc(sizeof(wam_iocntxt_t), GFP_KERNEL);
	if (wam_io == NULL)
		return -ENOMEM;

	memset(wam_io, 0, sizeof(*wam_io));
	wam_io->w_dma_cntxt = cookie;
	wam_io->w_io_done = done;
	wam_io->w_rw = rw;
	wam_io->w_buffer = buffer;
	wam_io->w_count = count;

	dma_req = &wam_io->w_dma_req;
	dma_req->dma_next = NULL;
	dma_req->dma_private = NULL;
	dma_req->dma_status = 0xFF;
	dma_req->dma_barrier_req = 0;
	dma_req->dma_done = wam_io_done;

	memset(&cntxt, 0, sizeof(prd_merge_cntxt_t));
	nv_get_req_prd(dma_req, rw, NV_HOST_PRD,
			&cntxt.prd, &cntxt.nr_prds);

	cntxt.prd_idx = -1;
	paddr = __pa(buffer);

	ret = host_prd_try_merge(&cntxt, paddr, count);
	if (ret != 0) {
		ret = -ENOMEM;
		goto prep_err;
	}

	cntxt.prd_idx++;
	if ((cntxt.nr_prds - cntxt.prd_idx) != 0) {
		memset(cntxt.prd + cntxt.prd_idx, 0,
			sizeof(prd_t) * (cntxt.nr_prds - cntxt.prd_idx));
	}

	dma_req->dma_byte_count = cntxt.byte_count;
	ret = cmd_init_ddr_prd(nvhba, dma_req, offset, rw);
	if (ret != 0)
		goto prep_err;

	if ((ret = nvram_queue_cmd(nvhba, dma_req, NULL) != 0)) {
		goto prep_err;
	}

	return 0;

prep_err:
	if (wam_io)
		kfree(wam_io);

	return ret;
}

/*
 * Write to NVRAM
 *
 * @dev_id - wam device id (0 to n-1, n =  nr of devices)
 * @offset - device offset in bytes
 * @count - number of bytes to write
 * @buffer - buffer address that needs to be written to device.
 * Note: the buffer should be allocated using kmalloc API (the
 * driver uses __pa() to get the physical address to do the DMA).
 * @done - callback function which takes user provided cookie and
 * a status.
 * @cookie - private reference from user which is returned in done
 * callback
 * @return 0 if successful,
 * -ENOMEM - not enough memory
 * -EINVAL - invalid device id
 * -ESPIPE - invalid offset or length
 */
int
wam_write(int dev_id, u64_t offset, u32_t count, char *buffer,
		void (*done)(void *, wam_errno_t), void *cookie)
{
	return wam_async_io(dev_id, DMA_WRITE, offset, count, buffer,
			done, cookie);
}
EXPORT_SYMBOL(wam_write);


/*
 * Read from NVRAM
 *
 * @dev_id - wam device id (0 to n-1, n =  nr of devices)
 * @offset - device offset in bytes
 * @count - number of bytes to read
 * @buffer - buffer address where data needs to be read into.
 * Note: the buffer should be allocated using kmalloc API (the
 * driver uses __pa() to get the physical address to do the DMA).
 * @done - callback function which takes user provided cookie and
 * a status.
 * @cookie - private reference from user which is returned in done
 * callback
 */
int
wam_read(int dev_id, u64_t offset, u32_t count, char *buffer,
		void (*done)(void *, wam_errno_t), void *cookie)
{
	return wam_async_io(dev_id, DMA_READ, offset, count, buffer,
			done, cookie);
}
EXPORT_SYMBOL(wam_read);

typedef struct {
	mv_waitq_t	t_waitq;
	wam_errno_t	t_status;
} wam_test_cntxt_t;

static void
wam_test_done(void *cookie, wam_errno_t err)
{
	wam_test_cntxt_t *cntxt = (wam_test_cntxt_t *)cookie;

	cntxt->t_status = err;
	mv_waitq_signal(&cntxt->t_waitq);
}

static int
wam_io_test(struct file *filp, unsigned long arg)
{
	int		ret;
	wam_iotest_t	info;
	wam_test_cntxt_t cntxt;
	char		*buffer;
	u32_t		i;
	u64_t		offset;
	nvram_hba_t	*nvhba = (nvram_hba_t *)filp->private_data;

	ret = copy_from_user(&info, (void __user *)arg, sizeof(wam_iotest_t));
	if (ret != 0)
		return -EACCES;

	memset(&cntxt, 0, sizeof(wam_test_cntxt_t));
	mv_waitq_init(&cntxt.t_waitq);

	buffer = kmalloc(info.req_size, GFP_KERNEL);
	if (buffer == NULL) {
		mv_printk("failed to allocate buffer of size: %u\n",
				info.req_size);
		return -ENOMEM;
	}

	if (info.rw) {
		memset(buffer, info.pattern, info.req_size);
	}

	offset = info.offset;
	for (i = 0; i < info.nr_ios; i++) {
		if (info.rw)
			ret = wam_write(nvhba->nv_minor, offset,
					info.req_size, buffer,
					wam_test_done, &cntxt);
		else
			ret = wam_read(nvhba->nv_minor, offset,
					info.req_size, buffer,
					wam_test_done, &cntxt);
		if (ret != 0) {
			goto test_err;
		}

		mv_waitq_wait(&cntxt.t_waitq);
		if (cntxt.t_status != WAM_SUCCESS) {
			ret = -EIO;
			goto test_err;
		}
		offset += info.req_size;
	}

	kfree(buffer);
	return 0;

test_err:
	mv_printk("IO test failed after %u iterations\n", i);
	kfree(buffer);
	return ret;
}

static inline void
wam_geniostats_record(wam_geniostats_t *stats, u32_t reqsize)
{
	stats->st_ios++;
	stats->st_bytes += reqsize;

	if (reqsize < stats->st_min_reqsize)
		stats->st_min_reqsize = reqsize;

	if (reqsize > stats->st_max_reqsize)
		stats->st_max_reqsize = reqsize;
}

static void
wam_iostats_record(wam_iostats_t *stats, int rw, u32_t reqsize)
{
	spin_lock(&stats->st_lock);

	if (rw) {
		wam_geniostats_record(&stats->st_wr_stats, reqsize);
	} else {
		wam_geniostats_record(&stats->st_rd_stats, reqsize);
	}

	spin_unlock(&stats->st_lock);
}

static inline void
wam_geniostats_init(wam_geniostats_t *stats)
{
	memset(stats, 0, sizeof(wam_geniostats_t));
	stats->st_min_reqsize = (u64_t)(u32_t)(-1);
}

static inline void
wam_iostats_init(wam_iostats_t *stats)
{
	spin_lock_init(&stats->st_lock);
	wam_geniostats_init(&stats->st_rd_stats);
	wam_geniostats_init(&stats->st_wr_stats);
}

static void
wam_stats_init(wam_stats_t *stats)
{
	wam_iostats_init(&stats->st_cdev_stats);
	wam_iostats_init(&stats->st_bdev_stats);
}

module_init(nvram_init);
module_exit(nvram_exit);
