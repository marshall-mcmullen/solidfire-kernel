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

#ifndef __LOKI_DRV_H
#define __LOKI_DRV_H

#define PCI_VENDOR_ID_MARVELL		0x11ab
#define PCI_DEVICE_ID_MARVELL_MV8180	0x8180

#define	WAM_DRV_VERSION_MAJOR		1
#define	WAM_DRV_VERSION_MINOR		7
#define	WAM_DRV_VERSION_BUGFIX		2

/*
 * printk wrappers
 */
#define	mv_printk(fmt, args...)		printk("wam: "fmt, ##args)

#ifdef LOKI_ENABLE_DBG_TRACE
#define mv_printk_dbg(fmt, args...)	printk("wam_dbg: "fmt, ##args)
#else
#define mv_printk_dbg(fmt, args...)
#endif

/* device register space size (1MB) */
#define NV_REG_SPACE_SIZE		(1 << 20)

/* pci-e window size to access internal scratchpad memory */
#define NV_SRAM_MEM_SIZE		(512 << 10)

/* pci-e window size to access boot flash */
#define NV_FLASH_ACCESS_SIZE		(64 << 10)

/* pci-e window size to access event log */
#define NV_NSRAM_ACCESS_SIZE		(128 << 10)

/* offset from the BAR to access boot flash */
#define NV_FLASH_OFFSET			NV_SRAM_MEM_SIZE

/* offset from the BAR to access event log */
#define NV_NVSRAM_OFFSET		(NV_FLASH_OFFSET + \
						2 * NV_FLASH_ACCESS_SIZE)

/* BAR ID to access scratchpad, boot flash and event log */
#define NV_CTRL_ACCESS_BAR		2

/* Size of any page exchanged with firmware during handshake */
#define NV_DMA_CTRL_PAGE_SIZE		(64 << 10)

/* char driver name */
#define MV_DRIVER_NAME			"mvwam"

/* block driver name */
#define MV_BLK_DRIVER_NAME		"nvdisk"

#define	MV_WAM_NAME_SIZE		64

/* Maximum size of a PRD buffer (4MB - 4KB) */
#define MV_MAX_PRD_BUF_SIZE             ((1 << 22) - 4096)

/* max hbas */
#define MV_MAX_HBA			8

#if 0
/* MMAP offset to access device registers */
#define NV_MMAP_REG_OFFSET		0

/* MMAP offset to access firmware boot flash */
#define NV_MMAP_FLASH_OFFSET		(NV_MMAP_REG_OFFSET + \
						NV_REG_SPACE_SIZE)

/* Max MMAP offset */
#define NV_MAX_MMAP_OFFSET		(NV_MMAP_FLASH_OFFSET + \
						NV_FLASH_ACCESS_SIZE)
#endif

/* PCI-E Window allocation for different accesses */
#define NV_PCIE_SRAM_WIN		0
#define NV_PCIE_FLASH_WIN		1
#define NV_PCIE_NVSRAM_WIN		2

/* Nr of src and dst sglist entries per command */
#define MV_PRDS_PER_CMD			16
#define MV_SRC_PRDS_PER_CMD		8
#define MV_DST_PRDS_PER_CMD		(MV_PRDS_PER_CMD - MV_SRC_PRDS_PER_CMD)

/*
 * BAR information
 */
typedef struct loki_bar_s {
	u64_t	bar_base;
	u32_t	bar_length;
	u32_t	rsvd;
} loki_bar_t;

/*
 * PRD Table per command
 */
typedef struct cmd_prd_tbl {
	prd_t	c_src_prds[MV_SRC_PRDS_PER_CMD];
	prd_t	c_dst_prds[MV_DST_PRDS_PER_CMD];
} cmd_prd_tbl_t;

/*
 * DMA Command Header Format
 *
 * The reserved fields are initialized by firmware. The driver needs to
 * update only the byte count while posting a command to hw.
 */
typedef struct cmd_hdr_struct {
	u32_t	cmd_rsvd[2];		/* fields initialized by firmware */
	u32_t	cmd_byte_count;		/* data transfer byte count */
	u32_t	cmd_rsvd1[11];		/* fields initialized by firmware */
} cmd_hdr_t;

/* forward type declaration of DMA request structure */
typedef struct dma_request dma_request_t;

/* DMA completion callback funtion type */
typedef void (*dma_done_t)(dma_request_t *);

/* forward declaration of nvram hba structure */
typedef struct nvram_hba nvram_hba_t;


/* dma command queuing error codes */
enum {
	DMA_SUCCESS = 0,	/* queued successfully */
	DMA_QUEUE_FULL,		/* dma command queue full */
	DMA_STATE_ERROR		/* ecc error state. dma engine to be reset */
};

/* dma state machine states */
enum {
	DMA_NOT_INITIALIZED,
	DMA_INITIALIZED,
	DMA_NEED_RESET,
};

/* DRAM access modes */
typedef enum {
	DRAM_WRITE_MODE,	/* Enable DRAM write mode */
	DRAM_RDONLY_MODE	/* Enable Read-only mode if data is already
				   backed up to SSD (used during supercap
				   calibration) */
} dram_access_mode_t;

/*
 * DMA Request structure
 */
struct dma_request {
	dlist_t		dma_link;		/* for lld queuing */
	dma_request_t	*dma_next;		/* link for chained request */
	cmd_prd_tbl_t	dma_prd_tbl;		/* PRD table with src & dst */
	u32_t		dma_byte_count;		/* data transfer count */
	short		dma_barrier_req;	/* boolean flag for barrier */
	short		dma_status;		/* success or failed */
	dma_done_t	dma_done;		/* callback function */
	void		*dma_private;		/* private for caller's use */
	u64_t		dma_id;			/* unique id for debug */
	u64_t		dma_ticks;		/* ticks for latency */
};


typedef struct cdev_request {
	u64_t		cr_sign;		/* signature */
	dma_request_t	cr_dma_req;		/* dma request */
	mv_waitq_t	dma_waitq;		/* waitq for waiting */
	struct page	**cr_page_list;		/* for pinning user pages */
	struct page	*cr_single_page;	/* for single page reqs */
	u32_t		cr_nr_pages;		/* nr of pages */
} cdev_request_t;


typedef struct chained_request {
	dma_done_t	ch_done;		/* request done upcall */
	void		*ch_private;		/* private for caller's use */
	u16_t		ch_nr_reqs;		/* nr of reqs in chain */
	u16_t		ch_nr_done;		/* nr of reqs completed */
	int		ch_status;		/* status of the request */
	dma_request_t	*ch_dma_req;		/* start of request chain */
} chained_request_t;

typedef struct wam_ioctxt {
	dma_request_t	w_dma_req;
	void		*w_dma_cntxt;
	void (*w_io_done)(void *, int);
	void		*w_buffer;
	u32_t		w_count;
	u32_t		w_rw;
} wam_iocntxt_t;

typedef struct wam_geniostats {
	u64_t		st_ios;
	u64_t		st_bytes;

	u32_t		st_min_reqsize;
	u32_t		st_max_reqsize;
} wam_geniostats_t;

typedef struct wam_iostats {
	spinlock_t		st_lock;
	wam_geniostats_t	st_rd_stats;
	wam_geniostats_t	st_wr_stats;
} wam_iostats_t;

typedef struct wam_stats {
	u64_t		st_intrs;
	u64_t		st_tsklt_runs;

	u32_t		st_io_rescues;
	u32_t		st_rsvd;

	wam_iostats_t	st_cdev_stats;
	wam_iostats_t	st_bdev_stats;
} wam_stats_t;

/*
 * DMA Control structure
 */
typedef struct nvram_dma {
	dma_addr_t	nv_shadow_prd_pa;	/* shadow prd table phy addr */
	dma_addr_t	nv_cmplq_pa;		/* cmpl q phy addr */
	cmd_prd_tbl_t	*nv_shadow_prd;		/* shadow prd table array */
	cmd_prd_tbl_t   *nv_sram_prd;           /* prd tbl location in sram */
	cmplq_t		*nv_cmplq;		/* completion queue */
	cmd_hdr_t	*nv_cmd_tbl;		/* command header table */
	u64_t		nv_queued_id;		/* id counter */
	u64_t		nv_cmpltd_id;		/* completion counter */
	dlist_t		nv_request_list;	/* request queue */
	dlist_t		nv_posted_list;		/* cmds posted to hw */
	spinlock_t	nv_lock;		/* lock for the queue */
	u32_t		nv_nr_slots;		/* nr of command slots */
	u32_t		nv_state;		/* dma engine state */
	u32_t		nv_cmplq_wptr;		/* last noted completion */
	u32_t		nv_sw_dlvryq_wptr;	/* delivery queue wr ptr */
	u32_t		nv_hw_dlvryq_wptr;	/* last posted wr ptr */
	dram_access_mode_t nv_mode;		/* access mode: rdonly? */
	wait_queue_head_t	nv_broadcast_event;	/* completion event */
	int		nv_errcode;		/* dma errno */
} nvram_dma_t;


/*
 * Thread per HBA structure
 */
typedef struct nvram_thread {
	mv_waitq_t	thr_event;		/* waitqueue to sleep */
	nvram_hba_t	*thr_hba;		/* hba reference */
	u32_t		thr_alive;		/* flag to quit the thread */
} nvram_thread_t;

/*
 *  Keeps track of all the information about the nvram device.
 */
struct nvram_hba {
	char		nv_name[MV_WAM_NAME_SIZE];	/* dev name */
	int		nv_major;		/* Major Number */
	int		nv_minor;		/* Minor Number */
	struct cdev	nv_cdev;		/* Char device */
	struct pci_dev	*nv_pcidev;		/* pci dev handle */
	void		*nv_regs;		/* register base addr */
	void		*nv_sram_base;		/* internal memory base */
	void		*nv_flash_base;		/* boot flash addr */
	void		*nv_log_base;		/* event log base addr */
	loki_bar_t	nv_bar[MV_MAX_BARS];	/* BAR Info */
	u32_t		nv_ddr_cs_cnt;		/* nr of ddr cs */
	u32_t		nv_ddr_size_shft;	/* size shift per cs */
	u64_t		nv_ddr_size;		/* DDR size */
	u64_t		nv_max_ddr_offset;	/* max offset */
	u32_t		nv_dma_init;		/* initialized? flag */
	u32_t		nv_irq_enabled;		/* irq flag */
	u32_t		nv_msi_enabled;		/* msi flag */
	nvram_dma_t	nv_dma_ctrl;		/* DMA Control */
	nvram_thread_t	*nv_thr_ctrl;		/* Thread control struct */
	void		*gdisk;			/* gendisk ptr */
	struct tasklet_struct nv_irq_tasklet;	/* linux tasklet */
	struct tasklet_struct nv_req_tasklet;	/* linux tasklet */
	u16_t		nv_scap_calib;		/* calib on/off flag */
	u16_t		nv_calib_type;		/* calib type */
	u32_t		nv_drbl_mask;		/* drbl mask */
	u32_t		nv_alert_count;		/* fw alert counter */
	wam_shdw_cause_t *nv_shdw_cause;	/* shadow cause */
	u32_t		(*nv_get_cause)(nvram_hba_t *);
	u32_t		nv_alerts;		/* alert delivered to app */
	u32_t		nv_pending_alerts;	/* new pending alerts */
	u32_t		nv_pkg_ver_comp;
	u32_t		nv_pci_state[64];       /* to store pci state */

	wam_stats_t	nv_stats;		/* device stats */

	struct semaphore	nv_msglock;	/* lock for FW messaging */
	struct semaphore	nv_oplock;	/* lock for operations */

	unsigned long	nv_op_expires;		/* op expiration time */

	u32_t		nv_exit_flag;		/* for async threads to quit */
	u32_t		nv_timer_delay;		/* in jiffies */

	u64_t		nv_prev_queued_id;
	u64_t		nv_prev_cmpltd_id;

	struct timer_list nv_timer;
};

#define	NV_MAX_MSG_DATA		3

typedef struct {
	u16_t	msg_code;
	u16_t	msg_rsp;
	u32_t	msg_data[NV_MAX_MSG_DATA];		/* msg args/rsps */
	u32_t	msg_waitmsecs;			/* timeout in millisecs */
} wam_fwmsg_t;

enum {
	NV_PKG_VER_MISMATCHED	= 0,
	NV_PKG_VER_COMPATIBLE	= 1,
};

#endif


