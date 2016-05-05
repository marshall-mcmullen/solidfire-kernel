/*****************************************************************************
 *
 * Copyright (C) 2009-2013  Integrated Device Technology, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * $RCSfile: nvme.h,v $
 * $Revision: 1.41 $
 * $Date: 2014/06/10 12:38:57 $
 * $Author: bermanor $
 * $Release: $ Name:  $
 *
 * You can contact Integrated Device Technology, Inc via email ssdhelp@idt.com
 * or mail Integrated Device Technology, Inc
 * 6024 Silver Creek Valley Road, San Jose, CA 95128, USA
 *
 *
 *****************************************************************************/

#ifndef NVME_H_
#define NVME_H_

#include <linux/version.h>

/**
 * @file nvme.h - header file describing IDT NVME driver data structures.
 * 		This file defines all data structures and constants used by
 *		NVME Linux driver module.
 */


/**
 * Driver conditional compile flags.
 */
#define NVME_NUMA_VECTORS 0	/* Enable to allow IRQ vector per NUMA node. */

#define BIT_0		(1 << 0)
#define BIT_1		(1 << 1)
#define BIT_2		(1 << 2)
#define BIT_3		(1 << 3)
#define BIT_4		(1 << 4)
#define BIT_5		(1 << 5)
#define BIT_6		(1 << 6)
#define BIT_7		(1 << 7)
#define BIT_8		(1 << 8)
#define BIT_9		(1 << 9)
#define BIT_10		(1 << 10)
#define BIT_11		(1 << 11)
#define BIT_12		(1 << 12)
#define BIT_13		(1 << 13)
#define BIT_14		(1 << 14)
#define BIT_15		(1 << 15)
#define BIT_16		(1 << 16)
#define BIT_17		(1 << 17)
#define BIT_18		(1 << 18)
#define BIT_19		(1 << 19)
#define BIT_20		(1 << 20)
#define BIT_21		(1 << 21)
#define BIT_22		(1 << 22)
#define BIT_23		(1 << 23)
#define BIT_24		(1 << 24)
#define BIT_25		(1 << 25)
#define BIT_26		(1 << 26)
#define BIT_27		(1 << 27)
#define BIT_28		(1 << 28)
#define BIT_29		(1 << 29)
#define BIT_30		(1 << 30)
#define BIT_31		(1 << 31)

#define	DRV_NAME	"nvme_radian"
#define	DEV_NAME	"nvme"

#ifdef DEBUG
#ifndef NVME_DEBUG
#define NVME_DEBUG	1
#endif
#endif

#ifdef NVME_DEBUG
#define NVME_DEBUG_ALL	(BIT_0|BIT_1|BIT_2|BIT_3|BIT_4|BIT_5|BIT_6|BIT_7|\
			BIT_8|BIT_9|BIT_10|BIT_11|BIT_12|BIT_16|BIT_17|BIT_31)
#define NVME_DEBUG_IO	(BIT_0|BIT_1|BIT_2|BIT_3|BIT_4|BIT_31)
#define NVME_DEBUG_TIMEOUT	(BIT_4)
#define NVME_DEBUG_IOCTL	(BIT_8|BIT_9|BIT_10)
#define NVME_DEBUG_DIF		(BIT_0|BIT_12|BIT_16)
#define NVME_DEBUG_LOG		(BIT_11)
#define NVME_DEBUG_DUMP		(BIT_16)
#define NVME_DEBUG_DUMP_CE	(BIT_17)
#define NVME_DEBUG_DUMP_Q	(BIT_18)
#define NVME_DEBUG_DUMP_TIME	(BIT_19)
#define NVME_DEBUG_TEMP		(BIT_31)

#define	DPRINT(fmt, arg...)	\
	if (nvme_dbg & BIT_0) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT1(fmt, arg...)	\
	if (nvme_dbg & BIT_1) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT2(fmt, arg...)	\
	if (nvme_dbg & BIT_2) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT3(fmt, arg...)	\
	if (nvme_dbg & BIT_3) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT4(fmt, arg...)	\
	if (nvme_dbg & BIT_4) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT5(fmt, arg...)	\
	if (nvme_dbg & BIT_5) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT6(fmt, arg...)	\
	if (nvme_dbg & BIT_6) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT8(fmt, arg...)	\
	if (nvme_dbg & BIT_8) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT9(fmt, arg...)	\
	if (nvme_dbg & BIT_9) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT10(fmt, arg...)	\
	if (nvme_dbg & BIT_10) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT11(fmt, arg...)	\
	if (nvme_dbg & BIT_11) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT12(fmt, arg...)	\
	if (nvme_dbg & BIT_12) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINT31(fmt, arg...)	\
	if (nvme_dbg & BIT_31) \
	    dev_printk(KERN_DEBUG, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	DPRINTX(fmt, arg...)	\
	if (nvme_dbg & BIT_0) \
	    printk(KERN_DEBUG DRV_NAME " [%s] " fmt , __func__ , ##arg)

#define	DPRINTX12(fmt, arg...)	\
	if (nvme_dbg & BIT_12) \
	    printk(KERN_DEBUG DRV_NAME " [%s] " fmt , __func__ , ##arg)
#else
#define	DPRINT(fmt, arg...)
#define	DPRINT1(fmt, arg...)
#define	DPRINT2(fmt, arg...)
#define	DPRINT3(fmt, arg...)
#define	DPRINT4(fmt, arg...)
#define	DPRINT5(fmt, arg...)
#define	DPRINT6(fmt, arg...)
#define	DPRINT8(fmt, arg...)
#define	DPRINT9(fmt, arg...)
#define	DPRINT10(fmt, arg...)
#define	DPRINT11(fmt, arg...)
#define	DPRINT12(fmt, arg...)
#define	DPRINT31(fmt, arg...)
#define	DPRINTX(fmt, arg...)
#define	DPRINTX12(fmt, arg...)
#endif

#define	EPRINT(fmt, arg...)	\
	dev_printk(KERN_ERR, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	NPRINT(fmt, arg...)	\
	dev_printk(KERN_NOTICE, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define	IPRINT(fmt, arg...)	\
	dev_printk(KERN_INFO, dev->dma_dev, "[%s] " fmt , __func__ , ##arg)

#define NVME_DELAY	2
#define NVME_WAIT	1
#define NVME_NO_WAIT	0

#define nvme_wait_cond(dev, wait, cond, result) \
do {\
	int max_wait = (wait) * 1000;\
	int next_wait = (999 + HZ)/HZ;\
	DPRINT2("waiting %d\n", max_wait);\
	(result) = 0;\
	while (!(cond)) {\
	    if (fatal_signal_pending(current)) {\
		(result) = -EINTR;\
		break;\
	    }\
	    if (max_wait <= 0) {\
		(result) = -ETIME;\
		break;\
	    }\
	    msleep_interruptible(next_wait);\
	    max_wait -= next_wait;\
	    next_wait <<= 1;\
	    if (next_wait > max_wait)\
	    	next_wait = max_wait;\
	    if (next_wait > 1000)\
	    	next_wait = 1000;\
	}\
	DPRINT2("cond %d, max_wait %d, result %d\n",\
				(cond), max_wait, (result));\
} while(0)

#define nvme_delay_cond(dev, wait, cond, result) \
do {\
	int max_wait = (wait) * 1000;\
	int next_wait = (999 + HZ)/HZ;\
	DPRINT2("waiting %d\n", max_wait);\
	(result) = 0;\
	while (!(cond)) {\
	    if (fatal_signal_pending(current)) {\
		(result) = -EINTR;\
		break;\
	    }\
	    if (max_wait <= 0) {\
		(result) = -ETIME;\
		break;\
	    }\
	    mdelay(next_wait);\
	    max_wait -= next_wait;\
	    next_wait <<= 1;\
	    if (next_wait > max_wait)\
	    	next_wait = max_wait;\
	    if (next_wait > 1000)\
	    	next_wait = 1000;\
	}\
	DPRINT2("cond %d, max_wait %d, result %d\n",\
				(cond), max_wait, (result));\
} while(0)

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )

#define BIO_REQ_WRITE           REQ_WRITE
#define BIO_REQ_FAILFAST_DEV    REQ_FAILFAST_DEV
#define BIO_REQ_RAHEAD          REQ_RAHEAD
#define BIO_REQ_FLUSH           REQ_FLUSH
#define BIO_REQ_FUA             REQ_FUA
#define BIO_REQ_DISCARD         REQ_DISCARD

#else

#define BIO_REQ_FAILFAST_DEV	REQ_FAILFAST_DEV
#define	BIO_REQ_FUA	 	REQ_FUA
#define BIO_REQ_RAHEAD		REQ_RAHEAD
#define BIO_REQ_WRITE		REQ_OP_WRITE
#define	BIO_REQ_FLUSH 		REQ_OP_FLUSH
#define	BIO_REQ_DISCARD 	REQ_OP_DISCARD

#endif

#define ADMIN_Q_SIZE	128
#define MAX_NR_QUEUES	128


enum
{
    NVME_STATE_INIT = 0x0,
    NVME_STATE_OPERATIONAL,
    NVME_STATE_SUSPEND,
    NVME_STATE_RESUME,
    NVME_STATE_RESET,
    NVME_STATE_ERROR,
    NVME_STATE_FAILED,
    NVME_STATE_SHUTDOWN,
};



#define MAX_MINORS	(8)			/* Allow Maximum 8 partitions */
#define MAX_NAMESPACES	(257)			/* Namespaces per controller */
#define MAX_MINOR_DEV	(MAX_NAMESPACES * MAX_MINORS)	/* Maximum minor/ctlr */
#define NVME_DEF_IRQ_MASK ((u32)~(1))		/* Mask all accept Admin Q */

#ifdef NVME_DEBUG
#define DEVICE_TIMEOUT	    100 		/* IO Timeout in seconds */
#define TIMEOUT_FREQ	    (10*HZ)
#else
#define DEVICE_TIMEOUT	    20 			/* IO Timeout in seconds */
#define TIMEOUT_FREQ	    (HZ)
#endif
#define	TIMEOUT_LIST	    ((DEVICE_TIMEOUT*HZ)/TIMEOUT_FREQ)
#define ADMIN_TIMEOUT	    (2*HZ)

#define MAX_EVENTS	    7

#define LOG_PG_SIZE	    (512)
#define MAX_DISCARD_BLK     (0x2000000)

struct queue_info;
struct sub_queue_info;
struct cmd_info;

/**
 * Device context data structure
 *
 */
struct nvme_dev {

	spinlock_t		lock;		/* Device context lock */
	ulong			lock_flags;	/* flags used for lock */
	int			instance;	/* device instance */
	int			major;		/* Block device instance. */
	struct pci_dev *	pci_dev;	/* PCI device 		*/
	struct device *		dma_dev;	/* DMA Device handle */
	struct device *		ctrl_dev;	/* control Device handle */
	int			state;		/* Device State */
	int			thread_stop;	/* flags to stop thread */

#define THREAD_STOP		BIT_0
#define THREAD_RESTART		BIT_1
#define THREAD_AEN		BIT_2
#define THREAD_LOG		BIT_3
#define THREAD_ERR		BIT_4

	__u32			thread_flags;	/* Flags to request a task */
	void  * __iomem		reg;		/* Controller BAR */
	int			vector_cnt;	/* Number of IRQ vectors */
	struct irq_vector *	irq_entry;	/* Array of IRQ vectors */
	int			que_count;	/* No of completion queues */
	struct queue_info *	que_list[MAX_NR_QUEUES]; /* Queue Info */
	struct sub_queue_info *	sque_list[MAX_NR_QUEUES]; /* Sub Queue Info */
	struct queue_info *	cpu_to_que_map[MAX_NR_QUEUES]; /* cpu to Queue Info mapping */
	int			ns_count;	/* No of Namespaces */
	struct list_head	ns_list;	/* List of Namespaces */
	char			serial[20];	/* Controller serial number */
	char			model[40];	/* Controller model number */
	char			firmware_rev[8];/* Firmware version */
	int			timeout_id;	/* Timeout index */
	__u16			max_aen;	/* Max nr of Async request */
	__u16			cur_aen;	/* Max nr of Async request */
	__u8			admVendCmdCfg;	/* Vendor Admin cmnd config */
	__u8			nvmVendCmdCfg;	/* Vendor NVM cmnd config */
	__u8			nvmCacheSupport;/* NVM supported cache Config */
	__u16			nvmCmdSupport;	/* NVM supported command */
	__u32               	wq_done;	/* nr of wq_thread processed */
	struct iden_controller *identify;	/* Controller Ident data */
	struct msix_entry *	msix_entry;	/* Array of MSI-X vectors */
	struct queue_info *	admin_queue;	/* Admin queue */
	int			event_count;	/* Number of pending events */
	struct list_head	event_list;	/* List of event info */
	spinlock_t		ioctl_lock;	/* IOCTL lock */
	wait_queue_head_t	ioctl_wait;	/* IOCTL waiters */
	wait_queue_head_t	event_wait;	/* IOCTL waiters */
	wait_queue_head_t	thread_wait;	/* device thread waiter */
	struct task_struct *	nvme_thread; 	/* Synchronous cmd thread */
	struct timer_list 	timer;		/* Device timer */

 	void (*lock_func)   (spinlock_t *, ulong *); /* lock function */
 	void (*unlock_func) (spinlock_t *, ulong *); /* unlock function */

	struct proc_dir_entry *proc_dir;
	struct proc_dir_entry *proc_entry_stats;
	struct proc_dir_entry *proc_entry_qinfo;
	u64					dma_int;
	u64					work_queued;
	u64					bytes_written;
	u64					bytes_read;
	u64					user_req;
	u64					sub_req;
	u64					complete_q_cnt;
	u64					doorbell_req;
#if 0
	struct timespec 	ts_start;
	struct timespec 	ts_end;
	u64					total_time;
#endif
	u64					ddr_phys;
	unsigned char  __iomem *ddr_remap;
	u64					ddr_len;		/* trimmed length due to system limitations */
	u64			hw_cap;
	u32			hw_timeout;
	u32			boot_cfg;

	int			rms_off;
};

/**
 * Namespace information block data structure
 */
struct ns_info {

	struct 	list_head	list;		/* list of Namespace */
	struct 	request_queue *	queue;		/* Block device request queue */
	struct 	gendisk *	disk;		/* Generic Disk */
	struct nvme_dev *	dev;		/* device context */
	__u32			flags;		/* Namespace flags */
	int			id;		/* Namespace ID */
	int			ref_count;	/* Namespace Reference count */

#define NS_ONLINE			BIT_0
#define NS_FLUSH			BIT_1
#define NS_READONLY			BIT_2
#define NS_TRANSIENT		BIT_3

	__u64			block_count;	/* Size of Namespace(blocks) */
	int			lba_shift;	/* Shift for lba address */
	__u8			feature;	/* features set NS_IDENTIFY */
	__u8			fmtLbaSize;	/* Formatted LBA size */
	__u8			metaDataCap;	/* Metadata Capability */
	__u8			dataProtCap;	/* End-to-End prot. cap. */
	__u8			dataProtSet;	/* End-to-End prot. set */
	__u16			metasize;	/* Size of Meta data */
	__u64			vendor;		/* RMS vendor specific extention */
};




/**
 * Queue Information data structure
 *
 */
struct queue_info {

	spinlock_t	   lock;	    /* lock controls this structure */
	ulong		   lock_flags;	    /* flags used for lock */
	struct  nvme_dev * dev;	    	    /* driver device context. */
	int		   nr_req;	    /* Number of BIO request */
	int		   nr_act;	    /* Number of active BIOs */
	int		   max_req;	    /* Max. Number of BIO request */
	int        nrf_sq_cong; /* number of flushed sq_conq bios */
	int        nrf_fq;      /* number of nvme_flush_queue bio ends */
	int 	   nrf_timeout; /* number of high level flush due to timeout */
#define	QUEUE_READY	(1 << 0)
#define	QUEUE_SUSPEND	(1 << 1)
#define	QUEUE_FLUSH	(1 << 2)
#define	QUEUE_BUSY	(1 << 8)

	__u32		    flags;	    /* queue flags */
	__u32		    id;		    /* Completion queue ID */
	__u32		    qsize;	    /* Completion queue entries */
	__u32		    id_count;	    /* io_command_id_size */
	__u32		    prp_cnt;	    /* number of pre-allocated PRPs */
	__u32		    cpu;	    /* 1st CPU associated with queue. */
	__u32		    node;	    /* Assigned NUMA node. */
	__u32		    cpu_cnt;	    /* Number of CPUs sharing queue. */
	__u64		    cpu_mask[2];    /* CPU iBit Mask */
 	__u32		    cq_ivec;	    /* Completion Queue vector ID */
	__u32		    ivec_acquired;  /* vector ID acquired */
	__u8		    ivec_name[32];  /* Vector ascii name */
	__u32		    phase;	    /* Completion queue phase (0/1) */
	__u32		    timeout_id;	    /* Expired timeout ID */
	__u16 		    tail;	    /* Completion queue tail index */
	__u16               head;	    /* Completion queue head index */
	struct cmd_info *   cmd_list;	    /* Command info list */
	struct list_head    cmd_free;	    /* free command info list */
	struct list_head    cmd_active;	    /* active command info list */
	struct bio_list     sq_cong;	    /* Congested BIO request list */
	struct cq_entry *   compq;	    /* Completion queue addr. */
	dma_addr_t	    compq_phy;	    /* Completion queue physical addr */
	struct dma_pool *   prp_pool;	    /* PRP list pool. */
	__u32 * __iomem	    doorbell;	    /* Queue doorbell register */
	wait_queue_head_t   waitq;          /* Wait queue TBD */
	wait_queue_t 	    cong_queue;	    /* Congestion queue */

	__u32 		    timeout[TIMEOUT_LIST];   /* timeout list */

	struct sub_queue_info *	sub_queue;  /* Submission queues */
	struct work_struct  wq_work;        /* Workqueue work */
	struct work_struct  wq_time;        /* Workqueue timeout */

 	void (*lock_func)   (spinlock_t *, ulong *); /* lock function */
 	void (*unlock_func) (spinlock_t *, ulong *); /* unlock function */
};



struct sub_queue_info {

	spinlock_t	    lock;	    /* lock controls this structure */
	ulong		    lock_flags;	    /* flags used for lock */
	struct  nvme_dev *  dev;	    /* driver device context. */
	__u32		    Flags;	    /* queue flags */
	__u32		    id;	    	    /* Submission queue ID */
	__u32		    qsize;	    /* number of submission entries */
	__u32		    entries;	    /* number of available entries */
	__u32		    throttle;  	    /* Submission Queue throttle */
	__u16   	    tail;	    /* Submission queue tail pointer */
	__u16   	    head;	    /* Submission queue head pointer */
	struct nvme_cmd *   subq;	    /* Submission queue addr. */
	dma_addr_t	    subq_phy;	    /* Submission queue physical addr */
	struct queue_info * compq;	    /* Completion Queue */
	__u32 * __iomem	    doorbell;	    /* Queue doorbell register */

 	void (*lock_func)(spinlock_t *, ulong *);   /* lock function */
 	void (*unlock_func)(spinlock_t *, ulong *); /* unlock function */
	u64					sub_req;
};



struct cmd_info {

	spinlock_t	    lock;	    /* lock controls this structure */
	ulong		    lock_flags;	    /* flags used for lock */
	struct list_head    list;      	    /* free/active list */
	void *		    req;	    /* Original request bio */
	void *		    part;	    /* request part (statatistic) */
	struct ns_info *    ns;	    	    /* Namespace */

#define FREE_CONTEXT	0		    /* Unused context */
#define ADMIN_CONTEXT	1		    /* Driver Admin Command request */
#define BIO_CONTEXT	2		    /* Block IO request */
#define IOCTL_CONTEXT	3		    /* IOCTL request */
#define EVENT_CONTEXT	4		    /* Async. Event request */
#define LOG_CONTEXT	5		    /* Log Page request */
#define ERR_CONTEXT	6		    /* Error Page request */
#define ABORT_CONTEXT	7		    /* Command aborted */

#define MAX_RETRY	2

	__u16		    type;	    /* Command type */
	__u32		    status;	    /* Command status */
	__u32		    cmd_status;	    /* cumulative status */
	__u16		    cmd_id;	    /* Command context */
	__u16		    timeout_id;	    /* Timeout ID */
	__u32		    count;	    /* Data transfer count */
	__u32		    req_length;	    /* Total request byte count */
	__u32		    cmd_count;	    /* Number of nvme commands sent */
	__u16		    cmd_retries;    /* retry count */
	struct bvec_iter cmd_bvec_iter;
	struct cmd_info *   cmd_base;	    /* base Command info */
	struct nvme_prp *   prps;	    /* PRP list */
	dma_addr_t	    prp_phy;	    /* PRP list physical address */
	struct nvme_cmd     nvme_cmd;	    /* NVMe Command template */
	__u64		    start_time;	    /* timer tick when req received */
#define cmd_param	    start_time	    /* used as admin req param holder */
	__u32		    sg_count;	    /* Number of SG list entries */
	struct scatterlist *sg_list;	    /* Array of SG list entries */
#ifdef CONFIG_BLK_DEV_INTEGRITY
	struct scatterlist piv_sg_list[2];  /* ProtInfo SG list */
#endif
	struct cq_entry	    cq_entry;	    /* Synchronous requests */
	struct task_struct *task;	    /* Synchronous requests */
	wait_queue_head_t   waitq;	    /* Synchronous wait queue entry */

 	void (*lock_func)(spinlock_t *, ulong *);   /* lock function */
 	void (*unlock_func)(spinlock_t *, ulong *); /* unlock function */
};

#define MAX_PAGE_SIZE	512		/* Maximum event log data size */

/**
 * Event information block data structure
 *
 */
struct event_info {

	struct 	list_head	list;	    /* list of Namespace */
	__u32			event_id;   /* Event ID */
	size_t			size;	    /* Event Page Data size */
	void *			buff;	    /* Buffer Page Data */
	wait_queue_head_t       waitq;	    /* Synchronous wait queue entry */
};


#endif
