/*****************************************************************************;
 *
 * Copyright (C) 2009-2013  Integrated Device Technology, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * $RCSfile: nvme.c,v $
 * $Revision: 1.76 $
 * $Date: 2014/06/10 12:38:41 $
 * $Author: bermanor $
 * $Release: $Name:  $
 *
 * You can contact Integrated Device Technology, Inc via email ssdhelp@idt.com
 * or mail Integrated Device Technology, Inc
 * 6024 Silver Creek Valley Road, San Jose, CA 95128, USA
 *
 *
 *****************************************************************************/

/**
 * @file nvme.c - Linux driver for IDT NVM Express Block Device Driver.
 *  	This module is a linux block device driver that exports
 *  	NVME controller Namespaced as block device to Linux.
 */
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kdev_t.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/poison.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/idr.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/libata.h>
#include <linux/crc-t10dif.h>
#include "nvme_express.h"
#include "nvme.h"
#include "nvme_ioctl.h"
#include "nvme_rms.h"

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/**
 * Conditional compile flags
 */
#define USE_NS_ATTR 	0
#define NVME_DISCARD		1
#define DO_IO_STAT  		0
#define SEND_AEN		1
#define FORCE_ERROR 	0
#define ERROR_FREQ  		1000
#define FORCE_DIF_ERROR 	0
#define ERROR_DIF_FREQ  	((1000/8) * 8)
#define FORCE_TIMEOUT   	0
#define FREQ_FORCE_TIMEOUT  100000

#define IDT_APP_TAG 	0x4944  /* "ID" */

/*
 * Attempt to reset hardware on certain paths
*/
/*#define CONFIG_HW_RESET_ON_ERR */


/*
 * devinit and devexit macros were removed in Kernel 3.8 and up
 * might need a further look at if they're replaced by anything
 * else. The capability for hotplug in the pci subsystem of 3.8
 * is reported to integrate dev<xx> macros.
 */
#define __devinit(x) x
#define __devexit(x) x
#define __devexit_p(x) x

/*
 * Move these global variables into the per device structure,
 * but keep the old names to help keep the code looking the
 * same in order to aid with merging new versions.
 */
#define nvme_hw_cap (dev->hw_cap)
#define nvme_hw_timeout (dev->hw_timeout)

/**
 * @note The threaded interrupt is diabled by default.
 *  	 Actual interrupt comes is on correct CPU but the
 *  	 interrupt thread may come in on different CPU.
 *  	 This may be an issue with kernel or driver and
 *  	 should be furthur investigated.
 */
static int  use_threaded_interrupts = 0;	/* Preemption issue */
static int  max_prp_list = 32;
static int  nvme_major = 0;
static int  nvme_cmajor = 0;
#if DO_IO_STAT
static int  nvme_do_io_stat = 0;
#endif
static int  nvme_msix_enable = 1;
static int  admin_sub_queue_size = 256;
static int  admin_cpl_queue_size = 256;
static int  io_sub_queue_size = 1024;
static int  io_cpl_queue_size = 1024;
static int  io_command_id_size = 1024;
static int  transfer_size = 128;
int 	forcerdy = 0;
static int  max_io_request = 1024-1;	 /** @todo - optimize */
static int  intr_coalescing_threshold = 0;  	/** Zero based value */
static int  intr_coalescing_time = 0;   	/** In 100 us count */
static int 	reset_card_on_timeout = 0;
#ifdef NVME_DEBUG
static int  nvme_idle_count = 0;
int nvme_dbg = NVME_DEBUG_ALL;
#endif
#define NVME_REL	"1.7"

/**
 * For Instance allocation.
 */
static DEFINE_MUTEX(nvme_idr_mutex);
static DEFINE_IDR(nvme_instance_idr);

static int nvme_intr_coalescing(struct nvme_dev *dev);


/**
 * @brief This function set max_io_request parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_set_max_req(const char *s, const struct kernel_param *kp)
{
		int req;

		if (sscanf(s, "0x%x", &req) != 1) {
		if (sscanf(s, "%d", &req) != 1)
				return (-EINVAL);
	}
	if ((req < 2) || (req > io_command_id_size))
			return (-EINVAL);

	max_io_request = req;
	DPRINTX("IO queue throttle changed to %d\n", max_io_request);
		return 0;
}

/**
 * @brief This function reports max_io_request parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_get_max_req(char *buffer, const struct kernel_param *kp)
{
	int result = 0;

	result = sprintf(buffer, "%d", max_io_request);
	return result;
}

static struct kernel_param_ops nvme_ops_max_req = {
	.set = nvme_set_max_req,
	.get = nvme_get_max_req,
};

/**
 * @brief This function set intr_coalescing_threshold parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_set_intr_thresh(const char *s, const struct kernel_param *kp)
{
		int req;

		if (sscanf(s, "0x%x", &req) != 1) {
		if (sscanf(s, "%d", &req) != 1)
				return (-EINVAL);
	}

	if ((req < 0) || (req > 127))
			return (-EINVAL);

	intr_coalescing_threshold = req;
	DPRINTX("Interrupt coalescing changed to %d\n",
					intr_coalescing_threshold);
	return(0);
}

/**
 * @brief This function reports intr_coalescing_threshold parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_get_intr_thresh(char *buffer, const struct kernel_param *kp)
{
	int result = 0;

	result = sprintf(buffer, "%d", intr_coalescing_threshold);
	return result;
}

static struct kernel_param_ops nvme_ops_intr_thresh = {
	.set = nvme_set_intr_thresh,
	.get = nvme_get_intr_thresh,
};

/**
 * @brief This function set intr_coalescing_time parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_set_intr_time(const char *s, const struct kernel_param *kp)
{
		int req;

		if (sscanf(s, "0x%x", &req) != 1) {
		if (sscanf(s, "%d", &req) != 1)
				return (-EINVAL);
	}

	if ((req < 0) || (req > 127))
			return (-EINVAL);

	intr_coalescing_time = req;
	DPRINTX("Interrupt coalescing timer changed to %d\n",
					intr_coalescing_time);
	return(0);
}

/**
 * @brief This function reports intr_coalescing_time parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_get_intr_time(char *buffer, const struct kernel_param *kp)
{
	int result = 0;

	result = sprintf(buffer, "%d", intr_coalescing_time);
	return result;
}

static struct kernel_param_ops nvme_ops_intr_time = {
	.set = nvme_set_intr_time,
	.get = nvme_get_intr_time,
};

/**
 * @brief sets the flag to reset card on timeout
 * driver module parameter function
 */
static int
nvme_set_reset_on_timeout(const char *s, const struct kernel_param *kp)
{
	int val;

	if (sscanf(s, "0x%x", &val) != 1) {
		if (sscanf(s, "%d", &val) != 1) {
			printk(KERN_INFO "Couldn't parse reset param %s\n", s);
			return (-EINVAL);
		}
	}

	reset_card_on_timeout = val;
	DPRINTX("Reset card on timeout changed: 0x%08x\n",
			reset_card_on_timeout);
	return 0;
}


/**
 * @brief This function reports nvme_dbg parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_get_reset_on_timeout(char *buffer, const struct kernel_param *kp)
{
	int result = 0;

	result = sprintf(buffer, "0x%08x (%d)",
		reset_card_on_timeout, reset_card_on_timeout);
	return result;
}

static struct kernel_param_ops nvme_ops_rot = {
	.set = nvme_set_reset_on_timeout,
	.get = nvme_get_reset_on_timeout,
};


#ifdef NVME_DEBUG
/**
 * @brief This function set nvme_dbg parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_set_dbg(const char *s, const struct kernel_param *kp)
{
		int dbg;

		if (sscanf(s, "0x%x", &dbg) != 1) {
		if (sscanf(s, "%d", &dbg) != 1)
				return (-EINVAL);
	}

	nvme_dbg = dbg;
	DPRINTX("Debug level changed to 0x%08x\n", nvme_dbg);
		return 0;
}

/**
 * @brief This function reports nvme_dbg parameter
 * Dynamic driver parameters functions.
 */
static int
nvme_get_dbg(char *buffer, const struct kernel_param *kp)
{
	int result = 0;

	result = sprintf(buffer, "0x%08x (%d)", nvme_dbg, nvme_dbg);
	return result;
}

static struct kernel_param_ops nvme_ops_dbg = {
	.set = nvme_set_dbg,
	.get = nvme_get_dbg,
};
#endif



#ifdef NVME_DEBUG
module_param_cb(nvme_dbg, &nvme_ops_dbg, &nvme_dbg, 0664);
#endif
module_param_cb(intr_coalescing_threshold, &nvme_ops_intr_thresh,
					&intr_coalescing_threshold, 0664);
module_param_cb(intr_coalescing_time, &nvme_ops_intr_time,
					&intr_coalescing_time, 0664);
module_param_cb(max_io_request, &nvme_ops_max_req, &max_io_request, 0664);
module_param_cb(reset_card_on_timeout, &nvme_ops_rot,
				&reset_card_on_timeout, 0664);
module_param(nvme_major, int, 0664);
module_param(nvme_cmajor, int, 0664);
#if DO_IO_STAT
module_param(nvme_do_io_stat, int, 0664);
#endif
module_param(nvme_msix_enable, int, 0664);
module_param(admin_cpl_queue_size, int, 0444);
module_param(admin_sub_queue_size, int, 0444);
module_param(io_cpl_queue_size, int, 0444);
module_param(io_sub_queue_size, int, 0444);
module_param(io_command_id_size, int, 0444);
module_param(transfer_size, int, 0444);
module_param(forcerdy, int, 0);

MODULE_PARM_DESC(nvme_major,
	" NVMe Block Device Major number default 0");
MODULE_PARM_DESC(nvme_cmajor,
	" NVMe Control Device Major number default 0");
#if DO_IO_STAT
MODULE_PARM_DESC(nvme_do_io_stat,
	" NVMe Block Device Enable/Disable IO Statistic default 0");
#endif
MODULE_PARM_DESC(nvme_msix_enable,
	" NVMe Block Device Enable/Disable MSIX support");
MODULE_PARM_DESC(admin_cpl_queue_size,
	" NVMe maximum number of IO queue entries [1024]");
MODULE_PARM_DESC(admin_sub_queue_size,
	" NVMe number of Admin submission queue entries [1024]");
MODULE_PARM_DESC(io_cpl_queue_size,
	" NVMe number of Admin completion queue entries [1024]");
MODULE_PARM_DESC(io_sub_queue_size,
	" NVMe number of IO submission queue entries [1024]");
MODULE_PARM_DESC(io_command_id_size,
	" Number of command IDs per submission queue [1024]");
MODULE_PARM_DESC(transfer_size,
	" Maximum IO request Xfer size in 1K bytes maximum 1024 [128]");
MODULE_PARM_DESC(forcerdy,
	" Force driver to ignore stuck RDY bit");
MODULE_PARM_DESC(max_io_request,
	" IO Queue high water marker throttle [1024]");
MODULE_PARM_DESC(intr_coalescing_threshold,
	" Interrupt coalescing IO request completion queue entry count [6]");
MODULE_PARM_DESC(nvme_intr_coalescing_time,
	" Interrupt coalescing IO request completion queue timer [4]");
#ifdef NVME_DEBUG
MODULE_PARM_DESC(nvme_dbg,
	" Driver NVME_DEBUG print level minimum 0 maximum 4 [0]");
#endif
MODULE_PARM_DESC(reset_card_on_timeout,
	" Driver will reset the card if needed on timeout [flag]");

static int nvme_offline_ns(struct nvme_dev *dev, int ns_id);
static int nvme_online_ns(struct nvme_dev *dev, int ns_id);
static void nvme_free_ns(struct nvme_dev *dev, struct ns_info * ns);
static void nvme_flush_io_queues(struct nvme_dev *dev, struct ns_info *ns,
							int status);
static int nvme_cache_enable(struct nvme_dev *dev, u16 enable);
static int nvme_cache_check(struct nvme_dev *dev);
static int nvme_get_ctrl_identify(struct nvme_dev *dev);
static int nvme_hw_shutdown(struct nvme_dev *dev);
static char * nvme_pci_id(struct pci_dev * pci_dev);
static char * nvme_pci_info(struct pci_dev * pci_dev);
static void nvme_flush_admin(struct  queue_info *qinfo);
static struct ns_info *nvme_alloc_ns(struct nvme_dev *dev, int ns_id, int lock);
static int nvme_attach_ns(struct nvme_dev *dev, struct ns_info *ns, int lock);

/* sysfs interface */
/**
 * @brief This function shows number of Namespace configured.
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
ns_count_show(struct device *class_dev, struct device_attribute *attr,
								char *buf)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	if (dev)
			return sprintf(buf, "%u\n", dev->ns_count);
	return (-EINVAL);
}

static DEVICE_ATTR_RO(ns_count);

/**
 * @brief This function shows number of Namespace configured.
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
stat_show(struct device *class_dev, struct device_attribute *attr,
								char *buf)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	struct queue_info *qinfo;
	int i, len = 0;
	int max_req = 0,tot_req = 0, tot_act = 0;

	if (dev) {

			for (i = 1; i <= dev->que_count; i++) {

				qinfo = dev->que_list[i];
				if (! qinfo)
				continue;

		sprintf(&buf[len], "%u:%u:%u   ", qinfo->max_req,
					qinfo->nr_req, qinfo->nr_act);
		tot_req += qinfo->nr_req;
		tot_act += qinfo->nr_act;
		len = strlen(buf);

			if (max_req < qinfo->max_req)
			max_req = qinfo->max_req;
			qinfo->max_req = 0;
		}
		sprintf(&buf[len], "   SUM %u:%u   MAX-Qued %u\n",
						tot_req, tot_act, max_req);
		max_req = 0;
			return strlen(buf);
	}
	return (-EINVAL);
}

static DEVICE_ATTR_RO(stat);

/**
 * @brief This function shows controller PCI BUS address information
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
pci_id_show(struct device *class_dev, struct device_attribute *attr,
								char *buf)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	if (dev) {

		sprintf(buf, "%s\n", nvme_pci_id(dev->pci_dev));
			return strlen(buf);
	}
	return (-EINVAL);
}

static DEVICE_ATTR_RO(pci_id);

/**
 * @brief This function shows offline Namespaces.
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
hot_remove_show(struct device *class_dev, struct device_attribute *attr,
								char *buf)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	struct ns_info *ns;
	int found = 0;

	DPRINT1("dev %p, buf %s\n", class_dev, buf);
	if (!dev)
		return (-EINVAL);
	sprintf(buf, "Offline NS: ");
	dev->lock_func(&dev->lock, &dev->lock_flags);
	list_for_each_entry(ns, &dev->ns_list, list) {
		if (!(ns->flags & NS_ONLINE)) {
			DPRINT1("NS [%d], offline\n", ns->id);
			sprintf(&buf[strlen(buf)], "%d ", ns->id);
			found = 1;
		}
	}
	if (found) {
		 sprintf(&buf[strlen(buf)], "\n");
	} else {
		 sprintf(&buf[strlen(buf)], "none.\n");
	}
	DPRINT1("output %s", buf);
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	return (strlen(buf));
}

/**
 * @brief This function offlines a Namespace
 *  	A Namespace is offlined upon user request via "sysfs" or IOCTL.
 *  	Disk node is removed and ns_info is flagged offline.
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 * @param[in] size_t count size of input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
hot_remove_store(struct device *class_dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	int ns_id, status;

	DPRINT1("dev %p, count %lld, buf %s\n",
					class_dev, (u64)count, buf);
	if (!dev)
		return (-EINVAL);
		if (1 != sscanf(buf, "%d", &ns_id)) {
		EPRINT("Invalid input %s\n", buf);
			return (-EINVAL);
	}
	DPRINT1("Namespace %d\n", ns_id);
	if ((status = nvme_offline_ns(dev, ns_id)))
			return status;
	return count;
}

static DEVICE_ATTR_RW(hot_remove);

/**
 * @brief This function shows online Namespace Ids.
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
hot_add_show(struct device *class_dev, struct device_attribute *attr,
								char *buf)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	struct ns_info *ns;
	int found = 0;

	DPRINT1("dev %p, buf %s\n", class_dev, buf);
	if (!dev)
		return (-EINVAL);
	sprintf(buf, "Online NS: ");
	dev->lock_func(&dev->lock, &dev->lock_flags);
	list_for_each_entry(ns, &dev->ns_list, list) {
		if (ns->flags & NS_ONLINE) {
			DPRINT1("NS [%d], online\n", ns->id);
			sprintf(&buf[strlen(buf)], "%d ", ns->id);
			found = 1;
		}
	}
	if (found) {
		 sprintf(&buf[strlen(buf)], "\n");
	} else {
		 sprintf(&buf[strlen(buf)], "none.\n");
	}
	DPRINT1("output %s", buf);
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	return (strlen(buf));
}


/**
 * @brief This function onlines a Namespace
 *  	A Namespace is onlined upon user request via "sysfs" or IOCTL.
 *  	Disk node is removed and ns_info is flagged offline.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] int ns_id Namespace ID.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
hot_add_store(struct device *class_dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	int ns_id, status;

	DPRINT1("dev %p, count %lld, buf %s\n",
					dev, (u64)count, buf);
	if (!dev)
		return (-EINVAL);
		if (sscanf(buf, "%d", &ns_id) != 1) {
		EPRINT("Invalid input %s\n", buf);
			return (-EINVAL);
	}
	DPRINT1("Namespace %d\n", ns_id);
	if ((status = nvme_online_ns(dev, ns_id)))
			return status;
	return count;
}

static DEVICE_ATTR_RW(hot_add);

/**
 * @brief This function shows device cache setting.
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
nvme_cache_show(struct device *class_dev, struct device_attribute *attr,
								char *buf)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	int enable;

	enable = nvme_cache_check(dev);
		return sprintf(buf, "%u\n", enable);
}


/**
 * @brief This function shows number of Namespace configured.
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
nvme_cache_store(struct device *class_dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	int enable;

		if ((sscanf(buf, "%d", &enable) != 1) || (enable & ~1)) {
		EPRINT("Invalid input %s\n", buf);
			return (-EINVAL);
	}
	if (nvme_cache_enable(dev, enable))
		return -EIO;
	return count;
}

static DEVICE_ATTR_RW(nvme_cache);

/**
 * @brief This function shuts down NVME controller
 *
 * @param[in] struct device *class_dev pointer to class device structure.
 * @param[in] struct device_attribute *attr, pointer to class attribute
 * @param[in] cnst char *buf ,pointer to user input string.
 *
 * @return This function returns ssize_t buffer character count
 */
static ssize_t
nvme_shutdown_store(struct device *class_dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct nvme_dev *dev = dev_get_drvdata(class_dev);
	int enable;

		if ((sscanf(buf, "%d", &enable) != 1) || (enable & ~1)) {
		EPRINT("Invalid input %s\n", buf);
			return (-EINVAL);
	}
	if (nvme_hw_shutdown(dev))
		return -EIO;
	return count;
}

static DEVICE_ATTR_WO(nvme_shutdown);

static struct attribute *nvme_sysfs_default_attrs[] = {
	&dev_attr_ns_count.attr,
	&dev_attr_stat.attr,
	&dev_attr_pci_id.attr,
	&dev_attr_hot_remove.attr,
	&dev_attr_hot_add.attr,
	&dev_attr_nvme_cache.attr,
	&dev_attr_nvme_shutdown.attr,

	NULL
};

ATTRIBUTE_GROUPS(nvme_sysfs_default);

/**
 * Function definitions.
 */
// @todo static int __devinit nvme_probe(struct pci_dev *pci_dev,);
static int nvme_open(struct inode *inode, struct file *file);
static int nvme_close(struct inode *inode, struct file *file);
static int nvme_get_geo(struct block_device *bdev, struct hd_geometry *geo);
static long nvme_admin_ioctl(struct file *file, uint cmd, ulong arg);
static int nvme_ioctl_common(struct nvme_dev *dev, struct ns_info *ns,
					fmode_t mode, uint cmd, ulong arg);
static int nvme_probe(struct pci_dev *pci_dev,
					const struct pci_device_id *id);
// @todo static void __devexit nvme_remove(struct pci_dev *pci_dev);
static void nvme_remove(struct pci_dev *pci_dev);
static int nvme_open_disk(struct block_device *bdev, fmode_t mode);
typedef void	rel_disk_t;
static rel_disk_t nvme_release_disk(struct gendisk *gendisk, fmode_t mode);
static int nvme_ioctl(struct block_device *bdev, fmode_t mode,
					uint cmd, ulong arg);
static int nvme_request_irq(struct nvme_dev *dev, struct queue_info *qinfo,
					const char *name, int cpu);
static void nvme_release_irq(struct nvme_dev *dev, struct queue_info *qinfo);
static void nvme_spin_lock(spinlock_t *lock, ulong *flags);
static void nvme_spin_unlock(spinlock_t *lock, ulong *flags);
static int nvme_assign_ivec(struct nvme_dev *dev, u32 qid, int *ishared);
int nvme_send_cmd(struct sub_queue_info *sqinfo,
					struct cmd_info *cmd_info);
static int nvme_wait_cmd(struct queue_info *qinfo,
					struct cmd_info *cmd_info, int timeout);
static void nvme_process_prps(struct queue_info *qinfo,
					struct cmd_info *cmd_info);
static void nvme_process_cong(struct queue_info *qinfo);
static void nvme_process_work(struct work_struct *work);
static blk_qc_t nvme_make_request(struct request_queue *q, struct bio *bio);
void nvme_put_cmd(struct queue_info *qinfo, struct cmd_info *cmd_info);
static int nvme_request_queues(struct nvme_dev *dev, u32 *nr_io_queues);
static struct cmd_info * nvme_get_cmd(struct queue_info *qinfo);
static void nvme_delete_io_queues(struct nvme_dev *dev, int resource);
void inline nvme_memcpy_64(void *dst, void *src, int cnt);
void inline nvme_memset_64(void *dst, u64 val, int cnt);
struct queue_info * get_nvmeq(struct nvme_dev *dev);
static void nvme_get_cpu(spinlock_t *lock, ulong *flags);
static void nvme_put_cpu(spinlock_t *lock, ulong *flags);
static int nvme_validate_params(struct nvme_dev *dev);
static void nvme_suspend_io_queues(struct nvme_dev *dev, u32 new_id);
static int nvme_hw_reset(struct nvme_dev *dev);
static int nvme_hw_stop(struct nvme_dev *dev, int wait);
static int nvme_submit_request(struct queue_info *qinfo, struct ns_info *ns,
					struct bio *bio, int retries);
#if SEND_AEN
static int nvme_send_aen(struct nvme_dev *dev);
#endif
static void nvme_process_aen(struct queue_info *qinfo);
static int nvme_wait_event(struct nvme_dev *dev,  void __user *p);
static void nvme_get_error_log(struct nvme_dev *dev, struct cmd_info *cmd_info);
static void nvme_report_log(struct queue_info *qinfo, int page_id,
					void *log_page);
static void nvme_report_error(struct nvme_dev *dev, void *log);

static pci_ers_result_t
nvme_error_detected(struct pci_dev *pci_dev, enum pci_channel_state error)
{
	struct nvme_dev *dev = (struct nvme_dev *)pci_get_drvdata(pci_dev);
	int result = PCI_ERS_RESULT_RECOVERED;

	printk(KERN_INFO "PCIe error detected ...... %s %s",
			nvme_pci_id(pci_dev), nvme_pci_info(pci_dev));
	if (dev) {
		if (nvme_hw_reset(dev))
		result = PCI_ERS_RESULT_NEED_RESET;
	}
	return (result);
}

static pci_ers_result_t
nvme_dump_registers(struct pci_dev *pci_dev)
{
	int result = PCI_ERS_RESULT_RECOVERED;

	printk(KERN_INFO "PCIe mmio enable detected ...... %s %s",
			nvme_pci_id(pci_dev), nvme_pci_info(pci_dev));

#ifdef CONFIG_HW_RESET_ON_ERR
	{
		struct nvme_dev *dev = (struct nvme_dev *)pci_get_drvdata(pci_dev);
		if (dev) {
			if (nvme_hw_reset(dev))
			result = PCI_ERS_RESULT_NEED_RESET;
		}
	}
#endif
	return (result);
}

static pci_ers_result_t
nvme_link_reset(struct pci_dev *pci_dev)
{
	int result = PCI_ERS_RESULT_RECOVERED;

	printk(KERN_INFO "PCIe link reset ...... %s %s",
			nvme_pci_id(pci_dev), nvme_pci_info(pci_dev));

#ifdef CONFIG_HW_RESET_ON_ERR
	{
		struct nvme_dev *dev = (struct nvme_dev *)pci_get_drvdata(pci_dev);
		if (dev) {
			if (nvme_hw_reset(dev))
			result = PCI_ERS_RESULT_NEED_RESET;
		}
	}
#endif
	return (result);
}

static pci_ers_result_t
nvme_slot_reset(struct pci_dev *pci_dev)
{
	int result = PCI_ERS_RESULT_RECOVERED;

	printk(KERN_INFO "PCIe slot reset ...... %s %s",
			nvme_pci_id(pci_dev), nvme_pci_info(pci_dev));

#ifdef CONFIG_HW_RESET_ON_ERR
	{
		struct nvme_dev *dev = (struct nvme_dev *)pci_get_drvdata(pci_dev);
		if (dev) {
			if (nvme_hw_reset(dev))
			result = PCI_ERS_RESULT_NEED_RESET;
		}
	}
#endif
	return (result);
}

static void
nvme_error_resume(struct pci_dev *pci_dev)
{

	printk(KERN_INFO "PCIe error resume ...... %s %s",
			nvme_pci_id(pci_dev), nvme_pci_info(pci_dev));
#ifdef CONFIG_HW_RESET_ON_ERR
	{
		struct nvme_dev *dev = (struct nvme_dev *)pci_get_drvdata(pci_dev);
		if (dev)
			nvme_hw_reset(dev);
	}
#endif
}


/**
 * These are PCI Bus error recovery procedures. We really cannot do much
 * if PCIe link goes down. the most we can do it to reset the controller
 * and hope that it recovers.
 */
static struct pci_error_handlers nvme_err_handler = {
	.error_detected = nvme_error_detected,
	.mmio_enabled   = nvme_dump_registers,
	.link_reset = nvme_link_reset,
	.slot_reset = nvme_slot_reset,
	.resume 	= nvme_error_resume,
};

#ifdef CONFIG_PM
/**
 * Power Management
 */
static int
nvme_suspend(struct pci_dev *pci_dev, pm_message_t state)
{
		u32 device_state;
		struct nvme_dev *dev = pci_get_drvdata(pci_dev);

		device_state=pci_choose_state(pci_dev, state);

		printk(KERN_INFO "%s%d: pci-suspend: pci_dev=0x%p, slot %s, "
				"state [D%d]\n", DEV_NAME,
		dev->instance, pci_dev, pci_name(pci_dev), device_state);

		pci_save_state(pci_dev);

		/** put ioc into SHUTDOWN STATE */
		dev->state = NVME_STATE_SUSPEND;
		if(nvme_hw_stop(dev, NVME_DELAY)) {
				printk(KERN_INFO "%s%d: "
				"pci-suspend:  Failed to halt controller!\n",
		DEV_NAME, dev->instance);
		}

		pci_disable_device(pci_dev);
		pci_set_power_state(pci_dev, device_state);

		return 0;
}

int
nvme_resume(struct pci_dev *pci_dev)
{
		struct nvme_dev *dev = pci_get_drvdata(pci_dev);
		u32 device_state = pci_dev->current_state;
		int result;

		printk(KERN_INFO "%s%d: "
			"pci-resume: pci_dev=0x%p, slot %s, state [D%d]\n",
				DEV_NAME, dev->instance,
		pci_dev, pci_name(pci_dev), device_state);

		pci_set_power_state(pci_dev, 0);
		pci_restore_state(pci_dev);
		if ((result = pci_enable_device(pci_dev))) {

				printk(KERN_INFO "%s%d: "
						"pci-resume: failed PCI enable, error:[%x]\n",
						DEV_NAME, dev->instance, result);
			dev->state = NVME_STATE_FAILED;
	}

		/* bring controler to operational state */
		if ((result = nvme_hw_reset(dev))) {
				printk(KERN_INFO "%s%d: "
						"pci-resume: Cannot recover, error:[%x]\n",
						DEV_NAME, dev->instance, result);
			dev->state = NVME_STATE_FAILED;
		} else {
				printk(KERN_INFO "%s%d: pci-resume: success\n",
			DEV_NAME, dev->instance);
			dev->state = NVME_STATE_OPERATIONAL;
		}
		return 0;
}
#endif


/**
 * Power Management - Shutdown
 */
static void
nvme_shutdown(struct pci_dev *pci_dev)
{
		struct nvme_dev *dev = pci_get_drvdata(pci_dev);

	printk(KERN_ERR "%s%d: pci-shutdown: slot %s\n",
			DEV_NAME, dev->instance, pci_name(pci_dev));

		if(nvme_hw_shutdown(dev)) {
				printk(KERN_ERR "%s%d: "
					"pci-shutdown:  Failed to shutdown controller!\n",
			DEV_NAME, dev->instance);
		}
	dev->lock_func(&dev->lock, &dev->lock_flags);
	nvme_flush_io_queues(dev, NULL, -EIO);  /* flush all IO requests */
	dev->unlock_func(&dev->lock, &dev->lock_flags);
}

#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )

static DEFINE_PCI_DEVICE_TABLE(nvme_id_table) = {
	{PCI_DEVICE_CLASS(0x010802, 0xffffff)},
	{ 0, }
};

#else

static const struct pci_device_id nvme_id_table[] = {
	{PCI_DEVICE_CLASS(0x010802, 0xffffff)},
	{ 0, }
};

#endif

MODULE_DEVICE_TABLE(pci, nvme_id_table);

static struct pci_driver nvme_driver = {
	.name   	= DRV_NAME,
	.id_table   = nvme_id_table,
	.probe  	= nvme_probe,
	.remove 	= nvme_remove,
#ifdef CONFIG_PM
	.suspend	= nvme_suspend,
	.resume 	= nvme_resume,
#endif
	.shutdown   = nvme_shutdown,
	.err_handler	= &nvme_err_handler
};

static const struct block_device_operations nvme_fops = {
	.open   	= nvme_open_disk,
	.release	= nvme_release_disk,
	.owner  		= THIS_MODULE,
	.ioctl  		= nvme_ioctl,
/*  .compat_ioctl   = nvme_ioctl,  @todo	compat_ioctl   */
	.getgeo 	= nvme_get_geo,
};

static const struct file_operations nvme_chrdev_fops = {
	.owner  	= THIS_MODULE,
	.open   	= nvme_open,
	.release	= nvme_close,
	.unlocked_ioctl = nvme_admin_ioctl,
	.compat_ioctl   = nvme_admin_ioctl,
/* @todo	.flush  = nvme_flush_cache, */
};


#define MAX_CONTROLLER  ((1U << MINORBITS) / MAX_MINOR_DEV)

static int
nvme_open_disk(struct block_device *bdev, fmode_t mode)
{
	struct gendisk *disk = bdev->bd_disk;
	struct ns_info *ns = disk->private_data;
	struct nvme_dev *dev = ns->dev;
	int result = 0;

	if (mode & FMODE_WRITE) {
		result = nvme_rms_wait_charge(dev, mode & FMODE_NDELAY);
		if (result)
			return result;
	}

	dev->lock_func(&dev->lock, &dev->lock_flags);
	DPRINT("dev %p, disk %p, ns %d, ref_count %d\n",
					dev, disk, ns->id, ns->ref_count);
	if (ns->flags & NS_ONLINE) {
		++ns->ref_count;
	} else {
		result = -EBADF;
	}
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	DPRINT("dev %p, disk %p, ref_count %d, result %d\n",
					dev, disk, ns->ref_count, result);

	return result;
}

static rel_disk_t
nvme_release_disk(struct gendisk *disk, fmode_t mode)
{
	struct ns_info *ns = disk->private_data;
	struct nvme_dev *dev = ns->dev;
	int result = 0;

	dev->lock_func(&dev->lock, &dev->lock_flags);
	DPRINT("dev %p, disk %p, ns %d, ref_count %d\n",
					dev, disk, ns->id, ns->ref_count);
	if (ns->ref_count) {
		--ns->ref_count;
	} else {
		result = -EBADF;
	}
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	DPRINT("dev %p, disk %p, ref_count %d, result %d\n",
					dev, disk, ns->ref_count, result);

}

static int
nvme_open(struct inode *inode, struct file *file)
{
	struct nvme_dev *dev;

	DPRINTX("minor %d, file %p\n", iminor(inode), file);
	dev = idr_find(&nvme_instance_idr, iminor(inode));
		if (!dev)
			return -ENODEV;

	DPRINTX("dev %p, minor %d, file %p\n", dev, iminor(inode), file);
	file->private_data = dev;
		return 0;
}

static int
nvme_close(struct inode *inode, struct file *file)
{
#ifdef NVME_DEBUG
	struct nvme_dev *dev = file->private_data;
#endif
	DPRINTX("dev %p, minor %d, file %p\n", dev, iminor(inode), file);
	file->private_data = NULL;

	return 0;
}

/**
 * @brief This function reports driver geometry information
 *
 * @param[in] bdev Pointer to block device data structure
 * @param[in] geo Pointer to hard drive geometry data structure
 *
 * @return This function always returns 0
 */
static int nvme_get_geo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct gendisk *disk = bdev->bd_disk;
	struct ns_info *ns = disk->private_data;

	geo->heads = 128;
	geo->sectors = 32;
	geo->cylinders = (ns->block_count << (ns->lba_shift - 9)) /
		(geo->heads * geo->sectors);
	return 0;
}

/**
 * @brief This function maps user pages into kernel memory
 *
 * @param[in] dev Pointer NVME device context
 * @param[in] uio Pointer to user request
 * @param[in] cmd_info Pointer to Command Information Block
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note: We have blocked meta data and fail any request with non-zero length.
 */
static int
nvme_map_user_pages(struct nvme_dev *dev, struct usr_io *uio,
						struct cmd_info *cmd_info)
{
	int i, status, count, nents, offset, length;
	struct page **pages;

	DPRINT8("mapping uio addr 0x%llx, len %d\n", uio->addr, uio->length);

	/**
	 * Validate data transfer length.
	 * @todo - Meta data is not currently supported.
	 */
	if (((ulong)uio->addr & 3) || (!uio->length) ||
		(uio->length > (transfer_size * 1024)) || uio->meta_length) {
		return -EINVAL;
	}
	offset = offset_in_page(uio->addr);
	count = DIV_ROUND_UP(offset + uio->length, PAGE_SIZE);
	pages = kcalloc(count, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	DPRINT8("offset %d, page count %d, pages %p\n", offset, count, pages);

	down_read(&current->mm->mmap_sem);
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
	status = get_user_pages(current, current->mm, uio->addr & PAGE_MASK,
				count, 1, 0, pages, NULL);
#else
	status = get_user_pages(uio->addr & PAGE_MASK, count, FOLL_WRITE, pages, NULL);

#endif
	up_read(&current->mm->mmap_sem);
	DPRINT8("user pages count %d, status 0x%x\n", count, status);

	if (status != count) {
		count = status;
		status = -EFAULT;
		goto put_pages;
	}

	sg_init_table(cmd_info->sg_list, count);
	length = uio->length;
	for (i = 0; i < count; i++) {
		sg_set_page(&cmd_info->sg_list[i], pages[i],
			min_t(int, length, PAGE_SIZE-offset), offset);
		length -= (PAGE_SIZE - offset);
		offset = 0;
	}
	sg_mark_end(&cmd_info->sg_list[i - 1]);

	nents = dma_map_sg(dev->dma_dev, cmd_info->sg_list, count,
				(XFER_FROM_DEV == uio->direction ?
					DMA_FROM_DEVICE : DMA_TO_DEVICE));
	DPRINT8("dma #ent %d\n", nents);
	if (!nents) {
		status = -ENOMEM;
		goto put_pages;
	}

	DPRINT8("kfree pages %p\n", pages);
	kfree(pages);
	return nents;

put_pages:
	for (i = 0; i < count; i++)
		put_page(pages[i]);
	kfree(pages);
	return status;
}

/**
 * @brief This function unmaps user data from kernel address space.
 *
 * @param[in] dev Pointer NVME device context
 * @param[in] uio Pointer to user request
 * @param[in] cmd_info Pointer to Command Information Block
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
void
nvme_unmap_user_pages(struct nvme_dev *dev, struct usr_io *uio,
				struct cmd_info *cmd_info)
{
	int i, count;

	count = DIV_ROUND_UP(offset_in_page(uio->addr) +
						uio->length, PAGE_SIZE);
	DPRINT8("user data pages %d\n", count);
	dma_unmap_sg(dev->dma_dev, cmd_info->sg_list, count,
				(XFER_FROM_DEV == uio->direction ?
					DMA_FROM_DEVICE : DMA_TO_DEVICE));
	for (i = 0; i < count; i++)
		put_page(sg_page(&cmd_info->sg_list[i]));
}


/**
 * @brief This function Validates user uio data structure.
 *  	 This function is called by IOCTL function to validate
 *  	 pass-through uio header and its content.
 *
 * @param[in] dev Pointer NVME device context
 * @param[in] void __user *p user address space uio header pointer
 * @param[in] struct usr_io *uio kernel address space uio header pointer
 * @param[in] Boolean usr_io flag to identify user IO request.
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_validate_uio(struct nvme_dev *dev, void __user *p, struct usr_io *uio,
							int usr_io)
{

	DPRINT8("Validating uio hdr %p, uio %p\n", p, uio);
	/**
	 * Read user space uio into kernel uio header.
	 */
	if (copy_from_user(uio, p, sizeof(* uio))) {
		DPRINT8("access violation user uio hdr %p\n", p);
		return -EFAULT;
	}
	DPRINT8("uio %p, opcode %x, data len %d, meta len %d\n",
				uio, uio->cmd.header.opCode,
				uio->length, uio->meta_length);

	if (unlikely(usr_io &&
		((uio->cmd.header.opCode >= NVME_VNDR_CMD_IO_CODE_START) &&
				(uio->cmd.header.opCode <= NVME_VNDR_CMD_IO_CODE_END)))) {
		if(!dev->nvmVendCmdCfg) {
		DPRINT8("Firmware does not support Vendor Specific\n");
		return (-ENOTTY);
		}
		if ((uio->length < uio->cmd.cmd.vendorSpecific.buffNumDW >> 2) ||
			(uio->meta_length <
						uio->cmd.cmd.vendorSpecific.metaNumDW >> 2)) {
		DPRINT8("length mismatch data %d, meta %d\n",
				uio->cmd.cmd.vendorSpecific.buffNumDW,
				uio->cmd.cmd.vendorSpecific.metaNumDW);
				return (-ENOTTY);
		}
	}

	/**
	 * Validate command buffer access.
	 */
	if (!access_ok(VERIFY_READ, uio->addr, uio->length)) {
		DPRINT8("Access violation command Ptr 0x%llx\n", uio->addr);
		return -EFAULT;
	}
	DPRINT8("uio %p, Access OK\n", uio);

	/**
	 * Validate data access.
	 */
	if (uio->length) {
		if (!access_ok((uio->direction == XFER_TO_DEV) ?
					VERIFY_WRITE : VERIFY_READ,
					uio->addr, uio->length)) {
		DPRINT8("Access violation data Ptr 0x%llx, length %d\n",
						uio->addr, uio->length);
		return (-EFAULT);
		}
		if (uio->length > (transfer_size * 1024)) {
		EPRINT("Request transfer length exceeds maximum allowed %d\n",
								uio->length);
		return (-EINVAL);
		}
	}
	DPRINT8("uio %p, addr %llx, len %d Access OK\n",
					uio, uio->addr, uio->length);
	/**
	 * Validate status buffer access.
	 */
	if (uio->meta_length) {
		if (!access_ok((uio->direction == XFER_TO_DEV) ?
					VERIFY_WRITE : VERIFY_READ,
					uio->meta_addr, uio->meta_length)) {
		DPRINT8("Access violation meta data Ptr 0x%llx, length %d\n",
					uio->meta_addr, uio->meta_length);
		return (-EFAULT);
		}
		if (uio->meta_length > PAGE_SIZE) {
		EPRINT("Request meta data length exceeds maxmimum allowed %d\n",
							uio->meta_length);
		return (-EINVAL);
		}
	}
	DPRINT8("uio %p, Meta addr 0x%llx, len %d Access OK\n",
				uio, uio->meta_addr, uio->meta_length);
	return 0;
}

/**
 * @brief This function updates user uio data structure.
 *  	This function is called by pass-through function to update
 *  	user uio data structure, updating data length, meta data
 *  	length, status and completion entry.
 *
 * @param[in] struct usr_io __user *puio Pointer to uio structure user address
 * @param[in] struct usr_io *uio Pointer to uio structure in kernel address
 * @param[in] struct cmd_info *cmd_info Pointer to command information block.
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 */
static int
nvme_put_uio(struct usr_io __user *puio, struct usr_io *uio,
						struct cmd_info *cmd_info)
{

	/**
	 * Update user uio data xfer count and status.
	 */
	put_user(uio->length, &puio->length);
	put_user(uio->meta_length, &puio->meta_length);
	put_user(uio->status, &puio->status);

	/**
	 * Copy Completion queue entry.
	 */
	if (copy_to_user(&puio->comp, &cmd_info->cq_entry,
						sizeof(struct cq_entry)))
		return -EFAULT;
	return 0;
}

/**
 * @brief This function checkd for list of disallowed user Admin requests.
 *     This function is called by IOCTL function to perform
 *     a check and valifdate that command opcode does not iterfear with
 *     driver operation.
 *
 * @param[in] struct usr_io *uio Pointer to uio structure in kernel space.
 *
 * @return This function returns int 0 if allowed, otherwise Error Code
 *
 * @note: ECN-23 requires us to check for vendor unique request and to
 *  	validate data length if supported.
 */
static int
nvme_allowed_admin_cmd(struct nvme_dev *dev, struct usr_io *uio)
{
	struct ns_info *ns;

	switch (uio->cmd.header.opCode) {

		case NVM_ADMIN_CMD_DEL_SQ:
		case NVM_ADMIN_CMD_CREATE_SQ:
		case NVM_ADMIN_CMD_DEL_CQ:
		case NVM_ADMIN_CMD_CREATE_CQ:
		case NVM_ADMIN_CMD_ABORT:
		case NVM_ADMIN_CMD_ASYNC_EVENT_REQ:
			DPRINT9("Disallowed Admin command 0x%x.\n",
					uio->cmd.header.opCode);
		return -ENOTTY;

		case NVM_ADMIN_CMD_FORMAT_NVM:
		{
			dev->lock_func(&dev->lock, &dev->lock_flags);
			list_for_each_entry(ns, &dev->ns_list, list) {
				DPRINT9("ns id %d command nsID %d flags %x\n",
					ns->id, uio->namespace, ns->flags);
				if (ns->id == uio->cmd.header.namespaceID) {
					if (ns->flags & NS_ONLINE) {
						dev->unlock_func(&dev->lock, &dev->lock_flags);
						DPRINT9("Disallowed Admin command Format 0x%x\n",
								uio->cmd.header.opCode);
						return -EINVAL;
					}
					DPRINT9("allowing Admin command Format 0x%x flags %x\n",
							uio->cmd.header.opCode, ns->flags);
					break;
				}
			}
			dev->unlock_func(&dev->lock, &dev->lock_flags);
			return 0;
		}
		default:
		if ((uio->cmd.header.opCode & NVME_VNDR_CMD_ADM_CODE_START) ==
						NVME_VNDR_CMD_ADM_CODE_START)
		{
			if (!dev->admVendCmdCfg) {
				DPRINT9("Vendor Specific command not supported.\n");
				return -ENOTTY;
			}
			if ((uio->length <
					uio->cmd.cmd.vendorSpecific.buffNumDW >> 2) ||
				(uio->meta_length <
					uio->cmd.cmd.vendorSpecific.metaNumDW >> 2)) {
				EPRINT("Vendor Specific data length mismatch in opcode %d. (%d < %d) o (%d < %d)\n",
					uio->cmd.header.opCode,
					uio->length, uio->cmd.cmd.vendorSpecific.buffNumDW >> 2,
					uio->meta_length, uio->cmd.cmd.vendorSpecific.metaNumDW >> 2);
				return -ENOTTY;
			}
			rms200_allowed_admin_cmd(dev, uio);
		}
		return 0;
	}
}


/**
 * @brief This function process user Admin request.
 *     This function is called by IOCTL function to perform
 *     a user requested Admin command.
 *
 * @param[in] struct nvme_dev *dev Pointer to NVME device control block
 * @param[in] void __user *p Pointer to uio structure in user address space.
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note: ECN-23 requires us to check for vendor unique request and to
 *  	validate data length if supported.
 */
static int
nvme_admin_passthru(struct nvme_dev *dev, void __user *p)
{
	struct usr_io __user *puio = p;
	struct cmd_info *cmd_info = NULL;
	struct queue_info *qinfo;
	struct usr_io *uio;
	struct usr_io ioctl_uio;
	int result = 0;

	uio = &ioctl_uio;

	if(NVME_STATE_OPERATIONAL != dev->state)
	{
		DPRINT9("Device state %d is not operational\n",dev->state);
		return -EINVAL;
	}

	if (nvme_validate_uio(dev, puio, uio, FALSE)) {
		DPRINT9("Failed validation %p\n", puio);
		return -EFAULT;
	}
	if (nvme_allowed_admin_cmd(dev, uio))
		return -ENOTTY;

	qinfo = dev->admin_queue;
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	cmd_info = nvme_get_cmd(dev->admin_queue);
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	if (!cmd_info) {
		DPRINT9("Out of Cmd_Info data %p\n", dev->admin_queue);
		return (-ENOMEM);
	}
	nvme_memcpy_64(&cmd_info->nvme_cmd, &uio->cmd,
				sizeof(struct nvme_cmd)/sizeof(u64));
#ifdef NVME_DEBUG
	if (nvme_dbg & NVME_DEBUG_DUMP) {
		int i;
		u32 *ptr;
		ptr = (u32 *)&cmd_info->nvme_cmd;
		for (i=0; i<sizeof(struct nvme_cmd)/sizeof(u32); i += sizeof(u32)) {
			DPRINT("%02x: %08x %08x %08x %08x\n", i,
				   ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
		}
	}
#endif
	cmd_info->req   	= uio;
	cmd_info->type  	= ADMIN_CONTEXT;
	cmd_info->count 	= uio->length;
	cmd_info->nvme_cmd.header.cmdID = cmd_info->cmd_id;
	DPRINT9("command ID %d\n", cmd_info->cmd_id);

	/**
	 * Map user space and Create an scatter gather list of user data.
	 */
	if (uio->length) {
		if ((result = nvme_map_user_pages(dev, uio, cmd_info)) < 0)
			goto out;

		DPRINT9("mapped user pages cmd_info = %p\n", cmd_info);
		nvme_process_prps(qinfo, cmd_info);
	}
	result = nvme_wait_cmd(qinfo, cmd_info, uio->timeout * HZ);
	uio->status = result;


	DPRINT9("command completion result 0x%x, uio->status %d\n",
					result, uio->status);

	nvme_unmap_user_pages(dev, uio, cmd_info);
	result = nvme_put_uio(puio, uio, cmd_info);
out:
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	if (cmd_info && (cmd_info->type != ABORT_CONTEXT)) {
		if (cmd_info->status!=EINTR)/* call putcmd only if flush admin was not called*/
			nvme_put_cmd(qinfo, cmd_info);
	} else {
		dev->state = NVME_STATE_SUSPEND;
		dev->thread_flags |= THREAD_RESTART;
		wake_up(&dev->thread_wait);
	}
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	DPRINT9("result %d, uio status 0x%x\n", result, uio->status);

	if(uio->status)
		return (uio->status);
	return result;
}

/**
 * @brief This function Process user IO request.
 *  		This function is called by IOCTL function to perform
 *  		a user requested IO command.
 *
 * @param[in] struct nvme_dev *dev Pointer to NVME device control block
 * @param[in] void __user *p Pointer to "ioctl_uio" structure in user address.
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 */
static int
nvme_io_passthru(struct nvme_dev *dev, struct ns_info *ns, void __user *p)
{
	struct usr_io __user *puio = p;
	struct cmd_info *cmd_info = NULL;
	struct queue_info *qinfo;
	struct usr_io ioctl_uio;
	struct usr_io *uio;
	int tmp_result;
	int result = 0;

	uio = &ioctl_uio;

	if(NVME_STATE_OPERATIONAL != dev->state)
	{
		DPRINT10("Device state %d is not operational\n",dev->state);
		return -EINVAL;
	}

	if ((result = nvme_validate_uio(dev, puio, uio, TRUE))) {
		DPRINT10("Failed validation %p\n", puio);
		return (result);
	}
	qinfo = get_nvmeq(dev);
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	cmd_info = nvme_get_cmd(qinfo);
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	DPRINT10("io_pass qinfo %p num_of_active %d\n", qinfo,qinfo->nr_act);
	if (!cmd_info) {
		DPRINT10("Out of Cmd_Info data %p\n", qinfo);
		return (-ENOMEM);
	}
	nvme_memcpy_64(&cmd_info->nvme_cmd, &uio->cmd,
				sizeof(cmd_info->nvme_cmd)/sizeof(u64));
#ifdef NVME_DEBUG
	if (nvme_dbg & NVME_DEBUG_DUMP) {
			int i;
			u32 *ptr;
		ptr = (u32 *)&cmd_info->nvme_cmd;
		DPRINT("Command :\n");
		for (i=0; i<sizeof(struct nvme_cmd)/sizeof(u32); i += sizeof(u32)) {
		    DPRINT("%02x: %08x %08x %08x %08x\n", i,
			ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
		}
		}
#endif

	cmd_info->req   	= uio;
	cmd_info->type  	= ADMIN_CONTEXT;
	cmd_info->count 	= uio->length;

	cmd_info->nvme_cmd.header.namespaceID = ns ? ns->id : -1;
	cmd_info->nvme_cmd.header.cmdID = cmd_info->cmd_id;
	DPRINT10("command ID %d\n", cmd_info->cmd_id);

	/**
	 * Map user space and Create an scatter gather list of user data.
	 */
	if ((result = nvme_map_user_pages(dev, uio, cmd_info)) < 0)
		goto out;

	DPRINT10("mapped user pages %p\n", cmd_info);

	nvme_process_prps(qinfo, cmd_info);

	result = nvme_wait_cmd(qinfo, cmd_info, uio->timeout * HZ);
	uio->status = result;
	DPRINT10("command completion result 0x%x, uio->status 0x%x\n",
					result, uio->status);

	nvme_unmap_user_pages(dev, uio, cmd_info);

	tmp_result = nvme_put_uio(puio, uio, cmd_info);
	if (result == 0) {
		result = tmp_result;
	}
out:
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	if (cmd_info && (cmd_info->type != ABORT_CONTEXT)) {
		if (cmd_info->status!=EINTR)/* call putcmd only if flush admin was not called*/
				nvme_put_cmd(qinfo, cmd_info);
	} else {
		dev->state = NVME_STATE_SUSPEND;
		dev->thread_flags |= THREAD_RESTART;
		wake_up(&dev->thread_wait);
	}
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

	if(uio->status)
		return (uio->status);
	return result;
}


/**
 * @brief This function offlines a Namespace.
 *  	This function is called by IOCTL function of SysFS function
 *  	to offline a Namespace.
 *
 * @param[in] struct nvme_dev *dev Pointer to NVME device control block
 * @param[in] int ns_id Namespace ID
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 */
static int
nvme_offline_ns(struct nvme_dev *dev, int ns_id)
{
	struct ns_info *ns;
	int found = 0, result = 0;

	dev->lock_func(&dev->lock, &dev->lock_flags);
	list_for_each_entry(ns, &dev->ns_list, list) {
		if (ns->id == ns_id) {
			found = 1;
			if (!(ns->flags & NS_ONLINE)) {
				EPRINT("NS [%d], already removed\n", ns->id);
				result = -EINVAL;
			}
			if (ns->ref_count) {
				EPRINT("NS [%d], in use ref count %d\n",
							ns->id, ns->ref_count);
				result = -EBUSY;
			}
			break;
		}
	}

	if ((result == 0) && found ) {
		/*
		 * Mark it as offline for nvme_open_disk
		*/
		ns->flags &= ~NS_ONLINE;
		/*
		 * Mark it as being removed for online_ns
		*/
		ns->flags |= NS_TRANSIENT;
		nvme_flush_io_queues(dev, ns, -EIO);	/* Flush all requests */
	}
	dev->unlock_func(&dev->lock, &dev->lock_flags);

	if (!found) {
		EPRINT("Invalid request; NS %d Offline or Busy.\n", ns_id);
		result = -EINVAL;
	}
	if (result == 0) {
		NPRINT("NS [%d], Removing Disk \"%s\"\n",
						ns->id, ns->disk->disk_name);
		if (ns->disk->flags & GENHD_FL_UP)
			del_gendisk(ns->disk);
		put_disk(ns->disk);
		blk_cleanup_queue(ns->queue);
		ns->disk = NULL;
		ns->queue = NULL;

		dev->lock_func(&dev->lock, &dev->lock_flags);
		ns->flags &= ~NS_TRANSIENT;
		dev->unlock_func(&dev->lock, &dev->lock_flags);
	}

	return (result);
}



/**
 * @brief This function onlines a Namespace offlined or recently added.
 *  	This function is called by IOCTL function of SysFS function
 *  	to online a Namespace.
 *
 * @param[in] struct nvme_dev *dev Pointer to NVME device control block
 * @param[in] int ns_id Namespace ID
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 */
static int
nvme_online_ns(struct nvme_dev *dev, int ns_id)
{
	struct ns_info *ns, *tmp;
	int result = 0;

	DPRINT8("Namespace %d\n", ns_id);
	dev->lock_func(&dev->lock, &dev->lock_flags);
	list_for_each_entry_safe(ns, tmp, &dev->ns_list, list) {
		if (ns->id == ns_id) {
			if (ns->flags & (NS_ONLINE | NS_TRANSIENT)) {
				EPRINT("NS [%d], already Online or being removed\n", ns->id);
				result = -EINVAL;
				break;
			}
			DPRINT8("releasing NS [%d]\n", ns->id);
			nvme_free_ns(dev, ns);
			ns = NULL;
			break;
		}
	}
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	result = nvme_get_ctrl_identify(dev);
	if (result) {
		EPRINT("Failed to retrieve Identify data.\n");
		return -EIO;
	}
	dev->ns_count = dev->identify->numNmspc;
	rms_check_ns_count(dev);

	if (!ns) {
		DPRINT2("**** New Namespace %d discovered *******\n", ns_id);
		if (!(ns = nvme_alloc_ns(dev, ns_id, 1))) {
			EPRINT("Failed to allocate NS information structure.\n");
			result = -ENOMEM;
		}
#if USE_NS_ATTR
		/*
		 * Get Namespace attributes
		 */
		if (nvme_ns_attr(dev, ns_id, ns)) {
			EPRINT("Failed get NS attributes.\n");
			result = -EINVAL;
		}
#endif
		if (result == 0) {
			result = nvme_attach_ns(dev, ns, 1);
			if (result) {
				EPRINT("Failed to create disk device for NS %d\n", ns->id);
			}
		}
	}
	return (result);
}

static int nvme_ioctl_vernum =  ((IOCTL_MAJOR_FUNCTION << 16) |
				(IOCTL_MINOR_NUMBER << 8) | IOCTL_FUNCTION);

static long
nvme_admin_ioctl(struct file *file, uint cmd, ulong arg)
{
	struct nvme_dev *dev = file->private_data;

	switch (cmd) {
	case NVME_GET_VERSION_NUM:
	case NVME_IOCTL_ADMIN_CMD:
	case NVME_IOCTL_RESTART:
	case NVME_IOCTL_EVENT:
	case NVME_IOCTL_HOTREMOVE:
	case NVME_IOCTL_HOTADD:
	case NVME_IOCTL_SET_CACHE:
		break;

	default:
		DPRINT8("Failing unsupported admin command 0x%x (%d)\n", cmd, cmd);
		return -ENOTTY;
	}
	return(nvme_ioctl_common(dev, NULL, file->f_mode, cmd, arg));
}

/**
 * @brief This function is driver ioctl entry point.
 *  		This function is called by IOCTL function to perform
 *  		a user requested IO command.
 *
 * @param[in] struct block_device *bdev Pointer to Block device structure.
 * @param[in] fmode_t mode File mode.
 * @param[in] uint cmd ioctl command.
 * @param[in] unsigned long argument.
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 */
static int
nvme_ioctl(struct block_device *bdev, fmode_t mode, uint cmd, ulong arg)
{
	struct gendisk *disk = bdev->bd_disk;
	struct ns_info *ns = disk->private_data;
	struct nvme_dev *dev = ns->dev;

	DPRINT8("bdev %p, disk %p, ns %p [%d]\n", bdev, disk, ns, ns->id);

	switch (cmd) {
	case NVME_GET_VERSION_NUM:
	case NVME_IOCTL_IO_CMD:
	case NVME_IOCTL_ADMIN_CMD:
	case NVME_RMS_IOCTL_IO_CMD_ASYNC:
		break;

	case BLKFLSBUF:
	case 0x5331:		/* CDROM_GET_CAPABILITY */
	case BLKROSET:
		return 0;

	case BLKROGET:
	{
		int __user *ip =  (int __user *)arg;
		return put_user(1, ip);
	}

	/**
	 * The following case was added to allow Centos mkinitrd to work.
	 * it examines driver object for specific symbols and if matched,
	 * it adds it to list of block drivers.
	 */
	case 0x0BADC0DE:
		ata_scsi_ioctl((struct scsi_device *)bdev, (int)cmd, (void __user *)arg);
		break;
	default:
		DPRINT8("Failing unsupported IO command 0x%x (%d)\n", cmd, cmd);
		return -ENOTTY;
	}
	return (nvme_ioctl_common(dev, ns, mode, cmd, arg));
}

static int
nvme_ioctl_common(struct nvme_dev *dev, struct ns_info *ns,
				 fmode_t mode, uint cmd, ulong arg)
{
	void __user *p = (void __user *)arg;
	int __user *ip = p;
	struct usr_io __user *uiop = p;

	int ns_id, enable;

	DPRINT8("dev %p, ns %p [%d]\n", dev, ns, ns ? ns->id : -1);
	DPRINT8("cmd 0x%0x, arg %p\n", cmd, p);

	switch (cmd) {
	case NVME_GET_VERSION_NUM:
		return put_user(nvme_ioctl_vernum, ip);

	case NVME_IOCTL_ADMIN_CMD:
		return nvme_admin_passthru(dev, uiop);

	case NVME_IOCTL_IO_CMD:
	  return nvme_io_passthru(dev, ns, uiop);

	case NVME_IOCTL_RESTART:
		return nvme_hw_reset(dev);

	case NVME_IOCTL_SET_CACHE:
		if (get_user(enable, ip))
			return (-EFAULT);
		DPRINT8("Received cache control 0x%0x, arg %d\n", cmd, enable);
		return (nvme_cache_enable(dev, enable));

	case NVME_IOCTL_EVENT:
		DPRINT8("Received event control 0x%0x, arg %p\n", cmd, p);
		return(nvme_wait_event(dev, p));

	case NVME_IOCTL_HOTREMOVE:
		if (get_user(ns_id, ip))
			return (-EFAULT);
		return (nvme_offline_ns(dev, ns_id));

	case NVME_IOCTL_HOTADD:
		if (get_user(ns_id, ip))
			return (-EFAULT);
		return (nvme_online_ns(dev, ns_id));

	default:
		break;
	}
	return -ENOTTY;
}

#if DO_IO_STAT
/**
 * @brief This function handles disk statistic when a request congested.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] ns, pointer to Namespace information block
 * @param[in] struct bio *bio, the BIO request.
 *
 * @return none.
 */
static void inline
nvme_disk_stat_cong(struct nvme_dev *dev, struct ns_info *ns, struct bio *bio)
{

		struct hd_struct *part;
		int cpu, rw = bio_data_dir(bio);

		if (!nvme_do_io_stat)
			return;
#if ( (LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
		if (bio->bi_rw & BIO_REQ_DISCARD)
#else
		if (bio_op(bio) & BIO_REQ_DISCARD)
#endif
			return;

		cpu = part_stat_lock();

		part = disk_map_sector_rcu(bio->bi_bdev->bd_disk, bio->bi_iter.bi_sector);
		if (!hd_struct_try_get(part)) {
			/*
			 * The partition is already being removed,
			 * the request will be accounted on the disk only
			 *
			 * We take a reference on disk->part0 although that
			 * partition will never be deleted, so we can treat
			 * it as any other partition.
			 */
			part = &bio->bi_bdev->bd_disk->part0;
		hd_struct_get(part);
		}
		part_round_stats(cpu, part);
		part_inc_in_flight(part, rw);
	DPRINT2("bio %p, part %p\n", bio, bio->bi_private);

		part_stat_unlock();
}


/**
 * @brief This function handles disk statistic when a request is received.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct cmd_info *, pointer to Command information block.
 * @param[in] struct bio *bio, the BIO request.
 *
 *
 * @return none.
 */
static void inline
nvme_disk_stat_in(struct nvme_dev *dev, struct cmd_info *cmd_info,
							struct bio *bio)
{

		struct hd_struct *part;
		int cpu, rw = bio_data_dir(bio);

		if (!nvme_do_io_stat)
			return;
#if ( (LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
		if (bio->bi_rw & BIO_REQ_DISCARD)
#else
		if (bio_op(bio) & BIO_REQ_DISCARD)
#endif
			return;

		cpu = part_stat_lock();
		part = disk_map_sector_rcu(bio->bi_bdev->bd_disk, bio->bi_iter.bi_sector);
		if (!hd_struct_try_get(part)) {
				/**
				 * The partition is already being removed,
				 * the request will be accounted on the disk only.
				 * We take a reference on disk->part0 although that
				 * partition will never be deleted, so we can treat
				 * it as any other partition.
				 */
				part = &bio->bi_bdev->bd_disk->part0;
		hd_struct_get(part);
		}
		part_round_stats(cpu, part);
		part_inc_in_flight(part, rw);
		cmd_info->part = part;
	DPRINT2("bio %p, part %p\n", bio, part);
		part_stat_unlock();
}


/**
 * @brief This function handles disk statistic when a cmd_info is completed.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct cmd_info, pointer to Command information block
 *
 *
 * @return none.
 */
static void inline
nvme_disk_stat_completion(struct nvme_dev *dev, struct cmd_info *cmd_info)
{
		struct hd_struct *part;
	struct bio *bio = cmd_info->req;
		int cpu;

		if (!nvme_do_io_stat)
		return;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
	if (!bio || (bio->bi_rw & BIO_REQ_DISCARD))
#else
	if (!bio || (bio_op(bio) & BIO_REQ_DISCARD))
#endif
			return;

		cpu = part_stat_lock();

		part = cmd_info->part;
	DPRINT2("bio %p, part %p, count %d\n",
					bio, bio->bi_private, cmd_info->count);
	if (part) {
			part_stat_add(cpu, part, sectors[bio_data_dir(bio)],
						cmd_info->count >> 9);
	}
		part_stat_unlock();
}


/**
 * @brief This function handles disk statistic when a request is completed.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct cmd_info, pointer to Command information block
 *
 *
 * @return none.
 */
static void inline
nvme_disk_stat_iodone(struct nvme_dev *dev, struct cmd_info *cmd_info)
{
		unsigned long duration = jiffies - cmd_info->start_time;
		struct hd_struct *part = cmd_info->part;
	struct bio *bio = cmd_info->req;
		int cpu, rw = bio_data_dir(bio);

		/*
		 * Account IO completion.  flush_rq isn't accounted as a
		 * normal IO on queueing nor completion.  Accounting the
		 * containing request is enough.
		 */
		if (!nvme_do_io_stat)
		return;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
		if (!bio || (bio->bi_rw & BIO_REQ_DISCARD))
#else
		if (!bio || (bio_op(bio) & BIO_REQ_DISCARD))
#endif
			return;

		cpu = part_stat_lock();
	DPRINT2("bio %p, part %p\n", bio, part);
	if (!part) {
		DPRINT2("*********  NULL part %p  ******\n", part);
			part_stat_unlock();
		return;
	}
		part_stat_add(cpu, part, sectors[bio_data_dir(bio)],
						cmd_info->count >> 9);
		part_stat_inc(cpu, part, ios[rw]);
		part_stat_add(cpu, part, ticks[rw], duration);
		part_round_stats(cpu, part);
		part_dec_in_flight(part, rw);

/* @todo ?  hd_struct_put(part); */
		atomic_dec_and_test(&part->ref);
		part_stat_unlock();
}
#else
#define nvme_disk_stat_iodone(dev, cmd_info)
#define nvme_disk_stat_completion(dev, cmd_info)
#define nvme_disk_stat_in(dev, cmd_info, bio)
#define nvme_disk_stat_cong(dev, ns, bio)
#endif  /* DO_IO_STAT */


/**
 * @brief This function duplicates blocks of 64bit aligned data.
 *  	 This routine assumes all blocke sizes 64bit aligned.
 *
 * @param[in] void *dst Destination address
 * @param[in] void *src Source address
 * @param[in] int cnt size in u64
 *
 * @return void none.
 */
void inline
nvme_memcpy_64(void *dst, void *src, int cnt)
{
	u64 *p1 = dst, *p2 = src;
	while(cnt--) {
		*p1++ = *p2++;
	}
}

/**
 * @brief This function set memory blocks of 64bit aligned data.
 *  	 This routine assumes all blocke sizes 64bit aligned.
 *
 * @param[in] void *dst Destination address
 * @param[in] u64 val value to set memory.
 * @param[in] int cnt size in u64
 *
 * @return void none.
 */
void inline
nvme_memset_64(void *dst, u64 val, int cnt)
{
	u64 *p1 = dst;
	while(cnt--) {
		*p1++ = val;
	}
}



/**
 * @brief This function returns controller PCI vendor and device IDs string.
 *
 * @param[in] struct pci_dev * pci_dev pointer PCI device.
 *
 * @return char * pointer for formated string "[VVVV:DDDD]"
 */
static char *
nvme_pci_id(struct pci_dev * pci_dev)
{
	static  char	pci_id[64]; /** PCI ID string */

	sprintf(pci_id, "PCI %02x:%02x.%0x [%04x:%04x] Rev: %02d,",
			pci_dev->bus->number, PCI_SLOT(pci_dev->devfn),
			PCI_FUNC(pci_dev->devfn),
			pci_dev->vendor, pci_dev->device, pci_dev->revision);
	return (pci_id);
}

/**
 * @brief This function returns controller PCI card information string.
 *
 * @param[in] struct pci_dev * pci_dev pointer PCI device.
 *
 * @return char * pointer for formated string
 */
static char *
nvme_pci_info(struct pci_dev * pci_dev)
{
	static  char	pci_str[32];	/** PCI Info string */
	int pcie_reg;
	char *str;

	str = pci_str;
	sprintf(str, "PCIe <unknown>");
	pcie_reg = pci_find_capability(pci_dev, PCI_CAP_ID_EXP);
	if (pcie_reg) {
		char local[6];
		uint16_t link_state, speed, link;

		pcie_reg += 0x12;
		pci_read_config_word(pci_dev, pcie_reg, &link_state);
		speed = link_state & 0x0F;
		link = (link_state & 0x3F0) >> 4;
		strcpy(str, "PCIe (");
		if (speed == 1)
			strcat(str, "2.5 GT/s ");
		else if (speed == 2)
			strcat(str, "5.0 GT/s ");
		else if (speed == 3)
			strcat(str, "8.0 GT/s ");
		else
			strcat(str, "<unknown> ");
		sprintf(local, "%1dX)", link);
		strcat(str, local);
	}
	return (str);
}

/**
 * @brief This function enables PCI INTx interrupt.
 *
 * @param[in] struct pci_dev * pci_dev pointer PCI device.
 *
 * @return void
 */
static void
nvme_pci_enable_intx(struct pci_dev * pci_dev)
{
	int pcie_reg = 0x04;
	u32 command;

	pci_read_config_dword(pci_dev, pcie_reg, &command);
	command &= ~((1 << 10) | (1 << 0)); 	/* clear INTx & IO */
	pci_write_config_dword(pci_dev, pcie_reg, command);

}


/**
 * @brief This function sets up admin queue parameters and resets controller
 *  	start controller operation:
 *  	1 - Setup Admin queue parameters.
 *  	2 - reset controller
 *  	3 - Wait controller READY state.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 */
static int
nvme_hw_start(struct nvme_dev *dev)
{
	struct queue_info *qinfo;
	struct sub_queue_info *sqinfo;
	struct cmd_info *cmd_info;
	int result = 0;
	u32 aqa, config;
	int i;
	void __iomem *reg;

	qinfo = dev->admin_queue;
	sqinfo = qinfo->sub_queue;
	reg = dev->reg;
	nvme_hw_cap = readq((reg + NVME_CAP));

	DPRINT2("controller capability 0x%llx\n", nvme_hw_cap);
	nvme_hw_timeout = (nvme_hw_cap & NVME_CAP_TO_MSK64) >> NVME_CAP_TO_LSB;
	nvme_hw_timeout = (nvme_hw_timeout+1) >> 1;
	DPRINT2("controller timeout %d\n", nvme_hw_timeout);

	/** Clear controller Enable (EN) */
	if (readl(reg+NVME_CSTS)&NVME_CSTS_RDY) {
		writel(0, (reg+NVME_CC));
		nvme_wait_cond(dev, nvme_hw_timeout,
				(!(readl(reg+NVME_CSTS)&NVME_CSTS_RDY)), result);
		DPRINT2("initial disable result %d\n", result);
		if (result) {
			EPRINT("Controller reset clear enable failure status 0x%x\n",
							readl(reg+NVME_CSTS));
			return (result);
		}
	}
	writel(NVME_CC_ENABLE, (reg+NVME_CC));
	nvme_wait_cond(dev, nvme_hw_timeout,
				(readl(reg+NVME_CSTS)&NVME_CSTS_RDY), result);
	DPRINT2("initial reset result %d\n", result);
	if (result && ! forcerdy) {
		EPRINT("Controller reset enable failure status 0x%x\n",
							readl(reg+NVME_CSTS));
		return (result);
	}
	writel(0, (reg+NVME_CC));
	nvme_wait_cond(dev, nvme_hw_timeout,
				(!(readl(reg+NVME_CSTS)&NVME_CSTS_RDY)), result);
	DPRINT2("controller disabled status %d\n", result);
	if (result) {
		EPRINT("Controller reset clear enable failure status 0x%x\n",
							readl(reg+NVME_CSTS));
		return (result);
	}
	if (!nvme_msix_enable)
		nvme_pci_enable_intx(dev->pci_dev);

	/** Set admin queue depth of completion and submission */
	aqa  = (sqinfo->qsize -1) << NVME_AQA_SQS_LSB;
	aqa |= (qinfo->qsize -1) << NVME_AQA_CQS_LSB;

	/** Set admin queue attributes */
	writel(aqa, (reg+NVME_AQA));
	writeq(qinfo->compq_phy, (reg+NVME_ACQ));
	writeq(sqinfo->subq_phy, (reg+NVME_ASQ));

	/** Setup controller configuration and enable */
	config  = NVME_CC_ENABLE;
	config |= NVME_CC_CSS_NVM << NVME_CC_CSS_LSB;
	config |= (PAGE_SHIFT - 12) << NVME_CC_MPS_LSB;
	config |= (NVME_CC_ARB_RR << NVME_CC_AMS_LSB);
	config |= (NVME_CC_SHN_NONE << NVME_CC_SHN_LSB);
	config |= (6 << NVME_CC_IOSQES_LSB);
	config |= (4 << NVME_CC_IOCQES_LSB);
	writel(config, (reg+NVME_CC));

	nvme_wait_cond(dev, nvme_hw_timeout,
				(readl(reg+NVME_CSTS)&NVME_CSTS_RDY), result);
	if (result && ! forcerdy) {
		EPRINT("Controller reset enable failure status 0x%x\n",
							readl(reg+NVME_CSTS));
		EPRINT("Failed to start controller %d\n", result);
		return (result);
	}

	nvme_rms_info(dev);

	/** aborting all pending  admin commands*/
	nvme_flush_admin(qinfo);

	/** Initialize Completion and submission queues info */
	qinfo->head = 0;
	qinfo->tail = 0;
	qinfo->phase = 1;
	qinfo->timeout_id = -1;
	nvme_memset_64(qinfo->compq, 0LL,
			(sizeof(* qinfo->compq)/sizeof(u64)) * qinfo->qsize);
	sqinfo->head = 0;
	sqinfo->tail = 0;
	sqinfo->entries = sqinfo->qsize - 1;
	nvme_memset_64(sqinfo->subq, 0LL,
			(sizeof(* sqinfo->subq)/sizeof(u64)) * sqinfo->qsize);

	qinfo->id_count = admin_sub_queue_size;
	INIT_LIST_HEAD(&qinfo->cmd_free);
	INIT_LIST_HEAD(&qinfo->cmd_active);
	cmd_info = qinfo->cmd_list;
	for (i = 0; i < qinfo->id_count; i++) {

		spin_lock_init(&cmd_info->lock);
		cmd_info->cmd_id	 = i+1; 		/** 0 is reserved */
		init_waitqueue_head(&cmd_info->waitq);
		list_add_tail(&cmd_info->list, &qinfo->cmd_free);
		cmd_info++;
	}
	dev->cur_aen = 0;
	return 0;
}


/**
 * @brief This function shuts down controller.
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_hw_shutdown(struct nvme_dev *dev)
{
	u32 config;
	int result = 0;
	void __iomem *reg;

	reg = dev->reg;
	dev->lock_func(&dev->lock, &dev->lock_flags);
	dev->state = NVME_STATE_SHUTDOWN;
	/** shutdown controller Enable */
	nvme_hw_stop(dev, NVME_DELAY);
	config  = (NVME_CC_SHN_NORMAL << NVME_CC_SHN_LSB);

	DPRINT2("writing 0x%08x to %p\n", config, (dev->reg+NVME_CC));
	writel(config, (dev->reg+NVME_CC));
	nvme_delay_cond(dev, nvme_hw_timeout,
			((readl(reg+NVME_CSTS)&NVME_CSTS_SHST_MSK) ==
						NVME_CSTS_SHST_CPL), result);
	DPRINT2("controller shutdown status 0x%08x, result %d\n",
			(readl(reg+NVME_CSTS)&NVME_CSTS_SHST_MSK), result);
	if (result) {
		EPRINT("Failed to shutdown controller %d\n", result);
	}
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	return (result);
}


/**
 * @brief This function stops controller operation by clearing CC.EN.
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_hw_stop(struct nvme_dev *dev, int wait)
{
	int result = 0;

	/** Clear controller Enable */
	if (readl(dev->reg+NVME_CSTS)&NVME_CSTS_RDY)
		writel(0, (dev->reg+NVME_CC));
	if (NVME_DELAY == wait) {
		nvme_delay_cond(dev, nvme_hw_timeout,
			(!(readl(dev->reg+NVME_CSTS)&NVME_CSTS_RDY)), result);
	} else if (NVME_WAIT == wait) {
		nvme_wait_cond(dev, nvme_hw_timeout,
			(!(readl(dev->reg+NVME_CSTS)&NVME_CSTS_RDY)), result);
	}
	return (result);
}


/**
 * @brief This function posts NVME command to submission queue.
 *
 *
 * @param[in] struct sub_queue_info *sqinfo pointer to queue information block
 * @param[in] struct cmd_info *cmd_info pointer to command information block
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note It is assumed that queue lock is held by caller.
 */
int
nvme_send_cmd(struct sub_queue_info *sqinfo, struct cmd_info *cmd_info)
{
	u32 tail;
	struct nvme_dev *dev = sqinfo->dev;

	tail = sqinfo->tail;
	cmd_info->status = -1;
	if (unlikely(!sqinfo->entries)) {
		EPRINT("Submission queue is full %p [%d]\n", sqinfo, sqinfo->id);
		return (-ENOMEM);
	}

	nvme_memcpy_64( &sqinfo->subq[sqinfo->tail], &cmd_info->nvme_cmd,
				sizeof(cmd_info->nvme_cmd)/sizeof(u64) );
	DPRINT5("Sub queue[%d] doorbell %p->[%d]\n", sqinfo->id,
				sqinfo->doorbell, sqinfo->tail);

#ifdef NVME_DEBUG
		if (nvme_dbg & NVME_DEBUG_DUMP) {
			int i;
			u32 *ptr;
		ptr = (u32 *)&cmd_info->nvme_cmd;
		for (i=0; i<sizeof(struct nvme_cmd)/sizeof(u32); i += 4) {
			DPRINT("%02x: %08x %08x %08x %08x\n", i,
			ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
		}
		}
#endif
	tail++;
	if (tail >= sqinfo->qsize)
		tail = 0;
	dev->doorbell_req++;
	writel(tail, sqinfo->doorbell);
	sqinfo->tail = tail;
	sqinfo->entries--;
	return 0;
}

/**
 * @brief This function Sends a command and waits for its completion
 *
 * @param[in] struct queue_info *qinfo pointer to Queue Information block
 * @param[in] struct cmd_info *cmd_info pointer to command information
 * @param[in] int timeout time allowed to complete request
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_wait_cmd(struct queue_info *qinfo, struct cmd_info *cmd_info, int timeout)
{
#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif
	int result;

	DPRINT3("cmd_info %p, timeout %d\n", cmd_info, timeout);
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	if (nvme_send_cmd(qinfo->sub_queue, cmd_info)) {
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		return -ENOMEM;
	}
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	result = wait_event_timeout(cmd_info->waitq,
					(cmd_info->status != -1), timeout);
	DPRINT3("Woke up result %d, status 0x%x\n", result, cmd_info->status);
	if (cmd_info->status == -1) {
		if (result)
			result = -EINTR;
		else
			result = -ETIME;
		cmd_info->type = ABORT_CONTEXT;
		cmd_info->cmd_status = SF_SC_CMD_ABORT_REQ;
		cmd_info->status = ETIME;
	} else if(cmd_info->status==EINTR){
			 result=-EINTR;
		}
		else
			result = 0;
	return (result);
}


#ifdef NVME_DEBUG
/**
 * @brief This function list active bio list
 *  	This function is used for debuggng purposes only.
 *
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 *
 * @note It is assumed that queue lock is held by caller.
 */
static void
nvme_dump_list(struct queue_info *qinfo)
{
	struct bio *bio;
	struct ns_info *ns;
	struct cmd_info *cmd_info;
	struct nvme_dev *dev = qinfo->dev;

	if (qinfo->flags & QUEUE_SUSPEND)
		return;

	if (qinfo->nr_req) {
		DPRINT("****** dev %p, state %d, timeout_id %d\n",
				dev, dev->state, dev->timeout_id);
		DPRINT("****** qinfo %p [%d], nr_req %d, timeout_id %d\n",
			qinfo, qinfo->id, qinfo->nr_req, qinfo->timeout_id);
	}
	bio_list_for_each(bio, &qinfo->sq_cong) {
		ns = bio->bi_bdev->bd_disk->private_data;
		DPRINT("------ Congested: bio %p, ns %p, [%d]\n", bio, ns, ns->id);
	}
	list_for_each_entry(cmd_info, &qinfo->cmd_active, list) {
		DPRINT("------ active: cmd_info %p, bio %p, id %d\n",
					cmd_info, cmd_info->req, cmd_info->timeout_id);
	}
	if (qinfo->nr_req) {
		list_for_each_entry(cmd_info, &qinfo->cmd_free, list) {
			 DPRINT("------ free: cmd_info %p, bio %p\n",
					cmd_info, cmd_info->req);
		}
	}
}
#endif

/**
 * @brief This function seraches active command list for staled command(s)
 *  	This function is used by timer thread to find commands that
 *  	did not complete in allotted time slot.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] int timeout_id currently selected timeout slot entry.
 *
 * @note It is assumed that dev lock is held by caller.
 *
 */
static void
nvme_stalled_command(struct work_struct *work)
{
	struct queue_info *qinfo =
				container_of(work, struct queue_info, wq_time);
	struct nvme_dev *dev = qinfo->dev;
	struct bio *bio;
	struct cmd_info *cmd_info, *tmp;
	u32 new_id;

	new_id = qinfo->timeout_id;
	DPRINT4("****** qinfo %p [%d], cpu %d, nr_req %d, timeout_id %d\n",
			qinfo, qinfo->id, qinfo->cpu,
			qinfo->nr_req, new_id);

	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	qinfo->timeout_id = -1;
	qinfo->flags |= QUEUE_SUSPEND;
	list_for_each_entry_safe(cmd_info, tmp, &qinfo->cmd_active, list) {

		if (cmd_info->timeout_id == new_id) {

			DPRINT4("Command timeout req %p, timeout id %d ****\n",
			cmd_info->req, cmd_info->timeout_id);
			DPRINT4("*** Command info %p, base_info %p ****\n",
					cmd_info, cmd_info->cmd_base);

			if (NULL == (bio = cmd_info->req)) {
				if ((cmd_info = cmd_info->cmd_base)) {
					bio = cmd_info->req;
				}
			}

			if (bio) {
#ifdef NVME_DEBUG
				if (nvme_dbg & NVME_DEBUG_DUMP_TIME) {
					int i;
					u32 *ptr;
					ptr = (u32 *)cmd_info;
					for (i=0; i<sizeof(struct cmd_info)/sizeof(u32);
						i += sizeof(u32)) {
						DPRINT4("%02x: %08x %08x %08x %08x\n", i,
								ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
					}
				}
#endif
				EPRINT("Command timeout \"%s\" dev [%d:%d], block %lld, "
					   "size %u\n",
					   cmd_info->ns->disk->disk_name,
					   MAJOR(bio->bi_bdev->bd_dev),
					   MINOR(bio->bi_bdev->bd_dev),
					   (u64)bio->bi_iter.bi_sector,
					   bio->bi_iter.bi_size);
				if (cmd_info->sg_count) {
					dma_unmap_sg(dev->dma_dev, cmd_info->sg_list,
								 cmd_info->sg_count, DMA_FROM_DEVICE);
					cmd_info->sg_count = 0;
				}
				qinfo->nr_req--;
#if FORCE_TIMEOUT
				bio->bi_error = 0;
				bio_endio(bio);  	/* allow test to continue */
#else
				bio->bi_error = -ETIME;
				bio_endio(bio);
#endif
				nvme_disk_stat_iodone(dev, cmd_info);
				cmd_info->req = NULL;
			}
		}
	}
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	dev->lock_func(&dev->lock, &dev->lock_flags);
	dev->wq_done++;
	DPRINT4("wq_count %d, wq_done %d\n",
				dev->que_count, dev->wq_done);
	if (dev->state <= NVME_STATE_SUSPEND) {
		dev->state = NVME_STATE_SUSPEND;
		if (dev->wq_done == dev->que_count) {
			dev->thread_flags |= THREAD_RESTART;
			wake_up(&dev->thread_wait);
		}
	}
	dev->unlock_func(&dev->lock, &dev->lock_flags);
}


/**
 * @brief This function is timer heardbeat to keep track of active commands.
 *  	This function is called TIMEOUT_INTERVAL to check for timed
 *  	out commands.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @param[in] none.
 */
void
nvme_timeout(ulong arg)
{
	struct nvme_dev *dev = (void *)arg;
	struct queue_info *qinfo;
	int i, new_id;

	/**
	 * Search all IO queues for staled request.
	 */
	dev->lock_func(&dev->lock, &dev->lock_flags);
	new_id = dev->timeout_id +1;
	if (new_id >= TIMEOUT_LIST)
		new_id = 0;

	/**
	 * Timer is only valid when we are operational.
	 */
	if (NVME_STATE_OPERATIONAL != dev->state)
		goto skip_timer;

	dev->wq_done = 0;

	for (i = 1; i <= dev->que_count; i++) {
		qinfo = dev->que_list[i];
		if (! qinfo)
			continue;

#ifdef NVME_DEBUG
		/* @todo - move after QUEUE_SUSPEND */
		if (nvme_dbg & NVME_DEBUG_DUMP_TIME)
		{
			int j;
			u32 *ptr= (u32 *)qinfo->timeout;
			for (j=0; j < TIMEOUT_LIST; j+= 5) {
				if (ptr[j] || ptr[j+1] || ptr[j+2] || ptr[j+3] || ptr[j+4]){
					DPRINT4("timeout_id %d\n", new_id);
					DPRINT4("non-zero qinfo %p [%d] timeout IDs:\n",
							qinfo, qinfo->id);
					DPRINT4("%02x: %08x %08x %08x %08x %08x\n", j,
							ptr[j], ptr[j+1], ptr[j+2], ptr[j+3], ptr[j+4]);
				}
			}
		}
#endif
		/**
		 * Timer is only valid when we are operational.
		 */
		if (qinfo->flags & QUEUE_SUSPEND) {
			DPRINT4("qinfo %p [%d] suspended, skipping ...\n",
					qinfo, qinfo->id);
			continue;
		}

		/**
		 * Update queue timer slot.
		 * Check next timer slot and see if there are currently commands
		 * pending on this slot. Any commands with matching timeout Ids
		 * must be aborted.
		 */
		if (qinfo->timeout[new_id]) {
			if (reset_card_on_timeout) {
				dev->state = NVME_STATE_SUSPEND;
				qinfo->timeout_id = new_id;
				nvme_hw_stop(dev, NVME_NO_WAIT);
				DPRINT4("qinfo %p, timeout[%d]= %d\n",
						qinfo, new_id, qinfo->timeout[new_id]);

				nvme_suspend_io_queues(dev, new_id);
				break;
			} else {
				struct cq_entry *cq_entry;

				cq_entry = &qinfo->compq[qinfo->head];
				if (cq_entry) {
					struct cmd_info *cmd_info = &qinfo->cmd_list[cq_entry->cmdID-1];

					printk(KERN_INFO "cpu %d timeout_id %d timeout %d type %d\n",
						current->on_cpu, new_id, qinfo->timeout[new_id], cmd_info->type);
					if (cmd_info->type == BIO_CONTEXT) {
						struct bio *bio = cmd_info->req;
						printk(KERN_INFO "%s bio %s size %u\n", cmd_info->ns->disk->disk_name,
							   bio_data_dir(bio) ? "Write" : "Read", bio->bi_iter.bi_size);
					}
				} else {
					printk(KERN_INFO "Timeout on qinfo %p\n", qinfo);
				}
				/* AC: should all the queues be flush or just the one? */
				qinfo->nrf_timeout++;
				nvme_flush_io_queues(dev, NULL, -EIO);		/* Flush all requests */
				break;
			}
		}
	}
skip_timer:
	if (dev->state <= NVME_STATE_OPERATIONAL) {
		mod_timer(&dev->timer, (jiffies + TIMEOUT_FREQ));
		dev->timeout_id = new_id;
	}
	dev->unlock_func(&dev->lock, &dev->lock_flags);
}


/**
 * @brief This function Sends an Admin Command to controller.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct nvme_cmd *entry pointer to NVME command entry
 * @param[in] struct cq_entry *cq_entry pointer to NVME completion entry
 * @param[in] int timeout command timeout in seconds
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_send_admin(struct nvme_dev *dev, struct nvme_cmd *entry,
				struct cq_entry *cq_entry, int timeout)
{
	struct nvme_cmd *cmd;
	struct queue_info *qinfo;
	struct cmd_info *cmd_info;
	int result = 0;

	qinfo = dev->admin_queue;
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	cmd_info = nvme_get_cmd(qinfo);
	if (! cmd_info) {
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		return (-ENOMEM);
	}

	cmd_info->timeout_id	= dev->timeout_id;
	if (timeout)
		qinfo->timeout[cmd_info->timeout_id]++;
	cmd_info->req   	= NULL;
	cmd_info->type  	= ADMIN_CONTEXT;
	entry->header.cmdID = cmd_info->cmd_id;
	cmd = &cmd_info->nvme_cmd;
	nvme_memcpy_64(&cmd_info->nvme_cmd, entry,
				sizeof(*entry)/sizeof(u64));

	DPRINT("admin cmd 0x%x \n", cmd->header.opCode);

	/**
	 * If there is no timeout value, this is an Asynchronous
	 * request. Just send the command and return the response.
	 */
	if (!timeout) {
		if (nvme_send_cmd(qinfo->sub_queue, cmd_info))
		result = -ENOMEM;
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		return(result);
	}
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

	/**
	 * Issue command, and wait a maximum timeout.
	 */
	result = nvme_wait_cmd(qinfo, cmd_info, timeout);
	if (cq_entry) {
		nvme_memcpy_64(cq_entry, &cmd_info->cq_entry,
					sizeof(*cq_entry)/sizeof(u64));
	}

	if (!result && (cmd_info->cq_entry.SC)) {
		result = -EIO;
	}

	/**
	 * release cmd_info and clear timer entry.
	 */
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	qinfo->timeout[cmd_info->timeout_id]--;
	nvme_put_cmd(qinfo, cmd_info);
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

	if (result)
		EPRINT("Failed Admin command 0x%x, result %d\n",
					entry->header.opCode, result);

	return  (result);
}

/**
 * @brief This function Retrieves controller/Namespace identify data.
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] int ns_id namespace ID
 * @param[in] void  *addr pointer buffer to copy Identify data
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_get_identify(struct nvme_dev *dev, int ns_id, void  *addr)
{
	void *buff;
	dma_addr_t buff_phy;
	struct nvme_cmd entry;
	struct cq_entry cq_entry;
	int result;

	buff = dma_alloc_coherent(dev->dma_dev, 4096,
					&buff_phy, GFP_KERNEL);
	if (! buff) {
		EPRINT("Identify DMA buffer allocation error\n");
		return (-ENOMEM);
	}

	nvme_memset_64(&entry, 0LL, sizeof(entry)/sizeof(u64));
	entry.header.opCode 		= NVM_ADMIN_CMD_IDENTIFY;
	if (ns_id < 0) {
		entry.cmd.identify.controllerStructure  = IDENTIFY_CONTROLLER;
	} else {
		entry.cmd.identify.controllerStructure  = IDENTIFY_NAMESPACE;
		entry.header.namespaceID			= ns_id;
	}
	entry.header.prp[0].addr = buff_phy;
	entry.header.prp[1].addr = (buff_phy+PAGE_SIZE) & ~(PAGE_SIZE -1);

	result = nvme_send_admin(dev, &entry, &cq_entry, ADMIN_TIMEOUT);
	DPRINT2("Identify [0x%04x] completion result %d, Status 0x%x\n",
					ns_id, result, cq_entry.SC);
	if (! result) {
		nvme_memcpy_64(addr, buff, 4096/sizeof(u64));
	}
	dma_free_coherent(dev->dma_dev, 4096, buff, buff_phy);
	return  (result);
}


/**
 * @brief This function Retrieves controller identify data.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] void  *addr pointer buffer to copy Identify data
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_get_ctrl_identify(struct nvme_dev *dev)
{
	int result;

	result = nvme_get_identify(dev, -1, dev->identify);
	if (result) {
		EPRINT("Failed to retrieve controller Identify data.\n");
		return -EIO;
	}
	dev->admVendCmdCfg   = dev->identify->admVendCmdCfg;
	dev->nvmVendCmdCfg   = dev->identify->nvmVendCmdCfg;
	dev->nvmCacheSupport = dev->identify->volWrCache;
	dev->nvmCmdSupport   = dev->identify->cmdSupt;
	return 0;

}

/**
 * @brief This function Deletes a submission queue
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] int id submisssion queue ID
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_delete_sq(struct nvme_dev *dev, u16 id)
{
	struct  nvme_cmd entry;

	nvme_memset_64(&entry, 0LL, sizeof(entry)/sizeof(u64));
	entry.header.opCode 	= NVM_ADMIN_CMD_DEL_SQ;
	entry.cmd.deleteSubQ.identifier = cpu_to_le16(id);
	return(nvme_send_admin(dev, &entry, NULL, ADMIN_TIMEOUT));
}


/**
 * @brief This function Deletes a completion queue
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] int id completion queue ID
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_delete_cq(struct nvme_dev *dev, u16 id)
{
	struct  nvme_cmd entry;

	nvme_memset_64(&entry, 0LL, sizeof(entry)/sizeof(u64));
	entry.header.opCode 	= NVM_ADMIN_CMD_DEL_CQ;
	entry.cmd.deleteCplQ.identifier = cpu_to_le16(id);
	return(nvme_send_admin(dev, &entry, NULL, ADMIN_TIMEOUT));
}

/**
 * @brief This function Creates a completion queue
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] int id queue ID
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_create_cq(struct nvme_dev *dev, struct queue_info *qinfo, u16 qid)
{
	struct nvme_cmd entry;

	nvme_memset_64(&entry, 0LL, sizeof(entry)/sizeof(u64));
	entry.header.opCode 			= NVM_ADMIN_CMD_CREATE_CQ;
	entry.header.prp[0].addr			= qinfo->compq_phy;
	entry.cmd.createCplQ.identifier 	= qid;
	entry.cmd.createCplQ.size   		= qinfo->qsize - 1;
	entry.cmd.createCplQ.contiguous 		= 1;
	entry.cmd.createCplQ.interruptEnable	= 1;
	entry.cmd.createCplQ.interruptVector	= cpu_to_le16(qinfo->cq_ivec);

	return(nvme_send_admin(dev, &entry, NULL, ADMIN_TIMEOUT));
}

/**
 * @brief This function Creates a submission queue
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] int id queue ID
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_create_sq(struct nvme_dev *dev, struct queue_info *qinfo, u16 qid)
{
	struct nvme_cmd entry;

	nvme_memset_64(&entry, 0LL, sizeof(entry)/sizeof(u64));
	entry.header.opCode 			= NVM_ADMIN_CMD_CREATE_SQ;
	entry.header.prp[0].addr			= qinfo->sub_queue->subq_phy;
	entry.cmd.createSubQ.identifier 	= qid;
	entry.cmd.createSubQ.size   		= qinfo->sub_queue->qsize - 1;
	entry.cmd.createSubQ.contiguous 		= 1;
	entry.cmd.createSubQ.priority   	= 0;	/** High */
	entry.cmd.createSubQ.completionQueueID  = qinfo->id;

	return(nvme_send_admin(dev, &entry, NULL, ADMIN_TIMEOUT));
}

/**
 * @brief This function Sends a set feature command
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] u16 feature feature ID
 * @param[in] u32 option feature option
 * @param[in] struct nvme_prp *prp pointer to prp list of feature data
 * @param[in] struct cq_entry *cq_entry pointer to completion entry
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_set_feature(struct nvme_dev *dev, u16 feature, u32 option,
			struct nvme_prp *prp, struct cq_entry *cq_entry)
{
	struct  nvme_cmd entry;

	DPRINT2("Feature ID 0x%0x, option 0x%08x\n", feature, option);
	nvme_memset_64(&entry, 0LL, sizeof(entry)/sizeof(u64));
	entry.header.opCode 		= NVM_ADMIN_CMD_SET_FEATURES;
	if (prp) {
		entry.header.prp[0] 		= *prp;
		entry.header.prp[1].addr	= (prp->addr + PAGE_SIZE) &
							~(PAGE_SIZE -1);
	}
	entry.cmd.setFeatures.featureID = feature;
	entry.cmd.asUlong[1]			= option;
	return(nvme_send_admin(dev, &entry, cq_entry, ADMIN_TIMEOUT));
}

/**
 * @brief This function retrieves a feature information
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] u16 feature feature ID
 * @param[in] u32 option feature option
 * @param[in] struct nvme_prp *prp pointer to prp list of feature data
 * @param[in] struct cq_entry *cq_entry pointer to completion entry
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_get_feature(struct nvme_dev *dev, int ns_id, u16 feature, u32 option,
			struct nvme_prp *prp, struct cq_entry *cq_entry)
{
	struct  nvme_cmd entry;

	DPRINT2("Feature ID 0x%0x\n", feature);
	nvme_memset_64(&entry, 0LL, sizeof(entry)/sizeof(u64));
	entry.header.opCode 		= NVM_ADMIN_CMD_GET_FEATURES;
	entry.header.namespaceID		= ns_id;
	if (prp) {
		entry.header.prp[0] 		= *prp;
		entry.header.prp[1].addr	= (prp->addr + PAGE_SIZE) &
							~(PAGE_SIZE -1);
	}
	entry.cmd.getFeatures.featureID = feature;
	entry.cmd.asUlong[1]			= option;
	return(nvme_send_admin(dev, &entry, cq_entry, ADMIN_TIMEOUT));
}


/**
 * @brief This function sends a request to retrieve a log page
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] u32 param log ID and log type
 * @param[in] void *req pointer to optional original request causing request.
 * @param[in] void *buff pointer to log page virtual address.
 * @param[in] struct nvme_prp *prp pointer to prp list of feature data
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note It is assumed that queue lock is held by caller.
 */
static int
nvme_get_log_page(struct nvme_dev *dev, u32 param, void *req, void *buff,
				struct nvme_prp *prp)
{
	struct nvme_cmd *entry;
	struct queue_info *qinfo;
	struct cmd_info *cmd_info, *cmd_req = req;;

	qinfo = dev->admin_queue;
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	DPRINT11("qinfo %p, param 0x%x, req %p\n", qinfo, param, req);
	DPRINT11("qinfo %p, buff %p, phy %llx\n", qinfo, buff, prp->addr);

	cmd_info = nvme_get_cmd(qinfo);
	if (! cmd_info) {
		EPRINT("Memory Allocation error to retrieve log.\n");
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		return (-1);
	}

	cmd_info->timeout_id	= dev->timeout_id;
	qinfo->timeout[cmd_info->timeout_id]++;
	cmd_info->req = buff;
	cmd_info->cmd_param = (u64)prp->addr;
	if (cmd_req) {
		cmd_info->ns = cmd_req->ns;
		cmd_info->count = cmd_req->count;
		cmd_info->type = ERR_CONTEXT;
	} else {
		cmd_info->type = LOG_CONTEXT;
	}
	DPRINT11("requesting log cmd_info %p, context %d, param 0x%x\n",
					cmd_info, cmd_info->type, param);
	entry = &cmd_info->nvme_cmd;
	entry->cmd.getLogPage.LogPageID = param & 0xFFFF;
	entry->cmd.getLogPage.numDW = (param >> 18);
	entry->header.opCode = NVM_ADMIN_CMD_GET_LOG_PAGE;
	entry->header.prp[0].addr = prp->addr;
	entry->header.prp[1].addr = (prp->addr + (PAGE_SIZE)) &
							~(PAGE_SIZE -1);
	entry->header.cmdID  = cmd_info->cmd_id;
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

	DPRINT11("admin cmd 0x%x \n", entry->header.opCode);
	return(nvme_send_cmd(qinfo->sub_queue, cmd_info));
}



/**
 * @brief This function called to retrive an event.
 *  Caller may specify the mask and matching ID to retrieve a unique
 *  event type.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct event_req *event_req pointer to event request info.
 *
 * @return struct event_info * if successful, otherwise NULL
 */
static struct event_info *
nvme_check_event(struct nvme_dev *dev, struct event_req *event_req)
{

	if (! dev->event_count)
		return NULL;

#if 0   /** @todo - should be tested. */
	struct event_info *tmp, *event_info = NULL;
	dev->lock_func(&dev->elock, &dev->elock_flags);
	list_for_each_entry_safe(event_info, tmp, &dev->event_list, list) {

		if (event_info->event_id ==
				(event_req->event_id & event_req->event_mask)) {

		--dev->event_count;
			DPRINT("Event matched, count %d, event_info %p %d\n",
			dev->event_count, event_info, event_info->event_id);

			DPRINT("Event removed %p %d, buff %p\n", event_info,
				event_info->event_id, event_info->buff);
			list_del(&event_info->list);
			kfree(event_info->buff);
			kfree(event_info);
		}
	}
	dev->unlock_func(&dev->elock, &dev->elock_flags);
	return (event_info);
#else
	return NULL;
#endif
}



/**
 * @brief This function called to process a received LOG PAGE data.
 *  The event information is copied into user buffer addressed by
 *  (struct event_req *).
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] void __user * pointer to user request (struct event_req *)
 *
 * @return int 0 if successful, otherwise Error Code
 */
static int
nvme_wait_event(struct nvme_dev *dev,  void __user *p)
{
	struct event_req __user *puio = p;
	struct event_req *uio, ioctl_uio;
	struct event_info *event_info = NULL;
	int result = 0;

	/**
	 * Read user space uio into kernel uio header.
	 */
	uio = &ioctl_uio;
	if (copy_from_user(uio, p, sizeof(struct event_req))) {
		EPRINT("access violation user uio hdr %p\n", p);
		return -EFAULT;
	}

	if (!access_ok(VERIFY_WRITE, uio->addr, uio->length)) {
		EPRINT("Access violation command Ptr 0x%llx\n", uio->addr);
		return -EFAULT;
	}
	DPRINT8("puio %p, Write Access OK\n", puio);

	dev->lock_func(&dev->lock, &dev->lock_flags);
	do {
		if (! (event_info = nvme_check_event(dev, uio))) {

		if (event_info) {
			if (copy_to_user(event_info->buff, (void *)uio->addr,
								uio->length)) {
				EPRINT("Access violation Log Page Data "
						"Ptr 0x%llx\n", uio->addr);
				result = -EFAULT;
			break;
				}
			put_user(uio->length, &puio->length);
			break;
		}
			dev->unlock_func(&dev->lock, &dev->lock_flags);
			result = wait_event_interruptible(dev->event_wait, event_info);
			dev->lock_func(&dev->lock, &dev->lock_flags);
			DPRINT8("Woke up result %d\n", result);
		}

		if (unlikely(result)) {
		result = -EINTR;
		break;
		}
	} while (NULL == event_info);
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	return result;
}



/**
 * @brief This function retrieves current cache state.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if disabled,, otherwise 1
 */
static int
nvme_cache_check(struct nvme_dev *dev)
{
	struct cq_entry cq_entry;
	int status;

	status = nvme_get_feature(dev, 0, FTR_ID_WRITE_CACHE, 0,
							NULL, &cq_entry);
	if (status) {
		EPRINT("Failed to retrieve caching state status %d\n", status);
		cq_entry.param.cmdSpecific = 0;
	}
	DPRINT2("Volatile Cache %s\n", cq_entry.param.cmdSpecific ?
						"Enabled" : "Disabled");
	return (cq_entry.param.cmdSpecific);
}

/**
 * @brief This function enables or disables cache feature
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] u16 enable flags to enable or disable device cache
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_cache_enable(struct nvme_dev *dev, u16 enable)
{
	DPRINT2("Volatile Cache %s\n", enable ? "Enabled" : "Disabled");
	return(nvme_set_feature(dev, FTR_ID_WRITE_CACHE, enable, NULL, NULL));
}


char	*context_type_string[] = {
		"Reserved",
		"Admin",
		"IO",
		"IOCTL",
		"AEN",
		"Log",
		"Error Log",
		"Abort",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved"
	};

char	*err_type_string[] = {
		"Generic Status",
		"Command Specific Error",
		"Media Error",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Vendor Specific"
	};

char	*err_generic_string[] = {   	/** Offset - 00h */
		"Successful Completion",
		"Invalid Command Opcode",
		"Invalid Field in Command",
		"Command ID Conflict",
		"Data Transfer Error",
		"Commands Aborted due to Power Loss Notification",
		"Internal Device Error",
		"Command Abort Requested",
		"Command Aborted due to SQ Deletion",
		"Command Aborted due to Failed Fused Command",
		"Command Aborted due to Missing Fused Command",
		"Invalid Namespace or Format",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		""
	};

char	*err_generic_io_string[] = {		/** Offset - 80h */
		"LBA Out of Range",
		"Capacity Exceeded",
		"Namespace Not Ready",
		""
	};

char	*err_command_string[] = {   	/** Offset - 00h */
		"Completion Queue Invalid",
		"Invalid Queue Identifier",
		"Maximum Queue Size Exceeded",
		"Abort Command Limit Exceeded",
		"Requested Command to Abort Not Found",
		"Asynchronous Event Request Limit Exceeded",
		"Invalid Firmware Slot",
		"Invalid Firmware Image",
		"Invalid Interrupt Vector",
		"Invalid Log Page",
		"Invalid Format",
		"Firmware Application Requires Conventional Reset",
		"Invalid Queue Deletion",
		"Reserved",
		"Reserved",
		"Reserved"
	};

char	*err_command_io_string[] = {		/** Offset - 80h */
		"Conflicting Attributes"
	};

char	*err_media_blank[] = {  		/** Offset - 00h */
		"Reserved"
	};

char	*err_media_string[] = { 		/** Offset - 80h */
		"Write Fault",
		"Unrecovered Read Error",
		"End-to-end Guard Check Error",
		"End-to-end Application Tag Check Error",
		"End-to-end Reference Tag Check Error",
		"Compare Failure",
		"Access Denied",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved",
		"Reserved"
	};

char	**err_tbl[] = { 		/** index by type << 1 */
		err_generic_string, 	/** Type 0 */
		err_generic_io_string,  /** Type 0 ; Command Set specific */
		err_command_string, 	/** Type 1 */
		err_command_io_string,  /** type 1 ; Command Specific */
		err_media_blank,		/** Media Error reserved */
		err_media_string		/** Media Error */
	};


/**
 * @brief This function flush congestion queue.
 *  	This function is called to flush/complete request posted on
 *  	on congestion queue.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] struct ns_info *ns pointer to Namespace information block
 * @param[in] int status, command completion status
 *
 * @return void
 *
 * @note It is assumed that queue lock is held by caller.
 */
static void
nvme_flush_cong(struct queue_info *qinfo, struct ns_info *ns, int status)
{
	struct bio *bio;
#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif

	DPRINT4("****** qinfo %p [%d], nr_req %d, nr_act %d\n", qinfo,
				qinfo->id, qinfo->nr_req, qinfo->nr_act);
	while ((bio = bio_list_pop(&qinfo->sq_cong))) {

		DPRINT4("****** bio %p\n", bio);
		if (ns && (ns != bio->bi_bdev->bd_disk->private_data))
			continue;

		DPRINT4("Flushing bio %p\n", bio);

		bio->bi_error = status;
		bio_endio(bio);
		qinfo->nrf_sq_cong++;
	}
}


//#define DEBUG_FLUSH_QUEUE
#ifdef DEBUG_FLUSH_QUEUE
#pragma push_macro("DPRINT4")
#undef DPRINT4
#define DPRINT4(fmt, arg...)	\
  printk(KERN_INFO "[%s]" fmt, __func__, ##arg)
#endif
/**
 * @brief This function is to Flush all outstanding queue BIO requests.
 *  	This function is called during error recovery to either
 *  	terminate all pending BIO request or to insert into
 *  	congestion queue.
 *
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] struct ns_info *ns optional pointer to namespace information
 * @param[in] int status if zero, no completions otherwise; completion status
 *
 * @return void
 *
 * @note It is assumed that queue lock is held by caller.
 * @note The optional input ns_info specifies to only flush request of a
 *   particular Namespace.
 */
static void
nvme_flush_queue(struct queue_info *qinfo, struct ns_info *ns, int status)
{
	struct nvme_dev *dev = qinfo->dev;
	struct cmd_info *cmd_info, *tmp;
	struct bio *bio;

	/**
	 * Run through list of active commands and requeue bio requests.
	 * IO Queue processing is suspended until resumed.
	 */
	DPRINT4("qinfo %p [%d], nr_req %d, nr_act %d\n", qinfo, qinfo->id,
					qinfo->nr_req, qinfo->nr_act);
	DPRINT4("qinfo %p [%d], ns %p\n", qinfo, qinfo->id, ns);
	printk(KERN_INFO "flush qinfo[%d] stat %d\n", qinfo->id, status);
	if (ns) {
		printk(KERN_INFO "qinfo ns: %d\n", ns->id);
	}

	list_for_each_entry_safe(cmd_info, tmp, &qinfo->cmd_active, list) {

		DPRINT4("qinfo %p [%d], cmd_info %p, req %p\n", qinfo, qinfo->id,
						cmd_info, cmd_info->req);
		/**
		 * move bio to congested list.
		 * We are not going to concern with command information blocks
		 * at this time. The list will be recreated at later time.
		 */


		/*checking if the command was generated by the passthru ioctl*/
		if ((cmd_info->status == -1) && (cmd_info->type == ADMIN_CONTEXT)) {
			DPRINT4("flush_admin cmd from io queue :qinfo %p [%d], cmd_info %p, type 0x%x\n",
				qinfo, qinfo->id, cmd_info, cmd_info->type);

			cmd_info->status = EINTR;
			cmd_info->cmd_status=SF_SC_CMD_ABORT_REQ;
			cmd_info->cq_entry.SC=SF_SC_CMD_ABORT_REQ;
			wake_up(&cmd_info->waitq);
			continue;
		}

		if ((ns) && (cmd_info->ns != ns))
			continue;

			/*checking if the command is of type BIO. if the command was split to several nvme commands,
		only the base command is relevant as it contains the original bio request*/
		if ((bio = cmd_info->req) && cmd_info->type==BIO_CONTEXT) {
			if (status) {
				DPRINT4("qinfo %p [%d], Aborting bio %p, status %d\n",
						qinfo, qinfo->id, bio, status);
				bio->bi_error = status;
				bio_endio(bio);
				nvme_disk_stat_iodone(dev, cmd_info);
				qinfo->nrf_fq++;
				qinfo->nr_req--;
			} else {
				DPRINT4("qinfo %p [%d], requeuing bio %p\n",
						qinfo, qinfo->id, bio);
				bio_list_add_head(&qinfo->sq_cong, bio);
			}
		}
		cmd_info->status = 0;
		qinfo->timeout[cmd_info->timeout_id]--;
		if (cmd_info->sg_count) {
			dma_unmap_sg(dev->dma_dev, cmd_info->sg_list,
						 cmd_info->sg_count, DMA_FROM_DEVICE);
			cmd_info->sg_count = 0;
		}
		nvme_put_cmd(qinfo, cmd_info);
	}

	if (status) {
		nvme_flush_cong(qinfo, ns, status);
	}
}
#ifdef DEBUG_FLUSH_QUEUE
#pragma pop_macro("DPRINT4")
#endif

/**
 * @brief This function suspends all IO queues.
 *  		This function is called during error recovery to
 *  		suspend IO queue processing.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] u32 new_id Timer slot of queue timeout slot.
 *
 * @return void
 *  	None.
 *
 * @note It is assumed that dev lock is held by caller.
 */
static void
nvme_suspend_io_queues(struct nvme_dev *dev, u32 new_id)
{
	struct queue_info *qinfo;
	int i;

	DPRINT4("device %p [%d], suspending %d queues\n",
				dev, dev->instance, dev->que_count);
	for( i = 1; i <= dev->que_count  ; i++ ) {

		qinfo = dev->que_list[i];
		if (! qinfo)
			continue;

		DPRINT4("qinfo %p [%d], nr_req %d, nr_act %d\n", qinfo, qinfo->id,
					qinfo->nr_req, qinfo->nr_act);
		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		qinfo->timeout_id = new_id;
		DPRINT4("qinfo %p, timeout[%d]= %d\n",
			qinfo, new_id, qinfo->timeout[new_id]);

#ifdef NVME_DEBUG
		if (nvme_dbg & NVME_DEBUG_DUMP_Q)
			nvme_dump_list(qinfo);
#endif
		schedule_work_on(qinfo->cpu, &qinfo->wq_time);
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	}
}

/**
 * @brief This function resumes all suspended BIO requests for all IO queues.
 *  		This function is called during error recovery to
 *  		resume normal IO queue processing.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return void
 *  	None.
 */
static void
nvme_resume_io_queues(struct nvme_dev *dev)
{
	struct queue_info *qinfo;
	struct bio *bio;
	struct ns_info *ns;
	int i;

	DPRINT4("device %p [%d], resuming %d queues\n",
				dev, dev->instance, dev->que_count);
	for( i = 1; i <= dev->que_count  ; i++ ) {

		qinfo = dev->que_list[i];
		if (! qinfo)
			continue;
		DPRINT4("qinfo %p [%d], nr_req %d, nr_act %d\n", qinfo, qinfo->id,
				qinfo->nr_req, qinfo->nr_act);
		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		if (qinfo->flags & QUEUE_SUSPEND) {
			qinfo->flags &= ~QUEUE_SUSPEND;
			if (qinfo->cpu_cnt > 1) {
				if ((bio = bio_list_pop(&qinfo->sq_cong))) {
					DPRINT3("****** bio %p\n", bio);
					ns = bio->bi_bdev->bd_disk->private_data;
					if (0 == nvme_submit_request(qinfo, ns, bio, 0)) {
						qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
						continue;
					}
					DPRINT("failed submit nr_req %d, nr_act %d queued %p\n",
						   qinfo->nr_req, qinfo->nr_act, bio);
					bio_list_add_head(&qinfo->sq_cong, bio);
				}
			}
			schedule_work_on(qinfo->cpu, &qinfo->wq_work);
		}
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	}
}



/**
 * @brief This function Flushes all outstanding BIO requests for all IO queues.
 *  		This function is called during error recovery to
 *  		either terminate all pending BIO request or to
 *  		insert into congestion queue.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct ns_info *ns optional pointer to namespace information
 * @param[in] int status parameter specifies bio completion status
 *
 * @return void
 *  	None.
 * @note It is assumed that dev lock is held by caller.
 */
static void
nvme_flush_io_queues(struct nvme_dev *dev, struct ns_info *ns, int status)
{
	struct queue_info *qinfo;
	int i, id;

	DPRINT4("device %p [%d], ns %p, flushing %d queues\n",
				dev, dev->instance, ns, dev->que_count);
	for( i = 1; i <= dev->que_count  ; i++ ) {

		qinfo = dev->que_list[i];
		if (! qinfo)
			continue;
		DPRINT4("qinfo %p [%d], nr_req %d, nr_act %d\n", qinfo, qinfo->id,
					qinfo->nr_req, qinfo->nr_act);
		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		if (qinfo->nr_req || !list_empty(&qinfo->cmd_active)) {
			/* when nr_req=0 the list can still be not empty if a passthru command is placed in the IO queue*/
			nvme_flush_queue(qinfo, ns, status);
		}
#ifdef NVME_DEBUG
		else
			DPRINT4("skipping queue %p, [%d]\n", qinfo, qinfo->id);
#endif
		/**
		 * Clear timeout table.
		 */
		for (id=0; id<TIMEOUT_LIST; id++)
			qinfo->timeout[id] = 0;
		qinfo->nr_act = 0;  	/* reset active requests */
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	}
}

/**
 * @brief This function Resets an IO queue.
 *  		This function is called during error recovery to
 *  		reset an IO queue. The only way to reset an IO queue,
 *  		is to remove and recreate it.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] int restart flag inidicating controller is restarted
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note It is assumed that queue lock is held by caller.
 */
static int
nvme_reset_io_queue(struct queue_info *qinfo, int restart)
{
	int i, result = 0;
	struct cmd_info  *cmd_info;
	struct sub_queue_info *sqinfo = qinfo->sub_queue;
	struct nvme_dev *dev = qinfo->dev;

	if (!restart) {
		/**
		 * unregister submission and completion queue from hardware.
		 */
		sqinfo = qinfo->sub_queue;
		if (nvme_delete_sq(dev, sqinfo->id)) {
			EPRINT("Failed to destroy hardware IO submission queue %d\n",
								sqinfo->id);
		}
		if (nvme_delete_cq(dev, qinfo->id)) {
			EPRINT("Failed to destroy hardware IO completion queue %d\n",
								qinfo->id);
		}
	}

	qinfo->id_count = io_command_id_size;
	INIT_LIST_HEAD(&qinfo->cmd_free);
	INIT_LIST_HEAD(&qinfo->cmd_active);
	cmd_info = qinfo->cmd_list;
	for (i = 0; i < qinfo->id_count; i++) {

		spin_lock_init(&cmd_info->lock);
		cmd_info->cmd_id	 = i+1; 		/** 0 is reserved */
		init_waitqueue_head(&cmd_info->waitq);
		list_add_tail(&cmd_info->list, &qinfo->cmd_free);
		cmd_info++;
	}

	/**
	 * Reset Queue Info runtime information.
	 */
	qinfo->head = 0;
	qinfo->tail = 0;
	qinfo->phase = 1;
	qinfo->nr_act = 0;
	qinfo->timeout_id = -1;
	nvme_memset_64(qinfo->compq, 0LL,
			(sizeof(* qinfo->compq)/sizeof(u64)) * qinfo->qsize);
	result = nvme_create_cq(dev, qinfo, qinfo->id);
	if (result) {
		EPRINT("Failed to create hardware IO completion queue %d\n",
								qinfo->id);
		goto err_out;
	}

	sqinfo->head = 0;
	sqinfo->tail = 0;
	sqinfo->entries = sqinfo->qsize - 1;
	nvme_memset_64(sqinfo->subq, 0LL,
			(sizeof(* sqinfo->subq)/sizeof(u64)) * sqinfo->qsize);
	result = nvme_create_sq(dev, qinfo, sqinfo->id);
	if (result) {
		EPRINT("Failed to create hardware IO submission queue %d\n",
								sqinfo->id);
		nvme_delete_cq(dev, qinfo->id);
		goto err_out;
	}
err_out:
	return (result);
}


/**
 * @brief This function Restarts an IO queue.
 *  		This function is called during error recovery to
 *  		restart an IO queue.
 *
 *  		a. Abort all outstanding BIO requests.
 *  		b. Destroy hardware submission and completion queues.
 *  		c. Create hardware submission and completion queues.
 *  		d. Recreate command information free list.
 *  		e. Restart IO queue.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] int restart flag inidicating controller is restarted
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note It is assumed that dev lock is held by caller.
 */
static int
nvme_restart_io_queue(struct queue_info *qinfo, int restart)
{
	int result = 0;
	struct nvme_dev *dev = qinfo->dev;

	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	result = nvme_reset_io_queue(qinfo, restart);
	if (result) {
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		EPRINT("Failed IO queue reset qid %d\n", qinfo->id);
		return result;
	}
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	return result;
}


/**
 * @brief This function Restarts all IO queues.
 *  		This function is called during error recovery to
 *  		reset controller.
 *
 *  		For all IO queue:
 *  		a. Create hardware submission and completion queues.
 *  		b. Recreate command information free list.
 *  		c. Restart IO queue.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] int restart flag inidicating controller is restarted
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note It is assumed that dev lock is held by caller.
 */
static int
nvme_restart_all_queues(struct nvme_dev *dev, int restart)
{
	struct  queue_info *qinfo;
	int i, result = 0;

	for (i = 1; i <= dev->que_count; i++) {

		qinfo = dev->que_list[i];
		result = nvme_restart_io_queue(qinfo, restart);
		if (result) {
			DPRINT("Failed IO queue reset, terminating restart\n");
			break;
		}
	}
	return result;
}



static void nvme_flush_admin(struct  queue_info *qinfo)
{
	struct cmd_info *cmd_info, *tmp;
#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif


	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);

	DPRINT4("flush_admin:qinfo %p [%d], nr_req %d, nr_act %d", qinfo, qinfo->id,
								qinfo->nr_req, qinfo->nr_act);

	if (list_empty(&qinfo->cmd_active)) {
		DPRINT4("queue [%d] Command list Empty\n", qinfo->id);

	}
	else
		DPRINT4("queue [%d] Command list is not Empty\n", qinfo->id);

	list_for_each_entry_safe(cmd_info, tmp, &qinfo->cmd_active, list) {

		DPRINT4("flush_admin2:qinfo %p [%d], cmdInfo %p, context 0x%x", qinfo, qinfo->id,
										cmd_info, cmd_info->type);
		if ((cmd_info->status == -1) && (cmd_info->type == ADMIN_CONTEXT)) {

			cmd_info->status = EINTR;
			cmd_info->cmd_status=SF_SC_CMD_ABORT_REQ;
			cmd_info->cq_entry.SC=SF_SC_CMD_ABORT_REQ;
			wake_up(&cmd_info->waitq);

			mdelay(2);  				/* Delay a bit */
		}
	}

	qinfo->nr_act = 0;
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

}

/**
 * @brief This function Allocates a disk and attach ns.
 *  	This function is called from probe and rescan functions to
 *  	create a genric disk device and attach namespace.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct ns_info *ns Namespace information block
 * @param[in] flag indicating whether a lock is needed
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_attach_ns(struct nvme_dev *dev, struct ns_info *ns, int lock)
{
	struct 	request_queue *	tmp_queue;
	/*
	 * allocate a block layer queue to register
	 * our request entry point.
	 */
	tmp_queue = blk_alloc_queue(GFP_KERNEL);
	if (!tmp_queue) {
		EPRINT("Disk allocation failure\n");
		ns->disk = NULL;
		return(-ENOMEM);
	}
	if (lock) {
		dev->lock_func(&dev->lock, &dev->lock_flags);
	}

	if (ns->queue != NULL) {
		if (lock) {
			dev->unlock_func(&dev->lock, &dev->lock_flags);
		}
		EPRINT("Namespace %d alreay has a blk queue\n", ns->id);
		blk_cleanup_queue(tmp_queue);
		return (-EBUSY);
	}
	ns->queue = tmp_queue;
	DPRINT2("NS [%d], allocated Blk queue %p\n", ns->id, ns->queue);
	ns->queue->queuedata = ns;  	/** so we know which ns */
	ns->queue->queue_flags = (QUEUE_FLAG_DEFAULT |
				 (1 << QUEUE_FLAG_IO_STAT) |
				 (1 << QUEUE_FLAG_NOMERGES) |
				 (1 << QUEUE_FLAG_NONROT)
				);

	DPRINT2("NS [%d], nvmCmdSupport 0x%0x\n", ns->id, dev->nvmCmdSupport);
#if NVME_DISCARD
	/**
	 * BIO discard request depend upon controller identify data.
	 */
	if (dev->nvmCmdSupport & (1 << 2)) {
		ns->queue->queue_flags |= (1 << QUEUE_FLAG_DISCARD);
	}
#endif
	DPRINT2("NS [%d], queue %p, queue flags 0x%0lx\n", ns->id, ns->queue,
					ns->queue->queue_flags);
	/**
	 * Allocate a Generic disk device.
	 */
	ns->flags |= NS_ONLINE;
	if (lock) {
		dev->unlock_func(&dev->lock, &dev->lock_flags);
	}
	ns->disk = alloc_disk(MAX_MINORS);
	if (! ns->disk) {
		 EPRINT("Disk device node allocation failure\n");
		 blk_cleanup_queue(ns->queue);
		 ns->queue = NULL;
		 return(-ENOMEM);
	}
	DPRINT2("NS [%d], allocated Disk %p\n", ns->id, ns->disk);
	/**
	 * We are bypassing Block layer request handling.
	 * We are customizing our processing for non-rotating
	 * device by receiving IO requests directly from file
	 * system/BIO interface.
	 */
	blk_queue_make_request(ns->queue, nvme_make_request);
	DPRINT2("NS [%d], nvme_dev %p, mjr %d, mnr %d\n", ns->id, dev,
						dev->major,
					(MAX_MINOR_DEV*dev->instance)+
						(MAX_MINORS*(ns->id -1)));
	DPRINT2("NS [%d], dev %p, pci_dev %p\n", ns->id,
							dev, dev->pci_dev);

	/*
	 * Limit the size of one request
	 */
	blk_queue_max_hw_sectors(ns->queue, (transfer_size * 1024) >> 9);
	blk_queue_max_discard_sectors(ns->queue,
								  MAX_DISCARD_BLK >> (ns->lba_shift - 9));
	blk_queue_alignment_offset(ns->queue, (1 << ns->lba_shift));
	ns->queue->limits.discard_granularity = (1 << ns->lba_shift);
	blk_queue_logical_block_size(ns->queue, 1 << ns->lba_shift);
	/*
	 * Limit the number of segments passed to the driver in one request
	 */
	blk_queue_max_segments(ns->queue, (transfer_size * 1024)/PAGE_SIZE);

	/**
	 * Initialize disk device characteristics.
	 */
	ns->disk->major 	= dev->major;
	ns->disk->minors	= MAX_MINORS;
	ns->disk->first_minor   = (dev->instance*MAX_MINOR_DEV)+
						(MAX_MINORS*(ns->id -1));
	ns->disk->private_data  = ns;
	ns->disk->queue 		= ns->queue;
	ns->disk->fops  		= &nvme_fops;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
	ns->disk->driverfs_dev  = &dev->pci_dev->dev;
#endif

	/**
	 * Assign disk name.
	 */
	sprintf(ns->disk->disk_name, "%s%dn%d",
				DEV_NAME, dev->instance, ns->id);


	DPRINT2("NS [%d], device name %s, minor %d\n",
				ns->id, ns->disk->disk_name,
				ns->disk->first_minor);
	DPRINT2("NS [%d], Capacity %lld\n",
				ns->id, ns->block_count << (ns->lba_shift - 9));

	/**
	 * NVMe block size may be multiple of OS block size.
	 * Make the appropriate adjusment and set disk capacity.
	 */
	set_capacity(ns->disk, ns->block_count << (ns->lba_shift - 9));
	NPRINT("NS [%d], Adding Disk \"%s\" [%d:%d] %lldGB\n",
				ns->id, ns->disk->disk_name,
				ns->disk->major, ns->disk->first_minor,
				(u64)get_capacity(ns->disk)/(2*1024*1024));

	add_disk(ns->disk);
	return 0;
}


/**
 * @brief This function release Namespace information block
 *  	This function is called from probe or remove function to
 *  	release a Namespace.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] ns, pointer to Namespace information block
 *
 * @return none.
 */
static void
nvme_free_ns(struct nvme_dev *dev, struct ns_info * ns)
{
	DPRINT2("Releasing Namespace [%d] %p\n", ns->id, ns);
	list_del(&ns->list);
	kfree(ns);
}


/**
 * @brief This function allocate a Namespace.
 *  	This function is called from probe and rescan functions to
 *  	create a Namespace.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] int ns_id Namespace ID
 * @param[in] flag indicating whether a lock is needed
 *
 * @return This function returns struct ns_info *, otherwise NULL
 */
static struct ns_info *
nvme_alloc_ns(struct nvme_dev *dev, int ns_id, int lock)
{
	int i, result;
	u32 lba_format;
	struct ns_info *ns;
	struct iden_namespace *ident;

	result = nvme_get_identify(dev, ns_id, dev->identify);
	if (result) {
		EPRINT("Failed get NS Identify data.\n");
		return (NULL);
	}

	ident = (struct iden_namespace *)dev->identify;

	ns = kzalloc(sizeof(*ns), GFP_KERNEL);
	if (! ns) {
		EPRINT("Failed NS memory allocation.\n");
		return (NULL);
	}

	DPRINT2("NS [%d], size %llu, lba_fmt 0x%02x, Formats 0x%02x\n",
				ns_id, ident->size,
				ident->fmtLbaSize, ident->numLbaFmt);
	DPRINT2("NS [%d], feature 0x%02x, Prot Cap 0x%02x, Prot Set 0x%02x\n",
				ns_id, ident->feat,
				ident->dataProtCap, ident->dataProtSet);
	if(ident->capacity==0)
	{
		EPRINT("NS [%d] capacity is zero\n",ns_id);
		kfree(ns);
		return (NULL);
	}
	for (i = 0; i <= ident->numLbaFmt; i++) {
		DPRINT2("supported LBA format 0x%08x\n",
					*(u32 *)&ident->lbaFmtSup[i]);
	}
	lba_format = *(u32 *)&ident->lbaFmtSup[ident->fmtLbaSize & 0x0F];
	DPRINT2("LBA format 0x%08x\n", lba_format);
	DPRINT2("Meta Data Capability 0x%02x\n", ident->metaDataCap);
	DPRINT2("LBA Data Prot Cap/Set 0x%02x/0x%02x\n",
				ident->dataProtCap, ident->dataProtSet);

	/* The init_module path does not need a lock */
	if (lock) {
		dev->lock_func(&dev->lock, &dev->lock_flags);
	}

	INIT_LIST_HEAD(&ns->list);

	ns->id = ns_id;
	ns->block_count = ident->size;
	ns->lba_shift = (lba_format >> 16) & 0x0F;
	ns->feature = ident->feat;

	/* Bit 4 of fmtLbaSize indicates type MetaData buffer.
	 *  Bit 4 Set, indicates 8 bytes meta data at end of buffer
	 *  Bit 4 Clear, indicated a seperate contiguous buffer.
	 */
	ns->metasize = lba_format & 0x0FFFF;
	ns->fmtLbaSize = ident->fmtLbaSize;
	ns->dataProtCap = ident->dataProtCap;
	ns->dataProtSet = ident->dataProtSet;
	ns->dev = dev;
	ns->vendor =  *(__u64 *)ident->vendor;

	DPRINT2("NS [%d] %p, adding to dev list %p, lba size %u\n",
			ns->id, ns, &dev->ns_list, (1 << ns->lba_shift));

	list_add_tail(&ns->list, &dev->ns_list);
	if (lock) {
		dev->unlock_func(&dev->lock, &dev->lock_flags);
	}

	return (ns);
}


#if USE_NS_ATTR
/**
 * @brief This function retrievs Namespace attributes
 *  	This function is called during Namespace discovery to
 *  	retrieve the Namespace attributes.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] ns_id, Namespace id to retrieve attributes of.
 * @param[in] ret, Optional pointer to 4096 bytes buffer to receive data.
 *
 * @return int 0 if successful, otherwise Error Code
 */
static int
nvme_ns_attr(struct nvme_dev *dev, int ns_id, struct ns_info *ns)
{
	struct cq_entry cq_entry;
	struct lba_range *range;
	dma_addr_t buff_phy;
	void	*buff;
	int i, count, result;

	/**
	 * allocate DMA buffer to retrieve ns attributes.
	 */
	buff = dma_alloc_coherent(dev->dma_dev, 4096, &buff_phy, GFP_KERNEL);
	if (! buff) {
		return (-ENOMEM);
	}
	count = 4096/sizeof(*range);
	result = nvme_get_feature(dev, ns_id, FTR_ID_LBA_RANGE_TYPE, count,
				(struct nvme_prp *)&buff_phy, &cq_entry);

	if (result < 0) {
		EPRINT("Failed Get LBA Range ns %d\n", ns_id);
		goto failed_ns_attr;
	}
#ifdef NVME_DEBUG
		if (nvme_dbg & NVME_DEBUG_DUMP) {
			int i;
			u32 *ptr;
		ptr = (u32 *)buff;
		for (i=0; i<32; i += 4) {
			DPRINT("%02x: %08x %08x %08x %08x\n", i,
			ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
		}
		}
#endif
	count = (cq_entry.param.cmdSpecific & 0x3F)+1;
	DPRINT2("Number of LBA ranges %d\n", count);
	range = (struct lba_range *)buff;
	for(i = 0; i < count; i++) {
		DPRINT2("Range %d, type %d, attr %0x, start %lld, size %llu\n",
				i, range->type, range->attributes,
				range->start, range->size);
		if (0 == (range->attributes & 1))
			ns->flags |= NS_READONLY;
		if (range->type & (1 << 9)) {   	/** Hidden range ? */
			ns->block_count = range->start;
			break;
		}
	}
	DPRINT2("ns block count %lld\n", ns->block_count);

failed_ns_attr:
	dma_free_coherent(dev->dma_dev, 4096, buff, buff_phy);
	return  (result);
}
#endif

/**
 * @brief This function Restarts Controller
 *  		This function is called during error recovery to
 *  		restart controller. All controller activities are
 *  		halted and pending IO requests placed on congestion
 *  		list. the controler is reset and all hwardware
 *  		resources reinitialized.
 *
 *  		a. Abort all outstanding BIO requests.
 *  		b. Destroy all submission and completion queues.
 *  		c. Initialize Admin Queue.
 *  		c. Reset controller.
 *  		c. Create all submission and completion queues.
 *  		d. Recreate command information free list.
 *  		e. Restart IO queues.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_hw_reset(struct nvme_dev *dev)
{
	int nr_io_queues;
	int i, result = 0;
	struct  queue_info *qinfo;

	EPRINT("Restarting Controller %d\n", dev->instance);
	dev->lock_func(&dev->lock, &dev->lock_flags);
	if (dev->state == NVME_STATE_RESET) {
		dev->unlock_func(&dev->lock, &dev->lock_flags);
		return (-EAGAIN);
	}
	dev->state = NVME_STATE_RESET;

	/**
	 * Inorder to reset an IO queue, we need to delete and
	 * recreate the IO queue. This is required to quiesce
	 * IO completions in progress bebfore we can reset hardware.
	 */


	for( i = 1; i <= dev->que_count  ; i++ ) {

		qinfo = dev->que_list[i];
		if (! qinfo)
			continue;
		/**
		 * Suspend all activities on this queue.
		 */
		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		qinfo->flags |= QUEUE_SUSPEND;
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	}
	result = nvme_hw_start(dev);		/* reset controller */
	if (result) {
		EPRINT("Controller Reset Failure.\n");
		EPRINT("Offlining Controller.\n");
		dev->unlock_func(&dev->lock, &dev->lock_flags);
		goto err_out;
	}

	//nvme_flush_admin(dev->admin_queue);

	nvme_flush_io_queues(dev, NULL, 0); /* flush BIOs to congestion Q */
	dev->unlock_func(&dev->lock, &dev->lock_flags);

	/**
	 * As part of reset, we need to verify controller configuration
	 * is still valid with existing driver configuration parameters.
	 */
	result = nvme_get_ctrl_identify(dev);
	if (result) {
		EPRINT("Controller Identify Failure.\n");
		EPRINT("Offlining Controller.\n");
		goto err_out;
	}

	/* Double check number of queues to be same as nr_io_queues */
	nr_io_queues = dev->que_count;
	result = nvme_request_queues(dev, &nr_io_queues);
	if (result) {
		EPRINT("Failed to allocate hardware IO Queue error.\n");
		goto err_out;
	}
	if (nr_io_queues != dev->que_count) {
		EPRINT("IO queue configuration changed!!! 0x%x 0x%x\n",nr_io_queues,dev->que_count);
		EPRINT("Unsupported configuration, failing controller.\n");
		goto err_out;
	}
	DPRINT("Got %d hw IO queues\n", nr_io_queues);

	result = nvme_restart_all_queues(dev, TRUE);
	if (result) {
		EPRINT("Failed to restart IO queue %0x.\n", result);
		goto err_out;
	}

	/**
	 * reinitiate AEN requests.
	 */
#if SEND_AEN
	nvme_send_aen(dev);
#endif
	/**
	 * Validate existing Namespace and if additional Namespaces
	 * discovered, create a ns information block and create disk device.
	 */
	DPRINT1("number of Namespaces %d\n", dev->ns_count);
	if (dev->ns_count
	 != min(dev->identify->numNmspc, (u32) MAX_NAMESPACES)) {
		EPRINT("Number Namespaces changed since last discovery %d\n",
						dev->identify->numNmspc);
		dev->ns_count = dev->identify->numNmspc;
		rms_check_ns_count(dev);
	}

	/**
	 * Device is operational, restart timer and kick restart
	 * IO queue processing.
	 */
	dev->lock_func(&dev->lock, &dev->lock_flags);
	dev->state = NVME_STATE_OPERATIONAL;
	mod_timer(&dev->timer, (jiffies + TIMEOUT_FREQ));
	nvme_resume_io_queues(dev);
	dev->unlock_func(&dev->lock, &dev->lock_flags);

	DPRINT1("Exit %d\n", result);
	return result;

err_out:
	dev->lock_func(&dev->lock, &dev->lock_flags);
	dev->state = NVME_STATE_FAILED;
	nvme_flush_io_queues(dev, NULL, -EIO);  	/* Flush all requests */
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	return (-EIO);
}


#define NVME_BIOVEC_PHYS_MERGEABLE(vec1, vec2) \
		((bvec_to_phys((vec1)) + (vec1)->bv_len) == bvec_to_phys((vec2)))

#define NVME_BIOVEC_NOT_VIRT_MERGEABLE(vec1, vec2) \
		   ((vec2)->bv_offset || \
						(((vec1)->bv_offset + (vec1)->bv_len) % PAGE_SIZE))

/**
 * @brief This function called to generate dataset management LBA range.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] struct cmd_info *cmd_info pointer to command information block
 * @param[in] struct ns_info *ns pointer to Namespace information block
 * @param[in] struct bio *bio pointer to block io request
 *
 * @return None.
 *
 * Note:	It is assumed that queue lock is held by caller.
 */
static void
nvme_process_lba_range(struct queue_info *qinfo, struct cmd_info *cmd_info,
					struct ns_info *ns, struct bio *bio)
{
#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif
	struct nvme_prp *prps;
	struct nvme_dataset_mgmt_data *rlba;

	DPRINT2("length %d, LBA %0llx\n", bio->bi_iter.bi_size, (u64)bio->bi_iter.bi_sector);

	prps = cmd_info->nvme_cmd.header.prp;
	prps->addr = cmd_info->prp_phy;
	DPRINT2("LBA RANGE PRP %016llx\n",
			cmd_info->nvme_cmd.header.prp[0].addr);

	rlba = (struct nvme_dataset_mgmt_data *)cmd_info->prps;
	nvme_memset_64(rlba, 0LL, sizeof(* rlba)/sizeof(u64));
	rlba->numLBA = cpu_to_le32((bio->bi_iter.bi_size >> ns->lba_shift)-1);
	rlba->startLBA = cpu_to_le64(bio->bi_iter.bi_sector >> (ns->lba_shift - 9));
	cmd_info->count = bio->bi_iter.bi_size; 		/** Error log Info */

#ifdef NVME_DEBUG
	if (nvme_dbg & NVME_DEBUG_DUMP)
	{
		int i;
		u32 *ptr;
		ptr = (u32 *)rlba;
		for (i=0; i<sizeof(* rlba)/sizeof(u32); i += 4) {
			DPRINT2("%02x: %08x %08x %08x %08x\n", i,
					ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
		}
		}
#endif
}

/**
 * @brief This function called to generate scatter gather list
 *
 * Note:	It is assumed that queue lock is held by caller.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] struct cmd_info *cmd_info pointer to command information block
 * @param[in] struct bio *bio pointer to block io request
 * @param[in] enum dma_data_direction dma_dir Data direction
 *
 * @return This function returns int length, number of bytes transferred.
 *
 */
static int
nvme_process_sglist(struct queue_info *qinfo, struct cmd_info *cmd_info,
			struct bio *bio, enum dma_data_direction dma_dir)
{
	int nsegs;
	size_t  length;
	int first = 1;
	struct bio_vec bvec, bvprv = {NULL,0,0};
	struct nvme_dev *dev = qinfo->dev;
	struct scatterlist *sg = NULL;

	nsegs = bio_phys_segments(cmd_info->ns->queue, bio);
	if (nsegs > max_prp_list) {
		nsegs = max_prp_list;
	}
	sg_init_table(cmd_info->sg_list, nsegs);
	nsegs = 0;
	length = 0;

	DPRINT6("bio %p, bvec idx %d, vcnt %d\n",
				bio, bio->bi_iter.bi_idx, bio->bi_vcnt);

	/**
	 * Proceed through sg list entries and try to merge two
	 * contiguous segments.
	 */
	bio_for_each_segment(bvec, bio, cmd_info->cmd_bvec_iter) {

		DPRINT6("bvec [%d] phys %llx, offset %x, length %d\n",
				iter.bi_idx, bvec_to_phys(bvec), bvec->bv_offset, bvec->bv_len);

		if (((length+bvec.bv_len) > (transfer_size * 1024)) || (nsegs == max_prp_list)) {
			DPRINT6("**** reached maximum xfer length %lld\n", (u64)length);
			break;
		}
		if (!first && NVME_BIOVEC_PHYS_MERGEABLE(&bvprv, &bvec)) {
			sg->length += bvec.bv_len;
			DPRINT6("***** Merged entries sg->length %d\n", sg->length);
		} else {
			if (!first && NVME_BIOVEC_NOT_VIRT_MERGEABLE(&bvprv, &bvec)) {
				DPRINT6("***** Unable to Merge v1 0x%x [%x] v2 0x%x [%x]\n",
						bvec.bv_offset, bvec.bv_len,
						bvprv.bv_offset, bvprv.bv_len);
			break;
		}
		sg = sg ? sg+1 : cmd_info->sg_list;

		sg_set_page(sg, bvec.bv_page, bvec.bv_len, bvec.bv_offset);
			DPRINT6("sgl [%d] page %p, phys 0x%llx 0x%x %x\n",
				nsegs, sg->page, bvec_to_phys(bvec),
				bvec.bv_offset, bvec.bv_len);
			nsegs++;
		}
		length += bvec.bv_len;
		bvprv = bvec;
		first = 0;
	}
	DPRINT6("processed %d of %d\n", i, bio->bi_vcnt);

	if (!sg) {
		EPRINT("bio %p, cmd_info %p, nsegs %d\n",
					bio, cmd_info, nsegs);
		panic("null sg");
	}
	sg_mark_end(sg);
	DPRINT6("nsegs %d\n", nsegs);
	if (dma_map_sg(qinfo->dev->dma_dev, cmd_info->sg_list,
						nsegs, dma_dir) == 0) {
		return -ENOMEM;
	}
	sg = cmd_info->sg_list;
#ifdef NVME_DEBUG
	for (i=0; i < nsegs; i++) {
		DPRINT6("sg[%d]: sg->addr %llx, off %d, length %d, dma_len %d\n",
				i, (u64)sg->dma_address, sg->offset,
				sg->length, sg->dma_length);
		sg++;
	}
#endif
	cmd_info->sg_count  = nsegs;
	cmd_info->count = length;
	DPRINT6("length %lld\n", (u64)length);
	nvme_process_prps(qinfo, cmd_info);
	return length;
}


/**
 * @brief This function prepares a prp list.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] struct cmd_info *cmd_info pointer to command information blcck
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note It is assumed that queue lock is held by caller.
 */
static void
nvme_process_prps(struct queue_info *qinfo, struct cmd_info *cmd_info)
{
#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif
	struct nvme_prp *prps;
	struct scatterlist *sg;
	int length, dma_len, offset;
	u64 dma_addr;

	sg =	   cmd_info->sg_list;
	prps =     cmd_info->nvme_cmd.header.prp;
	length =   cmd_info->count;

	dma_addr = sg_dma_address(sg);
	dma_len =  sg_dma_len(sg);
	offset =   sg->offset;

	DPRINT6("length %d, dma_addr %0llx, offset %0x, dma_len %d\n",
				length,  dma_addr, offset, dma_len);

	prps->addr = dma_addr;
	prps++;
	dma_addr += (PAGE_SIZE - offset);
	dma_len -=  (PAGE_SIZE - offset);
	length -=   (PAGE_SIZE - offset);

	/**
	 * If there is only a single entry of one page or less.
	 */
	if (length <= (int)(PAGE_SIZE)) {

		if (unlikely((length > 0) && (dma_len <= 0))) {
			sg = sg_next(sg);
			dma_addr = sg_dma_address(sg);
		}
		prps->addr = dma_addr;
		DPRINT6("No List PRP1 %016llx, PRP2 %016llx\n",
			cmd_info->nvme_cmd.header.prp[0].addr,
			cmd_info->nvme_cmd.header.prp[1].addr);
		return;
	}

	/**
	 * More than a single entry, should use prp list.
	 */
	prps->addr = cmd_info->prp_phy;
	prps = cmd_info->prps;
	DPRINT6("List PRP1 %016llx, PRP2 %016llx, length %d\n",
			cmd_info->nvme_cmd.header.prp[0].addr,
			cmd_info->nvme_cmd.header.prp[1].addr,
			length);

	/**
	 * Process the rest of sglist.
	 */
	while (length > 0) {
		if (dma_len > 0) {
			prps->addr = dma_addr;
			DPRINT3("PRP list [%p] = %016llx\n", prps, prps->addr);
			prps++;
			dma_addr +=  PAGE_SIZE;
			dma_len -=   PAGE_SIZE;
			length -=    PAGE_SIZE;
		} else {
			sg = sg_next(sg);
			dma_addr = sg_dma_address(sg);
			dma_len = sg_dma_len(sg);
		}
	}
}

/**
 * @brief This function submits a bio request to submission queue.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] struct ns_info *ns pointer to Namespace information block
 * @param[in] struct bio *bio pointer to Block io request
 * @param[in] int retries, command retry count.
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note It is assumed that queue lock is held by caller.
 */
static int
nvme_submit_request(struct queue_info *qinfo, struct ns_info *ns,
				struct bio *bio, int retries)
{
	struct nvme_dev * dev = qinfo->dev;
	struct sub_queue_info *sqinfo = qinfo->sub_queue;
	struct cmd_info * cmd_info, *base_info;
	struct nvme_cmd * cmd;
	enum dma_data_direction dma_dir;
	int length, status = 0;

	if ((dev->state != NVME_STATE_OPERATIONAL) || /* Device operational? */
		(qinfo->flags & QUEUE_SUSPEND)) {   	/* IO queue !Ready? */
		DPRINT2("qinfo %p [%d] failing request %p, state %d, flags %d\n",
				qinfo, qinfo->id, bio,
				dev->state, qinfo->flags);
		return -1;
	}

	dev->sub_req++;
#ifdef NVME_DEBUG
	if ((qinfo->cpu != -1) && (qinfo->cpu != smp_processor_id())) {
	   DPRINT2("cpu mismatch qinfo cpu %d, smp_id %d\n",
				qinfo->cpu, smp_processor_id());
	}
#endif
	base_info = NULL;

	do {
		if (qinfo->nr_act >= max_io_request-1) {
			status = -EBUSY;
			break;
		}

		cmd_info = nvme_get_cmd(qinfo);
		if (! cmd_info) {
			status = -ENOMEM;
			break;
		}
		if (!base_info) {
		/**
		 * This is a base command for case that we have to send
		 * multiple nvme_cmds to satisfy a request.
		 *
		 * Note: it is very important for queue lock held for this
		 *   procedure. This is since we can receive a completion
		 *   interrupt while still  preparing nvme_cmds for the
		 *   request.
		 */
			base_info = cmd_info;
			base_info->req = bio;
			base_info->cmd_status = 0;
			base_info->req_length = 0;
			base_info->cmd_bvec_iter = bio->bi_iter;
			base_info->cmd_retries = retries;
		} else {
			cmd_info->req = NULL;
		}
		cmd_info->cmd_count = 0;
		cmd_info->cmd_base = base_info;
		cmd_info->ns = ns;

		/**
		 * Make sure we have room in submission queue.
		 * @todo: We need to take care of shared mode. In shared mode,
		 *  	Multiple CPUs may try to use the same submission
		 *  	queue.
		 */
		if (!sqinfo->entries) {
			nvme_put_cmd(qinfo, cmd_info);
			status = -ENOMEM;
			if (base_info == cmd_info)
				base_info = NULL;
			break;
		}
		cmd = &sqinfo->subq[sqinfo->tail];
		nvme_memset_64(cmd, 0LL, sizeof(*cmd)/sizeof(u64));

		/**
		 * process bio sglist and setup prp list.
		 */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
		if (unlikely(bio->bi_rw & BIO_REQ_DISCARD)) {
#else
		if (unlikely(bio_op(bio) & BIO_REQ_DISCARD)) {
#endif
			DPRINT2("bio %p, ns %d, DISCARD size %u\n",
						bio, ns->id, bio->bi_iter.bi_size);
			length = bio->bi_iter.bi_size;
			cmd->header.opCode = NVM_CMD_DATASET_MGMNT;
			cmd->header.namespaceID = cpu_to_le32(ns->id);
			nvme_process_lba_range(qinfo, cmd_info, ns, bio);
			cmd->header.prp[0] = cmd_info->nvme_cmd.header.prp[0];
			cmd->cmd.dataset.numRanges = 0; 	/* single range */
			cmd->cmd.dataset.attribute = (1 << 2);  /* Deallocate */
			/* force getting out of the loop */
			base_info->cmd_bvec_iter.bi_size = 0;
		} else
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
			if (unlikely(bio->bi_rw & BIO_REQ_FLUSH)) {
#else
			if (unlikely(bio_op(bio) & BIO_REQ_FLUSH)) {
#endif
				DPRINT6("bio %p, ns %d, FLUSH cache...\n",bio, ns->id);
				cmd->header.opCode = NVM_CMD_FLUSH;
				cmd->header.namespaceID = cpu_to_le32(ns->id);
				length = 0;
			} else {
				if (!bio->bi_iter.bi_size) {
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
					EPRINT("bio %p, rw %lx, vcnt %d, len %d\n",
						   bio, bio->bi_rw, bio->bi_vcnt, bio->bi_iter.bi_size);
#else
					EPRINT("bio %p, rw %x, vcnt %d, len %d\n",
						   bio, bio->bi_opf, bio->bi_vcnt, bio->bi_iter.bi_size);
#endif
					nvme_put_cmd(qinfo, cmd_info);
					qinfo->nr_req--;
					bio->bi_error = -EINVAL;
					bio_endio(bio);
					status = 0;
					break;
				}
				if (bio_data_dir(bio)) {
					DPRINT6("bio %p, ns %d, WRITE %d\n",
							bio, ns->id, bio->bi_iter.bi_size);
					cmd->header.opCode = NVM_CMD_WRITE;
					dma_dir = DMA_TO_DEVICE;
				} else {
					DPRINT6("bio %p, ns %d, READ %d\n",
							bio, ns->id, bio->bi_iter.bi_size);
					cmd->header.opCode = NVM_CMD_READ;
					dma_dir = DMA_FROM_DEVICE;
				}
				cmd->header.namespaceID = cpu_to_le32(ns->id);
				length = nvme_process_sglist(qinfo, cmd_info, bio, dma_dir);
				if (base_info != cmd_info) {
					base_info->cmd_bvec_iter.bi_idx += cmd_info->cmd_bvec_iter.bi_idx;
				}

				cmd->header.prp[0] = cmd_info->nvme_cmd.header.prp[0];
				cmd->header.prp[1] = cmd_info->nvme_cmd.header.prp[1];
				cmd->cmd.read.numLBA = cpu_to_le16((length >> ns->lba_shift)-1);
				cmd->cmd.read.startLBA = cpu_to_le64(bio->bi_iter.bi_sector >>
														 (ns->lba_shift - 9));
				DPRINT6("ns %d, numLBA %d, start %lld, lba_shft %d\n",
						ns->id, cmd->cmd.read.numLBA,
						cmd->cmd.read.startLBA,
						ns->lba_shift);
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
				if (bio->bi_rw & BIO_REQ_RAHEAD)
#else
				if (bio_op(bio) & BIO_REQ_RAHEAD)
#endif
					cmd->cmd.read.datasetMgmnt = 7;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
				if (bio->bi_rw & (BIO_REQ_FAILFAST_DEV | BIO_REQ_RAHEAD))
#else
				if (bio_op(bio) & (BIO_REQ_FAILFAST_DEV | BIO_REQ_RAHEAD))
#endif
					cmd->cmd.read.limitedRetry = 1;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
				if (bio->bi_rw & BIO_REQ_FUA)
#else
				if (bio_op(bio) & BIO_REQ_FUA)
#endif
					cmd->cmd.read.forceUnitAccess = 1;
			}
			cmd->header.cmdID = cmd_info->cmd_id;
			cmd_info->timeout_id = dev->timeout_id;
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
			if (!(bio->bi_rw & BIO_REQ_DISCARD)) {
#else
			if (!(bio_op(bio) & BIO_REQ_DISCARD)) {
#endif
				qinfo->timeout[cmd_info->timeout_id]++;
			}
#if DO_IO_STAT
			cmd_info->start_time = jiffies;
#endif
			cmd_info->type = BIO_CONTEXT;
			cmd_info->status = -1;
			/**
			 * Update commands sent for request.
			 */
			base_info->cmd_count++;
			base_info->req_length += cmd_info->count;

			DPRINT6("Sub queue[%d] doorbell %p->[%d]\n", sqinfo->id,
					sqinfo->doorbell, sqinfo->tail);

#ifdef NVME_DEBUG
			if (nvme_dbg & NVME_DEBUG_DUMP) {
				int i;
				u32 *ptr;
				ptr = (u32 *)cmd;
				for (i=0; i<sizeof(struct nvme_cmd)/sizeof(u32); i += 4) {
					DPRINT("%02x: %08x %08x %08x %08x\n", i,
						   ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
				}
			}
#endif
			sqinfo->tail++;
			if (sqinfo->tail >= sqinfo->qsize)
				sqinfo->tail = 0;
			writel(sqinfo->tail, sqinfo->doorbell);
			sqinfo->entries--;
	} while (base_info->cmd_bvec_iter.bi_size);

	/**
	 * reset cmd_base if we managed it with a single cmd.
	 * Also, we have processed any port of BIO request, return
	 * success to prevent requeue of BIO.
	 */
	if (base_info) {
		if (base_info->cmd_count == 1)
			base_info->cmd_base = NULL;
		status = 0;
		DPRINT2("bio %p, base_info %p, cmd_count %d\n",
			bio, base_info, base_info->cmd_count);
		nvme_disk_stat_in(dev, base_info, bio);
	}
	DPRINT2("bio %p, status %d\n", bio, status);
	return status;
}

/**
 * @brief This function Processes congestion queue.
 *  	This function is called from worker thread or from 2nd half DPC
 *  	to process congested requests when additional submission
 *  	queue entries are available.
 *
 *  	a. Get the first entry from congestion list
 *  	b. Process BIO.
 *  	c. continue until run out of resources or submission queue entry
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 *
 * @return void
 *
 * @note It is assumed that queue lock is held by caller.
 */
static void
nvme_process_cong(struct queue_info *qinfo)
{
	struct bio *bio;
	struct ns_info *ns;
#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif

	DPRINT3("****** qinfo %p [%d], nr_req %d, nr_act %d\n", qinfo,
				qinfo->id, qinfo->nr_req, qinfo->nr_act);
	while ((bio = bio_list_pop(&qinfo->sq_cong))) {

		DPRINT6("****** bio %p\n", bio);
		ns = bio->bi_bdev->bd_disk->private_data;
		if (nvme_submit_request(qinfo, ns, bio, 0)) {
			DPRINT("*** SubQ Full/MaxIO, nr_req %d, nr_act %d queued %p\n",
				   qinfo->nr_req, qinfo->nr_act, bio);
			bio_list_add_head(&qinfo->sq_cong, bio);
			break;
		}
	}
}

/**
 * @brief This function translates error code and logs the error.
 *
 * @param[in] struct nvme_dev *dev pointer to NVME device data context.
 * @param[in] struct cq_entry *cq_entry optional pointer to cq_entry
 *
 * @return void None.
 */
static void
nvme_show_error(struct nvme_dev *dev, struct cq_entry *cq_entry)
{
	char **str;
	u32   sts;

	if (err_type_string[cq_entry->SCT]) {
		EPRINT("Error type [%02x]: \"%s\"\n",
				cq_entry->SCT, err_type_string[cq_entry->SCT]);
	} else {
		EPRINT("Error type [%02x] \"unknown\"\n", cq_entry->SCT);
	}
	sts = cq_entry->SC;

	str = err_tbl[cq_entry->SCT << 1];
	if (sts >= 0x80) {
		str = err_tbl[(cq_entry->SCT << 1)+1];
		sts -= 0x80;
	}
	EPRINT("Error status [%04x]: \"%s\"\n", cq_entry->SC, str[sts]);
}

/**
 * @brief This function logs an error on system console.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] struct cmd_info *cmd_info pointer to command information block
 * @param[in] struct cq_entry *cq_entry optional pointer to cq_entry
 *
 * @return void None.
 */
static void
nvme_log_err(struct queue_info *qinfo, struct cmd_info *cmd_info,
						struct cq_entry *cq)
{
	struct nvme_dev *dev = qinfo->dev;
	struct cq_entry *cq_entry;

	if (cq)
		cq_entry = cq;
	else
		cq_entry = &cmd_info->cq_entry;

	switch(cmd_info->type) {

		case BIO_CONTEXT:   /* Normal Error log */
		{
			struct ns_info *ns = cmd_info->ns;

			if (!cq_entry->SC)
				break;

			EPRINT("IO Error [%x:%x] namespace %d, block %lld, "
				   "count %d\n",
				   cq_entry->SCT, cq_entry->SC, ns->id,
				(u64)cmd_info->cmd_bvec_iter.bi_sector,
				cmd_info->count >> (ns->lba_shift - 9));

			nvme_show_error(dev, cq_entry);
			break;
		}

		case ERR_CONTEXT:   /* Error Page log */
		{
			struct error_log * err_log = cmd_info->req;

			nvme_report_error(dev, err_log);
			break;
		}
		default:
		{

			if (!cq_entry->SC)
				break;

			EPRINT("%s request error completion [%x:%x]\n",
				   context_type_string[cmd_info->type & 0x0F],
				   cq_entry->SCT, cq_entry->SC);

			nvme_show_error(dev, cq_entry);
			break;
		}
	}
}


/**
 * @brief This function Processes posted completions in a completion queue.
 *  	This function is called from ISR or from 2nd half DPC to
 *  	process completions posted by firmware.
 *
 *  	a. Check current completion head entry phase
 *  	b. Get the corresponding command info from queue list
 *  	c. Determine request type and status
 *  	d. Complete request according to its type.
 *  	e. Increment completion ptr and reset if wrapped around.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 *
 * @return irqreturn_t, result of interrupt processing (IRQ_HANDLED)
 */
static irqreturn_t
nvme_process_cq(struct queue_info *qinfo)
{
	u16    head, phase, sq_head = 0;
	u32 status;
	irqreturn_t irq_status;
	struct cq_entry *cq_entry;
	struct nvme_dev *dev = qinfo->dev;

	head = qinfo->head;
	phase = qinfo->phase;
	dev->complete_q_cnt++;


#ifdef NVME_DEBUG
	if ((qinfo->cpu != -1) && (qinfo->cpu != smp_processor_id())) {
	   DPRINT("cpu mismatch expected cpu %d, actually cpu %d\n",
				qinfo->cpu, smp_processor_id());
	}
#endif

	for (;;)
	{
		struct cmd_info *cmd_info, *base_info;
		struct bio *bio;

		cq_entry = &qinfo->compq[head];
		if (cq_entry->phaseTag != phase)
		break;

#ifdef NVME_DEBUG
		if (nvme_dbg & NVME_DEBUG_DUMP_CE)
		{
			int  i;
			u32 *ptr;
			ptr = (u32 *)cq_entry;
			for (i=0; i<sizeof(struct cq_entry)/sizeof(u32); i += 4) {
				DPRINT("%02x: %08x %08x %08x %08x\n", i,
				ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
			}
			}
#endif

#ifdef NVME_DEBUG
		/**
		 * Verify command context.
		 */
		if ( (!cq_entry->cmdID) || (cq_entry->cmdID > qinfo->id_count) ) {
			EPRINT("Invalid Command ID %d\n", cq_entry->cmdID);
			goto next_entry;
		}
#endif

		cmd_info = &qinfo->cmd_list[cq_entry->cmdID-1];
		status = cq_entry->SCT << 8 | cq_entry->SC;
		sq_head = (u16)cq_entry->sqHdPtr;

#ifdef NVME_DEBUG
		nvme_idle_count = 0;	/* @todo - debug only*/
		DPRINT6("cpu %d ==> CompQ %p [%d] cmd %d ->Hd %d, ph %d\n",
					smp_processor_id(),
						qinfo, qinfo->id, cq_entry->cmdID,
						cq_entry->sqHdPtr, phase);
		DPRINT6("cmd_info %p base %p [%04x/%d] -> TYPE %02x STS %02x\n",
					cmd_info, cmd_info->cmd_base,
					cq_entry->cmdID, cq_entry->cmdID,
					cq_entry->SCT, cq_entry->SC);
		DPRINT6("SubQ %p [%d] ->Hd %d\n",
						qinfo->sub_queue, cq_entry->sqID,
						cq_entry->sqHdPtr);
#endif

#if FORCE_ERROR
{
		static  u32 counter = 0;
		if ((++counter > ERROR_FREQ) && (!cq_entry->SC)) {

		static int error_type = 0;
		counter = 0;
		if (error_type == 0) {
			cq_entry->SCT = 0;
			cq_entry->SC = SF_SC_INT_DEV_ERR;
			cq_entry->noRetry = 1;
			cq_entry->more = 0;
			error_type++;
			DPRINT("Forcing non-recoverable error 0x%x\n",
							SF_SC_INT_DEV_ERR);
		} else if (error_type == 1) {
			cq_entry->SCT = 0;
			cq_entry->SC = SF_SC_INT_DEV_ERR;
			cq_entry->noRetry = 0;
			cq_entry->more = 0;
			error_type++;
			DPRINT("Forcing recoverable error 0x%x\n",
							SF_SC_INT_DEV_ERR);
		} else if (error_type == 2) {
			cq_entry->SCT = 0;
			cq_entry->SC = SF_SC_INT_DEV_ERR;
			cq_entry->noRetry = 1;
			cq_entry->more = 1;
			error_type = 0;
			DPRINT("Forcing non-recoverable error log 0x%x\n",
							SF_SC_INT_DEV_ERR);
		}
		}
}
#endif
#ifdef NVME_DEBUG
		if (cmd_info->status != -1) {   /** Active Command ? */
			DPRINT("*** InActive command %p, [%d]\n",
						cmd_info, cmd_info->cmd_id);
		goto next_entry;
		}
#endif

#if FORCE_TIMEOUT
{
		static  u32 counter = 0;
		if (++counter > FREQ_FORCE_TIMEOUT) {
			DPRINT4("*** Forcing timeout id %d, cmd_info %p, [%d] req %p\n",
						cmd_info->timeout_id, cmd_info,
					cmd_info->cmd_id, cmd_info->req);
		counter = 0;
			goto next_entry;
		}
}
#endif

		cmd_info->cmd_status = status;

#if FORCE_DIF_ERROR
		/*
		 * Clear error that we had forced for testing purposes to allow
		 * continuous testing.
		 */
		if ((cq_entry->SCT == 2) && ((cq_entry->SC == 0x82) ||
				(cq_entry->SC == 0x83) || (cq_entry->SC == 0x84))) {
#ifdef NVME_DEBUG
		nvme_show_error(dev, cq_entry);
#endif
			cq_entry->SC = cq_entry->SCT = 0;
			cmd_info->cmd_status = status = 0;
		}
#endif

		/**
		 * Print error type and status in string.
		 */
		if (unlikely(status)) {

#ifdef NVME_DEBUG
			nvme_show_error(dev, cq_entry);
#endif

			if (cq_entry->more) {
				struct queue_info *aqinfo = dev->admin_queue;

				/**
				 * Admin Queue is used to extract error logs.
				 */
				if (qinfo == aqinfo)
					aqinfo->unlock_func(&aqinfo->lock, &aqinfo->lock_flags);

				nvme_get_error_log(dev, cmd_info);

				if (qinfo == aqinfo)
					aqinfo->lock_func(&aqinfo->lock, &aqinfo->lock_flags);
			}
			nvme_log_err(qinfo, cmd_info, cq_entry);
			if ((!cq_entry->noRetry) && (cmd_info->type == BIO_CONTEXT)) {

				/* @todo - Must check base_info completion for retry sts */
				if (cmd_info->cmd_base) {
					base_info = cmd_info->cmd_base;

					if ((!base_info->cmd_status) ||
						(base_info->cmd_status > status))
						base_info->cmd_status = status;

					if (base_info != cmd_info) {
						if (qinfo->timeout[cmd_info->timeout_id] > 0)
							qinfo->timeout[cmd_info->timeout_id]--;
						if (cmd_info->sg_count) {
							dma_unmap_sg(dev->dma_dev, cmd_info->sg_list,
									 cmd_info->sg_count, DMA_FROM_DEVICE);
							cmd_info->sg_count = 0;
						}
						nvme_put_cmd(qinfo, cmd_info);
					}
					if (base_info->cmd_count-- > 1)
						goto next_entry;
					cmd_info = base_info;
				}

				bio = cmd_info->req;

				if ((cmd_info->cmd_retries < MAX_RETRY) &&
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
					!(bio->bi_rw & BIO_REQ_WRITE)) {
#else
					!(bio_op(bio) & BIO_REQ_WRITE)) {
#endif

					DPRINT("Retrying BIO %p, vcnt %d, idx %d\n",
						   bio, bio->bi_vcnt, bio->bi_iter.bi_idx);

					cmd_info->cmd_retries++;
					if (cmd_info->sg_count) {
						dma_unmap_sg(dev->dma_dev, cmd_info->sg_list,
									 cmd_info->sg_count, DMA_FROM_DEVICE);
						cmd_info->sg_count = 0;
					}
					qinfo->timeout[cmd_info->timeout_id]--;
				/* @todo - What if submit fails? */
					nvme_submit_request(qinfo, cmd_info->ns,
							bio, cmd_info->cmd_retries);
					nvme_put_cmd(qinfo, cmd_info);
					goto next_entry;
				}
			}
		}
		if (cmd_info->status == -1)
			{   /** Active Command ? */

			cmd_info->status = 0;
			if (cmd_info->type == BIO_CONTEXT) {

				/**
				 * ignore all completions while queue operarions are
				 * suspended or in process of flushing it.
				 */
				if ((dev->state >= NVME_STATE_SUSPEND) ||
					(qinfo->flags & QUEUE_SUSPEND))
					goto next_entry;

				if (qinfo->timeout[cmd_info->timeout_id] > 0)
					qinfo->timeout[cmd_info->timeout_id]--;
				if (cmd_info->cmd_base) {

					base_info = cmd_info->cmd_base;

					if ((!base_info->cmd_status) ||
						(base_info->cmd_status > status))
					base_info->cmd_status = status;

					if (base_info != cmd_info) {
						if (cmd_info->sg_count) {
							dma_unmap_sg(dev->dma_dev, cmd_info->sg_list,
							cmd_info->sg_count, DMA_FROM_DEVICE);
							cmd_info->sg_count = 0;
						}
						nvme_put_cmd(qinfo, cmd_info);
					}
					if (base_info->cmd_count-- > 1) {
						goto next_entry;
					}

					/**
					 * all commands have completed. Complete the request.
					 */
					cmd_info = base_info;
				}
				bio = cmd_info->req;

				qinfo->nr_req--;
				nvme_disk_stat_iodone(dev, cmd_info);
				bio->bi_error = (!cmd_info->cmd_status) ? 0 : -EIO;
				bio_endio(bio);

				if (cmd_info->sg_count) {
					dma_unmap_sg(dev->dma_dev, cmd_info->sg_list,
							cmd_info->sg_count, DMA_FROM_DEVICE);
					cmd_info->sg_count = 0;
				}
				nvme_put_cmd(qinfo, cmd_info);

			} else {

				nvme_memcpy_64(&cmd_info->cq_entry, cq_entry,
							sizeof(*cq_entry)/sizeof(u64));
				if (unlikely((cmd_info->type == ABORT_CONTEXT))) {
					DPRINT("Abort CONTEXT completion.....\n");
					cmd_info->status = EIO;
					nvme_put_cmd(qinfo, cmd_info);
				} else if (unlikely((cmd_info->type == EVENT_CONTEXT))) {
					DPRINT("Event CONTEXT completion.....\n");
					dev->thread_flags |= THREAD_AEN;
					wake_up(&dev->thread_wait);
				} else if (unlikely((cmd_info->type == LOG_CONTEXT))) {
					DPRINT("Log CONTEXT completion.....\n");
					dev->thread_flags |= THREAD_LOG;
					wake_up(&dev->thread_wait);
				} else if (unlikely((cmd_info->type == ERR_CONTEXT))) {
					DPRINT("Error CONTEXT completion.....\n");
					dev->thread_flags |= THREAD_ERR;
					wake_up(&dev->thread_wait);
				} else {	/* ADMIN_CONTEXT Async request */
					DPRINT("Internal request; wakeup signal .....\n");
					DPRINT("Admin CONTEXT cmd id %d\n",cmd_info->cmd_id);
					wake_up(&cmd_info->waitq);
				}
			}
		}
next_entry:
		/**
		 * next completion.
		 */
		if (++head >= qinfo->qsize) {
			cq_entry = qinfo->compq;
			head = 0;
			phase = !phase;
			DPRINT3("***** Phase Change %d *****\n", phase);
		}
	}

	if (unlikely((head == qinfo->head) && (phase == qinfo->phase))) {
		irq_status = IRQ_NONE;
		sq_head = qinfo->sub_queue->head;
	} else {
		qinfo->head  = head;
		qinfo->phase = phase;
		DPRINT5("Completion queue doorbell %p [%d]\n",
					qinfo->doorbell, qinfo->head);

		writel(head, qinfo->doorbell);  	/** Ack Interrupt */
		DPRINT5("Comp Entry [%d] phase %d, subq_head %d\n",
					head, phase, sq_head);
		irq_status = IRQ_HANDLED;
	}

	{
		/**
		 * @todo - This block of code blongs inside the for loop
		 * if we were to support multiple submission queues.
		 * We should also use the global "sque_list" to
		 * get to submission queue. however, since we
		 * only have a single submission queue, use the
		 * completion queue to get to it.
		 *
		 * sqinfo = dev->sque_list[cq_entrycq_entry.sqID];
		 */
		struct sub_queue_info *sqinfo = qinfo->sub_queue;

		if (sq_head <= sqinfo->tail) {
		sqinfo->entries = sqinfo->qsize - ((sqinfo->tail-sq_head)+1);
		} else {
		sqinfo->entries = (sq_head - sqinfo->tail) - 1;
		}
		DPRINT5("Sub Queue Entries [%d] tail %d, subq_head %d\n",
					sqinfo->entries, sqinfo->tail, sq_head);
		sqinfo->head = sq_head;
		if ((bio_list_peek(&qinfo->sq_cong)) && (sqinfo->entries)) {
			DPRINT2("**** congested qid [%d] ****\n", qinfo->id);
			if (!use_threaded_interrupts) {
				schedule_work_on(qinfo->cpu, &qinfo->wq_work);
			} else {
				nvme_process_cong(qinfo);
			}
		}
	}
	return (irq_status);
}

static void
nvme_process_work(struct work_struct *work)
{
	struct queue_info *qinfo =
				container_of(work, struct queue_info, wq_work);

#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif

	DPRINT3("Workqueue [%d], qinfo %p [%d]\n",
				smp_processor_id(), qinfo, qinfo->id);
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	if (!nvme_msix_enable) {
		nvme_process_cq(qinfo);
	} else {
		nvme_process_cong(qinfo);
	}
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
}


/**
 * @brief This function retrieves pointer to  queue information block
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return struct queue_info *qinfo pointer to queue information block
 */
struct queue_info *
get_nvmeq(struct nvme_dev *dev)
{
	return (dev->cpu_to_que_map[smp_processor_id()]);
}

/**
 * @brief This function called to handle a BIO request.
 *
 * @param[in] struct request_queue *q pointer to block layer queue
 * @param[in] struct bio *bio pointer to bio request
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note return value of non-zero would mean that we were a stacking driver.
 * @note make_request must always succeed.
 */
static blk_qc_t
nvme_make_request(struct request_queue *q, struct bio *bio)
{
	struct ns_info *	ns = q->queuedata;
	struct nvme_dev *   dev;
	struct queue_info * qinfo;
	int result = -EBUSY;

	/*
	 * The following call will ensure that the queue restrictions
	 * are met on the request size.
	 */
	blk_queue_split(q, &bio, q->bio_split);

	dev = ns->dev;
	if (!(ns->flags & NS_ONLINE)) {
		DPRINT1("*** ERROR *** Received request while Offlined. ns_id %d\n",
					ns->id);
		bio->bi_error = -EBADF;
		bio_endio(bio);
		return BLK_QC_T_NONE;
	}
	if (dev->state > NVME_STATE_RESET) {
		DPRINT("****** Error Completion BIO %p, dev state %d\n",
						bio, dev->state);
		bio->bi_error = -EIO;
		bio_endio(bio);
		return BLK_QC_T_NONE;
	}
#if USE_NS_ATTR
	if (unlikely((ns->flags & NS_READONLY) &&
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
				(bio->bi_rw & BIO_REQ_WRITE))) {
#else
				(bio_op(bio) & BIO_REQ_WRITE))) {
#endif
		DPRINT1("*** Rejecting WRITE request,  NS is READONLY. ns_id %d\n",
								ns->id);
		bio->bi_error = -EACCES;
		bio_endio(bio);
		return BLK_QC_T_NONE;
	}
#endif


#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0) )
	if ( bio_rw(bio) ) {
#else
	if ( bio_data_dir(bio) ) {
#endif
		dev->bytes_written += bio->bi_iter.bi_size;
	} else {
		dev->bytes_read += bio->bi_iter.bi_size;
	}
	qinfo = get_nvmeq(dev);
	DPRINT1("ns_id %d, BIO %p, Qinfo %p [%d], len %d, nr_req %d\n",
				ns->id, bio, qinfo, qinfo->id,
				bio->bi_iter.bi_size, qinfo->nr_req);

	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	qinfo->nr_req++;
	if (qinfo->max_req < qinfo->nr_req)
		qinfo->max_req = qinfo->nr_req;

	/**
	 * We, stop posting requests of there is already another request
	 * waiting to be posted.
	 */
	if (bio_list_empty(&qinfo->sq_cong)) {
		dev->user_req++;
		result = nvme_submit_request(qinfo, ns, bio, 0);
	}

	/**
	 * If we failed to post a resource due to hardware resources,
	 * just append it to congestion list for BH processing.
	 */
	if (unlikely(result)) {
		bio_list_add(&qinfo->sq_cong, bio);
#if DO_IO_STAT
		nvme_disk_stat_cong(dev, ns, bio);
#endif
	}
	/*checking for completions in order to reduce interrupt processing time*/
	nvme_process_cq(qinfo);
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	return BLK_QC_T_NONE;
}

/**
 * @brief This function Removes cmd_info from active list and add to free list.
 *
 * Note -   It is assummed that caller has held queue lock.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] struct cmd_info *cmd_info pointer to command information block
 *
 * @return void
 */
void
nvme_put_cmd(struct queue_info *qinfo, struct cmd_info *cmd_info)
{

	cmd_info->cmd_base = NULL;
	list_move_tail(&cmd_info->list, &qinfo->cmd_free);
	qinfo->nr_act--;
}


/**
 * @brief This function retrieves cmd_info from free list adds to active list.
 *
 * @param[in] struct queue_info *qinfo pointer to queue information block
 *
 * @return struct cmd_info *, pointer to cmd_info, otherwise NULL
 *
 * @note It is assumed that queue lock is held by caller.
 */
static struct cmd_info *
nvme_get_cmd(struct queue_info *qinfo)
{
	struct cmd_info *cmd_info;
#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif

	if (list_empty(&qinfo->cmd_free)) {
		DPRINT2("queue [%d] Command list Empty\n", qinfo->id);
		return (NULL);
	}
	cmd_info = list_first_entry(&qinfo->cmd_free, struct cmd_info , list);
	list_move_tail(&cmd_info->list, &qinfo->cmd_active);
	qinfo->nr_act++;

	DPRINT6("queue [%d] cmd_info [%d] %p\n",
				qinfo->id, cmd_info->cmd_id, cmd_info);
	return (cmd_info);
}

/**
 * @brief This function send a Request to retrieve no. available of IO queues
 *  		Send a set feature requesting optimum number of
 *  		queues. If hardware does not allow the requested
 *  		number of IO queues, request for at least for number
 *  		of NUMA nodes and if that fails, just request for
 *  		a single IO queue.
 *
 * Note:	For now, we always assume the number of completion and
 *  	submission queues are same.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] u32 *nr_io_queues pointer to variable returning number of queues
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
int
nvme_request_queues(struct nvme_dev *dev, u32 *nr_io_queues)
{
	struct cq_entry cq_entry;
	int result = 0;

	DPRINT3("attempting to allocate [%d] IO queues\n", *nr_io_queues);

	do {
		result = nvme_set_feature(dev, FTR_ID_NUM_QUEUE,
					(*nr_io_queues << 16) | *nr_io_queues,
				NULL, &cq_entry);
		if (result) {

			EPRINT("Failed requesting nr_io_queues 0x%x\n", cq_entry.SC);
		if (*nr_io_queues == 1)
				break;
			if (*nr_io_queues == num_online_cpus()) {
				*nr_io_queues = num_online_nodes();
			EPRINT("trying for num of NUMA nodes %d\n", *nr_io_queues);
			} else {
				*nr_io_queues = 1;
			}
		}

	} while(result);

	if (! result) {
		DPRINT3("maximum of [%d] IO queues\n",
						cq_entry.param.numCplQAlloc);
		if (cq_entry.param.numCplQAlloc < *nr_io_queues)
			*nr_io_queues =  cq_entry.param.numCplQAlloc;
	}
	return (result);
}

/**
 * @brief This function frees Command Information list.
 *  	This function releases cmd information block resources.
 *  	a.  free prp list
 *  	b.  free sg list
 *  	c.  free command information array
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct queue_info *qinfo pointer to queue information block
 *
 * @return void - none
 */
static void
cmd_info_dest(struct nvme_dev *dev, struct queue_info *qinfo)
{
	struct cmd_info *cmd_info;
	int i;

	DPRINT3("Executing ....\n");

	cmd_info = qinfo->cmd_list;
	DPRINT3("qinfo %p, cmd_list %p\n", qinfo, cmd_info);

	if (! cmd_info)
		return;
	/**
	 * Create free command information list
	 */
	for (i = 0; i < qinfo->qsize; i++) {

		dma_pool_free(qinfo->prp_pool, cmd_info->prps,
						cmd_info->prp_phy);
		if (cmd_info->sg_list)
		kfree(cmd_info->sg_list);
		cmd_info++;
	}
	kfree(qinfo->cmd_list);
	qinfo->cmd_list = NULL;
}

/**
 * @brief This function Constructs command information free list.
 *  	This function creates linked list of free cmd_information
 *  	block from queue's cmd_list array.
 *  	a.  alloc prp list
 *  	b.  alloc sg list
 *  	c.  create linked list of free command information blocks
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct queue_info *qinfo pointer to queue information block
 *
 * @return void - none
 *
 * @note This function manipulates queue's free and active list,
 *  caller should make sure queue is not used by any other thread.
 */
static int
cmd_info_cons(struct nvme_dev *dev, struct queue_info *qinfo)
{
	struct cmd_info *cmd_info;
	struct nvme_prp  *prps;
	dma_addr_t   prp_phy;
	int i;

	DPRINT3("Executing ....\n");
	INIT_LIST_HEAD(&qinfo->cmd_free);
	INIT_LIST_HEAD(&qinfo->cmd_active);

	cmd_info = qinfo->cmd_list;
	DPRINT3("qinfo %p, cmd_list %p\n", qinfo, cmd_info);

	/**
	 * Create free command information list
	 */
	for (i = 0; i < qinfo->id_count; i++) {

		spin_lock_init(&cmd_info->lock);
		cmd_info->cmd_id	 = i+1; 	/** 0 is reserved */

		prps = dma_pool_alloc(qinfo->prp_pool, GFP_KERNEL, &prp_phy);
		if (! prps) {
				EPRINT("cmd_info PRP DMA buffer alloc failure\n");
			goto err_cmd_cons;
		}
		cmd_info->prps  	 = prps;
		cmd_info->prp_phy    = prp_phy;

		cmd_info->sg_list = kzalloc_node(sizeof( * cmd_info->sg_list) *
				max_prp_list, GFP_KERNEL, qinfo->node);
		if (! cmd_info->sg_list) {
			EPRINT("SG list local node memory allocation error\n");
			cmd_info->sg_list = kzalloc(sizeof( * cmd_info->sg_list) *
						max_prp_list, GFP_KERNEL);
		}
		init_waitqueue_head(&cmd_info->waitq);
		if (! cmd_info->sg_list) {
			EPRINT("cmd_info SG list memory alloc failure\n");
			goto err_cmd_cons;
		}
		list_add_tail(&cmd_info->list, &qinfo->cmd_free);

		cmd_info++;
	}
	return (0);

err_cmd_cons:
	cmd_info_dest(dev, qinfo);
	return (-ENOMEM);
}


/**
 * @brief This function frees Queue resources.
 *  	This function releases all queue info resources allocated by
 *  	constructor.
 *  	a. free Command Information list
 *  	b. free PRP list DMA pool.
 *  	c. free submission queue DMA buffer
 *  	d. free completion queue DMA buffer
 *  	e. free submission queue information block
 *  	f. clear device queue information list
 *  	g. free queue information block
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct queue_info *qinfo pointer to queue information block
 *
 * @return void - none
 */
static void
queue_dest(struct nvme_dev *dev, struct queue_info *qinfo)
{
#ifdef CONFIG_NUMA
	int saved_node, node = qinfo->node;
	struct device *dmadev = dev->dma_dev;
#endif

	DPRINT3("qinfo %p [%d], cmd_list %p, cpu %d [%d]\n",
				qinfo, qinfo->id, qinfo->cmd_list,
				qinfo->cpu, qinfo->node);
	/**
	 * We override device NUMA node ID to release DMA memory
	 * to the same pool that we allocated it from.
	 * Device node ID is restored upon exit.
	 */
#ifdef CONFIG_NUMA
	saved_node = dev_to_node(dmadev);
	dmadev->numa_node = node;
#endif

	if (qinfo->cmd_list)
		cmd_info_dest(dev, qinfo);

	DPRINT3("prp_pool %p \n", qinfo->prp_pool);
	if (qinfo->prp_pool) {
		dma_pool_destroy(qinfo->prp_pool);
		qinfo->prp_pool = NULL;
	}
	if (qinfo->sub_queue && qinfo->sub_queue->subq) {
		DPRINT3("sub_queue %p \n", qinfo->sub_queue->subq);
		dma_free_coherent(dev->dma_dev,
			(qinfo->qsize * sizeof(struct nvme_cmd)),
			 qinfo->sub_queue->subq, qinfo->sub_queue->subq_phy);
		qinfo->sub_queue->subq = NULL;
	}

	DPRINT3("compq %p \n", qinfo->compq);
	if (qinfo->compq) {
		dma_free_coherent(dev->dma_dev,
			(qinfo->qsize * sizeof(struct cq_entry)),
			qinfo->compq, qinfo->compq_phy);
		qinfo->compq = NULL;
	}
	DPRINT3("sqinfo %p \n", qinfo->sub_queue);
	if (qinfo->sub_queue)  {
		dev->sque_list[qinfo->sub_queue->id] = NULL;
		kfree(qinfo->sub_queue);
	}
	dev->que_list[qinfo->id] = NULL;
	if (qinfo)
		kfree(qinfo);
#ifdef CONFIG_NUMA
	dmadev->numa_node = saved_node;
#endif
}


/**
 * @brief This function Allocate Queue resources.
 *  	This function allocates queue info, queue DMA buffer, Submission
 *  	queue(s), its DMA buffer(s) and command information block.
 *  	a. allocate queue information block.
 *  	b. allocate submission queue information block
 *  	c. Allocate Completion queue DMA buffer
 *  	d. allocate submission queue DMA buffer
 *  	e. Allocate PRP list DMA pool.
 *  	f. Set Device queue information list
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] int sqsize Submission queue size
 * @param[in] int cqsize Completion queue size
 * @param[in] int qid Queue ID
 * @param[in] int shared Flag indicating this is a shared CPU queue
 * @param[in] int cpu smp_processor id
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static struct queue_info *
queue_cons(struct nvme_dev *dev, int sqsize, int cqsize, int qid,
						int shared, int cpu)
{
	struct queue_info *qinfo;
	struct sub_queue_info *sqinfo;
	struct device *dmadev = dev->dma_dev;
	size_t size;
	int node;
#ifdef CONFIG_NUMA
	int saved_node;
#endif

	if ((cpu < 0) || (cpu > num_online_cpus()))
		node = 0;
	else
		node = cpu_to_node(cpu);
	/**
	 * We override device NUMA node ID to allocate DMA memory
	 * local to CPU instead of local to device.
	 * Device node ID is restored upon exit.
	 */
#ifdef CONFIG_NUMA
	saved_node = dev_to_node(dmadev);
	dmadev->numa_node = node;
#endif

	/**
	 * Allocate Completion queue Information block from
	 * local NUMA node memory.
	 */
	qinfo = kzalloc_node(sizeof(*qinfo), GFP_KERNEL, node);
	if (! qinfo) {
		EPRINT("Queue Info local node memory allocation error\n");
		qinfo = kzalloc(sizeof(*qinfo), GFP_KERNEL);
	}
	DPRINT3("queue info %p [%d], cpu %d, node %d %s\n",
				qinfo, qid, cpu, node, shared? "shared" : "");
	if (! qinfo) {
		EPRINT("Queue Info memory alloc failure\n");
		goto err_que_cons;
	}

	spin_lock_init(&qinfo->lock);
	bio_list_init(&qinfo->sq_cong);
	qinfo->qsize	= cqsize;
	qinfo->dev  = dev;
	qinfo->cpu  = cpu;
	qinfo->node = node;

	if (shared) {
		qinfo->lock_func	= nvme_spin_lock;
		qinfo->unlock_func  = nvme_spin_unlock;
	} else {
		qinfo->lock_func = (void (*)(spinlock_t *, ulong *))
								nvme_get_cpu;
		qinfo->unlock_func = (void (*)(spinlock_t *, ulong *))
							nvme_put_cpu;
	}

	/**
	 * Allocate Submission queue Information block from
	 * local NUMA node memory.
	 */
	sqinfo = kzalloc_node(sizeof(*sqinfo), GFP_KERNEL, node);
	if (! sqinfo) {
		EPRINT("SubQueue Info local node memory allocation error\n");
		sqinfo = kzalloc(sizeof(*sqinfo), GFP_KERNEL);
	}
	DPRINT3("sub_queue info %p \n", sqinfo);
	if (! sqinfo) {
		EPRINT("Submission Queue Info memory alloc failure\n");
		goto err_que_cons;
	}
	spin_lock_init(&sqinfo->lock);
	qinfo->sub_queue = sqinfo;
	sqinfo->qsize = sqsize;
	sqinfo->dev  = dev;

	/**
	 * Allocate Completion queue DMA buffer
	 */
	qinfo->compq = dma_alloc_coherent(dmadev,
				(cqsize * sizeof(struct cq_entry)),
					&qinfo->compq_phy, GFP_KERNEL);
	DPRINT3("Completion Queue %p, phy 0x%016llx\n",
			qinfo->compq, (u64)qinfo->compq_phy);
	if (! qinfo->compq) {
		EPRINT("Completion Queue DMA buffer alloc failure\n");
		goto err_que_cons;
	}

	/** Initialize Completion head and tail */
	qinfo->head = 0;
	qinfo->tail = 0;
	qinfo->phase = 1;
	qinfo->timeout_id = -1;

	/**
	 * Allocate Submission queue DMA buffer
	 */
	sqinfo->subq = dma_alloc_coherent(dmadev,
				(sqsize * sizeof(struct nvme_cmd)),
					&sqinfo->subq_phy, GFP_KERNEL);
	DPRINT3("Submission Queue %p, phy 0x%016llx\n",
			sqinfo->subq, (u64)sqinfo->subq_phy);
	if (NULL == sqinfo->subq) {
		EPRINT("Submission Queue DMA buffer alloc failure\n");
		goto err_que_cons;
	}

	/**
	 * Initialize Submission head and tail
	 * For now, we are assuming a single submission queue
	 * per completion queue. Eventually we need to allocate
	 * sequential submission queue ids from pool of available ids.
	 */
	sqinfo->head = 0;
	sqinfo->tail = 0;
	sqinfo->id   = qid;
	sqinfo->entries = sqinfo->qsize - 1;
	dev->sque_list[qid] = sqinfo;



	/** Allocate Command Information Block
	 * Number of cached command IDs for IO queues are defined by
	 * driver parameter "io_command_id_size".
	 * Admin Queue cached commands IDs are same as queue size.
	 *
	 */
	if (qid)
		qinfo->id_count = io_command_id_size;
	else
		qinfo->id_count = sqsize;
	size = qinfo->id_count * sizeof(* qinfo->cmd_list);

	DPRINT3("queue id_count %d, size of cmd_list %d\n",
						qinfo->id_count, (u32)size);
	qinfo->cmd_list = kzalloc_node(size, GFP_KERNEL, node);
	if (! qinfo->cmd_list) {
		EPRINT("Command Info local node memory allocation error\n");
		qinfo->cmd_list = kzalloc(size, GFP_KERNEL);
	}
	if (! qinfo->cmd_list) {
		EPRINT("Command Info memory alloc failure\n");
		goto err_que_cons;
	}

	size = sizeof(struct nvme_prp) * max_prp_list;
	qinfo->prp_pool = dma_pool_create("Queue prp list", dmadev,
						size, size, 0);
	DPRINT3("queue [%d] PRP pool %p, size 0x%x, align 0x%x\n",
			qinfo->id, qinfo->prp_pool, (u32)size, (u32)size);
	if (! qinfo->prp_pool) {
		EPRINT("PRP list pool alloc failure\n");
		goto err_que_cons;
	}

	sqinfo->lock_func   = qinfo->lock_func;
	sqinfo->unlock_func = qinfo->unlock_func;
	sqinfo->compq   	= qinfo;

	init_waitqueue_head(&qinfo->waitq);
	INIT_WORK(&qinfo->wq_work, nvme_process_work);
	INIT_WORK(&qinfo->wq_time, nvme_stalled_command);

	qinfo->id = qid;
	dev->que_list[qid] = qinfo;
	if(cpu>=0)
		dev->cpu_to_que_map[cpu]=qinfo;
#ifdef CONFIG_NUMA
	dmadev->numa_node = saved_node;
#endif
	return (qinfo);

err_que_cons:
	queue_dest(dev, qinfo);
#ifdef CONFIG_NUMA
	dmadev->numa_node = saved_node;
#endif
	return (NULL);
}

/**
 * @brief This function Constructs Admin queue.
 *  	This function allocates Admin queue info, and assigns
 *  	vector 0.
 *  	a. Construct queue memory and dma resources.
 *  	b. Construct admin commands.
 *  	c. assign and attach vector 0.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *  	dev, pointer driver context.
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 *
 * @note This function sets up admin queue information in "nvme_dev" structuure
 *   fields.
 */
static int
nvme_admin_cons(struct nvme_dev *dev)
{
	struct queue_info *qinfo;

	dev->admin_queue = queue_cons(dev, admin_cpl_queue_size,
					admin_sub_queue_size, 0, TRUE, -1);
	if (!(qinfo = dev->admin_queue))
		return (-ENOMEM);

	DPRINT3("Admin queue Constructed %p\n", qinfo);
	if (unlikely(cmd_info_cons(dev, qinfo))) {
		queue_dest(dev, qinfo);
		dev->admin_queue = NULL;
		return (-ENOMEM);
	}
	DPRINT3("Admin Commands Constructed %p\n", qinfo->cmd_list);

	/**
	 * Assigned vector 0 Admin Queue and request IRQ.
	 * we initially request normal INTA interrupt until we gather
	 * controller information and then deciding how many vectors
	 * we actually need.
	 *
	 */
	dev->msix_entry[0].vector   = dev->pci_dev->irq;
	dev->msix_entry[0].entry	= 0;
	qinfo->cq_ivec  = 0;
	qinfo->cpu_cnt  = num_online_cpus();
	qinfo->doorbell = dev->reg + NVME_ACQHDBL;
	qinfo->sub_queue->doorbell = dev->reg + NVME_ASQTDBL;
	return (0);
}

/**
 * @brief This function constructs all IO queues.
 *  	This function allocates IO queue info, and assigns
 *  	vector and queue ID to each queue sequentially.
 *  	a. Construct queue memory and dma resources.
 *  	b. Construct command information.
 *  	c. assign and attach IRQ vector.
 *  	d. register completion and submission queues with firmware.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] nr_io_queue, number of IO queues to create.
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_create_io_queues(struct nvme_dev *dev, u32 nr_io_queues)
{
	int i, cpu_cnt, shared, ishared = 0, result = 0;
	uint cpu;
	struct queue_info *qinfo;
	struct sub_queue_info *sqinfo;

	if (! nr_io_queues)
		return (-EINVAL);   	/** invalid request */

	cpu_cnt = num_online_cpus()/nr_io_queues;
	shared = (num_online_cpus() > nr_io_queues) ? TRUE : FALSE;

	DPRINT3("nr_io_queues %d, shared %d\n", nr_io_queues, shared);
	dev->que_count = nr_io_queues;
	for( i = 1, cpu = cpumask_first(cpu_online_mask); i <= nr_io_queues ;
				i++, cpu = cpumask_next(cpu, cpu_online_mask) )
	{

		/* @todo - Must handle cases where there is a gap in cpu ids. */
		qinfo = queue_cons(dev, io_cpl_queue_size,
					io_sub_queue_size, i, TRUE, cpu);
		if (! qinfo) {
		return (-ENOMEM);
		}
		DPRINT3("IO queue %p [%d] created.....\n", qinfo, qinfo->id);
		if (unlikely(cmd_info_cons(dev, qinfo))) {
			queue_dest(dev, qinfo);
			return (-ENOMEM);
		}
		DPRINT3("Command info %p list created\n", qinfo->cmd_list);
		sqinfo = qinfo->sub_queue;

		/**
		 * Assign vector i to IO Queue and request IRQ.
		 */
		if (nvme_msix_enable) {
			qinfo->cq_ivec  = nvme_assign_ivec(dev, qinfo->id, &ishared);
		} else {
			qinfo->cq_ivec  = 0;	/* all IOs share Admin queue vector */
			ishared 	= 1;
		}
		qinfo->cpu_cnt  = cpu_cnt;
		qinfo->doorbell 	= dev->reg + ((i * 8) + NVME_ACQHDBL);
		sqinfo->doorbell	= dev->reg + ((i * 8) + NVME_ASQTDBL);

		DPRINT3("IO queue [%d] cpu %d cpu count %d, vector %d\n",
			qinfo->id, cpu, cpu_cnt, dev->msix_entry[i].vector);
		DPRINT3("IO queue [%d] %p, Comp DB %p, Sub DB %p\n",
			qinfo->id, qinfo,
			qinfo->doorbell, qinfo->sub_queue->doorbell);

		result = nvme_create_cq(dev, qinfo, i);
		if (result) {
		EPRINT("Failed to create hardware IO completion queue %d\n",
								qinfo->id);
			queue_dest(dev, qinfo);
		break;
		}

		result = nvme_create_sq(dev, qinfo, sqinfo->id);
		if (result) {
		EPRINT("Failed to create hardware IO submission queue %d\n",
								sqinfo->id);
		nvme_delete_cq(dev, qinfo->id);
			queue_dest(dev, qinfo);
		break;
		}

		sprintf(qinfo->ivec_name, "%s%d-IO%d",
					DRV_NAME, dev->instance, qinfo->id);
		if (! ishared && (nvme_request_irq(dev, qinfo,
						qinfo->ivec_name, cpu) < 0)) {
			EPRINT("Failed to assign irq[%d] %03d\n", qinfo->cq_ivec,
				dev->msix_entry[qinfo->cq_ivec].vector);
		result = -ENOMEM;
		nvme_delete_sq(dev, qinfo->id);
		nvme_delete_cq(dev, qinfo->id);
			queue_dest(dev, qinfo);
		break;
		}
	}
	return (result);
}


/**
 * @brief This function deconstructs all IO queues.
 *  	This function releases all IO queue info, and releases
 *  	vector IDs and queue IDs.
 *  	a. delete hardware completion and submission queues.
 *  	b. Releas and detach IRQ vector.
 *  	c. release command information.
 *  	d. release queue memory and dma resources.
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] resource, flag indicating whether to keep or free IRQs and
 *  	information block.
 *
 * @return  none.
 *
 * @note It is assumed that dev lock is held by caller.
 */
static void
nvme_delete_io_queues(struct nvme_dev *dev, int keep_mem)
{
	int i;
	struct queue_info *qinfo;
	struct sub_queue_info *sqinfo;

	DPRINT3("queue count %d\n", dev->que_count);
	for( i = 1; i <= dev->que_count  ; i++ ) {

		qinfo = dev->que_list[i];
		if (! qinfo)
			continue;

		/**
		 * Suspend all activities on hardware queue pair.
		 */
		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		qinfo->flags |= QUEUE_SUSPEND;
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		if (!keep_mem)
			nvme_release_irq(dev, qinfo);

		if (dev->state != NVME_STATE_FAILED) {

			sqinfo = qinfo->sub_queue;
			if (nvme_delete_sq(dev, sqinfo->id)) {
			EPRINT("Failed to delete hardware IO submission queue %d\n",
								sqinfo->id);
		}
			if (nvme_delete_cq(dev, qinfo->id)) {
			EPRINT("Failed to delete hardware IO completion queue %d\n",
								qinfo->id);
		}
			DPRINT3("Deleted sub_que %d, and cpl_que %d\n",
						sqinfo->id, qinfo->id);
		}
		if (!keep_mem)
			queue_dest(dev, qinfo);
	}
}


#if SEND_AEN
/**
 * @brief This function sends Asynchronous Event Notification request.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return int 0 if successful, otherwise Error Code
 */
static int
nvme_send_aen(struct nvme_dev *dev)
{
	struct cmd_info *cmd_info = NULL;
	struct queue_info *qinfo;
	int result = 0;

	DPRINT2("dev %p [%d], max events %d, Number of events %d\n", dev,
				dev->instance, dev->max_aen, dev->cur_aen);
	dev->lock_func(&dev->lock, &dev->lock_flags);
	qinfo = dev->admin_queue;
	DPRINT4("Maximum of %d AEN requests\n", dev->max_aen);

	while(dev->cur_aen < dev->max_aen) {

		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		cmd_info = nvme_get_cmd(qinfo);
		if (!cmd_info) {
			DPRINT4("Out of Cmd_Info data %p\n", qinfo);
		result = -ENOMEM;
			qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		break;
		}
		cmd_info->req = NULL;
		cmd_info->type = EVENT_CONTEXT;
		cmd_info->nvme_cmd.header.cmdID = cmd_info->cmd_id;
		cmd_info->nvme_cmd.header.opCode = NVM_ADMIN_CMD_ASYNC_EVENT_REQ;
		DPRINT2("command ID %d\n", cmd_info->cmd_id);

		result = nvme_send_cmd(qinfo->sub_queue, cmd_info);
		if (result < 0) {
			EPRINT("Failed send Async Event\n");
		nvme_put_cmd(qinfo, cmd_info);
			qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		break;
		}
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		++dev->cur_aen;
	}
	dev->unlock_func(&dev->lock, &dev->lock_flags);
	DPRINT2("%d AEN requests pending\n", dev->cur_aen);
	return  (result);
}
#endif


/**
 * @brief This function called to report an error log.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context.
 * @param[in] void * log pointer to log page data.
 *
 * @return none.
 */
static void
nvme_report_error(struct nvme_dev *dev, void *log)
{
	struct error_log	*err_log = log;

	EPRINT("Error Log report %lld, Sub QID %d, command Context %d,"
				"status 0x%x\n",
				err_log->errorCount,
				err_log->sqID,
				err_log->cmdID,
				err_log->status);
	EPRINT("Error Log report %lld, error byte# %d, bit# %d, ns %d, lba %lld"
				"vendor Info 0x%x\n",
				err_log->errorCount,
				err_log->errorByte,
				err_log->errorBit,
				err_log->nameSpace,
				err_log->lba,
				err_log->vendorInfo);
	return;
}


/**
 * @brief This function called to report an smart log.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context.
 * @param[in] void * log pointer to log page data.
 *
 * @return none.
 */
static void
nvme_report_smart(struct nvme_dev *dev, void *log)
{
	struct smart_log	*smart_log = log;

	EPRINT("Smart Log Critical Info 0x%x, temperature %d, Avail %d%c,"
				"Threshold %d%c, Used %d%c\n",
					smart_log->criticalError,
					smart_log->temperature,
					smart_log->availableSpace, '%',
					smart_log->availableSpaceThreshold, '%',
					smart_log->precentageUsed, '%');
	EPRINT("Smart Log Unit Reads %lld, Unit Writes %lld,"
				"Host Reads %lld, Host Writes %lld\n",
					*(u64 *)smart_log->dataUnitsRead,
					*(u64 *)smart_log->dataUnitsWritten,
					*(u64 *)smart_log->hostReadCommands,
					*(u64 *)smart_log->hostWriteCommands);
	EPRINT("Smart Log Busy time %lld power cycles %lld, "
				"Power On %lld, unsafe Shutdowns %lld\n",
						*(u64 *)smart_log->controllerBusyTime,
						*(u64 *)smart_log->powerCycles,
						*(u64 *)smart_log->powerOnHours,
						*(u64 *)smart_log->unsafeShutdowns);
	EPRINT("Smart Log media Error %lld, No of Error Logs %lld\n",
						*(u64 *)smart_log->mediaErrors,
						*(u64 *)smart_log->numberOfErrorInfoLogs);
	return;
}

/**
 * @brief This function called to report firmware slot information.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context.
 * @param[in] void * log pointer to log page data.
 *
 * @return none.
 */
static void
nvme_report_firmware_info(struct nvme_dev *dev, void *log)
{
	struct firmware_slot_log *slot_log = log;

	EPRINT("Firmware Slot Information current active slot %d\n",
				slot_log->activeFirmwareInfo);
	EPRINT("Firmware Slot 1 Information %lld\n",
				*(u64 *)slot_log->FirmwareRevisionSlot1);
	EPRINT("Firmware Slot 2 Information %lld\n",
				*(u64 *)slot_log->FirmwareRevisionSlot2);
	EPRINT("Firmware Slot 3 Information %lld\n",
				*(u64 *)slot_log->FirmwareRevisionSlot3);
	EPRINT("Firmware Slot 4 Information %lld\n",
				*(u64 *)slot_log->FirmwareRevisionSlot4);
	EPRINT("Firmware Slot 5 Information %lld\n",
				*(u64 *)slot_log->FirmwareRevisionSlot5);
	EPRINT("Firmware Slot 6 Information %lld\n",
				*(u64 *)slot_log->FirmwareRevisionSlot6);
	EPRINT("Firmware Slot 7 Information %lld\n",
				*(u64 *)slot_log->FirmwareRevisionSlot7);
	return;
}


/**
 * @brief This function called to report trigger information.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context.
 * @param[in] void * log pointer to log page data.
 *
 * @return none.
 */
static void
nvme_report_trigger_info(struct nvme_dev *dev, void *log)
{
	EPRINT("Firmware trigger information sent\n");
}


/**
 * @brief This function called to report a log page data.
 *
 * @param[in] struct queue_info *qinfo pointer to nvme admin queue information
 * @param[in] void int  page_id, Log page ID.
 * @param[in] void *log_page pointer to Log page.
 *
 * @return none.
 */
static void
nvme_report_log(struct queue_info *qinfo, int page_id, void *log_page)
{
	struct nvme_dev *dev = qinfo->dev;

	DPRINT11("LOG buff %p, page type %d\n", log_page, page_id);

	switch(page_id) {
		case GLP_ID_ERR_INFO:   	/* 64 bytes */
			nvme_report_error(dev, log_page);
		break;
		case GLP_ID_SMART_HEALTH:   	/* 512 bytes */
			nvme_report_smart(dev, log_page);
		break;
		case GLP_ID_FIRMWARE_SLOT_INFO: /* 512 bytes */
			nvme_report_firmware_info(dev, log_page);
		break;
		case GLP_ID_DIALOG: 	 /* 4096 bytes */
		printk(KERN_INFO
			"Dialog page - needs to report to user space\n");
		break;
		case GLP_ID_AEN_TRIGGER:
			nvme_report_trigger_info(dev, log_page);
			break;
		default:
			break;
	}
	return;
}


/**
 * @brief This function called to process a received LOG PAGE data.
 *
 * @param[in] struct queue_info *qinfo pointer to nvme admin queue information
 * @param[in] struct cmd_info *cmd_info pointer to command to process.
 *
 * @return int the log page identifier
 *
 * @note It is assumed that dev and admin queue locks are held by caller.
 */
static u32
nvme_process_log_page(struct queue_info *qinfo, struct cmd_info *cmd_info)
{
	struct nvme_dev *dev = qinfo->dev;
	dma_addr_t log_phy;
	void	*log_page;
	u32 page_id;
	int size;


	log_page = cmd_info->req;
	log_phy = *(dma_addr_t *)&cmd_info->cmd_param;

	DPRINT11("cmd_info %p, req %p, log_phys %llx\n", cmd_info, log_page,
							(u64)log_phy);
	if (cmd_info->cq_entry.SC) {
		EPRINT("Failed Get Log Page command 0x%x, param %d, result 0x%0x\n",
					cmd_info->nvme_cmd.header.opCode,
				cmd_info->cq_entry.param.cmdSpecific,
				cmd_info->cq_entry.SC);
		return 0;
	}

	/**
	 * Retrieve the log page id we requested
	 */
	page_id = cmd_info->nvme_cmd.cmd.getLogPage.LogPageID & 0xFFFF;
	DPRINT11("cmd_info %p, page_id %d\n", cmd_info, page_id);

	/*
	 * Check if the status received on the completion queue
	 * does not contain any error.
	 */
	if (cmd_info->cq_entry.param.cmdSpecific
	 == SC_CMD_SPC_ERR_INV_LOG_PAGE) {
		EPRINT("Unknown Log page ID %d\n", page_id);
		return 0;
	}

	/**
	 * Determine log size based on its page_id.
	 */
	switch (page_id) {

	case GLP_ID_ERR_INFO:   	   /* 64 Bytes */
		size = 64;
		break;
	case GLP_ID_SMART_HEALTH:   		 /* 512 bytes */
		size = 512;
		break;
	case GLP_ID_FIRMWARE_SLOT_INFO: 	 /* 512 bytes */
		size = 512;
		break;
	case GLP_ID_DIALOG: 	 /* 4096 bytes */
		size = PAGE_SIZE;
		break;
	case GLP_ID_AEN_TRIGGER:
		size = GLP_ID_AEN_TRIGGER_LOGPGSIZ;
		break;
	default:
		EPRINT("Unknown Log page ID %d\n", page_id);
		return page_id;
	}

#ifdef NVME_DEBUG
	if (nvme_dbg & NVME_DEBUG_LOG)  	/* @todo - Debug only */
	{
		int  i;
		u32 *ptr;
		ptr = (u32 *)log_page;
		for (i=0; i<size/sizeof(u32); i += 4) {
			DPRINT("%02x: %08x %08x %08x %08x\n", i,
			ptr[i], ptr[i+1], ptr[i+2], ptr[i+3]);
		}
		}
#endif
	nvme_report_log(qinfo, page_id, log_page);

#if 0
	/**
	 * optional code to queue events to user application.
	 */
{
	void *  buff;
	struct event_info *event_info;

	/**
	 * This is is used to relay Events to user.
	 * @todo - Must use event_lock instead of dev_lock.
	 */
	event_info = kzalloc(sizeof(*event_info), GFP_KERNEL);
	if (!event_info) {
		EPRINT("Memory allocation error for \"event_info\"\n");
		return;
	}

	buff = kzalloc(MAX_PAGE_SIZE, GFP_KERNEL);
	if (NULL == buff) {
		EPRINT("Memory allocation error for \"page data\"\n");
		kfree(event_info);
		return;
	}

	/**
	 * Copy event Page information and release DMA memory.
	 * Specify event type and page information.
	 * Add event to list of pending events.
	 */
	nvme_memcpy_64(buff, log_page, size/sizeof(u64));
	event_info->buff = buff;
	event_info->event_id = page_id;

	INIT_LIST_HEAD(&event_info->list);
	dev->lock_func(&dev->elock, &dev->elock_flags);
	list_add_tail(&event_info->list, &dev->event_list);

	/**
	 * We can maintain last MAX_EVENTS entries in dev list.
	 * All additional events are delete on FIFO basis.
	 */
	if (++dev->event_count > MAX_EVENTS) {
		event_info = list_first_entry(&dev->event_list,
							struct event_info, list);
		DPRINT("Event Overflow, deleting %p %d\n",
						event_info, event_info->event_id);
		list_del(&event_info->list);
		kfree(event_info->buff);
		kfree(event_info);
	}
	DPRINT("Signaling Event pending..... %p\n", &dev->event_wait);
	wake_up(&dev->event_wait);
	dev->unlock_func(&dev->elock, &dev->elock_flags);
}
#endif
	if((GLP_ID_DIALOG == page_id) || (GLP_ID_AEN_TRIGGER == page_id))
		rms200_prepare_and_send_page(page_id, size, log_page);
	return page_id;
}


/**
 * @brief This function called to lookup for a completed comtext type.
 *
 * @param[in] struct queue_info *qinfo pointer to nvme admin queue information
 * @param[in] u32 context_type The context to search in active list.
 *
 * @return struct cmd_info * cmd_info Found matching context, otherwise NULL
 *
 * @note It is assumed that queue lock is held by caller.
 */
static struct cmd_info *
nvme_log_completed(struct queue_info *qinfo, u32 context_type)
{
	struct cmd_info *cmd_info;
#ifdef NVME_DEBUG
	struct nvme_dev *dev = qinfo->dev;
#endif
	DPRINT11("qinfo %p, Context TYPE %d\n", qinfo, context_type);
	list_for_each_entry(cmd_info, &qinfo->cmd_active, list) {

		if ((cmd_info->type == context_type) && (cmd_info->status != -1)) {
		DPRINT11("Completed cmd_info %p, Context status %d\n",
						cmd_info, cmd_info->status);
		return (cmd_info);
		}
	}
	return (NULL);
}


/**
 * @brief This function called to check for any completed log pages.
 *
 * @param[in] struct queue_info *qinfo pointer to nvme admin queue information
 *
 * @return int 0 if successful, otherwise Error Code
 */
static void
nvme_check_log(struct queue_info *qinfo, u32 context)
{
	struct nvme_dev *dev = qinfo->dev;
	struct cmd_info *cmd_info;
	u32 page_id;
	size_t page_size;

	DPRINT11("qinfo %p, context %d\n", qinfo, context);
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	do {

		cmd_info = nvme_log_completed(qinfo, context);
		if (!cmd_info) {
		break;
		}
		DPRINT11("cmd_info %p, LOG type %d\n", cmd_info,
						cmd_info->cq_entry.param.cmdSpecific);

		page_id = nvme_process_log_page(qinfo, cmd_info);
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		/*
		 * If getting dialog log page, a larger chunk was allocated
		 */
		page_size = LOG_PG_SIZE;
		if (page_id == GLP_ID_DIALOG) {
			page_size = PAGE_SIZE;
		}
		else if(GLP_ID_AEN_TRIGGER == page_id) {
			page_size = GLP_ID_AEN_TRIGGER_LOGPGSIZ;
		}

		dma_free_coherent(dev->dma_dev, page_size,
					cmd_info->req, (dma_addr_t)cmd_info->cmd_param);
		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
		qinfo->timeout[cmd_info->timeout_id]--;
		nvme_put_cmd(qinfo, cmd_info);

	} while (cmd_info);
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
}


/**
 * @brief This function called to process a received Error with more bit set.
 *  	Commands completed with more bit set, require additional
 *  	processing to retrieve error log page.
 *
 * @param[in] struct queue_info *qinfo pointer to nvme admin queue information
 * @param[in] struct cmd_info *cmd_info pointer to command information.
 * @param[in] struct cq_entry *cq_entry pointer to error completion queue entry.
 *
 * @return int 0 if successful, otherwise Error Code
 */
static void
nvme_get_error_log(struct nvme_dev *dev, struct cmd_info *cmd_info)
{
	dma_addr_t buff_phy;
	void	*buff;
	u32 param;
	int  size;

	DPRINT("cmd_info %p, Error Completion cmdSpecific %d\n", cmd_info,
						cmd_info->cq_entry.param.cmdSpecific);
	param = GLP_ID_ERR_INFO;		/* 64 Bytes */
	size = 64;

	buff = dma_alloc_coherent(dev->dma_dev, LOG_PG_SIZE,
							&buff_phy, GFP_KERNEL);
	if (!buff) {
		EPRINT("memory Allocation failure for Error Log.\n");
		return;
	}
	param |= size << 16;
	if (nvme_get_log_page(dev, param, cmd_info, buff,
					(struct nvme_prp *)&buff_phy) < 0) {
		DPRINT("Failed requesting for log page infor\n");
		dma_free_coherent(dev->dma_dev, LOG_PG_SIZE, buff, buff_phy);
	}
}


/**
 * @brief This function called to process a received AEN response.
 *
 * @param[in] struct queue_info *qinfo pointer to nvme admin queue information
 *
 * @return int 0 if successful, otherwise Error Code
 *
 * @note It is assumed that queue lock is held by caller.
 */
static void
nvme_process_aen(struct queue_info *qinfo)
{
	struct nvme_dev *dev = qinfo->dev;
	struct cmd_info *cmd_info;
	struct cq_entry *cq_entry;
	dma_addr_t       buff_phy;
	void	        *buff;
	u32              param;
	u8				 log_page_id;
	int              size;
	size_t           page_size;

	DPRINT11("qinfo %p\n", qinfo);
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	do {

		cmd_info = nvme_log_completed(qinfo, EVENT_CONTEXT);
		if (!cmd_info) {
		break;
		}
		--dev->cur_aen;
		if (cmd_info->cmd_status) {
			EPRINT("Get log page error condition 0x%04x\n",
							cmd_info->cmd_status);
			nvme_put_cmd(qinfo, cmd_info);
		continue;
		}
		cq_entry = &cmd_info->cq_entry;
		log_page_id = (cq_entry->param.cmdSpecific >> 16) & 0xFF;
		DPRINT11("AEN type %d, Info %d, page ID %d\n",
					cq_entry->param.cmdSpecific & 0x07,
					(cq_entry->param.cmdSpecific >> 8) & 0xFF,
					log_page_id);
		/* Assume normal log page */
		page_size = LOG_PG_SIZE;

		switch (cq_entry->param.cmdSpecific & 0x7) {

		case AER_ERR_STATUS:
			param = GLP_ID_ERR_INFO;
			size = 64;  			/* 64 Bytes */
			break;
		case AER_SMART_HEALTH_STATUS:
			param = GLP_ID_SMART_HEALTH;
			size = 512; 			/* 512 bytes */
			break;
		case AER_VENDOR_SPECIFIC:
			switch(log_page_id) {
			case GLP_ID_AEN_TRIGGER:
				param = GLP_ID_AEN_TRIGGER;
				page_size = size = GLP_ID_AEN_TRIGGER_LOGPGSIZ;
				break;

			case GLP_ID_DIALOG:
			default:
				param = GLP_ID_DIALOG;
				/* If getting dialog log page, a larger chunk
			 	 * needs to be allocated
			 	 */
				page_size = size = PAGE_SIZE;
				break;
			}
			break;

		default:
			EPRINT("Unsupported Asynchronous Notification type <%d>"
							" ignored\n",
						cq_entry->param.cmdSpecific);
				nvme_put_cmd(qinfo, cmd_info);
			continue;
		}
		nvme_put_cmd(qinfo, cmd_info);
		qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		buff = dma_alloc_coherent(dev->dma_dev, page_size,
								&buff_phy, GFP_KERNEL);
		if (!buff) {
			EPRINT("Memory Allocation error Event type <%d>, ignored\n",
						cq_entry->param.cmdSpecific);
			qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
			continue;
		}
		param |= size << 16;
		if (nvme_get_log_page(dev, param, NULL, buff,
						(struct nvme_prp *)&buff_phy) < 0) {
			EPRINT("Failed requesting log page, ignoring AEN\n");
			dma_free_coherent(dev->dma_dev, page_size, buff, buff_phy);
		}
		qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);

	} while (cmd_info);
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);

	/**
	 * Issue AEN request.
	 */
#if SEND_AEN
	nvme_send_aen(dev);
#endif
}


/**
 * @brief This function allocates a disk device list
 *  	This function is called from probe and rescan functions to
 *  	create list of disk devices currently discovered.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static void
nvme_alloc_disks(struct nvme_dev *dev)
{
	int ns_id, result;
	struct ns_info *ns;

	/**
	 * For number of Namespaces discovered:
	 *
	 * a. get Namespace identify data
	 * b. Create a block device queue
	 * c. create a disk device.
	 * d. add Namespace to list of devices
	 */
	for (ns_id = 1; ns_id <= dev->ns_count; ns_id++) {

		DPRINT2("allocating Namespace %d\n", ns_id);
		ns = nvme_alloc_ns(dev, ns_id, 0);
		if (! ns) {
			EPRINT("Failed to allocate NS information structure.\n");
			continue;
		}

#if USE_NS_ATTR
		/**
		 * Get Namespace attributes/
		 */
		if (nvme_ns_attr(dev, ns_id, ns)) {
			EPRINT("Failed get NS attributes, ignoring namespace.\n");
			continue;
		}
#endif
		result = nvme_attach_ns(dev, ns, 0);
		if (result) {
			EPRINT("Failed to create disk device for NS %d\n", ns->id);
		}
	}
}

/**
 * @brief This function Frees list of disk devices
 *  	This function is called from probe and rescan functions to
 *  	create list of disk devices currently discovered.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_free_disks(struct nvme_dev *dev)
{
	struct ns_info  *ns, *tmp;

	list_for_each_entry_safe(ns, tmp, &dev->ns_list, list) {

		DPRINT2("NS [%d], removing Disk %s, device %p\n",
					ns->id, ns->disk->disk_name, ns->disk);
		if (ns->flags & NS_ONLINE) {
			ns->flags &= ~NS_ONLINE;
			if (ns->disk->flags & GENHD_FL_UP)
				del_gendisk(ns->disk);
			put_disk(ns->disk);
			blk_cleanup_queue(ns->queue);
			ns->disk = NULL;
			ns->queue = NULL;
		}
	}

	list_for_each_entry_safe(ns, tmp, &dev->ns_list, list) {

		DPRINT2("NS [%d], releasing resource %p\n", ns->id, ns);
		nvme_free_ns(dev, ns);
	}
	return 0;
}


/**
 * @brief This function Checks Completion Queue tail for new completion(s).
 *
 * @param[in] int irq, IRQ id
 * @param[in] void *data, context passed when allocating IRQ (i.e qinfo).
 *
 * @return irqreturn_t, result of interrupt processing (IRQ_HANDLED)
 */
static irqreturn_t
nvme_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_HANDLED;
	struct queue_info * qinfo = data;
	struct nvme_dev *dev = qinfo->dev;

	dev->dma_int++;
	qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
	result = nvme_process_cq(qinfo);
	qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
	return (result);
}


/**
 * @brief This function Checks Completion Queue tail for new completion(s).
 *
 * @param[in] int irq, IRQ id
 * @param[in] void *data, context passed when allocating IRQ (i.e qinfo).
 *
 * @return irqreturn_t, result of interrupt processing (IRQ_HANDLED)
 */
static irqreturn_t
nvme_check_isr(int irq, void *data)
{
	struct queue_info *qinfo = data;
	struct cq_entry *  cq;

	cq = &qinfo->compq[qinfo->head];
	if (cq->phaseTag != qinfo->phase)
		return (IRQ_NONE);
	return (IRQ_WAKE_THREAD);
}

/**
 * @brief This function Checks Completion Queue tail for new completion(s).
 *
 * @param[in] int irq, IRQ id
 * @param[in] void *data, context passed when allocating IRQ (i.e qinfo).
 *
 * @return irqreturn_t, result of interrupt processing (IRQ_HANDLED)
 */
static irqreturn_t
nvme_intx_isr(int irq, void *data)
{
	struct nvme_dev *dev = data;
	struct queue_info *qinfo;
	struct cq_entry *  cq;
	irqreturn_t status;
	int i;

	for (i = 0; i <= dev->que_count; i++) { /* Include Admin queue */

		qinfo = dev->que_list[i];
		if (!qinfo)
		continue;

		cq = &qinfo->compq[qinfo->head];
		if (cq->phaseTag != qinfo->phase)
		continue;
		status = IRQ_HANDLED;
		if (i) {
		/**
		 * Process IO queue completion via worker thread.
		 */
			schedule_work_on(qinfo->cpu, &qinfo->wq_work);
		} else {
			/**
			 * Process admin queue inline.
			 */
			qinfo->lock_func(&qinfo->lock, &qinfo->lock_flags);
			nvme_process_cq(qinfo);
			qinfo->unlock_func(&qinfo->lock, &qinfo->lock_flags);
		}
	}
	return (status);
}

/**
 * @brief This function release IRQ vector assigned to a queue.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct queue_info *qinfo pointer to queue information block
 *
 * @return none.
 */
static void
nvme_release_irq(struct nvme_dev *dev, struct queue_info *qinfo)
{
	if (qinfo->ivec_acquired) {

		DPRINT2("Releasing IRQ qid %d, vector [%d] %d\n",
				qinfo->id, qinfo->cq_ivec,
				dev->msix_entry[qinfo->cq_ivec].vector);
		if (nvme_msix_enable) {

			irq_set_affinity_hint(dev->msix_entry[qinfo->cq_ivec].vector,
							NULL);
		free_irq(dev->msix_entry[qinfo->cq_ivec].vector, qinfo);
		} else {
		free_irq(dev->msix_entry[qinfo->cq_ivec].vector, dev);
		}
	}
	qinfo->ivec_acquired = 0;
}

/**
 * @brief This function request an IRQ vector and install interrupt handler.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] struct queue_info *qinfo pointer to queue information block
 * @param[in] char *name, name to be used with IRQ.
 * @param[in] int cpu, assigned CPU id (-1 if none).
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_request_irq(struct nvme_dev *dev, struct queue_info *qinfo,
						const char *name, int cpu)
{
	int result;

	if (qinfo->ivec_acquired) {
			DPRINT2("IRQ already acquired cpu %d, qid %d, vector [%d] %d\n",
				cpu, qinfo->id, qinfo->cq_ivec,
				dev->msix_entry[qinfo->cq_ivec].vector);
		return (-1);
	}
	if (nvme_msix_enable) {
		if (cpu >= 0) {
				DPRINT2("CPU Affinity cpu mask %p = 0x%lx\n",
						get_cpu_mask(cpu),
					*((ulong *)get_cpu_mask(cpu)));
				result = irq_set_affinity_hint(
				dev->msix_entry[qinfo->cq_ivec].vector,
				get_cpu_mask(cpu));
				DPRINT2("set CPU affinity result %x\n", result);
		}

		if (use_threaded_interrupts) {
			DPRINT2("irq: cpu %d, qid %d, %s -> vector [%d] %d\n",
				cpu, qinfo->id, name, qinfo->cq_ivec,
				dev->msix_entry[qinfo->cq_ivec].vector);
			result = request_threaded_irq(
				dev->msix_entry[qinfo->cq_ivec].vector,
				nvme_check_isr, nvme_isr,
				IRQF_SHARED, name, qinfo);
		} else
		{
			result = request_irq(dev->msix_entry[qinfo->cq_ivec].vector,
				nvme_isr,
				IRQF_SHARED, name, qinfo);
		}

	} else {
		DPRINT2("INTx: cpu %d, qid %d, %s -> vector [%d] %d\n",
				cpu, qinfo->id, name, qinfo->cq_ivec,
				dev->msix_entry[qinfo->cq_ivec].vector);
		use_threaded_interrupts = 0;
		nvme_pci_enable_intx(dev->pci_dev);
		result = request_irq(dev->msix_entry[qinfo->cq_ivec].vector,
				nvme_intx_isr, IRQF_SHARED,
				name, dev);
	}
	if (! result) {
		qinfo->ivec_acquired = 1;
		DPRINT2("Assigned cpu %d, qid %d, affinity %s vector [%d]"
					" %03d\n",
				cpu, qinfo->id, name, qinfo->cq_ivec,
				dev->msix_entry[qinfo->cq_ivec].vector);
	} else {
		EPRINT("Failed to assign cpu %d, qid %d, affinity %s vector [%d]"
					" %03d\n",
				cpu, qinfo->id, name, qinfo->cq_ivec,
				dev->msix_entry[qinfo->cq_ivec].vector);
	}
	return (result);
}

/**
 * @brief This is a dynamic lock function
 *
 * @param[in] spinlock_t *lock, pointer to sin lock
 * @param[in] ulong *flags, pointer to flags used for spin lock
 *
 * @return none.
 */
static void
nvme_spin_lock(spinlock_t *lock, ulong *flags)
{
	spin_lock_irqsave(lock, (*flags));
}

/**
 * @brief This is a dynamic unlock function
 *
 * @param[in] spinlock_t *lock, pointer to sin lock
 * @param[in] ulong *flags, pointer to flags used for spin lock
 *
 * @return none.
 */
static void
nvme_spin_unlock(spinlock_t *lock, ulong *flags)
{
	spin_unlock_irqrestore(lock, (*flags));
}

/**
 * @brief This is used to prevent CPU preemption
 *  	This function is used as a dynamic lock function for
 *  	non-shared queues.
 *
 * @param[in] none.
 *
 * @return none.
 */
static void
nvme_get_cpu(spinlock_t *lock, ulong *flags)
{
	preempt_disable();
}

/**
 * @brief This is used to prevent CPU preemption
 *  	This function is used as a dynamic lock function for
 *  	non-shared queues.
 *
 * @param[in] none.
 *
 * @return none.
 */
static void
nvme_put_cpu(spinlock_t *lock, ulong *flags)
{
	preempt_enable();
}

/**
 * @brief This function assigns an MSI-X IRQ vector.
 *  	The IRQ vector assigned, depends on number of available
 *  	vectors and number of IO queues. Admin queue is always
 *  	assigned vector ID 0.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 * @param[in] int qid, queue ID.
 *
 * @return int entry, MSI-X vector ID table entry.
 */
static int
nvme_assign_ivec(struct nvme_dev *dev, u32 qid, int *ishared)
{
	int ivec;

	/**
	 * Admin queue is always shared and gets vector index 0
	 */
	if (! qid)  			/** admin queue is always 0 */
		return (qid);

	*ishared = 0;
	if (dev->vector_cnt > num_online_cpus()) {
		ivec = qid;
	} else if (dev->vector_cnt > num_online_nodes()) {
		ivec = ((qid - 1)/num_online_nodes()) + 1;
		*ishared = (qid-1) % num_online_nodes();
	}
	else {
		ivec = 0;
		*ishared = 1;
	}
	DPRINT2("Assigned ivec %d, for qid %d, %sshared\n", ivec, qid,
						*ishared ? "" : "Not ");
	return (ivec);
}

/**
 * @brief This function attemps to create an msix vector per CPU.
 *  	If we are not granted, we shall try to allocate MSI-X vectors
 *  	per NUMA node, and if that fails, we'll fall back to single
 *  	IRQ and all queues would shared.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_enable_msix(struct nvme_dev *dev)
{

	int result, nr_cpu, i, nr_vec, nr_node;

	nr_cpu = num_online_cpus();
	nr_node = num_online_nodes();

	DPRINT2("nr_cpus  %d, nr_nodes %d\n",
			num_online_cpus(), num_online_nodes());

	nr_vec = nr_cpu + 1;
	for (i = 0; i < nr_vec; i++)
		dev->msix_entry[i].entry = i;

	for (;;) {
		result = pci_enable_msix(dev->pci_dev, dev->msix_entry, nr_vec);
		if (result == 0) {
		break;
#if NVME_NUMA_VECTORS
		} else if ((result > 0) && (result >= nr_node)) {
		nr_vec = result;
		continue;
#endif
		} else {
		nr_vec = 1;
		nvme_msix_enable = 0;
		break;
		}
	}

	dev->vector_cnt = nr_vec;
	DPRINT2("nr_vectors  %d\n", dev->vector_cnt);
	return (nr_vec);
}

/**
 * @brief This function releases MSIx vectors.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return none.
 */
static void
nvme_disable_msix(struct nvme_dev *dev)
{

	DPRINT2("nr_vectors  %d\n", dev->vector_cnt);

	if (dev->vector_cnt)
		pci_disable_msix(dev->pci_dev);
	dev->vector_cnt = 0;
}


/**
 * @brief This function validate device parameters.
 *  	Device parameters may be overwritten prior to driver
 *  	initialization. We need to validate these changes to make
 *  	sure they are within operational range of controller
 *  	capability and driver limitations.
 *  	Any parameters outside of its supported range are reported
 *  	and corrected accordingly.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return returns int 0 if successful, otherwise Error Code
 *
 */
static int
nvme_validate_params(struct nvme_dev *dev)
{
	u64 min_page, max_page, hw_max_qs;

	nvme_hw_cap = readq(dev->reg + NVME_CAP);
	hw_max_qs = (nvme_hw_cap & NVME_CAP_MQES_MSK64) + 1;

	DPRINT("Controller Capability reg: %016llx\n", nvme_hw_cap);

	if (hw_max_qs && ((io_cpl_queue_size > hw_max_qs) ||
					(io_sub_queue_size > hw_max_qs))) {

		EPRINT("Invalid parameter: maximum HW queue size %llu\n",
							hw_max_qs);
		NPRINT("Adapting Hardware suggested queue size.\n");

		if (io_cpl_queue_size > hw_max_qs)
			io_cpl_queue_size = hw_max_qs;

		if (io_sub_queue_size > hw_max_qs)
			io_sub_queue_size = hw_max_qs;
	}

	/**
	 * Validate number of command IDs to context size (16 bits).
	 * Limit number of command issued to number of command IDs available.
	 */
	if (io_command_id_size > 65535) {
		io_command_id_size = 65535;
		EPRINT("Adjusting \"io_command_id_size\" to %d\n",
							io_command_id_size);
	}
	if (max_io_request > io_command_id_size) {
		max_io_request = io_command_id_size;
		EPRINT("Adjusting \"max_io_request\" to %d\n", io_command_id_size);
	}

	min_page = (1 << (((nvme_hw_cap & NVME_CAP_MPSMIN_MSK64) >>
				NVME_CAP_MPSMIN_LSB) + 12));
	max_page = (1 << (((nvme_hw_cap & NVME_CAP_MPSMAX_MSK64) >>
				NVME_CAP_MPSMAX_LSB) + 12));
	DPRINT("hardware maximum page size %llu\n", max_page);
	DPRINT("hardware minimum page size %llu\n", min_page);

	if ((max_page < PAGE_SIZE) || (min_page > PAGE_SIZE)) {
		EPRINT("Controller does not support OS default Page size %lu\n",
							PAGE_SIZE);
		return (-EINVAL);
	}
	return (0);
}


/**
 * @brief This function sets controller features according to current driver
 *  	selected interrupt coalescing parameters.
 *  	This function is called once during driver probe time and also
 *  	upon driver parameter update.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
static int
nvme_intr_coalescing(struct nvme_dev *dev)
{
	int result = 0;
	struct cq_entry cq;
	u32 param;

	DPRINT("setting intr coalescing feature %d %d\n",
			intr_coalescing_time, intr_coalescing_threshold);

	if (intr_coalescing_threshold || intr_coalescing_time) {
		result |= nvme_set_feature(dev, FTR_ID_INT_COALESCING,
		(intr_coalescing_time << 8) | intr_coalescing_threshold,
								NULL, NULL);
	}

	result = nvme_get_feature(dev, 0, FTR_ID_INT_COALESCING, 0, NULL, &cq);
	if (result) {
		EPRINT("Failed to validate feature status %d\n", result);
	}
	param = cq.param.cmdSpecific;
	DPRINT("returned param 0x%x\n", param);
	if (((param >> 8) != intr_coalescing_time) ||
			((param & 0xFF) != intr_coalescing_threshold)) {
		EPRINT("Param validation error returned value 0x%x\n", param);
		result = -EINVAL;
	}
	return (result);
}



/**
 * @brief This function is driver worker thread that performs tasks that cannnot
 *  	be performed inline.
 *  	This thread is used to synchronous commands that require
 *  	special request handling. Thread created during driver
 *  	initialization and stopped when hardware is shutdown.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return none.
 *
 */
static int
nvme_thread(void *data)
{
	struct nvme_dev *dev = (struct nvme_dev *)data;

	DPRINT1("Starting ......  device context %p\n", dev);
	while (! kthread_should_stop()) {

		__set_current_state(TASK_RUNNING);
#ifdef NVME_DEBUG
		if (dev->thread_flags)
			DPRINT("thread_flags 0x%0x\n",  dev->thread_flags);
#endif

		if (dev->thread_flags & THREAD_STOP)
			break;

		dev->lock_func(&dev->lock, &dev->lock_flags);
		if (dev->thread_flags & THREAD_RESTART) {
		dev->thread_flags = 0;
		if (dev->state <= NVME_STATE_SUSPEND) {
				dev->unlock_func(&dev->lock, &dev->lock_flags);
			nvme_hw_reset(dev);
			continue;
		}
			dev->unlock_func(&dev->lock, &dev->lock_flags);
		continue;
		}

#ifdef NVME_DEBUG
		/**
		 * show stalled queues.
			 */
		if (nvme_idle_count++ > 120) {
		struct queue_info *dqinfo;
		int i;

			for (i = 1; i <= dev->que_count; i++) {

				dqinfo = dev->que_list[i];
				if (! dqinfo)
				continue;

			if (nvme_dbg & NVME_DEBUG_DUMP_Q) {
				dqinfo->lock_func(&dqinfo->lock, &dqinfo->lock_flags);
			DPRINT("Q flags 0x%x sub-entries %d, max_io_req %d\n",
				dqinfo->flags,
				dqinfo->sub_queue->entries, max_io_request);
				nvme_dump_list(dqinfo);
				dqinfo->unlock_func(&dqinfo->lock, &dqinfo->lock_flags);
			}
			}
			nvme_idle_count = 0;
		}
#endif
		if (dev->thread_flags & THREAD_AEN) {
			dev->thread_flags &= ~THREAD_AEN;
			dev->unlock_func(&dev->lock, &dev->lock_flags);
			nvme_process_aen(dev->admin_queue);
			dev->lock_func(&dev->lock, &dev->lock_flags);
		}
		if (dev->thread_flags & THREAD_LOG) {
			dev->thread_flags &= ~THREAD_LOG;
			dev->unlock_func(&dev->lock, &dev->lock_flags);
			nvme_check_log(dev->admin_queue, LOG_CONTEXT);
			dev->lock_func(&dev->lock, &dev->lock_flags);
		}
		if (dev->thread_flags & THREAD_ERR) {
			dev->thread_flags &= ~THREAD_ERR;
			dev->unlock_func(&dev->lock, &dev->lock_flags);
			nvme_check_log(dev->admin_queue, ERR_CONTEXT);
			dev->lock_func(&dev->lock, &dev->lock_flags);
		}
		dev->unlock_func(&dev->lock, &dev->lock_flags);

		/*
		 * Make it "interruptible" just to keep it from
		 * counting toward the load average.
		 */
		wait_event_interruptible(dev->thread_wait,
		dev->thread_flags || kthread_should_stop());
	}
	dev->nvme_thread = NULL;
	DPRINT1("Terminating flags 0x%x\n", dev->thread_flags);
	return 0;
}


/**
 * @brief This function releases device instance.
 *  		This function uses kernel idr library to release
 *  		device instance.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return none.
 */
static void
nvme_put_instance(struct nvme_dev *dev)
{
	mutex_lock(&nvme_idr_mutex);
	idr_remove(&nvme_instance_idr, dev->instance);
	mutex_unlock(&nvme_idr_mutex);
}

/**
 * @brief This function allocate device instance.
 *  		This function uses kernel idr library to allocate
 *  		device instance.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 * @note nvme_dev.instance, set to next available instance ID.
 */
static int
nvme_get_instance(struct nvme_dev *dev)
{
	int status = 0, idx;

	mutex_lock(&nvme_idr_mutex);
#ifdef IDR_FULL /* older kernels */
	do {
		if (!idr_pre_get(&nvme_instance_idr, GFP_KERNEL)) {
		status = -ENOMEM;
		break;
		}
		status = idr_get_new(&nvme_instance_idr, dev, &idx);
		if (status == 0 && idx >= MAX_CONTROLLER) {
		idr_remove(&nvme_instance_idr, idx);
		status = -ENOSPC;
		break;
		}
	} while (status == -EAGAIN);
#else   	/* newer kernels */
	idx = idr_alloc(&nvme_instance_idr, dev,
			0, MAX_CONTROLLER, GFP_KERNEL);
	if (idx < 0)
		status = idx;
#endif
	mutex_unlock(&nvme_idr_mutex);

	dev->instance = idx;
	return status;
}


static struct class *nvme_class = NULL;

/**
 * @brief Linux driver entry point for NVME controller discovery.
 *  All controller initialization and resources allocation
 *  is done here. Once we return from this function, Namespaces
 *  have been discovered, disk nodes created, hardware queues
 *  and their corresponding IRQ vectors assigned and driver is
 *  ready to perfom IO requests.
 *
 * @param[in] struct pci_dev *pci_dev, pointer to PCi driver allocated PCI dev.
 *
 * @return returns int 0 if successful, otherwise Error Code
 */
// @todo  static int __devinit nvme_probe(struct pci_dev *pci_dev,
static int
nvme_probe(struct pci_dev *pci_dev, const struct pci_device_id *id)
{

	int 	bar, result = -ENOMEM;
	int nr_io_queues;
	struct  nvme_dev *dev;
	char	buf[64];
	struct  nvme_cmd entry;
	resource_size_t reg_start, reg_len;

	dev_printk(KERN_INFO, &pci_dev->dev, "%s %s\n",
			nvme_pci_id(pci_dev), nvme_pci_info(pci_dev));

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (! dev) {
		EPRINT("Context allocation error.\n");
		return (result);
	}

	/**
	 * allocate MSIx interrupt vector information entries
	 */
	dev->msix_entry = kcalloc(num_online_cpus()+1, sizeof(*dev->msix_entry),
								GFP_KERNEL);
	if (! dev->msix_entry)
		goto free;

	/**
	 * Initialize Device structure.
	 */
	spin_lock_init(&dev->lock);
	dev->lock_func   = nvme_spin_lock;
	dev->unlock_func = nvme_spin_unlock;

	dev->pci_dev	= pci_dev;
	dev->dma_dev	= &pci_dev->dev;
	dev->state  = NVME_STATE_INIT;
	dev->timeout_id = 0;
	dev->major  = nvme_major;

	DPRINT1("Dev %p [Major %d/%d] pci_dev %p, drvdata %p\n",
					dev, nvme_cmajor, nvme_major,
					pci_dev, pci_get_drvdata(pci_dev));
	DPRINT1("%s Config/Alloc PCI resources\n", nvme_pci_id(pci_dev));

	/**
	 * Enable PCI device Bus master operation.
	 * Select memory mapped BAR address 0.
	 * Enable 64bit DMA operation.
	 */
	if (pci_enable_device_mem(pci_dev)) {
		result = -ENXIO;
		goto free;
	}
	pci_set_master(pci_dev);
	bar = pci_select_bars(pci_dev, IORESOURCE_MEM);

	DPRINT1("Selecting memory region %d\n", bar);
	if (pci_request_selected_regions(pci_dev, bar, DEV_NAME)) {
		result = -EBADF;
		goto disable;
	}
	pci_set_drvdata(pci_dev, dev);
	dma_set_mask(&pci_dev->dev, DMA_BIT_MASK(64));
	dma_set_coherent_mask(&pci_dev->dev, DMA_BIT_MASK(64));

	/**
	 * Map controller BAR address 0 to Kernel address space.
	 */
	reg_start = pci_resource_start(pci_dev, 0);
	reg_len = pci_resource_len(pci_dev, 0);
	if (reg_start != 0 && reg_len >= 8192)
		dev->reg = ioremap(reg_start, 8192);
	if (! dev->reg) {
		result = -EBADF;
		goto release_region;
	}
	DPRINT1("BAR address 0x%p\n", dev->reg);

	rms200_map_bar4(dev);

	/**
	 * Make sure we can operate with existing parameters.
	 */
	if ((result = nvme_validate_params(dev))) {

		EPRINT("Invalid Device parameters, terminating %d\n", result);
		goto release_bar;
	}

	nvme_get_instance(dev);
	init_waitqueue_head(&dev->thread_wait);
	dev->nvme_thread = kthread_run(nvme_thread, dev, "%s_thread %d",
						DRV_NAME, dev->instance);
	if (IS_ERR(dev->nvme_thread))
		goto release_instance;

	max_prp_list = (transfer_size * 1024)/PAGE_SIZE;

	DPRINT1("Max xfer %d, Max PRP %d\n", transfer_size, max_prp_list);

	/**
	 * Allocate Admin queue resources and start controller.
	 */
	if ((result = nvme_admin_cons(dev)))
		goto kill_thread;

	result = nvme_hw_start(dev);
	if (result < 0)
		goto free_adminq;

	/**
	 * Allocate MSIx interrupt vectors and assign vector 0 to Admin queue.
	 */
	if (nvme_msix_enable) {
		dev->vector_cnt = nvme_enable_msix(dev);
		DPRINT1("nr vectors %d\n", dev->vector_cnt);
		if (nvme_request_irq(dev, dev->admin_queue, "nvme-Admin", -1) < 0) {
			EPRINT("Failed to assign irq[%d]\n", dev->pci_dev->irq);
			goto free_msix;
		}
	} else {
		dev->vector_cnt = 1;
		if (nvme_request_irq(dev, dev->admin_queue, "nvme-INTx", -1) < 0) {
			EPRINT("Failed to assign INTx irq[%d]\n", dev->pci_dev->irq);
			goto free_msix;
		}
	}

	dev->identify = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (! dev->identify) {
		EPRINT("Controller Identify data allocation error.\n");
		goto free_msix;
	}
	result = nvme_get_ctrl_identify(dev);
	if (result) {
		EPRINT("Controller Failure.\n");
		goto free_msix;
	}

	snprintf(buf, sizeof(dev->identify->serialNum), "%s",
			 dev->identify->serialNum);

	IPRINT("controller:\t%d\n", dev->instance);
	IPRINT("serial no:\t%s\n", buf);
	snprintf(buf, sizeof(dev->identify->modelNum), "%s",
			 dev->identify->modelNum);

	IPRINT("model no:\t%s\n", buf);
	snprintf(buf, sizeof(dev->identify->firmwareRev) + 1, "%s",
			 dev->identify->firmwareRev);
	IPRINT("firmware rev:\t%s\n", buf);
	DPRINT("Admin Cmd Vendor Cfg  0x%0x\n", dev->identify->admVendCmdCfg);
	DPRINT("NVM Cmd Vendor Cfg  0x%0x\n", dev->identify->nvmVendCmdCfg);

	/**
	 * zero based number of Namespaces.
	 */
	dev->max_aen = dev->identify->asyncReqLmt+1;	/* Zero based value */
	if (dev->max_aen > MAX_EVENTS)
		dev->max_aen = MAX_EVENTS;
	dev->ns_count = dev->identify->numNmspc;

	DPRINT1("number of Namespaces %d\n", dev->ns_count);
	rms_check_ns_count(dev);

	/**
	 * Ideally we should be able to allocate an IO queue with a unique
	 * IRQ vector per CPU.
	 */
	nr_io_queues = num_online_cpus();
	DPRINT1("requesting %d IO queues\n", nr_io_queues);

	/**
	 * Determine number of queues required for optimum performance.
	 */
	result = nvme_request_queues(dev, &nr_io_queues);
	if (result) {
		EPRINT("Failed to allocate hardware IO Queue error.\n");
		goto free_msix;
	}
	DPRINT("Got %d hw IO queues\n", nr_io_queues);

	/**
	 * Allocate IO queue information blocks, required DMA resources
	 * and register IO queues with controller.
	 */
	result = nvme_create_io_queues(dev, nr_io_queues);
	if (result) {
		EPRINT("Failed to allocate IO Queues %d.\n", result);
		goto deleteq;
	}

	/**
	 * Setup controller features according to current device parameters.
	 */
	result = nvme_intr_coalescing(dev);
	if (result) {
		EPRINT("Failed to set features %d.\n", result);
	}
#if SEND_AEN
	nvme_send_aen(dev);
#endif

	if (nvme_cmajor) {
			dev->ctrl_dev = device_create(nvme_class,
				NULL, MKDEV(nvme_cmajor, dev->instance), dev,
			"%s%d", DEV_NAME, dev->instance);
		if (dev->ctrl_dev && !IS_ERR(dev->ctrl_dev)) {
			DPRINT("Created control device %p, minor %d\n",
					dev->ctrl_dev, dev->instance);
		} else {
			EPRINT("Control device minor %d, error %p\n",
					dev->instance, dev->ctrl_dev);
		}
	}

	/**
	 * Initialize kick off timer.
	 * @todo - It is best for timer to be per queue basis.
	 * Since queue lock only prevents preemption.
	 */
	init_timer(&dev->timer);
	setup_timer(&dev->timer, nvme_timeout, ((ulong)dev));
	dev->timer.expires = jiffies + TIMEOUT_FREQ;
	add_timer(&dev->timer);
	init_waitqueue_head(&dev->event_wait);
	INIT_LIST_HEAD(&dev->event_list);

	/**
	 * Device is now operational.
	 */
	dev->state = NVME_STATE_OPERATIONAL;

	/**
	 * Allocate Namespace control blocks, create disk devices
	 * and register block device interface.
	 */
	INIT_LIST_HEAD(&dev->ns_list);
	nvme_alloc_disks(dev);

	if (rms200_alloc_char_dev(dev) < 0) {
		EPRINT("Failed adding char device dev %d\n", dev->instance);
	}

	/**
	 * Radian specific code - let dialog know of the timestamp
	 */
	if (nvme_rms_send_timestamp(dev, &entry))
		(void) nvme_send_admin(dev, &entry, NULL, ADMIN_TIMEOUT);

	dev->proc_dir = proc_mkdir(dev_name(dev->ctrl_dev), NULL);
	dev->proc_entry_stats = proc_create_data("stats", 0444, dev->proc_dir, &rms200_stats_proc_fops, (void *) dev);
	dev->proc_entry_qinfo = proc_create_data("qinfo", 0444, dev->proc_dir, &rms_proc_qinfo_fops, (void *) dev);
	DPRINT("successful probe.....\n");
	return 0;

deleteq:
	/**
	 * Failed to allocate resources or to initialize hardware.
	 * Release all resources and exit with status code.
	 */
	nvme_delete_io_queues(dev, FALSE);

free_msix:
	nvme_release_irq(dev, dev->admin_queue);	/** free temp irq */
	if (nvme_msix_enable)
		nvme_disable_msix(dev);
	nvme_hw_stop(dev, TRUE);
free_adminq:
	if (dev->admin_queue) {
		queue_dest(dev, dev->admin_queue);
	}

kill_thread:
	dev->thread_flags |= THREAD_STOP;
	wake_up(&dev->thread_wait);
	schedule_timeout(HZ);
	if (dev->nvme_thread) {
		kthread_stop(dev->nvme_thread);
	}

release_instance:
	nvme_put_instance(dev);
release_bar:
	if (dev->ddr_remap)
		iounmap(dev->ddr_remap);
	iounmap(dev->reg);
release_region:
	pci_release_regions(pci_dev);
disable:
	pci_disable_device(pci_dev);
free:

	if (dev->identify)
		kfree(dev->identify);
	if (dev->msix_entry)
		kfree(dev->msix_entry);
	kfree(dev);

	dev_printk(KERN_ERR, &pci_dev->dev, "Failed probe..... %d\n",
								result);
	return (result);
}


/**
 * @brief Linux driver entry point for NVME controller detach.
 *  All previously allocated IO queues and DMA resources
 *  are released. Disk devices are removed and controller
 *  reset and PCI resources released.
 *
 * @param[in] struct pci_dev *pci_dev, pointer to PCi driver allocated PCI dev.
 *
 * @return none.
 *
 */
/* @todo  static void __devexit nvme_remove(struct pci_dev *pci_dev) */
static void
nvme_remove(struct pci_dev *pci_dev)
{
	struct nvme_dev *dev = pci_get_drvdata(pci_dev);

	DPRINT1("Executing  dev %p, %s ......\n", dev, nvme_pci_id(pci_dev));

#ifdef NVME_DEBUG
{
	int i;
	struct queue_info *qinfo;

	for( i = 1; i <= dev->que_count  ; i++ ) {
		qinfo = dev->que_list[i];
		DPRINT("****** Flushing qinfo %p, nr_req %d:\n",
						qinfo, qinfo->nr_req);
		if (qinfo->nr_req) {
			nvme_flush_queue(qinfo, NULL, -EIO);
		}
	}
}
#endif

	if (dev->state == NVME_STATE_OPERATIONAL)
		dev->state = NVME_STATE_SUSPEND;

	/**
	 * Release GenDisk associated with all Namespaces.
	 */
	nvme_free_disks(dev);

	rms200_free_char_dev(dev);

	DPRINT1("control device %p \n", dev->ctrl_dev);
	if (dev->ctrl_dev) {
		if (dev->proc_dir) {
			if (dev->proc_entry_stats) {
				remove_proc_entry("stats", dev->proc_dir);
			}
			if (dev->proc_entry_qinfo) {
				remove_proc_entry("qinfo", dev->proc_dir);
			}
		}
		remove_proc_entry(dev_name(dev->ctrl_dev),NULL);

		device_destroy(nvme_class, MKDEV(nvme_cmajor, dev->instance));
	}
	if (dev->nvme_thread) {
		dev->thread_flags |= THREAD_STOP;
		wake_up(&dev->thread_wait);
		schedule_timeout(HZ);
	}
	del_timer_sync(&dev->timer);

	/**
	 * Stop kernel thread.
	 */
	if (dev->nvme_thread) {
		int count = 5;
		kthread_stop(dev->nvme_thread);
		while(count--) {
			schedule_timeout(HZ);
			if(!dev->nvme_thread)
				break;
		}
		DPRINT("Wait count %d for nvme_thread\n",count);
	}

	/**
	 * Release hardware resources.
	 */
	nvme_delete_io_queues(dev, FALSE);
		nvme_hw_stop(dev, TRUE);
	nvme_release_irq(dev, dev->admin_queue);
	if (nvme_msix_enable)
		nvme_disable_msix(dev);

	/**
	 * Release kernel resources.
	 */
	DPRINT1("releasing Admin queue  %p ......\n", dev->admin_queue);
	queue_dest(dev, dev->admin_queue);
	nvme_put_instance(dev);

	/**
	 * Release PCI resources.
	 */
	DPRINT1("unmapping  reg %p\n", dev->reg);
	iounmap(dev->reg);
	DPRINT1("unmapping  DMI %p\n", dev->ddr_remap);
	if (dev->ddr_remap)
		iounmap(dev->ddr_remap);
	pci_disable_device(pci_dev);
	pci_release_regions(pci_dev);

#ifdef NVME_DEBUG
	DPRINT1("Releasing context 0x%p", dev);
#endif

	/**
	 * Release device.
	 */
	if (dev->identify)
		kfree(dev->identify);
	kfree(dev->msix_entry);
	kfree(dev);
}


/**
 * @brief Linux driver module init entry.
 *  This entry called by kernel module loader to initialize module.
 *
 * @param[in] none.
 *
 * @return returns int 0 if successful, otherwise Error Code
 */
static int __init
nvme_init(void)
{
	int result = -EBUSY;

	printk(KERN_INFO DRV_NAME" drive module %s\n",
						NVME_REL);
	/**
	 * Register Character device for Control node.
	 */
		result = register_chrdev(nvme_cmajor, DRV_NAME, &nvme_chrdev_fops);
		if (result <= 0) {
			printk(KERN_ERR "%s: Failed to register NVME control device %d\n",
						DRV_NAME, result);
		return result;
	}
	nvme_cmajor = result;

	/**
	 * We are not yet assigned a major device ID, let system assign one.
	 */
	result = register_blkdev( nvme_major, DRV_NAME );
	if (result <= 0) {
		printk(KERN_ERR "%s: Failed block register %d\n", DRV_NAME, result);
			unregister_chrdev(nvme_cmajor, DRV_NAME);
		return result;
	}
	nvme_major = result;
	printk(KERN_INFO DRV_NAME" Major %d\n", nvme_major);

	nvme_class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(nvme_class)) {
		printk(KERN_ERR "Failed to create NVME class \"%s\"\n", DRV_NAME);
		unregister_chrdev(nvme_cmajor, DRV_NAME);
		unregister_blkdev(nvme_major, DRV_NAME);
		return(PTR_ERR(nvme_class));
	}

	nvme_class->dev_groups = nvme_sysfs_default_groups;

	result = pci_register_driver(&nvme_driver);
	if (result) {
		printk(KERN_ERR "%s: Failed pci register %d\n", DRV_NAME, result);
			unregister_chrdev(nvme_cmajor, DRV_NAME);
		unregister_blkdev(nvme_major, DRV_NAME);
		class_destroy(nvme_class);
	}
	rms200_setup_genetlink();

   	return result;
}

/**
 * @brief Linux driver module exist entry.
 *  This entry is called by kernel prior to releasing module from kernel.
 *
 * @param[in] none.
 *
 * @return none.
 */
static void __exit
nvme_exit(void)
{
	unregister_chrdev(nvme_cmajor, DRV_NAME);
	unregister_blkdev(nvme_major, DRV_NAME);
	pci_unregister_driver(&nvme_driver);
	if (nvme_class)
		class_destroy(nvme_class);
	rms200_teardown_genetlink();
}

MODULE_AUTHOR("support@idt.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(NVME_REL);
module_init(nvme_init);
module_exit(nvme_exit);
