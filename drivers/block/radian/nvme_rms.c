/*****************************************************************************
 *
 * Copyright (C) 2013 Radian Memory Systems
 *
 * This program is free software: When used with a Linux driver,
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * When used with any other driver, it may be used (at your option)
 * with either the GPL license or with the Free BSD License below.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 *****************************************************************************/

/**
 * @file nvme_rms.c - Linux driver routines for RMS NVMe COntrollers.
 */


#include <linux/bio.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/ptrace.h>
#include <net/genetlink.h>
#include <linux/delay.h>
#include <asm/page.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "nvme_express.h"
#include "nvme.h"
#include "nvme_ioctl.h"
#include "nvme_rms.h"

#ifdef NVME_DEBUG
extern int	nvme_dbg;
#endif

static DEFINE_MUTEX(char_io_mutex);

static LIST_HEAD(rms200_cdev_list);


static u32
nvme_rms_cfg(struct nvme_dev *dev)
{

	if (dev->rms_off > 0
	 && pci_read_config_dword(dev->pci_dev, dev->rms_off + RMS_EXT_BOOT,
					&dev->boot_cfg) == PCIBIOS_SUCCESSFUL) {
		return dev->boot_cfg;
	} else {
		return 0;
	}
}

int
nvme_rms_wait_charge(struct nvme_dev *dev, int ndelay)
{
	int result;

	/*
	 * First check the cached value of the bit for efficiency.
	 * Only if it was set, we read from the device.  Once clear,
	 * we know it will stay clear.  For non-RMS devices, it is
	 * always clear.
	 */
	if (dev->boot_cfg & RMS_EXT_BOOT_CAPCHARGING
	 && !forcerdy
	 && nvme_rms_cfg(dev) & RMS_EXT_BOOT_CAPCHARGING) {
		dev_printk(KERN_INFO, &dev->pci_dev->dev,
						"waiting for cap charge\n");
		if (ndelay)
			return -EWOULDBLOCK;
		nvme_wait_cond(dev, 120,
				!(nvme_rms_cfg(dev) & RMS_EXT_BOOT_CAPCHARGING),
				result);
		if (result) {
			EPRINT("Capacitor charge failure %d\n", result);
			return result;
		}
	}

	return 0;
}

/**
 * @brief This function finds the RMS vendor-specific capability
 *		If found, its location is recorded in dev->rms_off.
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return void
 */
void
nvme_rms_info(struct nvme_dev *dev)
{
	u16 vendor_id;
	int pos = 0;

	while ((pos = pci_find_next_ext_capability(dev->pci_dev, pos,
						PCI_EXT_CAP_ID_VNDR)) > 0) {
		if (pci_read_config_word(dev->pci_dev, pos + RMS_EXT_VID,
					&vendor_id) == PCIBIOS_SUCCESSFUL
		 && vendor_id == RMS_VENDOR_ID) {
			dev->rms_off = pos;
			dev_printk(KERN_INFO, &dev->pci_dev->dev,
					"rms_off %x\n", dev->rms_off);
			break;
		}
	}

	/* cache the boot_cfg value for nvme_open() */
	(void) nvme_rms_cfg(dev);
}

/**
 * @brief This function uses Radian vendor specific
 * debug opcode in order to set the timestamp in dialog
 *
 *
 * @param[in] struct nvme_dev *dev pointer to nvme device context
 *
 * @return This function returns int 0 if successful, otherwise Error Code
 */
int
nvme_rms_send_timestamp(
	struct nvme_dev *dev, 
	struct nvme_cmd *entry)
{
	struct	timeval  now;

	if (dev->rms_off <= 0)
		return 0;

	do_gettimeofday(&now);

	memset(entry, 0, sizeof(*entry));
	entry->header.opCode	= NVME_VNDR_CMD_SYS_DEBUG;
	entry->cmd.vendorSpecific.vndrCDW12 = (u32) SYS_DEBUG_DIALOG_CTRL;
	entry->cmd.vendorSpecific.vndrCDW13 = (u32) 0; /* SET_TIME */
	entry->cmd.vendorSpecific.vndrCDW14 = (u32) now.tv_sec;
	return 1;
}

static struct genl_multicast_group nvme_event_mcgrp = {
	.name = NVME_EVENT_GROUP_NAME,
};

static struct genl_family nvme_event_family = {
	.id             = GENL_ID_GENERATE,
	.hdrsize        = 0,
	.name           = NVME_EVENT_NAME,
	.version        = 1,
	.maxattr        = DIALOG_CMD_MAX,
	.mcgrps			= &nvme_event_mcgrp,
	.n_mcgrps = 1,
};

static int genetlink_established;

int
rms200_prepare_and_send_page(u32 page_id, u32 page_size, void *log_page)
{
	struct sk_buff *skb;
	struct nlattr *attr;
	struct dialog_page_msg *msg;
	void *msg_header;
	int size;
	int result;

	pr_notice("%s(0x%02x, sz:%d)\n", __FUNCTION__, page_id, page_size);

	/* allocate memory */
	size = nla_total_size(sizeof(struct dialog_page_msg))
			+ nla_total_size(0);

	skb = genlmsg_new(size, GFP_ATOMIC);
	if (!skb) {
		pr_debug("%s:%d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	/* add the genetlink message header */
	msg_header = genlmsg_put(skb, 0, 0, &nvme_event_family, 0,
				DIALOG_CMD_GET_PAGE);
	if (!msg_header) {
		nlmsg_free(skb);
		pr_debug("%s:%d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	/* fill the data */
	attr = nla_reserve(skb, NLA_STRING, sizeof(struct dialog_page_msg));
	if (!attr) {
		nlmsg_free(skb);
		pr_debug("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	msg = nla_data(attr);
	if (!msg) {
		nlmsg_free(skb);
		pr_debug("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	attr->nla_type = NLA_STRING;
	attr->nla_len = sizeof(struct dialog_page_msg) + NLA_HDRLEN;
	memset(msg, 0, sizeof(struct dialog_page_msg));

	msg->page_id = page_id;
	msg->page_len = page_size;
	memcpy(msg->log_page, log_page, page_size);

	/* send multicast genetlink message */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0))
	result = genlmsg_end(skb, msg_header);
	if (result < 0) {
		nlmsg_free(skb);
		pr_debug("%s:%d\n", __func__, __LINE__);
		return result;
	}
#else
	genlmsg_end(skb, msg_header);
#endif

	result = genlmsg_multicast(&nvme_event_family, skb, 0, 0, GFP_ATOMIC);

	pr_debug("%s:%d ret = %d\n", __func__, __LINE__, result);

	return 0;
}

int
rms200_setup_genetlink(void)
{
	pr_notice("Initalizing genetlink service\n");

	if (genl_register_family(&nvme_event_family) < 0) {
		pr_notice("Could not create nvme netlink family\n");
		return -EFAULT;
	}
#if 0
	ret = genl_register_mc_group(&nvme_event_family, &nvme_event_mcgrp);
	if (ret) {
		pr_notice("Failed to register nvme netlink mc group\n");
		genl_unregister_family(&nvme_event_family);
		return -EFAULT;
	}
#endif

	genetlink_established = 1;
	return 0;
}

void
rms200_teardown_genetlink(void)
{
	genl_unregister_family(&nvme_event_family);
}

static u32 move_dmi_window(struct nvme_dev *dev, u32 memory_line)
{
	u32 val = 0xbeef;
	pci_write_config_dword(dev->pci_dev, dev->rms_off + RMS_DMIWINBASE, memory_line);
	pci_read_config_dword(dev->pci_dev, dev->rms_off + RMS_DMIWINBASE, &val);
//  printk("move dmi window to 0x%x\n", memory_line);
	return (val);
}

u32 check_local_pos(u32 local_pos, u32 global_pos, struct priv_char_dev *cdev, struct nvme_dev *dev)
{
	u32 memory_line;
	u32 val;
	if (local_pos == cdev->dmi_window_size * 64) {
		local_pos = 0;
		memory_line = global_pos / 64;
		val = move_dmi_window(dev, memory_line);
		if (val == memory_line) 
			cdev->dmi_window_base = memory_line;
		else
			return (u32) -1;
		}

	return (local_pos);
}

static ssize_t
rms200_char_io(struct file *file, char __user *buf, size_t size, loff_t *ppos, int is_write)
{
	unsigned int cnt = size;
	unsigned int i;
	unsigned int j;
	int ret = 0;
	struct priv_char_dev *cdev;
	struct nvme_dev *dev;
	u32 memory_line;
	volatile u32 val;
	u64 global_pos = *ppos; /* byte position in the DDR */
	u32 local_pos;		/* byte position in the current DMI window */
	u32 nbytes;
	u32 memlines;
	u64 total_out;
	int no_space = 0;
	
	union _outbuf {
		unsigned char chars[64];
		u32 dwords[16];
		u64 qwords[8];
	} outbuf;


	cdev = (struct priv_char_dev *) file->private_data;
	dev = cdev->dev;

	/* Arg check */

	/* check range */
	if (size == 0 || global_pos >= cdev->ddrsize) {
		if (size > 0 && is_write)
			return (-ENOSPC);
		else
			return(0);
	}

	/*
	 * PCIe memory read and write requests must start on a DWord aligned
	 * PCIe address. 
	 */
	if ((size & 3) != 0 || (global_pos & 3) != 0) {
		EPRINT("bad alignment\n");
		return(-EIO);
	}

	/* Truncate write at end of memory */
	if (global_pos + cnt > cdev->ddrsize)
	{
		cnt = cdev->ddrsize - global_pos;
		if (is_write)
			no_space = 1;
	}

    /* Get exclusive access to the DMI window */
	mutex_lock(&char_io_mutex);

	/* Check to see if file position pointer is within the current DMI window */
	memory_line = global_pos >> 6;
	if (memory_line < cdev->dmi_window_base || 
		memory_line >= cdev->dmi_window_base + cdev->dmi_window_size) {
		val = move_dmi_window(dev, memory_line);
		if (val == memory_line)
			cdev->dmi_window_base = memory_line;
		else {
			ret = -EBUSY;
			goto write_err;
		}
	}

	local_pos = global_pos - cdev->dmi_window_base * 64ULL;

	/* Write out or read in dwords up to the next memory line */

	nbytes = 64 - (local_pos & 0x3f);
        
	if (nbytes < 64) {
		if (nbytes > cnt)
			nbytes = cnt;
		if (is_write) {
			if (copy_from_user(outbuf.chars, buf, nbytes)) {
				ret = -EFAULT;
				goto write_err;
			}
			for (i = 0; i < nbytes / 4; i++)
				writel(outbuf.dwords[i], dev->ddr_remap + local_pos + i*4);
		} else {
			for (i = 0; i < nbytes / 4; i++)
				outbuf.dwords[i] = readl(dev->ddr_remap + local_pos + i*4);
			if (copy_to_user(buf, outbuf.chars, nbytes)) {
				ret = -EFAULT;
				goto write_err;
			}
		}
		buf += nbytes;
		local_pos += nbytes;
		global_pos += nbytes;
		cnt -= nbytes;
	}
	else
		nbytes = 0;

	*ppos += nbytes;
	total_out = nbytes;

	if (cnt == 0) {
		/* update ppos and return */
		ret = total_out;
		goto write_err;
	}

	/* Handle complete memory lines, a qword at a time */
	local_pos = check_local_pos(local_pos, global_pos, cdev, dev);
	if (local_pos == (u32) -1) {
		ret = -EBUSY;
		goto write_err;
	}
	memlines = cnt / 64;
	for (j = 0; j < memlines; j++) {
		if (is_write) {
			if (copy_from_user(outbuf.chars, buf, 64)) {
				ret = -EFAULT;
				goto write_err;
			}
			for (i = 0; i < 8; i++)
				writeq(outbuf.qwords[i], dev->ddr_remap + local_pos + i*8);
		} else {
			for (i = 0; i < 8; i++)
				outbuf.qwords[i] = readq(dev->ddr_remap + local_pos + i*8);
			if (copy_to_user(buf, outbuf.chars, 64)) {
				ret = -EFAULT;
				goto write_err;
			}
		}
		buf += 64;
		local_pos += 64;
		global_pos += 64;
		*ppos = global_pos;
		local_pos = check_local_pos(local_pos, global_pos, cdev, dev);
		if (local_pos == (u32) -1) {
			ret = -EBUSY;
			goto write_err;
		}
	}

	total_out += memlines * 64;
	cnt -= memlines * 64;

	if (cnt == 0)  {
		ret = total_out;
		goto write_err;
	}

    /* Take care of the last partial memory line */

	nbytes = cnt;
	if (is_write) {
		if (copy_from_user(outbuf.chars, buf, nbytes)) {
			ret = -EFAULT;
			goto write_err;
		}
		for (i = 0; i < nbytes / 4; i++)
			writel(outbuf.dwords[i], dev->ddr_remap + local_pos + i*4);
	} else {
		for (i = 0; i < nbytes / 4; i++)
			outbuf.dwords[i] = readl(dev->ddr_remap + local_pos + i*4);
		if (copy_to_user(buf, outbuf.chars, nbytes)) {
			ret = -EFAULT;
			goto write_err;
		}
	}

	total_out += nbytes;
	*ppos += nbytes;
	ret = total_out;
	if (is_write) {
		/* Issue a dummy read to flush posted PCI writes */
		outbuf.dwords[0] = readl(dev->ddr_remap);
		if (no_space)
			ret = -ENOSPC;
	}

write_err:

 	mutex_unlock(&char_io_mutex);
	return ret;
}

static ssize_t
rms200_write(struct file *file,
		const char __user *buf,
		size_t size,
		loff_t *ppos)
{
        return (rms200_char_io(file, (char __user *) buf, size, ppos, 1));
}

static ssize_t
rms200_read(struct file *file,
		char __user *buf,
		size_t size,
		loff_t *ppos)
{
        return (rms200_char_io(file, buf, size, ppos, 0));
}

static int
rms200_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	struct priv_char_dev *cdev, *tmp;
	struct nvme_dev *dev;
	int result;

	list_for_each_entry_safe(cdev, tmp, &rms200_cdev_list, node) {
		if (minor == cdev->miscdev.minor)
			break;
	}
	if (!cdev)
		return -ENXIO;

	if (file->f_mode & FMODE_WRITE) {
		dev = cdev->dev;
		result = nvme_rms_wait_charge(dev, file->f_flags & O_NDELAY);
		if (result)
			return result;
	}

	file->private_data = cdev;

	return 0;
}

/*
 * The memory devices use the full 32/64 bits of the offset, and so we cannot
 * check against negative addresses: they are ok. The return value is weird,
 * though, in that case (0).
 *
 */
static loff_t
rms200_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;
	struct priv_char_dev *cdev;
	struct nvme_dev *dev;

	cdev = (struct priv_char_dev *) file->private_data;
	dev = cdev->dev;

//	mutex_lock(&file->f_path.dentry->d_inode->i_mutex);
	switch (orig) {
	case SEEK_CUR:
		offset += file->f_pos;
	case SEEK_SET:
		/* to avoid userland mistaking f_pos=-9 as -EBADF=-9 */
		if ((unsigned long long)offset > cdev->ddrsize) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos = offset;
		ret = file->f_pos;
		force_successful_syscall_return();
		break;
	case SEEK_END:
		if (offset <= 0 && (abs(offset) < cdev->ddrsize)) {
			file->f_pos = cdev->ddrsize - offset;
			ret = file->f_pos;
			force_successful_syscall_return();
			break;
		} else {
			ret = -EINVAL;
		}
	default:
		ret = -EINVAL;
	}
//	mutex_unlock(&file->f_path.dentry->d_inode->i_mutex);
	return ret;
}

static int
rms200_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	struct priv_char_dev *cdev;
	struct nvme_dev *dev;
	int ret;
	unsigned int pgoff = vma->vm_pgoff;
	u32 val;

	cdev = (struct priv_char_dev *) file->private_data;
	dev = cdev->dev;

	vma->vm_pgoff = 0;
#ifdef pgprot_noncached
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif

	if (size > dev->ddr_len) {
		EPRINT("Size > DMI window length\n");
		return -EINVAL;
	}

	ret = io_remap_pfn_range(vma, vma->vm_start,
		dev->ddr_phys >> PAGE_SHIFT, size, vma->vm_page_prot);

	if (ret) {
		EPRINT("ret = %d\n", ret);
		return ret;
	}

	/* vm_pgoff is page number.  Convert to memory line number */
	pci_write_config_dword(dev->pci_dev, dev->rms_off + RMS_DMIWINBASE,
		pgoff * (PAGE_SIZE / 64));
	/* read to push the write though PCIe, value read doesn't matter */
	pci_read_config_dword(dev->pci_dev, dev->rms_off + RMS_DMIWINBASE,
		&val);

	return 0;
}

void
rms200_map_bar4(struct nvme_dev *dev)
{
	struct pci_dev *pci_dev;

	pci_dev = dev->pci_dev;
	dev->ddr_phys = pci_resource_start(pci_dev, 4);
	dev->ddr_len  = pci_resource_len(pci_dev, 4);
	if (!dev->ddr_len)
		return;

	/*
	 * Limit to 2 GB for now as it seems ioremap_nocache second argument
	 * is only 32 bit wide
	 */
	if (dev->ddr_len >= 0x100000000ULL) {
		dev->ddr_len = 0x80000000UL;
		printk("%s:%d Changing ddr_len to %08x_%08x\n",
			__FUNCTION__,
			__LINE__,
			(u32)(dev->ddr_len >> 32),
			(u32)(dev->ddr_len & ~0L));
	}
#if 0
	printk("%s:%d start %08x_%08x end %08x_%08x \n",
		__FUNCTION__,
		__LINE__,
		(u32)(pci_dev->resource[4].start >> 32),
		(u32)(pci_dev->resource[4].start & ~0L),
		(u32)(pci_dev->resource[4].end >> 32),
		(u32)(pci_dev->resource[4].end & ~0L));
#endif

	dev->ddr_remap = ioremap_nocache(dev->ddr_phys, dev->ddr_len);
	if (!dev->ddr_remap)
		EPRINT("Unable to remap bar4 memory region\n");
}

static const struct file_operations rms200_fops = {
	.owner		= THIS_MODULE,
	.llseek		= rms200_lseek,
	.read		= rms200_read,
	.write		= rms200_write,
	.open		= rms200_open,
	.mmap		= rms200_mmap,
};

void
rms200_allowed_admin_cmd(struct nvme_dev *dev, struct usr_io *uio)
{
#if 0
	u32 dmi_base;
	struct priv_char_dev *cdev;

	if (dev->rms_off <= 0)
		return;

	/*
	 * The base of the DMI window can change on the fly without restarting
	 * the host so here sniff RMS_200 vendor specific command in order to
	 * detect such a change. If the window base is located within the reserved
	 * area, restrict the character device by setting reserved_mem_len.
	 */
	if ((uio->cmd.header.opCode	== NVME_VNDR_CMD_RMS200)
	 && (uio->cmd.cmd.vendorSpecific.vndrCDW12 == 4)) {	/* RMS_SET_DMI_WINBASE = 4 */
		/*
		 * Get the first and only character device
		 */
		cdev = list_first_entry(&rms200_cdev_list, struct priv_char_dev, node);

		/*
		 * The base is expressed in units of 1 MB
		 */
		dmi_base = uio->cmd.cmd.vendorSpecific.vndrCDW13 << 20;

		/*
		 * Adjust the length of the resereved DRAM area used by the FW
		 * based on the location of the DMI window.
		 */
		if (dmi_base >= cdev->orig_reserved_mem_len)
			cdev->reserved_mem_len = 0;
		else
			cdev->reserved_mem_len = cdev->orig_reserved_mem_len - dmi_base;
		DPRINT9("new reserved_mem_len = %16llx\n", cdev->reserved_mem_len);
	}
#endif
}

/*
 * Figure out the size of the DMI based character device
*/
static u64 get_char_dev_size(struct nvme_dev *dev)
{
	struct ns_info *ns;
	u64 ddr_size = 0;

	switch (dev->pci_dev->device) {
	case 0x200:
		dev->lock_func(&dev->lock, &dev->lock_flags);
		list_for_each_entry(ns, &dev->ns_list, list) {
			if (ns->id == 1) {
				ddr_size = ns->block_count << ns->lba_shift;
				break;
			}
		}
		dev->unlock_func(&dev->lock, &dev->lock_flags);
		break;
	default:
		EPRINT("Unknown device %04x\n", dev->pci_dev->device);
		break;
	}
	return ddr_size;
}
/*
 * Add a character device to view the disk
 */
int
rms200_alloc_char_dev(struct nvme_dev *dev)
{
	struct priv_char_dev *cdev;
	int	result;
	u32 val;
	u64 ddr_size;

	if (dev->rms_off <= 0)
		return 0;

	if  (!(ddr_size = get_char_dev_size(dev))) {
		EPRINT("ddr_size is 0.\n");
		return 0;
	}

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev) {
		EPRINT("Failed NS memory allocation.\n");
		return -1;
	}
	cdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	sprintf(cdev->name, "%s%dc1", DEV_NAME, dev->instance);
	cdev->dev = dev;
	cdev->miscdev.name = cdev->name;
	cdev->miscdev.fops = &rms200_fops;

	pci_read_config_dword(dev->pci_dev, dev->rms_off + RMS_DMIWINBASE, &val);
	cdev->dmi_window_base = val;
	cdev->dmi_window_size = dev->ddr_len / 64;
	cdev->ddrsize = ddr_size;

	/*
	 * Get registered.
	 */
	result = misc_register(&cdev->miscdev);
	if (result < 0) {
		EPRINT("Unable to register\n");
		return -1;
	}
	NPRINT("Adding Misc device minor %d \"%s\" size %lld\n",
		cdev->miscdev.minor, cdev->name, dev->ddr_len);

	list_add_tail(&cdev->node, &rms200_cdev_list);
	return 0;
}

/*
 * Remove a character device
 */
void
rms200_free_char_dev(struct nvme_dev *dev)
{
	struct priv_char_dev *cdev, *tmp;

	if (dev->rms_off <= 0)
		return;

	list_for_each_entry_safe(cdev, tmp, &rms200_cdev_list, node) {
		if (cdev->dev != dev)
			continue;
		list_del(&cdev->node);
		misc_deregister(&cdev->miscdev);
		kfree(cdev);
	}
}

void
rms_check_ns_count(struct nvme_dev *dev)
{
	if (dev->ns_count > MAX_NAMESPACES) {
		dev_printk(KERN_WARNING, dev->dma_dev,
			"reducing from %d to %d Namespaces\n",
			dev->ns_count,
			MAX_NAMESPACES);
		dev->ns_count = MAX_NAMESPACES;
	}
}

static int
rms200_proc_dump_nvme_stats(struct seq_file *m, void *v)
{
	int i;
	struct	nvme_dev *priv = (struct nvme_dev *) m->private;

	if (priv) {
		seq_printf(m, " Interrupts completions = %lld\n", priv->dma_int);
		seq_printf(m, " Number of work queued = %lld\n", priv->work_queued);
		seq_printf(m, " User submitted requests = %lld\n", priv->user_req);
		seq_printf(m, " Device submitted requests = %lld\n", priv->sub_req);
		seq_printf(m, " Doorbell submitted requests = %lld\n", priv->doorbell_req);
		seq_printf(m, " Completed queue requests = %lld\n", priv->complete_q_cnt);
		seq_printf(m, " Bytes written to device = %lld, (0x%llx)\n", priv->bytes_written, priv->bytes_written);
		seq_printf(m, " Bytes read from device = %lld, (0x%llx)\n", priv->bytes_read, priv->bytes_read);
		for (i = 1; i <= priv->que_count; i++) {
			struct queue_info *qinfo = priv->que_list[i];
			int c = 0;
			struct cmd_info *ci;

			list_for_each_entry(ci, &qinfo->cmd_free, list) {
				c++;
			}
			seq_printf(m, " sub_que[%d] %lld act: %d bio %d free %d to %d\n", i,
				qinfo->sub_queue->sub_req, qinfo->nr_act, qinfo->nr_req, c,
				qinfo->nrf_timeout);
		}
	}

	return 0;
}

static int rms200_stats_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rms200_proc_dump_nvme_stats, PDE_DATA(inode));
}

const struct file_operations rms200_stats_proc_fops = {
	.open = rms200_stats_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int
rms_proc_qinfo_dump(struct seq_file *m, void *v)
{
	int i;
	struct	nvme_dev *priv = (struct nvme_dev *) m->private;

	// slmost the same as stats for now...
	if (!priv) {
		return 0;
	}

	for (i = 1; i <= priv->que_count; i++) {
		struct queue_info *qinfo = priv->que_list[i];
		int c = 0;
		struct cmd_info *ci;

		list_for_each_entry(ci, &qinfo->cmd_free, list) {
			c++;
		}
		seq_printf(m, " sub_que[%d] %lld act: %d bio %d free %d\n", i,
			qinfo->sub_queue->sub_req, qinfo->nr_act, qinfo->nr_req, c);

		{
			struct bio* bio;
			c = 0;
			bio_list_for_each(bio, &qinfo->sq_cong) {
				c++;
			}
			seq_printf(m, "  que congest list: nr %d\n", c);
			seq_printf(m, "  que nrf_sq_cong:     %d\n", qinfo->nrf_sq_cong);
			seq_printf(m, "  que nrf_fq:          %d\n", qinfo->nrf_fq);
			seq_printf(m, "  que nrf_timeout:     %d\n", qinfo->nrf_timeout);

			c = 0;
			list_for_each_entry(ci, &qinfo->cmd_active, list) {
				c++;
			}
			seq_printf(m, "  que cmd_active : nr %d nr_act %d\n", c, qinfo->nr_act);
		}
	}


	return 0;
}

static int rms_proc_qinfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, rms_proc_qinfo_dump, PDE_DATA(inode));
}

const struct file_operations rms_proc_qinfo_fops = {
	.open = rms_proc_qinfo_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
