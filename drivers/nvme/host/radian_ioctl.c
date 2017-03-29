/*****************************************************************************
 *
 *  Copyright (C) 2017  NetApp / SolidFire Inc.
 *  
 *  Derived from Copyright (C) 2009-2013  Integrated Device Technology, Inc.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/nvme.h>
#include <linux/nvme_ioctl.h>
#include "radian_ioctl.h"

static int radian_nvme_ioctl_vernum = ((RADIAN_IOCTL_MAJOR_FUNCTION << 16) |
				       (RADIAN_IOCTL_MINOR_NUMBER << 8) |
				        RADIAN_IOCTL_FUNCTION);

void inline radian_nvme_memcpy_64(void *dst, void *src, int cnt)
{
	u64 *p1 = dst, *p2 = src;
	while(cnt--) {
		*p1++ = *p2++;
	}
}

long radian_dev_ioctl(struct nvme_ctrl *ctrl,
		      unsigned int cmd,
		      unsigned long arg)
{
	struct nvme_dev *dev = to_nvme_dev(ctrl);
	void __user *p = (void __user *)arg;
	int __user *ip = p;

	printk(KERN_INFO "got admin command: 0x%08x, 0x%08x\n", cmd, RADIAN_NVME_IOCTL_ADMIN_CMD);
	switch (cmd) {
	case RADIAN_NVME_GET_VERSION_NUM:
		return put_user(radian_nvme_ioctl_vernum, ip);
	case RADIAN_NVME_IOCTL_ADMIN_CMD:
		printk(KERN_INFO "got admin command: 0x%08x\n", cmd);
		//return nvme_admin_passthru(dev, p);
		return nvme_user_cmd(ctrl, NULL, p);
	default:
		printk(KERN_INFO "unknown cmd\n");
		break;
	}

	return -ENOTTY;
}
