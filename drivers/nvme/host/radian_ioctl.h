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

#ifndef __RADIAN_NVME_IOCTL_HEADER__
#define __RADIAN_NVME_IOCTL_HEADER__

#include <linux/nvme_ioctl.h>
#include "nvme.h"
#include "radian_nvme_express.h"

#define RADIAN_NVME_VENDOR 0x1cc7
#define RADIAN_NVME_DEV_ID 0x0200

/*
 * Radian ioctl function command definitions - different than the UAPI
 * ioctl definitions for native NVMe devices
 */
#define RADIAN_RMS_ASYNC_LBA 0
#define RADIAN_RMS_ASYNC_SIZE 8192
#define RADIAN_RMS_ASYNC_DATA_OFFSET 4096
#define RADIAN_RMS_ASYNC_SIGNATURE "craig-hack"
#define RADIAN_RMS_ASYNC_SIGNATURE_SIZE 32
#define RADIAN_IOCTL_MAJOR_FUNCTION		0x0001
#define RADIAN_IOCTL_MINOR_NUMBER		0x00
#define RADIAN_IOCTL_FUNCTION			0x00

#define RADIAN_XFER_TO_DEV   			0
#define RADIAN_XFER_FROM_DEV 			1

#define RADIAN_NVME_GET_VERSION_NUM        _IOR('N', 0x40, int)
#define RADIAN_NVME_IOCTL_ADMIN_CMD        _IOWR('N', 0x41, struct radian_usr_io)
//#define RADIAN_NVME_IOCTL_IO_CMD           _IOWR('N', 0x42, struct radian_usr_io)
//#define RADIAN_NVME_IOCTL_ADMIN_CMD        _IOWR('N', 0x41, struct nvme_admin_cmd)
#define RADIAN_NVME_IOCTL_IO_CMD           _IOWR('N', 0x42, struct radian_usr_io)
#define RADIAN_NVME_IOCTL_RESTART          _IO('N', 0x43)
#define RADIAN_NVME_IOCTL_HOTREMOVE        _IO('N', 0x44)
#define RADIAN_NVME_IOCTL_HOTADD           _IO('N', 0x45)
#define RADIAN_NVME_IOCTL_EVENT            _IO('N', 0x46)
#define RADIAN_NVME_IOCTL_SET_CACHE        _IO('N', 0x47)

#define RADIAN_NVME_RMS_IOCTL_IO_CMD_ASYNC _IOWR('N', 0x48, struct radian_usr_io2)

/**
 * User Pass through data structure definition.
 */
struct radian_usr_io {
	struct nvme_cmd cmd;	        /* Submission queue entry. */
	struct cq_entry comp;		/* Completion entry */
#ifdef __cplusplus
	__u8    ns;
#else
	__u8	namespace;		/* Namespace ID, -1 for non-specific */
#endif
	__u8	direction;		/* direction TO_DEVICE/FROM_DEVICE */
	__u16	timeout;		/* timeout in seconds */
	__u32	status;			/* Command status */
	__u32	length;			/* data length */
	__u32	meta_length;		/* meta data length */
    __u64   addr;			/* data address */
    __u64   meta_addr;		/* meta data address */
};

/*
 * RMS extension for async io cmd
 */
struct radian_usr_io2 {
	struct radian_usr_io uio;
	__u64  async_opaque;
	__u32  eventfd;
};


/**
 * Event notification request data structure definition.
 */
struct radian_event_req {
	__u16	event_id;		/* Event Identification */
	__u16	event_mask;		/* Event Identification mask */
	__u32	length;			/* Event Page data length */
	__u64   addr;			/* Event Page data address */
};

long radian_dev_ioctl(struct nvme_ctrl *ctrl,
		      unsigned int cmd,
		      unsigned long arg);
#endif
