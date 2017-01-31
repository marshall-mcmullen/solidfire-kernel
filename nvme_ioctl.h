/*****************************************************************************
 *
 *  Copyright (C) 2009-2013  Integrated Device Technology, Inc.
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
 * $RCSfile: nvme_ioctl.h,v $
 * $Revision: 1.7 $
 * $Date: 2013/04/04 17:09:04 $
 * $Author: kpanah $
 * $Release: $ Name:  $
 *
 *  You can contact Integrated Device Technology, Inc via email ssdhelp@idt.com
 *  or mail Integrated Device Technology, Inc
 *  6024 Silver Creek Valley Road, San Jose, CA 95128, USA
 *
 *
 *****************************************************************************/

/**
 * @file nvme_ioctl.h - IOCTL header file IDT NVM Express Block Device Driver
 * 		This file defines all data structures and constants shared by
 *		NVME Linux driver module and user application.
 */
#ifndef NVME_IOCTL_HEADER_
#define NVME_IOCTL_HEADER_

#define RMS_ASYNC_LBA 0
#define RMS_ASYNC_SIZE 8192
#define RMS_ASYNC_DATA_OFFSET 4096
#define RMS_ASYNC_SIGNATURE "craig-hack"
#define RMS_ASYNC_SIGNATURE_SIZE 32


/**
 * IOCTL Version number, identifies IOCTL interface and supported
 * ioctl functions.
 *
 * IOCTL_MAJOR_FUNCTION		identifies driver major version function
 * IOCTL_MINOR_NUMBER		Identifies driver minor version function
 * IOCTL_FUNCTION		Identifies driver function version function
 *
 * user application should query driver IOCTL version number to identify
 * ioctl interface level and supported functions.
 */
#define IOCTL_MAJOR_FUNCTION		0x0001
#define IOCTL_MINOR_NUMBER		0x00
#define IOCTL_FUNCTION			0x00

#define XFER_TO_DEV   			0
#define XFER_FROM_DEV 			1

/**
 * IOCTL Function command definitions.
 */
#define NVME_GET_VERSION_NUM        _IOR('N', 0x40, int)
#define NVME_IOCTL_ADMIN_CMD        _IOWR('N', 0x41, struct usr_io)
#define NVME_IOCTL_IO_CMD           _IOWR('N', 0x42, struct usr_io)
#define NVME_IOCTL_RESTART          _IO('N', 0x43)
#define NVME_IOCTL_HOTREMOVE        _IO('N', 0x44)
#define NVME_IOCTL_HOTADD           _IO('N', 0x45)
#define NVME_IOCTL_EVENT            _IO('N', 0x46)
#define NVME_IOCTL_SET_CACHE        _IO('N', 0x47)

#define NVME_RMS_IOCTL_IO_CMD_ASYNC _IOWR('N', 0x48, struct usr_io2)

/**
 * User Pass through data structure definition.
 */
struct usr_io {
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
struct usr_io2 {
	struct usr_io uio;
	__u64  async_opaque;
	__u32  eventfd;
};


/**
 * Event notification request data structure definition.
 */
struct event_req {
	__u16	event_id;		/* Event Identification */
	__u16	event_mask;		/* Event Identification mask */
	__u32	length;			/* Event Page data length */
	__u64   addr;			/* Event Page data address */
};

#endif
