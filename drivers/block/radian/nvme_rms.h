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

/* Radian's Vendor-specific NVMe extensions */

#ifndef NVME_RMS_H_
#define NVME_RMS_H_

/* AER_VENDOR_SPECIFIC log page identifiers */
#define	GLP_ID_DIALOG		           0xD1 // 4096 bytes
#define GLP_ID_SPARE                   0xE0 /* Defect spare Log Page ID   */
#define GLP_ID_COLD                    0xE1 /* Cold data Log Page ID      */
#define GLP_ID_RELEASED                0xE2 /* Released pages Log Page ID */
#define GLP_ID_AEN_TRIGGER             0xF1 /* RMS-Trigger-specific       */

#define GLP_ID_AEN_TRIGGER_LOGPGSIZ    680
			/* little larger than RMSVendorSpecific.h: nvme_aer_ind_rms250_t -
			 * but certainly less than PAGE_SIZE for efficiency
			 * TODO we really should have rmsVendorSpecific reflected accurately
			 * in this driver since afterall it is RMS-specific.
			 */


/*
 * This is defined in NVMeVendorSpecific.h in the firmware
 * as NVM_VENDOR_RMS200
*/
#define NVME_VNDR_CMD_RMS200 		0xD8


/* Vendor-specific PCIe Capabilities */

#ifndef PCI_EXT_CAP_ID_VNDR
/* for older kernels that don't have this in pci_regs.h */
#define PCI_EXT_CAP_ID_VNDR	0x0B	/* Vendor Specific */
#endif

/* RMS Device ID Register */
#define RMS_EXT_VID			0x08
#define RMS_VENDOR_ID			0x5010

/* RMS Boot Configuration Vector Status Register */
#define RMS_EXT_BOOT			0x0C
#define RMS_EXT_BOOT_CAPCHARGING	0x80000000

/* RMS DDR base address of DMI window (units are pages? i.e. 4096?) */
#define RMS_DMIWINBASE			0x14

/**
  * These are the netlink names of the family and group
  * a user should register with in order to receiove generic
  * netlink messages from the kernel's nvme driver
  */
#define	NVME_EVENT_NAME					"nvme_event"
#define NVME_EVENT_GROUP_NAME			"nvme_mc_group"

/* These are the netlink message types for this protocol */
enum {
	DIALOG_CMD_UNSPEC = 0,
	DIALOG_CMD_GET_PAGE,
	_DIALOG_CMD_MAX,
};

#define DIALOG_CMD_MAX (_DIALOG_CMD_MAX - 1)

struct dialog_page_msg {
	u32 page_id;
	u32 page_len;
	u8	log_page[4000];
};

/*
 * Our group identifiers
 */
#define NVME_GRP_ALERT 1

#define	DIALOG_ATTR_MAX  1
#define	DIALOG_ATTR_PAGE  0

/*
 * Accessing in running FW mode
 */
#define PCS_INDICARADDR 0x1f8
#define PCS_INDICARDATA 0x1fc

/**
 * @brief Subcommand opcode for System Debug
 */
enum
{
    /**
     * @brief Critical Warning
     */
    SYS_DEBUG_CRITICAL_WARNING = 0x01,
    /**
     * @brief Media error
     */
    SYS_DEBUG_MEDIA_ERROR = 0x02,
    /**
     * @brief Error log update
     */
    SYS_DEBUG_ERROR_LOG_UPDATE = 0x03,
    /**
     * @bref Debug generic
     */
    SYS_DEBUG_DEBUG_GENERIC = 0x04,
    /**
     * @brief Global Error Interrupt
     */
    SYS_DEBUG_GLOBAL_ERR_INTR = 0x05,
    /**
     * @brief Tickle Asynchronous Event Notification
     */
    SYS_DEBUG_AEN_CTRL = 0x06,
    /**
	 * @brief Control various diallog functions
     */
    SYS_DEBUG_DIALOG_CTRL = 0x07,
    /**
     * @brief End of system debug opcode
     */
    SYSTEM_DEBUG_END,
};

struct priv_char_dev {
	struct list_head	node;
	struct miscdevice	miscdev;
	struct nvme_dev		*dev;
	u64			dmi_window_base;	/* in memory lines */
	u64			dmi_window_size;	/* in memory lines */
	u64			ddrsize;		/* in bytes */
	char			name[32];
};

extern int forcerdy;

int nvme_rms_wait_charge(struct nvme_dev *dev, int ndelay);
void nvme_rms_info(struct nvme_dev *dev);
int nvme_rms_send_timestamp(struct nvme_dev *dev, struct nvme_cmd *entry);
int rms200_prepare_and_send_page(u32 page_id, u32 page_size, void *log_page);
int rms200_setup_genetlink(void);
void rms200_teardown_genetlink(void);
void rms200_allowed_admin_cmd(struct nvme_dev *dev, struct usr_io *uio);
int rms200_alloc_char_dev(struct nvme_dev *dev);
void rms200_free_char_dev(struct nvme_dev *dev);
void rms200_map_bar4(struct nvme_dev *dev);
void rms_check_ns_count(struct nvme_dev *dev);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))
int rms200_proc_dump_nvme_stats(char *buf, char **start, off_t offset,
	int count, int *eof, void *data);
#else
extern const struct file_operations rms200_stats_proc_fops;
extern const struct file_operations rms_proc_qinfo_fops;
#endif

#endif /* NVME_RMS_H_ */
