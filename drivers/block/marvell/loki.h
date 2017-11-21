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


/*
 * NVRAM driver IOCTLS
 */

#ifndef __LOKI_IOCTL_H
#define __LOKI_IOCTL_H

#include "mv_types.h"
#include "mv_common.h"

#define NVRAM_IOC_MAGIC			101
#define NVRAM_IOC_RESET			_IO(NVRAM_IOC_MAGIC, 0)
#define NVRAM_IOC_DUMP_REG		_IOR(NVRAM_IOC_MAGIC, 1, int)
#define NVRAM_IOC_REG_IO		_IOWR(NVRAM_IOC_MAGIC, 2, reg_io_t)
#define NVRAM_IOC_GET_HBA_COUNT		_IOR(NVRAM_IOC_MAGIC, 3, int)
#define NVRAM_IOC_REMAP_FLASH		_IOWR(NVRAM_IOC_MAGIC, 4, int)
#define NVRAM_IOC_START_BACKUP		_IOWR(NVRAM_IOC_MAGIC, 5, int)
#define NVRAM_IOC_FORMAT_SSD		_IOWR(NVRAM_IOC_MAGIC, 6, int)
#define NVRAM_IOC_EN_FORCED_ECC		_IOWR(NVRAM_IOC_MAGIC, 7, int)
#define NVRAM_IOC_DIS_FORCED_ECC	_IOWR(NVRAM_IOC_MAGIC, 8, int)
#define NVRAM_IOC_START_RESTORE		_IOWR(NVRAM_IOC_MAGIC, 9, int)
#define NVRAM_IOC_READ_SCAP_REG		_IOWR(NVRAM_IOC_MAGIC, 10, reg_io_t)
#define NVRAM_IOC_AIO			_IOWR(NVRAM_IOC_MAGIC, 11, nv_aiocb_t)
#define NVRAM_IOC_AIO_CMPL_ID		_IOWR(NVRAM_IOC_MAGIC, 12, u64_t)
#define NVRAM_IOC_GET_SCAP_VOL		_IOWR(NVRAM_IOC_MAGIC, 13, scap_vol_t)
#define NVRAM_IOC_DMA_DUMP		_IOWR(NVRAM_IOC_MAGIC, 14, int)
#define NVRAM_IOC_PING_FW		_IOWR(NVRAM_IOC_MAGIC, 15, int)
#define NVRAM_IOC_FW_VERSION		_IOWR(NVRAM_IOC_MAGIC, 16, int)
#define NVRAM_IOC_GET_BACKUP_STATS	_IOWR(NVRAM_IOC_MAGIC, 17, \
						backup_stats_t)
#define NVRAM_IOC_CLEAR_BACKUP_STATS	_IOWR(NVRAM_IOC_MAGIC, 18, int)
#define NVRAM_IOC_SCAP_DISCHARGE_ON	_IOWR(NVRAM_IOC_MAGIC, 19, int)
#define NVRAM_IOC_SCAP_DISCHARGE_OFF	_IOWR(NVRAM_IOC_MAGIC, 20, int)
#define NVRAM_IOC_WRITE_READY		_IOWR(NVRAM_IOC_MAGIC, 21, int)
#define NVRAM_IOC_GET_EVNTLOG_PTRS	_IOWR(NVRAM_IOC_MAGIC, 22, event_log_t)
#define NVRAM_IOC_SET_EVNTLOG_RDPTR	_IOWR(NVRAM_IOC_MAGIC, 23, int)
#define NVRAM_IOC_IS_DATA_VALID		_IOWR(NVRAM_IOC_MAGIC, 24, int)
#define NVRAM_IOC_READ_VPD		_IOWR(NVRAM_IOC_MAGIC, 25, vpd_info_t)
#define NVRAM_IOC_GET_ERR_BLOCKS	_IOWR(NVRAM_IOC_MAGIC, 26, err_block_t)
#define NVRAM_IOC_SSD_IDENTIFY		_IOWR(NVRAM_IOC_MAGIC, 27, void*)
#define NVRAM_IOC_SSD_FW_UPDATE		_IOWR(NVRAM_IOC_MAGIC, 28, file_info_t)
#define NVRAM_IOC_READ_EEPROM		_IOWR(NVRAM_IOC_MAGIC, 29, file_info_t)
#define NVRAM_IOC_UPDATE_EEPROM		_IOWR(NVRAM_IOC_MAGIC, 30, file_info_t)
#define NVRAM_IOC_TRIGGER_COREDUMP	_IOWR(NVRAM_IOC_MAGIC, 31, int)
#define NVRAM_IOC_CLEAN_COREDUMP	_IOWR(NVRAM_IOC_MAGIC, 32, int)
#define NVRAM_IOC_GET_COREDUMP		_IOWR(NVRAM_IOC_MAGIC, 33, coredump_t)
#define NVRAM_IOC_SSD_TRIM		_IOWR(NVRAM_IOC_MAGIC, 34, int)
#define NVRAM_IOC_GET_INFO		_IOWR(NVRAM_IOC_MAGIC, 35, void*)
#define NVRAM_IOC_SCAP_CALIB_START	_IOWR(NVRAM_IOC_MAGIC, 36, int)
#define NVRAM_IOC_RESET_FW		_IOWR(NVRAM_IOC_MAGIC, 37, int)
#define NVRAM_IOC_GET_DBG_INFO		_IOWR(NVRAM_IOC_MAGIC, 38, wam_dma_dbg_t)
#define NVRAM_IOC_WAM_IOTEST		_IOWR(NVRAM_IOC_MAGIC, 39, wam_iotest_t)
#define NVRAM_IOC_CHARGER_READ		_IOWR(NVRAM_IOC_MAGIC, 40, scap_chrgr_cmd_t)
#define NVRAM_IOC_CHARGER_SET		_IOWR(NVRAM_IOC_MAGIC, 41, scap_chrgr_cmd_t)
#define NVRAM_IOC_CHARGER_SET_VOL	_IOWR(NVRAM_IOC_MAGIC, 42, int)
#define NVRAM_IOC_DDR_SEL_VOL		_IOWR(NVRAM_IOC_MAGIC, 43, int)
#define NVRAM_IOC_GET_NR_DEVICES	_IOWR(NVRAM_IOC_MAGIC, 44, u32_t*)
#define NVRAM_IOC_WAIT_ON_ALERTS	_IOWR(NVRAM_IOC_MAGIC, 45, wam_alert_info_t)
#define NVRAM_IOC_GET_DEV_ALERT		_IOWR(NVRAM_IOC_MAGIC, 46, u32_t*)
#define NVRAM_IOC_CLEAR_DEV_ALERT	_IOWR(NVRAM_IOC_MAGIC, 47, u32_t*)
#define NVRAM_IOC_INJ_ERR		_IOWR(NVRAM_IOC_MAGIC, 48, u32_t)
#define NVRAM_IOC_AIO_WAIT_ID		_IOWR(NVRAM_IOC_MAGIC, 49, unsigned long)
#define NVRAM_IOC_DRV_VERSION		_IOWR(NVRAM_IOC_MAGIC, 50, wam_drv_version_t)
#define NVRAM_IOC_HARDRESET		_IOWR(NVRAM_IOC_MAGIC, 51, int)
#define NVRAM_IOC_SET_ATTR		_IOWR(NVRAM_IOC_MAGIC, 52, attr_pair_t)
#define NVRAM_IOC_GET_ATTR		_IOWR(NVRAM_IOC_MAGIC, 53, attr_pair_t)
#define NVRAM_IOC_READ_TLOG		_IOWR(NVRAM_IOC_MAGIC, 54, thermal_log_t)
#define NVRAM_IOC_CLEAR_TLOG		_IOWR(NVRAM_IOC_MAGIC, 55, thermal_log_t)



/*
 * Address map of mmapable region
 *
 *		+---------------+ 0
 *		|		|
 *		|		|
 *		|		| Registers
 *		|		|
 *		|		|
 *		+---------------+ 0x100000 (1 MB)
 *		|		|
 *		|		| Boot Flash (64 KB)
 *		|		|
 *		+---------------+ 0x110000
 *		|		|
 *		|		|
 *		|		| Internal NVRAM (Event Log) (128 KB)
 *		|		|
 *		|		|
 *		+---------------+ 0x130000
 *		|		|
 *		|		|
 *		|		| 256KB page for SSD FW update
 *		|		|
 *		|		|
 *		+---------------+ 0x150000
 */


/* MMAP offset to access device registers */
#define NV_MMAP_REG_OFFSET		0

/* device register space size (1MB) */
#define NV_REG_SPACE_SIZE               (1 << 20)

/* MMAP offset to access firmware boot flash */
#define NV_MMAP_FLASH_OFFSET            (NV_MMAP_REG_OFFSET + \
						NV_REG_SPACE_SIZE)
/* MMAP size to access boot flash */
#define NV_FLASH_ACCESS_SIZE            (64 << 10)


/* MMAP offset to access internal NVRAM */
#define NV_MMAP_NVSRAM_OFFSET            (NV_MMAP_FLASH_OFFSET + \
						2 * NV_FLASH_ACCESS_SIZE)
/* MMAP size of internal NVRAM */
#define NV_NVSRAM_ACCESS_SIZE            (128 << 10)

/* Scratchpad access offset */
#define NV_MMAP_SRAM_OFFSET		(NV_MMAP_NVSRAM_OFFSET + \
						NV_NVSRAM_ACCESS_SIZE)
#define NV_MMAP_SRAM_SIZE		(256 << 10)

/* Max MMAP offset */
#define NV_MAX_MMAP_OFFSET		(NV_MMAP_SRAM_OFFSET + \
						NV_MMAP_SRAM_SIZE)

/* SATA identify page size */
#define SSD_IDENTIFY_PAGE_SIZE		512

/* SATA identify page size */
#define SSD_FW_UPDATE_PAGE_SIZE		(256 << 10)

#define MV_DMA_DUMP_PAGE_SIZE		(8 << 10)

/*
 * Critical alerts from WAM device that requires admin/application action
 *
 * Redeclared here to avoid namespace collision
 */
#define NV_ALRT_ECC_ERROR	0x1     /* Set when dev reports ECC errors */
#define NV_ALRT_COMP_FAIL	0x2     /* Set when SSD or Supercap fails */
#define NV_ALRT_FW_CRASH	0x4     /* Firmware crash alert */
#define NV_ALRT_INVALID_DATA	0x8     /* Backup or Restore error */
#define NV_ALRT_SCAP_VOL_LOW	0x10    /* Supercap charge dropping */
#define NV_ALRT_SCAP_DEGRADED	0x20    /* Supercap is degraded */
#define NV_ALRT_SCAP_GOOD	0x40    /* Supercap replaced */
#define	NV_ALRT_MAX_TEMP	0x80	/* thermal alert */


typedef struct {
	unsigned int rw;
	unsigned int reg_addr;
	unsigned int reg_val;
} reg_io_t;

/*
 * Async IO Interface
 */
typedef struct {
	u64_t		io_offset;
	u64_t		io_size;
	u32_t		io_op;
	u32_t		io_sync;
	u64_t		io_id;
	u32_t		io_type;
	u32_t		io_vcnt;
	struct iovec	*io_vec;
} nv_aiocb_t;

/*
 * Supercap voltage readings
 */
typedef struct {
	u32_t	sc_cur_vol;		/* current voltage in millivolts */
	u32_t	sc_exp_vol;		/* expected max voltage to reach */
} scap_vol_t;


/*
 * Event log info
 */
typedef struct {
	u32_t	e_nvsram_offset;	/* event log starting offset */
	u32_t	e_rd_ptr;		/* event log read pointer */
	u32_t	e_wr_ptr;		/* event log write pointer */
	u32_t	e_max_events;		/* maximum nr of events */
} event_log_t;

/*
 * VPD Info
 */
typedef struct {
	u32_t	size;			/* length in bytes */
	void	*buf;			/* read buffer */
} vpd_info_t;

/*
 * Error block info
 */
typedef struct {
	u32_t	e_qword_off;		/* qword offset in dram */
	u32_t	e_blk_size:24;		/* err block size in bytes */
	u32_t	e_err_op:8;		/* err reason */
} err_block_t;

/*
 * file transfer from user to kernel (for SSD fw update, eeprom update)
 */
typedef struct {
	void	*file_buf;		/* file contents read into this buf */
	u32_t	file_size;		/* file size */
} file_info_t;

/*
 * core dump transfer from user to kernel (for Firmware Crashdump)
 */
typedef struct {
	void	*core_buf;		/* buffer */
	u32_t	corebuf_size;		/* buffer size */
} coredump_t;

/*
 * WAM Kernel IO API test
 */
typedef struct {
	u32_t	nr_ios;
	u32_t	req_size;
	u8_t	pattern;
	u8_t	rw;
	u8_t	rsvd[6];
	u64_t	offset;
} wam_iotest_t;

/*
 * Supercap charger cmd and data structure
 */
typedef struct {
	u8_t	cmd;
	u8_t	nr_bytes;
	u16_t	data;
} scap_chrgr_cmd_t;

/*
 * Alert notification struct
 */
typedef struct {
	u32_t	w_dev_map;
	u32_t	w_alert_cause;
	u32_t	w_alert_wait_msec;
} wam_alert_info_t;

typedef struct {
	u8_t	w_drv_major;
	u8_t	w_drv_minor;
	u8_t	w_drv_bugfix;
	u8_t	w_drv_rsvd[61];
} wam_drv_version_t;

typedef struct {
	void	*temp_records;
	u32_t	nr_records;
} thermal_log_t;

typedef struct {
	u32_t	w_attr;
	u32_t	w_value;
} attr_pair_t;

#endif
