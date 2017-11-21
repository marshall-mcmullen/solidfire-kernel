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

#ifndef __MV_FW_MSG_H
#define __MV_FW_MSG_H

#define FW_MSG_VER_MAJOR	1
#define FW_MSG_VER_MINOR	0

/* First 8 KB reserved for Firmware */
#define MV_WAM_DRAM_RSVD_SIZE		(8 << 10)
#define MV_DDR_OFFSET			MV_WAM_DRAM_RSVD_SIZE

/* Size of the Auto-load EEPROM */
#define MV_EEPROM_SIZE		256

/* Size of the NVSRAM */
#define MV_NVSRAM_SIZE		(128 << 10)

/*
 * Driver and Firmware messaging sequence and codes
 *
 * Lokiplus provides two message registers and a doorbell register in each
 * direction for small message exchange.
 *
 * The role of the registers when a message is sent from driver to firmware...
 *
 *	1) PCIE_A_2_CPU_MSG_0	- shall contain msg code (0-15) and signature
 *				 (16-31)
 *	2) PCIE_A_2_CPU_MSG_1	- shall contain lower 32 bits of the physical
 *				address of msg payload (if any). This should
 *				be 64K aligned.
 *	3) CPU_2_PCIE_A_MSG_0	- upper 32 bits of physical address
 *	4) PCIE_A_2_CPU_DRBL	- doorbell register to contain appropriate
 *				cause bits to interrupt firmware.
 *
 * The role of registers when a response is sent from firmware to driver...
 *
 *	1) CPU_2_PCIE_A_MSG_0	- shalle contain response code (0-15) and
 *				 signature (16-31)
 *	2) CPU_2_PCIE_A_MSG_1	- 32 bit response (if any)
 *	3) PCIE_A_2_CPU_MSG_0	- 32 bit response (if any)
 *	4) CPU_2_PCIE_A_DRBL	- doorbell to interrupt host driver.
 *				(appropriate intr cause bit will be set)
 */


/** 16 bit signature that needs to be included in each doorbell msg */
#define FW_HOST_2_CPU_DRBL_SIGNATURE	0xCAFE
#define FW_CPU_2_HOST_DRBL_SIGNATURE	0xBABE

/* Doorbell message codes for various operations */
typedef enum {
	DRBL_MSG_PING = 1,		/* ping the fw */
	DRBL_MSG_DMA_SETUP,		/* setup DMA */
	DRBL_MSG_DMA_RESET,		/* reset DMA */
	DRBL_MSG_DMA_STOP,		/* stop DMA */
	DRBL_MSG_DMA_WRITE_READY,	/* Is it ok to start DMA Writes? */
	DRBL_MSG_DDR_BACKUP,		/* backup DDR to SSD */
	DRBL_MSG_DDR_RESTORE,		/* restore from SSD */
	DRBL_MSG_SSD_FORMAT,		/* format the SSD */
	DRBL_MSG_EN_FORCE_ECC,		/* enable forced ecc */
	DRBL_MSG_DIS_FORCE_ECC,		/* disable forced ecc */

	DRBL_MSG_READ_SCAP_REG,		/* read scap adc reg */
	DRBL_MSG_GET_SCAP_VOL,		/* get supercap voltage */
	DRBL_MSG_FW_VERSION,		/* get firmware version */
	DRBL_MSG_GET_BACKUP_STATS,	/* get backup stats record */
	DRBL_MSG_CLEAR_BACKUP_STATS,	/* clear backup stats record */
	DRBL_MSG_SCAP_DISCHARGE_ON,	/* enable supercap discharge */
	DRBL_MSG_SCAP_DISCHARGE_OFF,	/* stop supercap discharge */
	DRBL_MSG_SSD_IDENTIFY,		/* Get SATA Identify page */
	DRBL_MSG_GET_EVNTLOG_PTRS,	/* Get event log rd & wr ptrs */
	DRBL_MSG_SET_EVNTLOG_RDPTR,	/* update event log read pointer */

	DRBL_MSG_IS_DATA_VALID,		/* check for any backup/restore err */
	DRBL_MSG_GET_ERR_BLOCKS,	/* Get the bad DRAM blocks */
	DRBL_MSG_READ_VPD,		/* read VPD */
	DRBL_MSG_SSD_FW_UPDATE,		/* update SSD firmware */
	DRBL_MSG_RESET_CPU,		/* informed reset of CPU */
	DRBL_MSG_GET_FW_CRASHDUMP,	/* get firmware crash dump */
	DRBL_MSG_SSD_TRIM,		/* trim 8GB of SSD */
	DRBL_MSG_READ_EEPROM,		/* read eeprom image */
	DRBL_MSG_UPDATE_EEPROM,		/* update eeprom image */
	DRBL_MSG_CLEAR_FW_CRASHDUMP,	/* clear firmware crash dump */

	DRBL_MSG_TRIGGER_FW_CRASH,	/* simulate firmware crash */
	DRBL_MSG_GET_ECC_INFO,		/* get ecc error record */
	DRBL_MSG_SCAP_CALIB_START,	/* start supercap calibration */
	DRBL_MSG_GET_INFO,		/* get module info (wam_info_t) */
	DRBL_MSG_SET_OPTIONS,		/* command to enable/disable options */
	DRBL_MSG_GET_DMA_DBG_INFO,	/* get dma debug info */
	DRBL_MSG_CHARGER_SET,		/* send pmbus commands to charger */
	DRBL_MSG_CHARGER_READ,		/* read pmbus command output */
	DRBL_MSG_CHARGER_SET_VOL,	/* set charger voltage (in mV) */
	DRBL_MSG_SEL_DDR_VOL,		/* select one of four ddr voltage */

	DRBL_MSG_INJECT_ERR,		/* Error injection msg */
	DRBL_MSG_SET_TIME,		/* set epoch time */
	DRBL_MSG_SET_ATTR,		/* Set device attributes */
	DRBL_MSG_GET_ATTR,		/* Get device attributes */
	DRBL_MSG_GET_ALL_ATTRS,		/* Get all device attributes */
	DRBL_MSG_HARD_RESET,		/* Hard reset device */
	DRBL_MSG_READ_THERMAL_LOG,	/* Read thermal log */
	DRBL_MSG_CLEAR_THERMAL_LOG,	/* Clear thermal log */
	DRBL_MSG_READ_EVENT_LOG,	/* New Read Event log cmd */
	DRBL_MSG_CLEAR_EVENT_LOG,	/* New Clear Event log cmd */

	DRBL_MSG_LAST = 0xFFFF		/* Last msg code (16 bit) */
} mv_drbl_msg_code_t;


/* CPU Doorbell response codes (16 bit) */
typedef enum {
	DRBL_RSP_INVAL_SIGNATURE = 1,	/* Invalid drbl handshake signature */
	DRBL_RSP_HELLO,			/* response to ping */
	DRBL_RSP_VER_MISMATCH,		/* version mismatch of structures */
	DRBL_RSP_DMA_SETUP_DONE,	/* DMA setup done */
	DRBL_RSP_DMA_INVALID_N,		/* Nr of slots not a multiple of 3 */
	DRBL_RSP_DMA_NOMEM,		/* queue length too large */
	DRBL_RSP_DMA_BUSY,		/* DMA commands pending. Try reset */
	DRBL_RSP_DMA_RESET_DONE,	/* DMA reset done */
	DRBL_RSP_DMA_RESET_FAILED,	/* No previous DMA Config exists */
	DRBL_RSP_DMA_STOP_ACK,		/* DMA stop acknowledge */

	DRBL_RSP_DMA_WRITE_READY,	/* Ok to start DMA writes */
	DRBL_RSP_DMA_WRITE_NOTREADY,	/* Super-cap not charged yet */
	DRBL_RSP_DDR_BACKUP_DONE,	/* SSD Backup complete */
	DRBL_RSP_DDR_BACKUP_FAILED,	/* SSD Backup failed */
	DRBL_RSP_DDR_RESTORE_DONE,	/* SSD Restore complete */
	DRBL_RSP_DDR_RESTORE_FAILED,	/* SSD Restore failed */
	DRBL_RSP_SSD_FORMAT_DONE,	/* SSD format complete */
	DRBL_RSP_SSD_FORMAT_FAILED,	/* SSD format failed */
	DRBL_RSP_FORCE_ECC_ON,		/* Forced ECC On */
	DRBL_RSP_FORCE_ECC_OFF,		/* Forced ECC Off */

	DRBL_RSP_READ_SCAP_REG_DONE,	/* read scap reg done */
	DRBL_RSP_GET_SCAP_VOL_DONE,	/* get supercap voltage done */
	DRBL_RSP_FW_VERSION_DONE,	/* firmware version printed to buf */
	DRBL_RSP_GET_BACKUP_STATS_DONE,	/* backup stats written to buf */
	DRBL_RSP_BACKUP_STATS_CLEARED,	/* cleared backup stats record */
	DRBL_RSP_SCAP_DISCHARGE_ON,	/* supercap discharge enabled */
	DRBL_RSP_SCAP_DISCHARGE_OFF,	/* supercap discharge disabled */
	DRBL_RSP_SSD_IDENTIFY_DONE,	/* ssd identify command succeeded */
	DRBL_RSP_SSD_IDENTIFY_FAILED,	/* ssd identify command failed */
	DRBL_RSP_GET_EVNTLOG_PTRS_DONE,	/* get event log pointers done */

	DRBL_RSP_EVNTLOG_RDPTR_UPDATED,	/* event log read pointer updated */
	DRBL_RSP_DATA_IS_VALID,		/* no errors during backup/restore */
	DRBL_RSP_DATA_IS_NOT_VALID,	/* err during backup/restore */
	DRBL_RSP_GET_ERR_BLOCKS_DONE,	/* err blocks copied to buffer */
	DRBL_RSP_READ_VPD_DONE,		/* read VPD done */
	DRBL_RSP_SSD_FW_UPDATE_DONE,	/* SSD firmware update done */
	DRBL_RSP_SSD_FW_UPDATE_FAILED,	/* SSD firmware update failed */
	DRBL_RSP_RESET_CPU_ACK,		/* acknowledge reset CPU cmd */
	DRBL_RSP_FW_CRASHDUMP_DONE,	/* fw crashdump copied to buf */
	DRBL_RSP_FW_CRASHDUMP_EMPTY,	/* no crashdump stored */

	DRBL_RSP_INVALID_ARGS,		/* generic invalid arg response */
	DRBL_RSP_SSD_TRIM_DONE,		/* SSD trim successful */
	DRBL_RSP_SSD_TRIM_FAILED,	/* SSD trim failed */
	DRBL_RSP_EEPROM_READ_DONE,	/* eeprom image read successful */
	DRBL_RSP_EEPROM_READ_FAILED,	/* eeprom read failed */
	DRBL_RSP_EEPROM_UPDATE_DONE,	/* eeprom image update successful */
	DRBL_RSP_EEPROM_UPDATE_FAILED,	/* should never happen */
	DRBL_RSP_FW_CRASHDUMP_CLEARED,	/* crashdump copy cleaned */
	DRBL_RSP_FW_CRASH_TRIGGERED,	/* crashdump simulated */
	DRBL_RSP_ECC_INFO_COPIED,	/* ecc record copied to host */

	DRBL_RSP_DMA_ACTIVE,		/* DMA must be stopped for the cmd */
	DRBL_RSP_CALIB_START_FAILED,	/* scap is not fully charged */
	DRBL_RSP_SCAP_CALIB_STARTED,	/* scap calib started */
	DRBL_RSP_GET_INFO_DONE,		/* get info command done */
	DRBL_RSP_SET_OPTIONS_DONE,	/* option enabled/disabled */
	DRBL_RSP_SET_OPTIONS_FAILED,	/* set option failed */
	DRBL_RSP_INVALID_OPTION,	/* invalid option to en/disable */
	DRBL_RSP_DMA_DBG_INFO_COPIED,	/* Debug info copied to buffer */
	DRBL_RSP_CHARGER_ACCESS_DONE,	/* charger access done */
	DRBL_RSP_CHARGER_ACCESS_FAILED,	/* charger access failed */

	DRBL_RSP_DDR_VOLSEL_DONE,	/* DDR Voltage selection success */
	DRBL_RSP_DDR_VOLSEL_FAILED,	/* DDR Voltage selection failed */
	DRBL_RSP_ERR_INJECTED,		/* Error injection done */
	DRBL_RSP_TIME_SET,		/* Epoch time set */
	DRBL_RSP_ONLINE_CALIB_FAILED,	/* Cannot start online hchk */
	DRBL_RSP_SET_ATTR_DONE,		/* Attribute set */
	DRBL_RSP_GET_ATTR_DONE,		/* Got the attribute value */
	DRBL_RSP_GET_ALL_ATTRS_DONE,	/* Get all the settings */
	DRBL_RSP_INVALID_ATTR_KEY,	/* Invalid attr key */
	DRBL_RSP_INVALID_ATTR_VALUE,	/* Invalid attr value */

	DRBL_RSP_HARD_RESET_ACK,	/* Hard Reset Acknowledgement */
	DRBL_RSP_THERMAL_LOG_COPIED,	/* Thermal log copied to buffer */
	DRBL_RSP_THERMAL_LOG_CLEARED,	/* Thermal log cleared */
	DRBL_RSP_EVENT_LOG_COPIED,	/* Event log copied to buffer */
	DRBL_RSP_EVENT_LOG_CLEARED,	/* Event log cleared */

	DRBL_RSP_NOT_SUPPORTED = 0xF000,/* cmd/msg not supported yet */
	DRBL_RSP_CMD_DISABLED,		/* cmd disabled in this fw ver */
	DRBL_RSP_CMD_OBSOLETE,		/* cmd obsolete */

	DRBL_RSP_UNKNOWN_MSG = 0xFFFF	/* last response code */
} mv_drbl_rsp_code_t;

/*
 * NVRAM Write readiness status enumerations
 */
enum {
	SCAP_NOT_CONNECTED,		/* supercap not connected */
	SCAP_CHARGING,			/* supercap is charging */
	SCAP_CHARGED,			/* charging complete */
	SCAP_DISCHARGING,		/* supercap is discharging */
	SCAP_DEGRADED,			/* degraded supercap. cannot backup */
};

enum {
	SSD_NOT_CONNECTED,		/* ssd DOM not attached */
	SSD_ERASE_IN_PROGRESS,		/* ssd erase in progress. not ready */
	SSD_CONNECTED,			/* ssd connected and ready for IO */
	SSD_NON_RESPONSIVE,		/* ssd connected but no sign recvd */
	SSD_IO_FAILS,			/* ssd connected but I/O fails */
	SSD_NEED_TO_ERASE,		/* yet to start erasing */
	SSD_EOL_REACHED,		/* SSD End Of Life reached */
};

enum {
	DRAM_POST_SKIPPED,		/* memory test skipped */
	DRAM_POST_SUCCESS,		/* POST memory test was successful */
	DRAM_POST_FAILED,		/* POST memory test failed */
};


/*
 * Enable and disable different options
 */
enum {
	/* Move from RDONLY to write mode. This option may fail if either
	 * Supercap or SSD are in a failed state. This option will mark the
	 * DRAM to be in 'DIRTY' mode requiring backup if it is not already
	 * in that state.
	 */
	EN_DRAM_WRITE_MODE	= 1,

	/* Enable SSD Trim feature. This feature will trim (erase) 8GB of
	 * SSD space after data is restored to DRAM and after supercap
	 * reaches the safe voltage level. Trimming the SSD reduces the
	 * backup time to 50 seconds (from 60 seconds). This feature needs
	 * SSD firmware support and it is supported from SSD firmware version
	 * KCF7 and beyond.
	 */
	EN_SSD_TRIM,

	/* Disable SSD Trim feature */
	DIS_SSD_TRIM,
};


/*
 * Error Injection message codes
 */
enum {
	WAM_INJE_SCAP_DISCONNECT = 1,
	WAM_INJE_SCAP_LOW_VOL,
	WAM_INJE_SCAP_DEGRADED,
	WAM_INJE_SSD_NOT_DETECTED,
	WAM_INJE_SSD_IO_FAILURE,
	WAM_INJE_SSD_EOL,
	WAM_INJE_INV_DATA
};


/*
 * Severity level of WAM event log entries
 */
enum {
	WAM_SEV_DEBUG		= 0,
	WAM_SEV_INFO,
	WAM_SEV_WARN,
	WAM_SEV_CRITICAL,
};


/*
 * WAM Event log entry
 */
typedef struct {
	u32_t	e_time;			/* time in seconds when event occrd */
	u16_t	e_num;			/* event number */
	u8_t	e_severity;		/* event severity */
	u8_t	e_rsvd;			/* reserved */
	u32_t	e_data;			/* event specific data */
} wam_event_t;

/*
 * Event numbers
 */
enum {
	WAMEV_DRAM_ECC_ERR	= 1,	/* 1. DRAM ECC Error */
	WAMEV_CPU_CRASH,		/* 2. FW Crash */
	WAMEV_SSD_NOT_DETECTED,		/* 3. SSD not detected */
	WAMEV_SSD_NOT_RESPONDING,	/* 4. SSD is not responding */
	WAMEV_SSD_IO_FAILING,		/* 5. SSD IO fails */
	WAMEV_SSD_EOL,			/* 6. SSD nearing End of Life */
	WAMEV_SCAP_DISCONNECTED,	/* 7. SCAP not/dis-connected */
	WAMEV_SCAP_VOL_LOW,		/* 8. SCAP voltage too low */
	WAMEV_SCAP_DEGRADED,		/* 9. SCAP degraded. Replace it */
	WAMEV_MEM_TEST_FAILED,		/* 10. Simple Memory POST failed */
	WAMEV_DATA_BACKUP_FAILED,	/* 11. Data backup error detected */
	WAMEV_DATA_RESTORE_FAILED,	/* 12. Data could not be restored */
	WAMEV_DATA_LOSS_DETECTED,	/* 13. Data loss backup/restore err */
	WAMEV_MAX_TEMP_ALERT,		/* 14. Max temperature alert */


	/*
	 * Codes upto 31 reserved
	 */

	WAMEV_DEV_WR_READY	= 32,	/* 32. DRAM marked dirty. Bkup reqd */
	WAMEV_DEV_ERR_CLEARED,		/* 33. Prior Error Cleared */
	WAMEV_SSD_FW_UPDATED,		/* 34. SSD fw updated */
	WAMEV_SSD_FORMATTED,		/* 35. SSD formatted */
	WAMEV_POWER_DOWN,		/* 36. Power loss detected */
	WAMEV_POWER_ON,			/* 37. Power on during backup */
	WAMEV_PCIE_RESET,		/* 38. PCI-E link reset */
	WAMEV_CPU_RESET,		/* 39. WAM FW reset */
	WAMEV_DATA_BACKUP_START,	/* 40. Backup started */
	WAMEV_DATA_BACKUP_DONE,		/* 41. Backup completed */
	WAMEV_DATA_RESTORE_START,	/* 42. Restore started */
	WAMEV_DATA_RESTORE_DONE,	/* 43. Restore completed */
	WAMEV_FORCE_ECC_ENABLED,	/* 44. DRAM Forced ECC ON */
	WAMEV_FORCE_ECC_DISABLED,	/* 45. DRAM Forced ECC Off */
	WAMEV_FW_BOOTUP,		/* 46. Device FW just loaded */
	WAMEV_SCAP_VOL,			/* 47. Current Supercap vol */
	WAMEV_DMA_SETUP,		/* 48. Host setup DMA */
	WAMEV_MEM_TEST_START,		/* 49. Memory test started */
	WAMEV_MEM_TEST_DONE,		/* 50. Memory test done */
	WAMEV_ERR_INJECTED,		/* 51. Error Injection from user */
	WAMEV_SCAP_HEALTH,		/* 52. Supercap health result */
	WAMEV_SCAP_HCHK_START,		/* 53. Health check started */
	WAMEV_VPD_NOT_FOUND,		/* 54. VPD not programmed */
	WAMEV_PCIE_LINK_STATE,		/* 55. PCIE Link State */
	WAMEV_TIME_SET,			/* 56. Time synced with host */
	WAMEV_NO_AUTO_HCHK,		/* 57. Not safe to start auto check */
	WAMEV_HARD_RESET,		/* 58. Hard Reset from host */
	WAMEV_EVENT_LOG_INIT,		/* 59. First time event log init */

	WAMEV_LAST_ENTRY,

	WAMEV_DEV_EVENT	= 0xF000,	/* FW Internal events */
};




/* CPU to Host Doorbell Interrupt cause bits */
#define FW_DRBL_MSG_1_IRQ		0x80000000
#define FW_DRBL_MSG_0_IRQ		0x40000000
#define FW_DRBL_HANDSHAKE_IRQ		0x00000001
#define FW_DRBL_DMA_CMPLT		0x00000002	/* dma completion */
#define FW_DRBL_CPU_CRASH		0x00000004	/* fw crash */
#define FW_DRBL_UNKNOWN_RESET		0x00000008	/* unexpected reset */
#define FW_DRBL_ECC_ERROR		0x00000010	/* ECC error */
#define FW_DRBL_DEV_WR_READY		0x00000020	/* dev write ready */
#define FW_DRBL_SCAP_DISCONNECTED	0x00000040	/* scap disconnected */
#define FW_DRBL_SCAP_VOLTAGE_LOW	0x00000080	/* scap charge low */
#define FW_DRBL_SSD_DISCONNECTED	0x00000100	/* ssd disconnected */
#define FW_DRBL_SSD_FAILURE		0x00000200	/* ssd not responding*/
#define FW_DRBL_SSD_EOL			0x00000400	/* ssd reaching EOL */
#define FW_DRBL_MEMORY_TEST_FAILED	0x00000800	/* memtest failed */
#define FW_DRBL_SCAP_CALIB_DONE		0x00001000	/* scap calib done */
#define FW_DRBL_HANDSHAKE_READY		0x00002000	/* fw ready to talk */
#define FW_DRBL_DATA_BACKUP_DONE	0x00004000	/* backup completed */
#define FW_DRBL_SCAP_DEGRADED		0x00008000	/* scap degraded */
#define FW_DRBL_SCAP_GOOD		0x00010000	/* bad scap replaced */
#define	FW_DRBL_MAX_TEMP_ALERT		0x00020000	/* thermal alert */

/*
 * Crossbar interfaces to be specified in each scatter gather entry
 */
typedef enum {
	IFC_HOST_MEM = 0,	/* host memory location */
	IFC_DDR_CS0,		/* DDR chip select 0 (0 - 2 GB) */
	IFC_DDR_CS1,		/* DDR chip select 1 (2 - 4 GB) */
	IFC_DDR_CS2,		/* DDR chip select 2 (4 - 6 GB) */
	IFC_DDR_CS3,		/* DDR chip select 3 (6 - 8 GB) */
	IFC_SCRPAD,		/* internal scratchpad - only for fw */
	IFC_LAST_ENTRY		/* last entry */
} xbar_ifc_t;


/*
 * Physical Region Descriptor (PRD) Entry - Scatter Gather Entry
 */
typedef struct {
	u32_t	prd_addr_low;		/* addr low 32 bits */
	u32_t	prd_addr_high;		/* addr high 32 bits */
	u32_t	prd_rsvd;		/* reserved */
	u32_t	prd_byte_cnt:22;	/* length in bytes */
	u32_t	prd_rsvd2:6;		/* reserved */
	u32_t	prd_xbar_ifc:4;		/* xbar interface */
} prd_t;


/*
 * ECC Error Information structure
 *
 * Fields ecc_calc_val, ecc_rcvd_val, ecc_data_low, ecc_data_high values may
 * not be available for all the occurrences. This information is latched by
 * the hardware at the first occurrence and only cleared when the firmware
 * or driver clears it. Any further occurrence between the two events will
 * not be latched. With three data transfer engines, there is a possibility
 * of more than one engine running into ecc error and information about only
 * occurrence getting latched.
 */
typedef struct {
	u32_t	ecc_addr_low;		/* lower 32 bits of the address */
	u8_t	ecc_ifc;		/* ifx in which ecc error occured */
	u8_t	ecc_type;		/* single or double bit */
	u8_t	ecc_calc_val;		/* calculated ecc value */
	u8_t	ecc_rcvd_val;		/* received ecc value */
	u32_t	ecc_data_low;		/* lower 32 bits of data */
	u32_t	ecc_data_high;		/* upper 32 bits of data */
} ecc_err_info_t;


/*
 * DMA Completion Status Bits.
 */
enum {
	DMA_CMD_COMPLETE	= 0x0001,	/* Command complete bit */
	DMA_ECC_ERROR		= 0x0002,	/* ECC Error occurred */
};

/*
 * DMA Completion Queue Entry
 */
typedef struct {
	u16_t cmd_status;	/* status with one or more bits set */
	u16_t cmd_slot;		/* slot nr in the PRD table array which
				   was completed */
} cmplq_entry_t;

/*
 * WAM Shadow Cause notification structure
 *
 * The WAM firmware interrupts the host driver for DMA completions and other
 * alerts. DMA completions can occur at a very high rate in the order of 125K+
 * depending on the size of the IO. The host driver needs to read a device
 * register to identify the cause of the interrupt. To avoid reading this
 * register on every interrupt, the following structure is used by the firmware
 * to write the cause notification to the host memory. Since DMA completions
 * occur at a very high rate, this shadow notification is updated for all
 * alerts except for DMA completions.
 *
 * This structure has a counter and a shadow interrupt cause value. For every
 * alert, the firmware will copy the interrupt cause register to the shadow
 * location and increment the counter. The host can use a local counter to
 * identify whether a cause has been noted or not. On an interrupt, the host
 * checks the local counter and the alert counter. If they match, the interrupt
 * is for DMA completion. If they do not match, read the shadow interrupt cause
 * to identify the alert.
 *
 * To clear the interrupt, write to the actual interrupt cause register.
 */
typedef struct {
	u32_t	fw_alert_counter;
	u32_t	fw_shdw_intr_cause;
} wam_shdw_cause_t;


/*
 * Completion Queue structure.
 *
 * This structure resides in the host memory and updated by the firmware
 * for each DMA completion. In case of an ECC error, the error information
 * is recorded in one of the ECC records and the DMA operation is stopped.
 *
 * In case of any other firmware alerts, the shadow cause values will be
 * updated by the firmware.
 */
typedef struct {
	ecc_err_info_t	cmpl_ecc_info[3];	/* ecc records per eng */
	wam_shdw_cause_t cmpl_shdw_cause;	/* shadow intr cause */
	u32_t		cmpl_rsvd[10];	/* reserved fields */
	u32_t		cmpl_wr_ptr;	/* most recently updated entry index */
	cmplq_entry_t	cmpl_ents[1];	/* completion queue entries */
} cmplq_t;

/* Magic for new read-only option. For cases when data is safe in SSD and do
 * not want backup to be triggered on power failure.
 */
#define DRAM_RDONLY_MODE_MAGIC		0x10305070

/*
 * DMA configuration parameters
 *
 * To configure the firmware for DMA, the driver needs to initialize
 * this structure with the defined parameters at a 64K aligned memory
 * in the host. The driver should then program the below registers and
 * set the host_to_cpu doorbell register.
 *
 *	1) Register PCIE_A_2_CPU_MSG_1 with lower 32 bits of physical addr of
 *	this structure
 *	2) Register CPU_2_PCIE_A_MSG_0 with upper 32 bits of the address
 *	3) Register PCIE_A_2_CPU_MSG_0 with MSG signature and Msg code
 *	DRBL_MSG_DMA_SETUP
 */
typedef struct {
	/* Nr of cmd slots. should be a multiple of 3 */
	u32_t   d_nr_slots;

	/* Nr of interrupts to coalesce. 0 means no coalescing */
	u16_t   d_intr_coal_cnt;

	/* Nr of src prds per cmd */
	u8_t	d_src_prds;

	/* Nr of dst prds per cmd */
	u8_t	d_dst_prds;
	/* Note:  d_src_prds + d_dst_prds should be either
	 * 8 or 16 at the maximum. The format of PRD table for each command
	 * will be...
	 *	struct prd_table {
	 *		prd_t	src_prds[d_src_prds];
	 *		prd_t	dst_prds[d_dst_prds];
	 *	};
	 */

	/* lower 32 bits of PRD table array base address */
	u32_t   d_prd_arr_low;

	/* upper 32 bits of PRD table array base address */
	u32_t   d_prd_arr_high;

	/* lower 32 bits of completion queue */
	u32_t   d_cmplq_low;

	/* upper 32 bits of completion queue */
	u32_t   d_cmplq_high;

	/* Enable read-only mode. For cases when data is safe in SSD and do
	 * not want backup to be triggered on power failure. The magic value
	 * is defined by macro DRAM_RDONLY_MAGIC
	 */
	u32_t	d_rdonly_magic;
} mv_dma_cfg_t;


/*
 * Backup report and stats
 */
typedef struct {
	u16_t	b_start_time[4];	/* start time in secs for each CS */
	u16_t	b_end_time;		/* 8GB backup end time */
	u8_t	b_completed;		/* completed or not? flag
					   0 - Incomplete, 1 - Completed */
	u8_t	b_flush_count;		/* nr of ssd cache flushes */
	u16_t	b_max_time[4];		/* for each CS, max time for an IO */
	u16_t	b_min_time[4];		/* for each CS, min time for an IO */
	u16_t	b_err_count;		/* nr of errors during backup */
	u16_t	b_alive_time;		/* total time the supercap lasted */
	u16_t	b_start_scap_vol;	/* starting scap voltage */
	u16_t	b_end_scap_vol;		/* at the end of backup, scap vol */
	u32_t	b_ddr_offset;		/* quad-word offset completed */
	u32_t	b_sbe_cnt;		/* single bit ecc count at the end */
	u32_t	b_dbe_cnt;		/* double bit ecc count at the end */
	u16_t	b_ssd_flush_time;	/* time when ssd wc flush started */
	u16_t	b_ptr_mismatch;		/* reserved */
	u32_t	b_ptr_mismatch_delay;	/* reserved */
	u16_t	b_ssd_errs;		/* ssd error count */
	u16_t	b_dma_errs;		/* dma error count */
} backup_stats_t;


/*
 * write ready status format
 *
 * Size of this structure should be 32 bits.
 */
typedef struct {
	u8_t	w_post_res;
	u8_t	w_ssd_stat;
	u8_t	w_scap_stat;
	u8_t	w_scap_charge;
} wam_status_t;


/*
 * thermal log entry format
 *
 * A thermal record is logged every day with the avergage, max and min
 * thermal readings for that day. The record also has fields for hour
 * mark (0 to 23) when the temperatures hit the high and low.
 */
typedef struct {
	u32_t	wt_time;		/* time stamp */
	u32_t	wt_avg_temp:8;		/* average temperature for the day */
	u32_t	wt_max_temp:7;		/* max reading */
	u32_t	wt_min_temp:7;		/* min reading */
	u32_t	wt_max_ts:5;		/* hour mark of max reading (0 to 23) */
	u32_t	wt_min_ts:5;		/* hour mark of min reading (0 to 23) */
} wam_temprec_t;


/* String lengths used in WAM data structures */
#define WAM_MAX_SN_LEN			24
#define WAM_SN_LEN			18
#define WAM_FW_VER_LEN			64
#define WAM_PKG_VER_LEN			16
#define WAM_SSD_FW_VER_LEN		8
#define WAM_SSD_SN_LEN			20

/*
 * WAM device status flags
 */

/*
 * Data in DRAM is not valid due to backup/restore error. If data in DRAM is 
 * ever lost, the invalid status is persistent until it is cleared by the
 * application or admin using a seperate API (see wam_acquit)
 */
#define WAM_STAT_DATA_INVALID		0x00000001

/* The supercap is charged and the device is available for writes */
#define WAM_STAT_WRITE_READY		0x00000002

/* Set when SSD is connected */
#define WAM_STAT_SSD_CONN		0x00000004

/* Set when supercap is connected */
#define WAM_STAT_SCAP_CONN		0x00000008

/* Set when supercap is charging */
#define WAM_STAT_SCAP_CHARGING		0x00000010

/* Set when supercap discharging is enabled (usually for calibration) */
#define WAM_STAT_SCAP_DISCHARGE_ON	0x00000020

/* Set when a firmware crashdump is available to be read */
#define WAM_STAT_FW_CRASHDUMP		0x00000040

/* Set when forced ECC (false ECC) flag is enabled */
#define WAM_STAT_FORCED_ECC_ON		0x00000080

/* Set when firmware and host driver have initialized data path */
#define WAM_STAT_DMA_ON			0x00000100

/* Set when supercap calibration in ongoing */
#define WAM_STAT_SCAP_CALIB_ON		0x00000200

/* Set when DRAM state is dirty and must be backed up in case of power loss */
#define WAM_STAT_DATA_DIRTY		0x00000400

/* Set when SSD trim feature is enabled (for diagnostics) */
#define WAM_STAT_TRIM_ENABLED		0x00000800

/* Set when supercap is healthy enough to support power loss during
 * health check */
#define WAM_STAT_SCAP_HEALTHY		0x00001000


/*
 * Supercap life indicator
 *
 * The life indicator is based on the capacitance, the charging voltage
 * required to charge the capacitor and the maximum charging voltage allowed for
 * a board. The charging voltage ranges from a minimum of 4.55V to 5.35V. Based
 * on the board revision, the minimum and maximum allowed values vary.
 *
 * Below are the typical minimum and maximum values for different board
 * revisions
 *	Board Rev	Minimum		Maximum
 *	<= 3.3		  5V		  5.35V
 *	== 3.4		 4.6V		  5.05V
 *	>= 3.5		 4.6V		  5.35V
 *
 * Below are the different life indicators:
 *
 * Indicator	   Vmax - Vcharge	 Cap % (68F = 100%)
 * -------------------------------------------------------
 * VERY_GOOD	     >= 400 mV	    &&	     >= 95%
 * GOOD		     >= 300 mV	    &&	     >= 80%
 * FAIR		     >= 250 mV	    &&	     >= 75%
 * LOW		      < 250 mV	    &&	     >= 70%
 * DEGRADED	     == 0	    &&	      < 70%
 *
 * LOW:
 * When the life indicator reaches to LOW, further health measurements can be
 * done only in the offline mode. The device may still be used. However, it is
 * recommended to replace the supercapacitor
 *
 * DEGRADED:
 * The device will not transition to write-ready state. It will not be able to
 * support a backup in case of a power loss. The capacitor should be replaced.
 *
 * Replacing capacitor:
 * When the capacitor is replaced, the subsequent power on health check will be
 * started in the offline mode and at the end of the measurement, the life
 * indicator will change based on the new capacitor.
 */
enum {
	WAM_SC_VERY_GOOD = 1,
	WAM_SC_GOOD,
	WAM_SC_FAIR,
	WAM_SC_LOW,
	WAM_SC_DEGRADED,
};

/*
 * WAM Info structure
 *
 * This data structure contains the complete information about a WAM device
 * and its current state. Application(s) can use the wam_get_info API to
 * obtain this information and take necessary actions.
 *
 * In general, this structure contains
 *	- Software version info
 *		* WAM firmware version
 *		* SSD firmware version
 *		* Internal EEPROM version
 *		* Configuration file (if used) version
 *
 *	- Serial Numbers
 *		* device serial number
 *		* SSD serial number (for models involving removable SSD module)
 *
 *	- Device Status flags
 *	- Supercap health information
 *	- SSD health information
 *	- DRAM Size information
 *	- Board Temperature
 */
typedef struct {
	char	w_sn[WAM_MAX_SN_LEN];		/* serial number */
	char	w_fw_ver[WAM_FW_VER_LEN];	/* firmware version */

	u8_t	w_eeprom_ver;			/* eeprom version */
	u8_t	w_ddr_cfg_ver;			/* ddr config version */
	u16_t	w_dclk;				/* DDR Clock in MHz */
	u16_t	w_tclk;				/* Internal Bus Clock in MHz */
	u16_t	w_pclk;				/* CPU Clock in MHz */

	u32_t	w_rsvd;				/* reserved */

	u32_t	w_flags;			/* status flags */
	u32_t	w_uptime;			/* firmware uptime */

	/* Supercap info */
	u16_t	w_sc_vol;			/* scap voltage */
	u16_t	w_sc_exp_vol;			/* scap design voltage */
	u16_t	w_sc_cap;			/* calib result in Farads */
	u16_t	w_sc_design_cap;		/* scap designed capacitance */

	u16_t	w_sc_calib_progress;		/* scap calib progress in % */
	u16_t	w_sc_nr_calib;			/* nr of calibrations */
	u8_t	w_sc_max_alive;			/* max scap alive time (secs)*/
	u8_t	w_sc_min_alive;			/* min scap alive time */
	u8_t	w_sc_avg_alive;			/* avg scap alive time */
	u8_t	w_sc_life;			/* scap life indicator */

	/* SSD Info */
	char	w_ssd_sn[WAM_SSD_SN_LEN];	/* SSD serial number */
	char	w_ssd_fw_ver[WAM_SSD_FW_VER_LEN];	/*SSD fw version */
	u32_t	w_ssd_state;			/* current SSD state */
	u32_t	w_ssd_errors;			/* total ssd error count */

	/* SSD SMART Data */
	u32_t	w_ssd_prg_err_cnt;		/* Program Error Count */
	u32_t	w_ssd_erase_err_cnt;		/* Erase Error Count */
	u32_t	w_ssd_defect_blks;		/* defective blocks 8MB each */
	u32_t	w_ssd_avg_erase_cnt;		/* Average Error Count */
	u32_t	w_ssd_rsvd[2];			/* reserved */

	/* DRAM Info */
	u32_t	w_dram_size;			/* dram size in KB */
	u32_t	w_dram_rsvd_size;		/* reserved area for fw in KB*/
	u32_t	w_dram_sbe;			/* current sbe count */
	u32_t	w_dram_dbe;			/* current dbe count */
	u32_t	w_dram_total_sbe;		/* lifetime sbe count */
	u32_t	w_dram_total_dbe;		/* lifetime dbe count */

	/* Counters - TBD */
	u32_t	w_power_loss_count;		/* total power loss count */
	u32_t	w_backup_count;			/* total backup count */
	u32_t	w_data_loss_count;		/* data losses if any */
	u32_t	w_fw_crash_count;		/* total fw crashes */

	u32_t	w_events_unread;		/* nr of events unread */
	u32_t	w_board_temp;			/* board temparature in C */

	u32_t	w_host_time;			/* calendar time in secs */

	/* Additional SCAP info */
	u16_t	w_sc_min_vout;			/* min vol allowed to set */
	u16_t	w_sc_max_vout;			/* min vol allowed to set */
	int	w_sc_trim_vout;			/* charger trim voltage */
	u32_t	w_sc_next_hchk;			/* time when next hchk starts */
	u32_t	w_sc_rsvd2;
	u32_t	w_sc_rsvd3;
	u32_t	w_sc_rsvd4;

	/* Thermal info */
	u8_t	w_temp_avg;			/* Average temp for the day */
	u8_t	w_temp_max;			/* Max temp for the day */
	u8_t	w_temp_min;			/* Min temp for the day */
	u8_t	w_temp_alert;			/* Thermal alert temp */

	u32_t	w_temp_max_ts;			/* Max temp timestamp */
	u32_t	w_temp_min_ts;			/* Min temp timestamp */
} wam_info_t;


typedef struct {
	u32_t	xr_nr_slots;
	u32_t	xr_ctrl_xor_tbl;
	u32_t	xr_ctrl_prd_tbl;

	u32_t	xr_data_xor_tbl;
	u32_t	xr_data_prd_tbl;
	u32_t	xr_prds_per_cmd;

	u32_t	xr_cmd_tbl[MV_MAX_ENGINES];
	u32_t	xr_dlvryq[MV_MAX_ENGINES];
	u32_t	xr_cmplq[MV_MAX_ENGINES];
} wam_dma_dbg_t;

/*
 * Vital Product Data
 */
#define	WAM_VPD_FLASH_OFFSET		0x6C0000
#define	WAM_SN_FLASH_OFFSET		0x6F0000
#define WAM_VPD_MAGIC			(0xcadea5a5)

#define WAM_VPD_FMT_MAJOR		2
#define WAM_VPD_FMT_MINOR		0
#define WAM_VPD_BIN_LEN			(512)
#define WAM_VPD_NON_BIN_LEN		(508)

#define WAM_VPD_STR_VALUE_LEN		(32)
#define WAM_VPD_STR_VALUE_LEN_SHORT	(16)
#define WAM_VPD_STR_VALUE_LEN_LONG	(64)

typedef struct wam_vpd_s {
	/* VPD_BIN_LEN bytes of binary info (in little endian) */
	union {
		struct {
			/* 64 bytes vpd format info */
			u32_t   v_magic;
			u8_t    v_vpd_major;
			u8_t    v_vpd_minor;
			u16_t   v_vpd_size;
			u32_t   v_rsvd1[14];

			/* board information */
			u8_t    v_board_major;
			u8_t    v_board_minor;
			u16_t   v_subsys_id;
			u16_t   v_subsys_ven_id;
			u16_t   v_board_rsvd;
		};
		unsigned char v_bin_rvsd[WAM_VPD_BIN_LEN];
	};

	/* VPD_NON_BIN_LEN bytes of VPD info in ascii */
	union {
		struct {
			char    v_id[WAM_VPD_STR_VALUE_LEN_LONG];
			char    v_pn[WAM_VPD_STR_VALUE_LEN];
			char    v_ec[WAM_VPD_STR_VALUE_LEN];
			char    v_sn[WAM_VPD_STR_VALUE_LEN];
			char    v_mn[WAM_VPD_STR_VALUE_LEN];
			char    v_bn[WAM_VPD_STR_VALUE_LEN];
			u8_t    v_wwn[WAM_VPD_STR_VALUE_LEN_SHORT];
		};
		unsigned char v_rsvd2[WAM_VPD_NON_BIN_LEN];
	};

	u32_t    v_checksum;
} wam_vpd_t;


/*
 * WAM Attributes
 */

#define	WAM_MAX_BOOL_ATTR	128
#define	WAM_BOOL_ATTR_DWORDS	(WAM_MAX_BOOL_ATTR / 32)
#define WAM_ATTR_DWORDS	32

/*
 * Data structure holding WAM attributes
 *
 * This structures holds both boolean and dword type attributes for the
 * WAM device. The attributes can be modified using SET_ATTR FW command
 *
 */
typedef struct {
	u32_t	wa_bool_attr[WAM_BOOL_ATTR_DWORDS];
	u32_t	wa_attr[WAM_ATTR_DWORDS];
} wam_attr_t;


/*
 * Attribute identifiers
 */
enum {
	/* Boolean type attributes */
	WA_OFFLINE_HCHK_ON_COLDBOOT		= 0,

	WA_MAX_BOOL_ATTR_IMPL,
	/* Values 1-127 reserved */

	/* health check schedule time period in seconds */
	WA_SCAP_HEALTHCHECK_SCHEDULE		= 128,
	WA_THERMAL_ALERT_TEMP			= 129,
	WA_MAX_ATTR_IMPL,
};


#endif		/* __MV_FW_MSG_H */

