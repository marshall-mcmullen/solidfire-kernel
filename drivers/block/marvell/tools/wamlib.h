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

/** \file wamlib.h
 *
 * This file contains APIs to control and access WAM device
 *
 * WAM - Write Acceleration Module is a PCI-E device with up to 8GB memory on
 * board, a SSD and a super-capacitor module attached to it. The memory space
 * is presented to the applications as a persistent storage medium. When system
 * loses power, the data in memory is backed up to the SSD storage while the
 * super-capacitor provides the power. When system powers on, the data is
 * restored to the memory space from the SSD.
 *
 * When the WAM device driver is loaded, it creates two device interfaces for
 * each device discovered
 *	- a character device interface (/dev/mvwam[0 to n-1], n - nr of devs)
 *	- a block device interface (/dev/mvblk[0 to n-1], n - nr of devs)
 *
 * The APIs defined in this file work only with the character device interface.
 *
 * The characeter device interface provides both IO and Control path access to
 * the device.
 *
 * Before an application could start writing to the device, it needs to wait
 * until the super-capacitor is charged enough to support a backup. The API to
 * get the device information provides whether the device is ready or not.
 * (see wam_info_t and wam_get_info())
 *
 */

#ifndef __MV_WAMLIB_H
#define	__MV_WAMLIB_H


#include <stdint.h>

#include "mv_common.h"


/*
 * WAM IO Type.
 *
 * IO requests submitted by user space applications could be
 *	1) Regular IO requests which needs no ordering
 *	2) Barrier IO requests which will be scheduled for IO only after all
 *	prior pending requests have been completed
 */
typedef enum {
	NV_NORMAL_IO	= 0,		/* regular io request */
	NV_BARRIER_IO			/* Barrier io request */
} wam_iotype_t;


/*
 * Critical alerts from WAM device that requires admin/application action
 *
 * A device could raise multiple alerts at the same time in which case the
 * alerts are OR'ed together.
 */
#define	WAM_ALRT_ECC_ERROR	0x1	/* Set when dev reports ECC errors */
#define WAM_ALRT_COMP_FAIL	0x2	/* Set when SSD or Supercap fails */
#define	WAM_ALRT_FW_CRASH	0x4	/* Firmware crash alert */
#define	WAM_ALRT_INVALID_DATA	0x8	/* Backup or Restore error */
#define	WAM_ALRT_SCAP_VOL_LOW	0x10	/* Supercap charge dropping */
#define WAM_ALRT_SCAP_DEGRADED	0x20	/* Supercap is degraded */
#define WAM_ALRT_SCAP_GOOD	0x40	/* Bad Supercap replaced with new */


/**
 * API to get number of WAM devices discovered
 *
 * \param fd		- open file descriptor of any WAM char device
 * \param count		- reference to return the device count
 *
 * \return 0 on success, -1 in case of an error.
 */
int wam_get_dev_count(int fd, uint32_t *count);


/*
 * API to ping the device
 *
 * This is a helpful API to check if the firmware is responding to commands
 *
 * \param fd	- open file descriptor of the WAM device
 *
 * \return 0 on success or -1 on error
 */
int wam_ping(int fd);

/**
 * API to get WAM device info
 *
 * \param fd	- open file descriptor of the WAM device
 * \param info	- reference to wam_info_t structure to return the information
 *
 * \return 0 on success or -1 on error.
 */
int wam_get_info(int fd, wam_info_t *info);


/**
 * API to get WAM device status
 *
 * \param fd	- open file descriptor of the WAM device
 * \param info	- reference to wam_status_t structure to return the status
 * of DRAM, SSD and Supercap.
 *
 * \return 0 on success or -1 on error.
 */
int wam_get_status(int fd, wam_status_t *status);


/**
 * API to wait for alerts from any of the WAM devices
 *
 * The calling process will wait for any alerts from atleast one of the WAM
 * devices. If there are no alerts until the given timeout is specified, the
 * errno will be set to ETIMEDOUT.
 *
 * This API will return a device bitmap of all devices which has a pending
 * alert. For example, if dev 0 has an alert, bit 0 will be set to 1
 *
 *	MSB ---------------------------------- LSB
 *	31					0
 *
 * This API will also return the alert cause (one or more alert bits set) of
 * the first device that raises an alert. For example, if dev 2 and dev 4
 * have pending alerts, dev_map will have a value of 0x12, and alert_cause
 * will the 32 bit cause of device 2. 
 *
 * \param fd		- open file descriptor of any WAM char device.
 * \param dev_map	- reference to return a bitmap of device having a
 *			pending alert
 * \param alert_cause	- reference to return alert cause of the first device
 *			that has an alert
 * \param msec		- timeout period in milliseconds, 0 for no wait,
 *			0xffffffff for waiting infinitely
 * 
 * \return 0 on success, -1 in case of timeout. 'errno' variable will
 * be set to ETIMEDOUT
 */
int wam_wait_on_alerts(int fd, uint32_t *dev_map, uint32_t *alert_cause,
		uint32_t msec);

/**
 * API to read device alert cause
 *
 * Get the alert cause of a WAM device
 *
 * \param fd		- open file descriptor of the WAM device
 * \param alert_cause	- reference to return alert cause of the device
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_get_dev_alert(int fd, uint32_t *alert_cause);


/**
 * API to clear or acknowledge device alert
 *
 * Clear or acknowledge device alert by sending a bitmap of alerts that have
 * been handled.
 *
 * \param fd		- open file descriptor of the WAM device
 * \param alert_cause	- alert cause that have been handled so far
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_clear_dev_alert(int fd, uint32_t alert_cause);


/**
 * API to get number of events logged by the WAM device
 *
 * \param fd		- open file descriptor of the WAM device
 * \param nr_events	- reference to return the event count
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_get_nr_events(int fd, uint32_t *nr_events);

/**
 * API to get events
 *
 * Get event log entries from the WAM device. The caller allocates the memory
 * for the events.
 *
 * \param fd		- open file descriptor of the WAM device
 * \param events	- reference to memory allocated by caller to return
 *			the events
 * \param max_events	- max number of events the given buffer can store
 * \param nr_events	- reference to return the number of events returned
 *			by the device
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_get_events(int fd, wam_event_t *events, uint32_t max_events,
		uint32_t *nr_events);

/**
 * API to clear the given number of events
 *
 * Once the event log entries have been to handled/stored to persistent storage
 * the event log can be cleared. This caller provides the number of events to
 * clear.
 *
 * \param fd		- open file descriptor of the WAM device
 * \param nr_events	- number of events to clear
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_clear_events(int fd, uint32_t nr_events);

/**
 * API to read thermal log
 *
 * Get thermal log entries from the WAM device. The caller allocates the memory
 * for the log. The memory should be 8KB of minimum size.
 *
 * \param fd		- open file descriptor of the WAM device
 * \param tlog		- reference to memory allocated by caller to return
 *			the thermal log records
 * \param nr_events	- reference to return the number of records returned
 *			by the device
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_read_thermal_log(int fd, wam_temprec_t *tlog, uint32_t *nr_events);

/**
 * API to clear the given number of records in thermal log
 *
 * After the thermal log entries have been to stored to persistent storage
 * the thermal log can be cleared. This caller provides the number of records
 * to clear.
 *
 * \param fd		- open file descriptor of the WAM device
 * \param nr_records	- number of records to clear
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_clear_thermal_log(int fd, uint32_t nr_records);


/**
 * API to set attribute
 *
 * \param fd		- open file descriptor of the WAM device
 * \param attr_key	- attribute identifier
 * \param attr_value	- attribute value
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_set_attr(int fd, uint32_t attr_key, u32_t attr_value);


/**
 * API to get attribute
 *
 * \param fd		- open file descriptor of the WAM device
 * \param attr_key	- attribute identifier
 * \param attr_value	- reference to return attribute value
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_get_attr(int fd, uint32_t attr_key, u32_t *attr_value);


/**
 * API to read from a WAM character device
 *
 * \param fd		- open file descriptor of the nvram device
 * \param buf		- buffer to read into
 * \param count		- number of bytes to read
 * \param offset	- byte offset on the device to read from
 *
 * \return the number of bytes read, -1 in case of an error; errno will be set
 * EINVAL - offset + count is beyond the device range.
 * ENOMEM - not enough memory to allocate resources
 */
ssize_t wam_read(int fd, void *buf, size_t count, off_t offset);

/**
 * API to write to a WAM character device
 *
 * \param fd		- open file descriptor of the nvram device
 * \param buf		- buffer to write from
 * \param count		- number of bytes to write
 * \param offset	- byte offset on the device to write to
 *
 * \return the number of bytes written, -1 in case of an error; errno will be set
 * EINVAL - offset + count is beyond the device range.
 * ENOMEM - not enough memory to allocate resources
 */
ssize_t wam_write(int fd, void *buf, size_t count, off_t offset);

#ifdef CONFIG_ENABLE_ASYNC_APIS

/**
 * API to write to nvram character device asynchronously
 *
 * The call returns a 64 bit io id (in nv_id). It is a monotonicaly increasing
 * number and a unique value number is assigned for every io.
 *
 * An io with numerically lower id will start and finish before an io with a
 * numerically higher id.
 *
 * If (nv_id == NULL), the call will block until the IO is complete.
 *
 * If the IO type is specified as NV_IO_BARRIER, the data transfer will be
 * initiated only after all pending requests are completed.
 *
 * \param fd		- open file descriptor of the nvram device
 * \param type		- normal request or barrier request
 * \param iovs		- array of iovec structure containing the data buffers
 * \param num_vec	- number of entries in the iovec
 * \param count		- number of bytes to write (<= the sub of all the iovec
 *				 entries)
 * \param offset	- starting write offset on the device
 * \param io_id		- place holder to return a monotonically increasing 64
 *			 bit id for this IO
 *
 * \return : the number of bytes scheduled to be written (actually
 * written for the synchronous case). -1 in case of an error; errno will be
 * set to
 * EINVAL - offset + count is beyond the device range
 * ENOMEM - not enough memory to allocate resources
 */
ssize_t wam_async_write_vec(int fd, wam_iotype_t type, struct iovec *iovs,
				int num_vec, size_t count, off_t offset,
				uint64_t *io_id);

/**
 * API to poll for completion of an IO
 *
 * (should the API block until the IO is complete?)
 *
 * \param fd		- open file descriptor of a nvram character device
 * \param io_id		- id of the io request that was queued earlier
 *
 * See the description for the nv_async_write_vec call for assumptions
 * made about request id.
 *
 * \return : 0 if successful, -1 in case of error; errno will be set to
 * EINVAL - invalid nv_id
 * EAGAIN - request not completed yet. Poll again.
 */
int wam_async_poll(int fd, uint64_t io_id);

/**
 * API to get the completed ID of IO
 *
 * \param fd - open file descriptor of the wam character device
 *
 * \return - ID of the request completed so far
 */
uint64_t wam_get_completed_id(int fd);

/**
 * API to wait on IO completion.
 *
 * The caller provides the IO id to wait on.
 *
 * \param fd - open file descriptor of the wam character device
 * \param io_id - request id on which to wait on completion.
 */
int wam_wait_on_completion(int fd, uint64_t io_id);

#endif /* CONFIG_ENABLE_ASYNC_APIS */


/*
 * WAM Private APIs
 */

/** \fn wam_map_file
 *
 * API to map a file
 */
int
wam_map_file(char *file, int *filefd, char **map, int *size);

/** \fnwam_unmap_file
 *
 * API to unmap a file.
 */
void
wam_unmap_file(int filefd, char *map, int size);


/** \fn wam_update_package
 *
 * API to update software package in wbin format.
 */
int
wam_update_package(int fd, char *filename, int force);

/** \fn wam_update_firmware
 *
 * API to update the firmware in wbin format.
 */
int
wam_update_firmware(int fd, char *filename, int force);

/** \fn wam_update_uboot
 *
 * API to update uboot with header information
 */
int
wam_update_uboot(int fd, char *filename, int force);

/** \fn wam_update_ddr_cfg
 *
 * API to update DDR configuration in wbin format.
 */
int
wam_update_ddr_cfg(int fd, char *filename, int force);

/** \fn wam_update_eeprom
 *
 * API to update EEPROM in wbin format.
 */
int
wam_update_eeprom(int fd, char *filename, int force);

/** \fn wam_update_ssdfw
 *
 * API to update ssd firmware in wbin format.
 */
int
wam_update_ssdfw(int fd, char *filename, int force);

/** \fn wam_get_board_id
 *
 * API to return the board major/minor version.
 */
int
wam_get_board_id(int fd, int *major, int *minor);

/** \fn wam_vpd_print
 *
 * API to print the vpd information
 */
int
wam_vpd_print(int fd);

/** \fn wam_vpd_v2_print
 *
 * API to print the vpd information
 */
int
wam_vpd_v2_print(int fd);

/** \fn wam_convert_vpd
 *
 * API to convert vpd version 1 to version 2.
 * The new vpd information will be at a different sector.
 */
int
wam_convert_vpd(int fd);

int
wam_prog_vpd(int fd, int argc, char **argv);

int
wam_create_vgimg(char *vg_uboot, char *vg_firmware, char *out_file);

int
wam_nvsram_dump(int fd, u8_t *buffer, u32_t bufsize);

int
wam_relative_time2str(uint32_t seconds, char *buffer, uint32_t len);

/*
 * API to print event log of a WAM device to a buffer
 *
 * The caller allocates at least 128KB buffer for this API to print all
 * the information.
 *
 * @fd		- open file descriptor of the WAM device
 * @buffer	- buffer where the output will be printed
 * @length	- buffer size (at least 128KB)
 * @clear	- clear the event log after print
 */
int
wam_print_events(int fd, char *buffer, u32_t length, int clear);

/*
 * API to print general information about WAM device to a buffer
 *
 * The caller allocates at least 4KB page for this API to print all
 * the information.
 *
 * @fd		- open file descriptor of the WAM device
 * @buffer	- buffer where the output will be printed
 * @length	- buffer size (at least 4KB)
 */
int
wam_print_info(int fd, char *buffer, u32_t length, u32_t verbose);

/*
 * API to print thermal log of a WAM device to a buffer
 *
 * The caller allocates at least 128KB buffer for this API to print all
 * the information.
 *
 * @fd		- open file descriptor of the WAM device
 * @buffer	- buffer where the output will be printed
 * @length	- buffer size (at least 128KB)
 * @clear	- clear the thermal log after print
 */
int
wam_print_thermal_log(int fd, char *buffer, u32_t length, int clear);


/*
 * API to print bad blocks that encountered error during backup/restore
 *
 * The caller allocates at least 8KB page for this API to print all
 * the information.
 *
 * @fd		- open file descriptor of the WAM device
 * @buffer	- buffer where the output will be printed
 * @length	- buffer size (at least 8KB)
 */
int
wam_print_bad_blks(int fd, char *buffer, u32_t length);

#endif	/* __MV_WAMLIB_H */
