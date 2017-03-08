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

/** \file wamlib.c
 *
 * This file contains APIs to control and access WAM device
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/uio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <ctype.h>
#include <time.h>

#include "loki.h"
#include "wamlib.h"
#include "package.h"
#include "wamlib.h"
#include "wamlib_priv.h"


#define	WPRINT_ERR(fmt, args...)		fprintf(stderr, fmt, ##args)



const char *event_str[] = {
	"",
	"DRAM ECC Error",
	"FW Crashed",
	"SSD Not Detected",
	"SSD Not Responding",
	"SSD IO Fails",
	"SSD End of Life Reached",
	"Supercap Not Connected",
	"Supercap Voltage Low",
	"Supercap Degraded",
	"Memory Test Failed",
	"Data Backup Failed",
	"Data Restore Failed",
	"Data Loss Detected",
	"Max Temperature Alert",

	"", "", "", "",
	"", "", "", "",
	"", "", "", "",
	"", "", "", "",
	"",

	"Device Write Ready",
	"Backup/Restore Error was cleared",
	"SSD FW updated",
	"SSD Formatted",

	"Power Loss Detected",
	"Power On Detected",
	"PCI-E Reset",
	"User/FW triggered CPU Reset",

	"Data Backup Started",
	"Data Backup Completed",
	"Data Restore Started",
	"Data Restore Completed",

	"User enabled forced ECC option",
	"User disabled forced ECC option",
	"FW Bootup",
	"Supercap Voltage",

	"Host DMA Initialized",
	"Memory Test Started",
	"Memory Test Completed",
	"Error Injected",
	"Supercap Health",
	"Supercap Health Check Started",
	"VPD Not Programmed",
	"PCIE Link State",
	"Time Synced with Host",
	"Supercap Not Healthy for auto measurement",
	"Hard Reset by host",
	"Event Log Initialized",

	"Unknown Event"
};

char * scap_status[] = {
	"Not connected",
	"Charging",
	"Charging complete",
	"Discharging",
	"Degraded",
};

char * ssd_status[] = {
	"Not connected",
	"Erase in progress",
	"Connected",
	"Not responding",
	"IO Failing",
	"Erase pending",
	"End Of Life Reached",
};

char * memtest_status[] = {
	"Skipped",
	"Passed",
	"Failed"
};

const char * error_type[] = {
	"",
	"DMA Error",
	"SSD Write Error",
	"SSD Read Error",
};


/*
 * Wrapper for snprintf
 *
 * When the given buffer size is not large enough, snprintf returns the size
 * of the buffer required for successful print
 *
 * This wrapper returns 0 if snprintf returns size larger than given size. This
 * will void corruption.
 */
static int
wm_snprintf(char *str, u32_t len, const char *fmt, ...)
{
	u32_t count;
	va_list ap;

	va_start(ap, fmt);
	count = vsnprintf(str, len, fmt, ap);
	va_end(ap);

	if (count >= len)
		return 0;

	return count;
}


/**
 * API to get number of WAM devices discovered
 *
 * \param fd		- open file descriptor of any WAM char device
 * \param count		- reference to return the device count
 *
 * \return 0 on success, -1 in case of an error.
 */
int
wam_get_dev_count(int fd, uint32_t *count)
{
	int		ret;
	uint32_t	nr_devices = 0;

	*count = 0;

	ret = ioctl(fd, NVRAM_IOC_GET_NR_DEVICES, &nr_devices);
	if (ret == 0)
		*count = nr_devices;

	return ret;
}

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
 * \param msec		- timeout period in milliseconds
 *
 * \return 0 on success, -1 in case of timeout. 'errno' variable will
 * be set to ETIMEDOUT
 */
int
wam_wait_on_alerts(int fd, uint32_t *dev_map, uint32_t *alert_cause,
		uint32_t msec)
{
	int			ret;
	wam_alert_info_t	alert_info;

	*dev_map = 0;
	*alert_cause = 0;

	memset(&alert_info, 0, sizeof(wam_alert_info_t));

	alert_info.w_alert_wait_msec = msec;

	ret = ioctl(fd, NVRAM_IOC_WAIT_ON_ALERTS, &alert_info);
	if (ret != 0)
		return ret;

	*dev_map = alert_info.w_dev_map;
	*alert_cause = alert_info.w_alert_cause;

	return 0;
}

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
int
wam_get_dev_alert(int fd, uint32_t *alert_cause)
{
	int	ret;
	u32_t	alert = 0;

	*alert_cause = 0;

	ret = ioctl(fd, NVRAM_IOC_GET_DEV_ALERT, &alert);
	if (ret != 0)
		return ret;

	*alert_cause = alert;

	return 0;
}


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
int
wam_clear_dev_alert(int fd, uint32_t alert_cause)
{
	return ioctl(fd, NVRAM_IOC_CLEAR_DEV_ALERT, alert_cause);
}


/**
 * API to get number of events logged by the WAM device
 *
 * \param fd		- open file descriptor of the WAM device
 * \param nr_events	- reference to return the event count
 *
 * \return 0 on success or -1 in case of an error
 */
int
wam_get_nr_events(int fd, uint32_t *nr_events)
{
	event_log_t	e_log;
	int		count;

	*nr_events = 0;

	/* Get Event Log Parameters */
	if (ioctl(fd, NVRAM_IOC_GET_EVNTLOG_PTRS, &e_log) < 0) {
		return -1;
	}

	count = e_log.e_wr_ptr - e_log.e_rd_ptr;
	if (count < 0)
		count += e_log.e_max_events;

	*nr_events = count;

	return 0;
}


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
		uint32_t *nr_events)
{
	char		*addr;
	event_log_t	e_log;
	wam_event_t	*event_log;
	u32_t		rd_ptr, wr_ptr, max_e;
	int		count, tmp_count;
	u32_t		idx;

	if (events == NULL || max_events == 0)
		return -1;

	*nr_events = 0;

	/* Get Event Log Parameters */
	if (ioctl(fd, NVRAM_IOC_GET_EVNTLOG_PTRS, &e_log) < 0)
		return -1;

	rd_ptr = e_log.e_rd_ptr;
	wr_ptr = e_log.e_wr_ptr;
	max_e  = e_log.e_max_events;

	if ((rd_ptr >= max_e) || (wr_ptr >= max_e)) {
		fprintf(stderr, "Event Log Pointers out of range! rd: 0x%08X, "
				"wr: 0x%08X, max: 0x%08X\n",
				rd_ptr, wr_ptr, max_e - 1);
		return -1;
	}

	count = wr_ptr - rd_ptr;
	if (count < 0)
		count += max_e;

	if (count == 0)
		return 0;

	addr = mmap(0, NV_NVSRAM_ACCESS_SIZE, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd, NV_MMAP_NVSRAM_OFFSET);
	if (addr == MAP_FAILED)
		return -1;

	event_log = (wam_event_t *)(addr + e_log.e_nvsram_offset);

	rd_ptr++;
	if (rd_ptr >= max_e)
		rd_ptr = 0;

	if (count > max_events)
		count = max_events;

	*nr_events = count;

	idx = 0;
	if (rd_ptr + count > max_e) {
		tmp_count = max_e - rd_ptr;
		memcpy(events, event_log + rd_ptr,
				sizeof(wam_event_t) * tmp_count);
		count -= tmp_count;
		rd_ptr = 0;
		idx = tmp_count;
	}

	memcpy(events + idx, event_log + rd_ptr, sizeof(wam_event_t) * count);

	munmap(addr, NV_NVSRAM_ACCESS_SIZE);

	return 0;
}


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
int wam_clear_events(int fd, uint32_t nr_events)
{
	event_log_t	e_log;
	u32_t		rd_ptr;
	u32_t		wr_ptr;
	u32_t		max_e;
	int		count;
	int		ret;

	/* Get Event Log Parameters */
	if (ioctl(fd, NVRAM_IOC_GET_EVNTLOG_PTRS, &e_log) < 0) {
		return -1;
	}

	rd_ptr = e_log.e_rd_ptr;
	wr_ptr = e_log.e_wr_ptr;
	max_e  = e_log.e_max_events;

	if ((rd_ptr >= max_e) || (wr_ptr >= max_e)) {
		fprintf(stderr, "Event Log Pointers out of range! rd: 0x%08X, "
				"wr: 0x%08X, max: 0x%08X\n",
				rd_ptr, wr_ptr, max_e - 1);
		return -1;
	}

	count = wr_ptr - rd_ptr;
	if (count < 0)
		count += max_e;

	if (count < nr_events)
		return 0;

	rd_ptr += nr_events;
	if (rd_ptr >= max_e) {
		rd_ptr -= max_e;
	}

	ret = ioctl(fd, NVRAM_IOC_SET_EVNTLOG_RDPTR, rd_ptr);

	return ret;
}


#ifdef CONFIG_ENABLE_ASYNC_APIS

/**
 * API to read from a nvram character device
 *
 * \param fd		- open file descriptor of the nvram device
 * \param buf		- buffer to read into
 * \param count		- number of bytes to read
 * \param offset	- offset on the device to read from
 *
 * \return the number of bytes read, -1 in case of an error; errno will be set
 * EINVAL - offset + count is beyond the device range.
 * ENOMEM - not enough memory to allocate resources
 */
ssize_t
wam_read(int fd, void *buf, size_t count, off_t offset)
{
	nv_aiocb_t aiocb;
	struct iovec iov;
	int ret;

	memset(&aiocb, 0, sizeof(nv_aiocb_t));

	iov.iov_base = buf;
	iov.iov_len = count;

	aiocb.io_op = DMA_READ;
	aiocb.io_offset = offset;
	aiocb.io_size = count;
	aiocb.io_vec = &iov;
	aiocb.io_vcnt = 1;
	aiocb.io_type = NV_BARRIER_IO;
	aiocb.io_sync = 1;

	ret = ioctl(fd, NVRAM_IOC_AIO, &aiocb);
	if (ret == 0)
		return count;
	else
		return ret;
}

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
 * If the IO type is specified as NV_BARRIER_IO, the data transfer will be
 * initiated only after all pending requests are completed.
 *
 * \param fd		- open file descriptor of the nvram device
 * \param type		- normal request or barrier request
 * \param iovs		- array of iovec structure containing the data buffers
 * \param num_vec	- number of entries in the iovec
 * \param count		- number of bytes to write (<= the sub of all the iovec
 *				 entries)
 * \param offset	- starting write offset on the device
 * \param nv_id		- place holder to return a monotonically increasing 64
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
		uint64_t *nv_id)
{
	nv_aiocb_t aiocb;
	int ret;

	memset(&aiocb, 0, sizeof(nv_aiocb_t));

	aiocb.io_op = DMA_WRITE;
	aiocb.io_offset = offset;
	aiocb.io_size = count;
	aiocb.io_vec = iovs;
	aiocb.io_vcnt = num_vec;
	aiocb.io_type = type;
	aiocb.io_sync = (nv_id == NULL ? 1 : 0);

	ret = ioctl(fd, NVRAM_IOC_AIO, &aiocb);
	if (ret == 0) {
		if (nv_id)
			*nv_id = aiocb.io_id;

		return count;
	}

	return ret;
}

/**
 * API to get the completed ID of IO
 *
 * \param fd - open file descriptor of a nvram character device
 *
 * \return : ID of the request completed so far
 */
uint64_t
wam_get_completed_id(int fd)
{
	int ret;
	uint64_t cmpltd_id;

	ret = ioctl(fd, NVRAM_IOC_AIO_CMPL_ID, &cmpltd_id);
	if (ret == 0) {
		return cmpltd_id;
	}

	return -1;
}


/**
 * API to wait on IO completion.
 *
 * The caller provides the IO id to wait on.
 *
 * \param fd - open file descriptor of the wam character device
 * \param io_id - request id on which to wait on completion.
 */
int
wam_wait_on_completion(int fd, uint64_t io_id)
{
	return ioctl(fd, NVRAM_IOC_AIO_WAIT_ID, &io_id);
}

#endif		/* CONFIG_ENABLE_ASYNC_APIS */

/**
 * API to get WAM device info
 *
 * \param fd	- open file descriptor of the WAM device
 * \param info	- reference to wam_info_t structure to return the information
 *
 * \return 0 on success or -1 on error.
 */
int wam_get_info(int fd, wam_info_t *info)
{
	void *info_buf;
	int ret;

	if (info == NULL)
		return -EINVAL;

	info_buf = malloc(4096);
	if (info_buf == NULL)
		return -ENOMEM;

	memset(info_buf, 0, sizeof(wam_info_t));

	ret = ioctl(fd, NVRAM_IOC_GET_INFO, info_buf);
	if (ret != 0) {
		free(info_buf);
		return ret;
	}

	memcpy((void *)info, info_buf, sizeof(wam_info_t));

	free(info_buf);
	return 0;
}


/**
 * API to get WAM device status
 *
 * \param fd	- open file descriptor of the WAM device
 * \param info	- reference to wam_status_t structure to return the status
 * of DRAM, SSD and Supercap.
 *
 * \return 0 on success or -1 on error.
 */
int wam_get_status(int fd, wam_status_t *status)
{
	u64_t	stat = 0;
	u32_t	tmp;
	int	ret;

	ret = ioctl(fd, NVRAM_IOC_WRITE_READY, &stat);
	if (ret != 0) {
		return -1;
	}

	tmp = (u32_t)stat;
	memcpy(status, &tmp, sizeof(wam_status_t));

	return 0;
}


#if 0
static int
wam_pflash_sector_size(char *addr, unsigned int offset)
{
	int             i, ret;
	int             sect_sz;
	int             erase_region;
	int             erase_idx;
	int             cmd_sz;

	ret = -1;
	addr[CMD_REG] = CMD_CFG;
	erase_region = addr[CMD_ERASE_REGION_CNT];

	WAM_PRINT_DBG("erase_region %d\n", erase_region);

	for (i = 0; i < erase_region; i++) {
		cmd_sz = i * CMD_EBA_STRUCT_SZ;
		erase_idx = addr[CMD_EBA_INDEX1 + cmd_sz];
		sect_sz = (addr[CMD_EBA_SEC_SZ1 + cmd_sz + 2] << 8) |
			addr[CMD_EBA_SEC_SZ1 + cmd_sz];
		WAM_PRINT_DBG("addr %p, erase idx %x, sect_size %x\n",
				addr, erase_idx, sect_sz);

		if (offset < (sect_sz * FLASH_SEC_SZ_MULTI * (erase_idx + 1))) {
			ret = sect_sz * FLASH_SEC_SZ_MULTI;
			break;
		}
	}

	*addr = CMD_RESET;

	return (ret);
}
#else
static int
wam_pflash_sector_size(char *addr, unsigned int offset)
{
	// XXX: failed at offset 0x7f0000
	return (0x10000);
}
#endif

/** \fn mvf_check_status
 * Check the status of flash write or erase command
 * (Toggle Bit Algorithm specified in the spec)
 *
 * @addr - memory mapped address of flash area
 * @return 0 if successful,
 * -1 in case of error
 */
static int
mvf_check_status(char *addr)
{
        char ret, ret1;

        do {
                /* read the location twice */
                ret  = *addr;
                ret1 = *addr;

                /* Success if Bit 6 does not toggle */
                if ((ret & 0x40 ) == (ret1 & 0x40))
                        return 0;

                /* repeat if bit 5 is not set */
        } while ((*addr & 0x20) != 0x20);

        ret  = *addr;
        ret1 = *addr;

        /* check the bit 6 toggling again */
        if ((ret & 0x40 ) == (ret1 & 0x40)) {
                return 0;
        }

        /* program or erase failed. reset */
        *addr = CMD_RESET;

        return -1;
}

/** \fn wam_map_flash
 */
static int
wam_map_flash(int devfd, char **addr, unsigned int offset)
{
        char            *taddr;
        int             ret = 0;

        /* memory map flash area */
        if ((taddr = mmap (0, FLASH_WIN_SIZE, PROT_READ | PROT_WRITE,
                  MAP_SHARED, devfd, NV_MMAP_FLASH_OFFSET)) == MAP_FAILED) {
                fprintf(stderr, "mmap error\n");
                close(devfd);
                return (-1);
        }

        /* map the flash access window to 'offset' */
        if (ioctl(devfd, NVRAM_IOC_REMAP_FLASH, offset) < 0) {
                fprintf(stderr, "remap error\n");
                ret = -1;
                if (taddr != MAP_FAILED) {
                        munmap(addr, FLASH_WIN_SIZE);
                }
                close(devfd);
        }

        *addr = taddr;

	WAM_PRINT_DBG("mapping fd/addr: %d %p\n", devfd, *addr);

        return (ret);
}

static void
wam_unmap_flash(char *addr)
{
	munmap(addr, FLASH_WIN_SIZE);
}

/** \fn wam_map_file
 *
 * API to map a file
 */
int
wam_map_file(char *file, int *filefd, char **map, int *size)
{
        int             fdin;
        struct stat     statbuf;
        char            *file_buf;

        /* open input file */
        if ((fdin = open(file, O_RDONLY)) < 0) {
                fprintf(stderr, "can't open input file %s, %d\n",
                        file, fdin);
                perror ("can't open input file");
                return (-1);
        }

        if (fstat(fdin, &statbuf) < 0) {
                perror ("fstat error");
                close(fdin);
                return (-1);
        }

        *size = statbuf.st_size;

        file_buf = (char *)mmap(NULL, *size, PROT_READ, MAP_PRIVATE, fdin, 0);
        if (file_buf == MAP_FAILED) {
                fprintf(stderr, "Failed to mmap file %s\n", file);
                perror("mmap");
                return (-1);
        }

        *filefd = fdin;
	*map = file_buf;

        return (0);

}

/** \fn wam_unmap_file
 *
 * API to un-map a file
 */
void
wam_unmap_file(int filefd, char *map, int size)
{
	munmap(map, size);
	close(filefd);
}

/** \fn wam_pflash_write_byte
 * Program a byte on to the flash location
 *
 * @addr - memory mapped address of the flash location
 * @byte - byte to be written to the flash
 * @return 0 if successful, -1 otherwise
 */
int
wam_pflash_write_byte(char *addr, unsigned int offset, char byte, char *written)
{
	addr[CMD_ADDR1] = CMD_WRITE1;
	addr[CMD_ADDR2] = CMD_WRITE2;
	addr[CMD_ADDR1] = CMD_WRITE3;

	addr[offset] = byte;

	do {
		if (mvf_check_status(addr + offset)) {
			WAM_PRINT("write fail\n");
			return -1;
		}
	} while (addr[offset] != byte);

	if (written) {
		*written = addr[offset];
	}

	return (0);
}

/** \fn wam_pflash_write_sect
 *
 * API to write to Parallel flash.
 *
 * \param fd            - open file descriptor of the nvram device.
 * \param buf           - buffer to write from.
 * \param count         - number of bytes to write.
 * \param offset        - byte offset on the device to write to.
 *
 * \return the number of bytes written, -1 in case of an err; errno will be set.
 * EINVAL - offset + count is beyond the device range.
 * ENOMEM - not enough memory to allocate resources.
 */
static ssize_t
wam_pflash_write_sect(int fd, void *buf, size_t count, off_t offset)
{
	int		ret;
	char		*addr;
	int		sector_size;
	int		idx;
	char		*buf_tmp;
	char		byte;

	ret = wam_map_flash(fd, &addr, offset);
	if (ret != 0) {
		return (-1);
	}

	sector_size = wam_pflash_sector_size(addr, offset);
	assert((offset & (sector_size - 1)) == 0);
	assert(count <= sector_size);

//	WAM_PRINT("writing at offset %08x, size: %u\n",
//			(unsigned int)offset, (unsigned int)count);
	buf_tmp = (char *)buf;
	idx = 0;
	ret = 0;
	while (idx < count) {
		ret = wam_pflash_write_byte(addr, idx, buf_tmp[idx], &byte);

		if ((ret == 0) && (byte == buf_tmp[idx])) {
			idx++;
		} else {
			WAM_PRINT("write flash fail, ret %d, %x %x\n",
					ret, buf_tmp[idx], byte);
			ret = -1;
			goto exit;
		}
	}
	ret = count;

	WAM_PRINT_DBG("write success(%lu B)\n", count);

exit:
	wam_unmap_flash(addr);

	return (ret);
}

/**
 * Routine to read from flash in byte mode
 *
 * Using standard memcpy does not work.
 */
static void
flash_memcpy(char *dst, char *src, u32_t size)
{
	u32_t i;

	for (i = 0; i < size; i++) {
		dst[i] = src[i];
	}
}

/** \fn wam_pflash_read
 *
 * API to read from Parallel flash.
 *
 * \param fd            - open file descriptor of the wam character device.
 * \param buf           - buffer to read into.
 * \param size          - number of bytes to read.
 * \param offset        - byte offset on the device to read from.
 *
 * \return the number of bytes read, -1 in case of an error; errno will be set.
 * EINVAL - offset + count is beyond the device range.
 * ENOMEM - not enough memory to allocate resources.

 */
ssize_t
wam_pflash_read(int fd, void *buf, size_t size, off_t offset)
{
	u32_t   sector;
	u32_t   sec_off;
	u32_t   buf_off;
	u32_t   count;
	u32_t   bytes_left;
	char	*map;
	int     ret;
	int	sector_size;

	ret = wam_map_flash(fd, &map, offset);
	if (ret != 0) {
		return (-1);
	}

	sector_size = wam_pflash_sector_size(map, offset);
	assert(sector_size == FLASH_WIN_SIZE);
	if (offset & (sector_size - 1)) {
		wam_unmap_flash(map);
		return (-1);
	}


	sector = offset & ~(sector_size - 1);
	sec_off = offset & (sector_size - 1);

	count = sector_size - sec_off;
	if (size < count)
		count = size;

	bytes_left = size;
	buf_off = 0;

	do {
		ret = ioctl(fd, NVRAM_IOC_REMAP_FLASH, sector);
		if (ret != 0) {
			goto end;
		}

		flash_memcpy(buf + buf_off, map + sec_off, count);

		sec_off = 0;
		sector += sector_size;
		buf_off += count;
		bytes_left -= count;

		count = MIN(bytes_left, sector_size);

	} while (bytes_left);

end:
	wam_unmap_flash(map);
	return (size - bytes_left);
}

/** \fn wam_pflash_erase_sector
 *
 * API to erase a sector in Parallel flash.
 * Sector offset must be align to sector size.
 *
 * \param fd            - open file descriptor of the nvram device.
 *
 * \return : 0 if successful, -1 in case of error.
 */
static int
wam_pflash_erase_sector(int fd, off_t offset)
{
	int		ret;
	char		*addr;
	int		sector_size;
	char		value = FLG_ERASE_DONE;

	ret = wam_map_flash(fd, &addr, offset);
	if (ret != 0) {
		return (-1);
	}

	ret = 0;
	sector_size = wam_pflash_sector_size(addr, offset);
	if (offset & (sector_size - 1)) {
		ret = -1;
		goto exit;
	}

	addr[CMD_ADDR1] = CMD_ERASE1;
	addr[CMD_ADDR2] = CMD_ERASE2;
	addr[CMD_ADDR1] = CMD_ERASE3;
	addr[CMD_ADDR1] = CMD_ERASE4;
	addr[CMD_ADDR2] = CMD_ERASE5;

	*addr = CMD_ERASE_CODE;

	do {
		if (mvf_check_status(addr)) {
			WAM_PRINT("erase fail\n");
			ret = -1;
			goto exit;
		}
	} while (*addr != value);

exit:
	wam_unmap_flash(addr);

	return (ret);
}

/** \fn wam_pflash_program
 *
 * API to program a buffer to parallel flash at the given offset
 *
 * \param fd            - open file descriptor of the wam character device.
 * \param bufr		- buffer to be programmed on to flash
 * \param size		- buffer size
 * \param offset        - sector aligned offset
 * \return 0 if successful, -1 in case of error.
 */
size_t
wam_pflash_program(int fd, void *bufr, size_t size, off_t offset)
{
	int     ret;
	u8_t    *buffer = (u8_t *)bufr;
	u32_t   bytes_left;
	u32_t   count;
	char	*addr;
        int	sector_size;
	u8_t	*rdbuf;

	ret = wam_map_flash(fd, &addr, offset);
	if (ret != 0) {
		return (-1);
	}

	sector_size = wam_pflash_sector_size(addr, offset);
	assert(sector_size == FLASH_WIN_SIZE);

	rdbuf = malloc(sector_size);
	if (rdbuf == NULL) {
		return -1;
	}

	if (offset & (sector_size - 1)) {
		wam_unmap_flash(addr);
		return (-1);
	}
	wam_unmap_flash(addr);

	bytes_left = size;
	while (bytes_left) {
		count = MIN(bytes_left, sector_size);
		ret = wam_pflash_erase_sector(fd, offset);
		if (ret != 0) {
			WAM_PRINT_DBG("erase sector error, offset 0x%llx\n",
					(unsigned long long)offset);
			ret = -1;
			goto exit;
		}

		ret = wam_pflash_write_sect(fd, buffer, count, offset);
		if (ret != (int)count) {
			WAM_PRINT_DBG("prog sector error 0x%x, offset 0x%llx\n",
					count, (unsigned long long)offset);
			ret = -1;
			goto exit;
		}

		wam_pflash_read(fd, rdbuf, count, offset);
		if (memcmp(rdbuf, buffer, count) != 0) {
			printf("program flash: read verify failed\n");
			ret = -1;
			goto exit;
		}

		buffer += count;
		bytes_left -= count;
		offset += count;
	}
	ret = 0;

exit:
	free(rdbuf);
	return (ret);
}

/**
 * API to program a file to parallel flash at the given offset
 *
 * \param fd            - open file descriptor of the wam character device.
 * \param file          - file to be programmed on to flash
 * \param offset        - sector aligned offset
 * \return 0 if successful, -1 in case of error.
 */
int wam_pflash_program_file(int fd, char *file, off_t offset)
{
	int     filefd;
	int     ret;
	int	size;
	char    *buffer;

	ret = wam_map_file(file, &filefd, &buffer, &size);
	if (ret != 0)
		return -1;

	ret = wam_pflash_program(fd, buffer, size, offset);

	wam_unmap_file(filefd, buffer, size);

	return ((ret == (int)size) ? 0 : -1);
}

/* ***************************** update package	**************************** */

typedef struct {
        unsigned int            fw_addr;
        unsigned int            fw_ctrl_sta;
        unsigned int            fw_size;
        unsigned int            fw_load_addr;
        unsigned char           fw_ver[FW_HDR_VER_LEN];
        unsigned int            fw_chksum;
        unsigned int            rsvd[NR_FW_RSVD];
} FW_ENT;

typedef struct {
        unsigned int            rec_start_flag;
        FW_ENT                  rec_fw_ent[2];
        unsigned int            rec_end_flag;
        char                    rec_uboot_ver[FW_HDR_VER_LEN];
        char                    rec_pkg_ver[PKG_HDR_VER_LEN];
} FW_REC;


static int
pkg_hdr_sanity_check(pkg_hdr_t *pkg)
{
	if (pkg->pkg_magic != PKG_HEADER_MAGIC) {
		fprintf(stderr, "Cannot recognize the file format\n");
		return (-1);
	}

	if (pkg->pkg_major != PKG_VER_MAJOR) {
		fprintf(stderr, "Package major version not match.\n");
		return (-1);
	}

	if (pkg->pkg_count != PKG_FILE_COUNT) {
		fprintf(stderr, "images count not matched %d, %d\n",
			pkg->pkg_count, PKG_FILE_COUNT);
		return (-1);
	}

	/* XXX: validate checksum */

	return (0);
}

static fw_hdr_t *
pkg_get_image(pkg_hdr_t *pkg, int index)
{
	unsigned int	offset;

	assert((index >= 0) && (index < pkg->pkg_count));
	offset = pkg->pkg_offset_list[index].pkg_offset;
	pkg++;
	return ((fw_hdr_t *)((char *)pkg + offset));
}

static int
fw_hdr_sanity_check(fw_hdr_t *hdr)
{
	fw_hdr_gen_t		*gen_hdr;

	gen_hdr = &hdr->hdr_gen;
	if (gen_hdr->hdr_magic != HEADER_MAGIC ||
		gen_hdr->hdr_magic2 != HEADER_MAGIC ||
		hdr->hdr_magic3 != HEADER_MAGIC) {
		fprintf(stderr, "image magic mismatch\n");
		return (-1);
	}

	// XXX: validate checksum

	return (0);
}

static char *
fw_hdr_get_file(fw_hdr_t *file_hdr)
{
	file_hdr++;
	return ((char *)file_hdr);
}

static char *
fw_hdr_tostr(int hdr_type)
{
	switch (hdr_type) {
	case FTYPE_BOOTLOADER:
		return ("bootloader");
	case FTYPE_DDR_CONFIG:
		return ("ddr_config");
	case FTYPE_EEPROM:
		return ("eeprom");
	case FTYPE_FIRMWARE:
		return ("firmware");
	case FTYPE_SSD_FW:
		return ("ssd_fw");
	default:
		return ("file_type_error");
	}
}

static void
wam_dump_pkg_header(pkg_hdr_t *hdr)
{
        int             i;
        pkg_offset_t    *offset;

        fprintf(stderr, "Header:\n");
        fprintf(stderr, "\tmagic    : %08X\n", hdr->pkg_magic);
        fprintf(stderr, "\tver      : %s\n", hdr->pkg_ver);
        fprintf(stderr, "\tbuild ts : %s\n", hdr->pkg_build_ts);
        fprintf(stderr, "\tcount    : %u\n", hdr->pkg_count);
        fprintf(stderr, "\tcksm     : %08X\n", hdr->pkg_chksum);

        offset = hdr->pkg_offset_list;

        fprintf(stderr, "\n");
        for (i = 0; i < PKG_FILE_COUNT; i++, offset++) {
                fprintf(stderr, "\toffset 0x%x, length %u\n",
                        offset->pkg_offset, offset->pkg_length);
        }
}

static void
wam_dump_fw_header(fw_hdr_t *hdr)
{
        fw_hdr_gen_t *p;

        p = &hdr->hdr_gen;
        fprintf(stderr, "Header:\n");
        fprintf(stderr, "\tmagic    : %08X\n", p->hdr_magic);
        fprintf(stderr, "\tformat   : %u\n", p->hdr_file_fmt_ver);
        fprintf(stderr, "\tver      : %s\n", p->hdr_ver);
        fprintf(stderr, "\tbuild ts : %s\n", p->hdr_build_ts);
        fprintf(stderr, "\tsize     : %u\n", p->hdr_size);
        fprintf(stderr, "\ttype     : %u\n", p->hdr_type);
        fprintf(stderr, "\tint      : %u\n", p->hdr_interruptible);
        fprintf(stderr, "\tcksm     : %08X\n", p->hdr_chksum);
        fprintf(stderr, "\tladd     : %08X\n", p->hdr_ld_addr);
        fprintf(stderr, "\tmagic2   : %08X\n", p->hdr_magic2);
        fprintf(stderr, "\tmagic3   : %08X\n", hdr->hdr_magic3);
}

static int
read_fw_rec(int fd, FW_REC *fw, unsigned int start)
{
	int		rd;

	rd = wam_pflash_read(fd, (void *)fw, sizeof (FW_REC), start);

	if (rd != sizeof (FW_REC)) {
		return (-1);
	}

	return (0);
}

static int
check_fw_rec(FW_REC *fw)
{
	if ((fw->rec_start_flag != FW_START) ||
		(fw->rec_end_flag != FW_END)) {

		return -1;
	} else {
		return 0;
	}
}

static int
get_fw_rec_state(int fd, FW_REC *fw_rec, int rec_cnt)
{
        int ret, ret1;

	assert(rec_cnt == WAM_FW_REC_MAX);

	/* read record 0 */
	ret = read_fw_rec(fd, &fw_rec[0], FW_REC_0_ADDR);
	if (ret) {
		fprintf(stderr, "read fw rec 0 wrong\n");
		return FW_STATE_READ_FW_REC0_FAILED;
	}

	/* read record 1 */
	ret = read_fw_rec(fd, &fw_rec[1], FW_REC_1_ADDR);
	if (ret) {
		fprintf(stderr, "read fw rec 1 wrong\n");
		return FW_STATE_READ_FW_REC1_FAILED;
	}

	/* if record 0 is complete */
	ret = check_fw_rec(&fw_rec[0]);

	/* if record 1 is complete */
	ret1 = check_fw_rec(&fw_rec[1]);

	if ((ret == 0) && (ret1 == 0)) {
		if (!memcmp(&fw_rec[0], &fw_rec[1], sizeof (FW_REC))) {
			return FW_STATE_NORMAL;
		} else {
			return FW_STATE_MISMATCH;
		}
	} else if ((ret == 0) && (ret1 != 0)) {
		return FW_STATE_BACKUP_NOT_EXIST;
	} else if ((ret != 0) && (ret1 == 0)) {
		return FW_STATE_ROOT_NOT_EXIST;
	} else {
		return FW_STATE_FRESH;
	}

}

#if MV_FLASH_DBG
static void
dump_fw_rec(FW_REC *fw)
{
	FW_ENT		*ent;

	WAM_PRINT_DBG("start flag - 0x%x\n", fw->rec_start_flag);

	ent = &fw->rec_fw_ent[0];
	WAM_PRINT_DBG("fw_addr - 0x%x\n", ent->fw_addr);
	WAM_PRINT_DBG("ctrl_sta - 0x%x\n", ent->fw_ctrl_sta);
	WAM_PRINT_DBG("size - 0x%x\n", ent->fw_size);
	WAM_PRINT_DBG("load addr - 0x%x\n", ent->fw_load_addr);
	WAM_PRINT_DBG("ver - 0x%s\n", ent->fw_ver);
	WAM_PRINT_DBG("checksum- 0x%x\n", ent->fw_chksum);

	ent = &fw->rec_fw_ent[1];
	WAM_PRINT_DBG("fw_addr - 0x%x\n", ent->fw_addr);
	WAM_PRINT_DBG("ctrl_sta - 0x%x\n", ent->fw_ctrl_sta);
	WAM_PRINT_DBG("size - 0x%x\n", ent->fw_size);
	WAM_PRINT_DBG("load addr - 0x%x\n", ent->fw_load_addr);
	WAM_PRINT_DBG("ver - 0x%s\n", ent->fw_ver);
	WAM_PRINT_DBG("checksum- 0x%x\n", ent->fw_chksum);


	WAM_PRINT_DBG("end flag - 0x%x\n", fw->rec_end_flag);
	WAM_PRINT_DBG("bootloader ver - 0x%s\n", fw->rec_uboot_ver);
	WAM_PRINT_DBG("pkg ver - 0x%s\n", fw->rec_pkg_ver);
}

#else

#define	dump_fw_rec(x)

#endif

static int
update_fw_rec(int devfd, FW_REC *fw, unsigned int start)
{
	int ret = 0;
	unsigned int byte_count;

	byte_count = (int) sizeof (FW_REC);

	/* dump_fw_rec(fw); */

	if (wam_pflash_program(devfd, (void *)fw, byte_count, start) != 0) {
		fprintf(stderr, "Write flash failed\n");
		ret = -1;
	}

	return (ret);
}

static void
build_default_fw_rec(FW_REC *fw_rec, int cnt)
{
	assert(cnt == WAM_FW_REC_MAX);
	memset(&fw_rec[0], 0, sizeof (FW_REC));

	fw_rec[0].rec_start_flag = FW_START;
	fw_rec[0].rec_end_flag   = FW_END;
	fw_rec[0].rec_fw_ent[0].fw_addr = FW_DATA_0_ADDR;
	fw_rec[0].rec_fw_ent[1].fw_addr = FW_DATA_1_ADDR;
	fw_rec[0].rec_fw_ent[0].fw_ctrl_sta = F_INV;
	fw_rec[0].rec_fw_ent[1].fw_ctrl_sta = F_INV;

	memcpy(&fw_rec[1], &fw_rec[0], sizeof (FW_REC));
}

static int
get_free_fw_ent_id(FW_REC *fw)
{
	unsigned int val0;

	val0 = fw->rec_fw_ent[0].fw_ctrl_sta & F_MASK;

	if(val0 == F_INV)
		return 0;
	else
		return 1;
}

static unsigned int fw_addr[] = {
        FW_DATA_0_ADDR,
        FW_DATA_1_ADDR,
        FW_DATA_2_ADDR
};

static unsigned int
get_free_fw_addr(FW_REC *fw)
{
	int i;
	unsigned int val0, val1;

	val0 = fw->rec_fw_ent[0].fw_ctrl_sta & F_MASK;
	val1 = fw->rec_fw_ent[1].fw_ctrl_sta & F_MASK;

	/* both entries are invalid */
	if(val0 == F_INV)
		return fw_addr[0];

	/* entry 0 is valid, entry 1 is invalid */
	if(val1 == F_INV)
		return fw_addr[1];

	/* both enties are valid, use third one */
	for (i = 0; i < 3; i++) {
		if( (fw_addr[i] != fw->rec_fw_ent[0].fw_addr )
				&& (fw_addr[i] != fw->rec_fw_ent[1].fw_addr))
			return fw_addr[i];
	}

	return 0;
}

static void
switch_fw_entry(FW_REC *fw_rec, int cnt)
{
	FW_ENT tmp;

	assert(cnt == WAM_FW_REC_MAX);
	memcpy(&tmp, &fw_rec[0].rec_fw_ent[1], sizeof (FW_ENT));
	memcpy(&fw_rec[0].rec_fw_ent[1], &fw_rec[0].rec_fw_ent[0],
		sizeof (FW_ENT));
	memcpy(&fw_rec[0].rec_fw_ent[0], &tmp, sizeof (FW_ENT));

	memcpy(&fw_rec[1], &fw_rec[0], sizeof (FW_REC));
}

static int
normal_fw_update(int fd, char *image, int fsize, FW_REC *fw_rec, int cnt)
{
	unsigned int write_addr;
	int fw_ent_id, ret = 0;
	fw_hdr_t *fwh;
	fw_hdr_gen_t *fw_hdr;

	assert(cnt == WAM_FW_REC_MAX);
	fwh = (fw_hdr_t *)image;
	fw_hdr = &fwh->hdr_gen;
	image += sizeof (fw_hdr_t);
	fsize -= sizeof (fw_hdr_t);

	fw_ent_id = get_free_fw_ent_id(&fw_rec[0]);
//	WAM_PRINT("fw_ent_id = %d\n", fw_ent_id);

	write_addr = get_free_fw_addr(&fw_rec[0]);
//	WAM_PRINT("write_addr = %x\n", write_addr);
	if (write_addr == 0) {
		ret = -1;
		goto end;
	}

	ret = wam_pflash_program(fd, image, fsize, write_addr);
	if (ret != 0) {
		fprintf(stderr, "write new firmware failed\n");
		goto end;
	}

	fw_rec[0].rec_fw_ent[fw_ent_id].fw_size      = fsize;
	fw_rec[0].rec_fw_ent[fw_ent_id].fw_addr      = write_addr;
	memset(fw_rec[0].rec_fw_ent[fw_ent_id].fw_ver, 0, FW_HDR_VER_LEN);
	memcpy(fw_rec[0].rec_fw_ent[fw_ent_id].fw_ver, fw_hdr->hdr_ver,
			FW_HDR_VER_LEN);
	fw_rec[0].rec_fw_ent[fw_ent_id].fw_ctrl_sta  = F_VALID;
	fw_rec[0].rec_fw_ent[fw_ent_id].fw_load_addr = fw_hdr->hdr_ld_addr;
	fw_rec[0].rec_fw_ent[fw_ent_id].fw_chksum    = fw_hdr->hdr_chksum;

	if (fw_ent_id == 1) {
		switch_fw_entry(fw_rec, 2);
	} else {
		memcpy(&fw_rec[1], &fw_rec[0], sizeof (FW_REC));
	}


	ret = update_fw_rec(fd, &fw_rec[1], FW_REC_1_ADDR);
	if (ret != 0) {
		fprintf(stderr, "update fw record 1 failed\n");
		goto end;
	}


	ret = update_fw_rec(fd, &fw_rec[0], FW_REC_0_ADDR);
	if (ret != 0) {
		fprintf(stderr, "update fw record 0 failed\n");
		goto end;
	}

end:
	return ret;
}

static int
wam_update_fw_image(int fd, fw_hdr_t *file_hdr, wam_info_t *info)
{
	int		state, ret = 0;
	fw_hdr_gen_t    *gen_hdr;
	FW_REC		fw_rec[2];
	int		fsize;

	if (fw_hdr_sanity_check(file_hdr) != 0) {
		fprintf(stderr, "Error in checking package checksum\n");
		return (-1);
	}

	gen_hdr   = &file_hdr->hdr_gen;
	fsize = gen_hdr->hdr_size + sizeof (*file_hdr);

	if (info) {
		if (strcmp(gen_hdr->hdr_ver, info->w_fw_ver) == 0)
			return (1);

		printf("Updating firmware to %s, current running version: %s\n",
				gen_hdr->hdr_ver, info->w_fw_ver);
	}

	state = get_fw_rec_state(fd, fw_rec, WAM_FW_REC_MAX);

	switch (state) {
	case FW_STATE_NORMAL:
		/* both fw records are good, normal update */
//		printf("Regular Update\n");
		ret = normal_fw_update(fd, (char *) file_hdr, fsize,
				fw_rec, WAM_FW_REC_MAX);
		break;

	case FW_STATE_MISMATCH:
		/*
		 * both record exist, but contents of them differ
		 * update record 0 with record 1 before update fw
		 */

	case FW_STATE_ROOT_NOT_EXIST:
		/*
		 * only backup fw record is good,
		 * update record 0 with record 1 before update fw
		 */
//		printf("Recover Main Record\n");
		memcpy(&fw_rec[0], &fw_rec[1], sizeof (FW_REC));
		ret = update_fw_rec(fd, &fw_rec[0], FW_REC_0_ADDR);
		if(ret != 0) {
			fprintf(stderr, "recover fw record 0 failed\n");
			break;
		}

//		printf("Update Firmware\n");
		ret = normal_fw_update(fd, (char *) file_hdr, fsize,
				fw_rec, WAM_FW_REC_MAX);
		break;

	case FW_STATE_FRESH:
		/* no fw records, start from scratch */
//		printf("Build Records\n");
		build_default_fw_rec(fw_rec, 2);

//		printf("Update Firmware\n");
		ret = normal_fw_update(fd, (char *) file_hdr, fsize,
				fw_rec, WAM_FW_REC_MAX);
		break;

	case FW_STATE_BACKUP_NOT_EXIST:
		/*
		 * only normal fw record is good,
		 * recover backup fw record before update fw
		 */
//		printf("Recover Backup Record\n");
		memcpy(&fw_rec[1], &fw_rec[0], sizeof (FW_REC));
		ret = update_fw_rec(fd, &fw_rec[1], FW_REC_1_ADDR);
		if(ret != 0) {
			fprintf(stderr, "recover fw record 1 failed\n");
			break;
		}

//		printf("Update Firmware\n");
		ret = normal_fw_update(fd, (char *) file_hdr, fsize,
				fw_rec, WAM_FW_REC_MAX);
		break;

	case FW_STATE_READ_FW_REC0_FAILED:
	case FW_STATE_READ_FW_REC1_FAILED:
	default:
		fprintf(stderr, "Read Records Failed\n");
		ret = -1;
		break;
	}

	if (ret != 0) {
		printf("Firmware update failed!\n");
		ret = -1;
	} else {
		printf("Firmware update successful!\n");
	}

	return ret;
}

/** \fn wam_update_firmware
 *
 * API to update the firmware in wbin format.
 */
int
wam_update_firmware(int fd, char *filename, int force)
{
	int             fdin;
	int             ret;
	char            *buf;
	int             fsize;

	ret = wam_map_file(filename, &fdin, &buf, &fsize);
	if (ret != 0) {
		return (-1);
	}

	ret = wam_update_fw_image(fd, (fw_hdr_t *)buf, NULL);
	wam_unmap_file(fdin, buf, fsize);

	return (ret);
}


static int
wam_update_uboot_image(int fd, fw_hdr_t *file_hdr, wam_info_t *info)
{
	int             ret = 0;
	fw_hdr_gen_t    *gen_hdr;
	void            *image_ptr;
	FW_REC		fw_rec[2];


	assert(info == NULL);

	if (fw_hdr_sanity_check(file_hdr) != 0) {
		fprintf(stderr, "Error in checking package checksum\n");
		return (-1);
	}

	get_fw_rec_state(fd, fw_rec, WAM_FW_REC_MAX);
	gen_hdr = &file_hdr->hdr_gen;
	if (strcmp(gen_hdr->hdr_ver, fw_rec[0].rec_uboot_ver) == 0) {
//		printf("Boot Loader version is the same, skipping upgrade.\n");

		return (1);
	}

	gen_hdr = &file_hdr->hdr_gen;
	image_ptr = fw_hdr_get_file(file_hdr);

	printf("Updating bootloader to %s\n", gen_hdr->hdr_ver);
	ret = wam_pflash_program(fd, image_ptr, gen_hdr->hdr_size,
					gen_hdr->hdr_ld_addr);
	if (ret != 0) {
		fprintf(stderr, "program boot loader image failed, ret %d\n",
				ret);
		return (-1);
	}

	memset(fw_rec[0].rec_uboot_ver, 0, FW_HDR_VER_LEN);
	memcpy(fw_rec[0].rec_uboot_ver, gen_hdr->hdr_ver, FW_HDR_VER_LEN);
	memset(fw_rec[1].rec_uboot_ver, 0, FW_HDR_VER_LEN);
	memcpy(fw_rec[1].rec_uboot_ver, gen_hdr->hdr_ver, FW_HDR_VER_LEN);

	ret = update_fw_rec(fd, &fw_rec[1], FW_REC_1_ADDR);
	if (ret != 0) {
		fprintf(stderr, "update bootloader version failed, ret %d\n",
				ret);
		return (-1);
	}
	ret = update_fw_rec(fd, &fw_rec[0], FW_REC_0_ADDR);
	if (ret != 0) {
		fprintf(stderr, "update boot loader version failed, ret %d\n",
				ret);
		return (-1);
	}

	return (0);
}

/** \fn wam_update_uboot
 *
 * API to update uboot with header information
 */
int
wam_update_uboot(int fd, char *filename, int force)
{
	int		fdin;
	int		ret;
	char		*buf;
	int		fsize;

	ret = wam_map_file(filename, &fdin, &buf, &fsize);
	if (ret != 0) {
		return (-1);
	}

	ret = wam_update_uboot_image(fd, (fw_hdr_t *)buf, NULL);
	wam_unmap_file(fdin, buf, fsize);

	return (ret);
}

static int
wam_update_ddr_cfg_image(int fd, fw_hdr_t *file_hdr, wam_info_t *info)
{
	int		ret;
	fw_hdr_gen_t    *gen_hdr;
	void            *image_ptr;

	if (fw_hdr_sanity_check(file_hdr) != 0) {
		fprintf(stderr, "Error in checking package checksum\n");
		return (-1);
	}

	gen_hdr   = &file_hdr->hdr_gen;
	image_ptr = fw_hdr_get_file(file_hdr);

	if (info &&
		(strtoul(gen_hdr->hdr_ver, NULL, 10) == info->w_ddr_cfg_ver)) {
//		WAM_PRINT("ddr cfg version is the same, skipping upgrade.\n");
		return (1);
	}

	WAM_PRINT("Updating ddr config to %s\n", gen_hdr->hdr_ver);

	ret = wam_pflash_program(fd, image_ptr, gen_hdr->hdr_size,
			gen_hdr->hdr_ld_addr);

	if (ret != 0) {
	    ret = -1;
	}

	return (ret);
}

/** \fn wam_update_ddr_cfg
 *
 * API to update DDR configuration in wbin format.
 */
int
wam_update_ddr_cfg(int fd, char *filename, int force)
{
	int		fdin;
	int		ret;
	char		*buf;
	int		fsize;

	ret = wam_map_file(filename, &fdin, &buf, &fsize);
	if (ret != 0) {
		return (-1);
	}

	ret = wam_update_ddr_cfg_image(fd, (fw_hdr_t *)buf, NULL);
	wam_unmap_file(fdin, buf, fsize);

	return (ret);

}

static int
wam_update_eeprom_image(int fd, fw_hdr_t *file_hdr, wam_info_t *info)
{
	int             ret;
	fw_hdr_gen_t    *gen_hdr;
	void            *image_ptr;
	file_info_t     file_info;

	if (fw_hdr_sanity_check(file_hdr) != 0) {
		fprintf(stderr, "Error in checking package checksum\n");
		return (-1);
	}

	gen_hdr   = &file_hdr->hdr_gen;
	image_ptr = fw_hdr_get_file(file_hdr);

	if (info &&
		(strtoul(gen_hdr->hdr_ver, NULL, 10) == info->w_eeprom_ver)) {
//		printf("eeprom version is the same, skipping upgrade.\n");
		return (1);
	}

	printf("Updating eeprom to %s\n", gen_hdr->hdr_ver);


	file_info.file_buf = image_ptr;
	file_info.file_size = gen_hdr->hdr_size;

	ret = ioctl(fd, NVRAM_IOC_UPDATE_EEPROM, &file_info);
	if (ret != 0) {
		fprintf(stderr, "EEPROM Updated failed! err: %d\n", ret);
		perror("ioctl");
		ret = -1;
	}

	return (ret);
}

/** \fn wam_update_eeprom
 *
 * API to update EEPROM in wbin format.
 */
int
wam_update_eeprom(int fd, char *filename, int force)
{
	int             fdin;
	int             ret;
	char            *buf;
	int             fsize;

	ret = wam_map_file(filename, &fdin, &buf, &fsize);
	if (ret != 0) {
		return (-1);
	}

	ret = wam_update_eeprom_image(fd, (fw_hdr_t *)buf, NULL);
	wam_unmap_file(fdin, buf, fsize);

	return (ret);
}

static int
wam_update_ssdfw_image(int fd, fw_hdr_t *file_hdr, wam_info_t *info)
{
	int             ret;
	fw_hdr_gen_t    *gen_hdr;
	void            *image_ptr;
	file_info_t     file_info;

	if (fw_hdr_sanity_check(file_hdr) != 0) {
		fprintf(stderr, "Error in checking package checksum\n");
		return (-1);
	}

	gen_hdr   = &file_hdr->hdr_gen;
	image_ptr = fw_hdr_get_file(file_hdr);

	if (info && (strcmp(gen_hdr->hdr_ver, info->w_ssd_fw_ver) == 0)) {
//		printf("SSD firmware version is the same, skipping upgrade.\n");

		return (1);
	}

	printf("Updating ssd fw to %s\n", gen_hdr->hdr_ver);


	file_info.file_buf = image_ptr;
	file_info.file_size = gen_hdr->hdr_size;

	ret = ioctl(fd, NVRAM_IOC_SSD_FW_UPDATE, &file_info);
	if (ret != 0) {
		fprintf(stderr, "Update SSD FW ioctl failed: %d\n", ret);
		perror("ioctl");
		ret = -1;
	}

	return (ret);
}

/** \fn wam_update_ssdfw
 *
 * API to update ssd firmware in wbin format.
 */
int
wam_update_ssdfw(int fd, char *filename, int force)
{
        int             fdin;
        int             ret;
        char            *buf;
        int             fsize;

        ret = wam_map_file(filename, &fdin, &buf, &fsize);
        if (ret != 0) {
                return (-1);
        }

        ret = wam_update_ssdfw_image(fd, (fw_hdr_t *)buf, NULL);
        wam_unmap_file(fdin, buf, fsize);

        return (ret);
}

static int
wam_pkg_verify_interruptible(pkg_hdr_t *pkg, int force)
{
	int             i;
	fw_hdr_t        *file_hdr;
	fw_hdr_gen_t    *gen_hdr;
	int             not_interruptible;
	char            inpc[16];

	for (i = 0; i < PKG_FILE_COUNT; i++) {
		file_hdr = pkg_get_image(pkg, i);
		/* wam_dump_fw_header(file_hdr); */
		if (fw_hdr_sanity_check(file_hdr) != 0) {
			fprintf(stderr, "Error in checking package checksum\n");
			return (-1);
		}

		gen_hdr = &file_hdr->hdr_gen;
		if (!gen_hdr->hdr_interruptible) {
			not_interruptible = 1;
		}
	}

	if (not_interruptible && !force) {
		do {
			printf("Warning: This package update is not "
					"interruptible.  Continue? (y/n): ");
			scanf("%15s", inpc);
		} while (inpc[0] != 'y' && inpc[0] != 'n');


		if (inpc[0] == 'n') {
			return (-1);
		}
	}

	return (0);
}

static int
wam_update_package_rec(int fd, pkg_hdr_t *pkg)
{
	int             ret = 0;
	FW_REC		fw_rec[WAM_FW_REC_MAX];

	/* Already have firmware record? */
	get_fw_rec_state(fd, fw_rec, WAM_FW_REC_MAX);

	memset(fw_rec[0].rec_pkg_ver, 0, PKG_HDR_VER_LEN);
	memcpy(fw_rec[0].rec_pkg_ver, pkg->pkg_ver, PKG_HDR_VER_LEN);
	memset(fw_rec[1].rec_pkg_ver, 0, PKG_HDR_VER_LEN);
	memcpy(fw_rec[1].rec_pkg_ver, pkg->pkg_ver, PKG_HDR_VER_LEN);

	ret = update_fw_rec(fd, &fw_rec[1], FW_REC_1_ADDR);
	if (ret != 0) {
		fprintf(stderr, "update package version failed, ret %d\n", ret);
		return (ret);
	}
	ret = update_fw_rec(fd, &fw_rec[0], FW_REC_0_ADDR);
	if (ret != 0) {
		fprintf(stderr, "update package version failed, ret %d\n", ret);
		return (ret);
	}
	WAM_PRINT_DBG("package version updated to %s\n", pkg->pkg_ver);

	return (0);
}

#define VPD_VERSION             1
#define VPD_PRODUCT_ID_STRING   "Marvell Write Accelerator Module - "\
                                        "PCIe NVRAM"
#define VPD_MANUFACT_NAME       "MARVELL SEMICONDUCTORS INC"
#define VPD_WWN_SIZE            8
#define VPD_PN_SIZE             19
#define VPD_SN_SIZE             18
#define VPD_EC_SIZE             2
#define VPD_MN_SIZE             26
#define VPD_RV_SIZE             1
#define VPD_BN_SIZE		8
#define VPD_KEY_SIZE            2

#define VPD_SIZE                0x94
#define VPD_SUBSYS_ID           0x8180
#define VPD_SUBSYS_VEN_ID       0x11AB
#define VPD_PRODUCT_ID_TAG      0x82
#define VPD_PRODUCT_ID_SIZE     48
#define VPD_RO_TAG              0x90
#define VPD_RO_SIZE             0x43
#define VPD_PN_KEY              "PN"
#define VPD_EC_KEY              "EC"
#define VPD_SN_KEY              "SN"
#define VPD_MN_KEY              "MN"
#define VPD_RV_KEY              "RV"
#define VPD_BN_KEY		"BN"
#define VPD_END_TAG             0x78



typedef struct kimchee_vpd {
        u8_t    v_version;
        u8_t    v_length[2];
        u8_t    v_wwn[VPD_WWN_SIZE];
        u8_t    v_subsys_id[2];
        u8_t    v_subsys_ven_id[2];
        u8_t    v_id_tag;
        u8_t    v_id_length[2];
        u8_t    v_id[VPD_PRODUCT_ID_SIZE];
        u8_t    v_ro_tag;
        u8_t    v_ro_length[2];
        u8_t    v_pn_key[VPD_KEY_SIZE];
        u8_t    v_pn_length;
        u8_t    v_pn[VPD_PN_SIZE];
        u8_t    v_ec_key[VPD_KEY_SIZE];
        u8_t    v_ec_length;
        u8_t    v_ec[VPD_EC_SIZE];
        u8_t    v_sn_key[VPD_KEY_SIZE];
        u8_t    v_sn_length;
        u8_t    v_sn[VPD_SN_SIZE];
        u8_t    v_mn_key[VPD_KEY_SIZE];
        u8_t    v_mn_length;
        u8_t    v_mn[VPD_MN_SIZE];
        u8_t    v_rv_key[VPD_KEY_SIZE];
        u8_t    v_rv_length;
        u8_t    v_checksum;
        u8_t    v_end_tag;
} kimchee_vpd_t;


typedef enum {
        VPD_FIELD_SN            = 1,
        VPD_FIELD_PN            = 2,
        VPD_FIELD_WWN           = 3,

        VPD_FIELD_INVALID       = -1,
} vpd_field_t;

/* VPD offset and size */
#define VPD_OFF			(0x6d0000)
#define VPD_V2_OFF		WAM_VPD_FLASH_OFFSET
#define WAM_SN_OFF		WAM_SN_FLASH_OFFSET

#define VPD_PN_WAM_1_REV4	("HA1CA22650LH-RA1v04")
#define VPD_PN_WAM_1_REV3	("HA1CA22640LH-RA1v01")
#define VPD_PN_WAM1P_REV3	("HA1CA22640FH-RA1v03")
#define VPD_PN_WAM1P_REV4	("HA1CA22640FH-RA1v3a")
#define VPD_PN_WAM1P_REV1	("HA1CA22640FH-RA1v01")
#define VPD_PN_WAM1P_REV2	("HA1CA22640FH-RA1v02")
#define VPD_EC_WAM1_REV3	("03")
#define VPD_EC_WAM1_REV4	("04")
#define VPD_EC_WAM1P_REV1	("01")
#define VPD_EC_WAM1P_REV2	("02")
#define VPD_EC_WAM1P_REV3	("03")
#define VPD_EC_WAM1P_REV4	("3a")
#define WAM_SN_WAM1PR1_STR	("wam1p")
#define WAM_SN_WAM1PR2_STR	("wam1pr2")
#define WAM_SN_WAM1R4_STR	("SFR20101018")

#define WAM_PKG_UPD_MIN_DEV	(1)
#define WAM_PKG_UPD_SUP_DEV	(3)
#define WAM_PKG_UPD_MAX_DEV	(3)

static u32_t vpd_calc_checksum(kimchee_vpd_t *vpd);
static u32_t vpd_v2_calc_checksum(wam_vpd_t *vpd);

void
vpd_print(kimchee_vpd_t *vpd)
{
	u32_t   tmp = 0;
	char    str[256];
	int     i;

	printf("VPD Version\t\t: %u\n", vpd->v_version);

	memcpy(&tmp, vpd->v_length, 2);
	printf("VPD Size\t\t: %u bytes\n", tmp);

	printf("WWN\t\t\t: ");
	for (i = 0; i < 8; i++)
		printf("%02X ", vpd->v_wwn[i]);

	printf("\n");

	tmp = (vpd->v_subsys_id[0] << 8);
	tmp |= vpd->v_subsys_id[1];
	printf("Subsytem ID\t\t: 0x%X\n", tmp);

	tmp = (vpd->v_subsys_ven_id[0] << 8);
	tmp |= vpd->v_subsys_ven_id[1];
	printf("Subsytem Vendor ID\t: 0x%X\n", tmp);

	printf("Product ID Tag\t\t: 0x%X\n", vpd->v_id_tag);

	memcpy(&tmp, vpd->v_id_length, 2);
	printf("Product ID Length\t: %u\n", tmp);

	memcpy(str, vpd->v_id, VPD_PRODUCT_ID_SIZE);
	str[VPD_PRODUCT_ID_SIZE] = 0;
	printf("Product ID\t\t: %s\n", str);

	printf("VPD-R Tag\t\t: 0x%X\n", vpd->v_ro_tag);

	memcpy(&tmp, vpd->v_ro_length, 2);
	printf("VPD-RO Length \t\t: %u\n", tmp);

	memcpy(str, vpd->v_pn_key, VPD_KEY_SIZE);
	str[VPD_KEY_SIZE] = 0;
	printf("Key\t\t\t: %s\n", str);
	printf("Key Length\t\t: %u\n", vpd->v_pn_length);

	memcpy(str, vpd->v_pn, VPD_PN_SIZE);
	str[VPD_PN_SIZE] = 0;
	printf("Value\t\t\t: %s\n", str);

	memcpy(str, vpd->v_ec_key, VPD_KEY_SIZE);
	str[VPD_KEY_SIZE] = 0;
	printf("Key\t\t\t: %s\n", str);
	printf("Key Length\t\t: %u\n", vpd->v_ec_length);

	memcpy(str, vpd->v_ec, VPD_EC_SIZE);
	str[VPD_EC_SIZE] = 0;
	printf("Value\t\t\t: %s\n", str);

	memcpy(str, vpd->v_sn_key, VPD_KEY_SIZE);
	str[VPD_KEY_SIZE] = 0;
	printf("Key\t\t\t: %s\n", str);
	printf("Key Length\t\t: %u\n", vpd->v_sn_length);

	memcpy(str, vpd->v_sn, VPD_SN_SIZE);
	str[VPD_SN_SIZE] = 0;
	printf("Value\t\t\t: %s\n", str);

	memcpy(str, vpd->v_mn_key, VPD_KEY_SIZE);
	str[VPD_KEY_SIZE] = 0;
	printf("Key\t\t\t: %s\n", str);
	printf("Key Length\t\t: %u\n", vpd->v_mn_length);

	memcpy(str, vpd->v_mn, VPD_MN_SIZE);
	str[VPD_MN_SIZE] = 0;
	printf("Value\t\t\t: %s\n", str);

	memcpy(str, vpd->v_rv_key, VPD_KEY_SIZE);
	str[VPD_KEY_SIZE] = 0;
	printf("Key\t\t\t: %s\n", str);
	printf("Key Length\t\t: %u\n", vpd->v_rv_length);


	printf("Stored Checksum\t\t: 0x%X\n", vpd->v_checksum);
	printf("Calculated Checksum\t: 0x%X\n", vpd_calc_checksum(vpd));

	printf("VPD End Tag\t\t: 0x%X\n", vpd->v_end_tag);
}

void
vpd_v2_print(wam_vpd_t *vpd)
{
	u32_t   tmp = 0;
	char    str[256];
	int     i;

	printf("VPD Version\t\t: %u.%u\n", vpd->v_vpd_major, vpd->v_vpd_minor);
	printf("VPD Size\t\t: %u bytes\n", vpd->v_vpd_size);

	printf("Board Rev\t\t: %u.%u\n", vpd->v_board_major,
			vpd->v_board_minor);

	printf("WWN\t\t\t: ");
	for (i = 0; i < 8; i++)
		printf("%02x ", vpd->v_wwn[i]);

	printf("\n");

	tmp = vpd->v_subsys_id;
	printf("Subsytem ID\t\t: 0x%X\n", tmp);

	tmp = vpd->v_subsys_ven_id;
	printf("Subsytem Vendor ID\t: 0x%X\n", tmp);

	memcpy(str, vpd->v_id, VPD_PRODUCT_ID_SIZE);
	str[VPD_PRODUCT_ID_SIZE] = 0;
	printf("Product ID\t\t: %s\n", str);

	memcpy(str, vpd->v_pn, VPD_PN_SIZE);
	str[VPD_PN_SIZE] = 0;
	printf("Part Number\t\t: %s\n", str);

	memcpy(str, vpd->v_ec, VPD_EC_SIZE);
	str[VPD_EC_SIZE] = 0;
	printf("Engg Change\t\t: %s\n", str);

	memcpy(str, vpd->v_sn, VPD_SN_SIZE);
	str[VPD_SN_SIZE] = 0;
	printf("Serial Number\t\t: %s\n", str);

	memcpy(str, vpd->v_mn, VPD_MN_SIZE);
	str[VPD_MN_SIZE] = 0;
	printf("Manufacturer's Name\t: %s\n", str);

	memcpy(str, vpd->v_bn, VPD_BN_SIZE);
	str[VPD_BN_SIZE] = 0;
	printf("Batch\t\t\t: %s\n", str);

	if (vpd_v2_calc_checksum(vpd) == vpd->v_checksum)
		printf("Checksum\t\t: Good\n");
	else
		printf("Checksum\t\t: Failed (0x%X|0x%X)\n",
				vpd->v_checksum, vpd_v2_calc_checksum(vpd));
}



static int
wam_read_vpd(int fd, kimchee_vpd_t *vpd)
{
	int			size;

	/* read vpd from flash, offset VPD_OFF (0x6d0000), len VPD_SIZE */
	size = wam_pflash_read(fd, vpd, VPD_SIZE, VPD_OFF);
	if (size != VPD_SIZE) {
		printf("Unable to read vpd information, ret %d.\n", size);
		return (-1);
	}

	return (0);
}

static int
wam_read_vpd_v2(int fd, wam_vpd_t *vpd)
{
	int	size;

	size = wam_pflash_read(fd, vpd, sizeof(wam_vpd_t), VPD_V2_OFF);
	if (size != sizeof(wam_vpd_t)) {
		printf("Unable to read vpd information, ret %d.\n", size);
		return (-1);
	}

	return (0);
}



static int
wam_write_vpd_v2(int fd, wam_vpd_t *vpd)
{
	int			ret;

	ret = wam_pflash_program(fd, vpd, sizeof(wam_vpd_t), VPD_V2_OFF);
	if (ret != 0) {
		printf("Unable to write vpd information.\n");
		return (-1);
	}

	return (0);

}

int
wam_read_serial_number(int fd, char *serial_num)
{
	int	size;
	int	i;

	memset(serial_num, 0, WAM_SN_LEN);
	size = wam_pflash_read(fd, serial_num, WAM_SN_LEN, WAM_SN_OFF);
	if (size != WAM_SN_LEN) {
		return (-1);
	}

	for (i = 0; i < WAM_SN_LEN; i++) {
		if (!isalnum(serial_num[i]) && serial_num[i] != '-')
			serial_num[i] = ' ';
	}
	serial_num[WAM_SN_LEN - 1] = '\0';
	return (0);
}

static inline int
wam_vpd_v2_is_valid(wam_vpd_t *vpd)
{
	if (vpd->v_magic == WAM_VPD_MAGIC) {
		return (1);
	}
	return (0);
}


int
wam_get_board_id(int fd, int *major, int *minor)
{
	kimchee_vpd_t		vpd;
	wam_vpd_t		vpd_v2;
	char			serial_num[WAM_SN_LEN];
	int			ret;
	u32_t			i;

	// Read VPD v2
	if (wam_read_vpd_v2(fd, &vpd_v2) != 0) {
		WPRINT_ERR("Failed to read VPD Data!\n");
		return (-1);
	}

	/* check if VPD V2 is valid */
	if (wam_vpd_v2_is_valid(&vpd_v2)) {
		*major = vpd_v2.v_board_major;
		*minor = vpd_v2.v_board_minor;
		return (0);
	}

	ret = wam_read_vpd(fd, &vpd);
	if (ret != 0) {
		WPRINT_ERR("Failed to read VPD V1 Data!\n");
		return (-1);
	}

	if (strncmp((char *)vpd.v_pn, VPD_PN_WAM1P_REV3,
				strlen(VPD_PN_WAM1P_REV3)) == 0) {
		*major = 3;
		*minor = 2;
		return (0);
	} else if (strncmp((char *)vpd.v_pn, VPD_PN_WAM_1_REV4,
				strlen(VPD_PN_WAM_1_REV4)) == 0) {
		*major = 2;
		*minor = 0;
		return (0);
	} else if (strncmp((char *)vpd.v_pn, VPD_PN_WAM_1_REV3,
				strlen(VPD_PN_WAM_1_REV3)) == 0) {
		*major = 1;
		*minor = 2;
		return (0);
	}

	/* continue with other methods to find board id */
	/* check for WAM1+ Rev 1 and 2 by reading serial number */

	ret = wam_read_serial_number(fd, serial_num);
	if (ret != 0) {
		WPRINT_ERR("Unable to read serial number.\n");
		return (-1);
	}

	for (i = 0; i < WAM_SN_LEN - 1; i++) {
		if (serial_num[i] != (char)0xFF)
			break;
	}
	if (i == (WAM_SN_LEN - 1)) {
		WPRINT_ERR("Serial number not programmed. Cannot "
				"determine board id!\n");
		return (-1);
	}

	/* Note: important to check for rev 2 first! */
	if (strncmp(serial_num, WAM_SN_WAM1PR2_STR,
				strlen(WAM_SN_WAM1PR2_STR)) == 0) {
		*major = 3;
		*minor = 1;
		return (0);
	} else if (strncmp(serial_num, WAM_SN_WAM1PR1_STR,
				strlen(WAM_SN_WAM1PR1_STR)) == 0) {
		*major = 3;
		*minor = 0;
		return (0);
	} else if (strncmp(serial_num, WAM_SN_WAM1R4_STR,
				strlen(WAM_SN_WAM1R4_STR)) == 0) {
		*major = 2;
		*minor = 0;
		return (0);
	}

	return (-1);
}

int
wam_vpd_print(int fd)
{
	int			ret;
	kimchee_vpd_t		*vpd;

	vpd = malloc(sizeof (*vpd));
	ret = wam_read_vpd(fd, vpd);
	if (ret != 0) {
		return (-1);
	}

	vpd_print(vpd);
	return (0);
}

int
wam_vpd_v2_print(int fd)
{
	int		ret;
	wam_vpd_t	vpd;

	ret = wam_read_vpd_v2(fd, &vpd);
	if (ret != 0) {
		return (-1);
	}

	if (!wam_vpd_v2_is_valid(&vpd)) {
		WPRINT_ERR("VPD not programmed\n");
		return (-1);
	}

	vpd_v2_print(&vpd);
	return (0);
}


static int
wam_vpd_is_valid(kimchee_vpd_t *vpd)
{
	if ((vpd->v_version == 1) &&
	    (vpd->v_length[0] == (VPD_SIZE & 0xFF)) &&
	    (vpd->v_length[1] == ((VPD_SIZE >> 8) & 0xFF)) &&
	    (vpd->v_subsys_id[0] == ((VPD_SUBSYS_ID >> 8) & 0xFF)) &&
	    (vpd->v_subsys_id[1] == ((VPD_SUBSYS_ID) & 0xFF)) &&
	    (vpd->v_subsys_ven_id[0] == ((VPD_SUBSYS_VEN_ID >> 8) & 0xFF)) &&
	    (vpd->v_subsys_ven_id[1] == ((VPD_SUBSYS_VEN_ID) & 0xFF))) {
		return (1);
	}
	return (0);
}

static u32_t
vpd_calc_checksum(kimchee_vpd_t *vpd)
{
        u8_t            sum = 0;
        u8_t            *byte_stream;

        byte_stream = (u8_t *)&vpd->v_id_tag;
        while(byte_stream != (u8_t *)&vpd->v_checksum) {
                sum += *byte_stream;
                byte_stream++;
        }

        return (256 - sum);
}


static u32_t
vpd_v2_calc_checksum(wam_vpd_t *vpd)
{
        u8_t            sum = 0;
        u8_t            *byte_stream;

        byte_stream = (u8_t *)vpd;
        while(byte_stream != (u8_t *)&vpd->v_checksum) {
                sum += *byte_stream;
                byte_stream++;
        }

        return (256 - sum);
}

int
wam_request_boardid(int *major, int *minor)
{
	int devrev = 0;
	int ret = 0;

	/* ask user for device rev */
	printf("\nPlease select the WAM board from the above list: \n");
	printf("1 - WAM1 Rev 3      - Non-DIMM, Half Height Board\n");
	printf("2 - WAM1 Rev 4      - Non-DIMM, Full Height Board\n");
	printf("3 - WAM1 Plus Rev 1 - DIMM based, Full Height Board\n");
	printf("4 - WAM1 Plus Rev 2 - DIMM based, Full Height Board\n");
	printf("5 - WAM1 Plus Rev 3 - DIMM based, Full Height Board\n");
	printf("0 - Non of the above\n");
	printf("Board: \n");
	scanf("%d", &devrev);

	switch (devrev) {
	case 1:
		*major = 1;
		*minor = 2;
		break;
	case 2:
		*major = 2;
		*minor = 0;
		break;
	case 3:
		*major = 3;
		*minor = 0;
		break;
	case 4:
		*major = 3;
		*minor = 1;
		break;
	case 5:
		*major = 3;
		*minor = 2;
		break;
	default:
		printf("Board type not supported for VPD update\n");
		ret = -1;
	}

	return ret;
}


static int
get_batch_nr(char *serial_nr, char *batch)
{
	if (strstr(serial_nr, "SFR") == NULL) {
		return -1;
	}

	memcpy(batch, serial_nr + 3, 8);
	batch[8] = 0;

	return 0;
}

static void
vpd_v1_to_v2(wam_vpd_t *vpd_v2, kimchee_vpd_t *vpd)
{
	memcpy(vpd_v2->v_wwn, vpd->v_wwn, VPD_WWN_SIZE);
	memcpy(vpd_v2->v_pn, vpd->v_pn, VPD_PN_SIZE);
	memcpy(vpd_v2->v_ec, vpd->v_ec, VPD_EC_SIZE);
	memcpy(vpd_v2->v_sn, vpd->v_sn, VPD_SN_SIZE);
}

static void
get_dflt_vpd_v2(wam_vpd_t *vpd, char *sn, u32_t major, u32_t minor)
{
	char			*pn, *ec;

	if (major == 1 && minor == 2) {
		pn = VPD_PN_WAM_1_REV3;
		ec = VPD_EC_WAM1_REV3;
	} else if (major == 2 && minor == 0) {
		pn = VPD_PN_WAM_1_REV4;
		ec = VPD_EC_WAM1_REV4;
	} else if (major == 3 && minor == 0) {
		pn = VPD_PN_WAM1P_REV1;
		ec = VPD_EC_WAM1P_REV1;
	} else if (major == 3 && minor == 1) {
		pn = VPD_PN_WAM1P_REV2;
		ec = VPD_EC_WAM1P_REV2;
	} else if (major == 3 && minor == 2) {
		pn = VPD_PN_WAM1P_REV3;
		ec = VPD_EC_WAM1P_REV3;
	} else {
		assert(!"Bug, major/minor invalid");
	}

	memcpy(vpd->v_pn, pn, VPD_PN_SIZE);
	memcpy(vpd->v_ec, ec, VPD_EC_SIZE);
	memcpy(vpd->v_sn, sn, VPD_SN_SIZE);
}

static void
wam_vpd_set_constants(wam_vpd_t *vpd)
{
	memset(vpd, 0, sizeof (*vpd));

	// set vpd information
	vpd->v_magic = WAM_VPD_MAGIC;
	vpd->v_vpd_major = WAM_VPD_FMT_MAJOR;
	vpd->v_vpd_minor = WAM_VPD_FMT_MINOR;
	vpd->v_vpd_size = sizeof(wam_vpd_t);

	/* subsystem id in big endian */
        vpd->v_subsys_id = VPD_SUBSYS_ID;

        /* subsystem vendor id in big endian */
        vpd->v_subsys_ven_id = VPD_SUBSYS_VEN_ID;

	memcpy(vpd->v_id, VPD_PRODUCT_ID_STRING, VPD_PRODUCT_ID_SIZE);
	vpd->v_id[VPD_PRODUCT_ID_SIZE] = '\0';

	memcpy(vpd->v_mn, VPD_MANUFACT_NAME, VPD_MN_SIZE);
	vpd->v_mn[VPD_MN_SIZE] = '\0';
}


int
wam_convert_vpd(int fd)
{
	kimchee_vpd_t		*vpd = NULL;
	wam_vpd_t		*vpd_v2 = NULL;
	char			batch[64];
	int			major, minor;
//	int			devrev;
	char			*sn;
	char			serial_number[32];
	int			ret = 0;

	vpd_v2 = malloc(sizeof (*vpd_v2));
	if (vpd_v2 == NULL) {
		ret = -1;
		goto end;
	}

	// check existing v2
	if (wam_read_vpd_v2(fd, vpd_v2) == 0) {
		if (wam_vpd_v2_is_valid(vpd_v2)) {
			ret = 0;
			goto end;
		}
	}

	// malloc
	vpd = malloc(sizeof (*vpd));
	if (vpd == NULL) {
		ret = -1;
		goto end;
	}

        // read vpd
	if (wam_read_vpd(fd, vpd) != 0) {
		printf("Failed to read vpd\n");
		goto end;
	}

	if (wam_read_serial_number(fd, serial_number) == 0) {
		sn = serial_number;
	} else {
		sn = "Invalid-Serial";
	}

	printf("SN: %s\n", serial_number);

	major = 0;
	minor = 0;
	/* get board id */
	if (wam_get_board_id(fd, &major, &minor) != 0) {
		if (wam_request_boardid(&major, &minor) != 0) {
			goto end;
		}
	}

	wam_vpd_set_constants(vpd_v2);

	// update board major/minor
	vpd_v2->v_board_major = major;
	vpd_v2->v_board_minor = minor;

	if (wam_vpd_is_valid(vpd)) {
		printf("vpd is valid\n");
		vpd_v1_to_v2(vpd_v2, vpd);
	} else {
		printf("vpd not valid\n");
		get_dflt_vpd_v2(vpd_v2, sn, major, minor);
	}

	get_batch_nr(sn, batch);
	memcpy(vpd_v2->v_bn, batch, VPD_BN_SIZE);

	// update checksum
	vpd_v2->v_checksum = vpd_v2_calc_checksum(vpd_v2);

#if 1
	vpd_v2_print(vpd_v2);
#endif
	// program vpd at a different sector
	ret = wam_write_vpd_v2(fd, vpd_v2);
end:

	if (vpd)
		free(vpd);

	if (vpd_v2)
		free(vpd_v2);

	return (ret);
}


typedef enum {
        WAM_VPD_FIELD_INVALID       = -1,
        WAM_VPD_FIELD_SN            = 0,
        WAM_VPD_FIELD_PN            = 1,
        WAM_VPD_FIELD_WWN           = 2,
        WAM_VPD_FIELD_BOARDV        = 3,
        WAM_VPD_FIELD_BN            = 4,
	WAM_VPD_FIELD_MAX
} wam_vpd_field_t;

static vpd_field_t
vpd_get_arg(char *arg)
{
        char    str[256];
        u32_t   i = 0;

        while (*arg != '=' && i < 256)
                str[i++] = *arg++;

        if (i == 256)
                return -1;

        str[i] = 0;

        if (strcasecmp(str, "PN") == 0)
                return WAM_VPD_FIELD_PN;

        if (strcasecmp(str, "SN") == 0)
                return WAM_VPD_FIELD_SN;

        if (strcasecmp(str, "WWN") == 0)
                return WAM_VPD_FIELD_WWN;

        if (strcasecmp(str, "BOARDV") == 0)
                return WAM_VPD_FIELD_BOARDV;

        if (strcasecmp(str, "BN") == 0)
                return WAM_VPD_FIELD_BN;

        return WAM_VPD_FIELD_INVALID;
}

static int
vpd_program_field(wam_vpd_t *vpd, wam_vpd_field_t field, char *value)
{
        u32_t   i;
        u64_t   wwn;

        switch (field) {
        case WAM_VPD_FIELD_SN:
                memset(vpd->v_sn, ' ', VPD_SN_SIZE);
                strncpy((char *)vpd->v_sn, value, VPD_SN_SIZE);
                break;

        case WAM_VPD_FIELD_PN:
                if (strlen(value) != VPD_PN_SIZE) {
                        fprintf(stderr, "Part number (%s) is not of "
                                        "expected length(%u)\n",
                                        value, VPD_PN_SIZE);
                        return -1;
                }
                strncpy((char *)vpd->v_pn, value, VPD_PN_SIZE);
                vpd->v_ec[0] = value[VPD_PN_SIZE - 2];
                vpd->v_ec[1] = value[VPD_PN_SIZE - 1];
                break;

        case WAM_VPD_FIELD_WWN:
                wwn = strtoull(value, NULL, 16);
                for (i = 0; i < 8; i++) {
                        vpd->v_wwn[7 - i] = (wwn & 0xFF);
                        wwn >>= 8;
                }
                break;
	case WAM_VPD_FIELD_BOARDV:

		vpd->v_board_major = atoi(strtok(value, "."));
		vpd->v_board_minor = atoi(strtok(NULL, "."));
		break;

	case WAM_VPD_FIELD_BN:
		memcpy(vpd->v_bn, value, VPD_BN_SIZE);
		vpd->v_bn[VPD_BN_SIZE] = '\0';

		break;

        case VPD_FIELD_INVALID:
        default:
                return -1;
        }

        return 0;
}


int
wam_prog_vpd(int fd, int argc, char **argv)
{
	int		i;
	int		ret;
	char		*str;
	u32_t		req_args = WAM_VPD_FIELD_MAX;
	wam_vpd_t	vpd;
	wam_vpd_field_t field;

	wam_vpd_set_constants(&vpd);

	for (i = 0; i < argc; i++) {
                field = vpd_get_arg(argv[i]);
                if (field == -1) {
                        fprintf(stderr, "Invalid argument: %s\n", argv[i]);
                        return -1;
                }

                str = strchr(argv[i], '=');
                ret = vpd_program_field(&vpd, field, str + 1);
                if (ret != 0) {
                        return -1;
                }

                req_args--;
                if (req_args == 0)
                        break;
	}

        if (req_args != 0) {
                fprintf(stderr, "Not enough arguments\n");
                return -1;
        }

	vpd.v_checksum = vpd_v2_calc_checksum(&vpd);

#if 1
	vpd_v2_print(&vpd);
#endif
	// program vpd at a different sector
	wam_write_vpd_v2(fd, &vpd);

	return (0);
}

static int
wam_pkg_compatibility_check(int fd, pkg_hdr_t *pkg)
{
	int			devrev;
	int			major, minor;

	if (wam_get_board_id(fd, &major, &minor) != 0) {

		/* ask user for device rev */
		printf("\nPlease select the WAM board from the above list: \n");
		printf("1 - WAM1 Rev 3      - Non-DIMM, Half Height Board\n");
		printf("2 - WAM1 Rev 4      - Non-DIMM, Full Height Board\n");
		printf("3 - WAM1 Plus Rev X - DIMM based, Full Height "
				"Boards, all revisions\n");
		printf("Board: \n");
		scanf("%d", &devrev);

		switch (devrev) {
		case 1:
			major = 1;
			minor = 2;
			break;
		case 2:
			major = 2;
			minor = 0;
			break;
		case 3:
			major = 3;
			minor = 2;	/* XXX: should be 0-2 */
			break;
		default:
			return (-1);
		}
	}

	if (pkg->pkg_sup_matrix[major - 1] & ( 1 << minor)) {
		return (0);
	}

	printf("Package not compatible with the device. Update Incomplete.\n");

	return (-1);
}

// #define WAM_PKG_DRYRUN

/** \fn wam_update_package
 *
 * API to update software package in wbin format.
 */
int
wam_update_package(int fd, char *filename, int force)
{
        int             i;
        int             fdin;
        char            *file_buf;
        int		fsize;
        pkg_hdr_t       *pkg;
        fw_hdr_t        *file_hdr;
        fw_hdr_gen_t    *gen_hdr;
        int             ret;
        wam_info_t      info;
	int		update_pkg_ver = 0;
	int		nofw = 0;
	int		update_count = 0;

        WAM_PRINT("updating package: %s\n", filename);

	ret = wam_map_file(filename, &fdin, &file_buf, &fsize);

	if (ret != 0) {
		ret = -1;
		goto end;
	}

        pkg = (pkg_hdr_t *)file_buf;

	if (pkg_hdr_sanity_check(pkg) != 0) {
                ret = -1;
                goto end;
	}

        /* wam_dump_pkg_header(pkg); */
        if (wam_pkg_verify_interruptible(pkg, force) != 0) {
                ret = -1;
                goto end;
        }

        ret = wam_get_info(fd, &info);
        if (ret != 0 ) {
		nofw = 1;
		if (!force) {
			fprintf(stderr, "Failed to get dev info. "
					"Cannot update\n");
                        ret = -1;
                        goto end;
                }
        }

	if (wam_pkg_compatibility_check(fd, pkg) != 0) {
		ret = -1;
		goto end;
	}


        for (i = 0; i < PKG_FILE_COUNT; i++) {
                file_hdr = pkg_get_image(pkg, i);
                gen_hdr = &file_hdr->hdr_gen;
                /* image_ptr = fw_hdr_get_file(file_hdr); */

		ret = -1;
		switch (gen_hdr->hdr_type) {
		case FTYPE_BOOTLOADER:
			ret = wam_update_uboot_image(fd, file_hdr,
					NULL);
			break;

		case FTYPE_DDR_CONFIG:
			ret = wam_update_ddr_cfg_image(fd, file_hdr,
					&info);
			break;

		case FTYPE_EEPROM:
			if (nofw) {
				printf("Skipping EEPROM update\n");
				ret = 1;
				break;
			}

			ret = wam_update_eeprom_image(fd, file_hdr,
					&info);
			break;

		case FTYPE_FIRMWARE:
			ret = wam_update_fw_image(fd, file_hdr, &info);
			break;

		case FTYPE_SSD_FW:
			if (nofw) {
				printf("Skipping SSD FW update\n");
				ret = 1;
				break;
			}

			ret = wam_update_ssdfw_image(fd, file_hdr,
					&info);
			break;

		default:
			fprintf(stderr, "Invalid image type %d\n",
					gen_hdr->hdr_type);
			break;
		}

                if (ret < 0) {
                        fprintf(stderr, "Update failed, Cannot continue.\n");
			update_count = -1;
                        break;
                } else if (ret == 0) {
			update_pkg_ver = 1;
			update_count++;
		}
        }

	if (nofw && ret == 0) {
		printf("Firmware updated. Please reset firmware and update "
				"again to complete package update\n");
	}

        /* update the package version if all upgrade is successful */
        if (update_pkg_ver) {
                ret = wam_update_package_rec(fd, pkg);

                if (ret != 0) {
                        fprintf(stderr, "Updating package version failed, "
					"ret %d\n", ret);
			update_count = -1;
                }
        } else {
		printf("Device already has latest package. No changes made\n");
	}

end:
        close(fdin);

        return (update_count);
}

#define WAM_VG_SPI_IMG_SIZE             (1 << 19)
#define WAM_VG_BOOTLOADER_OFF           (0)
#define WAM_VG_BOOTLOADER_MAX_SIZE      (0x10000)
#define WAM_VG_FW_IMG0_OFF              (0x10000)
#define WAM_VG_FW_IMG1_OFF              (0x40000)
#define WAM_VG_FW_MAX_SIZE             (WAM_VG_FW_IMG1_OFF - WAM_VG_FW_IMG0_OFF)

int
wam_create_vgimg(char *vg_uboot, char *vg_firmware, char *out_file)
{
        int ret = 0;
        int file;
        int outfd;
        u32_t   img_size = WAM_VG_SPI_IMG_SIZE;
        u8_t    *buffer;
        char    *map = NULL;
        int	fsize;

        buffer = (u8_t *)malloc(img_size);
        if (buffer == NULL) {
                fprintf(stderr,"memory allocation error (%u bytes)\n",
			img_size);
                return -1;
        }

        memset(buffer, 0xFF, img_size);

        ret = wam_map_file(vg_uboot, &file, &map, &fsize);
        if (ret != 0) {
                fprintf(stderr, "File map error!");
                ret = -1;
                goto exit;
        }

        if (fsize > WAM_VG_BOOTLOADER_MAX_SIZE) {
                fprintf(stderr,
			"Bootloader file size %u too large. max size: %u\n",
                                fsize, WAM_VG_BOOTLOADER_MAX_SIZE);
                wam_unmap_file(file, map, fsize);
                ret = -1;
                goto exit;
        }

        memcpy(buffer + WAM_VG_BOOTLOADER_OFF, map, fsize);
        wam_unmap_file(file, map, fsize);

        ret = wam_map_file(vg_firmware, &file, &map, &fsize);
        if (ret != 0) {
                fprintf(stderr, "File map error!, %s", vg_firmware);
                perror("mmap");
                ret = -1;
                goto exit;
        }

        if (fsize > WAM_VG_FW_MAX_SIZE) {
                fprintf(stderr, "FW file size %u too large. max size: %u\n",
                                fsize, WAM_VG_FW_MAX_SIZE);
                wam_unmap_file(file, map, fsize);
                ret = -1;
                goto exit;
        }

        memcpy(buffer + WAM_VG_FW_IMG0_OFF, map, fsize);
        memcpy(buffer + WAM_VG_FW_IMG1_OFF, map, fsize);

        wam_unmap_file(file, map, fsize);

        outfd = open(out_file, O_CREAT | O_RDWR, 0666);
        if (outfd <= 0) {
                fprintf(stderr, "Failed to open/create output file %s\n",
			out_file);
                perror("open");
                ret = -1;
                goto exit;
        }

        fsize = write(outfd, buffer, img_size);
        if (fsize < img_size) {
                fprintf(stderr, "Failed to write to file %s [%u/%u]\n",
			out_file,
                                fsize, img_size);
                perror("write");
                ret = -1;
                goto exit;
        }

        close(outfd);
        ret = 0;

exit:
        free(buffer);
        return ret;
}


/**
 * Routine to read from flash in byte mode
 *
 * Using standard memcpy does not work.
 */
static void
pcie_memcpy(u8_t *dst, u8_t *src, u32_t size)
{
	u32_t i;

	for (i = 0; i < size; i++) {
		dst[i] = src[i];
	}
}

int
wam_nvsram_dump(int fd, u8_t *buffer, u32_t bufsize)
{
	u8_t	*addr;
	u32_t	byte_count;

	if (buffer == NULL)
		return -EINVAL;

	/* memory map flash area */
	addr = mmap(0, NV_NVSRAM_ACCESS_SIZE, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd, NV_MMAP_NVSRAM_OFFSET);
	if (addr == MAP_FAILED) {
		return -1;
	}

	byte_count = MIN(bufsize, NV_NVSRAM_ACCESS_SIZE);

	pcie_memcpy(buffer, addr, byte_count);

	return 0;
}


/*
 * WAM Info Display Helper functions
 */
static inline int
rel_time2str(u32_t t, char *str, u32_t len,
		char *singular, char *plural)
{
	if (t == 0)
		return 0;

	return wm_snprintf(str, len, "%u %s ", t, (t == 1) ? singular : plural);
}

int
wam_relative_time2str(uint32_t uptime, char *str, uint32_t len)
{
	u32_t tmp;
	u32_t t;
	u32_t count = 0;

	if (uptime == 0)
		return wm_snprintf(str, len, "0");

	/* secs per day */
	tmp = 60 * 60 * 24;
	t = uptime / tmp;
	uptime %= tmp;

	count += rel_time2str(t, str + count, len - count, "day", "days");

	/* secs per hour */
	tmp = 60 * 60;
	t = uptime / tmp;
	uptime %= tmp;

	if (t) {
		count += wm_snprintf(str + count, len - count,
				"%02u:%02u:%02u", t, uptime / 60, uptime % 60);
		return count;
	}

	/* secs per min */
	tmp = 60;
	t = uptime / tmp;
	uptime %= tmp;

	count += rel_time2str(t, str + count, len - count, "min", "mins");

	t = uptime;
	count += rel_time2str(t, str + count, len - count, "sec", "secs");

	str[count - 1] = 0;

	return count;
}


static u32_t
print_status_bit(char *buffer, u32_t length,
		u32_t flags, u32_t bit, char *str, char *true_state,
		char *false_state)
{
	return wm_snprintf(buffer, length, "    %s: %s\n", str,
			(flags & bit) ? true_state : false_state);
}

static u32_t
print_status_flags(char *buffer, u32_t length, u32_t flags)
{
	u32_t count = 0;
	char *no = "No";

	if ((flags & WAM_STAT_DATA_INVALID) &&
			!(flags & WAM_STAT_DATA_DIRTY)) {
		no = "No (10 min Power cycle needed)";
	}

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_DATA_INVALID,
		       "Data Valid\t\t", no, "Yes");

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_DATA_DIRTY,
		       "Data State\t\t", "Dirty", "Clean");

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_WRITE_READY,
		       "Write Ready\t\t", "Yes", "No");

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_SSD_CONN,
		       "SSD Connected\t", "Yes", "No");

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_SCAP_CONN,
		       "Supercap Connected\t", "Yes", "No");

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_SCAP_CHARGING,
		       "Supercap Charging\t", "Yes", "No");

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_SCAP_DISCHARGE_ON,
		       "Supercap Discharge\t", "On", "Off");

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_SCAP_HEALTHY,
		       "Online Health Check\t", "Yes", "No");

	count += print_status_bit(buffer + count, length - count,
			flags, WAM_STAT_DMA_ON,
		       "Host DMA\t\t", "Initialized", "Not Initialized");

	return count;
}

static void
wc_check_sn(char *sn, u32_t size)
{
	u32_t i;

	for (i = 0; i < size; i++) {
		if (isprint(sn[i]))
			continue;
		sn[i] = 0;
		break;
	}
}

static void
epoch_to_str(u32_t uptime, char *str, u32_t len)
{
	time_t ts = uptime;
	struct tm *tmp;

	*str = 0;
	tmp = localtime(&ts);
	if (tmp == NULL) {
		perror("localtime");
		return;
	}

	if (strftime(str, len, "%Y-%m-%d %H:%M:%S", tmp) == 0) {
		fprintf(stderr, "strftime returned 0");
		return;
	}
}

static inline u32_t
print_time_digits(char *str, u32_t len, u32_t digit,
		char sep, u32_t *leading_zero, u32_t yr)
{
	if ((*leading_zero) == 0 && digit == 0) {
		if (yr)
			return wm_snprintf(str, len, "____%c", sep);
		else
			return wm_snprintf(str, len, "__%c", sep);
	}

	if (digit)
		(*leading_zero)++;

	return wm_snprintf(str, len, "%02u%c", digit, sep);
}

static void
uptime_to_str(u32_t uptime, char *str, u32_t len)
{
	u32_t tmp;
	u32_t time_unit;
	u32_t count = 0;
	u32_t leading_zero = 0;

	/* secs per year */
	tmp = 60 * 60 * 24 * 365;
	if (uptime > 10 * tmp) {
		epoch_to_str(uptime, str, len);
		return;
	}

	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count,
			time_unit, '-', &leading_zero, 1);
	uptime %= tmp;

	/* secs per month */
	tmp = 60 * 60 * 24 * 30;
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count,
			time_unit, '-', &leading_zero, 0);
	uptime %= tmp;

	/* secs per day */
	tmp = 60 * 60 * 24;
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count,
			time_unit, ' ', &leading_zero, 0);
	uptime %= tmp;


	/* secs per hour */
	tmp = 60 * 60;
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count,
			time_unit, ':', &leading_zero, 0);
	uptime %= tmp;

	/* secs per min */
	tmp = 60;
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count,
			time_unit, ':', &leading_zero, 0);
	uptime %= tmp;

	leading_zero = 1;
	count += print_time_digits(str + count, len - count,
			uptime, ' ', &leading_zero, 0);
	str[count - 1] = 0;
}

static u32_t
print_event_data(char *str, u32_t len, u32_t event, u32_t data)
{
	int ret = 0;
	u32_t kb, mb;
	u32_t tmp;
	u64_t offset;
	char buf[256];

	switch (event) {
	case WAMEV_SCAP_VOL:
	case WAMEV_SCAP_VOL_LOW:
		ret = wm_snprintf(str, len, "%u mV", data);
		break;

	case WAMEV_FW_BOOTUP:
		if (data)
			ret = wm_snprintf(str, len, "Cold Boot");
		else
			ret = wm_snprintf(str, len, "Warm Boot");
		break;

	case WAMEV_DEV_WR_READY:
		ret = wm_snprintf(str, len, "0x%08X", data);
		break;

	case WAMEV_DATA_BACKUP_START:
	case WAMEV_DATA_BACKUP_DONE:
		tmp = data;
		offset = data & ~0x1FFF;
		offset <<= 3;
		data &= 0x1FFF;

		ret = wm_snprintf(str, len, "%u mV, ", data);

		mb = (offset >> 20);
		kb = ((offset >> 10) & ((1 << 10) - 1));
		if (mb)
			ret += wm_snprintf(str + ret, len -ret, "%u MB ", mb);

		if (!mb) {
			if (!kb)
				kb = (MV_DDR_OFFSET >> 10);

			ret += wm_snprintf(str + ret, len - ret, "%u KB", kb);
		}

		ret += wm_snprintf(str + ret, len - ret, " [0x%X]", tmp);
		break;

	case WAMEV_POWER_DOWN:
	case WAMEV_POWER_ON:
	case WAMEV_PCIE_RESET:
	case WAMEV_HARD_RESET:
		ret = wm_snprintf(str, len, "%u mV, 0x%x", data & 0xFFFF,
				data >> 24);
		break;

	case WAMEV_TIME_SET:
		uptime_to_str(data, buf, 256);
		ret = wm_snprintf(str, len, "[%s]", buf);
		break;

	case WAMEV_SCAP_HCHK_START:
		tmp = (data >> 16) & 0xFFFF;
		ret = wm_snprintf(str, len, "%u mV, %u mV", (data & 0xFFFF),
				tmp);
		break;

	case WAMEV_SCAP_HEALTH:
		tmp = (data >> 16) & 0xFFFF;
		ret = wm_snprintf(str, len, "%u F, %u mV", (data & 0xFFFF),
				tmp);
		break;

	case WAMEV_MAX_TEMP_ALERT:
		ret = wm_snprintf(str, len, "%u C", data);
		break;

	default:
		ret = wm_snprintf(str, len, "0x%X", data);
		break;
	}

	return ret;
}


static int
wam_print_backup_log(int fd, char *buffer, u32_t length)
{
	int ret;
	int count = 0;
	backup_stats_t	stats;
	u64_t ddr_offset;
	int i;

	memset(&stats, 0, sizeof(backup_stats_t));

	ret = ioctl(fd, NVRAM_IOC_GET_BACKUP_STATS, &stats);
	if (ret != 0) {
		fprintf(stderr, "Get Backup Stats failed: %d\n", ret);
		perror("ioctl");
		return -1;
	}

	ddr_offset = stats.b_ddr_offset;
	ddr_offset <<= 3;

	count += wm_snprintf(buffer + count, length - count,
			"\nBackup Log:\n");

	count += wm_snprintf(buffer + count, length - count,
			"    Backup Status\t: %s\n",
			stats.b_completed ? "Completed" : "INCOMPLETE");
	count += wm_snprintf(buffer + count, length - count,
			"    IO Error count\t: %u\n", stats.b_err_count);
	count += wm_snprintf(buffer + count, length - count,
			"    DRAM Offset\t\t: %llu MB\n", ddr_offset >> 20);

	for (i = 0; i < 4; i++) {
		count += wm_snprintf(buffer + count, length - count,
				"    Progress %2u%%\t: %u s\t"
				"Min IO Time: %u us\t"
				"Max IO Time: %u us\n", i * 25,
				stats.b_start_time[i],
				stats.b_min_time[i],
				stats.b_max_time[i]);
	}

	count += wm_snprintf(buffer + count, length - count,
			"    SSD WC Flush\t: %u s\n",
			stats.b_ssd_flush_time);
	count += wm_snprintf(buffer + count, length - count,
			"    Backup End\t\t: %u s\n",
			stats.b_end_time);
	count += wm_snprintf(buffer + count, length - count,
			"    Supercap Lasted\t: %u s\n",
			stats.b_alive_time);
	count += wm_snprintf(buffer + count, length - count,
			"    Supercap Start Vol\t: %u mV\n",
			stats.b_start_scap_vol);
	count += wm_snprintf(buffer + count, length - count,
			"    Supercap End Vol\t: %u mV\n",
			stats.b_end_scap_vol);
	count += wm_snprintf(buffer + count, length - count,
			"    DRAM SBE Count\t: %u\n",
			stats.b_sbe_cnt);
	count += wm_snprintf(buffer + count, length - count,
			"    DRAM DBE Count\t: %u\n\n",
			stats.b_dbe_cnt);

	return count;
}

static inline u32_t
print_scap_health(char *buffer, u32_t length, u32_t life)
{
	char *str[] = {
		"(Invalid Health Indicator)",
		"Very Good",
		"Good",
		"Fair",
		"Low",
		"Degraded",
	};

	if (life > WAM_SC_DEGRADED)
		life = 0;

	/* very good - if health above 90% */
	return wm_snprintf(buffer, length, "    Health\t\t: %s\n", str[life]);
}

int
wam_print_info(int fd, char *buffer, u32_t length, u32_t verbose)
{
	int		ret;
	char		str[256];
	u32_t		tmp;
	u32_t		count = 0;
	wam_info_t	info;
	wam_status_t	*dev_stat;
	u64_t		status;
	int		b_major = 0;
	int		b_minor = 0;
	wam_drv_version_t	ver;

	memset(&ver, 0, sizeof(wam_drv_version_t));

	ret = ioctl(fd, NVRAM_IOC_DRV_VERSION, &ver);
	if (ret != 0) {
		fprintf(stderr, "DRV_VERSION call failed!\n");
		perror("ioctl");
		/* proceed further without bailing out */
	}

	wam_get_board_id(fd, &b_major, &b_minor);

	ret = wam_get_info(fd, &info);
	if (ret != 0) {
		return ret;
	}

	ret = ioctl(fd, NVRAM_IOC_WRITE_READY, &status);
	if (ret != 0) {
		fprintf(stderr, "Get dev status failed!\n");
		perror("ioctl");
		return ret;
	}

	dev_stat = (wam_status_t *)&status;

	wc_check_sn(info.w_sn, WAM_MAX_SN_LEN);
	info.w_sn[WAM_MAX_SN_LEN - 1] = 0;

	count += wm_snprintf(buffer + count, length - count,
			"Serial Numbers:\n");
	count += wm_snprintf(buffer + count, length - count,
			"    Device\t\t: %s\n", info.w_sn);

	if (b_major < 3) {
		memcpy(str, info.w_ssd_sn, WAM_SSD_SN_LEN);
		str[WAM_SSD_SN_LEN] = 0;
		count += wm_snprintf(buffer + count, length - count,
				"    SSD\t\t\t: %s\n", str);
	}

	count += wm_snprintf(buffer + count, length - count,
			"\nVersions:\n");
	count += wm_snprintf(buffer + count, length - count,
			"    Driver\t\t: %u.%u.%u\n",
			ver.w_drv_major, ver.w_drv_minor, ver.w_drv_bugfix);
	count += wm_snprintf(buffer + count, length - count,
			"    Firmware\t\t: %s\n", info.w_fw_ver);

	memcpy(str, info.w_ssd_fw_ver, WAM_SSD_FW_VER_LEN);
	str[WAM_SSD_FW_VER_LEN] = 0;

	count += wm_snprintf(buffer + count, length - count,
			"    SSD Firmware\t: %s\n", str);

	count += wm_snprintf(buffer + count, length - count,
			"    EEPROM\t\t: %u\n", info.w_eeprom_ver);
	count += wm_snprintf(buffer + count, length - count,
			"    DDR Config\t\t: %u\n", info.w_ddr_cfg_ver);
	count += wm_snprintf(buffer + count, length - count,
			"    Board Rev\t\t: %u.%u\n", b_major, b_minor);

	if (verbose) {
		count += wm_snprintf(buffer + count, length - count,
				"\nClock Settings:\n");
		count += wm_snprintf(buffer + count, length - count,
				"    DDR\t\t\t: %u MHz\n", info.w_dclk);
		count += wm_snprintf(buffer + count, length - count,
				"    Internal Bus\t: %u MHz\n", info.w_tclk);
		count += wm_snprintf(buffer + count, length - count,
				"    SoC CPU\t\t: %u MHz\n", info.w_pclk);
	}

	wam_relative_time2str(info.w_uptime, str, 256);
	count += wm_snprintf(buffer + count, length - count,
			"\nFirmware Uptime\t\t: %s\n", str);

	if (info.w_host_time) {
		uptime_to_str(info.w_host_time, str, 256);
		count += wm_snprintf(buffer + count, length - count,
				"    Calendar Time\t: %s\n\n", str);
	}

	count += wm_snprintf(buffer + count, length - count,
			"Status Flags\t\t: %08X\n", info.w_flags);
	count += print_status_flags(buffer + count, length - count,
			info.w_flags);

	/* show backup error log in case of error */
	if (info.w_flags & WAM_STAT_DATA_INVALID)
		verbose = 1;

	count += wm_snprintf(buffer + count, length - count,
			"\nSupercap Info:\n");
	count += wm_snprintf(buffer + count, length - count,
			"    Status\t\t: %s\n",
			scap_status[dev_stat->w_scap_stat]);
	count += wm_snprintf(buffer + count, length - count,
			"    Current charge\t: %u mV (%u %%)\n",
			info.w_sc_vol, dev_stat->w_scap_charge);
	count += wm_snprintf(buffer + count, length - count,
			"    Expected charge\t: %u mV\n", info.w_sc_exp_vol);

	tmp = info.w_sc_cap * 10000;
	tmp /= info.w_sc_design_cap;
	if (info.w_sc_cap) {
		count += wm_snprintf(buffer + count, length - count,
				"    Capacitance\t\t: %u F (%u.%u %%)\n",
				info.w_sc_cap, tmp / 100, tmp % 100);
		count += print_scap_health(buffer + count, length - count,
				info.w_sc_life);
	} else {
		count += wm_snprintf(buffer + count, length - count,
				"    Capacitance\t\t: Health Check Not Done\n");
	}

	count += wm_snprintf(buffer + count, length - count,
			"    Design Cap\t\t: %u F\n", info.w_sc_design_cap);

	if (info.w_flags & WAM_STAT_SCAP_CALIB_ON) {
		count += wm_snprintf(buffer + count, length - count,
				"    Health Check\t: In Progress "
				"(%u %% complete)\n",
				info.w_sc_calib_progress);
	}

	if (info.w_sc_next_hchk) {
		uptime_to_str(info.w_sc_next_hchk, str, 256);
		count += wm_snprintf(buffer + count, length - count,
				"    Next Health Check\t: %s\n", str);
	}

	if (verbose) {
		count += wm_snprintf(buffer + count, length - count,
				"    Maximum Voltage\t: %u mV\n",
				info.w_sc_max_vout);

		count += wm_snprintf(buffer + count, length - count,
				"    Minimum Voltage\t: %u mV\n",
				info.w_sc_min_vout);

		count += wm_snprintf(buffer + count, length - count,
				"    Trim Voltage\t: %d mV\n",
				info.w_sc_trim_vout);
	}

	count += wm_snprintf(buffer + count, length - count,
			"\nDRAM Info\n");
	count += wm_snprintf(buffer + count, length - count,
			"    Size\t\t: %u KB\n", info.w_dram_size);
	count += wm_snprintf(buffer + count, length - count,
			"    Usable Size\t\t: %u KB\n",
			info.w_dram_size - info.w_dram_rsvd_size);

	count += wm_snprintf(buffer + count, length - count,
			"    Memory Test\t\t: %s\n",
			memtest_status[dev_stat->w_post_res]);

	count += wm_snprintf(buffer + count, length - count,
			"    Single Bit Errors\t: %u\n", info.w_dram_sbe);
	count += wm_snprintf(buffer + count, length - count,
			"    Multi Bit Errors\t: %u\n", info.w_dram_dbe);

	count += wm_snprintf(buffer + count, length - count,
			"\nSSD Info:\n");
	count += wm_snprintf(buffer + count, length - count,
			"    Status\t\t: %s\n",
			ssd_status[dev_stat->w_ssd_stat]);
	count += wm_snprintf(buffer + count, length - count,
			"    Average Erase Count\t: %u\n",
			info.w_ssd_avg_erase_cnt);
	count += wm_snprintf(buffer + count, length - count,
			"    Program Error Count\t: %u\n",
			info.w_ssd_prg_err_cnt);
	count += wm_snprintf(buffer + count, length - count,
			"    Erase Error Count\t: %u\n",
			info.w_ssd_erase_err_cnt);
	count += wm_snprintf(buffer + count, length - count,
			"    Defective Blocks\t: %u (8MB each)\n",
			info.w_ssd_defect_blks);

	count += wm_snprintf(buffer + count, length - count,
			"\nUnread Events\t\t: %u\n", info.w_events_unread);

	count += wm_snprintf(buffer + count, length - count,
			"Board Temperature\t: %u C", info.w_board_temp);
	if (info.w_temp_avg != 0) {
		count += wm_snprintf(buffer + count, length - count,
			" (Avg: %u C, Max: %u C, Min: %u C)",
			info.w_temp_avg, info.w_temp_max, info.w_temp_min);
	}
	count += wm_snprintf(buffer + count, length - count, "\n");

	if (verbose)
		count += wm_snprintf(buffer + count, length - count,
			"Alert Temperature\t: %u C\n", info.w_temp_alert);

	if (verbose)
		count += wam_print_backup_log(fd, buffer + count, length - count);

	return 0;
}

static u32_t
print_event(char *buffer, u32_t length, wam_event_t *event)
{
	char	str[256];
	u32_t	count = 0;

	uptime_to_str(event->e_time, str, 256);

	count += wm_snprintf(buffer + count, length - count, "[%s]: ", str);

	if (event->e_num >= WAMEV_LAST_ENTRY) {
		if (event->e_num >= WAMEV_DEV_EVENT) {
			wm_snprintf(str, 64, "FW Internal Event [%u]",
					event->e_num - WAMEV_DEV_EVENT);
		} else {
			wm_snprintf(str, 64, "Unknown Event [%u]",
					event->e_num);
		}
		count += wm_snprintf(buffer + count, length - count,
				"%-40s - ", str);
	} else {
		count += wm_snprintf(buffer + count, length - count,
				"%-40s - ", event_str[event->e_num]);
	}

	print_event_data(str, 256, event->e_num, event->e_data);
	count += wm_snprintf(buffer + count, length - count,
			"%s\n", str);

	return count;
}

int
wam_print_events(int fd, char *buffer, u32_t length, int clear)
{
	int		ret;
	u32_t		i;
	u32_t		size = 32 << 10;
	u32_t		count = 0;
	u32_t		nr_events = 0;
	wam_event_t	*events;

	events = (wam_event_t *)malloc(size);
	if (events == NULL)
		return -1;

	ret = wam_get_events(fd, events, size / sizeof(wam_event_t),
			&nr_events);
	if (ret != 0) {
		free(events);
		return ret;
	}

	if (nr_events == 0) {
		wm_snprintf(buffer, length, "No new event\n");
		free(events);
		return 0;
	}

	for (i = 0; i < nr_events; i++) {
		count += print_event(buffer + count, length - count, &events[i]);
	}

	if (clear)
		wam_clear_events(fd, nr_events);

	free(events);
	return 0;
}

int
wam_print_bad_blks(int fd, char *buffer, u32_t length)
{
	int ret;
	err_block_t *eblks;
	u64_t ddr_offset;
	u32_t count = 0;
	int i;

	eblks = (err_block_t *) malloc(8192);
	memset(eblks, 0, 8192);

	ret = ioctl(fd, NVRAM_IOC_GET_ERR_BLOCKS, eblks);
	if (ret != 0) {
		free(eblks);
		return -1;
	}

	i = 0;
	while (eblks[i].e_blk_size != 0 || eblks[i].e_qword_off != 0) {
		ddr_offset = eblks[i].e_qword_off;
		ddr_offset <<= 3;
		count += wm_snprintf(buffer + count, length - count,
				"[%u] offset: 0x%010llx size: 0x%08x, "
				"op: %s\n", i, ddr_offset, eblks[i].e_blk_size,
				error_type[eblks[i].e_err_op]);
		i++;
	}

	count += wm_snprintf(buffer + count, length - count,
			"Device recorded %u errors\n", i);
	free(eblks);
	return 0;
}

/**
 * API to read thermal log
 *
 * Get thermal log entries from the WAM device. The caller allocates the memory
 * for the log. The memory should be 8KB of minimum size.
 *
 * \param fd		- open file descriptor of the WAM device
 * \param trecords	- reference to memory allocated by caller to return
 *			the thermal log records
 * \param nr_records	- reference to return the number of records returned
 *			by the device
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_read_thermal_log(int fd, wam_temprec_t *trecords, uint32_t *nr_records)
{
	thermal_log_t	tlog;

	memset(&tlog, 0, sizeof(thermal_log_t));
	tlog.temp_records = (void *)trecords;

	*nr_records = 0;

	if (ioctl(fd, NVRAM_IOC_READ_TLOG, &tlog) < 0) {
		return -1;
	}

	*nr_records = tlog.nr_records;

	return 0;
}

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
int wam_clear_thermal_log(int fd, uint32_t nr_records)
{
	return ioctl(fd, NVRAM_IOC_CLEAR_TLOG, nr_records);
}


/**
 * API to set attribute
 *
 * \param fd		- open file descriptor of the WAM device
 * \param attr_key	- attribute identifier
 * \param attr_value	- attribute value
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_set_attr(int fd, uint32_t attr_key, u32_t attr_value)
{
	attr_pair_t	attr;

	attr.w_attr = attr_key;
	attr.w_value = attr_value;

	return ioctl(fd, NVRAM_IOC_SET_ATTR, &attr);
}

/**
 * API to get attribute
 *
 * \param fd		- open file descriptor of the WAM device
 * \param attr_key	- attribute identifier
 * \param attr_value	- reference to return attribute value
 *
 * \return 0 on success or -1 in case of an error
 */
int wam_get_attr(int fd, uint32_t attr_key, u32_t *attr_value)
{
	attr_pair_t	attr;

	memset(&attr, 0, sizeof(attr));

	attr.w_attr = attr_key;

	if (ioctl(fd, NVRAM_IOC_GET_ATTR, &attr) < 0) {
		return -1;
	}

	*attr_value = attr.w_value;

	return 0;
}

static u32_t
print_thermal_rec(char *buffer, u32_t length, wam_temprec_t *record)
{
	char	str[256];
	u32_t	count = 0;

	uptime_to_str(record->wt_time, str, 256);

	count += wm_snprintf(buffer + count, length - count, "[%s] ", str);
	count += wm_snprintf(buffer + count, length - count,
			"%4uC %4uC (%02u) %4uC (%02u)\n",
			record->wt_avg_temp, record->wt_max_temp,
			record->wt_max_ts, record->wt_min_temp,
			record->wt_min_ts);
	return count;
}

int
wam_print_thermal_log(int fd, char *buffer, u32_t length, int clear)
{
	int		ret;
	int		i;
	u32_t		nr_recs = 0;
	u32_t		count = 0;
	wam_temprec_t	*tlog;

	tlog = (wam_temprec_t *)malloc(32 << 10);
	if (tlog == NULL)
		return -1;

	ret = wam_read_thermal_log(fd, tlog, &nr_recs);
	if (ret != 0) {
		free(tlog);
		return ret;
	}

	if (nr_recs == 0) {
		wm_snprintf(buffer, length, "No entries in thermal log\n");
		free(tlog);
		return 0;
	}

	count += wm_snprintf(buffer + count, length - count,
			"%21s %5s %5s %4s %5s %4s\n",
			"Timestamp", "Avg", "Max", "(HR)", "Min", "(HR)");
	for (i = 0; i < nr_recs; i++) {
		count += print_thermal_rec(buffer + count, length - count,
				tlog + i);
	}

	if (clear)
		wam_clear_thermal_log(fd, nr_recs);

	free(tlog);
	return 0;
}

int
wam_ping(int fd)
{
	return ioctl(fd, NVRAM_IOC_PING_FW, 0);
}

/* *************************** Debug functions ***************************** */

void
wam_stub(void)
{
	wam_dump_fw_header(NULL);
	wam_dump_pkg_header(NULL);
	fw_hdr_tostr(0);
	dump_fw_rec(NULL);
}


