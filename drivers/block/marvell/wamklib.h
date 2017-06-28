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
 * WAM Kernel APIs
 */

#ifndef __WAM_KAPI_H
#define	__WAM_KAPI_H

#include "mv_types.h"

/*
 * WAM Error codes
 */
typedef enum {
	WAM_SUCCESS	= 0,
	WAM_ECC_ERR	= -1,
	WAM_IO_ERR	= -2,
} wam_errno_t;


/*
 * Write to NVRAM
 *
 * @dev_id - wam device id (0 to n-1, n =  nr of devices)
 * @offset - device offset in bytes
 * @count - number of bytes to write
 * @buffer - buffer address that needs to be written to device.
 * Note: the buffer should be allocated using kmalloc API (the
 * driver uses __pa() to get the physical address to do the DMA).
 * @done - callback function which takes user provided cookie and
 * a status.
 * @cookie - private reference from user which is returned in done
 * callback
 * @return 0 if successful,
 * -ENOMEM - not enough memory
 * -EINVAL - invalid device id
 * -ESPIPE - invalid offset or length
 */
int wam_write(int dev_id, u64_t offset, u32_t count, char *buffer,
		void (*done)(void *, wam_errno_t), void *cookie);

/*
 * Read from NVRAM
 *
 * @dev_id - wam device id (0 to n-1, n =  nr of devices)
 * @offset - device offset in bytes
 * @count - number of bytes to read
 * @buffer - buffer address where data needs to be read into.
 * Note: the buffer should be allocated using kmalloc API (the
 * driver uses __pa() to get the physical address to do the DMA).
 * @done - callback function which takes user provided cookie and
 * a status.
 * @cookie - private reference from user which is returned in done
 * callback
 * @return 0 if successful,
 * -ENOMEM - not enough memory
 * -EINVAL - invalid device id
 * -ESPIPE - invalid offset or length
 */
int wam_read(int dev_id, u64_t offset, u32_t count, char *buffer,
		void (*done)(void *, wam_errno_t), void *cookie);

#endif
