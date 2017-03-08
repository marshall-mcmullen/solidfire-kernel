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

#ifndef _FW_HEADER_H_
#define _FW_HEADER_H_


// #include "mv_util.h"

#define FW_HDR_GEN_SIZE		(512)
#define FW_HDR_SIZE		(4096)
#define HEADER_MAGIC		(0x5ACAFE5A)
#define FW_HDR_FILE_FMT_VER	(0)
#define FW_HDR_VER_LEN		(64)
#define FW_HDR_BUILDTS_LEN	(64)

#define CC_ASSERT(label, cond)		enum label { label = 1 / (cond) }

/*
 * File generic header format:
 * ____________________________
 * |     Header Magic (4b)    |
 * ---------------------------
 * | File Format Version (4b) |
 * ---------------------------
 * |       Version (64b)      |
 * ---------------------------
 * |  Build Time Stamp (64b)  |
 * ---------------------------
 * |        Size  (4b)        |
 * ---------------------------
 * |     File Type (4b)	      |
 * ---------------------------
 * |     Interruptible (4b)   |
 * ---------------------------
 * |    Payload Checksum (4b) |
 * ---------------------------
 * |     Loading Addr (4b)    |
 * ---------------------------
 * |       Reserved (12b)     |
 * ---------------------------
 * |      Firmware  File      |
 * ____________________________
 */

/** \typedef fw_hdr_gen_t
 *
 * Generic header for upgrade file types
 */
typedef struct fw_hdr_gen fw_hdr_gen_t;
struct fw_hdr_gen {
	unsigned int	hdr_magic;
	unsigned int	hdr_file_fmt_ver;
	char		hdr_ver[FW_HDR_VER_LEN];
	unsigned char	hdr_build_ts[FW_HDR_BUILDTS_LEN];
	unsigned int	hdr_size;
	unsigned int	hdr_type;
	unsigned int	hdr_interruptible;
	unsigned int	hdr_chksum;
	unsigned int	hdr_ld_addr;

	unsigned char	hdr_rsvd[FW_HDR_GEN_SIZE - 160];
	unsigned int	hdr_magic2;
};
CC_ASSERT(fw_hdr_gen_t0, sizeof (fw_hdr_gen_t) == FW_HDR_GEN_SIZE);

/** \typedef fw_hdr_t
 */
typedef  struct fw_hdr fw_hdr_t;
struct fw_hdr {
	fw_hdr_gen_t	hdr_gen;

	unsigned char	hdr_rsvd[FW_HDR_SIZE - sizeof (fw_hdr_gen_t) - 4];
	unsigned int	hdr_magic3;
};
CC_ASSERT(fw_hdr_t0, sizeof (fw_hdr_t) == FW_HDR_SIZE);


/** \enum file_hdr_type_e
 *
 * Define file types
 */
typedef enum {
	FTYPE_BOOTLOADER	= 0,
	FTYPE_DDR_CONFIG	= 1,
	FTYPE_EEPROM		= 2,
	FTYPE_FIRMWARE		= 3,
	FTYPE_SSD_FW		= 4,
	FTYPE_MAX
} file_hdr_type_e;

#endif /* _FW_HEADER_H_ */
