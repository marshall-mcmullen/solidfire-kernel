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

#ifndef _PACKAGE_H_
#define _PACKAGE_H_

#include "fw_header.h"

#define PKG_HDR_SIZE		(1024)
#define PKG_HEADER_MAGIC	(0xCAFE5B5B)
#define PKG_HDR_VER_LEN		(64)
#define PKG_HDR_BUILDTS_LEN	(64)
#define PKG_FILE_COUNT		(5)
#define PKG_MAX_COMP_VER	(16)

#define PKG_VER_MAJOR		(1)
#define PKG_VER_MINOR		(1)

#ifndef CC_ASSERT
#define CC_ASSERT(label, cond)		enum label { label = 1 / (cond) }
#endif

/*
 * Package header format:
 * ____________________________
 * |     Header Magic (4b)    |
 * ---------------------------
 * |  Package  Version (64b)  |
 * ---------------------------
 * |  Build Time Stamp (64b)  |
 * ---------------------------
 * |      File Count (4b)     |
 * ---------------------------
 * |  Package Checksum (4b)   |
 * ---------------------------
 * |        Offset (4b)       |
 * ---------------------------
 * |        Size (4b)         |
 * ---------------------------
 * |        Offset (4b)       |
 * ---------------------------
 * |        Size (4b)         |
 * ---------------------------
 * |        Offset (4b)       |
 * ---------------------------
 * |        Size (4b)         |
 * ---------------------------
 * |        Offset (4b)       |
 * ---------------------------
 * |        Size (4b)         |
 * ---------------------------
 * |        Offset (4b)       |
 * ---------------------------
 * |        Size (4b)         |
 * ---------------------------
 * |       Reserved (nb)      |
 * ---------------------------
 */

typedef struct pkg_offset pkg_offset_t;
struct pkg_offset {
	unsigned int	pkg_offset;
	unsigned int	pkg_length;
};

/** \typedef pkg_pkg_t
 *
 * Package header for upgrade files
 */
typedef struct pkg_hdr pkg_hdr_t;
struct pkg_hdr {
	union {
	struct {
	unsigned int	pkg_magic;
	unsigned char	pkg_ver[PKG_HDR_VER_LEN];
	unsigned char	pkg_build_ts[PKG_HDR_BUILDTS_LEN];
	unsigned short	pkg_major;
	unsigned short	pkg_minor;
	unsigned int	pkg_count;
	unsigned char	pkg_sup_matrix[PKG_MAX_COMP_VER];
	unsigned char	pkg_rsvd1[96];
	unsigned int	pkg_chksum;			/* 256 bytes */
	pkg_offset_t	pkg_offset_list[0];
	};
	unsigned char	pkg_rsvd2[PKG_HDR_SIZE];
	};
};
CC_ASSERT(pkg_hdr_t0, sizeof (pkg_hdr_t) == PKG_HDR_SIZE);

void
dump_pkg_header(pkg_hdr_t *hdr);

#endif /* _PACKAGE_H_ */
