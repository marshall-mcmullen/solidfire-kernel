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
 * NVRAM Utility macros and functions
 */

#ifndef __LOKI_MVUTIL_H
#define __LOKI_MVUTIL_H

#include "mv_types.h"

#define LSB_32(x)			(unsigned int)(x)
#define MSB_32(x)			((u64_t)(x) >> 32)

#define MV_MAX(x, y)			((x) < (y) ? (y) : (x))
#define MV_MIN(x, y)			((x) < (y) ? (x) : (y))
#define MV_MAX_U64(x, y)		((u64_t)(x) < (u64_t)(y) ? (y) : (x))
#define MV_MIN_U64(x, y)		((u64_t)(x) < (u64_t)(y) ? (x) : (y))

#define MV_OFFSET_OF(type, var)	  ((void*)(&((type *)0x0)->var))
#define MV_CONTAINER_OF(addr, type, var)				\
	(type *)((void *)(addr) - MV_OFFSET_OF(type, var))

#define ROUNDING(value, align)						\
	(((value) + (align) - 1) / (align) * (align))

#ifdef ARCH_I386
#define GET_CLOCK_TICKS(ticks)      __asm__("rdtsc" : "=A" (ticks) : );
#else
#define GET_CLOCK_TICKS(ticks)						\
	do {								\
		unsigned int lo, hi;					\
		__asm__("rdtsc" : "=a" (lo),  "=d" (hi));		\
		ticks = (((u64_t)hi << 32) | lo);			\
	} while (0)
#endif

static inline unsigned long long
get_clock_ticks(void)
{
    unsigned long long ticks;
    GET_CLOCK_TICKS(ticks);
    return ticks;
}


#define CC_ASSERT(label, cond)		enum label { label = 1 / (cond) }

#endif
