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

#ifndef __MV_WAITQ_H
#define __MV_WAITQ_H


#include <linux/version.h>

typedef struct mv_wait_queue {
	wait_queue_head_t waitq;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	unsigned int cond;
#else
	unsigned long cond;
#endif
} mv_waitq_t;

void mv_waitq_init(mv_waitq_t *event);

void mv_waitq_destroy(mv_waitq_t *event);

void mv_waitq_wait(mv_waitq_t *event);

int mv_waitq_wait_sig(mv_waitq_t *event);

int mv_waitq_timedwait(mv_waitq_t *event, unsigned int timo);

int mv_waitq_timedwait_sig(mv_waitq_t *event, unsigned int timo);

void mv_waitq_signal(mv_waitq_t *event);

void mv_waitq_broadcast(mv_waitq_t *event);


#endif   /* __MV_WAITQ_H */
