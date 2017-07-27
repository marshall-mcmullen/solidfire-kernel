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
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/jiffies.h>

#include "event.h"

#define EVENT_TEST_CONDITION(event)					\
	(test_bit(1, &(event)->cond) || test_and_clear_bit(0, &(event)->cond))

#define EVENT_SET_SIGNAL(event)		set_bit(0, &(event)->cond)
#define EVENT_SET_BROADCAST(event)	set_bit(1, &(event)->cond)


void mv_waitq_init(mv_waitq_t *event)
{
	event->cond = 0;
	init_waitqueue_head(&event->waitq);
}

/**
 * API to destroy a condition variable
 *
 * This API is a wrapper to the native condvar destroy routine.
 */
void mv_waitq_destroy(mv_waitq_t *event)
{
	/* No equivalent destroy in linux kernel */
	return;
}

void mv_waitq_wait(mv_waitq_t *event)
{
	/* clear the broadcast bit */
	clear_bit(1, &event->cond);

	/* wait until to be signaled or broadcast */
	wait_event(event->waitq, EVENT_TEST_CONDITION(event));
}

int mv_waitq_wait_sig(mv_waitq_t *event)
{
	int ret;

	/* clear the broadcast bit */
	clear_bit(1, &event->cond);

	/* wait until to be signaled or broadcast */
	ret = wait_event_interruptible(event->waitq,
			EVENT_TEST_CONDITION(event));

	ret = (ret ? -EINTR : 0);

	return ret;
}

int mv_waitq_timedwait(mv_waitq_t *event, unsigned int msec)
{
	int		ret;
	unsigned long   timo;

	timo = msecs_to_jiffies(msec);

	/* clear the broadcast bit */
	clear_bit(1, &event->cond);

	/* wait until to be signaled or broadcast */
	ret = wait_event_timeout(event->waitq,
			EVENT_TEST_CONDITION(event), timo);

	ret = (ret ? 0 : -ETIMEDOUT);

	return ret;
}

int mv_waitq_timedwait_sig(mv_waitq_t *event, unsigned int msec)
{
	int		ret;
	unsigned long	timo;

	timo = msecs_to_jiffies(msec);

	/* clear the broadcast bit */
	clear_bit(1, &event->cond);

	/* wait until to be signaled or broadcast */
	ret = wait_event_interruptible_timeout(event->waitq,
			EVENT_TEST_CONDITION(event), timo);

	ret = (!ret ? -ETIMEDOUT : ((ret == -ERESTARTSYS) ? -EINTR : 0));

	return ret;
}

void mv_waitq_signal(mv_waitq_t *event)
{
	EVENT_SET_SIGNAL(event);
	wake_up(&event->waitq);
}

void mv_waitq_broadcast(mv_waitq_t *event)
{
	EVENT_SET_BROADCAST(event);
	wake_up_all(&event->waitq);
}

