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

#define _XOPEN_SOURCE 600

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/uio.h>
#include <errno.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <string.h>

#include "mv_types.h"
#include "mv_util.h"
#include "wamlib.h"
#include "loki.h"


#define PROC_CPU_INFO   "/proc/cpuinfo"
#define MAX_BUF_SIZE	256

enum {
	ARG_PRG = 0,
	ARG_FILE,
	ARG_IO_SIZE,
	ARG_SIZE,
	ARG_QUEUE_DEPTH,
	ARG_BARRIER_CNT,
	ARG_MAX
};

enum {
	OP_VERIFY,
	OP_WRITE
};

u32_t eng_base[] = { 0x0, 0x70000, 0x60000 };
struct iovec *iov;

unsigned int cpu_frequency = 0;

static void
print_usage(char *prg)
{
	printf("This program does asynchronous IO on the nvram device and "
			"calculates the throughput of the device\n");
	printf("Usage: <%s> <file path> <request size[k]> <total length[k|m]> "
			"<queue depth> <nr of requests after which a barrier request is sent>\n", prg);
}

static unsigned long long
get_bytes(char *str)
{
	char *unit;
	size_t size;

	size = strtoull(str, &unit, 10);
	switch (*unit) {
	case 'k':
		size <<= 10;
		break;

	case 'm':
		size <<= 20;
		break;

	case 'g':
		size <<= 30;
		break;

	case 'h':
		size = strtoull(str, &unit, 16);
		break;

	default:
		break;
	}

	return size;
}


static void
free_buffers(u32_t queue_depth)
{
	u32_t i;

	if (iov == NULL)
		return;

	for (i = 0; i < queue_depth; i++) {
		if (iov[i].iov_base)
			free(iov[i].iov_base);
	}

	free(iov);
}

unsigned int
get_cpu_freq(void)
{
	int fd;
	int ret;
	char buffer[MAX_BUF_SIZE];
	char *str;

	fd = open(PROC_CPU_INFO, O_RDONLY);
	if (fd < 0) {
		printf("Unable to open file %s\n", PROC_CPU_INFO);
		perror("open");
		return 0;
	}

	ret = read(fd, buffer, MAX_BUF_SIZE);
	if (ret == 0) {
		printf("read failed\n");
		perror("read");
		return 0;
	}

	str = strstr(buffer, "cpu MHz");
	if (str == NULL){
		printf("Could not find the cpu frequency string\n");
		return 0;
	}

	while (*str != ':')
		str++;

	str++;

	return strtoul(str, NULL, 10);
}


static int
prepare_buffers(u32_t req_size, u32_t queue_depth)
{
	int i;

	iov = (struct iovec *)malloc(sizeof(struct iovec) * queue_depth);
	if (iov == NULL) {
		printf("Not enough memory to allocate\n");
		return -ENOMEM;
	}

	for (i = 0; i < queue_depth; i++) {
		iov[i].iov_base = valloc(req_size);
		if (iov[i].iov_base == NULL) {
			printf("Not enough memory to allocate\n");
			goto buf_out;
		}

		memset(iov[i].iov_base, 0, req_size);
		iov[i].iov_len = req_size;
	}

	return 0;

buf_out:
	free_buffers(queue_depth);
	return -ENOMEM;
}

static void
print_engine_status(int fd, u32_t id)
{
	int         ret;
	reg_io_t    reg_cfg;

	memset(&reg_cfg, 0, sizeof(reg_io_t));

	reg_cfg.reg_addr = 0x904 + eng_base[id];

	ret = ioctl(fd, NVRAM_IOC_REG_IO, &reg_cfg);

	if (ret != 0) {
		return;
	}

	printf("Engine %u: ", id);
	if (reg_cfg.reg_val & 0x80000000)
		printf("Stuck. Please reboot the machine\n");
	else 
		printf("OK\n");

	return;
}

static void
print_all_eng_status(int fd)
{
	int i;

	for (i = 0; i < 3; i++)
		print_engine_status(fd, i);
}

static void
print_thruput(u64_t clock_tcks, u64_t txfr_len,
		unsigned int qd, unsigned long req_size)
{
	unsigned long ticks_per_usec = cpu_frequency;
	unsigned long long time;
	unsigned long perf_us = 0;
	unsigned long perf_s = 0;
	unsigned long usec_per_io;
	unsigned long iops;

	//    printf("Clock ticks recorded    : %llu\n", clock_tcks);
	//    printf("Bytes Transferred       : %llu bytes / %llu KB / %llu MB\n",
	//          txfr_len, (txfr_len >> 10), (txfr_len >> 20));

	time = clock_tcks;
	time /= ticks_per_usec;
	if (time)
		perf_us = ((txfr_len) * 1000 / (unsigned int)time);

	perf_s = perf_us * 1000000;
	perf_s >>= 20;

	usec_per_io = time * 1000;
	usec_per_io /= (txfr_len / req_size);
	if (usec_per_io == 0)
		printf("usec_per_io = 0\n");

	iops = 1000000;
	iops *= 1000000;
	iops /= usec_per_io;
	iops /= 1000;			/* for better precision */

	//printf("%u,%lu.%03lu,%lu\n", qd, perf_us / 1000, perf_us % 1000, req_size);
	printf("Throughput\t\t: %lu.%lu B/us (%lu.%lu MB/s)\n",
			perf_us / 1000, perf_us % 1000,
			perf_s / 1000, perf_s % 1000);
	printf("IOPS\t\t\t: %lu\n", iops);

	return;
}

static int
start_test(int fd, u32_t req_size, u64_t total_size,
		u32_t queue_depth, u32_t barrier)
{
	uint64_t queued_id = -1;
	uint64_t cmpltd_id = -1;
	u64_t io_id;
	u32_t buffer_idx = 0;
	u64_t offset = 0;
	u64_t start_tcks;
	u64_t end_tcks;
	u64_t clock_tcks;
	u64_t total_bytes;
	u32_t req_cnt = 0;
	u32_t nr_reqs;
	u32_t io_type;
	u32_t size;
	u32_t barrier_req_size = 8;
	u32_t nr_barriers = 0;
	u32_t reqs_so_far = 0;
	u32_t retry = 100000;
	int slots;
	int tmp;
	int i;
	int ret;

	nr_reqs = total_size / req_size;
	nr_barriers = nr_reqs / barrier;
	nr_reqs += nr_barriers;
	start_tcks = get_clock_ticks();

	while (req_cnt < nr_reqs) {
		tmp = (int)(queue_depth - (queued_id - cmpltd_id));
		if (tmp < 0)
			tmp = 0;

		slots = MV_MIN(tmp, (nr_reqs - req_cnt));

		for (i = 0; i < slots; i++) {
			if (reqs_so_far == barrier) {
				io_type = NV_BARRIER_IO;
				size = barrier_req_size;
				reqs_so_far = 0;
			} else {
				io_type = NV_NORMAL_IO;
				size = req_size;
				reqs_so_far++;
			}

			iov[buffer_idx].iov_len = size;
			ret = wam_async_write_vec(fd, io_type, &iov[buffer_idx],
					1, size, offset, &queued_id);
			if (ret != size) {
				perror("async io");
				printf("async io failed after %u requests\n", req_cnt);
				return ret;
			}
			req_cnt++;
			buffer_idx++;
			if (buffer_idx == queue_depth)
				buffer_idx = 0;
		}

		ret = wam_wait_on_completion(fd, cmpltd_id + 1);
		if (ret != 0) {
			printf("Wait on IO %llu failed\n", (u64_t)cmpltd_id + 1);
			return -1;
		}

		io_id = wam_get_completed_id(fd);
		if (io_id == -1L) {
			printf("Get completed ID failed\n");
			return -1;
		}
		if (cmpltd_id + 1 > io_id) {
			printf("Wait on IO %llu failed. Cmpltd: %llu\n",
					(u64_t)cmpltd_id + 1, io_id);
			return -1;
		}
#if 0
		do {
			io_id = wam_get_completed_id(fd);
			if (io_id == -1L) {
				printf("Get completed ID failed\n");
				return -1;
			}
			retry--;
			if (retry == 0) {
				printf("Max retry reached. IO Failed.\n");
				printf("IO ID: %llu, Completed ID: %lu, "
						"Queued ID: %lu\n",
						io_id, cmpltd_id,
						queued_id);
				printf("Total Reqs: %u, Queued Reqs:%u\n",
						nr_reqs, req_cnt);
				print_all_eng_status(fd);
				return -1;
			}

		} while (io_id == cmpltd_id);
#endif
		cmpltd_id = io_id;
		retry = 100000;
	}

	retry = 100000;
	/* wait for all requests to complete */
	do {
		ret = wam_wait_on_completion(fd, queued_id);
		if (ret != 0) {
			printf("Wait on IO %llu failed\n", (u64_t)queued_id + 1);
			return -1;
		}

		io_id = wam_get_completed_id(fd);
		if (io_id == -1L) {
			printf("Get completed ID failed\n");
			return -1;
		}

		retry--;
		if (retry == 0) {
			printf("Max retry reached. IO Failed.\n");
			printf("IO ID: %llu, Completed ID: %lu, "
					"Queued ID: %lu\n",
					io_id, cmpltd_id,
					queued_id);
			printf("Total Reqs: %u, Queued Reqs:%u\n",
					nr_reqs, req_cnt);
			print_all_eng_status(fd);
			return -1;
		}
	} while (io_id != queued_id);

	end_tcks = get_clock_ticks();

	clock_tcks = end_tcks - start_tcks;

	total_bytes = nr_reqs - nr_barriers;
	total_bytes *= req_size;
	print_thruput(clock_tcks,
			total_bytes + (nr_barriers * barrier_req_size),
			queue_depth, req_size);

	return 0;
}

int
main(int argc, char **argv)
{
	int fd;
	int ret;
	u32_t req_size;
	u32_t queue_depth;
	u64_t total_size;
	u32_t barrier;

	if (argc < ARG_MAX) {
		print_usage(argv[ARG_PRG]);
		return -1;
	}

	cpu_frequency = get_cpu_freq();

	fd = open(argv[ARG_FILE], O_RDWR);
	if (fd <= 0) {
		printf("Failed to open %s\n", argv[ARG_FILE]);
		perror("open");
		return -1;
	}

	req_size = get_bytes(argv[ARG_IO_SIZE]);
	if (req_size == 0) {
		printf("Invalid request size: %s\n", argv[ARG_IO_SIZE]);
		return -1;
	}

	total_size = get_bytes(argv[ARG_SIZE]);
	if (total_size == 0) {
		printf("Invalid total size: %s\n", argv[ARG_SIZE]);
		return -1;
	}

	queue_depth = strtoull(argv[ARG_QUEUE_DEPTH], NULL, 10);
	if (queue_depth == 0) {
		printf("Invalid queue depth: %s\n", argv[ARG_QUEUE_DEPTH]);
		return -1;
	}

	barrier = strtoul(argv[ARG_BARRIER_CNT], NULL, 10);
	if (barrier == 0)
		barrier = -1;

	ret = prepare_buffers(req_size, queue_depth);
	if (ret != 0) {
		return ret;
	}

	start_test(fd, req_size, total_size, queue_depth, barrier);

	free_buffers(queue_depth);
	return 0;
}
