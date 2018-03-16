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

#include <linux/kernel.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <ctype.h>

#include "loki.h"
#include "wamlib.h"

char *wam_dev_prefix = "/dev/mvwam";
int g_devfd;
uint32_t g_nr_devs;

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

	"", "", "", "",
	"", "", "", "",
	"", "", "", "",
	"", "", "", "",
	"", "", "",

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

	"Unknown Event"
};


static void
discover_devs(void)
{
	wam_get_dev_count(g_devfd, &g_nr_devs);
	printf("Nr of WAM devices detected: %u\n", g_nr_devs);
}

static void
print_alerts(u32_t alerts)
{
	if (alerts & WAM_ALRT_ECC_ERROR)
		printf("ECC Error Alert!\n");

	if (alerts & WAM_ALRT_COMP_FAIL)
		printf("Component failed!\n");

	if (alerts & WAM_ALRT_FW_CRASH)
		printf("FW Crash Alert!\n");

	if (alerts & WAM_ALRT_INVALID_DATA)
		printf("Data Loss reported!\n");

	if (alerts & WAM_ALRT_SCAP_VOL_LOW)
		printf("Supercap voltage low!\n");

	if (alerts & WAM_ALRT_SCAP_DEGRADED)
		printf("Supercap degraded alert!\n");
}

static void
wait_on_alerts(void)
{
	int ret;
	int secs;
	int i;
	uint32_t dev_map, alerts;
	struct timespec start_ts, end_ts;

	for (i = 0; i < 12; i++) {
		clock_gettime(CLOCK_MONOTONIC, &start_ts);

		ret = wam_wait_on_alerts(g_devfd, &dev_map, &alerts, 1000);

		clock_gettime(CLOCK_MONOTONIC, &end_ts);
		secs = end_ts.tv_sec - start_ts.tv_sec;

		if (ret != 0) {
			if (errno == ETIME) {
				printf("Timed out after %d secs\n", secs);
			} else {
				printf("Unknown error! Bailing out!\n");
				perror("ioctl");
				return;
			}
		} else {
			printf("Alerts in devices: %08x, cause: %08x\n",
					dev_map, alerts);
			print_alerts(alerts);

			/* with single device, g_devfd is the device fd */
			wam_clear_dev_alert(g_devfd, alerts);
		}
	}
}

static u32_t
print_event_data(char *str, u32_t len, u32_t event, u32_t data)
{
	int ret = 0;
	u32_t kb, mb;

	switch (event) {
	case WAMEV_SCAP_VOL:
	case WAMEV_SCAP_VOL_LOW:
		ret = snprintf(str, len, "%u mV", data);
		break;

	case WAMEV_FW_BOOTUP:
		if (data)
			ret = snprintf(str, len, "Cold Boot");
		else
			ret = snprintf(str, len, "Warm Boot");
		break;

	case WAMEV_DEV_WR_READY:
		ret = snprintf(str, len, "0x%08X", data);
		break;

	case WAMEV_DATA_BACKUP_START:
		mb = (data >> 17);
		kb = ((data >> 7) & ((1 << 10) - 1));
		if (mb)
			ret = snprintf(str, len, "%u MB ", mb);

		if (!mb && kb)
			ret += snprintf(str + ret, len - ret, "%u KB", kb);
		break;

	case WAMEV_POWER_DOWN:
	case WAMEV_POWER_ON:
	case WAMEV_PCIE_RESET:
		ret = snprintf(str, len, "%u mV, 0x%x", data & 0xFFFF, data >> 24);
		break;

	default:
		ret = snprintf(str, len, "0x%X", data);
		break;
	}

	return ret;
}

static inline u32_t
print_time_digits(char *str, u32_t len, u32_t digit, char sep, u32_t *leading_zero)
{
	if ((*leading_zero) == 0 && digit == 0) {
		return snprintf(str, len, "__%c", sep);
	}

	if (digit)
		(*leading_zero)++;

	return snprintf(str, len, "%02u%c", digit, sep);
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
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count, time_unit, '-', &leading_zero);
	uptime %= tmp;

	/* secs per month */
	tmp = 60 * 60 * 24 * 30;
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count, time_unit, '-', &leading_zero);
	uptime %= tmp;

	/* secs per day */
	tmp = 60 * 60 * 24;
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count, time_unit, ' ', &leading_zero);
	uptime %= tmp;


	/* secs per hour */
	tmp = 60 * 60;
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count, time_unit, ':', &leading_zero);
	uptime %= tmp;

	/* secs per min */
	tmp = 60;
	time_unit = uptime / tmp;

	count += print_time_digits(str + count, len - count, time_unit, ':', &leading_zero);
	uptime %= tmp;

	leading_zero = 1;
	count += print_time_digits(str + count, len - count, uptime, ' ', &leading_zero);
}


static void
dump_dev_events(int dev)
{
	uint32_t nr_events, max_events;
	int fd;
	int i;
	int ret;
	wam_event_t *events;
	char str[256];

	sprintf(str, "%s%u", wam_dev_prefix, dev);

	fd = open(str, O_RDWR);
	if (fd <= 0) {
		printf("Failed to open device %s\n", str);
		return;
	}

	ret = wam_get_nr_events(fd, &max_events);

	if (max_events == 0) {
		printf("No new event\n");
		close(fd);
		return;
	}

	events = (wam_event_t *)malloc(max_events * sizeof(wam_event_t));
	if (events == NULL) {
		printf("failed to alloc mem for events\n");
		return;
	}

	ret = wam_get_events(fd, events, max_events, &nr_events);
	if (ret != 0) {
		printf("get events failed with err: %d\n", ret);
		free(events);
		return;
	}

	for (i = 0; i < nr_events; i++) {
		uptime_to_str(events[i].e_time, str, 256);
		printf("[%s]: ", str);

		if (events[i].e_num > WAMEV_LAST_ENTRY) {
			snprintf(str, 64, "Unknown Event [%u]",
					events[i].e_num);
			printf("%-40s - ", str);
		} else {
			printf("%-40s - ", event_str[events[i].e_num]);
		}

		print_event_data(str, 256, events[i].e_num, events[i].e_data);
		printf("%s\n", str);
	}

	free(events);
	close(fd);

	return;
}

static void
dump_events(void)
{
	int i;

	for (i = 0; i < g_nr_devs; i++) {
		dump_dev_events(i);
	}
}

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

char * post_status[] = {
	"POST Skipped",
	"POST Passed",
	"POST Failed"
};


static void
print_dev_status(int dev)
{
	char str[256];
	int fd;
	wam_status_t status;
	wam_status_t *dev_stat;

	sprintf(str, "%s%u", wam_dev_prefix, dev);

	fd = open(str, O_RDWR);
	if (fd <= 0) {
		printf("Failed to open device %s\n", str);
		return;
	}

	wam_get_status(fd, &status);

	dev_stat = &status;

	printf("Supercap status\t\t: %s[%u]\n",
			scap_status[dev_stat->w_scap_stat], dev_stat->w_scap_stat);
	printf("Supercap charge\t\t: %u%%\n",
			dev_stat->w_scap_charge);
	printf("SSD Status\t\t: %s\n",
			ssd_status[dev_stat->w_ssd_stat]);
	printf("Memory Test Status\t: %s\n",
			post_status[dev_stat->w_post_res]);


	close(fd);
}

static void
print_status(void)
{
	int i;

	for (i = 0; i < g_nr_devs; i++) {
		print_dev_status(i);
	}
}

int
main(int argc, char **argv)
{
	char device[128];

	sprintf(device, "%s%u", wam_dev_prefix, 0);
	g_devfd = open(device, O_RDWR);
	if (g_devfd < 0) {
		printf("Failed to open device %s\n", device);
		return -1;
	}

	discover_devs();

	wait_on_alerts();

	dump_events();

	print_status();

	close(g_devfd);

	return 0;
}

