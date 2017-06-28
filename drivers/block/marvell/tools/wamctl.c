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


#include <assert.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <errno.h>
#include <time.h>


#include "loki.h"
#include "fw_header.h"
#include "wamlib.h"
#include "package.h"
#include "wamlib_priv.h"


static int wc_info(int argc, char **argv);
static int wc_events(int argc, char **argv);
static int wc_ping(int argc, char **argv);
static int wc_reset(int argc, char **argv);
static int wc_hardreset(int argc, char **argv);
static int wc_ssd_format(int argc, char **argv);
static int wc_acquit(int argc, char **argv);
static int wc_list_errs(int argc, char **argv);
static int wc_ssd_update_raw(int argc, char **argv);
static int wc_update_eeprom_raw(int argc, char **argv);
static int wc_read_eeprom(int argc, char **argv);
static int wc_regdump(int argc, char **argv);
static int wc_scap_calib(int argc, char **argv);
static int wc_reg_rw(int argc, char **argv);
static int wc_nvsram_dump(int argc, char **argv);
static int wc_thermal_log(int argc, char **argv);
static int wc_set_attr(int argc, char **argv);
static int wc_get_attr(int argc, char **argv);

static int wc_update(int argc, char **argv);
static int wc_board_id(int argc, char **argv);
static int wc_print_hidden(int argc, char **argv);
static int wc_print_vpd(int argc, char **argv);
static int wc_convert_vpd(int argc, char **argv);
static int wc_prog_vpd(int argc, char **argv);


#define WAM_HIDDEN_HELP_STR		"-H"
#define WAM_UPDATE_MAX                  (5)

/*
 * Time period constants for scheduling
 */
#define ONE_MINUTE                      (60)
#define ONE_HOUR                        (60 * ONE_MINUTE)
#define ONE_DAY                         (24 * ONE_HOUR)
#define ONE_WEEK                        (7 * ONE_DAY)
#define ONE_MONTH			(30 * ONE_DAY)
#define ONE_YEAR                        (365 * ONE_DAY)


typedef struct {
	u32_t	attr_key;
	char	*attr_str;
} wam_attr_ent_t;

wam_attr_ent_t wam_attr_db[] = {
	{ WA_OFFLINE_HCHK_ON_COLDBOOT,	"pwronhchktype" },
	{ WA_SCAP_HEALTHCHECK_SCHEDULE,	"hchkschedule" },
	{ WA_THERMAL_ALERT_TEMP,	"maxtemp" },
};

/** \wc_update_fn
 */
typedef int
(*wc_update_fn)(int fd, char *filename, int force);


/** \typedef wc_update_cmd_t
 *
 */
typedef struct wc_update_cmd wc_update_cmd_t;
struct wc_update_cmd {
        char            *u_long_cmd;	/**< long command               */
        char            *u_short_cmd;	/**< short command              */
        wc_update_fn   u_update_fn;	/**< update function            */
};


typedef struct {
        char    *long_cmd;
        char    *short_cmd;
        char    *desc;
        char    *usage;
        int     (*do_cmd)(int, char **);
        u32_t   min_args;
        u32_t   max_args;
	int	cmd_hidden;
} cmd_option_t;

static cmd_option_t g_cmds[] = {
	{
		.long_cmd	= "info",
		.short_cmd	= "I",
		.desc		= "Show device information",
		.usage		= "",
		.do_cmd		= wc_info,
		.min_args	= 0,
		.max_args	= 1,
	},
	{
		.long_cmd	= "events",
		.short_cmd	= "E",
		.desc		= "Show device event log. Option "
					"'clear' will show and clear the log",
		.usage		= "[clear]",
		.do_cmd		= wc_events,
		.min_args	= 0,
		.max_args	= 1,
	},
	{
		.long_cmd	= "update",
		.short_cmd	= "U",
		.desc		= "Update device firmware package",
		.usage		= "package|pkg=<package file>",
		.do_cmd		= wc_update,
		.min_args	= 1,
		.max_args	= 2,
	},
	{
		.long_cmd	= "ping",
		.short_cmd	= "p",
		.desc		= "Ping device for responsiveness",
		.usage		= "",
		.do_cmd		= wc_ping,
		.min_args	= 0,
		.max_args	= 0,
	},
	{
		.long_cmd	= "reset",
		.short_cmd	= "R",
		.desc		= "Reset device firmware",
		.usage		= "",
		.do_cmd		= wc_reset,
		.min_args	= 0,
		.max_args	= 0,
	},
	{
		.long_cmd	= "hardreset",
		.short_cmd	= "HR",
		.desc		= "Hard reset",
		.usage		= "",
		.do_cmd		= wc_hardreset,
		.min_args	= 0,
		.max_args	= 0,
	},
	{
		.long_cmd	= "list_errors",
		.short_cmd	= "L",
		.desc		= "List error blocks log "
					"(errors that occured during backup/"
					"restore)",
		.usage		= "",
		.do_cmd		= wc_list_errs,
		.min_args	= 0,
		.max_args	= 0,
	},
	{
		.long_cmd	= "clear_error",
		.short_cmd	= "C",
		.desc		= "Clear device error log (latched error "
					"needs to be cleared by user)",
		.usage		= "",
		.do_cmd		= wc_acquit,
		.min_args	= 0,
		.max_args	= 0,
	},
	{
		.long_cmd	= "update_ssd",
		.short_cmd	= "us",
		.desc		= "Update ssd firmware",
		.usage		= "<ssd firmware file>",
		.do_cmd		= wc_ssd_update_raw,
		.min_args	= 1,
		.max_args	= 1,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "format_ssd",
		.short_cmd	= "fssd",
		.desc		= "Format ssd attached to the device",
		.usage		= "",
		.do_cmd		= wc_ssd_format,
		.min_args	= 0,
		.max_args	= 0,
	},
	{
		.long_cmd	= "scap_calib",
		.short_cmd	= "calib",
		.desc		= "Start supercap health measurement.",
		.usage		= "[offline] - to trigger offline measurement",
		.do_cmd		= wc_scap_calib,
		.min_args	= 0,
		.max_args	= 1,
	},
	{
		.long_cmd	= "update_eeprom",
		.short_cmd	= "ue",
		.desc		= "Update the device eeprom",
		.usage		= "<eeprom binary file>",
		.do_cmd		= wc_update_eeprom_raw,
		.min_args	= 1,
		.max_args	= 1,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "read_eeprom",
		.short_cmd	= "re",
		.desc		= "Read the device eeprom and dump to console "
					"(in binary format)",
		.usage		= " > filename",
		.do_cmd		= wc_read_eeprom,
		.min_args	= 0,
		.max_args	= 0,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "regdump",
		.short_cmd	= "rdmp",
		.desc		= "Print device registers",
		.usage		= "",
		.do_cmd		= wc_regdump,
		.min_args	= 0,
		.max_args	= 0,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "reg_rw",
		.short_cmd	= "i",
		.desc		= "Read/Write register",
		.usage		= "<reg address> [value to write in hex]",
		.do_cmd		= wc_reg_rw,
		.min_args	= 1,
		.max_args	= 2,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "boardid",
		.short_cmd	= "bid",
		.desc		= "Identify the board ID/revision",
		.usage		= "",
		.do_cmd		= wc_board_id,
		.min_args	= 0,
		.max_args	= 0,
		.cmd_hidden	= 0,
	},
	{
		.long_cmd	= "hidden",
		.short_cmd	= "H",
		.desc		= "print all/hidden commands",
		.usage		= "",
		.do_cmd		= wc_print_hidden,
		.min_args	= 0,
		.max_args	= 0,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "vpd",
		.short_cmd	= "vpd",
		.desc		= "Print Vital Product Data(VPD) info",
		.usage		= "",
		.do_cmd		= wc_print_vpd,
		.min_args	= 0,
		.max_args	= 1,
	},
	{
		.long_cmd	= "convert_vpd",
		.short_cmd	= "cvpd",
		.desc		= "Convert the vpd to V2",
		.usage		= "",
		.do_cmd		= wc_convert_vpd,
		.min_args	= 0,
		.max_args	= 0,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "prog_vpd",
		.short_cmd	= "pvpd",
		.desc		= "Program wam vpd",
		.usage		= "\n\t\tBOARDV=<major.minor>\n"
				  "\t\tBN=<8 bytes batch number>\n"
				  "\t\tSN=<serial number>\n"
				  "\t\tWWN=<wwn> <part number>\n"
				  "\t\tPN=<part number>\n",
		.do_cmd		= wc_prog_vpd,
		.min_args	= 5,
		.max_args	= 5,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "nvsdump",
		.short_cmd	= "nvs",
		.desc		= "Dump internal nvram to stdout",
		.usage		= "> nvsfile.bin",
		.do_cmd		= wc_nvsram_dump,
		.min_args	= 0,
		.max_args	= 0,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "tlog",
		.short_cmd	= "TL",
		.desc		= "Print thermal log",
		.usage		= "[clear] ",
		.do_cmd		= wc_thermal_log,
		.min_args	= 0,
		.max_args	= 1,
		.cmd_hidden	= 1,
	},
	{
		.long_cmd	= "setattr",
		.short_cmd	= "set",
		.desc		= "Set device attributes\n"
					"\t  key-value pairs:\n"
					"\t  pwronhchktype\t= "
					"<online | offline>\n"
					"\t  hchkschedule\t= <n><d|w|m> "
					" example: 2w -> 2 weeks\n"
					"\t  maxtemp\t= <max thermal "
					"alert temp in C>",
		.usage		= "<key1=value1> [key2=value2] ...",
		.do_cmd		= wc_set_attr,
		.min_args	= 1,
		.max_args	= 3,
	},
	{
		.long_cmd	= "getattr",
		.short_cmd	= "get",
		.desc		= "Get device attributes\n"
					"\t  keys to query: "
					"pwronhchktype, "
					"hchkschedule, "
					"maxtemp",
		.usage		= "<key1> [key2] ...",
		.do_cmd		= wc_get_attr,
		.min_args	= 1,
		.max_args	= 3,
	},
};

static int g_nr_cmds = sizeof (g_cmds) / sizeof (cmd_option_t);
static int g_devfd;
static char *g_cmd_string;
static char *g_dev_fname;

static void
print_cmd_usage(u32_t cmd_idx, int print_hidden)
{
	cmd_option_t	*cmd = &g_cmds[cmd_idx];

	if (cmd->cmd_hidden && !print_hidden) {
		return;
	}

	printf("\n\t%s|%s %s\n\t- %s\n", cmd->long_cmd, cmd->short_cmd,
			cmd->usage, cmd->desc);
}


static void
print_usage(int print_hidden)
{
	u32_t i;

	printf("Usage: %s <device file> <cmd> <arguments>\n", g_cmd_string);
	printf("Commands:");

	for (i = 0; i < g_nr_cmds; i++)
		print_cmd_usage(i, print_hidden);
}

static int
wc_print_hidden(int argc, char **argv)
{
	print_usage(1);
	return (0);
}

static int
check_cmd(u32_t idx, char *cmd, int argc)
{
	if (strncmp(cmd, g_cmds[idx].long_cmd, 64) &&
			strncmp(cmd, g_cmds[idx].short_cmd, 64)) {
		return (-1);
	}

	if (argc - 1 < g_cmds[idx].min_args) {
		fprintf(stderr, "Too few arguments for %s\n", cmd);
		print_cmd_usage(idx, 1);
		exit(1);
	}

	if (argc - 1 > g_cmds[idx].max_args) {
		fprintf(stderr, "Too many arguments for %s\n", cmd);
		print_cmd_usage(idx, 1);
		exit(1);
	}

	return (0);
}

static int
open_dev_file(char *devfile, int *devfd)
{
	int ret;
	struct stat devstat;

	memset(&devstat, 0, sizeof(struct stat));
	ret = stat(devfile, &devstat);
	if (ret != 0) {
		fprintf(stderr, "Cannot stat file %s\n", devfile);
		perror("stat");
		return ret;
	}

	if (!S_ISCHR(devstat.st_mode)) {
		fprintf(stderr, "File %s is not a character device\n",
				devfile);
		return -1;
	}

	*devfd = open(devfile, O_RDWR);
	if (*devfd < 0) {
		fprintf(stderr, "Failed to open device: %s\n", devfile);
		perror("open");
		return -1;
	}

	return 0;
}

int
main(int argc, char **argv)
{
	u32_t	i;
	int	ret = 0;

	g_cmd_string = argv[0];

	if (argc < 3) {
		print_usage(0);
		return -1;
	}

	g_dev_fname = argv[1];
	if (open_dev_file(g_dev_fname, &g_devfd) != 0)
		return -1;

	argc -= 2;
	argv += 2;

	for (i = 0; i < g_nr_cmds; i++) {
		if (check_cmd(i, argv[0], argc) == 0) {
			argc--;
			argv++;
			ret = g_cmds[i].do_cmd(argc, argv);
			break;
		}
	}

	if (i == g_nr_cmds) {
		fprintf(stderr, "Unknown command '%s'\n", argv[0]);
		ret = -1;
	}

	close(g_devfd);
	return ret;
}

static void
print_cmd_error(char *err_str)
{
	if (errno == ETIME) {
		fprintf(stderr, "Device is busy. Please try later\n");
	} else if (errno != 0) {
		perror("ioctl");
		fprintf(stderr, "%s\n", err_str);
	}
}

static int
wc_info(int argc, char **argv)
{
	int	ret;
	int	verbose = 0;
	char	*buffer;
	u32_t	length = (32 << 10);

	if (argc >= 1) {
		if (argv[0][0] == 'v') {
			verbose = 1;
		} else {
			fprintf(stderr, "Unknown option '%s'\n", argv[0]);
			return -1;
		}
	}

	buffer = (char *)malloc(length);
	if (buffer == NULL) {
		fprintf(stderr, "Failed to allocate memory for this command\n");
		return -1;
	}

	ret = wam_print_info(g_devfd, buffer, length, verbose);
	if (ret != 0) {
		print_cmd_error("Failed to get the device info");
		free(buffer);
		return -1;
	}

	printf("%s", buffer);
	free(buffer);

	return 0;
}

static int
wc_events(int argc, char **argv)
{
	int	ret;
	int	clear = 0;
	char	*buffer;
	u32_t	length = (128 << 10);

	if (argc >= 1) {
		if (strcasecmp(argv[0], "clear") == 0) {
			clear = 1;
		} else {
			fprintf(stderr, "Unknown option '%s'\n", argv[0]);
			return -1;
		}
	}

	buffer = (char *)malloc(length);
	if (buffer == NULL) {
		fprintf(stderr, "Failed to allocate memory for this command\n");
		return -1;
	}

	ret = wam_print_events(g_devfd, buffer, length, clear);
	if (ret != 0) {
		print_cmd_error("Failed to get the event log");
		free(buffer);
		return -1;
	}

	printf("%s", buffer);
	free(buffer);

	return 0;
}

static int
wc_ping(int argc, char **argv)
{
	int ret;

	ret = ioctl(g_devfd, NVRAM_IOC_PING_FW, 0);
	if (ret != 0) {
		fprintf(stderr, "Firmware ping failed\n");
		perror("ioctl");
		return -1;
	}

	printf("Firmware responded with Hello!\n");
	return 0;
}

/*
 * Firmware reset
 */
static int
wc_reset(int argc, char **argv)
{
	int ret;

	ret = ioctl(g_devfd, NVRAM_IOC_RESET_FW, 0);
	if (ret != 0) {
		fprintf(stderr, "Resetting device failed: %d\n", ret);
		perror("ioctl");
		return -1;
	}

	printf("Firmware Reset successful\n");
	return 0;
}

/*
 * Hard reset
 */
static int
wc_hardreset(int argc, char **argv)
{
	int ret;

	ret = ioctl(g_devfd, NVRAM_IOC_HARDRESET, 0);
	if (ret != 0) {
		fprintf(stderr, "Hard resetting failed: %d\n", ret);
		perror("ioctl");
		return -1;
	}

	printf("Hard Reset successful\n");
	return 0;
}

/*
 * Format the SSD
 */
static int
wc_ssd_format(int argc, char **argv)
{
	int ret;

	ret = ioctl(g_devfd, NVRAM_IOC_FORMAT_SSD, 0);
	if (ret != 0) {
		print_cmd_error("Failed to format device SSD");
		return -1;
	}

	printf("SSD DOM formatted\n");
	return 0;
}

static int
wc_list_errs(int argc, char **argv)
{
	int	ret;
	char	*buffer;
	u32_t	length = (128 << 10);

	buffer = (char *)malloc(length);
	if (buffer == NULL) {
		fprintf(stderr, "Failed to allocate memory for this command\n");
		return -1;
	}

	ret = wam_print_bad_blks(g_devfd, buffer, length);
	if (ret != 0) {
		print_cmd_error("Failed to get the bad blocks");
		free(buffer);
		return -1;
	}

	printf("%s", buffer);
	free(buffer);

	return 0;
}


static int
wc_acquit(int argc, char **argv)
{
	int ret;

	ret = ioctl(g_devfd, NVRAM_IOC_CLEAR_BACKUP_STATS, 0);
	if (ret != 0) {
		print_cmd_error("Failed to clear device error state");
		return -1;
	}

	printf("Backup/Restore error cleared\n");

	return 0;
}

static int
wc_ssd_update_raw(int argc, char **argv)
{
	int		ret;
	int		fw_fd;
	struct stat	filestat;
	unsigned long	fsize;
	char		*srcbuf;
	file_info_t	file_info;

	memset(&file_info, 0, sizeof(file_info_t));

	fw_fd = open(argv[0], O_RDONLY);
	if (fw_fd < 0) {
		fprintf(stderr, "Failed to open file %s\n", argv[0]);
		perror("open");
		return -1;
	}

	if (fstat(fw_fd, &filestat) != 0) {
		fprintf(stderr, "Failed to fstat file %s\n", argv[0]);
		perror("fstat");
		return -1;
	}

	fsize = filestat.st_size;
	srcbuf = (char *)mmap(NULL, fsize, PROT_READ, MAP_PRIVATE, fw_fd, 0);
	if (srcbuf == MAP_FAILED) {
		fprintf(stderr, "Failed to mmap ssd firmware file\n");
		perror("mmap");
		return -1;
	}

	file_info.file_buf = srcbuf;
	file_info.file_size = fsize;

	ret = ioctl(g_devfd, NVRAM_IOC_SSD_FW_UPDATE, &file_info);
	if (ret != 0) {
		fprintf(stderr, "Update SSD FW ioctl failed: %d\n", ret);
		perror("ioctl");
		return -1;
	}

	munmap(srcbuf, fsize);
	close(fw_fd);
	return 0;
}

static int
wc_update_eeprom_raw(int argc, char **argv)
{
	int		ret;
	int		eeprom_fd;
	struct stat	filestat;
	unsigned long	fsize;
	char		*srcbuf;
	file_info_t	file_info;

	memset(&file_info, 0, sizeof(file_info_t));

	eeprom_fd = open(argv[0], O_RDONLY);
	if (eeprom_fd < 0) {
		fprintf(stderr, "Failed to open file %s\n", argv[0]);
		perror("open");
		return -1;
	}

	if (fstat(eeprom_fd, &filestat) != 0) {
		fprintf(stderr, "Failed to fstat file %s\n", argv[0]);
		perror("fstat");
		close(eeprom_fd);
		return -1;
	}

	fsize = filestat.st_size;
	if (fsize > MV_EEPROM_SIZE) {
		printf("File size (%lu) too large for eeprom. Max size: %u\n",
				fsize, MV_EEPROM_SIZE);
		close(eeprom_fd);
		return -1;
	}

	srcbuf = mmap(NULL, fsize, PROT_READ, MAP_PRIVATE, eeprom_fd, 0);
	if (srcbuf == MAP_FAILED) {
		fprintf(stderr, "Failed to mmap eeprom file\n");
		perror("mmap");
		close(eeprom_fd);
		return -1;
	}

	file_info.file_buf = srcbuf;
	file_info.file_size = fsize;

	ret = ioctl(g_devfd, NVRAM_IOC_UPDATE_EEPROM, &file_info);
	if (ret != 0)
		printf("EEPROM Updated failed! err: %d\n", ret);

	munmap(srcbuf, fsize);
	close(eeprom_fd);

	return ret;
}

static int
wc_read_eeprom(int argc, char **argv)
{
	int		ret;
	u8_t		*buf;
	file_info_t	file_info;

	memset(&file_info, 0, sizeof(file_info_t));

	buf = (u8_t *) malloc(MV_EEPROM_SIZE);
	if (buf == NULL) {
		fprintf(stderr, "Failed to allocate memory\n");
		return -1;
	}

	file_info.file_buf = buf;
	file_info.file_size = MV_EEPROM_SIZE;

	ret = ioctl(g_devfd, NVRAM_IOC_READ_EEPROM, &file_info);
	if (ret != 0) {
		fprintf(stderr, "EEPROM read failed! err: %d\n", ret);
		free(buf);
		return -1;
	}

	ret = write(1, buf, MV_EEPROM_SIZE);

	if (ret != MV_EEPROM_SIZE) {
		fprintf(stderr, "write to stdout failed!\n");
		perror("write");
	}

	return ret;
}

static void
dump_reg_range(void *addr, u32_t start, u32_t end)
{
	u32_t i;

	for (i = start; i <= end; i += 4)
		printf("%05X: %08X\n", i, *(u32_t *)(addr + i));

	printf("\n");
}


static int
wc_regdump(int argc, char **argv)
{
	char *addr;

	/* memory map flash area */
	addr = mmap (0, NV_REG_SPACE_SIZE, PROT_READ | PROT_WRITE,
			MAP_SHARED, g_devfd, NV_MMAP_REG_OFFSET);
	if (addr == MAP_FAILED) {
		fprintf(stderr, "mmap of device memory failed\n");
		perror("mmap");
		return -1;
	}

	dump_reg_range(addr, 0x10010, 0x10010);
	dump_reg_range(addr, 0x10100, 0x1011c);
	dump_reg_range(addr, 0x1045c, 0x104e8);
	dump_reg_range(addr, 0x11010, 0x1105c);
	dump_reg_range(addr, 0x12000, 0x12100);
	dump_reg_range(addr, 0x20000, 0x20080);
	dump_reg_range(addr, 0x20100, 0x20114);
	dump_reg_range(addr, 0x20200, 0x20210);
	dump_reg_range(addr, 0x20300, 0x20324);
	dump_reg_range(addr, 0x20400, 0x2044c);
	dump_reg_range(addr, 0x20500, 0x2050c);
	dump_reg_range(addr, 0x20544, 0x2054c);
	dump_reg_range(addr, 0x31804, 0x31808);
	dump_reg_range(addr, 0x31820, 0x318c4);
	dump_reg_range(addr, 0x31820, 0x318c4);
	dump_reg_range(addr, 0x31900, 0x31910);
	dump_reg_range(addr, 0x31A00, 0x31abc);
	dump_reg_range(addr, 0x31b00, 0x31b24);
	dump_reg_range(addr, 0x80000, 0x801fc);
	dump_reg_range(addr, 0xf1400, 0xf1628);
	dump_reg_range(addr, 0x900, 0x998);
	dump_reg_range(addr, 0x70900, 0x70998);
	dump_reg_range(addr, 0x50000, 0x50004);
	dump_reg_range(addr, 0x60900, 0x60998);
	dump_reg_range(addr, 0x61900, 0x61998);

	munmap(addr, NV_REG_SPACE_SIZE);

	return 0;
}

static int
get_pwronhchktype_value(char *value_str)
{
	if (strcasecmp(value_str, "online") == 0) {
		return 0;
	} else if (strcasecmp(value_str, "offline") == 0) {
		return 1;
	} else {
		fprintf(stderr, "Invalid option: %s\n", value_str);
		return -1;
	}
}



static int
wc_scap_calib(int argc, char **argv)
{
	int ret;
	int type = 1;
	char *suffix;
	wam_info_t winfo;

	if (argc == 1) {
		if (strcasecmp("offline", (argv[0])) == 0) {
			type = 0;
		} else {
			if (strtoul(argv[0], &suffix, 10) != 0)
				fprintf(stderr, "Option deprecated. See "
						"'setattr' option for more "
						"on health check scheduling\n");
			else
				fprintf(stderr, "Invalid option: %s\n",
						argv[0]);

			return -1;
		}
	}

	if (wam_ping(g_devfd) != 0) {
		fprintf(stderr, "Device is busy. Please try later\n");
		return -1;
	}

	memset(&winfo, 0, sizeof(wam_info_t));
	if (wam_get_info(g_devfd, &winfo) != 0) {
		fprintf(stderr, "Failed to get device info.\n");
		return -1;
	}

	if (!(winfo.w_flags & WAM_STAT_SCAP_CONN)) {
		fprintf(stderr, "Supercap not connected. "
				"Cannot run health check\n");
		return -1;
	}

	if (winfo.w_flags & WAM_STAT_SCAP_CALIB_ON) {
		fprintf(stderr, "Supercap health check already in progress\n");
		return 0;
	}

	if (type && !(winfo.w_flags & WAM_STAT_SCAP_HEALTHY)) {
		fprintf(stderr, "Cannot start health measurement in "
				"online mode. Please try 'offline' mode\n");
		return -1;
	}

	ret = ioctl(g_devfd, NVRAM_IOC_SCAP_CALIB_START, type);
	if (ret != 0) {
		if (errno == EBUSY) {
			fprintf(stderr, "Supercap health check already "
					"in progress.\n");
			return 0;
		} else {
			fprintf(stderr, "Start supercap health check "
					"failed: %d\n", ret);
			perror("ioctl");
			return ret;
		}
	}

	printf("Supercap health check started. Estimated time: 10 to "
		       "30 mins\n");
	printf("Run 'wamctl %s info' to see the progress\n", g_dev_fname);
	return 0;
}

static int
wc_reg_rw(int argc, char **argv)
{
	int		ret;
	reg_io_t	reg_cfg;
	char		*next_char;

	memset(&reg_cfg, 0, sizeof (reg_io_t));

	reg_cfg.reg_addr = strtoull(argv[0], &next_char, 16);

	if (*next_char == '=' && *(next_char + 1) != 0) {
		reg_cfg.rw = 1;
		reg_cfg.reg_val = strtoull(next_char + 1, NULL, 16);
	}

	if (argc > 1) {
		if (reg_cfg.rw) {
			fprintf(stderr, "Too many arguments!\n");
			return -1;
		}
		reg_cfg.rw = 1;
		reg_cfg.reg_val = strtoull(argv[1], NULL, 16);
	}

	if (reg_cfg.reg_addr > (1 << 20)) {
		fprintf(stderr, "Invalid address: %X. Addressable range: "
				"0 to %X\n", reg_cfg.reg_addr, (1 << 20) - 1);
		return -1;
	}

	ret = ioctl(g_devfd, NVRAM_IOC_REG_IO, &reg_cfg);

	if (ret == 0)
		printf("%08X: %08X\n", reg_cfg.reg_addr, reg_cfg.reg_val);
	else
		perror("ioctl");

	return ret;
}

static wc_update_cmd_t wc_update_cmd_tbl[] = {
	{
		.u_long_cmd	= "firmware",
		.u_short_cmd	= "fw",
		.u_update_fn	= wam_update_firmware
	},
	{
		.u_long_cmd	= "ssd_firmware",
		.u_short_cmd	= "ssdfw",
		.u_update_fn	= wam_update_ssdfw
	},
	{
		.u_long_cmd	= "eeprom",
		.u_short_cmd	= "ep",
		.u_update_fn	= wam_update_eeprom
	},
	{
		.u_long_cmd	= "bootloader",
		.u_short_cmd	= "bl",
		.u_update_fn	= wam_update_uboot
	},
	{
		.u_long_cmd	= "device_config",
		.u_short_cmd	= "cfg",
		.u_update_fn	= wam_update_ddr_cfg
	},
	{
		.u_long_cmd	= "package",
		.u_short_cmd	= "pkg",
		.u_update_fn	= wam_update_package
	},
};


static wc_update_cmd_t *
wc_update_parse_cmd(char *cmd_str, char **parse_filename)
{
	int			i;
	char			*arg, *cmd;

	assert(cmd_str && parse_filename);

	cmd = strtok(cmd_str, "=");
	arg = strtok(NULL, "=");

	for (i = 0; i < NUM_OF_ELM(wc_update_cmd_tbl); i++) {
		if ((strcmp(cmd, wc_update_cmd_tbl[i].u_long_cmd) == 0) ||
		(strcmp(cmd, wc_update_cmd_tbl[i].u_short_cmd) == 0)) {
			*parse_filename = arg;
			return (&wc_update_cmd_tbl[i]);
		}
	}

	*parse_filename = arg;
	return (NULL);
}

static void
print_hidden_update_usage(void)
{
	printf("Usage:\tpackage|pkg=<package file>\t\tOR\n"
			"\tfirmware|fw=<firmware file>\t\tOR\n"
			"\tssd_firmware|ssdfw=<ssd firmware file>\tOR\n"
			"\teeprom|ep=<eeprom image file>\t\tOR\n"
			"\tbootloader|bl=<boot loader file>\tOR\n"
			"\tdevice_config|cfg=<device config file>\n");
}

/**
 * wc_update:
 * -----------
 *
 * Update firmware, ssd_firmware, eeprom, bootloader, and device_config
 *
 *
 * .usage		= "firmware|fw=<firmware file> "
 *				"ssd_firmware|ssdfw=<ssd firmware file>"
 *				" eeprom|ep=<eeprom image file> "
 *				"bootloader|bl=<boot loader file> "
 *				"device_config|cfg=<device config file>",
 */
static int
wc_update(int argc, char **argv)
{
	int			ret;
	int			force = 0;
	char			*update_file = NULL;
	wc_update_cmd_t	*update_cmd = NULL;

	if (strcasecmp(argv[0], WAM_HIDDEN_HELP_STR) == 0) {
		print_hidden_update_usage();
		return (0);
	}

	update_cmd = wc_update_parse_cmd(argv[0], &update_file);
	if (update_cmd == NULL || update_file == NULL) {
		fprintf(stderr, "Invalid command option: %s\n", argv[0]);
		return (-1);
	}

	if (argc > 1) {
		if ((strcasecmp(argv[1], "-f") == 0) ||
				(strcasecmp(argv[1], "-force") == 0)) {
			force = 1;
		} else {
			fprintf(stderr, "Unknown option %s\n", argv[1]);
			return (-1);
		}
	}

	ret = update_cmd->u_update_fn(g_devfd, update_file, force);

	return (ret);
}

static int
wc_board_id(int argc, char **argv)
{
	int		major, minor;

	if (wam_get_board_id(g_devfd, &major, &minor) == 0) {
		printf("%d.%d\n", major, minor);
		return (0);
	} else {
		printf("Unable to determine board ID\n");
		return (-1);
	}

}

static int
wc_print_vpd(int argc, char **argv)
{
	if ((argc == 1) && (strcmp(argv[0], "old") == 0)) {
		wam_vpd_print(g_devfd);
		printf("---------------------------------------------------\n");
	}
	wam_vpd_v2_print(g_devfd);
	return (0);
}

static int
wc_convert_vpd(int argc, char **argv)
{
	return (wam_convert_vpd(g_devfd));
}

static int
wc_prog_vpd(int argc, char **argv)
{
	assert(argc == 5);
	return (wam_prog_vpd(g_devfd, argc, argv));
}


static int
wc_nvsram_dump(int argc, char **argv)
{
	int		ret;
	u8_t		*buf;

	buf = (u8_t *) malloc(MV_NVSRAM_SIZE);
	if (buf == NULL) {
		fprintf(stderr, "Failed to allocate memory\n");
		return -1;
	}

	ret = wam_nvsram_dump(g_devfd, buf, MV_NVSRAM_SIZE);
	if (ret != 0) {
		fprintf(stderr, "nvsram dump api failed!\n");
		free(buf);
		return ret;
	}

	ret = write(1, buf, MV_NVSRAM_SIZE);
	if (ret != MV_NVSRAM_SIZE) {
		perror("write");
	}

	return !ret;
}

static int
wc_thermal_log(int argc, char **argv)
{
	int	ret;
	int	clear = 0;
	char	*buffer;
	u32_t	length = (128 << 10);

	if (argc >= 1) {
		if (strcasecmp(argv[0], "clear") == 0) {
			clear = 1;
		} else {
			fprintf(stderr, "Unknown option '%s'\n", argv[0]);
			return -1;
		}
	}

	buffer = (char *)malloc(length);
	if (buffer == NULL) {
		fprintf(stderr, "Failed to allocate memory for this command\n");
		return -1;
	}

	ret = wam_print_thermal_log(g_devfd, buffer, length, clear);
	if (ret != 0) {
		fprintf(stderr, "Failed to get the event log\n");
		free(buffer);
		return -1;
	}

	printf("%s", buffer);
	free(buffer);

	return 0;
}

static u32_t
get_attr_key(char *key_str)
{
	int i;

	for (i = 0; i < NUM_OF_ELM(wam_attr_db); i++) {
		if (strcmp(key_str, wam_attr_db[i].attr_str) == 0)
			return wam_attr_db[i].attr_key;
	}

	return -1;
}

static int
get_hchkschedule_value(char *value_str)
{
	u32_t period;
	char *suffix = NULL;
	char str[256];
	int err = 0;

	if (strcasecmp(value_str, "disable") == 0) {
		period = 50 * ONE_YEAR;
		goto schedattr_exit;
	}

	period = strtoul(value_str, &suffix, 10);
	if (period == 0) {
		fprintf(stderr, "Invalid time period: %s\n", value_str);
		return -1;
	}

	if (suffix == NULL || *suffix == 0) {
		fprintf(stderr, "Missing time period suffix [d|w|m]\n");
		return -1;
	}

	if (strcasecmp(suffix, "hours") == 0) {
		period *= ONE_HOUR;
		goto schedattr_exit;
	}

	switch (*suffix) {
	case 'd':
	case 'D':
		period *= ONE_DAY;
		break;

	case 'w':
	case 'W':
		period *= ONE_WEEK;
		break;

	case 'm':
	case 'M':
		period *= ONE_MONTH;
		break;

	default:
		err = 1;
		break;
	}

	suffix++;
	if (err || (*suffix != 0)) {
		fprintf(stderr, "Invalid time suffix: %s\n", value_str);
		return -1;
	}

schedattr_exit:
	wam_relative_time2str(period, str, 256);
	fprintf(stderr, "Setting health check schedule to once in: %s\n", str);
	return period;
}

static int
wc_set_attr(int argc, char **argv)
{
	int i;
	char *key_str;
	char *value_str;
	u32_t key;
	u32_t value;

	for (i = 0; i < argc; i++) {
		key_str = argv[i];
		value_str = strchr(key_str, '=');
		if (value_str == NULL) {
			fprintf(stderr, "Invalid option %s\n", argv[i]);
			continue;
		}

		*value_str = 0;
		value_str++;

		key = get_attr_key(key_str);
		if (key == -1) {
			fprintf(stderr, "Invalid key %s\n", argv[i]);
			continue;
		}

		switch (key) {
		case WA_OFFLINE_HCHK_ON_COLDBOOT:
			value = get_pwronhchktype_value(value_str);
			if (value == -1)
				continue;
			break;

		case WA_SCAP_HEALTHCHECK_SCHEDULE:
			value = get_hchkschedule_value(value_str);
			if (value == -1)
				continue;
			break;

		case WA_THERMAL_ALERT_TEMP:
			value = strtoul(value_str, NULL, 10);
			break;

		default:
			fprintf(stderr, "Invalid key %s\n", argv[i]);
			continue;
		}

		if (wam_set_attr(g_devfd, key, value) != 0) {
			fprintf(stderr, "Failed to set attr %s\n", key_str);
			perror("ioctl");
			continue;
		}
	}

	return 0;
}

static int
wc_get_attr(int argc, char **argv)
{
	int i;
	char *value_str;
	u32_t key;
	u32_t value;
	char str[256];

	for (i = 0; i < argc; i++) {
		key = get_attr_key(argv[i]);
		if (key == -1) {
			fprintf(stderr, "Invalid key %s\n", argv[i]);
			continue;
		}

		if (wam_get_attr(g_devfd, key, &value) != 0) {
			fprintf(stderr, "Failed to get attr %s\n", argv[i]);
			perror("ioctl");
			continue;
		}

		printf("%s = ", argv[i]);

		switch (key) {
		case WA_OFFLINE_HCHK_ON_COLDBOOT:
			if (value)
				value_str = "offline";
			else
				value_str = "online";
			printf("%s\n", value_str);
			break;

		case WA_SCAP_HEALTHCHECK_SCHEDULE:
			wam_relative_time2str(value, str, 256);
			printf("%s\n", str);
			break;

		case WA_THERMAL_ALERT_TEMP:
			printf("%u C\n", value);
			break;

		default:
			fprintf(stderr, "Invalid key %s\n", argv[i]);
			continue;
		}

	}

	return 0;
}
