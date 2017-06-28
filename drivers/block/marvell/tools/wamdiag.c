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
#include "wamlib_priv.h"

char *exe_str;


char * scap_status_str[] = {
	"Not connected",
	"Charging",
	"Charging complete",
	"Discharging",
	"Degraded",
};

char * ssd_status_str[] = {
	"Not connected",
	"Erase in progress",
	"Connected",
	"Not responding",
	"IO Failing",
	"Erase pending",
	"End Of Life Reached",
};

char * post_status_str[] = {
	"POST Skipped",
	"POST Passed",
	"POST Failed"
};

enum {
	PMBUS_VOUT_MODE,
	PMBUS_LITERAL,
	PMBUS_VOUT_LINEAR,
	PMBUS_CUSTOM,
};

typedef struct {
	char	*cmd_name;
	char	*unit;
	u8_t	cmd;
	u8_t	bytes;
	char	sign;
	u8_t	format;
} scap_chrgr_cmd_set_t; 

scap_chrgr_cmd_set_t chrgr_cmd_set[] = {
	{ "vout_mode",		"--", 0x20, 1, 'u', PMBUS_VOUT_MODE },
	{ "vout_command",	"mV", 0x21, 2, 'u', PMBUS_VOUT_LINEAR },
	{ "vout_trim",		"mV", 0x22, 2, 's', PMBUS_VOUT_LINEAR },
	{ "vout_cal_offset",	"mV", 0x23, 2, 's', PMBUS_VOUT_LINEAR },
	{ "vout_max",		"mV", 0x24, 2, 'u', PMBUS_VOUT_LINEAR },
	{ "vout_margin_high",	"mV", 0x25, 2, 'u', PMBUS_VOUT_LINEAR },
	{ "vout_margin_low",	"mV", 0x26, 2, 'u', PMBUS_VOUT_LINEAR },
	{ "vout_transition",	"mV/ms", 0x27, 2, 'u', PMBUS_LITERAL },
	{ "read_vin",		"mV", 0x88, 2, 'u', PMBUS_LITERAL },
	{ "read_vout",		"mV", 0x8b, 2, 'u', PMBUS_VOUT_LINEAR },
	{ "read_iout",		"mA", 0x8c, 2, 'u', PMBUS_LITERAL },
	{ "vout_ov_limit",	"mV", 0x40, 2, 'u', PMBUS_VOUT_LINEAR },
	{ "vout_uv_limit",	"mV", 0x44, 2, 'u', PMBUS_VOUT_LINEAR },
	{ "iout_oc_limit",	"mA", 0x46, 2, 'u', PMBUS_LITERAL},
};

typedef struct {
	char    *cmd_name;
	u8_t    cmd;
	u8_t    bytes;
	char	**status_strings;
} scap_chrgr_status_set_t;

static char *status_bit_reason[] = {
	"Other fault not listed in bits[7:1]",
	"Communications, memory or logic fault",
	"Temperature fault",
	"VIN undervoltage fault",
	"IOUT overcurrent fault",
	"VOUT overvoltage fault",
	"The charger is OFF and not providing power",
	"The device is busy",
	"Unknown fault",
	"Rsvd",
	"Rsvd",
	"Power good signal negated",
	"Manufacturer specific fault",
	"Input current/voltage fault or warning",
	"Output current fault or warning",
	"Output voltage fault or warning",
};

static char *status_vout_reason[] = {
	"Rsvd",
	"TOFF_MAX_FAULT or VOUT Tracking Error on PDOWN",
	"TON_MAX_FAULT or VOUT Tracking Error on PUP",
	"VOUT_MAX Warning - Trying to set VOUT > VOUT_MAX",
	"VOUT Undervoltage fault",
	"VOUT Undervoltage warning",
	"VOUT Overvoltage warning",
	"VOUT Overvoltage fault"
};

static char *status_iout_reason[] = {
	"Rsvd",
	"Rsvd",
	"Power Limiting - Unit operating with output in constant power mode",
	"Current share fault",
	"IOUT Undercurrent fault",
	"IOUT Overcurrent warning",
	"IOUT Overcurrent and Low voltage shutdown fault",
	"IOUT Overcurrent fault"
};

static char *status_input_reason[] = {
	"Rsvd",
	"I_IN Overcurrent warning",
	"I_IN Overcurrent fault",
	"Unit is OFF for insufficient input voltage",
	"VIN Undervoltage fault",
	"VIN Undervoltage warning",
	"VIN Overvoltage warning",
	"VIN Overvoltage fault"
};

static char *status_temp_reason[] = {
	"Rsvd",
	"Rsvd",
	"Rsvd",
	"Rsvd",
	"Undertemperature fault",
	"Undertemperature warning",
	"Overtemperature warning",
	"Overtemperature fault"
};

static char *status_cml_reason[] = {
	"Unknown fault",
	"Unknown Comm fault",
	"Rsvd",
	"Processor fault",
	"Memory fault",
	"Packet error check fail",
	"Invalid or Unsupported Data Received",
	"Invalid or Unsupported Command Received",
};

static char *status_mfr_reason[] = {
	"VMON OV fault",
	"VMON UV fault",
	"Rsvd",
	"TSW",
	"VMON OV Warning",
	"VMON UV Warning",
	"Rsvd",
	"Rsvd",
};

static scap_chrgr_status_set_t status_cmd_set[] = {
	{ "status_word",	0x79, 2,  status_bit_reason },
	{ "status_vout",	0x7A, 1,  status_vout_reason },
	{ "status_iout",	0x7b, 1,  status_iout_reason },
	{ "status_input",	0x7c, 1,  status_input_reason },
	{ "status_temp",	0x7d, 1,  status_temp_reason },
	{ "status_cml",		0x7e, 1,  status_cml_reason },
	{ "status_mfr",		0x80, 1,  status_mfr_reason },
};


static void
print_usage(void)
{
	printf("Usage: %s <device file> i|b|f|e|s|d|p|v|g|c|q|z|w\n"
			"I - Device Info\n"
			"i <register in hex> [value in hex] "
			"- set/read the register\n"
			"b - backup ddr contents to ssd\n"
			"f - format the SSD\n"
			"e <on/off> [ecc byte in hex] - "
			"enable/disable forced ECC\n"
			"s - display supercap voltage\n"
			"d - dump driver dma pointers\n"
			"p - ping the firmware\n"
			"v - print firmware version\n"
			"g - get backup statistics\n"
			"l - list error blocks\n"
			"c - clear backup stats (in case of backup error)\n"
			"q - query data validity\n"
			"t - dump ssd identify info\n"
			"u <SSD firmware image> - update ssd firmware\n"
			"z - get event log\n"
			"w - write ready check\n"
			"C - start supercap calibration\n"
			"R - reset firmware\n"
			"V - Create VG images - <uloader> <firmware> <out>\n"
			"k <r|w> <nr of ios> <request size> <hex byte> "
			"- kernel IO API test\n"
			"x - dump supercap charger settings\n"
			"S <vol in millivolts>\n"
			"D <id> - Set DDR voltage, id can be 0 to 3, "
			"0: 1.62V, 1: 1.75V, 2: 1.8V, 3: 1.98V\n"
			"E <error nr> - Inject Error\n"
			"\t1 - Supercap Disconnect\n"
			"\t2 - Supercap Low Voltage\n"
			"\t3 - Supercap Degrade Error\n"
			"\t4 - SSD Detection Issue\n"
			"\t5 - SSD Failure\n"
			"\t6 - SSD End Of Life Issue\n", exe_str);
}

/*
 * Read or Write a register
 */
static int
rw_reg(int fd, int argc, char **argv)
{
	int         ret;
	reg_io_t    reg_cfg;

	if (argc < 3) {
		print_usage();
		return -1;
	}

	memset(&reg_cfg, 0, sizeof(reg_io_t));

	reg_cfg.reg_addr = strtoull(argv[2], NULL, 16);

	if (argc > 3) {
		reg_cfg.rw = 1;
		reg_cfg.reg_val = strtoull(argv[3], NULL, 16);
	}

	ret = ioctl(fd, NVRAM_IOC_REG_IO, &reg_cfg);

	if (ret == 0) {
		printf("%08X: %08X\n", reg_cfg.reg_addr, reg_cfg.reg_val);
	} else {
		printf("Invalid address: %X. Addressable range: 0 to %X\n",
				reg_cfg.reg_addr, (1 << 20) - 1);
	}

	return ret;
}


static u32_t
pmbus_decode_literal(u16_t val)
{
	int	n;
	u32_t	x;

	x = (val & 0x7FF);
	x *= 1000;

	n = (val >> 11);
	if (n & 0x10)
		n |= 0xFFFFFFF0;

	if (n < 0)
		x >>= (-n);
	else
		x <<= n;

	return x;
}

static u32_t
pmbus_decode_linear(u16_t val)
{
	u32_t	x;

	x = val * 1000;
	x >>= 13;

	return x;
}

static int
scap_chrgr_read(int fd, u8_t cmd, u8_t nr_bytes, u16_t *data)
{
	int			ret;
	scap_chrgr_cmd_t	chrgr_cmd;

	memset(&chrgr_cmd, 0, sizeof(scap_chrgr_cmd_t));

	chrgr_cmd.cmd = cmd;
	chrgr_cmd.nr_bytes = nr_bytes;

	ret = ioctl(fd, NVRAM_IOC_CHARGER_READ, &chrgr_cmd);

	if (ret != 0) {
		printf("read from charger failed. cmd: %02x, bytes: %u\n",
				cmd, nr_bytes);
	}

	*data = chrgr_cmd.data;

	return ret;
}


int
scap_chrgr_set(int fd, u8_t cmd, u8_t nr_bytes, u16_t data)
{
	int			ret;
	scap_chrgr_cmd_t	chrgr_cmd;

	memset(&chrgr_cmd, 0, sizeof(scap_chrgr_cmd_t));

	chrgr_cmd.cmd = cmd;
	chrgr_cmd.nr_bytes = nr_bytes;
	chrgr_cmd.data = data;

	ret = ioctl(fd, NVRAM_IOC_CHARGER_SET, &chrgr_cmd);

	if (ret != 0) {
		printf("writing to charger failed. cmd: %02x, bytes: %u, "
				"data: %04x\n", cmd, nr_bytes, data);
	}

	return ret;
}

static void
scap_chrgr_print_status(scap_chrgr_status_set_t *status_ent, u32_t val)
{
	int i;

	if (val)
		printf("\n");

	printf("%-20s: 0x%02x\n", status_ent->cmd_name, val);

	for (i = 0; i < status_ent->bytes * 8; i++) {
		if (val & (1 << i))
			printf("    [%2u]: %s\n", i,
					status_ent->status_strings[i]);
	}

	if (val)
		printf("\n");
}

static int
scap_chrgr_status_dump(int fd)
{
	u32_t	i;
	u32_t	nr_cmds = sizeof(status_cmd_set)/sizeof(scap_chrgr_status_set_t);
	int	ret;
	u16_t	val = 0;

	for (i = 0; i < nr_cmds; i++) {
		ret = scap_chrgr_read(fd, status_cmd_set[i].cmd,
				status_cmd_set[i].bytes,
				&val);

		if (ret != 0)
			return ret;

		scap_chrgr_print_status(&status_cmd_set[i], val);
	}

	return 0;
}

static int
scap_chrgr_dump(int fd, int argc, char **argv)
{
	u32_t	i;
	u32_t	nr_cmds = sizeof(chrgr_cmd_set)/sizeof(scap_chrgr_cmd_set_t);
	int	ret;
	u32_t	vol;
	u16_t	val = 0;

	if (argc > 2 && argv[2][0] == 'c') {
		printf("Sending clear faults command!\n");
		ret = scap_chrgr_set(fd, 0x7B, 0, 0);
		if (ret != 0) {
			printf("clear faults failed!\n");
			return ret;
		}

		printf("faults cleared!\n");
	}

	for (i = 0; i < nr_cmds; i++) {
		ret = scap_chrgr_read(fd, chrgr_cmd_set[i].cmd,
				chrgr_cmd_set[i].bytes,
				&val);

		if (ret != 0)
			return ret;

		printf("%-20s: ", chrgr_cmd_set[i].cmd_name);
		switch (chrgr_cmd_set[i].format) {
		case PMBUS_VOUT_LINEAR:
			vol = pmbus_decode_linear(val);
			printf("%u %s", vol, chrgr_cmd_set[i].unit);
			break;

		case PMBUS_LITERAL:
			vol = pmbus_decode_literal(val);
			printf("%u %s", vol, chrgr_cmd_set[i].unit);
			break;

		default:
			printf("0x%02x", val);
			break;
		}

		printf("\n");
	}

	ret = scap_chrgr_status_dump(fd);

	return ret;
}

/*
 * Set Supercap charging voltage 
 */
static int
scap_chrgr_set_vol(int fd, int argc, char **argv)
{
	int ret;
	u32_t vol;

	if (argc < 3) {
		print_usage();
		return -1;
	}

	vol = strtoul(argv[2], NULL, 10);
	if (vol > 5500 || vol < 3000) {
		printf("Supercap charging voltage[%u] should in "
				"the range of 3000 mV to 5500 mV\n",
				vol);
		return -1;
	}

	ret = ioctl(fd, NVRAM_IOC_CHARGER_SET_VOL, vol);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	return 0;
}

/*
 * Set Supercap charging voltage 
 */
static int
ddr_select_vol(int fd, int argc, char **argv)
{
	int ret;
	u32_t id;

	if (argc < 3) {
		print_usage();
		return -1;
	}

	id = strtoul(argv[2], NULL, 10);
	if (id > 3) {
		printf("Not a valid voltage selector: %s "
				"(valid values: 0 to 3)\n", argv[2]);
		return -1;
	}

	ret = ioctl(fd, NVRAM_IOC_DDR_SEL_VOL, id);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	return 0;
}

/*
 * Inject Error
 */
static int
inject_error(int fd, int argc, char **argv)
{
	int ret;
	u32_t err;

	if (argc < 3) {
		print_usage();
		return -1;
	}

	err = strtoul(argv[2], NULL, 10);
	if (err >= WAM_INJE_INV_DATA) {
		printf("Not a valid selection: %s "
				"(valid values: 1 to 6)\n", argv[2]);
		return -1;
	}

	ret = ioctl(fd, NVRAM_IOC_INJ_ERR, err);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	return 0;
}

/*
 * Read supercap register
 */
static int
nvram_read_scap_reg(int fd, int argc, char **argv)
{
	int ret;
	int i;
	reg_io_t    reg_cfg;

	for (i = 0; i < 8; i++) {
		memset(&reg_cfg, 0, sizeof(reg_io_t));

		reg_cfg.reg_addr = i;

		ret = ioctl(fd, NVRAM_IOC_READ_SCAP_REG, &reg_cfg);
		if (ret != 0) {
			printf("Driver returned error: %d\n", ret);
			return -1;
		}

		printf("ADC Channel [%u]: %u mV\n", i, reg_cfg.reg_val);
	}

	return 0;
}


/*
 * Read supercap voltage
 */
static int
nvram_get_scap_vol(int fd)
{
	int ret;
	scap_vol_t    vol;

	memset(&vol, 0, sizeof(scap_vol_t));

	ret = ioctl(fd, NVRAM_IOC_GET_SCAP_VOL, &vol);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("Supercap current voltage: %u mV, Expected Max: %u mV\n",
		       vol.sc_cur_vol, vol.sc_exp_vol);
	return 0;
}

static int
nvram_dma_dump(int fd)
{
	int ret;
	char *buf;

	buf = (char *) malloc(MV_DMA_DUMP_PAGE_SIZE);
	memset(buf, 0, MV_DMA_DUMP_PAGE_SIZE);

	ret = ioctl(fd, NVRAM_IOC_DMA_DUMP, buf);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("%s\n", buf);
	return 0;
}

static int
nvram_ping_fw(int fd)
{
	int ret;

	ret = ioctl(fd, NVRAM_IOC_PING_FW, 0);
	if (ret != 0) {
		printf("Firmware ping failed\n");
		return -1;
	}

	printf("Firmware responded with Hello!\n");
	return 0;
}


/*
 * Get firmware version
 */
static int
nvram_get_fw_ver(int fd)
{
	int ret;
	char buf[4096];

	memset(buf, 0, 4096);

	ret = ioctl(fd, NVRAM_IOC_FW_VERSION, buf);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("Firmware version: %s\n", buf);
	return 0;
}

#ifdef CONFIG_ENABLE_CONFIRMATION
int
get_confirmation(char *cmd)
{
	char str[8];
	int ret;

	printf("Do you want to continue with %s? (yes/no): ", cmd);

	ret = scanf("%7s", str);

	if (ret != 1 || strncmp(str, "yes", 4) != 0)
		return -1;

	return 0;
}
#else

int
get_confirmation(char *cmd)
{
	return 0;
}

#endif
/*
 * Flush the contents of ddr to ssd
 */
static int
nvram_backup(int fd)
{
	int ret;

	if (get_confirmation("DRAM backup") != 0) {
		return -1;
	}

	ret = ioctl(fd, NVRAM_IOC_START_BACKUP, 0);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("WAM Backup successful\n");
	return 0;
}

/*
 * Format the SSD
 */
static int
ssd_format(int fd)
{
	int ret;

	if (get_confirmation("DOM format") != 0) {
		return -1;
	}

	ret = ioctl(fd, NVRAM_IOC_FORMAT_SSD, 0);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("WAM SSD format successful\n");
	return 0;
}


/*
 * Trim the SSD
 */
static int
ssd_trim(int fd)
{
	int ret;

	ret = ioctl(fd, NVRAM_IOC_SSD_TRIM, 0);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("WAM SSD Trimmed\n");
	return 0;
}


static void
ata_id_to_string(u16_t *ident, char *str, u32_t len)
{
	u16_t c;
	u16_t i = 0;

	while (len) {
		c = ident[i];
		*str = c >> 8;
		str++;

		c = ident[i];
		*str = c & 0xFF;
		str++;

		i++;
		len--;
	}
}

static int
ssd_identify(int fd)
{
	char buf[512];
	int ret;
	ata_identify_data_t *ident;
	char str[64];

	memset(buf, 0, 512);

	ret = ioctl(fd, NVRAM_IOC_SSD_IDENTIFY, buf);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	ident = (ata_identify_data_t *)buf;
	printf("General Config\t\t: %04x\n", ident->general_config);

	ata_id_to_string((u16_t *)ident->serial_number, str, 10);
	str[20] = 0;
	printf("Serial Number\t\t: %s\n", str);

	ata_id_to_string((u16_t *)ident->firmware_revision, str, 4);
	str[8] = 0;
	printf("Firmware Rev\t\t: %s\n", str);

	ata_id_to_string((u16_t *)ident->model_number, str, 20);
	str[40] = 0;
	printf("Model Number\t\t: %s\n", str);

	printf("Max Block Txfr\t\t: %04x\n", ident->maximum_block_transfer);
	printf("Capabilities\t\t: %04x\n", ident->capabilities[0]);
	printf("Queue Depth\t\t: %u\n", ident->queue_depth);

	return 0;
}


int ssd_updatefw(int fd, int argc, char **argv)
{
	int		ret;
	int		fw_fd;
	struct stat	filestat;
	unsigned long	fsize;
	char		*srcbuf;
	file_info_t	file_info;

	argc--;
	argv++;

	memset(&file_info, 0, sizeof(file_info_t));

	if (argc < 2) {
		printf("firmware image file missing\n");
		return -1;
	}

	fw_fd = open(argv[1], O_RDONLY);
	if (fw_fd < 0) {
		printf("Failed to open file %s\n", argv[1]);
		perror("open");
		return -1;
	}


	if (fstat(fw_fd, &filestat) != 0) {
		printf("Failed to fstat file %s\n", argv[1]);
		perror("fstat");
		return -1;
	}

	fsize = filestat.st_size;
	srcbuf = (char *)mmap(NULL, fsize, PROT_READ, MAP_PRIVATE, fw_fd, 0);
	if (srcbuf == MAP_FAILED) {
		printf("Failed to mmap firmware file\n");
		perror("mmap");
		return -1;
	}

	file_info.file_buf = srcbuf;
	file_info.file_size = fsize;

	ret = ioctl(fd, NVRAM_IOC_SSD_FW_UPDATE, &file_info);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	munmap(srcbuf, fsize);
	close(fw_fd);
	return 0;
}

static int
scap_calibrate_start(int fd)
{
	int ret;

	ret = ioctl(fd, NVRAM_IOC_SCAP_CALIB_START, 0);
	if (ret != 0)
		printf("Driver returned error: %d\n", ret);

	printf("Supercap calibration initiated. Estimated time: 7 to 30 mins\n");
	printf("run wamdiag /dev/mvwamX I: to check the calibration progress/completion\n");

	return ret;
}



static int
wam_info(int fd)
{
	int	ret;
	int	verbose = 0;
	char	*buffer;
	u32_t	length = (8 << 10);

	buffer = (char *)malloc(length);
	if (buffer == NULL) {
		fprintf(stderr, "Failed to allocate memory for this command\n");
		return -1;
	}

	ret = wam_print_info(fd, buffer, length, verbose);
	if (ret != 0) {
		fprintf(stderr, "Failed to get the device info\n");
		free(buffer);
		return -1;
	}

	printf("%s", buffer);
	free(buffer);
	return 0;
}

/*
 * Firmware reset
 */
static int
nvram_reset(int fd)
{
	int ret;

	if (get_confirmation("Reset Firmware") != 0) {
		return -1;
	}

	ret = ioctl(fd, NVRAM_IOC_RESET_FW, 0);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("Firmware Reset successful\n");
	return 0;
}



/*
 * Restore DDR by reading from SSD
 */
static int
nvram_restore(int fd)
{
	int ret;

	if (get_confirmation("DRAM restore") != 0) {
		return -1;
	}

	ret = ioctl(fd, NVRAM_IOC_START_RESTORE, 0);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("WAM Restore successful\n");
	return 0;
}


/*
 * Enable or Disable forced ECC
 */
static int
forced_ecc(int fd, int argc, char **argv)
{
	int ret;
	int ecc_byte = 0;
	int cmd;

	if (argc < 3) {
		printf("arguments missing\n");
		print_usage();
		return -1;
	}

	if (strcasecmp(argv[2], "on") == 0) {
		if (argc < 4) {
			printf("missing ecc byte argument\n");
			print_usage();
			return -1;
		}

		ecc_byte = strtoull(argv[3], NULL, 16);
		ecc_byte &= 0xFF;
		cmd = NVRAM_IOC_EN_FORCED_ECC;

		printf("Enabling forced ECC with byte %02x\n",
				ecc_byte);
	} else {
		cmd = NVRAM_IOC_DIS_FORCED_ECC;
		printf("Disabling forced ECC\n");
	}

	ret = ioctl(fd, cmd, ecc_byte);

	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	return 0;
}


static int
get_backup_stats(int fd)
{
	int ret;
	backup_stats_t	stats;
	u64_t ddr_offset;
	int i;

	memset(&stats, 0, sizeof(backup_stats_t));

	ret = ioctl(fd, NVRAM_IOC_GET_BACKUP_STATS, &stats);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	ddr_offset = stats.b_ddr_offset;
	ddr_offset <<= 3;

	printf("Backup status\t\t: %s\n",
			stats.b_completed ? "Completed" : "INCOMPLETE");
	printf("IO Error count\t\t: %u\n", stats.b_err_count);
	printf("DDR Offset\t\t: %llu MB\n", ddr_offset >> 20);

	for (i = 0; i < 4; i++) {
		printf("Progress %u%%\t\t: %u s\tMin IO Time: %u us\t"
				"Max IO Time: %u us\n", i * 25,
				stats.b_start_time[i],
				stats.b_min_time[i],
				stats.b_max_time[i]);
	}

	printf("SSD WC Flush time\t: %u s\n", stats.b_ssd_flush_time);
	printf("Backup end time\t\t: %u s\n", stats.b_end_time);
	printf("Supercap Alive time\t: %u s\n", stats.b_alive_time);
	printf("Supercap start vol\t: %u mV\n", stats.b_start_scap_vol);
	printf("Supercap end vol\t: %u mV\n", stats.b_end_scap_vol);
	printf("Single Bit ECC count\t: %u\n", stats.b_sbe_cnt);
	printf("Double Bit ECC count\t: %u\n", stats.b_dbe_cnt);

	return 0;
}


static int
get_err_blks(int fd)
{
	int ret;
	err_block_t *eblks;
	u64_t ddr_offset;
	int i;

	eblks = (err_block_t *) malloc(4096);
	memset(eblks, 0, 4096);

	ret = ioctl(fd, NVRAM_IOC_GET_ERR_BLOCKS, eblks);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	i = 0;
	while (eblks[i].e_blk_size != 0 || eblks[i].e_qword_off != 0) {
		ddr_offset = eblks[i].e_qword_off;
		ddr_offset <<= 3;
		printf("[%u] offset: 0x%010llx size: 0x%08x, op: %u\n", i,
				ddr_offset, eblks[i].e_blk_size,
				eblks[i].e_err_op);
		i++;
	}

	printf("Device recorded %u errors\n", i);
	return 0;
}


static int
clear_backup_stats(int fd)
{
	int ret;

	ret = ioctl(fd, NVRAM_IOC_CLEAR_BACKUP_STATS, 0);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	printf("Backup statistics record cleared\n");

	return 0;
}

static int
query_data_validity(int fd)
{
	int ret;
	u64_t status;

	ret = ioctl(fd, NVRAM_IOC_IS_DATA_VALID, &status);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	if ((status >> 32) == 0) {
		printf("Data is corrupt... ");
		if (status & 0xFFFF) {
			printf("Restore error! (%04x)\n",
					(u32_t)(status & 0xFFFF));
		} else {
			printf("Backup error (%04x)\n",
					(u32_t)((status >> 16) & 0xFFFF));
		}
		return -1;
	}

	printf("Data is valid... No backup/restore errors\n");
	return 0;
}

static int
query_write_readiness(int fd)
{
	int ret = -1;
	u64_t status;
	u32_t tmp;
	wam_status_t *dev_stat;

	ret = ioctl(fd, NVRAM_IOC_WRITE_READY, &status);
	if (ret != 0) {
		printf("Driver returned error: %d\n", ret);
		return -1;
	}

	if ((status >> 32) == 0) {
		printf("NVRAM not ready for write commands\n");
	} else {
		printf("NVRAM ready for write commands\n");
		ret = 0;
	}

	tmp = (u32_t)status;
	dev_stat = (wam_status_t *)&tmp;

	if (dev_stat->w_post_res == DRAM_POST_FAILED ||
			(dev_stat->w_ssd_stat != SSD_ERASE_IN_PROGRESS &&
			 dev_stat->w_ssd_stat != SSD_CONNECTED) ||
			dev_stat->w_scap_stat == SCAP_NOT_CONNECTED) {
		ret = -1;
	} else {
		/* if not ready but charging */
		if (ret != 0)
			ret = 1;
	}

	printf("Supercap status\t\t: %s[%u]\n",
			scap_status_str[dev_stat->w_scap_stat],
			dev_stat->w_scap_stat);
	printf("Supercap charge\t\t: %u%%\n",
			dev_stat->w_scap_charge);
	printf("SSD Status\t\t: %s\n",
			ssd_status_str[dev_stat->w_ssd_stat]);
	printf("Memory Test Status\t: %s\n",
			post_status_str[dev_stat->w_post_res]);

	return ret;
}

static int
dump_event_log(int fd)
{
	int	ret;
	int	clear = 1;
	char	*buffer;
	u32_t	length = (128 << 10);

	buffer = (char *)malloc(length);
	if (buffer == NULL) {
		fprintf(stderr, "Failed to allocate memory for this command\n");
		return -1;
	}

	ret = wam_print_events(fd, buffer, length, clear);
	if (ret != 0) {
		fprintf(stderr, "Failed to get the event log\n");
		free(buffer);
		return -1;
	}

	printf("%s", buffer);
	free(buffer);

	return 0;
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

/*
 * WAM Kernel IO API Test
 */
static int
wam_io_test(int fd, int argc, char **argv)
{
	int		ret;
	wam_iotest_t	info;

	if (argc < 6) {
		print_usage();
		return -1;
	}

	memset(&info, 0, sizeof(wam_iotest_t));

	switch (argv[2][0]) {
	case 'R':
	case 'r':
		info.rw = 0;
		break;

	case 'w':
	case 'W':
		info.rw = 1;
		break;

	default:
		printf("Invalid op type %s\n", argv[1]);
		return -1;
	}


	info.nr_ios = strtoul(argv[3], NULL, 10);
	if (info.nr_ios == 0) {
		printf("Invalid number of IO requests %s\n", argv[3]);
		return -1;
	}

	info.req_size = (u32_t)get_bytes(argv[4]);
	if (info.req_size == 0) {
		printf("Invalid IO Size %s\n", argv[4]);
		return -1;
	}

	info.offset = 0;
	info.pattern = strtoul(argv[5], NULL, 16);

	ret = ioctl(fd, NVRAM_IOC_WAM_IOTEST, &info);

	if (ret == 0) {
		printf("IO test completed\n");
	} else {
		printf("IO test failed\n");
		perror("io test:");
	}

	return ret;
}

static int
create_vg_image(char **argv, int argc)
{
	printf("creating vg image with %s %s, output %s\n",
		argv[2], argv[3], argv[4]);
	if (argc != 5) {
		print_usage();
		return (-1);
	}

	return (wam_create_vgimg(argv[2], argv[3], argv[4]));
}

int
main(int argc, char **argv)
{
	int fd, ret;

	exe_str = argv[0];

	if (argc < 3) {
		print_usage();
		return -1;
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		printf("Failed to open device %s\n", argv[1]);
		return -1;
	}

	argc--;
	argv++;

	switch (argv[1][0]) {
		case 'i':
			ret = rw_reg(fd, argc, argv);
			break;

		case 'b':
			ret = nvram_backup(fd);
			break;

		case 'f':
			ret = ssd_format(fd);
			break;

		case 'e':
			ret = forced_ecc(fd, argc, argv);
			break;

		case 'r':
			ret = nvram_restore(fd);
			break;

		case 'a':
			ret = nvram_read_scap_reg(fd, argc, argv);
			break;

		case 's':
			ret = nvram_get_scap_vol(fd);
			break;

		case 'd':
			ret = nvram_dma_dump(fd);
			break;

		case 'p':
			ret = nvram_ping_fw(fd);
			break;

		case 'v':
			ret = nvram_get_fw_ver(fd);
			break;

		case 'g':
			ret = get_backup_stats(fd);
			break;

		case 'c':
			ret = clear_backup_stats(fd);
			break;

		case 'q':
			ret = query_data_validity(fd);
			break;

		case 'w':
			ret = query_write_readiness(fd);
			break;

		case 'l':
			ret = get_err_blks(fd);
			break;

		case 't':
			ret = ssd_identify(fd);
			break;

		case 'u':
			ret = ssd_updatefw(fd, argc, argv);
			break;

		case 'z':
			ret = dump_event_log(fd);
			break;

		case 'T':
			ret = ssd_trim(fd);
			break;

		case 'I':
			ret = wam_info(fd);
			break;

		case 'C':
			ret = scap_calibrate_start(fd);
			break;

		case 'R':
			ret = nvram_reset(fd);
			break;

		case 'k':
			ret = wam_io_test(fd, argc, argv);
			break;

		case 'x':
			ret = scap_chrgr_dump(fd, argc, argv);
			break;

		case 'S':
			ret = scap_chrgr_set_vol(fd, argc, argv);
			break;

		case 'D':
			ret = ddr_select_vol(fd, argc, argv);
			break;

		case 'E':
			ret = inject_error(fd, argc, argv);
			break;

		case 'V':
			ret = create_vg_image(argv, argc);
			break;

		default:
			print_usage();
			break;
	}

	close(fd);

	return ret;
}

