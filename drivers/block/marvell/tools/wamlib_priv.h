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

#ifndef _MV_WAMLIB_PRIV_H_
#define _MV_WAMLIB_PRIV_H_

#include <mv_types.h>

typedef struct _ata_identify_data {
	u16_t general_config;				/*	0	*/
	u16_t obsolete0;				/*	1	*/
	u16_t specific_config;				/*	2	*/
	u16_t obsolete1;				/*	3	*/
	u16_t retired0[2];				/*	4-5	*/
	u16_t obsolete2;				/*	6	*/
	u16_t reserved0[2];				/*	7-8	*/
	u16_t retired1;					/*	9	*/
	u8_t serial_number[20];			/*	10-19	*/
	u16_t retired2[2];				/*	20-21	*/
	u16_t obsolete3;				/*	22	*/
	u8_t firmware_revision[8];			/*	23-26	*/
	u8_t model_number[40];				/*	27-46	*/
	u16_t maximum_block_transfer;			/*	47	*/
	u16_t reserved1;				/*	48	*/
	u16_t capabilities[2];				/*	49-50	*/
	u16_t obsolete4[2];				/*	51-52	*/
	u16_t fields_valid;				/*	53	*/
	u16_t obsolete5[5];				/*	54-58	*/
	u16_t current_multiple_sector_setting;		/*	59	*/
	u16_t user_addressable_sectors[2];		/*	60-61	*/
	u16_t atapi_dmadir;				/*	62	*/
	u16_t multiword_dma_modes;			/*	63	*/
	u16_t pio_modes;				/*	64	*/
	u16_t minimum_multiword_dma_cycle_time;		/*	65	*/
	u16_t recommended_multiword_dma_cycle_time;	/*	66	*/
	u16_t minimum_pio_cycle_time;			/*	67	*/
	u16_t minimum_pio_cycle_time_iordy;		/*	68	*/
	u16_t reserved2[2];				/*	69-70	*/
	u16_t atapi_reserved[4];			/*	71-74	*/
	u16_t queue_depth;				/*	75	*/
	u16_t sata_capabilities;			/*	76	*/
	u16_t sata_reserved;				/*	77	*/
	u16_t sata_feature_supported;			/*	78	*/
	u16_t sata_feature_enabled;			/*	79	*/
	u16_t major_version;				/*	80	*/
	u16_t minor_version;				/*	81	*/
	u16_t command_set_supported[2];			/*	82-83	*/
	u16_t command_set_supported_extension;		/*	84	*/
	u16_t command_set_enabled[2];			/*	85-86	*/
	u16_t command_set_default;			/*	87	*/
	u16_t udma_modes;				/*	88	*/
	u16_t time_for_security_erase;			/*	89	*/
	u16_t time_for_enhanced_security_erase;		/*	90	*/
	u16_t current_advanced_power_manage_value;	/*	91	*/
	u16_t master_password_revision;			/*	92	*/
	u16_t hardware_reset_result;			/*	93	*/
	u16_t acoustic_manage_value;			/*	94	*/
	u16_t stream_minimum_request_size;		/*	95	*/
	u16_t stream_transfer_time_dma;			/*	96	*/
	u16_t stream_access_latency;			/*	97	*/
	u16_t stream_performance_granularity[2];	/*	98-99	*/
	u16_t max_lba[4];				/*	100-103	*/
	u16_t stream_transfer_time_pio;			/*	104	*/	
	u16_t reserved3;				/*	105	*/
	u16_t physical_logical_sector_size;		/*	106	*/
	u16_t delay_acoustic_testing;			/*	107	*/
	u16_t naa;					/*	108	*/
	u16_t unique_id1;				/*	109	*/
	u16_t unique_id2;				/*	110	*/
	u16_t unique_id3;				/*	111	*/
	u16_t reserved4[4];				/*	112-115	*/
	u16_t reserved5;				/*	116	*/
	u16_t words_per_logical_sector[2];		/*	117-118	*/
	u16_t reserved6[8];				/*	119-126	*/
	u16_t removable_media_status_notification;	/*	127	*/
	u16_t security_status;				/*	128	*/
	u16_t vendor_specific[31];			/*	129-159	*/
	u16_t cfa_power_mode;				/*	160	*/
	u16_t reserved7[15];				/*	161-175	*/
	u16_t current_media_serial_number[30];		/*	176-205	*/
	u16_t reserved8[49];				/*	206-254	*/
	u16_t integrity_word;				/*	255	*/
} ata_identify_data_t;

#define FLASH_WIN_SIZE          (64 * 1024)

#define FLASH_BLOCK_ALIGN       0xFFFF0000
#define FLASH_SEC_SZ_MULTI	(256)
#define CMD_EBA_STRUCT_SZ	(8)

/*
 * Flash access codes
 */
enum {
        CMD_ADDR1               = 0xAAA,
        CMD_ADDR2               = 0x555,

        CMD_RESET               = 0xF0,
        CMD_ERASE1              = 0xAA,
        CMD_ERASE2              = 0x55,
        CMD_ERASE3              = 0x80,
        CMD_ERASE4              = 0xAA,
        CMD_ERASE5              = 0x55,
        CMD_ERASE6              = 0x10,
        CMD_GETSS_CODE          = 0x90,
        CMD_GETSS_CHIP          = 0x02,
        CMD_ERASE_CODE          = 0x30,
        FLG_ERASE_DONE          = 0xFF,

        CMD_WRITE1              = 0xAA,
        CMD_WRITE2              = 0x55,
        CMD_WRITE3              = 0xA0,

        CMD_REG                 = 0xAA,
        CMD_CFG                 = 0x98,
        CMD_CHK_SEC_SZ          = 0x5E,
        CMD_GET_SZ              = 0x4E,

	CMD_ERASE_REGION_CNT	= 0x58,
	CMD_EBA_INDEX1		= 0x5A,
	CMD_EBA_SEC_SZ1		= 0x5E,

};

#define CHIP_MX29LV640ET        0xC9
#define CHIP_MX29LV640EB        0xCB



#define NR_FW_RSVD              11
#define FW_START                0xAA5555AA
#define FW_END                  0x55AAAA55
#define FW_INV                  0xFF

#define F_INV                   0x0
#define F_VALID                 0x1
#define F_MASK                  0x3

/* 256K each */
#define FW_DATA_0_ADDR          0x220000
#define FW_DATA_1_ADDR          0x260000
#define FW_DATA_2_ADDR          0x2A0000

/* 64K each */
#define FW_REC_0_ADDR           0x300000
#define FW_REC_1_ADDR           0x310000

#define FW_STATE_NORMAL                 0
#define FW_STATE_FRESH                  1
#define FW_STATE_MISMATCH               2
#define FW_STATE_ROOT_NOT_EXIST         3
#define FW_STATE_BACKUP_NOT_EXIST       4
#define FW_STATE_READ_FW_REC0_FAILED    5
#define FW_STATE_READ_FW_REC1_FAILED    6

#define	WAM_FW_REC_MAX		(2)


#define MIN(x, y)		((x) < (y) ? (x) : (y))
#define NUM_OF_ELM(array)	(sizeof (array) / sizeof (array[0]))

// #define CONFIG_ENABLE_DEBUG
#ifdef  CONFIG_ENABLE_DEBUG
#define MV_DBG_PRINTF           printf
#else
#define MV_DBG_PRINTF(fmt, ...)
#endif

// #define MV_FLASH_DEBUG
#ifdef MV_FLASH_DEBUG
#define WAM_PRINT_DBG            printf
#else
#define WAM_PRINT_DBG(s, ...)
#endif

#define WAM_PRINT                printf
#define MV_PRINT_ERR            printf



/* derived from linux/lib/crc32.c, GNU GPL v2 */
static inline unsigned int
crc32_priv(unsigned char const *p, size_t len, unsigned int crc_init)
{
    int i;
    unsigned int crc = crc_init;

    while (len--) {
        crc ^= *p++;
        for (i = 0; i < 8; i++)
                crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    }
    return crc;

}

static inline unsigned int
crc32(unsigned char const *p, size_t len)
{
    return (crc32_priv(p, len, 0xFFFFEEEE));
}

static inline unsigned int
crc32_2(unsigned char const *p, size_t len,
        unsigned char const *p2, size_t len2)
{
        unsigned int crc;

        crc = crc32_priv(p, len, 0xFFFFEEEE);
        crc = crc32_priv(p2, len2, crc);

        return (crc);
}

static inline unsigned int
crc32_next(unsigned char const *p, size_t len, unsigned int crc_next)
{
        return (crc32_priv(p, len, crc_next));
}


#endif /* _MV_WAMLIB_PRIV_H_ */
