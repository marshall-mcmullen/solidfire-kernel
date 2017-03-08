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

#ifndef __LOKI_REG_H
#define __LOKI_REG_H

#include "mv_types.h"

/*
 * PCI CFG values
 */
#define MRVL_VENDOR_ID		0x11AB
#define LOKI_DEVICE_ID		0x8180

/* Default Address mapping */
#define NV_REG_SPACE_LEN	(1 << 20)
#define NV_SCRPAD_SPACE_LEN	(512 << 10)

/*
 * PCI-E Registers
 */
enum {
	PCIE_BAR1		= 0x00018,	/* PCI-E BAR-1 register */
	PCIE_BAR2		= 0x00020,	/* PCI-E BAR-2 register */

	PCIE_WIN0_CTRL		= 0x01820,	/* Window 0 Control */
	PCIE_WIN0_BASE		= 0x01824,	/* Window 0 Base */
	PCIE_WIN0_REMAP		= 0x0182C,	/* Window 0 Remap */
	PCIE_WIN1_CTRL		= 0x01830,	/* Window 1 Control */
	PCIE_WIN1_BASE		= 0x01834,	/* Window 1 Base */
	PCIE_WIN1_REMAP		= 0x0183C,	/* Window 1 Remap */
	PCIE_WIN2_CTRL		= 0x01840,	/* Window 2 Control */
	PCIE_WIN2_BASE		= 0x01844,	/* Window 2 Base */
	PCIE_WIN2_REMAP		= 0x0184C,	/* Window 2 Remap */
};

enum {
	DMA_DLVRY_Q_WR_PTR	= 0x6191C,	/* Ctrl Eng Dlvry Q wr ptr */
	DMA_DLVRY_Q_RD_PTR	= 0x61920,	/* Ctrl Eng Dlvry Q rd ptr */

	DMA_A_DLVRY_Q_WR_PTR	= 0x0091C,	/* Eng A Dlvry Q wr ptr */
	DMA_A_DLVRY_Q_RD_PTR	= 0x00920,	/* Eng A Dlvry Q rd ptr */

	DMA_B_DLVRY_Q_WR_PTR	= 0x7091C,	/* Eng B Dlvry Q wr ptr */
	DMA_B_DLVRY_Q_RD_PTR	= 0x70920,	/* Eng B Dlvry Q rd ptr */

	DMA_C_DLVRY_Q_WR_PTR	= 0x6091C,	/* Eng C Dlvry Q wr ptr */
	DMA_C_DLVRY_Q_RD_PTR	= 0x60920,	/* Eng C Dlvry Q rd ptr */
};

enum {
	XOR_CTRL_STATUS		= 0x904,
	XOR_DLVRY_Q_WR_PTR	= 0x91C,
	XOR_DLVRY_Q_RD_PTR	= 0x920,
	XOR_CMPL_Q_WR_PTR	= 0x930,
};

#define PCIE_A_REG_BASE		0x30000	/* PCI-E Port A Reg Base */
#define PCIE_WCTRL_SIZE_SHFT	16	/* Used to set size of Window */
#define PCIE_WCTRL_ATTR_SHFT	8	/* Used to set TGT attr value */
#define PCIE_WCTRL_TGT_SHFT	4	/* Used to set TGT ID */
#define PCIE_WCTRL_BAR_SHFT	1
#define PCIE_WCTRL_BASE_SHFT	16
#define PCIE_WCTRL_WEN		0x1	/* Used to Enable Window */
#define PCIE_WCTRL_BAR2_MAP	0x2
#define PCIE_BAR_CONST		0xC	/* Prefetch En, Bar Type, Mem space */
#define PCIE_WIN_CTRL_OFF	0
#define PCIE_WIN_BASE_OFF	4
#define PCIE_WIN_REMAP_OFF	12
#define PCIE_WIN_REMAP_HIGH	16


/*
 * MBus Target IDs and Attributes
 */
enum {
	SCRPAD_MBUS_ID		= 0x5,		/* Scratchpad RAM MBus ID */
	FLASH_MBUS_ID		= 0x1,		/* Flash MBus ID */
	NVSRAM_MBUS_ID		= 0x1,		/* Internal NVRAM MBus ID */

	SCRPAD_MBUS_BM		= (1 << 0x5),   /* SRAM MBus Bitmap value */
	FLASH_MBUS_BM		= (1 << 0x1),   /* FLASH MBus Bitmap value */

	SCRPAD_MBUS_ATTR	= 0x0,		/* Attribute not applicable */
	FLASH_MBUS_ATTR		= 0xF,		/* FLASH MBus Attribute */
	NVSRAM_MBUS_ATTR	= 0x1E,		/* NVSRAM MBus Attribute */
};


/*
 * CPU Message and Doorbell Registers
 */
enum {
	PCIE_A_2_CPU_DRBL	= 0x20400,
	PCIE_A_2_CPU_DRBL_MASK  = 0x20404,
	CPU_2_PCIE_A_DRBL	= 0x20408,
	CPU_2_PCIE_A_DRBL_MASK  = 0x2040C,
	PCIE_B_2_CPU_DRBL	= 0x20410,
	PCIE_B_2_CPU_DRBL_MASK  = 0x20414,
	CPU_2_PCIE_B_DRBL	= 0x20418,
	CPU_2_PCIE_B_DRBL_MASK  = 0x2041C,
	PCIE_A_2_CPU_MSG_0	= 0x20430,
	PCIE_A_2_CPU_MSG_1	= 0x20434,
	CPU_2_PCIE_A_MSG_0	= 0x20438,
	CPU_2_PCIE_A_MSG_1	= 0x2043C,
	PCIE_B_2_CPU_MSG_0	= 0x20440,
	PCIE_B_2_CPU_MSG_1	= 0x20444,
	CPU_2_PCIE_B_MSG_0	= 0x20448,
	CPU_2_PCIE_B_MSG_1	= 0x2044C
};

/*
 * Interrupt Registers
 */
enum {
	MAIN_INTR_CAUSE		= 0x20200,	/* main intr cause */
	MAIN_INTR_MASK		= 0x2020C,	/* Main intr mask */
	CPU_INTR_MASK		= 0x20204,	/* Main intr mask */
};

enum {
	DDR_ERR_DATA_HIGH       = 0xF1440,
	DDR_ERR_DATA_LOW        = 0xF1444,
	DDR_RCVD_ECC            = 0xF1448,
	DDR_CALC_ECC            = 0xF144C,
	DDR_ERR_ADDR            = 0xF1450,
	DDR_ECC_CTRL            = 0xF1454,
	DDR_ECC_SBIT_CNTR       = 0xF1458,
	DDR_ECC_DBIT_CNTR       = 0xF145C,

	DDR_INTR_CAUSE          = 0xF14D0,
	DDR_INTR_MASK           = 0xF14D4,
};

enum {
	DDR_DBIT_ECC_ERR        = (1 << 1),
	DDR_SBIT_ECC_ERR        = (1 << 0),
};


/*
 * CPU Registers
 */
enum {
	CPU_CTRL_STAT_REG	= 0x20104,
	CPU_RSTOUT_MASK         = 0x20108,
	CPU_SOFT_RESET          = 0x2010c,
};


enum {
	CPU_RESET_BIT		= 0x2
};


/*
 * Interrupt cause bits
 */
#define INTR_CPU_2_PCIEB_DRBL	0x00000008
#define INTR_PCIEB_2_CPU_DRBL	0x00000004
#define INTR_CPU_2_PCIEA_DRBL	0x00000002
#define INTR_PCIEA_2_CPU_DRBL	0x00000001
#define INTR_DDR_CTLR		0x00010000

#endif /* __LOKI_REG_H */
