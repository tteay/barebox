/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Raghavendra KH <r-khandenahally@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/**
 * @file
 * @brief Beagle Specific Board Initialization routines
 */

/**
 * @page ti_beagle Texas Instruments Beagle Board
 *
 * FileName: arch/arm/boards/omap/board-beagle.c
 *
 * Beagle Board from Texas Instruments as described here:
 * http://www.beagleboard.org
 *
 * This board is based on OMAP3530.
 * More on OMAP3530 (including documentation can be found here):
 * http://focus.ti.com/docs/prod/folders/print/omap3530.html
 *
 * This file provides initialization in two stages:
 * @li boot time initialization - do basics required to get SDRAM working.
 * This is run from SRAM - so no case constructs and global vars can be used.
 * @li run time initialization - this is for the rest of the initializations
 * such as flash, uart etc.
 *
 * Boot time initialization includes:
 * @li SDRAM initialization.
 * @li Pin Muxing relevant for Beagle.
 *
 * Run time initialization includes
 * @li serial @ref serial_im98xx.c driver device definition
 *
 * Originally from arch/arm/boards/omap/board-sdp343x.c
 */

#include <common.h>
#include <console.h>
#include <init.h>
#include <driver.h>
#include <partition.h>
#include <mach/im98xx.h>
#include <asm/io.h>
#include <asm/armlinux.h>
#include <generated/mach-types.h>
#include <mach/sys_info.h>
#include <mach/syslib.h>
#include <mach/control.h>
#include <mach/omap3-clock.h>
#include <mach/omap3-silicon.h>
#include <mach/im98xx_nand.h>
#include <mach/magic.h>
#include <mach/mem_map.h>
#include "board.h"

struct ddr_param {
	int DAT_Driving;
	int CLK_Driving;
	int DQS_Driving;
	int DQM_Driving;
	int CTL_Driving;

	int PHY01_2to0;
	int PHY01_6to4;
	int PHY01_11to8;
	int PHY01_15to12;
	int PHY01_16;
	int PHY01_26to24;
	int PHY01_28;

	int PHY03_2to0;
	int PHY03_6to4;
	int PHY03_11to8;
	int PHY03_15to12;
	int PHY03_16;
	int PHY03_26to24;
	int PHY03_28;

	int PHY05_2to0;
	int PHY05_6to4;
	int PHY05_11to8;
	int PHY05_15to12;
	int PHY05_16;
	int PHY05_26to24;
	int PHY05_28;

	int PHY07_2to0;
	int PHY07_6to4;
	int PHY07_11to8;
	int PHY07_15to12;
	int PHY07_16;
	int PHY07_26to24;
	int PHY07_28;

	int PHY10_15to9;
	int PHY12_15to9;
	int PHY14_15to9;
	int PHY16_15to9;

	int PHY11_7to0;
	int PHY13_7to0;
	int PHY15_7to0;
	int PHY17_7to0;

	int DeviceDS;
};

static struct ddr_param ddr = {
#if defined(CONFIG_SDRAM_DEVICE_MICRON_MT29C4G48MAZBAAKS_5WT_CONFIG)

#if (defined(CONFIG_IM98XX_XMEM_CLOCK_130_CONFIG) || \
     defined(CONFIG_IM98XX_XMEM_CLOCK_104_CONFIG) || \
     defined(CONFIG_IM98XX_XMEM_CLOCK_78_CONFIG))
	0x3, 0x2, 0x3, 0x3, 0x3,
	0x3, 0x0, 0x6, 0x1, 0x1, 0x3, 0x0,
	0x3, 0x0, 0x6, 0x1, 0x1, 0x3, 0x0,
	0x3, 0x0, 0x6, 0x1, 0x1, 0x3, 0x0,
	0x3, 0x0, 0x6, 0x1, 0x1, 0x3, 0x0,
	0x25, 0x25, 0x25, 0x25,
	0xe, 0xe, 0xe, 0xe,
	0x0
#else
	0x3, 0x2, 0x3, 0x3, 0x3,
	0x3, 0x0, 0x6, 0x1, 0x1, 0x3, 0x0,
	0x3, 0x0, 0x6, 0x1, 0x1, 0x3, 0x0,
	0x3, 0x0, 0x6, 0x1, 0x1, 0x3, 0x0,
	0x3, 0x0, 0x6, 0x1, 0x1, 0x3, 0x0,
	0x25, 0x25, 0x25, 0x25,
	0x4, 0x4, 0x4, 0x4,
	0x0
#endif

#else

#if (defined(CONFIG_IM98XX_XMEM_CLOCK_130_CONFIG) || \
     defined(CONFIG_IM98XX_XMEM_CLOCK_104_CONFIG) || \
     defined(CONFIG_IM98XX_XMEM_CLOCK_78_CONFIG))
	0x3, 0x2, 0x2, 0x3, 0x3,
	0x4, 0x0, 0x8, 0x0, 0x0, 0x3, 0x0,
	0x4, 0x0, 0x8, 0x0, 0x0, 0x3, 0x0,
	0x4, 0x0, 0x8, 0x0, 0x0, 0x3, 0x0,
	0x4, 0x0, 0x8, 0x0, 0x0, 0x3, 0x0,
	0x28, 0x28, 0x28, 0x28,
	0xe, 0xe, 0xe, 0xe,
	0x1
#else
	0x3, 0x2, 0x2, 0x3, 0x3,
	0x4, 0x0, 0x8, 0x0, 0x0, 0x3, 0x0,
	0x4, 0x0, 0x8, 0x0, 0x0, 0x3, 0x0,
	0x4, 0x0, 0x8, 0x0, 0x0, 0x3, 0x0,
	0x4, 0x0, 0x8, 0x0, 0x0, 0x3, 0x0,
	0x28, 0x28, 0x28, 0x28,
	0x4, 0x4, 0x4, 0x4,
	0x1
#endif

#endif
};

extern int dev_add_bb_dev(char *path, const char *name);

/******************** Board Boot Time *******************/

void DenaliPHY_Write (unsigned int u32_mmr_addr, unsigned int u32_mmr_wpat)
{
	int i;

	/* To avoid problem when MMU or cache enabled */
	for (i = 0; i < 4; i++)
		writel(u32_mmr_addr, DENALY_PHY_ADR);

	/* DenaliPHY_Write: Address */
	writel(u32_mmr_addr, DENALY_PHY_ADR);

	/* DenaliPHY_Write: Data */
	writel(u32_mmr_wpat, DENALY_PHY_WDATA);
}

unsigned int DenaliPHY_Read(unsigned int u32_mmr_addr)
{
	int i;

	/* To avoid problem when MMU or cache enabled */
	for (i = 0; i < 4; i++)
		writel(u32_mmr_addr, DENALY_PHY_ADR);

	/* DenaliPHY_Read: Address */
	writel(u32_mmr_addr, DENALY_PHY_ADR);

	/* To avoid problem when MMU or cache enabled */
	for (i = 0; i < 4; i++)
		readl(DENALY_PHY_RDATA);

	/* DenaliPHY_Read: Data */
	return readl(DENALY_PHY_RDATA);
}

/* ---------------------------------------------------------
 * mDDR_Setting:
 * 	xxx_Driving: 0:10mA, 1:8mA, 2:4mA, 3:2mA
 * 	PHY01, PHY03, PHY05, PHY07: (130MHz default setting 0xf3003926)
 * 		2:0, Adjusts the ending point of the DQ pad output enable window (5, 6, 7)
 * 		6:4, Adjusts the starting point of the DQ pad output enable window (1, 2, 3)
 * 		11:8, Adjusts the ending point of the DQS pad output enable window (8, 9, a)
 * 		15:12, Adjusts the starting point of the DQS pad output enable window (2, 3, 4)
 * 		16, Subtrats 1/2 cycle from the DQS gate value programmed (0, 1)
 * 		26:24, Defines the read data delay (2, 3, 4)
 * 		28, Echo gate control (0, 1)
 * 	PHY10, PHY12, PHY14, PHY16: (130MHz default setting 0x20103c33)
 * 		15:9, read DQS delay setting when the DLL is operating in normal mode (0x00~0x40, inc 2)
 * 	PHY11, PHY13, PHY15, PHY17: (130MHz default setting 0x00081f0e)
 * 		7:0, DLL increment used by the DLL when searching for a lock (0x4, 0x9, 0xe, 0x13)
 * 	DeviceDS: Device Driving Strength, 0:Full, 1:1/2, 2:1/4, 3:3/4
 * --------------------------------------------------------- */
void mDDR_Setting(void)
{
	unsigned int rdtmp;

	/* --- Driving --- */
	rdtmp = DenaliPHY_Read(DENALY_PHY_00);
	rdtmp = (rdtmp & 0xfffff003) |
		(ddr.CTL_Driving << 10) |
		(ddr.DQM_Driving << 8) |
		(ddr.DQS_Driving << 6) |
		(ddr.CLK_Driving << 4) |
		(ddr.DAT_Driving << 2);
	DenaliPHY_Write(DENALY_PHY_00, rdtmp);

	/* --- PHY01,03,05,07 --- */
	rdtmp = DenaliPHY_Read(DENALY_PHY_01);
	rdtmp = (rdtmp & 0xe8fe0088) |
		(ddr.PHY01_28 << 28) |
		(ddr.PHY01_26to24 << 24) |
		(ddr.PHY01_16 << 16) |
		(ddr.PHY01_15to12 << 12) |
		(ddr.PHY01_11to8 << 8) |
		(ddr.PHY01_6to4 << 4) |
		(ddr.PHY01_2to0);
	DenaliPHY_Write(DENALY_PHY_01, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_03);
	rdtmp = (rdtmp& 0xe8fe0088) |
		(ddr.PHY03_28 << 28) |
		(ddr.PHY03_26to24 << 24) |
		(ddr.PHY03_16 << 16) |
		(ddr.PHY03_15to12 << 12) |
		(ddr.PHY03_11to8 << 8) |
		(ddr.PHY03_6to4 << 4) |
		(ddr.PHY03_2to0);
	DenaliPHY_Write(DENALY_PHY_03, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_05);
	rdtmp = (rdtmp& 0xe8fe0088) |
		(ddr.PHY05_28 << 28) |
		(ddr.PHY05_26to24 << 24) |
		(ddr.PHY05_16 << 16) |
		(ddr.PHY05_15to12 << 12) |
		(ddr.PHY05_11to8 << 8) |
		(ddr.PHY05_6to4 << 4) |
		(ddr.PHY05_2to0);
	DenaliPHY_Write(DENALY_PHY_05, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_07);
	rdtmp = (rdtmp & 0xe8fe0088) |
		(ddr.PHY07_28 << 28) |
		(ddr.PHY07_26to24 << 24) |
		(ddr.PHY07_16 << 16) |
		(ddr.PHY07_15to12 << 12) |
		(ddr.PHY07_11to8 << 8) |
		(ddr.PHY07_6to4 << 4) |
		(ddr.PHY07_2to0);
	DenaliPHY_Write(DENALY_PHY_07, rdtmp);

	/* --- PHY10,12,14,16 --- */
	rdtmp = DenaliPHY_Read(DENALY_PHY_10);
	rdtmp = (rdtmp & 0xffff01ff) | (ddr.PHY10_15to9 << 9);
	DenaliPHY_Write(DENALY_PHY_10, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_12);
	rdtmp = (rdtmp & 0xffff01ff) | (ddr.PHY12_15to9 << 9);
	DenaliPHY_Write(DENALY_PHY_12, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_14);
	rdtmp = (rdtmp & 0xffff01ff) | (ddr.PHY14_15to9 << 9);
	DenaliPHY_Write(DENALY_PHY_14, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_16);
	rdtmp = (rdtmp & 0xffff01ff) | (ddr.PHY16_15to9 << 9);
	DenaliPHY_Write(DENALY_PHY_16, rdtmp);

	/* --- PHY11,13,15,17 --- */
	rdtmp = DenaliPHY_Read(DENALY_PHY_11);
	rdtmp = (rdtmp & 0xffffff00) | ddr.PHY11_7to0;
	DenaliPHY_Write(DENALY_PHY_11, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_13);
	rdtmp = (rdtmp & 0xffffff00) | ddr.PHY13_7to0;
	DenaliPHY_Write(DENALY_PHY_13, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_15);
	rdtmp = (rdtmp & 0xffffff00) | ddr.PHY15_7to0;
	DenaliPHY_Write(DENALY_PHY_15, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_17);
	rdtmp = (rdtmp & 0xffffff00) | ddr.PHY17_7to0;
	DenaliPHY_Write(DENALY_PHY_17, rdtmp);

	/*---------------*/
	/* Wait DLL lock */
	/*---------------*/
	writel(readl(SDR_AC2_REG) | 0x00100000, SDR_AC2_REG); /* dll_rst_n to "0" */
	writel(readl(SDR_AC2_REG) & 0xffefffff, SDR_AC2_REG); /* dll_rst_n to "1" */
	writel(readl(SDR_AC2_REG) | 0x00080000, SDR_AC2_REG); /* ctrlupd_req to "1" */
	writel(readl(SDR_AC2_REG) & 0xfff7ffff, SDR_AC2_REG); /* ctrlupd_req to "0" */

	rdtmp = 0;
	while((rdtmp& 0x1)!= 0x1) rdtmp= DenaliPHY_Read(DENALY_PHY_19);	/* wait DLL lock */
	rdtmp = 0;
	while((rdtmp& 0x1)!= 0x1) rdtmp= DenaliPHY_Read(DENALY_PHY_22);	/* wait DLL lock */
	rdtmp = 0;
	while((rdtmp& 0x1)!= 0x1) rdtmp= DenaliPHY_Read(DENALY_PHY_25);	/* wait DLL lock */
	rdtmp = 0;
	while((rdtmp& 0x1)!= 0x1) rdtmp= DenaliPHY_Read(DENALY_PHY_28);	/* wait DLL lock */

	while((readl(SDR_AC2_REG) & 0x10000) == 0) {};

	/*---------------------------*/
	/* Device Driving Re-Setting */
	/*---------------------------*/
	writel(SDM_EMR_BA_2 | SDM_PASR_FUL | (ddr.DeviceDS << 5), SDR_MODE3_REG);
	writel(readl(SDR_MODE1_REG) | SDM_EMR, SDR_MODE1_REG);
	while((readl(SDR_MODE1_REG) & SDM_EMR) == SDM_EMR);
}

void sdram_denali_phy_init (void)
{
/* V3 default parameters */
#if defined(CONFIG_SDRAM_DEVICE_MICRON_MT29C4G48MAZBAAKS_5WT_CONFIG)
	DenaliPHY_Write(DENALY_PHY_00, 0x00003aa8);
	DenaliPHY_Write(DENALY_PHY_01, 0xf4014a37);
	DenaliPHY_Write(DENALY_PHY_02, 0x07400ad0);
	DenaliPHY_Write(DENALY_PHY_03, 0xf4014a37);
	DenaliPHY_Write(DENALY_PHY_04, 0x07400ad0);
	DenaliPHY_Write(DENALY_PHY_05, 0xf4014a37);
	DenaliPHY_Write(DENALY_PHY_06, 0x07400ad0);
	DenaliPHY_Write(DENALY_PHY_07, 0xf4014a37);
	DenaliPHY_Write(DENALY_PHY_08, 0x07400ad0);
	DenaliPHY_Write(DENALY_PHY_09, 0x00800004);
	DenaliPHY_Write(DENALY_PHY_10, 0xa0104633);
	DenaliPHY_Write(DENALY_PHY_11, 0x00081304);
	DenaliPHY_Write(DENALY_PHY_12, 0xa0104633);
	DenaliPHY_Write(DENALY_PHY_13, 0x00081504);
	DenaliPHY_Write(DENALY_PHY_14, 0xa0104633);
	DenaliPHY_Write(DENALY_PHY_15, 0x00081204);
	DenaliPHY_Write(DENALY_PHY_16, 0xa0104633);
	DenaliPHY_Write(DENALY_PHY_17, 0x00081404);
	DenaliPHY_Write(DENALY_PHY_18, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_19, 0x00000071);
	DenaliPHY_Write(DENALY_PHY_20, 0x0006000f);
	DenaliPHY_Write(DENALY_PHY_21, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_22, 0x00000071);
	DenaliPHY_Write(DENALY_PHY_23, 0x0009000f);
	DenaliPHY_Write(DENALY_PHY_24, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_25, 0x00000071);
	DenaliPHY_Write(DENALY_PHY_26, 0x0004800f);
	DenaliPHY_Write(DENALY_PHY_27, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_28, 0x00000071);
	DenaliPHY_Write(DENALY_PHY_29, 0x0007800f);
#else
	DenaliPHY_Write(DENALY_PHY_00, 0x00003aa8);
	DenaliPHY_Write(DENALY_PHY_01, 0xf3003926);
	DenaliPHY_Write(DENALY_PHY_02, 0x07400ad0);
	DenaliPHY_Write(DENALY_PHY_03, 0xf3003926);
	DenaliPHY_Write(DENALY_PHY_04, 0x07400ad0);
	DenaliPHY_Write(DENALY_PHY_05, 0xf3003926);
	DenaliPHY_Write(DENALY_PHY_06, 0x07400ad0);
	DenaliPHY_Write(DENALY_PHY_07, 0xf3003926);
	DenaliPHY_Write(DENALY_PHY_08, 0x07400ad0);
	DenaliPHY_Write(DENALY_PHY_09, 0x00800004);
	DenaliPHY_Write(DENALY_PHY_10, 0xa0103c33);
	DenaliPHY_Write(DENALY_PHY_11, 0x0008070e);
	DenaliPHY_Write(DENALY_PHY_12, 0xa0103c33);
	DenaliPHY_Write(DENALY_PHY_13, 0x0008080e);
	DenaliPHY_Write(DENALY_PHY_14, 0xa0103c33);
	DenaliPHY_Write(DENALY_PHY_15, 0x0008070e);
	DenaliPHY_Write(DENALY_PHY_16, 0xa0103c33);
	DenaliPHY_Write(DENALY_PHY_17, 0x0008080e);
	DenaliPHY_Write(DENALY_PHY_18, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_19, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_20, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_21, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_22, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_23, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_24, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_25, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_26, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_27, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_28, 0x00000000);
	DenaliPHY_Write(DENALY_PHY_29, 0x00000000);
#endif
}

#define RESET	0
#define INIT	1

void sdram_controller(int mode)
{
/* Target : Hynix H8BCS0UN0MCR-4EM. NAND/DDR MCP(4G+2G) x32, 400MHz, 1.8V */
/* 195MHz => 5.13ns */
/* 198.25MHz => 5.05ns */
/*
 * make menuconfig => CONFIG_SDRAM_DEVICE_HYNIX_H8BCS0UN0MCR_CONFIG
 * Type: Hynix H8BCS0UN0MCR-4EM
 *
 * 1. AC	Timing
 * tRCD		20ns
 * tRAS		40ns
 * tRP		15ns
 * tRFC		90ns
 * tXSR		140ns
 * tWPRES	0ns
 * tXP		1CLK
 * tREF		64ms
 *
 * 2. DC Characteristic
 * IDD4R (Continuous Read of Burst mode Operating Current) => 160mA
 * IDD4W (Continuous Write of Burst mode Operating Current) => 150mA
 * IDD0 (Operating one bank active-precharge current) => 110mA
 * IDD5 (Self Refresh Current) => 85 degrees: 1800uA, 45 degrees: 800uA
 */
/* Replace Hynix MCP H8BCS0UN0MCR-4EM with Hynix MCP H9DA4GH2GJAMCR
 *
 * make menuconfig => CONFIG_SDRAM_DEVICE_HYNIX_H9DA4GH2GJAMCR_CONFIG
 * Type: Hynix H9DA4GH2GJAMCR
 *
 * 1. Hynix H9DA4GH2GJAMCR DDR AC Timing
 * tRCD		15ns
 * tRAS		40ns
 * tRP		15ns
 * tRFC		90ns
 * tXSR		140ns
 * tWPRES	0ns
 * tXP		1CLK
 * tREF		64ms
 *
 * 2. DC Characteristic
 * IDD4R (Continuous Read of Burst mode Operating Current) => 100mA
 * IDD4W (Continuous Write of Burst mode Operating Current) => 90mA
 * IDD0 (Operating one bank active-precharge current) => 75mA
 * IDD5 (Self Refresh Current) => 85 degrees: 2000uA, 45 degrees: 900uA
 */
/* Replace Hynix MCP with Samsung MCP
 *
 * make menuconfig => CONFIG_SDRAM_DEVICE_SAMSUNG_K524G2GACG_CONFIG
 * Type: Samsung K524G2GACG-B050
 *
 * 1. Samsung K524G2GACG-B050 DDR AC Timing
 * tRCD		15ns
 * tRAS		40ns
 * tRP		15ns
 * tRFC		120ns
 * tXSR		120ns
 * tWPRES	0ns
 * tXP		2CLK
 * tREF		64ms
 *
 * 2. DC Characteristic
 * IDD4R (Continuous Read of Burst mode Operating Current) => 100mA
 * IDD4W (Continuous Write of Burst mode Operating Current) => 80mA
 * IDD0 (Operating one bank active-precharge current) => 70mA
 * IDD5 (Self Refresh Current) => 85 degrees: 1700uA, 45 degrees: 400uA
 */

/*
 * SDRAM normal configuration procedure with warm reset issue : 
 * Setup procedure to reset controller with Deep Power Down(DPD) mode
 * (1) reset to initate controller -> (2) DPD enter -> (3) initate Denali PHY -> (4) DPD exit -> (5) initate controller
 */

//	unsigned int i;

	unsigned int u32_tmp;

	if (mode == INIT) {
		/* step.1 : exit DPD mode */
		writel(readl(SDR_MODE1_REG) & (~SDM_DPD_ETR), SDR_MODE1_REG);
	}

	/* step.2 : Delay 200us for stable clock. */
	//im98xx_26MHz_halt(200, GPT1);
	im98xx_udelay(200);

//	for (i = 0 ; i < 20 ; i++)
//		sdelay(416);

	/* step.3 : Size configuration */
	writel(SDM_4_BNK | SDM_ROW_14bit | SDM_COL_10bit | SDM_BUS_32bit | SDM_DDR, SDR_SIZE_REG);

	/* 1 cyc = 1000 / 198.25ns = 5.044ns */

	/* step.4 : AC timing configuration */
#if defined(CONFIG_SDRAM_DEVICE_HYNIX_H8BCS0UN0MCR_CONFIG)
	writel(SDM_tRCD_4cyc | SDM_tRAS_8cyc | SDM_tRP_3cyc | SDM_tRFC_18cyc | SDM_tXSR_28cyc | SDM_tWPRES_0cyc | SDM_tXP_1cyc, SDR_AC_REG);
#endif
#if defined(CONFIG_SDRAM_DEVICE_HYNIX_H9DA4GH2GJAMCR_CONFIG)
	writel(SDM_tRCD_3cyc | SDM_tRAS_8cyc | SDM_tRP_3cyc | SDM_tRFC_18cyc | SDM_tXSR_28cyc | SDM_tWPRES_0cyc | SDM_tXP_1cyc, SDR_AC_REG);
#endif
#if defined(CONFIG_SDRAM_DEVICE_SAMSUNG_K524G2GACG_CONFIG)
	writel(SDM_tRCD_3cyc | SDM_tRAS_8cyc | SDM_tRP_3cyc | SDM_tRFC_24cyc | SDM_tXSR_24cyc | SDM_tWPRES_0cyc | SDM_tXP_2cyc, SDR_AC_REG);
#endif
#if defined(CONFIG_SDRAM_DEVICE_SPECTEK_4G48MZAP_5BT_CONFIG)
	writel(SDM_tRCD_3cyc | SDM_tRAS_8cyc | SDM_tRP_3cyc | SDM_tRFC_15cyc | SDM_tXSR_24cyc | SDM_tWPRES_0cyc | SDM_tXP_2cyc, SDR_AC_REG);
#endif
#if defined(CONFIG_SDRAM_DEVICE_MICRON_MT29C4G48MAZBAAKS_5WT_CONFIG)
	writel(SDM_tRCD_4cyc | SDM_tRAS_8cyc | SDM_tRP_3cyc | SDM_tRFC_15cyc | SDM_tXSR_23cyc | SDM_tWPRES_0cyc | SDM_tXP_2cyc, SDR_AC_REG);
#endif

	/* step.5 : Refresh control */
	writel((SDM_ARF_1560cyc-SDM_ARF_32cyc) | SDM_BRF_INI_16cyc | SDM_BRF_OPR_1time | SDM_BRF_SR_1Ktime, SDR_REFRESH_REG);

	/* step.6 : Mode control */
	writel(SDM_APD_NON, SDR_MODE1_REG);

	/* step.7 : Mode setting */
	writel(SDM_CAS_3 | SDM_BL_2, SDR_MODE2_REG);

	/* step.8 : Extended mode setting */
#if defined(CONFIG_IM98XX_CTL_DRIVER_STRENGTH_FUL_CONFIG)
	writel(SDM_EMR_BA_2 | SDM_PASR_FUL | SDM_DRI_STNG_FUL, SDR_MODE3_REG);
#endif
#if defined(CONFIG_IM98XX_CTL_DRIVER_STRENGTH_HAF_CONFIG)
	writel(SDM_EMR_BA_2 | SDM_PASR_FUL | SDM_DRI_STNG_HAF, SDR_MODE3_REG);
#endif
#if defined(CONFIG_IM98XX_CTL_DRIVER_STRENGTH_QUA_CONFIG)
	writel(SDM_EMR_BA_2 | SDM_PASR_FUL | SDM_DRI_STNG_QUA, SDR_MODE3_REG);
#endif
#if defined(CONFIG_IM98XX_CTL_DRIVER_STRENGTH_OCT_CONFIG)
	writel(SDM_EMR_BA_2 | SDM_PASR_FUL | SDM_DRI_STNG_OCT, SDR_MODE3_REG);
#endif

	/* step.9 : Enable mode control to setup all of the setting */
	if (mode == INIT)
		writel((readl(SDR_MODE1_REG) | SDM_INIT) & (~SDM_DPD_ETR), SDR_MODE1_REG);
	if (mode == RESET)
		writel(readl(SDR_MODE1_REG) | SDM_INIT, SDR_MODE1_REG);

	writel(readl(SDR_MODE1_REG) | SDM_MR | SDM_EMR, SDR_MODE1_REG);
	writel(readl(SDR_MODE1_REG) | SDM_APD_DYN_PRG | SDM_WAT_to_APD_16cyc, SDR_MODE1_REG);
	writel((readl(SDR_MODE1_REG) & ~SDM_WAT_to_ASR_MASK) | SDM_WAT_to_ASR_01, SDR_MODE1_REG);

	/* step.10 : polling to finish step.9 */
	do {
		u32_tmp = readl(SDR_MODE1_REG);
	} while ((u32_tmp & SDM_INIT) == SDM_INIT);

	if (mode == RESET) {
		/* step.11 : enter DPD mode */
		writel(u32_tmp | SDM_DPD_ETR, SDR_MODE1_REG);
	}
}

/**
 * @brief Do the SDRC initialization for 128Meg Micron DDR for CS0
 *
 * @return void
 */
static void sdrc_init(void)
{
/* SDRAM Controller reset */
	sdram_controller(RESET);

/* SDRAM PHY init */
	sdram_denali_phy_init();

/* SDRAM Controller */
	sdram_controller(INIT);

/* mDDR_Setting */
/* Get mDDR parameters from ETT tool scanning */
/* V3 parameters */
	mDDR_Setting();

	return;
}


/**
 * @brief Get the upper address of current execution
 *
 * we can use this to figure out if we are running in SRAM /
 * XIP Flash or in SDRAM
 *
 * @return base address
 */
u32 get_base(void)
{
	u32 val;
	__asm__ __volatile__("mov %0, pc \n":"=r"(val)::"memory");
	val >>= 28;
	return val;
}

u32 running_in_sdram(void)
{
	if (get_base() > 4)
		return 1;	/* in sdram */
	return 0;		/* running in SRAM or FLASH */
}

/**
 * @brief The basic entry point for board initialization.
 *
 * This is called as part of machine init (after arch init).
 * This is again called with stack in SRAM, so not too many
 * constructs possible here.
 *
 * @return void
 */
void board_init_lowlevel(void)
{
	int in_sdram = running_in_sdram();
//	mux_config();

/* V3: VIO18 power good debounce enable (2504b to support) */
/* Mask: Samuel.Lin suggested to disable this feature.
 * 		It was designed for experiments and debug. */
#if 0
	writel(readl(ABB_PMU_LDO_REG) | (0x1<<13), ABB_PMU_LDO_REG);
#endif
#if 1
	/* Pull High GPIO12 output: IM98XX_EVB_V3 GPIO12<=>ATV_PWR_EN */
	/* Note: If ATV device driver is ready, this setting could be removed. */
	writel(readl(GPIO12_REG) & ~(0x1 << 2), GPIO12_REG);
	writel(readl(GPIO12_REG) | (0x1 << 1), GPIO12_REG);
#endif

	/* Dont reconfigure SDRAM while running in SDRAM! */
	if (!in_sdram)
		sdrc_init();
}
