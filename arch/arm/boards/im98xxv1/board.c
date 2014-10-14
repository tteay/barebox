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
#include <im98xx.h>
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

extern int dev_add_bb_dev(char *path, const char *name);

#if defined(CONFIG_MACH_IM98XXV1)
extern unsigned int reg_adc_ctl_flag;
extern unsigned int reg_adc_ctl_value;
extern unsigned int reg_adc_str_flag;
extern unsigned int reg_adc_str_value;
extern unsigned int reg_pmu_ldovs4_flag;
extern unsigned int reg_pmu_ldovs4_value;
extern unsigned int reg_pmu_a9psc_flag;
extern unsigned int reg_pmu_a9psc_value;
extern unsigned int reg_pmu_modpsc_flag;
extern unsigned int reg_pmu_modpsc_value;
#endif

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
void mDDR_Setting
	(int DAT_Driving, int CLK_Driving, int DQS_Driving, int DQM_Driving, int CTL_Driving,
	 int PHY01_2to0, int PHY01_6to4, int PHY01_11to8, int PHY01_15to12, int PHY01_16,
	 int PHY01_26to24, int PHY01_28,
	 int PHY03_2to0, int PHY03_6to4, int PHY03_11to8, int PHY03_15to12, int PHY03_16,
	 int PHY03_26to24, int PHY03_28,
	 int PHY05_2to0, int PHY05_6to4, int PHY05_11to8, int PHY05_15to12, int PHY05_16,
	 int PHY05_26to24, int PHY05_28,
	 int PHY07_2to0, int PHY07_6to4, int PHY07_11to8, int PHY07_15to12, int PHY07_16,
	 int PHY07_26to24, int PHY07_28,
	 int PHY10_15to9, int PHY12_15to9, int PHY14_15to9, int PHY16_15to9,
	 int PHY11_7to0, int PHY13_7to0, int PHY15_7to0, int PHY17_7to0,
	 int DeviceDS)
{
	unsigned int rdtmp;

	/* --- Driving --- */
	rdtmp = DenaliPHY_Read(DENALY_PHY_00);
	rdtmp = (rdtmp & 0xfffff003) |
		(CTL_Driving<<10) |
		(DQM_Driving<<8) |
		(DQS_Driving<<6) |
		(CLK_Driving<<4) |
		(DAT_Driving<<2);
	DenaliPHY_Write(DENALY_PHY_00, rdtmp);

	/* --- PHY01,03,05,07 --- */
	rdtmp = DenaliPHY_Read(DENALY_PHY_01);
	rdtmp = (rdtmp & 0xe8fe0088) |
		(PHY01_28<<28) |
		(PHY01_26to24<<24) |
		(PHY01_16<<16) |
		(PHY01_15to12<<12) |
		(PHY01_11to8<<8) |
		(PHY01_6to4<<4) |
		(PHY01_2to0);
	DenaliPHY_Write(DENALY_PHY_01, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_03);
	rdtmp = (rdtmp& 0xe8fe0088) |
		(PHY03_28<<28) |
		(PHY03_26to24<<24) |
		(PHY03_16<<16) |
		(PHY03_15to12<<12) |
		(PHY03_11to8<<8) |
		(PHY03_6to4<<4) |
		(PHY03_2to0);
	DenaliPHY_Write(DENALY_PHY_03, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_05);
	rdtmp = (rdtmp& 0xe8fe0088) |
		(PHY05_28<<28) |
		(PHY05_26to24<<24) |
		(PHY05_16<<16) |
		(PHY05_15to12<<12) |
		(PHY05_11to8<<8) |
		(PHY05_6to4<<4) |
		(PHY05_2to0);
	DenaliPHY_Write(DENALY_PHY_05, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_07);
	rdtmp = (rdtmp & 0xe8fe0088) |
		(PHY07_28<<28) |
		(PHY07_26to24<<24) |
		(PHY07_16<<16) |
		(PHY07_15to12<<12) |
		(PHY07_11to8<<8) |
		(PHY07_6to4<<4) |
		(PHY07_2to0);
	DenaliPHY_Write(DENALY_PHY_07, rdtmp);

	/* --- PHY10,12,14,16 --- */
	rdtmp = DenaliPHY_Read(DENALY_PHY_10);
	rdtmp = (rdtmp & 0xffff01ff) | (PHY10_15to9<<9);
	DenaliPHY_Write(DENALY_PHY_10, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_12);
	rdtmp = (rdtmp & 0xffff01ff) | (PHY12_15to9<<9);
	DenaliPHY_Write(DENALY_PHY_12, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_14);
	rdtmp = (rdtmp & 0xffff01ff) | (PHY14_15to9<<9);
	DenaliPHY_Write(DENALY_PHY_14, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_16);
	rdtmp = (rdtmp & 0xffff01ff) | (PHY16_15to9<<9);
	DenaliPHY_Write(DENALY_PHY_16, rdtmp);

	/* --- PHY11,13,15,17 --- */
	rdtmp = DenaliPHY_Read(DENALY_PHY_11);
	rdtmp = (rdtmp & 0xffffff00) | PHY11_7to0;
	DenaliPHY_Write(DENALY_PHY_11, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_13);
	rdtmp = (rdtmp & 0xffffff00) | PHY13_7to0;
	DenaliPHY_Write(DENALY_PHY_13, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_15);
	rdtmp = (rdtmp & 0xffffff00) | PHY15_7to0;
	DenaliPHY_Write(DENALY_PHY_15, rdtmp);

	rdtmp = DenaliPHY_Read(DENALY_PHY_17);
	rdtmp = (rdtmp & 0xffffff00) | PHY17_7to0;
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
	writel(SDM_EMR_BA_2 | SDM_PASR_FUL | (DeviceDS << 5), SDR_MODE3_REG);
	writel(readl(SDR_MODE1_REG) | SDM_EMR, SDR_MODE1_REG);
	while((readl(SDR_MODE1_REG) & SDM_EMR) == SDM_EMR);
}

void sdram_denali_phy_init (void)
{
/* V1 default parameters */
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
	DenaliPHY_Write(DENALY_PHY_11, 0x0008180e);
	DenaliPHY_Write(DENALY_PHY_12, 0xa0103c33);
	DenaliPHY_Write(DENALY_PHY_13, 0x0008200e);
	DenaliPHY_Write(DENALY_PHY_14, 0xa0103c33);
	DenaliPHY_Write(DENALY_PHY_15, 0x0008140e);
	DenaliPHY_Write(DENALY_PHY_16, 0xa0103c33);
	DenaliPHY_Write(DENALY_PHY_17, 0x00081c0e);
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
}

void sdram_controller_reset (void)
{
/*
 * SDRAM normal configuration procedure with warm reset issue : 
 * Setup procedure to reset controller with Deep Power Down(DPD) mode
 * (1) reset to initate controller -> (2) DPD enter -> (3) initate Denali PHY -> (4) DPD exit -> (5) initate controller
 */
#if 0
	unsigned int i;
#endif
	unsigned int u32_tmp;

	/* step.1 : Delay 200us for stable clock. */
	im98xx_26MHz_halt(200, GPT1);
#if 0
	for (i = 0 ; i < 20 ; i++)
		sdelay(416);
#endif
	/* step.2 : Size configuration */
	writel(SDM_4_BNK | SDM_ROW_14bit | SDM_COL_10bit | SDM_BUS_32bit | SDM_DDR, SDR_SIZE_REG);
	/* step.3 : AC timing configuration */
#if defined(CONFIG_SDRAM_DEVICE_HYNIX_H8BCS0UN0MCR_CONFIG)
	writel(SDM_tRCD_4cyc | SDM_tRAS_8cyc | SDM_tRP_3cyc | SDM_tRFC_18cyc | SDM_tXSR_28cyc | SDM_tWPRES_0cyc | SDM_tXP_1cyc, SDR_AC_REG);
#endif
#if defined(CONFIG_SDRAM_DEVICE_HYNIX_H9DA4GH2GJAMCR_CONFIG)
	writel(SDM_tRCD_3cyc | SDM_tRAS_8cyc | SDM_tRP_3cyc | SDM_tRFC_18cyc | SDM_tXSR_28cyc | SDM_tWPRES_0cyc | SDM_tXP_1cyc, SDR_AC_REG);
#endif
#if defined(CONFIG_SDRAM_DEVICE_SAMSUNG_K524G2GACG_CONFIG)
	writel(SDM_tRCD_3cyc | SDM_tRAS_8cyc | SDM_tRP_3cyc | SDM_tRFC_24cyc | SDM_tXSR_24cyc | SDM_tWPRES_0cyc | SDM_tXP_2cyc, SDR_AC_REG);
#endif
	/* step.4 : Refresh control */
	writel((SDM_ARF_1560cyc-SDM_ARF_32cyc) | SDM_BRF_INI_16cyc | SDM_BRF_OPR_1time | SDM_BRF_SR_1Ktime, SDR_REFRESH_REG);
	/* step.5 : Mode control */
	writel(SDM_APD_NON, SDR_MODE1_REG);
	/* step.6 : Mode setting */
	writel(SDM_CAS_3 | SDM_BL_2, SDR_MODE2_REG);
	/* step.7 : Extended mode setting */
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
	/* step.8 : Enable mode control to setup all of the setting */
	writel(readl(SDR_MODE1_REG) | SDM_INIT, SDR_MODE1_REG);
	writel(readl(SDR_MODE1_REG) | SDM_MR | SDM_EMR, SDR_MODE1_REG);
	writel(readl(SDR_MODE1_REG) | SDM_APD_DYN_PRG | SDM_WAT_to_APD_16cyc, SDR_MODE1_REG);
	writel((readl(SDR_MODE1_REG) & ~SDM_WAT_to_ASR_MASK) | SDM_WAT_to_ASR_01, SDR_MODE1_REG);
	/* step.9 : polling to finish step.8 */
	do {
		u32_tmp = readl(SDR_MODE1_REG);
	} while ((u32_tmp & SDM_INIT) == SDM_INIT);
	/* step.10 : enter DPD mode */
	writel(u32_tmp | SDM_DPD_ETR, SDR_MODE1_REG);
}

void sdram_controller_init (void)
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
#if 0
	unsigned int i;
#endif
	unsigned int u32_tmp;

	/* step.1 : exit DPD mode */
	writel(readl(SDR_MODE1_REG) & (~SDM_DPD_ETR), SDR_MODE1_REG);
	/* step.2 : Delay 200us for stable clock. */
	im98xx_26MHz_halt(200, GPT1);

//	for (i = 0 ; i < 20 ; i++)
//		sdelay(416);

	/* step.3 : Size configuration */
	writel(SDM_4_BNK | SDM_ROW_14bit | SDM_COL_10bit | SDM_BUS_32bit | SDM_DDR, SDR_SIZE_REG);

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
	writel((readl(SDR_MODE1_REG) | SDM_INIT) & (~SDM_DPD_ETR), SDR_MODE1_REG);
	writel(readl(SDR_MODE1_REG) | SDM_MR | SDM_EMR, SDR_MODE1_REG);
	writel(readl(SDR_MODE1_REG) | SDM_APD_DYN_PRG | SDM_WAT_to_APD_16cyc, SDR_MODE1_REG);
	writel((readl(SDR_MODE1_REG) & ~SDM_WAT_to_ASR_MASK) | SDM_WAT_to_ASR_01, SDR_MODE1_REG);

	/* step.10 : polling to finish step.9 */
	do {
		u32_tmp = readl(SDR_MODE1_REG);
	} while ((u32_tmp & SDM_INIT) == SDM_INIT);
}

/**
 * @brief Do the SDRC initialization for 128Meg Micron DDR for CS0
 *
 * @return void
 */
static void sdrc_init(void)
{
#if 0
	/* SDRAM software reset */
	/* No idle ack and RESET enable */
	writel(0x1A, SDRC_REG(SYSCONFIG));
	sdelay(100);
	/* No idle ack and RESET disable */
	writel(0x18, SDRC_REG(SYSCONFIG));

	/* SDRC Sharing register */
	/* 32-bit SDRAM on data lane [31:0] - CS0 */
	/* pin tri-stated = 1 */
	writel(0x00000100, SDRC_REG(SHARING));

	/* ----- SDRC Registers Configuration --------- */
	/* SDRC_MCFG0 register */
	writel(0x02584099, SDRC_REG(MCFG_0));

	/* SDRC_RFR_CTRL0 register */
	writel(0x54601, SDRC_REG(RFR_CTRL_0));

	/* SDRC_ACTIM_CTRLA0 register */
	writel(0xA29DB4C6, SDRC_REG(ACTIM_CTRLA_0));

	/* SDRC_ACTIM_CTRLB0 register */
	writel(0x12214, SDRC_REG(ACTIM_CTRLB_0));

	/* Disble Power Down of CKE due to 1 CKE on combo part */
	writel(0x00000081, SDRC_REG(POWER));

	/* SDRC_MANUAL command register */
	/* NOP command */
	writel(0x00000000, SDRC_REG(MANUAL_0));
	/* Precharge command */
	writel(0x00000001, SDRC_REG(MANUAL_0));
	/* Auto-refresh command */
	writel(0x00000002, SDRC_REG(MANUAL_0));
	/* Auto-refresh command */
	writel(0x00000002, SDRC_REG(MANUAL_0));

	/* SDRC MR0 register Burst length=4 */
	writel(0x00000032, SDRC_REG(MR_0));

	/* SDRC DLLA control register */
	writel(0x0000000A, SDRC_REG(DLLA_CTRL));
#endif

/* SDRAM Controller reset */
	sdram_controller_reset();

/* SDRAM PHY init */
	sdram_denali_phy_init();

/* SDRAM Controller */
	sdram_controller_init();

/* mDDR_Setting */
/* Get mDDR parameters from ETT tool scanning */
/* V1 parameters */
#if ( defined(CONFIG_IM98XX_XMEM_CLOCK_130_CONFIG) || defined(CONFIG_IM98XX_XMEM_CLOCK_104_CONFIG) || defined(CONFIG_IM98XX_XMEM_CLOCK_78_CONFIG) )
	mDDR_Setting(0x3, 0x2, 0x3, 0x3, 0x3,
		0x4, 0x1, 0x8, 0x2, 0x1, 0x3, 0x0,
		0x4, 0x1, 0x8, 0x2, 0x1, 0x3, 0x0,
		0x4, 0x1, 0x8, 0x2, 0x1, 0x3, 0x0,
		0x4, 0x1, 0x8, 0x2, 0x1, 0x3, 0x0,
		0x18, 0x18, 0x18, 0x18,
		0xe, 0xe, 0xe, 0xe,
		0x1);
#else
	mDDR_Setting(0x3, 0x2, 0x3, 0x3, 0x3,
		0x4, 0x1, 0x8, 0x2, 0x1, 0x3, 0x0,
		0x4, 0x1, 0x8, 0x2, 0x1, 0x3, 0x0,
		0x4, 0x1, 0x8, 0x2, 0x1, 0x3, 0x0,
		0x4, 0x1, 0x8, 0x2, 0x1, 0x3, 0x0,
		0x18, 0x18, 0x18, 0x18,
		0x4, 0x4, 0x4, 0x4,
		0x1);
#endif

	return;
}

/**
 * @brief Do the pin muxing required for Board operation.
 * We enable ONLY the pins we require to set. OMAP provides pins which do not
 * have alternate modes. Such pins done need to be set.
 *
 * See @ref MUX_VAL for description of the muxing mode.
 *
 * @return void
 */
#if 0
static void mux_config(void)
{
	/* SDRC_D0 - SDRC_D31 default mux mode is mode0 */

	/* GPMC */
	MUX_VAL(CP(GPMC_A1), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A2), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A3), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A4), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A5), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A6), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A7), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A8), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A9), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_A10), (IDIS | PTD | DIS | M0));

	/* D0-D7 default mux mode is mode0 */
	MUX_VAL(CP(GPMC_D8), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_D9), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_D10), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_D11), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_D12), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_D13), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_D14), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_D15), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_CLK), (IDIS | PTD | DIS | M0));
	/* GPMC_NADV_ALE default mux mode is mode0 */
	/* GPMC_NOE default mux mode is mode0 */
	/* GPMC_NWE default mux mode is mode0 */
	/* GPMC_NBE0_CLE default mux mode is mode0 */
	MUX_VAL(CP(GPMC_NBE0_CLE), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_NBE1), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(GPMC_NWP), (IEN | PTD | DIS | M0));
	/* GPMC_WAIT0 default mux mode is mode0 */
	MUX_VAL(CP(GPMC_WAIT1), (IEN | PTU | EN | M0));

	/* SERIAL INTERFACE */
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN | PTD | EN | M0));
	MUX_VAL(CP(UART3_RTS_SD), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(UART3_RX_IRRX), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(UART3_TX_IRTX), (IDIS | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_CLK), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_STP), (IDIS | PTU | EN | M0));
	MUX_VAL(CP(HSUSB0_DIR), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_NXT), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_DATA0), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_DATA1), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_DATA2), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_DATA3), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_DATA4), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_DATA5), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_DATA6), (IEN | PTD | DIS | M0));
	MUX_VAL(CP(HSUSB0_DATA7), (IEN | PTD | DIS | M0));
	/* I2C1_SCL default mux mode is mode0 */
	/* I2C1_SDA default mux mode is mode0 */
}
#endif

void pmic_2504a_verify(void)
{
	unsigned int reset_error_cnt = 0;

	if (readl(ABB_GCLK_REG) != 0x10)
	{
		printf("\n ABB_GCLK_REG != 0x10 (0x%x)\n", readl(ABB_GCLK_REG));
		writel(0x10, ABB_GCLK_REG);
		reset_error_cnt++;
	}
#if 0
	if (readl(ABB_ADC_CTL_REG) != 0x3F07)
	{
		printf("\n ABB_ADC_CTL_REG != 0x3F07 (0x%x)\n", readl(ABB_ADC_CTL_REG));
		writel(0x3F07, ABB_ADC_CTL_REG);
		reset_error_cnt++;
	}
#else
	if (reg_adc_ctl_flag != 1)
	{
		printf("\n ABB_ADC_CTL_REG != 0x3F07 (0x%x)\n", reg_adc_ctl_value);
		reset_error_cnt++;
	}
#endif
#if 0
	if (readl(ABB_ADC_STR_REG) != 0x0)
	{
		printf("\n ABB_ADC_STR_REG != 0x0 (0x%x)\n", readl(ABB_ADC_STR_REG));
		writel(0x0, ABB_ADC_STR_REG);
		reset_error_cnt++;
	}
#else
	if (reg_adc_str_flag != 1)
	{
		printf("\n ABB_ADC_STR_REG != 0x0 (0x%x)\n", reg_adc_str_value);
		reset_error_cnt++;
	}
#endif
	if (readl(ABB_CHR_CTDCTL_REG) != 0x2D0)
	{
		printf("\n ABB_CHR_CTDCTL_REG != 0x2D0 (0x%x)\n", readl(ABB_CHR_CTDCTL_REG));
		writel(0x2D0, ABB_CHR_CTDCTL_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_CHR_CM60_REG) != 0x0)
	{
		printf("\n ABB_CHR_CM60_REG != 0x0 (0x%x)\n", readl(ABB_CHR_CM60_REG));
		writel(0x0, ABB_CHR_CM60_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_CHR_TM60_REG) != 0x0)
	{
		printf("\n ABB_CHR_TM60_REG != 0x0 (0x%x)\n", readl(ABB_CHR_TM60_REG));
		writel(0x0, ABB_CHR_TM60_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_CHR_SDP_REG) != 0x0)
	{
		printf("\n ABB_CHR_SDP_REG != 0x0 (0x%x)\n", readl(ABB_CHR_SDP_REG));
		writel(0x0, ABB_CHR_SDP_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_CHR_BDP_REG) != 0x0)
	{
		printf("\n ABB_CHR_BDP_REG != 0x0 (0x%x)\n", readl(ABB_CHR_BDP_REG));
		writel(0x0, ABB_CHR_BDP_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_CHR_DONEFLG_REG) != 0x0)
	{
		printf("\n ABB_CHR_DONEFLG_REG != 0x0 (0x%x)\n", readl(ABB_CHR_DONEFLG_REG));
		writel(0x0, ABB_CHR_DONEFLG_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_VBAT_CTL_REG) != 0x10)
	{
		printf("\n ABB_VBAT_CTL_REG != 0x10 (0x%x)\n", readl(ABB_VBAT_CTL_REG));
		writel(0x10, ABB_VBAT_CTL_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_VBAT_SDP_REG) != 0x0)
	{
		printf("\n ABB_VBAT_SDP_REG != 0x0 (0x%x)\n", readl(ABB_VBAT_SDP_REG));
		writel(0x0, ABB_VBAT_SDP_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_VBAT_BDP_REG) != 0x0)
	{
		printf("\n ABB_VBAT_BDP_REG != 0x0 (0x%x)\n", readl(ABB_VBAT_BDP_REG));
		writel(0x0, ABB_VBAT_BDP_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_VBAT_DONEFLG_REG) != 0x0)
	{
		printf("\n ABB_VBAT_DONEFLG_REG != 0x0 (0x%x)\n", readl(ABB_VBAT_DONEFLG_REG));
		writel(0x0, ABB_VBAT_DONEFLG_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_BAT_MOCTL_REG) != 0x0)
	{
		printf("\n ABB_BAT_MOCTL_REG != 0x0 (0x%x)\n", readl(ABB_BAT_MOCTL_REG));
		writel(0x0, ABB_BAT_MOCTL_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_BAT_CHRSTOP_REG) != 0x0)
	{
		printf("\n ABB_BAT_CHRSTOP_REG != 0x0 (0x%x)\n", readl(ABB_BAT_CHRSTOP_REG));
		writel(0x0, ABB_BAT_CHRSTOP_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_BAT_CHRCFG_REG) != 0x40)
	{
		printf("\n ABB_BAT_CHRCFG_REG != 0x40 (0x%x)\n", readl(ABB_BAT_CHRCFG_REG));
		writel(0x41, ABB_BAT_CHRCFG_REG); /* change to 20/Rs */
		reset_error_cnt++;
	}
	if (readl(ABB_PMU_LDO_REG) != 0x0)
	{
		printf("\n ABB_PMU_LDO_REG != 0x0 (0x%x)\n", readl(ABB_PMU_LDO_REG));
		writel(0x0, ABB_PMU_LDO_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_PMU_LDOLP_REG) != 0x0)
	{
		printf("\n ABB_PMU_LDOLP_REG != 0x0 (0x%x)\n", readl(ABB_PMU_LDOLP_REG));
		writel(0x0, ABB_PMU_LDOLP_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_PMU_LDOVS1_REG) != 0x2A0)
	{
		printf("\n ABB_PMU_LDOVS1_REG != 0x2A0 (0x%x)\n", readl(ABB_PMU_LDOVS1_REG));
		writel(0x2A0, ABB_PMU_LDOVS1_REG);
		reset_error_cnt++;
	}
	if (readl(ABB_PMU_LDOVS2_REG) != 0x313)
	{
		printf("\n ABB_PMU_LDOVS2_REG != 0x313 (0x%x)\n", readl(ABB_PMU_LDOVS2_REG));
		writel(0x313, ABB_PMU_LDOVS2_REG);
		reset_error_cnt++;
	}
#if 0
	if (readl(ABB_PMU_LDOVS4_REG) != 0x2D9A)
	{
		printf("\n ABB_PMU_LDOVS4_REG != 0x2D9A (0x%x)\n", readl(ABB_PMU_LDOVS4_REG));
		writel(0x2D9A, ABB_PMU_LDOVS4_REG);
		reset_error_cnt++;
	}
#else
	if (reg_pmu_ldovs4_flag != 1)
	{
		printf("\n ABB_PMU_LDOVS4_REG != 0x2D9A (0x%x)\n", reg_pmu_ldovs4_value);
		reset_error_cnt++;
	}
#endif
#if 0
	if (readl(ABB_PMU_A9PSC_REG) != 0xAA)
	{
		printf("\n ABB_PMU_A9PSC_REG != 0xAA (0x%x)\n", readl(ABB_PMU_A9PSC_REG));
		writel(0xAA, ABB_PMU_A9PSC_REG);
		reset_error_cnt++;
	}
#else
	if (reg_pmu_a9psc_flag != 1)
	{
		printf("\n ABB_PMU_A9PSC_REG != 0xAA (0x%x)\n", reg_pmu_a9psc_value);
		reset_error_cnt++;
	}
#endif
#if 0
	if (readl(ABB_PMU_MODPSC_REG) != 0x8)
	{
		printf("\n ABB_PMU_MODPSC_REG != 0x8 (0x%x)\n", readl(ABB_PMU_MODPSC_REG));
		writel(0x8, ABB_PMU_MODPSC_REG);
		reset_error_cnt++;
	}
#else
	if (reg_pmu_modpsc_flag != 1)
	{
		printf("\n ABB_PMU_MODPSC_REG != 0x8 (0x%x)\n", reg_pmu_modpsc_value);
		reset_error_cnt++;
	}
#endif
	if (readl(ABB_PMU_DCDCSDT_REG) != 0x1111)
	{
		printf("\n ABB_PMU_DCDCSDT_REG != 0x1111 (0x%x)\n", readl(ABB_PMU_DCDCSDT_REG));
		writel(0x911, ABB_PMU_DCDCSDT_REG); /* 2504a ? => R: Normal, but W: Shift 1 bit */
		reset_error_cnt++;
	}
	if ( (readl(ABB_BGSEL_REG) & 0xF) != 0x8 )
	{
		printf("\n ABB_BGSEL_REG != 0x8 (0x%x)\n", readl(ABB_BGSEL_REG));
		writel(0x0, ABB_BGSEL_REG); /* Only reset the bit #0, #1, w/o reference status */
		reset_error_cnt++;
	}
	if (readl(ABB_PMU_TRIM_REG) != 0x0)
	{
		printf("\n ABB_PMU_TRIM_REG != 0x0 (0x%x)\n", readl(ABB_PMU_TRIM_REG));
		writel(0x0, ABB_PMU_TRIM_REG);
		reset_error_cnt++;
	}

	if (reset_error_cnt != 0)
	{
		printf("\n\n\n\n\n\n PMIC 2504A Error: %d of registers without default setting! \n", reset_error_cnt);
		printf(" Please check the registers for default setting(except Vtrim value)! \n\n\n\n\n\n");
	}
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
void board_init(void)
{
	int in_sdram = running_in_sdram();
//	mux_config();
	/* Dont reconfigure SDRAM while running in SDRAM! */
	if (!in_sdram)
		sdrc_init();
}

/******************** Board Run Time *******************/

static struct IM98XX_plat serial_plat = {
	.clock		= 26000000,	/* 26MHz */
	.f_caps		= CONSOLE_STDIN | CONSOLE_STDOUT | CONSOLE_STDERR,
	.reg_read	= im98xx_uart_read,
	.reg_write	= im98xx_uart_write,
};

static struct device_d im98xx_serial_device = {
	.name		= "serial_im98xx",
	.map_base	= IM98XX_UART3_BASE,
	.size		= 1024,
	.platform_data	= (void *)&serial_plat,
};

/**
 * @brief UART serial port initialization - remember to enable COM clocks in
 * arch
 *
 * @return result of device registration
 */
static int im98xx_console_init(void)
{
	/* Register the serial port */
	return register_device(&im98xx_serial_device);
}
console_initcall(im98xx_console_init);

static struct memory_platform_data sram_pdata = {
	.name	= "ram0",
	.flags	= DEVFS_RDWR,
};

static struct device_d sdram_dev = {
	.id		= -1,
	.name		= "mem",
	.map_base	= 0x40800000,
	.size		= 198 * 1024 * 1024,
	.platform_data	= &sram_pdata,
};

static struct memory_platform_data internal_sram_pdata = {
	.name = "ram1",
	.flags = DEVFS_RDWR,
};

static struct device_d sram_dev = {
	.id		= -1,
	.name		= "mem",
	.map_base	= 0x1FFE0000,
	.size		= 128 * 1024,
	.platform_data	= &internal_sram_pdata,
};

static struct memory_platform_data factory_sram_pdata = {
	.name = "ram2",
	.flags = DEVFS_RDWR,
};

static struct device_d imei_dev = {
	.id		= -1,
	.name		= "mem",
	.map_base	= 0x407F2000,
	.size		= 128 * 1024,
	.platform_data	= &factory_sram_pdata,
};

static struct memory_platform_data chargingIcon_sram_pdata = {
	.name = "ram3",
	.flags = DEVFS_RDWR,
};

static struct device_d icon_dev = {
	.id		= -1,
	.name		= "mem",
	.map_base	= 0x40000000,
	.size		= 128 * 1024,
	.platform_data	= &chargingIcon_sram_pdata,
};

/*RD3, bohung.wu, 20101013, add NOR FLASH support */
static struct device_d cfi_dev = {
	.id		= -1,
	.name		= "cfi_flash",
	.map_base	= 0x10000000,
	.size		= 4 * 1024 * 1024,
};

struct im98xx_nand_platform_data nand_info = {
	.timing		= 0x252d4b,
	.width		= 1,
};

static struct device_d nand_dev = {
	.id		= -1,
	.name		= "im98xx_nand",
	.platform_data	= &nand_info,
};

/*
 * SMSC 9118 network controller
 * connected to CS line 1 and interrupt line
 * GPIO3, data width is 16 bit
 */
static struct device_d network_dev = {
	.id		= -1,
	.name		= "smc911x",
	.map_base	= 0x16000000,
	.size		= 0x1000000,	/* area size */
};

static int im98xx_devices_init(void)
{
	int ret;
	int i;
	int env_block_is_found = 0;
	struct im98xx_nand_block_mark_info *self_block_info = NULL;
	struct im98xx_nand_block_mark_info *env_block_info = NULL;
	struct im98xx_nand_block_mark_info *ramloader_block_info = NULL;
	struct im98xx_nand_block_mark_info *factory_block_info = NULL;
	struct im98xx_nand_block_mark_info *icon_block_info = NULL;
	char null_block[4] = {0xff, 0xff, 0xff, 0xff};

	register_device(&cfi_dev);
	register_device(&nand_dev);
	register_device(&network_dev);
	ret = register_device(&sdram_dev);
	if (ret)
		goto failed;

	register_device(&sram_dev);
	register_device(&imei_dev);
	register_device(&icon_dev);

#ifdef BOOT_DBG
	printf("Block:\tOffset:\tSize:\tMark:\n");
	for (i = 0; i < 16; i++) {
		printf("%02d:\t%x\t%x\t\"%s\"\n", i,
			boot_region_info[i].offset,
			boot_region_info[i].size,
			boot_region_info[i].mark_id);
	}
#endif

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im00", 4) == 0) {
			self_block_info = &boot_region_info[i];
			printf("SELF Block is found!\n");
			break;
		}
	}

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im01", 4) == 0) {
			env_block_info = &boot_region_info[i];
			env_block_is_found = 1;
			printf("ENV Block is found!\n");
			break;
		}
	}

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im02", 4) == 0) {
			ramloader_block_info = &boot_region_info[i];
			printf("RAMLOADER Block is found!\n");
			break;
		}
	}

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im03", 4) == 0) {
			factory_block_info = &boot_region_info[i];
			printf("IMEI/SN Block is found!\n");
			break;
		}
	}

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im04", 4) == 0) {
			icon_block_info = &boot_region_info[i];
			printf("ICON Block is found!\n");
			break;
		}
	}

	if (env_block_is_found == 0) {
		BAREBOX_P("Find an empty block!\n");
		for (i = 0; i < 16; i++) {
			if ((strncmp(null_block, boot_region_info[i].mark_id, 4) == 0) &&
			    (boot_region_info[i].offset != 0)) {
				env_block_info = &boot_region_info[i];
				break;
			}
		}
	}

#ifdef BOOT_DBG
	printf("env_block:\t%x\t%x\t\"%s\"\n", self_block_info->offset,
		self_block_info->size, self_block_info->mark_id);

	printf("env_block:\t%x\t%x\t\"%s\"\n", env_block_info->offset,
		env_block_info->size, env_block_info->mark_id);

	printf("ramloader_block:\t%x\t%x\t\"%s\"\n", ramloader_block_info->offset,
		ramloader_block_info->size, ramloader_block_info->mark_id);

	printf("factory_block:\t%x\t%x\t\"%s\"\n", factory_block_info->offset,
		factory_block_info->size, factory_block_info->mark_id);

	printf("icon_block:\t%x\t%x\t\"%s\"\n", icon_block_info->offset,
		icon_block_info->size, icon_block_info->mark_id);
#endif

	/* ----------- add some vital partitions -------- */
	devfs_add_partition("nand0", self_block_info->offset,
			self_block_info->size, PARTITION_FIXED, "self_raw");
	dev_add_bb_dev("self_raw", "self0");

	devfs_add_partition("nand_im98xx_0", env_block_info->offset,
			env_block_info->size, PARTITION_FIXED, "env_raw");
	dev_add_bb_dev("env_raw", "env0");

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im02", 4) == 0) {
			devfs_add_partition("nand0", ramloader_block_info->offset,
					ramloader_block_info->size,
					PARTITION_FIXED, "ramloader_raw");
			dev_add_bb_dev("ramloader_raw", "ramloader0");
			break;
		}
	}

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im03", 4) == 0) {
			devfs_add_partition("nand0", factory_block_info->offset,
					factory_block_info->size,
					PARTITION_FIXED, "factory_raw");
			dev_add_bb_dev("factory_raw", "factory0");
			break;
		}
	}

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im04", 4) == 0) {
			devfs_add_partition("nand0", icon_block_info->offset,
					icon_block_info->size,
					PARTITION_FIXED, "icon_raw");
			dev_add_bb_dev("icon_raw", "icon0");
			break;
		}
	}

	armlinux_add_dram(&sram_dev);
	armlinux_add_dram(&imei_dev);
	armlinux_add_dram(&icon_dev);
	armlinux_add_dram(&sdram_dev);
	armlinux_set_bootparams((void *)sdram_dev.map_base + 0x100);
	armlinux_set_architecture(MACH_TYPE_IM98XXV1);
failed:
	return ret;
}
device_initcall(im98xx_devices_init);
