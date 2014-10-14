/**
 * @file
 */
/*
 * (C) Copyright 2006-2008
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
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

#include <common.h>
#include <asm/io.h>
#include <mach/magic.h>
#include <mach/mem_map.h>
#include <mach/omap3-clock.h>
#include <command.h>
#include <errno.h>

/* Setup VIO18 DC/DC */
#if defined(CONFIG_VIO18_1750)
	#define VIO18_VAL	0x0
#elif defined(CONFIG_VIO18_1800)
	#define VIO18_VAL	0x1
#elif defined(CONFIG_VIO18_1850)
	#define VIO18_VAL	0x2
#elif defined(CONFIG_VIO18_1900)
	#define VIO18_VAL	0x3
#endif

#if defined(CONFIG_MARCH_2504D)

/* Setup VSYS DC/DC */
#if defined(CONFIG_VSYS_1000)
	#define VSYS_VAL	0x6
#elif defined(CONFIG_VSYS_1050)
	#define VSYS_VAL	0x7
#elif defined(CONFIG_VSYS_1100)
	#define VSYS_VAL	0x8
#elif defined(CONFIG_VSYS_1150)
	#define VSYS_VAL	0x9
#elif defined(CONFIG_VSYS_1200)
	#define VSYS_VAL	0xa
#elif defined(CONFIG_VSYS_1250)
	#define VSYS_VAL	0xb
#elif defined(CONFIG_VSYS_1300)
	#define VSYS_VAL	0xc
#elif defined(CONFIG_VSYS_1350)
	#define VSYS_VAL	0xd
#endif

/* Setup VARM9 normal mode */
#if defined(CONFIG_VARM9_NORMAL_1000)
	#define VARM9_NORMAL_VAL	0x6
#elif defined(CONFIG_VARM9_NORMAL_1050)
	#define VARM9_NORMAL_VAL	0x7
#elif defined(CONFIG_VARM9_NORMAL_1100)
	#define VARM9_NORMAL_VAL	0x8
#elif defined(CONFIG_VARM9_NORMAL_1150)
	#define VARM9_NORMAL_VAL	0x9
#elif defined(CONFIG_VARM9_NORMAL_1200)
	#define VARM9_NORMAL_VAL	0xa
#elif defined(CONFIG_VARM9_NORMAL_1250)
	#define VARM9_NORMAL_VAL	0xb
#elif defined(CONFIG_VARM9_NORMAL_1300)
	#define VARM9_NORMAL_VAL	0xc
#elif defined(CONFIG_VARM9_NORMAL_1350)
	#define VARM9_NORMAL_VAL	0xd
#elif defined(CONFIG_VARM9_NORMAL_1400)
	#define VARM9_NORMAL_VAL	0xe
#elif defined(CONFIG_VARM9_NORMAL_1450)
	#define VARM9_NORMAL_VAL	0xf
#endif

/* Setup VARM9 sleep mode */
#if defined(CONFIG_VARM9_SLEEP_1000)
	#define VARM9_SLEEP_VAL		0x6
#elif defined(CONFIG_VARM9_SLEEP_1050)
	#define VARM9_SLEEP_VAL		0x7
#elif defined(CONFIG_VARM9_SLEEP_1100)
	#define VARM9_SLEEP_VAL		0x8
#elif defined(CONFIG_VARM9_SLEEP_1150)
	#define VARM9_SLEEP_VAL		0x9
#elif defined(CONFIG_VARM9_SLEEP_1200)
	#define VARM9_SLEEP_VAL		0xa
#elif defined(CONFIG_VARM9_SLEEP_1250)
	#define VARM9_SLEEP_VAL		0xb
#elif defined(CONFIG_VARM9_SLEEP_1300)
	#define VARM9_SLEEP_VAL		0xc
#elif defined(CONFIG_VARM9_SLEEP_1350)
	#define VARM9_SLEEP_VAL		0xd
#endif

#elif defined(CONFIG_MARCH_2504E)

/* Setup VSYS DC/DC */
#if defined(CONFIG_VSYS_0950)
	#define VSYS_VAL	0x0
#elif defined(CONFIG_VSYS_0975)
	#define VSYS_VAL	0x1
#elif defined(CONFIG_VSYS_1000)
	#define VSYS_VAL	0x2
#elif defined(CONFIG_VSYS_1025)
	#define VSYS_VAL	0x3
#elif defined(CONFIG_VSYS_1050)
	#define VSYS_VAL	0x4
#elif defined(CONFIG_VSYS_1075)
	#define VSYS_VAL	0x5
#elif defined(CONFIG_VSYS_1100)
	#define VSYS_VAL	0x6
#elif defined(CONFIG_VSYS_1125)
	#define VSYS_VAL	0x7
#elif defined(CONFIG_VSYS_1150)
	#define VSYS_VAL	0x8
#elif defined(CONFIG_VSYS_1175)
	#define VSYS_VAL	0x9
#elif defined(CONFIG_VSYS_1200)
	#define VSYS_VAL	0xa
#elif defined(CONFIG_VSYS_1225)
	#define VSYS_VAL	0xb
#elif defined(CONFIG_VSYS_1250)
	#define VSYS_VAL	0xc
#elif defined(CONFIG_VSYS_1275)
	#define VSYS_VAL	0xd
#elif defined(CONFIG_VSYS_1300)
	#define VSYS_VAL	0xe
#elif defined(CONFIG_VSYS_1325)
	#define VSYS_VAL	0xf
#endif

/* Setup VARM9 normal mode */
#if defined(CONFIG_VARM9_NORMAL_1400)
	#define VARM9_NORMAL_VAL	0x8
#elif defined(CONFIG_VARM9_NORMAL_1430)
	#define VARM9_NORMAL_VAL	0x9
#elif defined(CONFIG_VARM9_NORMAL_1460)
	#define VARM9_NORMAL_VAL	0xa
#elif defined(CONFIG_VARM9_NORMAL_1490)
	#define VARM9_NORMAL_VAL	0xb
#elif defined(CONFIG_VARM9_NORMAL_1520)
	#define VARM9_NORMAL_VAL	0xc
#elif defined(CONFIG_VARM9_NORMAL_1550)
	#define VARM9_NORMAL_VAL	0xd
#elif defined(CONFIG_VARM9_NORMAL_1580)
	#define VARM9_NORMAL_VAL	0xe
#elif defined(CONFIG_VARM9_NORMAL_1610)
	#define VARM9_NORMAL_VAL	0xf
#endif

/* Setup VARM9 sleep mode */
#if defined(CONFIG_VARM9_SLEEP_0920)
	#define VARM9_SLEEP_VAL		0x0
#elif defined(CONFIG_VARM9_SLEEP_0950)
	#define VARM9_SLEEP_VAL		0x1
#elif defined(CONFIG_VARM9_SLEEP_0980)
	#define VARM9_SLEEP_VAL		0x2
#elif defined(CONFIG_VARM9_SLEEP_1010)
	#define VARM9_SLEEP_VAL		0x3
#elif defined(CONFIG_VARM9_SLEEP_1040)
	#define VARM9_SLEEP_VAL		0x4
#elif defined(CONFIG_VARM9_SLEEP_1070)
	#define VARM9_SLEEP_VAL		0x5
#elif defined(CONFIG_VARM9_SLEEP_1100)
	#define VARM9_SLEEP_VAL		0x6
#elif defined(CONFIG_VARM9_SLEEP_1130)
	#define VARM9_SLEEP_VAL		0x7
#endif

#endif

#if defined(CONFIG_MACH_IM98XXV1)
unsigned int reg_pmu_ldovs4_flag = 1;
unsigned int reg_pmu_ldovs4_value = 0;
unsigned int reg_pmu_a9psc_flag = 1;
unsigned int reg_pmu_a9psc_value = 0;
unsigned int reg_pmu_modpsc_flag = 1;
unsigned int reg_pmu_modpsc_value = 0;
#endif

void im98xx_enable_ldo(unsigned char u8_source)
{
	unsigned int val = readl(ABB_PMU_LDO_REG);

	switch (u8_source) {
		case V_SIM1:		val |= (1 << 0);	break;
		case V_SIM2:		val |= (1 << 1);	break;
		case V_CAM_DIG:		val |= (1 << 2);	break;
		case V_CAM_ANA:		val |= (1 << 3);	break;
		case V_WIFI:		val |= (1 << 4);	break;
		case V_BT:		val |= (1 << 5);	break;
		case V_AUX2:		val |= (1 << 6);	break;
		case V_AUX3:		val |= (1 << 7);	break;
		case V_AUX4:		val |= (1 << 8);	break;
		case VIO18_AUX1:	val |= (1 << 12);	break;
		case VIO18_AUX2:	val |= (1 << 13);	break;
		case LED_DRIVE:		val |= (1 << 14);	break;
		case VIBRATOR:		val |= (1 << 15);	break;
		default:
//			printf("\n Error input LDO source! \n");
			return;
	}
	writel(val, ABB_PMU_LDO_REG);
}

void im98xx_disable_ldo(unsigned char u8_source)
{
	unsigned int val = readl(ABB_PMU_LDO_REG);

	switch (u8_source) {
		case V_SIM1:		val &= ~(1 << 0);	break;
		case V_SIM2:		val &= ~(1 << 1);	break;
		case V_CAM_DIG:		val &= ~(1 << 2);	break;
		case V_CAM_ANA:		val &= ~(1 << 3);	break;
		case V_WIFI:		val &= ~(1 << 4);	break;
		case V_BT:		val &= ~(1 << 5);	break;
		case V_AUX2:		val &= ~(1 << 6);	break;
		case V_AUX3:		val &= ~(1 << 7);	break;
		case V_AUX4:		val &= ~(1 << 8);	break;
		case VIO18_AUX1:	val &= ~(1 << 12);	break;
		case VIO18_AUX2:	val &= ~(1 << 13);	break;
		case LED_DRIVE:		val &= ~(1 << 14);	break;
		case VIBRATOR:		val &= ~(1 << 15);	break;
		default:
//			printf("\n Error input LDO source! \n");
			return;
	}
	writel(val, ABB_PMU_LDO_REG);
}

void im98xx_sim1_ldo(unsigned char u8_voltage)
{
	unsigned int val = readl(ABB_PMU_LDOVS1_REG);

	if (u8_voltage == V_3_0) {
		val |= (1 << 0);
	} else if (u8_voltage == V_1_8) {
		val &= ~(1 << 0);
	} else {
//		printf("\n Error LDO Voltage input! \n");
		return;
	}
	writel(val, ABB_PMU_LDOVS1_REG);
}

void im98xx_sim2_ldo(unsigned char u8_voltage)
{
	unsigned int val = readl(ABB_PMU_LDOVS1_REG);

	if (u8_voltage == V_3_0) {
		val |= (1 << 1);
	} else if (u8_voltage == V_1_8) {
		val &= ~(1 << 1);
	} else {
//		printf("\n Error LDO Voltage input! \n");
		return;
	}
	writel(val, ABB_PMU_LDOVS1_REG);
}

void im98xx_sim_volt(unsigned char u8_selection)
{
	switch (u8_selection) {
		case V_SIM_LOW:
			im98xx_enable_ldo(V_SIM1);
			im98xx_enable_ldo(V_SIM2);
			im98xx_sim1_ldo(V_1_8);
			im98xx_sim2_ldo(V_1_8);
			break;
		case V_SIM1_HIGH:
			im98xx_enable_ldo(V_SIM1);
			im98xx_sim1_ldo(V_3_0);
			im98xx_sim2_ldo(V_1_8);
			break;
		case V_SIM2_HIGH:
			im98xx_enable_ldo(V_SIM2);
			im98xx_sim1_ldo(V_1_8);
			im98xx_sim2_ldo(V_3_0);
			break;
		case V_SIM_HIGH:
			im98xx_enable_ldo(V_SIM1);
			im98xx_enable_ldo(V_SIM2);
			im98xx_sim1_ldo(V_3_0);
			im98xx_sim2_ldo(V_3_0);
			break;
		default:
			im98xx_disable_ldo(V_SIM1);
			im98xx_disable_ldo(V_SIM2);
			im98xx_sim1_ldo(V_1_8);
			im98xx_sim2_ldo(V_1_8);
			break;
	}
}

static void pmic_pm_init(void)
{
#if defined(CONFIG_MACH_IM98XXV1)
	if (readl(ABB_PMU_LDOVS4_REG) != 0x2D9A) {
		reg_pmu_ldovs4_flag = 0;
		reg_pmu_ldovs4_value = readl(ABB_PMU_LDOVS4_REG);
		writel(0x2D9A, ABB_PMU_LDOVS4_REG);
	}
#endif

	/* Setup VIO18 DC/DC */
	writel((readl(ABB_PMU_LDOVS4_REG) & ~(0x3 << 4)) | (VIO18_VAL << 4), ABB_PMU_LDOVS4_REG);

	/* Setup VSYS DC/DC */
	writel((readl(ABB_PMU_LDOVS4_REG) & ~(0xF)) | VSYS_VAL, ABB_PMU_LDOVS4_REG);

#if defined(CONFIG_MACH_IM98XXV1)
	if (readl(ABB_PMU_A9PSC_REG) != 0xAA) {
		reg_pmu_a9psc_flag = 0;
		reg_pmu_a9psc_value = readl(ABB_PMU_A9PSC_REG);
		writel(0xAA, ABB_PMU_A9PSC_REG);
	}
#endif

	/* Setup VARM9 normal mode */
	writel((readl(ABB_PMU_A9PSC_REG) & ~(0xF)) | VARM9_NORMAL_VAL, ABB_PMU_A9PSC_REG);

	/* Setup VARM9 sleep mode */
	writel((readl(ABB_PMU_A9PSC_REG) & ~(0xF << 4)) | (VARM9_SLEEP_VAL << 4), ABB_PMU_A9PSC_REG);
}

void im98xx_pm_init(void)
{
/* ARM9 */
	pmic_pm_init();

	/* Disable interrupt */
	writel(0, A9ITR_IRQ_REG);
	writel(0, A9ITR_FIQ_REG);

/* ARM7 */
	/* switch modem power supply to communication mode */
#if defined(CONFIG_MACH_IM98XXV1)
	if (readl(ABB_PMU_MODPSC_REG) != 0x8) {
		reg_pmu_modpsc_flag = 0;
		reg_pmu_modpsc_value = readl(ABB_PMU_MODPSC_REG);
		writel(0x8, ABB_PMU_MODPSC_REG);
	}
#endif
	writel(readl(ABB_PMU_MODPSC_REG) | 0x1, ABB_PMU_MODPSC_REG);

	/* BaseBand Power Control : Disable BaseBand Power-Down */
	writel(readl(BB_PWR_CTL_REG) & ~(0x00000031), BB_PWR_CTL_REG);
}

void im98xx_a9_wdt_reset(void)
{
	ulong iflag;

	/* The magic number will be cleaned up from a non WDT reset */
	do {
		writel(0x1234ABCD, WDT_SW1_REG);
	} while (readl(WDT_SW1_REG) != 0x1234ABCD);

	
	//---lanbo---,iflag = disable_interrupts();

	/* iM98XX WDT reset*/
//	printf("\n iM98XX WDT Reset! \n");
	writel(0x87654321, WDT_KEY_REG); /* WDT reset */
}

static int do_sim_open(int argc, char *argv[])
{
	unsigned char u8_selection;

	if (argc < 1)
		return COMMAND_ERROR_USAGE;

	u8_selection = simple_strtoul(argv[1], NULL, 16);

	im98xx_sim_volt(u8_selection);

	return 0;
}

/* default usage to dual SIM */
static const __maybe_unused char cmd_sim_open_help[] =
"Usage: sim_open <selection>\n";

BAREBOX_CMD_START(sim_open)
	.cmd		= do_sim_open,
	BAREBOX_CMD_HELP(cmd_sim_open_help)
BAREBOX_CMD_END

