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

