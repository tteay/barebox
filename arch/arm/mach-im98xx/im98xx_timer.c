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

#define GPT_32K		32768


void im98xx_hippo_halt(void)
{
	asm volatile (
		"mov r0, #0                \n"
		"mcr p15, 0, r0, c7, c0, 4 \n" /* Wait for interrupt */
		: : : "r0"
	);
}

/* Max. : 131071(s) */ /* To application part hibernation (sleep) control */
void im98xx_32K_halt(unsigned int u32_second)
{
	unsigned int u32_counter;

	if ((u32_second * GPT_32K) > 0xFFFFFFFE) {
		u32_counter = 0xFFFFFFFE;
		BAREBOX_P("\n Warning!! Out of Max. 32K Timer counter!! \n");
	} else {
		u32_counter = (u32_second * GPT_32K) - 1;
		BAREBOX_P("\n 32K Timer : %d s!! \n", u32_second);
	}

/* Setup GPT32K*/
	/* 32K free run */
	writel(readl(WDT_DEB_GCK_REG) & ~(0x1<<9), WDT_DEB_GCK_REG);
	/* Enable 32K GPT ITR to wake up ARM9 */
	writel(readl(A9_MSK_WKUPTR_REG) & ~(0x1<<13), A9_MSK_WKUPTR_REG);
	/* Disable 32K ITR */
	writel(readl(GPT32_TMSTA_CTL_REG) & ~(0x1<<1), GPT32_TMSTA_CTL_REG);
	/* 32K ITR mode : default as one-shot to halt mode */
	writel(readl(GPT32_TMSTA_CTL_REG) & ~(0x1<<0), GPT32_TMSTA_CTL_REG);
	/* Clear 32K Flag */
	writel(readl(A9_32K_LOGIC_ITR_CLR) | GPT_32K_ITR_FLAG, A9_32K_LOGIC_ITR_CLR);
	/* 32K counter value */
	writel(u32_counter , GPT32_TMC_CTL_REG);
	/* Enable 32K ITR */
	writel(readl(GPT32_TMSTA_CTL_REG) | (0x1<<1), GPT32_TMSTA_CTL_REG);
	/* Enable ITR select : 32K_A9_ITR */
	writel(readl(A9ITR_IRQ_REG) | (0x1<<2), A9ITR_IRQ_REG);
	/* Select A9ITR Priority : 2 -> 32K_A9_ITR */
	//writel(0xFFFFFF20, A9ITR_PRIO1_REG);
	writel(0x76543210, A9ITR_PRIO1_REG);
	writel(0xFEDCBA98, A9ITR_PRIO2_REG);

/* Halt mode */
	im98xx_hippo_halt();

	/* Disable ITR select : 32K_A9_ITR */
	writel(readl(A9ITR_IRQ_REG) & ~(0x1<<2), A9ITR_IRQ_REG);
	/* Unselect A9ITR Priority */
	writel(0xFFFFFFFF, A9ITR_PRIO1_REG);
	writel(0xFFFFFFFF, A9ITR_PRIO2_REG);
	/* Clear 32K Flag */
	writel(readl(A9_32K_LOGIC_ITR_CLR) | GPT_32K_ITR_FLAG, A9_32K_LOGIC_ITR_CLR);

/* Restore 32K setup to s32k_cs */
	writel(0xFFFFFFFE, GPT32_TMC_CTL_REG);
	writel(0x1, GPT32_TMSTA_CTL_REG);
}

/* Max. : 165191049(us) */
void im98xx_26MHz_halt(unsigned int u32_microsecond, unsigned char u8_gpt_number)
{
	unsigned int u32_counter;

	if ((u32_microsecond * 26) > 0xFFFFFFFE) {
		u32_counter = 0xFFFFFFFE;
//		printf("\n Warning!! Out of Max. GP Timer counter!! \n");
	} else {
		u32_counter = (u32_microsecond * 26) - 1;
//		printf("\n GP Timer : %d us!! \n", u32_microsecond);
	}

	/* Setup GPT26M */
	switch (u8_gpt_number) {
	case GPT0:
		/* GPT0 free run */
		writel(readl(A9GPT_GCLK_REG) & ~(0x1 << 0), A9GPT_GCLK_REG);
		/* Disable GPT0 ITR */
		writel(readl(A9GPT_CTL_STA_REG) & ~(0x1 << 4), A9GPT_CTL_STA_REG);
		/* GPT0 ITR mode : default as one-shot to halt mode */
		writel(readl(A9GPT_CTL_STA_REG) & ~(0x1 << 8), A9GPT_CTL_STA_REG);
		/* Clear GPT0 Flag */
		writel(readl(A9GPT_CTL_STA_REG) | (0x1 << 0), A9GPT_CTL_STA_REG);
		/* GPT0 counter value */
		writel(u32_counter, A9GPT_T0_CNT_REG);
		/* Enable GPT0 ITR */
		writel(readl(A9GPT_CTL_STA_REG) | (0x1 << 4), A9GPT_CTL_STA_REG);
		break;
	case GPT1:
		/* GPT1 free run */
		writel(readl(A9GPT_GCLK_REG) & ~(0x1 << 1), A9GPT_GCLK_REG);
		/* Disable GPT1 ITR */
		writel(readl(A9GPT_CTL_STA_REG) & ~(0x1 << 5), A9GPT_CTL_STA_REG);
		/* GPT1 ITR mode : default as one-shot to halt mode */
		writel(readl(A9GPT_CTL_STA_REG) & ~(0x1 << 9), A9GPT_CTL_STA_REG);
		/* Clear GPT1 Flag */
		writel(readl(A9GPT_CTL_STA_REG) | (0x1 << 1), A9GPT_CTL_STA_REG);
		/* GPT1 counter value */
		writel(u32_counter, A9GPT_T1_CNT_REG);
		/* Enable GPT1 ITR */
		writel(readl(A9GPT_CTL_STA_REG) | (0x1 << 5), A9GPT_CTL_STA_REG);
		break;
	default:
		break;
	}

	/* Enable ITR select : GPT_A9_ITR */
	writel(readl(A9ITR_IRQ_REG) | (0x1 << 2), A9ITR_IRQ_REG);
	/* Select A9ITR Priority : 2 -> GPT_A9_ITR */
	//writel(0xFFFFFF20, A9ITR_PRIO1_REG);
	writel(0x76543210, A9ITR_PRIO1_REG);
	writel(0xFEDCBA98, A9ITR_PRIO2_REG);

	/* Halt mode */
	im98xx_hippo_halt();

	/* Disable ITR select : GPT_A9_ITR */
	writel(readl(A9ITR_IRQ_REG) & ~(0x1 << 2), A9ITR_IRQ_REG);
	/* Unselect A9ITR Priority */
	writel(0xFFFFFFFF, A9ITR_PRIO1_REG);
	writel(0xFFFFFFFF, A9ITR_PRIO2_REG);

	switch (u8_gpt_number) {
	case GPT0:
		writel(readl(A9GPT_CTL_STA_REG) | (0x1 << 0), A9GPT_CTL_STA_REG);
		break;
	case GPT1:
		writel(readl(A9GPT_CTL_STA_REG) | (0x1 << 1), A9GPT_CTL_STA_REG);
		break;
	default:
		break;
	}
}

static int do_gpt_halt(int argc, char *argv[])
{
	unsigned char u8_timer;
	unsigned int u32_sleep;

	if (argc < 2)
		return COMMAND_ERROR_USAGE;

	u8_timer = simple_strtoul(argv[1], NULL, 10);
	u32_sleep = simple_strtoul(argv[2], NULL, 10);

	switch (u8_timer) {
		case GPT0:
			im98xx_26MHz_halt(u32_sleep, GPT0);
			break;
		case GPT1:
			im98xx_26MHz_halt(u32_sleep, GPT1);
			break;
		case GPT32K:
			im98xx_32K_halt(u32_sleep);
			BAREBOX_P("\n GP=%d sleep=%d!! \n", u8_timer, u32_sleep);
			break;
		default:
			BAREBOX_P("\n Warning!! Error usage!! \n");
			break;
	}

	return 0;
}

/* default usage to GPTimer1 */
static const __maybe_unused char cmd_gpt_halt_help[] =
"Usage: gpt_halt GPTimer1 us \n";

BAREBOX_CMD_START(gpt_halt)
	.cmd		= do_gpt_halt,
	BAREBOX_CMD_HELP(cmd_gpt_halt_help)
BAREBOX_CMD_END

