/**
 * @file
 * @brief OMAP DPLL and various clock configuration
 *
 * FileName: arch/arm/mach-omap/omap3_clock.c
 *
 * @ref prcm_init This is the second level clock init for PRCM as defined in
 * clocks.h -- called from SRAM, or Flash (using temp SRAM stack).
 *
 * During reconfiguring the clocks while in SDRAM/Flash, we can have invalid
 * clock configuration to which ARM instruction/data fetch ops can fail.
 * This critical path is handled by relocating the relevant functions in
 * omap3_clock_core.S to OMAP's ISRAM and executing it there.
 *
 * @warning IMPORTANT: These functions run from ISRAM stack, so no bss sections
 * should be used, functions cannot use global variables/switch constructs.
 *
 * Originally from http://linux.omap.com/pub/bootloader/3430sdp/u-boot-v1.tar.gz
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


void im98xx_pll_init(void)
{
#if 0
	unsigned int i;
#endif
	unsigned int u32_hclk_div = (0x2 << 24);
	unsigned int val = 0;

	/* step.0 : > 0.5(s) delay for power supply stable */
	//im98xx_26MHz_halt(500000, GPT1);
	im98xx_udelay(500000);

	
#if 0
	for (i = 0 ; i < 100000 ; i++)
		sdelay(26);
#endif

	/* step.1 : check default Clock Switch & default ARM9/XMEM/PARK PLL */

	/* default : ARM9 416 and XMEM 198 for step.2 and step.3 */

	/* step.2: setup ARM9 PLL as 416MHz (default clock for System developing) */
	/* ARM9 */

	val = (PLL_DIVR1 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
#if defined(CONFIG_IM98XX_ARM9_CLOCK_416_CONFIG)
	/* A9, 416MHz = DIVR1 | DIVF32  | DIVQ1,  means 26MHz/1 *32 /2^1 */
	val |= (PLL_DIVF32 | PLL_DIVQ1);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_494_CONFIG)
	/* A9, 494MHz = DIVR1 | DIVF19  | DIVQ0,  means 26MHz/1 *19 /2^0 */
	val |= (PLL_DIVF19 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_520_CONFIG)
	/* A9, 520MHz = DIVR1 | DIVF20  | DIVQ0,  means 26MHz/1 *20 /2^0 */
	val |= (PLL_DIVF20 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_624_CONFIG)
	/* A9, 624MHz = DIVR1 | DIVF24  | DIVQ0,  means 26MHz/1 *24 /2^0 */
	val |= (PLL_DIVF24 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_650_CONFIG)
        /* A9, 650MHz = DIVR1 | DIVF25  | DIVQ0,  means 26MHz/1 *25 /2^0 */
	val |= (PLL_DIVF25 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_702_CONFIG)
	/* A9, 702MHz = DIVR1 | DIVF27  | DIVQ0,  means 26MHz/1 *27 /2^0 */
	val |= (PLL_DIVF27 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_728_CONFIG)
        /* A9, 728MHz = DIVR1 | DIVF31  | DIVQ0,  means 26MHz/1 *28 /2^0 */
	val |= (PLL_DIVF28 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_754_CONFIG)
        /* A9, 754MHz = DIVR1 | DIVF30  | DIVQ0,  means 26MHz/1 *29 /2^0 */
	val |= (PLL_DIVF29 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_780_CONFIG)
        /* A9, 780MHz = DIVR1 | DIVF30  | DIVQ0,  means 26MHz/1 *30 /2^0 */
	val |= (PLL_DIVF30 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_806_CONFIG)
        /* A9, 806MHz = DIVR1 | DIVF31  | DIVQ0,  means 26MHz/1 *31 /2^0 */
	val |= (PLL_DIVF31 | PLL_DIVQ0);
#endif
#if defined(CONFIG_IM98XX_ARM9_CLOCK_910_CONFIG)
        /* A9, 910MHz = DIVR1 | DIVF35  | DIVQ0,  means 26MHz/1 *35 /2^0 */
	val |= (PLL_DIVF35 | PLL_DIVQ0);
#endif
	writel(val, A9_PLL_REG);

/* step.3: setup XMEM PLL as 130MHz (default clock for System developing) */
/* XMEM */
#if defined(CONFIG_IM98XX_XMEM_CLOCK_198_CONFIG)
	/* SDM, 198.25MHz = DIVR2 | DIVF61 | DIVQ2,  means 26MHz /2 *61 /2^2 */
	writel(PLL_DIVR2 | PLL_DIVF61 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, XMEM_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_XMEM_CLOCK_182_CONFIG)
	/* SDM, 182MHz = DIVR1 | DIVF28 | DIVQ2,  means 26MHz /1 *28 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF28 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, XMEM_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_XMEM_CLOCK_169_CONFIG)
	/* SDM, 169MHz = DIVR1 | DIVF26 | DIVQ2,  means 26MHz /1 *26 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF26 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, XMEM_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_XMEM_CLOCK_156_CONFIG)
	/* SDM, 156MHz = DIVR1 | DIVF24 | DIVQ2,  means 26MHz /1 *24 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF24 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, XMEM_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_XMEM_CLOCK_143_CONFIG)
	/* SDM, 143MHz = DIVR1 | DIVF22 | DIVQ2,  means 26MHz /1 *22 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF22 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, XMEM_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_XMEM_CLOCK_130_CONFIG)
	/* SDM, 130MHz = DIVR1 | DIVF20 | DIVQ2,  means 26MHz /1 *20 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF20 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, XMEM_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_XMEM_CLOCK_104_CONFIG)
	/* SDM, 104MHz = DIVR1 | DIVF32 | DIVQ3,  means 26MHz /1 *32 /2^3 */
	writel(PLL_DIVR1 | PLL_DIVF32 | PLL_DIVQ3 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, XMEM_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_XMEM_CLOCK_78_CONFIG)
	/* SDM, 78MHz = DIVR1 | DIVF24 | DIVQ3,  means 26MHz /1 *24 /2^3 */
	writel(PLL_DIVR1 | PLL_DIVF24 | PLL_DIVQ3 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, XMEM_PLL_REG);
#endif

/* step.3: setup ARM7 PLL as 130MHz (default clock for System developing) */
/* ARM7 */
#if defined(CONFIG_IM98XX_ARM7_CLOCK_130_CONFIG)
	/* ARM7, 130MHz = DIVR1 | DIVF20 | DIVQ2,  means 26MHz /1 *20 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF20 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, A7_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_ARM7_CLOCK_143_CONFIG)
	/* ARM7, 143MHz = DIVR1 | DIVF22 | DIVQ2,  means 26MHz /1 *22 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF22 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, A7_PLL_REG);
#endif
#if defined(CONFIG_IM98XX_ARM7_CLOCK_156_CONFIG)
	/* ARM7, 156MHz = DIVR1 | DIVF24 | DIVQ2,  means 26MHz /1 *24 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF24 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, A7_PLL_REG);
#endif

/* step.4: setup PARK PLL as 130MHz (default clock for System developing) */
/* PARK */
	/* PARK, 130MHz = DIVR1 | DIVF20 | DIVQ2,  means 26MHz /1 *20 /2^2 */
	writel(PLL_DIVR1 | PLL_DIVF20 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS, PARK_PLL_REG);

/* step.5: setup USB PLL as 312MHz (default clock for System developing) */
/* USB */
	/* USB, 312MHz = PLL_DIVR1 | PLL_DIVF12 | PLL_DIVQ0, means 26MHz /1 *12 /2^0 */
	writel(PLL_DIVR1 | PLL_DIVF12 | PLL_DIVQ0 | PLL_BS0 | USB_PLL_FSEL | PLL_RESET | PLL_NO_BYPAS, USB_PLL_REG);

/* step.6: de-assert reset ARM9/XMEM/ARM7/PARK PLL */
/*--- De-assert RESET ---*/
	writel(readl(A9_PLL_REG) & ~(PLL_RESET), A9_PLL_REG);		/* De-assert RESET ARM9 */
	writel(readl(XMEM_PLL_REG) & ~(PLL_RESET), XMEM_PLL_REG);	/* De-assert RESET XMEM */
	writel(readl(A7_PLL_REG) & ~(PLL_RESET), A7_PLL_REG);		/* De-assert RESET ARM7 */
	writel(readl(PARK_PLL_REG) & ~(PLL_RESET), PARK_PLL_REG);	/* De-assert RESET PARK */
	writel(readl(USB_PLL_REG) & ~(PLL_RESET), USB_PLL_REG);		/* De-assert RESET USB */

/*--- Wait PLL Stable ---*/
/* ARM9 */
/* step.7: polling ARM9 PLL LOCK, note: if some noise on board, cancel this polling with 200us delay */
	while ((readl(A9_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA) {}  /* wait PLL freq LOCK */
/* step.8: setup ARM9 Clock Switch */
	/* default divisor : divided by 2 */
#if defined(CONFIG_IM98XX_ARM9_HCLK_DIV1_CONFIG)
	u32_hclk_div = (0x1 << 24);
#endif
#if defined(CONFIG_IM98XX_ARM9_HCLK_DIV2_CONFIG)
	u32_hclk_div = (0x2 << 24);
#endif
#if defined(CONFIG_IM98XX_ARM9_HCLK_DIV3_CONFIG)
	u32_hclk_div = (0x3 << 24);
#endif
#if defined(CONFIG_IM98XX_ARM9_HCLK_DIV4_CONFIG)
	u32_hclk_div = (0x4 << 24);
#endif
#if defined(CONFIG_IM98XX_ARM9_HCLK_DIV6_CONFIG)
	u32_hclk_div = (0x6 << 24);
#endif
#if defined(CONFIG_IM98XX_ARM9_HCLK_DIV8_CONFIG)
	u32_hclk_div = (0x8 << 24);
#endif
	/* Switch */
	writel((readl(A9_CLK_CTL_REG) & ~(HCLK_DIV_CLR)) | (u32_hclk_div) | HCLK_DIV_UPDATE, A9_CLK_CTL_REG);
	writel((readl(A9_CLK_CTL_REG) & ~(HCLK_DIV_CLR | A9_CKSW0_CLR | A9_CKSW1_CLR | A9_CKSW2_CLR)) | (u32_hclk_div) | A9_CKSW1_FR_A9PLL | A9_CKSW2_FR_SW1 | HCLK_DIV_UPDATE, A9_CLK_CTL_REG);

	/* XMEM */
	/* step.9: polling XMEM PLL LOCK, note: if some noise on board, cancel this polling with 200us delay */
	while ((readl(XMEM_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA) {}  /* wait PLL freq LOCK */

	/* step.10: setup SDRAM Clock Switch */
	/* Switch */
	writel((readl(SDR_CLK_CTL_REG) & ~(CKSW1_CLR | CKSW2_CLR)) | CKSW1_FR_SDM | CKSW2_FR_SW1 | CLK_DIV_UPDATE | CLK_DIV_EN, SDR_CLK_CTL_REG);

	/* ARM7 */
	/* step.11: polling ARM7 PLL LOCK, note: if some noise on board, cancel this polling with 200us delay */
	while ((readl(A7_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA) {}  /* wait PLL freq LOCK */

	/* step.12: setup ARM7 Clock Switch */
	/* Switch */
	writel((readl(A7_CLKO_CTL_REG) & ~(CKSW2_CLR)) | A7_CKSW2_FR_SW1 | CLK_DIV_UPDATE | CLK_DIV_EN, A7_CLKO_CTL_REG);

	/* step.13: setup DSP Clock Switch */
	/* Switch */
	writel((readl(DSP_CLKO_CTL_REG) & ~(CKSW2_CLR)) | A7_CKSW2_FR_SW1 | CLK_DIV_UPDATE | CLK_DIV_EN, DSP_CLKO_CTL_REG);

/* USB */
/* step.14: polling USB PLL LOCK, note: if some noise on board, cancel this polling with 200us delay */
	while ((readl(USB_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA) {}  /* wait PLL freq LOCK */

	/* step.15: setup USB device PHY Clock Switch */
	val = readl(USBDY_CLK_CTL_REG);
	writel((val & ~(USB_DIV26)), USBDY_CLK_CTL_REG);

	if ((val & USBDY_CLK_READY) != USBDY_CLK_READY) {
		val &= ~(CLK_DIV_EN | USBDY_CLK_RUN);
	} else {
		val &= ~(CLK_DIV_EN);
	}
	writel(val, USBDY_CLK_CTL_REG);

	/* step.16: setup USB host PHY Clock Switch */
#if 0 /* Disable => To setup Host PHY clock by Kernel USB driver */
	writel((readl(USBHY_CLK_CTL_REG) & ~(USB_DIV26)), USBHY_CLK_CTL_REG);
	writel(((readl(USBHY_CLK_CTL_REG) & ~(CLK_DIV_EN))), USBHY_CLK_CTL_REG);
#endif
	/* PARK */

	/* step.17: polling PARK PLL LOCK, note: if some noise on board, cancel this polling with 200us delay */
	while ((readl(PARK_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA);	/* wait PLL freq LOCK */
}

