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

#if 0
/* Following functions are exported from omap3_clock_core.S */
#ifdef CONFIG_OMAP3_COPY_CLOCK_SRAM
/* A.K.A go_to_speed */
static void (*f_lock_pll) (u32, u32, u32, u32);
#endif
/* Helper functions */
static u32 get_osc_clk_speed(void);
static void get_sys_clkin_sel(u32 osc_clk, u32 *sys_clkin_sel);
static void per_clocks_enable(void);

/**
 * @brief Determine reference oscillator speed
 *
 *  This is based on known 32kHz clock and gptimer.
 *
 * @return clock speed S38_4M, S26M S24M S19_2M S13M S12M
 */
static u32 get_osc_clk_speed(void)
{
	u32 start, cstart, cend, cdiff, val;

	val = readl(PRM_REG(CLKSRC_CTRL));
	/* If SYS_CLK is being divided by 2, remove for now */
	val = (val & (~(0x1 << 7))) | (0x1 << 6);
	writel(val, PRM_REG(CLKSRC_CTRL));

	/* enable timer2 */
	val = readl(CM_REG(CLKSEL_WKUP)) | (0x1 << 0);
	writel(val, CM_REG(CLKSEL_WKUP));	/* select sys_clk for GPT1 */

	/* Enable I and F Clocks for GPT1 */
	val = readl(CM_REG(ICLKEN_WKUP)) | (0x1 << 0) | (0x1 << 2);
	writel(val, CM_REG(ICLKEN_WKUP));
	val = readl(CM_REG(FCLKEN_WKUP)) | (0x1 << 0);
	writel(val, CM_REG(FCLKEN_WKUP));
	/* start counting at 0 */
	writel(0, OMAP_GPTIMER1_BASE + TLDR);
	/* enable clock */
	writel(GPT_EN, OMAP_GPTIMER1_BASE + TCLR);
	/* enable 32kHz source - enabled out of reset */
	/* determine sys_clk via gauging */

	start = 20 + readl(S32K_CR);	/* start time in 20 cycles */
	while (readl(S32K_CR) < start) ;	/* dead loop till start time */
	/* get start sys_clk count */
	cstart = readl(OMAP_GPTIMER1_BASE + TCRR);
	while (readl(S32K_CR) < (start + 20)) ;	/* wait for 40 cycles */
	/* get end sys_clk count */
	cend = readl(OMAP_GPTIMER1_BASE + TCRR);
	cdiff = cend - cstart;	/* get elapsed ticks */

	/* based on number of ticks assign speed */
	if (cdiff > 19000)
		return S38_4M;
	else if (cdiff > 15200)
		return S26M;
	else if (cdiff > 13000)
		return S24M;
	else if (cdiff > 9000)
		return S19_2M;
	else if (cdiff > 7600)
		return S13M;
	else
		return S12M;
}

/**
 * @brief Returns the sys_clkin_sel field value
 *
 * This is based on input oscillator clock frequency.
 *
 * @param[in] osc_clk - Oscilaltor Clock to OMAP
 * @param[out] sys_clkin_sel - returns the sys_clk selection
 *
 * @return void
 */
static void get_sys_clkin_sel(u32 osc_clk, u32 *sys_clkin_sel)
{
	if (osc_clk == S38_4M)
		*sys_clkin_sel = 4;
	else if (osc_clk == S26M)
		*sys_clkin_sel = 3;
	else if (osc_clk == S19_2M)
		*sys_clkin_sel = 2;
	else if (osc_clk == S13M)
		*sys_clkin_sel = 1;
	else if (osc_clk == S12M)
		*sys_clkin_sel = 0;
}

/**
 * @brief Inits clocks for PRCM
 *
 * This is called from SRAM, or Flash (using temp SRAM stack).
 * if CONFIG_OMAP3_COPY_CLOCK_SRAM is defined, @ref go_to_speed
 *
 * @return void
 */
void prcm_init(void)
{
	int xip_safe;
	u32 osc_clk = 0, sys_clkin_sel = 0;
	u32 clk_index, sil_index = 0;
	struct dpll_param *dpll_param_p;
#ifdef CONFIG_OMAP3_COPY_CLOCK_SRAM
	int p0, p1, p2, p3;
	f_lock_pll = (void *)(OMAP_SRAM_INTVECT + OMAP_SRAM_INTVECT_COPYSIZE);
#endif

	xip_safe = running_in_sram();

	/* Gauge the input clock speed and find out the sys_clkin_sel
	 * value corresponding to the input clock.
	 */
	osc_clk = get_osc_clk_speed();
	get_sys_clkin_sel(osc_clk, &sys_clkin_sel);
	/* set input crystal speed */
	sr32(PRM_REG(CLKSEL), 0, 3, sys_clkin_sel);

	/* If the input clock is greater than 19.2M always divide/2 */
	if (sys_clkin_sel > 2) {
		/* input clock divider */
		sr32(PRM_REG(CLKSRC_CTRL), 6, 2, 2);
		clk_index = sys_clkin_sel / 2;
	} else {
		/* input clock divider */
		sr32(PRM_REG(CLKSRC_CTRL), 6, 2, 1);
		clk_index = sys_clkin_sel;
	}

	/* Unlock MPU DPLL (slows things down, and needed later) */
	sr32(CM_REG(CLKEN_PLL_MPU), 0, 3, PLL_LOW_POWER_BYPASS);
	wait_on_value((0x1 << 0), 0, CM_REG(IDLEST_PLL_MPU), LDELAY);

	/* Getting the base address of Core DPLL param table */
	dpll_param_p = (struct dpll_param *)get_core_dpll_param();
	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + MAX_SIL_INDEX * clk_index + sil_index;
	if (xip_safe) {
		/* CORE DPLL */
		/* sr32(CM_REG(CLKSEL2_EMU)) set override to work when asleep */
		sr32(CM_REG(CLKEN_PLL), 0, 3, PLL_FAST_RELOCK_BYPASS);
		wait_on_value((0x1 << 0), 0, CM_REG(IDLEST_CKGEN), LDELAY);
		/* For 3430 ES1.0 Errata 1.50, default value directly doesnt
		   work. write another value and then default value. */
		sr32(CM_REG(CLKSEL1_EMU), 16, 5, CORE_M3X2 + 1);
		sr32(CM_REG(CLKSEL1_EMU), 16, 5, CORE_M3X2);
		sr32(CM_REG(CLKSEL1_PLL), 27, 2, dpll_param_p->m2);
		sr32(CM_REG(CLKSEL1_PLL), 16, 11, dpll_param_p->m);
		sr32(CM_REG(CLKSEL1_PLL), 8, 7, dpll_param_p->n);
		sr32(CM_REG(CLKSEL1_PLL), 6, 1, 0);
		sr32(CM_REG(CLKSEL_CORE), 8, 4, CORE_SSI_DIV);
		sr32(CM_REG(CLKSEL_CORE), 4, 2, CORE_FUSB_DIV);
		sr32(CM_REG(CLKSEL_CORE), 2, 2, CORE_L4_DIV);
		sr32(CM_REG(CLKSEL_CORE), 0, 2, CORE_L3_DIV);
		sr32(CM_REG(CLKSEL_GFX), 0, 3, GFX_DIV);
		sr32(CM_REG(CLKSEL_WKUP), 1, 2, WKUP_RSM);
		sr32(CM_REG(CLKEN_PLL), 4, 4, dpll_param_p->fsel);
		sr32(CM_REG(CLKEN_PLL), 0, 3, PLL_LOCK);
		wait_on_value((0x1 << 0), 1, CM_REG(IDLEST_CKGEN), LDELAY);
	} else if (running_in_flash()) {
#ifdef CONFIG_OMAP3_COPY_CLOCK_SRAM
		/* if running from flash,
		 * jump to small relocated code area in SRAM.
		 */
		p0 = readl(CM_REG(CLKEN_PLL));
		sr32((u32) &p0, 0, 3, PLL_FAST_RELOCK_BYPASS);
		sr32((u32) &p0, 4, 4, dpll_param_p->fsel);

		p1 = readl(CM_REG(CLKSEL1_PLL));
		sr32((u32) &p1, 27, 2, dpll_param_p->m2);
		sr32((u32) &p1, 16, 11, dpll_param_p->m);
		sr32((u32) &p1, 8, 7, dpll_param_p->n);
		sr32((u32) &p1, 6, 1, 0);	/* set source for 96M */
		p2 = readl(CM_REG(CLKSEL_CORE));
		sr32((u32) &p2, 8, 4, CORE_SSI_DIV);
		sr32((u32) &p2, 4, 2, CORE_FUSB_DIV);
		sr32((u32) &p2, 2, 2, CORE_L4_DIV);
		sr32((u32) &p2, 0, 2, CORE_L3_DIV);

		p3 = CM_REG(IDLEST_CKGEN);

		(*f_lock_pll) (p0, p1, p2, p3);
#else
		/***Oopps.. Wrong .config!! *****/
		hang();
#endif
	}

	/* PER DPLL */
	sr32(CM_REG(CLKEN_PLL), 16, 3, PLL_STOP);
	wait_on_value((0x1 << 1), 0, CM_REG(IDLEST_CKGEN), LDELAY);

	/* Getting the base address to PER  DPLL param table */
	/* Set N */
	dpll_param_p = (struct dpll_param *)get_per_dpll_param();
	/* Moving it to the right sysclk base */
	dpll_param_p = dpll_param_p + clk_index;
	/* Errata 1.50 Workaround for 3430 ES1.0 only */
	/* If using default divisors, write default divisor + 1
	   and then the actual divisor value */
	/* Need to change it to silicon and revisino check */
	if (1) {
		sr32(CM_REG(CLKSEL1_EMU), 24, 5, PER_M6X2 + 1);	/* set M6 */
		sr32(CM_REG(CLKSEL1_EMU), 24, 5, PER_M6X2);	/* set M6 */
		sr32(CM_REG(CLKSEL_CAM), 0, 5, PER_M5X2 + 1);	/* set M5 */
		sr32(CM_REG(CLKSEL_CAM), 0, 5, PER_M5X2);	/* set M5 */
		sr32(CM_REG(CLKSEL_DSS), 0, 5, PER_M4X2 + 1);	/* set M4 */
		sr32(CM_REG(CLKSEL_DSS), 0, 5, PER_M4X2);	/* set M4 */
		sr32(CM_REG(CLKSEL_DSS), 8, 5, PER_M3X2 + 1);	/* set M3 */
		sr32(CM_REG(CLKSEL_DSS), 8, 5, PER_M3X2);	/* set M3 */
		/* set M2 */
		sr32(CM_REG(CLKSEL3_PLL), 0, 5, dpll_param_p->m2 + 1);
		sr32(CM_REG(CLKSEL3_PLL), 0, 5, dpll_param_p->m2);
	} else {
		sr32(CM_REG(CLKSEL1_EMU), 24, 5, PER_M6X2);	/* set M6 */
		sr32(CM_REG(CLKSEL_CAM), 0, 5, PER_M5X2);	/* set M5 */
		sr32(CM_REG(CLKSEL_DSS), 0, 5, PER_M4X2);	/* set M4 */
		sr32(CM_REG(CLKSEL_DSS), 8, 5, PER_M3X2);	/* set M3 */
		sr32(CM_REG(CLKSEL3_PLL), 0, 5, dpll_param_p->m2);
	}
	sr32(CM_REG(CLKSEL2_PLL), 8, 11, dpll_param_p->m);	/* set m */
	sr32(CM_REG(CLKSEL2_PLL), 0, 7, dpll_param_p->n);	/* set n */
	sr32(CM_REG(CLKEN_PLL), 20, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_REG(CLKEN_PLL), 16, 3, PLL_LOCK);	/* lock mode */
	wait_on_value((0x1 << 1), 2, CM_REG(IDLEST_CKGEN), LDELAY);

	/* Getting the base address to MPU DPLL param table */
	dpll_param_p = (struct dpll_param *)get_mpu_dpll_param();
	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + MAX_SIL_INDEX * clk_index + sil_index;
	/* MPU DPLL (unlocked already) */
	sr32(CM_REG(CLKSEL2_PLL_MPU), 0, 5, dpll_param_p->m2);	/* Set M2 */
	sr32(CM_REG(CLKSEL1_PLL_MPU), 8, 11, dpll_param_p->m);	/* Set M */
	sr32(CM_REG(CLKSEL1_PLL_MPU), 0, 7, dpll_param_p->n);	/* Set N */
	sr32(CM_REG(CLKEN_PLL_MPU), 4, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_REG(CLKEN_PLL_MPU), 0, 3, PLL_LOCK);	/* lock mode */
	wait_on_value((0x1 << 0), 1, CM_REG(IDLEST_PLL_MPU), LDELAY);

	/* Getting the base address to IVA DPLL param table */
	dpll_param_p = (struct dpll_param *)get_iva_dpll_param();
	/* Moving it to the right sysclk and ES rev base */
	dpll_param_p = dpll_param_p + MAX_SIL_INDEX * clk_index + sil_index;
	/* IVA DPLL (set to 12*20=240MHz) */
	sr32(CM_REG(CLKEN_PLL_IVA2), 0, 3, PLL_STOP);
	wait_on_value((0x1 << 0), 0, CM_REG(IDLEST_PLL_IVA2), LDELAY);
	sr32(CM_REG(CLKSEL2_PLL_IVA2), 0, 5, dpll_param_p->m2);	/* set M2 */
	sr32(CM_REG(CLKSEL1_PLL_IVA2), 8, 11, dpll_param_p->m);	/* set M */
	sr32(CM_REG(CLKSEL1_PLL_IVA2), 0, 7, dpll_param_p->n);	/* set N */
	sr32(CM_REG(CLKEN_PLL_IVA2), 4, 4, dpll_param_p->fsel);	/* FREQSEL */
	sr32(CM_REG(CLKEN_PLL_IVA2), 0, 3, PLL_LOCK);	/* lock mode */
	wait_on_value((0x1 << 0), 1, CM_REG(IDLEST_PLL_IVA2), LDELAY);

	/* Set up GPTimers to sys_clk source only */
	sr32(CM_REG(CLKSEL_PER), 0, 8, 0xff);
	sr32(CM_REG(CLKSEL_WKUP), 0, 1, 1);

	sdelay(5000);

	/* Enable Peripheral Clocks */
	per_clocks_enable();
}

/**
 * @brief Enable the clks & power for perifs
 *
 * GPT2 Sysclk, ICLK,FCLK, 32k Sync is enabled by default
 * Uses CONFIG_OMAP_CLOCK_UART to enable UART clocks
 * Uses CONFIG_OMAP_CLOCK_I2C to enable I2C clocks
 * Uses CONFIG_OMAP_CLOCK_ALL to enable All Clocks!
 *    - Not a wise idea in most cases
 *
 * @return void
 */
static void per_clocks_enable(void)
{
	/* Enable GP2 timer. */
	sr32(CM_REG(CLKSEL_PER), 0, 1, 0x1);	/* GPT2 = sys clk */
	sr32(CM_REG(ICLKEN_PER), 3, 1, 0x1);	/* ICKen GPT2 */
	sr32(CM_REG(FCLKEN_PER), 3, 1, 0x1);	/* FCKen GPT2 */
	/* Enable the ICLK for 32K Sync Timer as its used in udelay */
	sr32(CM_REG(ICLKEN_WKUP), 2, 1, 0x1);

#ifdef CONFIG_OMAP_CLOCK_UART
	/* Enable UART1 clocks */
	sr32(CM_REG(FCLKEN1_CORE), 13, 1, 0x1);
	sr32(CM_REG(ICLKEN1_CORE), 13, 1, 0x1);
#endif
#ifdef CONFIG_OMAP_CLOCK_I2C
	/* Turn on all 3 I2C clocks */
	sr32(CM_REG(FCLKEN1_CORE), 15, 3, 0x7);
	sr32(CM_REG(ICLKEN1_CORE), 15, 3, 0x7);	/* I2C1,2,3 = on */
#endif

#ifdef CONFIG_OMAP_CLOCK_ALL
#define FCK_IVA2_ON	0x00000001
#define FCK_CORE1_ON	0x03fffe29
#define ICK_CORE1_ON	0x3ffffffb
#define ICK_CORE2_ON	0x0000001f
#define	FCK_WKUP_ON	0x000000e9
#define ICK_WKUP_ON	0x0000003f
#define FCK_DSS_ON	0x00000005	/* tv+dss1 (not dss2) */
#define ICK_DSS_ON	0x00000001
#define FCK_CAM_ON	0x00000001
#define ICK_CAM_ON	0x00000001
#define FCK_PER_ON	0x0003ffff
#define ICK_PER_ON	0x0003ffff
	sr32(CM_REG(FCLKEN_IVA2), 0, 32, FCK_IVA2_ON);
	sr32(CM_REG(FCLKEN1_CORE), 0, 32, FCK_CORE1_ON);
	sr32(CM_REG(ICLKEN1_CORE), 0, 32, ICK_CORE1_ON);
	sr32(CM_REG(ICLKEN2_CORE), 0, 32, ICK_CORE2_ON);
	sr32(CM_REG(FCLKEN_WKUP), 0, 32, FCK_WKUP_ON);
	sr32(CM_REG(ICLKEN_WKUP), 0, 32, ICK_WKUP_ON);
	sr32(CM_REG(FCLKEN_DSS), 0, 32, FCK_DSS_ON);
	sr32(CM_REG(ICLKEN_DSS), 0, 32, ICK_DSS_ON);
	sr32(CM_REG(FCLKEN_CAM), 0, 32, FCK_CAM_ON);
	sr32(CM_REG(ICLKEN_CAM), 0, 32, ICK_CAM_ON);
	sr32(CM_REG(FCLKEN_PER), 0, 32, FCK_PER_ON);
	sr32(CM_REG(ICLKEN_PER), 0, 32, ICK_PER_ON);
#endif
	/* Settle down my friend */
	sdelay(1000);
}
#endif

#if 0
/* Park-in only for ARM9/XMEM */
void im98xx_park_in(unsigned char u8_clock_id)
{
	switch (u8_clock_id) {
		case ARM9_SW:
			writel(readl(A9_CLK_CTL_REG) | ARM9_PARK, A9_CLK_CTL_REG);
			break;
		case XMEM_SW:
			writel(readl(SDR_CLK_CTL_REG) | XMEM_PARK, SDR_CLK_CTL_REG);
			break;
		default:
			/* printf("Unknown park-in ID!!\n"); */
			break;
	}
}

/* Park-in only for ARM9/XMEM */
void im98xx_park_out(unsigned char u8_clock_id)
{
	switch (u8_clock_id) {
		case ARM9_SW:
			writel(readl(A9_CLK_CTL_REG) & ~(ARM9_PARK), A9_CLK_CTL_REG);
			break;
		case XMEM_SW:
			writel(readl(SDR_CLK_CTL_REG) & ~(XMEM_PARK), SDR_CLK_CTL_REG);
			break;
		default:
			/* printf("Unknown park-out ID!!\n"); */
			break;
	}
}

void im98xx_pll_setup(unsigned short u16_clock_id, unsigned short u16_clock_out)
{
	unsigned int u32_div_value = 0;

	switch (u16_clock_out)
	{
		case 52:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF16 | PLL_DIVQ3 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 104:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF32 | PLL_DIVQ3 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 130:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF20 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 143:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF22 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 156:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF24 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 198:
			u32_div_value = (PLL_DIVR2 | PLL_DIVF61 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 208:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF32 | PLL_DIVQ2 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 260:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF20 | PLL_DIVQ1 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 416:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF32 | PLL_DIVQ1 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		case 520:
			u32_div_value = (PLL_DIVR1 | PLL_DIVF20 | PLL_DIVQ0 | PLL_BS1 | PLL_RESET | PLL_NO_BYPAS);
			break;
		default:
			/* printf("Unknown clock!!\n"); */
			break;
	}

	switch (u16_clock_id) {
		case ARM9_ID:
			writel(u32_div_value, A9_PLL_REG);
			writel(readl(A9_PLL_REG) & ~(PLL_RESET), A9_PLL_REG); /* De-assert RESET */
			while ((readl(A9_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA) {}  /* wait PLL freq LOCK */
			writel(u16_clock_out, WDT_SW1_REG); /* Note: Turbo mode for SW Clock development */
			break;
		case PARK_ID:
			writel(u32_div_value, PARK_PLL_REG);
			writel(readl(PARK_PLL_REG) & ~(PLL_RESET), PARK_PLL_REG); /* De-assert RESET */
			while ((readl(PARK_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA) {}   /* wait PLL freq LOCK */
			break;
		case XMEM_ID:
			writel(u32_div_value, XMEM_PLL_REG);
			writel(readl(XMEM_PLL_REG) & ~(PLL_RESET), XMEM_PLL_REG); /* De-assert RESET */
			while ((readl(XMEM_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA) {}  /* wait PLL freq LOCK */
			break;
/* Note: this is only for ARM9/XMEM/PARK setting */
#if 0
		case USB_ID:
			writel(u32_div_value, USB_PLL_REG);
			break;
#endif
		case ARM7_ID:
			writel(u32_div_value, A7_PLL_REG);
			writel(readl(A7_PLL_REG) & ~(PLL_RESET), A7_PLL_REG); /* De-assert RESET */
			while ((readl(A7_PLL_REG) & PLL_LOCK_INDCA) != PLL_LOCK_INDCA) {}  /* wait PLL freq LOCK */
			break;
		default:
//			printf("iM98XX PLL setup Error: Unknown PLL ID!!\n");
			break;
	}
}

void im98xx_clock_switch_init(void)
{
	/* ARM9 Clock Switch */
	writel((readl(A9_CLK_CTL_REG) & ~(HCLK_DIV_CLR)) | HCLK_DIV_2 | HCLK_DIV_UPDATE, A9_CLK_CTL_REG);
	writel((readl(A9_CLK_CTL_REG) & ~(HCLK_DIV_CLR | A9_CKSW0_CLR | A9_CKSW1_CLR | A9_CKSW2_CLR)) | HCLK_DIV_2 | A9_CKSW1_FR_A9PLL | A9_CKSW2_FR_SW1 | HCLK_DIV_UPDATE, A9_CLK_CTL_REG);

	/* SDRAM Clock Switch */
	writel((readl(SDR_CLK_CTL_REG) & ~(CKSW1_CLR | CKSW2_CLR)) | CKSW1_FR_SDM | CKSW2_FR_SW1, SDR_CLK_CTL_REG);

	/* ARM7 Clock Switch */
	writel((readl(A7_CLKO_CTL_REG) & ~(CKSW1_CLR | CKSW2_CLR)) | A7_CKSW1_FR_A7PLL | A7_CKSW2_FR_SW1, A7_CLKO_CTL_REG);

	/* USB Host Clock Switch */
	writel((readl(USBHY_CLK_CTL_REG) & ~(CKSW1_CLR | CKSW2_CLR)) | CKSW1_FR_SDM | CKSW2_FR_SW1, USBHY_CLK_CTL_REG);
}
#endif

void im98xx_pll_init(void)
{
#if 0
	unsigned int i;
#endif
	unsigned int u32_hclk_div = (0x2 << 24);
	unsigned int val = 0;

	/* step.0 : > 0.5(s) delay for power supply stable */
	im98xx_26MHz_halt(500000, GPT1);
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

