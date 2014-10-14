/**
 * @file
 * @brief Contains the PRM and CM definitions
 *
 * FileName: include/asm-arm/arch-omap/omap3-clock.h
 *
 * Originally from http://linux.omap.com/pub/bootloader/3430sdp/u-boot-v1.tar.gz
 *
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
#ifndef _OMAP343X_CLOCKS_H_
#define _OMAP343X_CLOCKS_H_

/** CM Clock Regs Wrapper */
#define CM_REG(REGNAME)	(OMAP_CM_BASE + CM_##REGNAME)

#define CM_FCLKEN_IVA2		0X0000
#define CM_CLKEN_PLL_IVA2	0X0004
#define CM_IDLEST_PLL_IVA2	0X0024
#define CM_CLKSEL1_PLL_IVA2	0X0040
#define CM_CLKSEL2_PLL_IVA2	0X0044
#define CM_CLKEN_PLL_MPU	0X0904
#define CM_IDLEST_PLL_MPU	0X0924
#define CM_CLKSEL1_PLL_MPU	0X0940
#define CM_CLKSEL2_PLL_MPU	0X0944
#define CM_FCLKEN1_CORE		0X0A00
#define CM_ICLKEN1_CORE		0X0A10
#define CM_ICLKEN2_CORE		0X0A14
#define CM_CLKSEL_CORE		0X0A40
#define CM_FCLKEN_GFX		0X0B00
#define CM_ICLKEN_GFX		0X0B10
#define CM_CLKSEL_GFX		0X0B40
#define CM_FCLKEN_WKUP		0X0C00
#define CM_ICLKEN_WKUP		0X0C10
#define CM_CLKSEL_WKUP		0X0C40
#define CM_IDLEST_WKUP		0X0C20
#define CM_CLKEN_PLL		0X0D00
#define CM_IDLEST_CKGEN		0X0D20
#define CM_CLKSEL1_PLL		0X0D40
#define CM_CLKSEL2_PLL		0X0D44
#define CM_CLKSEL3_PLL		0X0D48
#define CM_FCLKEN_DSS		0X0E00
#define CM_ICLKEN_DSS		0X0E10
#define CM_CLKSEL_DSS		0X0E40
#define CM_FCLKEN_CAM		0X0F00
#define CM_ICLKEN_CAM		0X0F10
#define CM_CLKSEL_CAM		0X0f40
#define CM_FCLKEN_PER		0X1000
#define CM_ICLKEN_PER		0X1010
#define CM_CLKSEL_PER		0X1040
#define CM_CLKSEL1_EMU		0X1140

/** PRM Clock Regs */
#define PRM_REG(REGNAME)	(OMAP_PRM_BASE + PRM_##REGNAME)
#define PRM_CLKSEL		0x0D40
#define PRM_RSTCTRL		0x1250
#define PRM_CLKSRC_CTRL		0x1270

/*************** Clock Values */
#define PLL_STOP		1	/* PER & IVA */
#define PLL_LOW_POWER_BYPASS	5	/* MPU, IVA & CORE */
#define PLL_FAST_RELOCK_BYPASS	6	/* CORE */
#define PLL_LOCK		7	/* MPU, IVA, CORE & PER */

/* The following configurations are OPP and SysClk value independant
 * and hence are defined here.
 */

/* CORE DPLL */
#define CORE_M3X2		2        /* 332MHz : CM_CLKSEL1_EMU */
#define CORE_SSI_DIV		3        /* 221MHz : CM_CLKSEL_CORE */
#define CORE_FUSB_DIV		2        /* 41.5MHz: */
#define CORE_L4_DIV		2        /*  83MHz : L4 */
#define CORE_L3_DIV		2        /* 166MHz : L3 {DDR} */
#define GFX_DIV			2        /*  83MHz : CM_CLKSEL_GFX */
#define WKUP_RSM		2        /* 41.5MHz: CM_CLKSEL_WKUP */

/* PER DPLL */
#define PER_M6X2		3         /* 288MHz: CM_CLKSEL1_EMU */
#define PER_M5X2		4         /* 216MHz: CM_CLKSEL_CAM */
#define PER_M4X2		9         /* 96MHz : CM_CLKSEL_DSS-dss1 */
#define PER_M3X2		16        /* 54MHz : CM_CLKSEL_DSS-tv */

#define CLSEL1_EMU_VAL ((CORE_M3X2 << 16) | (PER_M6X2 << 24) | (0x0a50))

#define MAX_SIL_INDEX	1

#ifndef __ASSEMBLY__
void prcm_init(void);
/* Used to index into DPLL parameter tables -See TRM for further details */
struct dpll_param {
	unsigned int m;
	unsigned int n;
	unsigned int fsel;
	unsigned int m2;
};
/* External functions see omap3_clock_core.S */
extern struct dpll_param *get_mpu_dpll_param(void);
extern struct dpll_param *get_iva_dpll_param(void);
extern struct dpll_param *get_core_dpll_param(void);
extern struct dpll_param *get_per_dpll_param(void);

#endif /* __ASSEMBLY__ */

void im98xx_pll_init(void);
void im98xx_pm_init(void);
void im98xx_a9_wdt_reset(void);
void im98xx_26MHz_halt(unsigned int u32_microsecond, unsigned char u8_gpt_number);
void im98xx_pll_setup(unsigned short u16_clock_id, unsigned short u16_clock_out);
void im98xx_park_in(unsigned char u8_clock_id);
void im98xx_park_out(unsigned char u8_clock_id);
void im98xx_saradc_init(void);
#endif  /* endif _OMAP343X_CLOCKS_H_ */
