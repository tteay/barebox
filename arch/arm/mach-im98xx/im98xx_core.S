/**
 * @file
 * @brief Provide Architecture level Initialization
 *
 * FileName: arch/arm/mach-omap/omap3_core.S
 *
 * This provides OMAP3 Architecture initialization. Among these,
 * @li OMAP ROM Code is located in SRAM, we can piggy back on
 * the same addresses
 * @li If clock initialization is required, call the same.
 * @li Setup a temporary SRAM stack which is necessary to call C
 * functions.
 * @li Call architecture initialization function a_init
 *
 * arch_init_lowlevel is enabled if CONFIG_ARCH_HAS_LOWLEVEL_INIT is defined
 * cpy_clk_code is called if CONFIG_OMAP3_COPY_CLOCK_SRAM is defined
 */
/*
 * (C) Copyright 2006-2008
 * Texas Instruments, <www.ti.com>
 * Nishanth Menon <x0nishan@ti.com>
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

#include <config.h>
#include <mach/omap3-silicon.h>
#ifdef CONFIG_ARCH_HAS_LOWLEVEL_INIT

#ifdef CONFIG_ARCH_VECTOR_A7_CONFIG
.globl a7_vector_sdram_start
.globl a7_vector_sdram_end
.globl a7_vector_debug_start
.globl a7_vector_debug_end
#endif

.globl arch_init_lowlevel
arch_init_lowlevel:
	/* Setup a temporary stack so that we can call C functions */
	ldr	sp,	SRAM_STACK
        str     ip,	[sp]    /* stash old link register */
        str     lr,	[sp]    /* stash current link register */
	mov	ip,	lr	/* save link reg across call */
	bl      a_init          /* Architecture init */
        ldr     lr,	[sp]    /* restore current link register */
        ldr     ip,	[sp]    /* restore save ip */

	/* back to arch calling code */
	mov	pc,	lr

arch_start:
	.word	arch_init_lowlevel

//barebox_start:
//	.word	start

SRAM_STACK:
	.word	IM98XX_SRAM_STACK

#ifdef CONFIG_ARCH_VECTOR_A7_CONFIG
a7_vector_sdram_start:
	mov	pc,	#0x40000000 
a7_vector_sdram_end:

a7_vector_debug_start:
	mov	pc,	#0x0
a7_vector_debug_end:
#endif
#endif	/* CONFIG_ARCH_HAS_LOWLEVEL_INIT */

