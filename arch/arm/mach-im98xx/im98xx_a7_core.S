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

#ifdef CONFIG_ARCH_VECTOR_A7_CONFIG
.globl a7_vector_sdram_start
.globl a7_vector_sdram_end
.globl a7_vector_debug_start
.globl a7_vector_debug_end

a7_vector_sdram_start:
	mov	pc,	#0x40000000 
a7_vector_sdram_end:

a7_vector_debug_start:
	mov	pc,	#0x0
a7_vector_debug_end:
#endif
 

