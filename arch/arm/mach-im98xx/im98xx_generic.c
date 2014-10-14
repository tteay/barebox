/**
 * @file
 * @brief Provide Generic implementations for OMAP3 architecture
 *
 * FileName: arch/arm/mach-omap/omap3_generic.c
 *
 * This file contains the generic implementations of various OMAP3
 * relevant functions
 * For more info on OMAP34XX, see http://focus.ti.com/pdfs/wtbu/swpu114g.pdf
 *
 * Important one is @ref a_init which is architecture init code.
 * The implemented functions are present in sys_info.h
 *
 * Originally from http://linux.omap.com/pub/bootloader/3430sdp/u-boot-v1.tar.gz
 */
/*
 * (C) Copyright 2006-2008
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
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
#include <common.h>
#include <init.h>
#include <asm/io.h>
#include <mach/syslib.h>
#include <mach/omap3-clock.h>
#include <mach/magic.h>

/**
 * @brief Reset the CPU
 *
 * In case of crashes, reset the CPU
 *
 * @param[in] addr -Cause of crash
 *
 * @return void
 */
void __noreturn reset_cpu(ulong addr)
{
	/* FIXME: Enable WDT and cause reset */
	im98xx_a9_wdt_reset();
	hang();
}
EXPORT_SYMBOL(reset_cpu);

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

/**
 * @brief Are we running in Flash XIP?
 *
 * If the base is in GPMC address space, we probably are!
 *
 * @return 1 if we are running in XIP mode, else return 0
 */
u32 running_in_flash(void)
{
	if (get_base() < 4)
		return 1;	/* in flash */
	return 0;		/* running in SRAM or SDRAM */
}

/**
 * @brief Are we running in OMAP internal SRAM?
 *
 * If in SRAM address, then yes!
 *
 * @return  1 if we are running in SRAM, else return 0
 */
u32 running_in_sram(void)
{
	if (get_base() == 4)
		return 1;	/* in SRAM */
	return 0;		/* running in FLASH or SDRAM */
}

/**
 * @brief Are we running in SDRAM?
 *
 * if we are not in GPMC nor in SRAM address space,
 * we are in SDRAM execution area
 *
 * @return 1 if we are running from SDRAM, else return 0
 */
u32 running_in_sdram(void)
{
	if (get_base() > 4)
		return 1;	/* in sdram */
	return 0;		/* running in SRAM or FLASH */
}
EXPORT_SYMBOL(running_in_sdram);


/**
 * @brief Uart port register read function for OMAP3
 *
 * @param base base address of UART
 * @param reg_idx register index
 *
 * @return character read from register
 */
unsigned int im98xx_uart_read(unsigned long base, unsigned char reg_idx)
{
	unsigned int *reg_addr = (unsigned int *)base;
	reg_addr += reg_idx;
	return readl(reg_addr);
}
EXPORT_SYMBOL(im98xx_uart_read);

/**
 * @brief Uart port register write function for OMAP3
 *
 * @param val value to write
 * @param base base address of UART
 * @param reg_idx register index
 *
 * @return void
 */
void im98xx_uart_write(unsigned int val, unsigned long base,
		     unsigned char reg_idx)
{
	unsigned int *reg_addr = (unsigned int *)base;
	reg_addr += reg_idx;
	writel(val, reg_addr);
}
EXPORT_SYMBOL(im98xx_uart_write);
