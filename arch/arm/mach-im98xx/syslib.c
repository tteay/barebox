/**
 * @file
 * @brief Provide OMAP independent utility APIs
 *
 * FileName: arch/arm/mach-omap/syslib.c
 *
 * Provide APIs which can be used from platform/architecture code
 * to operate on
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

#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <mach/syslib.h>

/**
 * @brief simple spin loop
 *
 * Will be constant time as its generally used in bypass conditions only.
 * This is necessary until timers are accessible. if you need timed delays
 * use @ref mdelay or @ref udelay instead
 *
 * @param[in] loops number of loops
 *
 * @return void
 */
void sdelay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
					  "bne 1b":"=r" (loops):"0"(loops));
}

/**
 * @brief clear & set a value in a bit range for a 32 bit address
 *
 * @param[in] addr Address to set/read from
 * @param[in] start_bit Where to put the value
 * @param[in] num_bits number of bits the value should be set
 * @param[in] value the value to set
 *
 * @return void
 */
void sr32(u32 addr, u32 start_bit, u32 num_bits, u32 value)
{
	u32 tmp;
	u32 msk = 1 << num_bits;
	--msk;
	tmp = readl(addr) & ~(msk << start_bit);
	tmp |= value << start_bit;
	writel(tmp, addr);
}

/**
 * @brief common routine to allow waiting for changes in volatile regs.
 *
 * @param[in] read_bit_mask  the bit mask to read
 * @param[in] match_value  match to which value
 * @param[in] read_addr  address to read from
 * @param[in] bound max iterations
 *
 * @return 1 if match_value is found, else if bound iterations reached,
 *         returns 0
 */
u32 wait_on_value(u32 read_bit_mask, u32 match_value, u32 read_addr, u32 bound)
{
	u32 i = 0, val;
	do {
		++i;
		val = readl(read_addr) & read_bit_mask;
		if (val == match_value)
			return 1;
		if (i == bound)
			return 0;
	} while (1);
}

