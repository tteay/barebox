/**
 * @file
 * @brief This defines the Register defines for OMAP GPTimers and Sync32 timers.
 *
 * FileName: include/asm-arm/arch-omap/timers.h
 *
 * Originally from Linux kernel:
 * http://linux.omap.com/pub/kernel/3430zoom/linux-ldp-v1.0b.tar.gz
 *
 */
/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Nishanth Menon <x0nishan@ti.com>
 *
 * Copyright (C) 2007 Texas Instruments, <www.ti.com>
 * Copyright (C) 2007 Nokia Corporation.
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
#ifndef __ASM_ARCH_GPT_H
#define __ASM_ARCH_GPT_H

/** 32Khz Timer registers */
#define S32K_CR			(0xF8001500)
#define S32K_ST			(0xF8001504)
#define S32K_FREQUENCY		32768

#endif	/*__ASM_ARCH_GPT_H */
