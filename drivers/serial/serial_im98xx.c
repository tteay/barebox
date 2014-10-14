/**
 * @file
 * @brief IM98XX Driver implementation
 *
 * FileName: drivers/serial/serial_ns16550.c
 *
 * IM98XX support
 * Modified from u-boot drivers/serial.c and drivers/ns16550.c
 * originally from linux source (arch/ppc/boot/ns16550.c)
 * modified to use CFG_ISA_MEM and new defines
 */
/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Nishanth Menon <x0nishan@ti.com>
 *
 * (C) Copyright 2000
 * Rob Taylor, Flying Pig Systems. robt@flyingpig.com.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
#include <driver.h>
#include <init.h>
#include <errno.h>
#include <malloc.h>
#include <asm/io.h>

#include "serial_im98xx.h"
#include <mach/im98xx.h>
#include <mach/magic.h>

/*********** Private Functions **********************************/

/**
 * @brief Compute the divisor for a baud rate
 *
 * @param[in] cdev pointer to console device
 * @param[in] baudrate baud rate
 *
 * @return divisor to be set
 */
static unsigned int im98xx_calc_divisor(struct console_device *cdev,
					unsigned int baudrate)
{
	struct IM98XX_plat *plat =
			(struct IM98XX_plat *)cdev->dev->platform_data;

	return (plat->clock / baudrate);
}

/**
 * @brief Initialize the device
 *
 * @param[in] cdev pointer to console device
 */
static void im98xx_serial_init_port(struct console_device *cdev)
{
	struct IM98XX_plat *plat =
			(struct IM98XX_plat *)cdev->dev->platform_data;
	unsigned long base = plat->base;
	unsigned int baud_divisor;

	/* Setup the serial port with the defaults first */
	baud_divisor = im98xx_calc_divisor(cdev, CONFIG_BAUDRATE);

	/* initializing the device for the first time */
	plat->reg_write(0x00, base, ier);
	plat->reg_write(LCR_BKSE | LCRVAL, base, lcr);
//	plat->reg_write(baud_divisor & 0xFF, base, dll);
//	plat->reg_write((baud_divisor >> 8) & 0xff, base, dlm);
	plat->reg_write(0x1, base, dll);
	plat->reg_write(0x0, base, dlm);
//	plat->reg_write(~LCR_BKSE & LCRVAL, base, lcr);
	plat->reg_write(LCRVAL, base, lcr);
	plat->reg_write(baud_divisor - 1, base, ecr);
	plat->reg_write(MCRVAL, base, mcr);
	plat->reg_write(FCRVAL, base, fcr);
}

/*********** Exposed Functions **********************************/

/**
 * @brief Put a character to the serial port
 *
 * @param[in] cdev pointer to console device
 * @param[in] c character to put
 */
static void im98xx_putc(struct console_device *cdev, char c)
{
	struct IM98XX_plat *plat =
			(struct IM98XX_plat *)cdev->dev->platform_data;
	unsigned long base = plat->base;

	/* Loop Doing Nothing */
	while ((plat->reg_read(base, lsr) & LSR_THRE) == 0);

	plat->reg_write(c, base, thr);
}

/**
 * @brief Retrieve a character from serial port
 *
 * @param[in] cdev pointer to console device
 *
 * @return return the character read
 */
static int im98xx_getc(struct console_device *cdev)
{
	struct IM98XX_plat *plat =
			(struct IM98XX_plat *)cdev->dev->platform_data;
	unsigned long base =plat->base;

	/* Loop Doing Nothing */
	while ((plat->reg_read(base, lsr) & LSR_DR) == 0);

	return plat->reg_read(base, rbr);
}

/**
 * @brief Test if character is available
 *
 * @param[in] cdev pointer to console device
 *
 * @return  - status based on data availability
 */
static int im98xx_tstc(struct console_device *cdev)
{
	struct IM98XX_plat *plat =
			(struct IM98XX_plat *)cdev->dev->platform_data;

	unsigned long base = plat->base;

	return ((plat->reg_read(base, lsr) & LSR_DR) != 0);
}

/**
 * @brief Set the baudrate for the uart port
 *
 * @param[in] cdev  console device
 * @param[in] baud_rate baud rate to set
 *
 * @return  0-implied to support the baudrate
 */
static int im98xx_setbaudrate(struct console_device *cdev, int baud_rate)
{
	struct IM98XX_plat *plat =
			(struct IM98XX_plat *)cdev->dev->platform_data;
	unsigned long base = plat->base;
	unsigned int baud_divisor = im98xx_calc_divisor(cdev, baud_rate);

	plat->reg_write(0x00, base, ier);
	plat->reg_write(LCR_BKSE, base, lcr);
//	plat->reg_write(baud_divisor & 0xFF, base, dll);
//	plat->reg_write((baud_divisor >> 8) & 0xff, base, dlm);
	plat->reg_write(0x1, base, dll);
	plat->reg_write(0x0, base, dlm);
//	plat->reg_write(~LCR_BKSE & LCRVAL, base, lcr);
	plat->reg_write(LCRVAL, base, lcr);
	plat->reg_write(baud_divisor - 1, base, ecr);
	plat->reg_write(MCRVAL, base, mcr);
	plat->reg_write(FCRVAL, base, fcr);

	return 0;
}

/**
 * @brief Probe entry point -called on the first match for device
 *
 * @param[in] dev matched device
 *
 * @return EINVAL if platform_data is not populated,
 *	   ENOMEM if calloc failed
 *	   else return result of console_register
 */
static int im98xx_probe(struct device_d *dev)
{
	struct console_device *cdev;
	struct IM98XX_plat *plat =
			(struct IM98XX_plat *)dev->platform_data;

	writel(0x0, UR_GCLK_REG);
	writel(0x0, UR_RXDMAEN_REG);

	/* we do expect platform specific data */
	if (plat == NULL)
		return -EINVAL;

	if ((plat->reg_read == NULL) || (plat->reg_write == NULL))
		return -EINVAL;

	cdev = calloc(1, sizeof(struct console_device));
	if (cdev == NULL)
		return -ENOMEM;

	dev->type_data = cdev;
	cdev->dev = dev;
	//cdev->f_caps = plat->f_caps;
	cdev->tstc = im98xx_tstc;
	cdev->putc = im98xx_putc;
	cdev->getc = im98xx_getc;
	cdev->setbrg = im98xx_setbaudrate;

	im98xx_serial_init_port(cdev);

	return console_register(cdev);
}

/**
 * @brief Driver registration structure
 */
static struct driver_d im98xx_serial_driver = {
	.name	= "serial_im98xx",
	.probe	= im98xx_probe,
};

/**
 * @brief driver initialization function
 *
 * @return result of register_driver
 */
 /*
static int im98xx_serial_init(void)
{
	return register_driver(&im98xx_serial_driver);
}
*/
//console_initcall(im98xx_serial_init);
console_platform_driver(im98xx_serial_driver);

