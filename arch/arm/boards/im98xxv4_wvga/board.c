/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Raghavendra KH <r-khandenahally@ti.com>
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

/**
 * @file
 * @brief Beagle Specific Board Initialization routines
 */

/**
 * @page ti_beagle Texas Instruments Beagle Board
 *
 * FileName: arch/arm/boards/omap/board-beagle.c
 *
 * Beagle Board from Texas Instruments as described here:
 * http://www.beagleboard.org
 *
 * This board is based on OMAP3530.
 * More on OMAP3530 (including documentation can be found here):
 * http://focus.ti.com/docs/prod/folders/print/omap3530.html
 *
 * This file provides initialization in two stages:
 * @li boot time initialization - do basics required to get SDRAM working.
 * This is run from SRAM - so no case constructs and global vars can be used.
 * @li run time initialization - this is for the rest of the initializations
 * such as flash, uart etc.
 *
 * Boot time initialization includes:
 * @li SDRAM initialization.
 * @li Pin Muxing relevant for Beagle.
 *
 * Run time initialization includes
 * @li serial @ref serial_im98xx.c driver device definition
 *
 * Originally from arch/arm/boards/omap/board-sdp343x.c
 */

#include <common.h>
#include <console.h>
#include <init.h>
#include <driver.h>
#include <partition.h>
#include <mach/im98xx.h>
#include <asm/io.h>
#include <asm/armlinux.h>
#include <generated/mach-types.h>
#include <mach/sys_info.h>
#include <mach/syslib.h>
#include <mach/control.h>
#include <mach/omap3-clock.h>
#include <mach/omap3-silicon.h>
#include <mach/im98xx_nand.h>
#include <mach/magic.h>
#include <mach/mem_map.h>


static struct IM98XX_plat serial_plat = {
	.clock		= 26000000,	/* 26MHz */
	.f_caps		= CONSOLE_STDIN | CONSOLE_STDOUT | CONSOLE_STDERR,
	.reg_read	= im98xx_uart_read,
	.reg_write	= im98xx_uart_write,
	.base		=IM98XX_UART3_BASE,
};

static struct device_d im98xx_serial_device = {
	.name		= "serial_im98xx",
	//.map_base	= IM98XX_UART3_BASE,
	//.size		= 1024,
	.platform_data	= (void *)&serial_plat,
};

/**
 * @brief UART serial port initialization - remember to enable COM clocks in
 * arch
 *
 * @return result of device registration
 */
static int im98xx_console_init(void)
{
	/* Register the serial port */
	return platform_device_register(&im98xx_serial_device);
}
console_initcall(im98xx_console_init);
/*
static struct memory_platform_data sram_pdata = {
	.name	= "ram0",
	.flags	= DEVFS_RDWR,
};
*/
static struct device_d sdram_dev = {
	.id		= -1,
	.name		= "mem",
	//.map_base	= 0x40800000,
	//.size		= 198 * 1024 * 1024,
	//.platform_data	= &sram_pdata,
};
/*
static struct memory_platform_data internal_sram_pdata = {
	.name = "ram1",
	.flags = DEVFS_RDWR,
};
*/

static struct device_d sram_dev = {
	.id		= -1,
	.name		= "mem",
	//.map_base	= 0x1FFE0000,
	//.size		= 128 * 1024,
	//.platform_data	= &internal_sram_pdata,
};
/*
static struct memory_platform_data factory_sram_pdata = {
	.name = "ram2",
	.flags = DEVFS_RDWR,
};
*/

static struct device_d imei_dev = {
	.id		= -1,
	.name		= "mem",
	//.map_base	= 0x407F2000,
	//.size		= 128 * 1024,
	//.platform_data	= &factory_sram_pdata,
};
/*
static struct memory_platform_data chargingIcon_sram_pdata = {
	.name = "ram3",
	.flags = DEVFS_RDWR,
};
*/

static struct device_d icon_dev = {
	.id		= -1,
	.name		= "mem",
	//.map_base	= 0x40000000,
	//.size		= 128 * 1024,
	//.platform_data	= &chargingIcon_sram_pdata,
};

/*RD3, bohung.wu, 20101013, add NOR FLASH support */
static struct device_d cfi_dev = {
	.id		= -1,
	.name		= "cfi_flash",
	//.map_base	= 0x10000000,
	//.size		= 4 * 1024 * 1024,
};

struct im98xx_nand_platform_data nand_info = {
	.timing		= 0x252d4b,
	.width		= 1,
};

static struct device_d nand_dev = {
	.id		= -1,
	.name		= "im98xx_nand",
	.platform_data	= &nand_info,
};

/*
 * SMSC 9118 network controller
 * connected to CS line 1 and interrupt line
 * GPIO3, data width is 16 bit
 */
static struct device_d network_dev = {
	.id		= -1,
	.name		= "smc911x",
	//.map_base	= 0x16000000,
	//.size		= 0x1000000,	/* area size */
};

static int im98xx_mem_init(void)
{//we already init sdram,so no need init it here
	arm_add_mem_device("ram0", 0x40800000, 0xF800000);//sdram 198*1024*1024,
	arm_add_mem_device("ram1", 0x1ffe0000, 128*1024);//sram
	//arm_add_mem_device("ram2", 0x407f2000, 128*1024);//imei
	arm_add_mem_device("ram3", 0x40000000, 128*1024);//icon


}
mem_initcall(im98xx_mem_init);

static int im98xx_devices_init(void)
{
	int ret;
	int i;
	int env_block_is_found = 0;
	struct im98xx_nand_block_mark_info *self_block_info = NULL;
	struct im98xx_nand_block_mark_info *env_block_info = NULL;
	struct im98xx_nand_block_mark_info *ramloader_block_info = NULL;
	struct im98xx_nand_block_mark_info *factory_block_info = NULL;
	struct im98xx_nand_block_mark_info *icon_block_info = NULL;
	char null_block[4] = {0xff, 0xff, 0xff, 0xff};

	platform_device_register(&cfi_dev);
	platform_device_register(&nand_dev);
	platform_device_register(&network_dev);

/*already added when im98xx_mem_init is called
		add_generic_device("mem", DEVICE_ID_DYNAMIC, "ram0", 0x43800000,
						198*1024*1024, IORESOURCE_MEM, NULL);
		add_generic_device("mem", DEVICE_ID_DYNAMIC, "ram1", 0x1ffe0000,
					128*1024, IORESOURCE_MEM, NULL);
		add_generic_device("mem", DEVICE_ID_DYNAMIC, "ram2", 0x407f2000,
						128*1024, IORESOURCE_MEM, NULL);
		add_generic_device("mem", DEVICE_ID_DYNAMIC, "ram3", 0x40000000,
						128*1024, IORESOURCE_MEM, NULL);
*/


#ifdef BOOT_DBG
	printf("Block:\tOffset:\tSize:\tMark:\n");
	for (i = 0; i < 16; i++) {
		printf("%02d:\t%x\t%x\t\"%s\"\n", i,
			boot_region_info[i].offset,
			boot_region_info[i].size,
			boot_region_info[i].mark_id);
	}
#endif

	for (i = 0; i < 16; i++) {
		if (strncmp(boot_region_info[i].mark_id, "im00", 4) == 0) {
			self_block_info = &boot_region_info[i];
			printf("SELF Block is found!\n");
			devfs_add_partition("nand0", self_block_info->offset,
					self_block_info->size,
					DEVFS_PARTITION_FIXED, "self_raw");
			dev_add_bb_dev("self_raw", "self0");
		}

		if (strncmp(boot_region_info[i].mark_id, "im01", 4) == 0) {
			env_block_info = &boot_region_info[i];
			env_block_is_found = 1;
			printf("ENV Block is found!\n");
			devfs_add_partition("nand_im98xx_0", env_block_info->offset,
					env_block_info->size,
					DEVFS_PARTITION_FIXED, "env_raw");
			dev_add_bb_dev("env_raw", "env0");
		}

		if (strncmp(boot_region_info[i].mark_id, "im02", 4) == 0) {
			ramloader_block_info = &boot_region_info[i];
			printf("RAMLOADER Block is found!\n");
			devfs_add_partition("nand0", ramloader_block_info->offset,
					ramloader_block_info->size,
					DEVFS_PARTITION_FIXED, "ramloader_raw");
			dev_add_bb_dev("ramloader_raw", "ramloader0");
		}

		if (strncmp(boot_region_info[i].mark_id, "im03", 4) == 0) {
			factory_block_info = &boot_region_info[i];
			printf("IMEI/SN Block is found!\n");
			devfs_add_partition("nand0", factory_block_info->offset,
					factory_block_info->size,
					DEVFS_PARTITION_FIXED, "factory_raw");
			dev_add_bb_dev("factory_raw", "factory0");
		}

		if (strncmp(boot_region_info[i].mark_id, "im04", 4) == 0) {
			icon_block_info = &boot_region_info[i];
			printf("ICON Block is found!\n");
			devfs_add_partition("nand0", icon_block_info->offset,
					icon_block_info->size,
					DEVFS_PARTITION_FIXED, "icon_raw");
			dev_add_bb_dev("icon_raw", "icon0");
		}
	}

	if (env_block_is_found == 0) {
		BAREBOX_P("Find an empty block!\n");
		for (i = 0; i < 16; i++) {
			if ((strncmp(null_block, boot_region_info[i].mark_id, 4) == 0) &&
			    (boot_region_info[i].offset != 0)) {
				env_block_info = &boot_region_info[i];
				devfs_add_partition("nand0", env_block_info->offset,//nand_im98xx_0
						env_block_info->size,
						DEVFS_PARTITION_FIXED, "env_raw");
				dev_add_bb_dev("env_raw", "env0");
				break;
			}
		}
	}

#ifdef BOOT_DBG
	printf("env_block:\t%x\t%x\t\"%s\"\n", self_block_info->offset,
		self_block_info->size, self_block_info->mark_id);

	printf("env_block:\t%x\t%x\t\"%s\"\n", env_block_info->offset,
		env_block_info->size, env_block_info->mark_id);

	printf("ramloader_block:\t%x\t%x\t\"%s\"\n", ramloader_block_info->offset,
		ramloader_block_info->size, ramloader_block_info->mark_id);

	printf("factory_block:\t%x\t%x\t\"%s\"\n", factory_block_info->offset,
		factory_block_info->size, factory_block_info->mark_id);

	printf("icon_block:\t%x\t%x\t\"%s\"\n", icon_block_info->offset,
		icon_block_info->size, icon_block_info->mark_id);
#endif

	arm_add_mem_device("ram1", 0x1ffe0000, 128*1024);//sram
/*	arm_add_mem_device("ram2", 0x407f2000, 128*1024);//imei
	arm_add_mem_device("ram3", 0x40000000, 128*1024);//icon
	arm_add_mem_device("ram0", 0x43800000, 192*1024*1024);//sdram
*/
	armlinux_set_bootparams((void *)(0x40800000 + 0x100));//sdram base:0x40800000
	armlinux_set_architecture(MACH_TYPE_IM98XXV4_WVGA);
failed:
	return ret;
}
device_initcall(im98xx_devices_init);
