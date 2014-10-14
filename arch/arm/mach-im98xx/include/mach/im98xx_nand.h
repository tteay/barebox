#ifndef __ASM_ARCH_NAND_H
#define __ASM_ARCH_NAND_H

#include <linux/mtd/mtd.h>

struct im98xx_nand_platform_data {
	unsigned long base;
	unsigned int timing;
	int width;
};

struct im98xx_nand_block_mark_info {
        char mark_id[4];
        unsigned int offset;
        unsigned int size;
};

extern struct im98xx_nand_block_mark_info boot_region_info[16];

#endif /* __ASM_ARCH_NAND_H */

