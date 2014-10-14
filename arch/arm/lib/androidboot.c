#include <common.h>
#include <command.h>
#include <fs.h>
#include <of.h>
#include <fcntl.h>
#include <errno.h>
#include <malloc.h>
#include <sizes.h>
#include <asm/byteorder.h>
#include <asm/armlinux.h>
#include <asm/system.h>
#include <asm-generic/memory_layout.h>
#include <memory.h>

#include <asm/io.h>
#if defined (CONFIG_ARCH_IM98XX)
#include <mach/magic.h>
#endif

#ifdef CONFIG_CMD_ANDROIDBOOT

typedef struct boot_img_hdr boot_img_hdr;

#define BOOT_MAGIC "ANDROID!"
#define BOOT_MAGIC_SIZE 8
#define BOOT_NAME_SIZE 16
#define BOOT_ARGS_SIZE 512
#define FLASH_PAGE_SIZE 2048

struct boot_img_hdr
{
    unsigned char magic[BOOT_MAGIC_SIZE];

    unsigned kernel_size;  /* size in bytes */
    unsigned kernel_addr;  /* physical load addr */

    unsigned ramdisk_size; /* size in bytes */
    unsigned ramdisk_addr; /* physical load addr */

    unsigned second_size;  /* size in bytes */
    unsigned second_addr;  /* physical load addr */

    unsigned tags_addr;    /* physical addr for kernel tags */
    unsigned page_size;    /* flash page size we assume */
    unsigned unused[2];    /* future expansion: should be 0 */

    unsigned char name[BOOT_NAME_SIZE]; /* asciiz product name */

    unsigned char cmdline[BOOT_ARGS_SIZE];

    unsigned id[8]; /* timestamp / checksum / sha1 / etc */
};

/*
** +-----------------+
** | boot header     | 1 page
** +-----------------+
** | kernel          | n pages
** +-----------------+
** | ramdisk         | m pages
** +-----------------+
** | second stage    | o pages
** +-----------------+
**
** n = (kernel_size + page_size - 1) / page_size
** m = (ramdisk_size + page_size - 1) / page_size
** o = (second_size + page_size - 1) / page_size
**
** 0. all entities are page_size aligned in flash
** 1. kernel and ramdisk are required (size != 0)
** 2. second is optional (second_size == 0 -> no second)
** 3. load each element (kernel, ramdisk, second) at
**    the specified physical address (kernel_addr, etc)
** 4. prepare tags at tag_addr.  kernel_args[] is
**    appended to the kernel commandline in the tags.
** 5. r0 = 0, r1 = MACHINE_TYPE, r2 = tags_addr
** 6. if second_size != 0: jump to second_addr
**    else: jump to kernel_addr
*/
#if 0
static int do_bootimg(int argc, char *argv[])
{
	//void (*theKernel)(int zero, int arch, void *params);
	int fd, ret, swap = 0;
	struct boot_img_hdr __header, *header;
    int kernel_page_size,ramdisk_page_size,second_stage_size;

	void *bootimg =NULL;
	void *oftree = NULL;

	u32 end;
	int usemap = 0;
	struct memory_bank *bank = list_first_entry(&memory_banks, struct memory_bank, list);
	struct resource *res = NULL;
	
#if defined (CONFIG_ARCH_IM98XX)
	/* turn-off LED backlight */
	writel(readl(ABB_PMU_LDO_REG) & ~(1 << 14), ABB_PMU_LDO_REG);
	writel(0xff, ABB_PWL_REG);
#endif
	printf("do_bootimg argc:%d\n",argc);
	if (argc != 2)
		return COMMAND_ERROR_USAGE;

	fd = open(argv[1], O_RDONLY);

	if (fd < 0) {
		perror("open");
		return 1;
	}

	/*
	 * We can save the memcpy of the zImage if it already is in
	 * the first 128MB of SDRAM.
	 */
	bootimg = memmap(fd, PROT_READ);
	if (bootimg && (unsigned long)bootimg  >= bank->start &&
			(unsigned long)bootimg < bank->start + SZ_128M) {
		usemap = 1;
		header = bootimg;
	}

	if (!usemap) {
		header = &__header;
		ret = read(fd, header, sizeof(*header));
		if (ret < sizeof(*header)) {
			printf("could not read %s\n", argv[1]);
			goto err_out;
		}
	}

        if (memcmp(header->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
		printf("invalid magic %s\n", header->magic);
		goto err_out;
	}
	
	kernel_page_size = header->kernel_size;
	ramdisk_page_size = header->ramdisk_size;
	second_stage_size = header->second_size;
	
	if(swap){
		kernel_page_size = swab32(kernel_page_size);
		ramdisk_page_size = swab32(ramdisk_page_size);
		second_stage_size = swab32(second_stage_size);
	}

    kernel_page_size = (kernel_page_size + (FLASH_PAGE_SIZE - 1)) &
        (~(FLASH_PAGE_SIZE - 1));
    ramdisk_page_size = (ramdisk_page_size + (FLASH_PAGE_SIZE - 1)) &
            (~(FLASH_PAGE_SIZE - 1));
	second_stage_size = (second_stage_size + (FLASH_PAGE_SIZE - 1)) &
				(~(FLASH_PAGE_SIZE - 1));

	end = (kernel_page_size+ramdisk_page_size+second_stage_size);

	if (!usemap) {
		if (bank->size <= SZ_256M) {//SZ_128M //here just for set copy image to malloc area
			bootimg = xmalloc(end);
			printf("bank->size:0x%x,end:0x%x bootimg:%p\n",bank->size, end,bootimg);
		} else {
			bootimg = (void *)bank->start + SZ_8M;
			printf("bootimg:%p, bank->start:0x%x ,end:0x%x\n",bootimg,bank->start, end);
			res = request_sdram_region("bootimg",
					bank->start + SZ_8M, end);
			if (!res) {
				printf("can't request region for kernel\n");
				goto err_out1;
			}
		}
	}


	ret = read(fd, (void *)header->kernel_addr, kernel_page_size);  // don't need to read again, because we've read it to bootimg_buf


	if (ret < kernel_page_size) {
		printf("could not read %s\n", argv[1]);
		goto err_out2;
	} else
		memcpy((void *)header->kernel_addr, bootimg + FLASH_PAGE_SIZE, kernel_page_size);



	ret = read(fd, (void *)header->ramdisk_addr, ramdisk_page_size); // don't need to read again, because we've read it to bootimg_buf

    if (ret < ramdisk_page_size) {
		printf("could not read %s\n", argv[1]);
		goto err_out2;
	} else
		memcpy((void *)header->ramdisk_addr, bootimg + FLASH_PAGE_SIZE + kernel_page_size, ramdisk_page_size);

        //theKernel = (void *)header.kernel_addr;

	printf("struct boot_img_hdr\n");
	printf("	unsigned char magic[BOOT_MAGIC_SIZE]:%s\n",header->magic);
	printf("	unsigned kernel_size; 0x%x\n",header->kernel_size);
	printf("	unsigned kernel_addr; 0x%x\n",header->kernel_addr);
	printf("	\n");
	printf("	unsigned ramdisk_size; 0x%x\n",header->ramdisk_size);
	printf("	unsigned ramdisk_addr; 0x%x\n",header->ramdisk_addr);
	printf("	\n");
	printf("	unsigned second_size; 0x%x\n",header->second_size);
	printf("	unsigned second_addr; 0x%x\n",header->second_addr);
	printf("	\n");
	printf("	unsigned tags_addr 0x%x\n",header->tags_addr);
	printf("	unsigned page_size 0x%x\n",header->page_size);
	printf("	unsigned unused[2] 0x%x 0x%x\n",header->unused[0],header->unused[1]);
	printf("	\n");
	printf("	unsigned char name[BOOT_NAME_SIZE]; %s\n",header->name);
	printf("	unsigned char cmdline[BOOT_ARGS_SIZE]; %s\n",header->cmdline);
	printf("	unsigned id[8];\n");
	printf("	\n");


	printf("loaded Android Boot Image from %s\n", argv[1]);
#ifdef CONFIG_OFTREE
	oftree = of_get_fixed_tree(NULL);
#endif
/*
        setup_tags_with_initrd(header->ramdisk_addr, header->ramdisk_addr +
                        header->ramdisk_size);
*/
	start_linux((void*)header->kernel_addr, swap, header->ramdisk_addr, header->ramdisk_size, oftree);
	//theKernel(0, armlinux_architecture, armlinux_bootparams);

	return 0;

err_out2:
	if (res)
		release_sdram_region(res);
err_out1:
	free(bootimg);
err_out:
	close(fd);

	return 1;
}
#else
static int do_bootimg(int argc, char *argv[])
{
	void (*theKernel)(int zero, int arch, void *params);
	int fd, ret;
	struct boot_img_hdr header;
        int kernel_page_size, ramdisk_page_size;
	char *bootimg_buf;

#if defined (CONFIG_ARCH_IM98XX)
	/* turn-off LED backlight */
	writel(readl(ABB_PMU_LDO_REG) & ~(1 << 14), ABB_PMU_LDO_REG);
	writel(0xff, ABB_PWL_REG);
#endif

	if (argc != 2) {
		return 1;
	}

	fd = open(argv[1], O_RDONLY);

	if (fd < 0) {
		perror("open");
		return 1;
	}

	// copy bootimg from nand flash to buffer instead of directly to ram.

	// read whole boot image to buffer
	bootimg_buf = malloc(0x400000);
	if (!bootimg_buf) {
		printf("%s: Out of memory\n", __FUNCTION__);
		return -1;
	}
	ret = read(fd, bootimg_buf, 0x400000);
//	ret = read(fd, &header, FLASH_PAGE_SIZE); // don't need to read again, because we've read it to bootimg_buf

	if (ret < FLASH_PAGE_SIZE) {
		printf("could not read %s\n", argv[1]);
		goto err_out;
	}

	// copy from bootimg_buffer
	memcpy(&header, bootimg_buf, FLASH_PAGE_SIZE);

        if (memcmp(header.magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
		printf("invalid magic %s\n", header.magic);
		goto err_out;
	}

        kernel_page_size = (header.kernel_size + (FLASH_PAGE_SIZE - 1)) &
                (~(FLASH_PAGE_SIZE - 1));

//	ret = read(fd, (void *)header.kernel_addr, kernel_page_size);  // don't need to read again, because we've read it to bootimg_buf

        if (ret < kernel_page_size + FLASH_PAGE_SIZE) {
//	if (ret < kernel_page_size) {
		printf("could not read %s\n", argv[1]);
		//goto err_out1;
	} else
		memcpy((void *)header.kernel_addr, bootimg_buf + FLASH_PAGE_SIZE, kernel_page_size);

        ramdisk_page_size = (header.ramdisk_size + (FLASH_PAGE_SIZE - 1)) &
                (~(FLASH_PAGE_SIZE - 1));

//	ret = read(fd, (void *)header.ramdisk_addr, ramdisk_page_size); // don't need to read again, because we've read it to bootimg_buf

        if (ret < ramdisk_page_size) {
		printf("could not read %s\n", argv[1]);
		//goto err_out1;
	} else
		memcpy((void *)header.ramdisk_addr, bootimg_buf + FLASH_PAGE_SIZE + kernel_page_size, ramdisk_page_size);

        theKernel = (void *)header.kernel_addr;

	printf("loaded Android Boot Image from %s\n", argv[1]);


	start_linux((void*)header.kernel_addr, 0, header.ramdisk_addr, header.ramdisk_size, 0);

	return 0;

//err_out1:
err_out:
	if (bootimg_buf)
		free(bootimg_buf);	// free the buffer

	close(fd);

	return 1;
}

#endif

static const __maybe_unused char cmd_bootimg_help[] =
"Usage: bootimg [FILE]\n"
"Boot an Android Booting Image\n";

BAREBOX_CMD_START(bootimg)
	.cmd            = do_bootimg,
	BAREBOX_CMD_DESC("Boot an Android Booting Image")
	BAREBOX_CMD_OPTS("FILE")
	BAREBOX_CMD_GROUP(CMD_GRP_BOOT)
	BAREBOX_CMD_HELP(cmd_bootimg_help)
BAREBOX_CMD_END

#endif /* CONFIG_CMD_BOOTIMG */
