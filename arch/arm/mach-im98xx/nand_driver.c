/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <common.h>
#include <driver.h>
#include <malloc.h>
#include <init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <mach/magic.h>
#include <asm/io.h>
#include <errno.h>

#include <command.h>

struct block_mark_info {
	char mark_id[4];
	unsigned int offset;
	unsigned int size;
};

/**
 * Enable the NAND flash controller
 */
static void enable_nand_controller(void)
{
	unsigned int i;

	*SYS_PINMUX_REG 	&= 0xfeffffff;		// P114,
	*HIF_CS_POLAR_REG	= 0x0;			// P239, Set all CSs to Active Low
	i = (*HIF_DMA1_STA_REG & 0xff88ffb3);		// P242, Set DMA1 to 16 bit I/F, NAND Flash I/F, Set DMA channel to CS2
	*HIF_DMA1_STA_REG	= (i | 0x00230043);	// 16bit : 8bit
	*HIF_NAND_REG		= (0x13fffff);	// P256, Timing setting, NAND IO Width = 16 bit
	*HIF_NAND_TAGSL_REG	= 2;		// P257, CS2 is be selected
}

/**
 * This function issues the specified command to the NAND device and
 * waits for completion.
 *
 * @param       cmd     command for NAND Flash
 */
static void send_cmd(u16 cmd)
{
//	printf("send_cmd(host, 0x%x)\n", cmd);

	writew(cmd, HIF_NAND_CMDT_REG);

	/* Wait for operation to complete */
	while ((*HIF_NAND_FIFOSTA_REG) & 0x1f);
}

/**
 * This function sends an address (or partial address) to the
 * NAND device.  The address is used to select the source/destination for
 * a NAND command.
 *
 * @param       addr    address to be written to NFC.
 */
static void noinline send_addr(u16 addr)
{
//	printf("send_addr(host, 0x%x)\n", addr);

	writew(addr, HIF_NAND_WTADRT_REG);

	/* Wait for operation to complete */
	while ((*HIF_NAND_FIFOSTA_REG) & 0x1f);
}

/**
 * This function controls the mode of hardware ecc unit
 *
 * @param       addr    address to be written to NFC.
 */
static void im98xx_nand_enable_hwecc(int mode)
{
	if (mode == NAND_ECC_READ)
		*HIF_ECC_REG = 0x03;

	if (mode == NAND_ECC_WRITE)
		*HIF_ECC_REG = 0x01;

	if (mode == NAND_ECC_READSYN)
		*HIF_ECC_REG = 0x03;
}

static void im98xx_nand_hwcontrol(int cmd, unsigned int ctrl)
{

	if (cmd == NAND_CMD_NONE)
		return;

	/*
	* If the CLE should be active, this call is a NAND command
	*/
	if (ctrl & NAND_CLE)
		send_cmd(cmd);

	/*
	* If the ALE should be active, this call is a NAND address
	*/
	if (ctrl & NAND_ALE)
		send_addr(cmd);
}

static int im98xx_nand_correct_data(u_char * dat, u_char * read_ecc, u_char * calc_ecc)
{
	int err_num;
	volatile unsigned int *nand_err_ptr;
	int err_addr = 0;
	int err_val = 0;

	while ((*HIF_ECC_REG & 0x0001) == 0x0001);

	err_num = *HIF_ERR_REG;

	if (err_num <= 4) {
		nand_err_ptr = HIF_ERR1_ADR_REG;
		for (;err_num;err_num--) {
			err_addr = *nand_err_ptr++;
			err_val = *nand_err_ptr++;
			if (err_addr < 512)
				dat[err_addr] ^= err_val;
			else
				read_ecc[err_addr - 512] ^= err_val;
		}
		return 0;
	}

	return -1;
}

/**
 * This function reads byte from the NAND Flash
 *
 * @return    data read from the NAND Flash
 */
static u_char im98xx_nand_read_byte(void)
{
	u_char ret = 0xFF;

	*HIF_NAND_RDDAT_REG = 0x0;
	while(!( *HIF_NAND_FIFOSTA_REG & 0x00001F00 ));
	ret = (uint8_t) *HIF_NAND_RDFIFO_REG;

	return ret;
}

/**
 * This function is used to read the data buffer from the NAND Flash. To
 * read the data from NAND Flash first the data output cycle is initiated by
 * the NFC, which copies the data to RAMbuffer. This data of length \b len is
 * then copied to buffer \b buf.
 *
 * @param       buf     data to be read from NAND Flash
 * @param       len     number of bytes to be read
 */
static void im98xx_nand_read_buf(u_char * buf, int len)
{
	u16 *p = (u16 *) buf;
	len >>= 1;

	while(len--) {
		*HIF_NAND_RDDAT_REG = 0x0;
		while(!( *HIF_NAND_FIFOSTA_REG & 0x00001F00 ));
		*p++ = *HIF_NAND_RDFIFO_REG;
	}
}

/*
 * This function is used by the upper layer to write command to NAND Flash for
 * different operations to be carried out on NAND Flash
 *
 * @param       command         command for NAND Flash
 * @param       column          column offset for the page read
 * @param       page_addr       page to be read from NAND Flash
 */
static void im98xx_nand_command(unsigned command, int column, int page_addr)
{
	int chip_delay = 50;

//	printf("im98xx_nand_command (cmd = 0x%x, col = 0x%x, page = 0x%x)\n",
//			command, column, page_addr);

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += 2048;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	im98xx_nand_hwcontrol(command & 0xff,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			column >>= 1;
			im98xx_nand_hwcontrol(column & 0xff, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			im98xx_nand_hwcontrol((column >> 8) & 0xff, ctrl);
		}
		if (page_addr != -1) {
			im98xx_nand_hwcontrol(page_addr & 0xff, ctrl);
			im98xx_nand_hwcontrol((page_addr >> 8) & 0xff,
					NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			im98xx_nand_hwcontrol((page_addr >> 16) & 0xff,
					NAND_NCE | NAND_ALE);
		}
	}
	im98xx_nand_hwcontrol(NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status, sequential in, and deplete1 need no delay
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		return;

		/*
		 * read error status commands require only a short delay
		 */
	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(chip_delay);
		return;

	case NAND_CMD_RESET:
		udelay(chip_delay);
		im98xx_nand_hwcontrol(NAND_CMD_STATUS,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		im98xx_nand_hwcontrol(NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		while (!(im98xx_nand_read_byte() & NAND_STATUS_READY)) ;
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		im98xx_nand_hwcontrol(NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		im98xx_nand_hwcontrol(NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		im98xx_nand_hwcontrol(NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		im98xx_nand_hwcontrol(NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);

		udelay(chip_delay);
		im98xx_nand_hwcontrol(NAND_CMD_STATUS,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		im98xx_nand_hwcontrol(NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		while (!(im98xx_nand_read_byte() & NAND_STATUS_READY)) ;

		im98xx_nand_hwcontrol(NAND_CMD_READ0,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		im98xx_nand_hwcontrol(NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;
		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		udelay(chip_delay);
		return;
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */

}

/**
 * im98xx_nand_read_page_syndrome - [REPLACABLE] hardware ecc syndrom based page read
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 *
 * The hw generator calculates the error syndrome automatically. Therefor
 * we need a special oob layout and handling.
 */
static int im98xx_nand_ecc_read_page(int page, uint8_t *buf)
{
	int eccsize = 512;
	int eccbytes = 16;
	int eccsteps = 4;
	uint8_t *p = buf;
	uint8_t oob[64];
	int pos = 0, oob_pos = 2048;

	for (; eccsteps; eccsteps--, p += eccsize, pos += eccsize,
			oob_pos += eccbytes) {
		int stat;

		im98xx_nand_command(NAND_CMD_READ0, pos, page);

		im98xx_nand_enable_hwecc(NAND_ECC_READ);
		im98xx_nand_read_buf(p, eccsize);

		im98xx_nand_command(NAND_CMD_RNDOUT, oob_pos, -1);

		im98xx_nand_read_buf(oob, eccbytes);
		stat = im98xx_nand_correct_data(p, oob, NULL);

		if (stat < 0)
			printk(KERN_INFO, "Error Correction Failed!\n");
	}
	return 0;
}

/**
 * nand_read_oob_std - [REPLACABLE] the most common OOB data read function
 * @page:	page number to read
 * @oob_buf:	buffer to store read oob data
 */
static int im98xx_nand_read_oob(int page, uint8_t *oob_buf)
{
	int oobsize = 64;

	im98xx_nand_command(NAND_CMD_READOOB, 0, page);
	im98xx_nand_read_buf(oob_buf, oobsize);

	return 0;
}

/**
 * im98xx_nand_read_page_syndrome - [REPLACABLE] hardware ecc syndrom based page read
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 *
 * The hw generator calculates the error syndrome automatically. Therefor
 * we need a special oob layout and handling.
 */
static int im98xx_nand_ecc_read_page_raw(int page, uint8_t *buf)
{
	int eccsize = 512;
	int eccbytes = 16;
	int eccsteps = 4;
	uint8_t *p = buf;
	uint8_t *oob = buf + 2048;
	int pos = 0, oob_pos = 2048;

	for (; eccsteps; eccsteps--, p += eccsize, oob += eccbytes, pos += eccsize,
			oob_pos += eccbytes) {
		int stat;

		im98xx_nand_command(NAND_CMD_READ0, pos, page);

		im98xx_nand_enable_hwecc(NAND_ECC_READ);
		im98xx_nand_read_buf(p, eccsize);

		im98xx_nand_command(NAND_CMD_RNDOUT, oob_pos, -1);

		im98xx_nand_read_buf(oob, eccbytes);
		stat = im98xx_nand_correct_data(p, oob, NULL);

		if (stat < 0)
			printk(KERN_INFO, "Error Correction Failed!\n");
	}
	return 0;
}
/**
 * Load a sequential count of blocks from the NAND into memory
 * @param[out] dest Pointer to target area (in SDRAM)
 * @param[in] size Bytes to read from NAND device
 * @param[in] page Start page to read from
 * @param[in] pagesize Size of each page in the NAND
 *
 * This function must be located in the first 4kiB of the barebox image
 * (guess why). When this routine is running the SDRAM is up and running
 * and it runs from the correct address (physical=linked address).
 * TODO Could we access the platform data from the boardfile?
 * Due to it makes no sense this function does not return in case of failure.
 */
void im98xx_nand_load_image(void *dest, int offset, int size)
{
	int page, pagesize = 2048;
	uint8_t *p = (uint8_t *)dest;

	enable_nand_controller();

	/* Reset the NAND device */
	im98xx_nand_command(NAND_CMD_RESET, -1, -1);

	page = offset / pagesize;

	do {
		im98xx_nand_ecc_read_page(page, p);

		page++;
		p += pagesize;
		size -= pagesize;
	} while (size >= 0);
}

void im98xx_nand_dump(int off)
{
	int i, page;
	u_char *buf, *p;

	buf = malloc(2048 + 64);
	if (!buf) {
		puts("No memory for page buffer\n");
		return;
	}
	off &= ~(2048 - 1);
	page = off / 2048;
	i = im98xx_nand_ecc_read_page_raw(page, buf);
	if (i < 0) {
		printf("Error (%d) reading page %08x\n", i, off);
		free(buf);
		return;
	}
	printf("Page %08x dump:\n", off);
	i = 2048 >> 4; p = buf;
	while (i--) {
		printf( "\t%02x %02x %02x %02x %02x %02x %02x %02x"
			"  %02x %02x %02x %02x %02x %02x %02x %02x\n",
			p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
			p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
		p += 16;
	}
	puts("OOB:\n");
	i = 64 >> 3;
	while (i--) {
		printf( "\t%02x %02x %02x %02x %02x %02x %02x %02x\n",
			p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
		p += 8;
	}
	free(buf);
}

void im98xx_nand_dump_oob(int off)
{
	int i, page;
	u_char *buf, *p;

	buf = malloc(64);
	if (!buf) {
		puts("No memory for page buffer\n");
		return;
	}
	off &= ~(2048 - 1);
	page = off / 2048;
	i = im98xx_nand_read_oob(page, buf);
	if (i < 0) {
		printf("Error (%d) reading page %08x\n", i, off);
		free(buf);
		return;
	}
	printf("Page %08x OOB dump:\n", off);
	i = 64 >> 3; p = buf;
	while (i--) {
		printf( "\t%02x %02x %02x %02x %02x %02x %02x %02x\n",
			p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
		p += 8;
	}
	free(buf);
}

static void boot_bootloader(unsigned addr)
{
	void (*entry)(void) = (void*) addr;

	entry();
}

/**
 * nand_read_oob_std - [REPLACABLE] the most common OOB data read function
 * @page:	page number to read
 * @mark_buf:	buffer to store read oob data
 */
static int im98xx_nand_read_block_mark(int page, uint8_t *oob_buf)
{
	int marksize = 6;

	page &= ~(64 - 1);

	im98xx_nand_command(NAND_CMD_READOOB, 0, page);
	im98xx_nand_read_buf(oob_buf, marksize);

	return 0;
}

#define CONFIG_NAND_IM98XX_BOOT_DEBUG
#ifdef CONFIG_NAND_IM98XX_BOOT_DEBUG
static int do_nand_dump_oob(int argc, char *argv[])
{
	int offset;

	enable_nand_controller();

	if (argc < 1)
		return COMMAND_ERROR_USAGE;

	offset = strtoul_suffix(argv[1], NULL, 0);

	im98xx_nand_dump_oob(offset);

	return 0;
}

static const __maybe_unused char cmd_nand_dump_oob_help[] =
"Usage: nand_dump_oob <offset>\n";

BAREBOX_CMD_START(nand_dump_oob)
	.cmd		= do_nand_dump_oob,
	.usage		= "dump oob from NAND",
	BAREBOX_CMD_HELP(cmd_nand_dump_oob_help)
BAREBOX_CMD_END

static int do_nand_dump(int argc, char *argv[])
{
	int offset;

	enable_nand_controller();

	if (argc < 1)
		return COMMAND_ERROR_USAGE;

	offset = strtoul_suffix(argv[1], NULL, 0);

	im98xx_nand_dump(offset);

	return 0;
}

static const __maybe_unused char cmd_nand_dump_help[] =
"Usage: nand_dump <offset>\n";

BAREBOX_CMD_START(nand_dump)
	.cmd		= do_nand_dump,
	.usage		= "dump data from NAND",
	BAREBOX_CMD_HELP(cmd_nand_dump_help)
BAREBOX_CMD_END

static int do_nand_boot_test(int argc, char *argv[])
{
	void *dest;
	int offset, size;

	if (argc < 3)
		return COMMAND_ERROR_USAGE;

	dest = (void *)simple_strtoul(argv[1], NULL, 16);
	offset = simple_strtoul(argv[2], NULL, 16);
	size = simple_strtoul(argv[3], NULL, 16);

	im98xx_nand_load_image(dest, offset, size);

	boot_bootloader((unsigned)dest);

	return 0;
}

static const __maybe_unused char cmd_nand_boot_test_help[] =
"Usage: nand_boot_test <dest> <size> <pagesize>\n";

BAREBOX_CMD_START(nand_boot_test)
	.cmd		= do_nand_boot_test,
	.usage		= "load an image from NAND",
	BAREBOX_CMD_HELP(cmd_nand_boot_test_help)
BAREBOX_CMD_END

static int do_nand_scan_boot_region(int argc, char *argv[])
{
	int block = 0;
	int i;
	u_char *buf, *p;
	struct block_mark_info boot_region_info[16];

	buf = malloc(6);
	if (!buf) {
		puts("No memory for page buffer\n");
		return -1;
	}

	for (i = 0; i < 16; i++) {
		im98xx_nand_read_block_mark(block/2048, buf);
		p = buf;
/*
		printf("block: %x\n", block);
		printf( "\t%02x %02x %02x %02x %02x %02x\n",
			p[0], p[1], p[2], p[3], p[4], p[5]);
*/
		strncpy(boot_region_info[i].mark_id, p + 2, 4);
		boot_region_info[i].offset = block;
		boot_region_info[i].size = 0x20000;

		block += (2048 * 64);
	}

	free(buf);

	for (i = 0; i < 16; i++) {
		printf("Block Mark: %s, Offset: %x, Size: %x.\n", boot_region_info[i].mark_id,
				boot_region_info[i].offset, boot_region_info[i].size);
	}

	return 0;
}

static const __maybe_unused char cmd_nand_scan_boot_region_help[] =
"Usage: nand_scan_boot_region <dest> <size> <pagesize>\n";

BAREBOX_CMD_START(nand_scan_boot_region)
	.cmd		= do_nand_scan_boot_region,
	.usage		= "load an image from NAND",
	BAREBOX_CMD_HELP(cmd_nand_scan_boot_region_help)
BAREBOX_CMD_END
#endif

static int do_load_bootloader(int argc, char *argv[])
{
	void *dest;
	int offset, size;

	if (argc < 3)
		return COMMAND_ERROR_USAGE;

	dest = (void *)simple_strtoul(argv[1], NULL, 16);
	offset = simple_strtoul(argv[2], NULL, 16);
	size = simple_strtoul(argv[3], NULL, 16);

	im98xx_nand_load_image(dest, offset, size);

	boot_bootloader((unsigned)dest);

	return 0;
}

static const __maybe_unused char cmd_load_bootloader_help[] =
"Usage: load_bootloader <dest> <offset> <size>\n";

BAREBOX_CMD_START(load_bootloader)
	.cmd		= do_load_bootloader,
	.usage		= "load bootloader from NAND",
	BAREBOX_CMD_HELP(cmd_load_bootloader_help)
BAREBOX_CMD_END
