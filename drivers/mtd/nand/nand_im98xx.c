/*
 * Copyright 2010 Infomax Communication, Inc. All Rights Reserved.
 */

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
#include <mach/im98xx_nand.h>
#include <mach/magic.h>
#include <asm/io.h>
#include <errno.h>

#define DVR_VER "2.0"

#ifdef CONFIG_NAND_IM98XX_BOOT
#define __nand_boot_init __bare_init
#else
#define __nand_boot_init
#endif

struct im98xx_nand_block_mark_info boot_region_info[16];

#define NAND_CMD_READ_REPEAT	0x99
static unsigned long page_addr_r;

struct im98xx_nand_host {
	struct mtd_info		mtd;		
	struct nand_chip	nand;
	struct mtd_partition	*parts;
	struct device_d		*dev;

	void			*spare0;
	void			*main_area0;
	void			*main_area1;

	//void __iomem		*base;
	void __iomem		*regs;
	int			status_request;
	struct clk		*clk;

	int			pagesize_2k;
	uint8_t			*data_buf;
	unsigned int		buf_start;
	int			spare_len;
};

/*
 * OOB placement block for use with hardware ecc generation
 */

/* OOB description for 2048 byte pages with 64 byte OOB */
static struct nand_ecclayout im98xx_hw_eccoob_page = {
	.eccbytes = 40,
	.eccpos = {
		 6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
		22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
		38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		54, 55, 56, 57, 58, 59, 60, 61, 62, 63
	},
	.oobfree = {
		{.offset =  2, .length = 4},
		{.offset = 16, .length = 6},
		{.offset = 32, .length = 6},
		{.offset = 48, .length = 6}
	}
};

static struct nand_ecclayout im98xx_hw_eccoob_page_4K = {
	.eccbytes = 80,
	.eccpos = {
		  6,   7,   8,   9,  10,  11,  12,  13,  14,  15,
		 22,  23,  24,  25,  26,  27,  28,  29,  30,  31,
		 38,  39,  40,  41,  42,  43,  44,  45,  46,  47,
		 54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
		 70,  71,  72,  73,  74,  75,  76,  77,  78,  79,
		 86,  87,  88,  89,  90,  91,  92,  93,  94,  95,
		102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
		118, 119, 120, 121, 122, 123, 124, 125, 126, 127},

	.oobfree = {
		{ .offset = 2,   .length = 4 },
		{ .offset = 16,  .length = 6 },
		{ .offset = 32,  .length = 6 },
		{ .offset = 48,  .length = 6 },
		{ .offset = 64,  .length = 6 },
		{ .offset = 80,  .length = 6 },
		{ .offset = 96,  .length = 6 },
		{ .offset = 112, .length = 6 },
	}
};

void print_data16(u_char *buf, int len) {
	int i = len;
	u_char *p;

	printf("%s len: %d\n", __FUNCTION__, len);

	i >>= 4;
	p = buf;

	while (i--) {
		printf("\t%02x %02x %02x %02x %02x %02x %02x %02x "
		       "%02x %02x %02x %02x %02x %02x %02x %02x\n",
		       p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
		       p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
		p += 16;
	}
}

static int all_ff(const void *buf, int len)
{
	int i;
	const uint8_t *p = buf;

	for (i = 0; i < len; i++)
		if (p[i] != 0xFF)
			return 0;
	return 1;
}

/**
 * Enable the NAND flash controller
 *
 */
static void __nand_boot_init config_nand_controller(void)
{
//	unsigned int i;

	*HIF_CS_POLAR_REG = 0x0;
//	i = (*HIF_DMA1_STA_REG & 0xff88ffb3);
//	*HIF_DMA1_STA_REG = (i | 0x00230043);
	*HIF_DMA1_STA_REG = 0x00031040;
//	*HIF_NAND_REG = 0x1252d4b;
	*HIF_NAND_REG = 0x13fffff;
//	printf("RS_CTL_REG: 0x%08x\n", *RS_CTL_REG);
	*GPIO16_REG = 0x0;
	*GPIO16_REG = 0xa;//0x2;//---lanbo+++.for not wr protect
}

/**
 * This function issues the specified command to the NAND device and
 * waits for completion.
 *
 * @param	cmd	command for NAND Flash
 */
static void __nand_boot_init send_cmd(u16 cmd)
{
	MTD_DEBUG(MTD_DEBUG_LEVEL3, "send_cmd(host, 0x%x)\n", cmd);

	writew(cmd, HIF_NAND_CMDT_REG);

	/* Wait for operation to complete */
	while ((*HIF_NAND_FIFOSTA_REG) & 0x1f);
}

/**
 * This function sends an address (or partial address) to the
 * NAND device.  The address is used to select the source/destination for
 * a NAND command.
 *
 * @param	addr	address to be written to NFC.
 */
static void __nand_boot_init noinline send_addr(u16 addr)
{
	MTD_DEBUG(MTD_DEBUG_LEVEL3, "send_addr(host, 0x%x)\n", addr);

	writew(addr, HIF_NAND_WTADRT_REG);

	/* Wait for operation to complete */
	while ((*HIF_NAND_FIFOSTA_REG) & 0x1f);
}

static void im98xx_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
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

/*
 * This function is used by upper layer to checks if device is ready
 *
 * @param	   mtd	 MTD structure for the NAND Flash
 *
 * @return  0 if device is busy else 1
 */
#if 1
static int im98xx_nand_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	chip->cmd_ctrl(mtd, NAND_CMD_STATUS,
			NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
	chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			NAND_NCE | NAND_CTRL_CHANGE);
	while (!(chip->read_byte(mtd) & NAND_STATUS_READY));

	return 1;
}
#endif

static void im98xx_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	if (mode == NAND_ECC_WRITE)
		*RS_CTL_REG |= 0x1;

	if (mode == NAND_ECC_READ || mode == NAND_ECC_READSYN)
		*RS_CTL_REG |= 0x2;
}

static int im98xx_nand_correct_data(struct mtd_info *mtd, u_char * dat,
				    u_char * read_ecc, u_char * calc_ecc)
{
	int err_num;
	volatile unsigned int *nand_err_ptr;
	int err_addr = 0;
	int err_val = 0;

	if ((*RS_CTL_REG & 0x4) == 0x4) {
		while ((*RS_CTL_REG & 0x40) != 0x40);

		if ((*RS_CTL_REG & 0x80) != 0x80) {
			err_num = *RS_NE_REG;
			nand_err_ptr = RS_EADR1_REG;
			for (;err_num;err_num--) {
				err_addr = *nand_err_ptr++;
				err_val = *nand_err_ptr++;
				if (err_addr < 512)
					dat[err_addr] ^= err_val;
				else
					read_ecc[err_addr - 512] ^= err_val;
			}
			printf("Correct!\n");
			return 0;
		} else {
			if (!all_ff(read_ecc, 16)) {
				printf("Error Correction Failed!\n");
			}
			return -1;
		}
	} else {
		return 0;
	}
}

static int im98xx_nand_calculate_ecc(struct mtd_info *mtd, const u_char * dat,
				     u_char * ecc_code)
{
	uint32_t a[3];
	u_char *p;
	int i;

	a[0] = *RS_PTY03_00;
	a[1] = *RS_PTY07_04;
	a[2] = *RS_PTY09_08;

	p = (u_char *)a;

	for (i = 0; i < 10; i++)
		ecc_code[i] = *p++;

	return 0;
}

/*
 * This function reads byte from the NAND Flash
 *
 * @param	mtd	MTD structure for the NAND Flash
 *
 * @return	data read from the NAND Flash
 */
static u_char im98xx_nand_read_byte(struct mtd_info *mtd)
{
	*HIF_NAND_RDDAT_REG = 0x0;

	while (!(*HIF_NAND_FIFOSTA_REG & 0x00001F00));

	return (uint8_t) *HIF_NAND_RDFIFO_REG;
}

/*
  * This function reads word from the NAND Flash
  *
  * @param	mtd	MTD structure for the NAND Flash
  *
  * @return	data read from the NAND Flash
  */
static u16 im98xx_nand_read_word(struct mtd_info *mtd)
{
	*HIF_NAND_RDDAT_REG = 0x0;

	while (!(*HIF_NAND_FIFOSTA_REG & 0x00001F00));

	return (uint16_t) *HIF_NAND_RDFIFO_REG;
}

/*
 * This function writes data of length \b len to buffer \b buf. The data to be
 * written on NAND Flash is first copied to RAMbuffer. After the Data Input
 * Operation by the NFC, the data is written to NAND Flash
 *
 * @param	mtd	MTD structure for the NAND Flash
 * @param	buf	data to be written to NAND Flash
 * @param	len	number of bytes to be written
 */
static void im98xx_nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	u16 *p = (u16 *) buf;

	len >>= 1;

	while (len--) {
		*HIF_NAND_WTDAT_REG = *p++;
		while ((*HIF_NAND_FIFOSTA_REG) & 0x1F);
	}
}

/*
 * This function is used to read the data buffer from the NAND Flash. To
 * read the data from NAND Flash first the data output cycle is initiated by
 * the NFC, which copies the data to RAMbuffer. This data of length \b len is
 * then copied to buffer \b buf.
 *
 * @param	mtd	MTD structure for the NAND Flash
 * @param	buf	data to be read from NAND Flash
 * @param	len	number of bytes to be read
 */
static void im98xx_nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	u16 *p = (u16 *) buf;
	
	len >>= 1;

	while (len--) {
		*HIF_NAND_RDDAT_REG = 0x0;

		while (!(*HIF_NAND_FIFOSTA_REG & 0x00001F00));

		*p++ = *HIF_NAND_RDFIFO_REG;
	}
}

/*
 * This function is used by the upper layer to verify the data in NAND Flash
 * with the data in the \b buf.
 *
 * @param	mtd	MTD structure for the NAND Flash
 * @param	buf	data to be verified
 * @param	len	length of the data to be verified
 *
 * @return	-EFAULT if error else 0
 *
 */
static int im98xx_nand_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	return -EFAULT;
}

/*
 * This function is used by the upper layer to write command to NAND Flash for
 * different operations to be carried out on NAND Flash
 *
 * @param	mtd		MTD structure for the NAND Flash
 * @param	command		command for NAND Flash
 * @param	column		column offset for the page read
 * @param	page_addr	page to be read from NAND Flash
 */
static void im98xx_nand_command(struct mtd_info *mtd, unsigned command,
				int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;

	MTD_DEBUG(MTD_DEBUG_LEVEL3,
		  "im98xx_nand_command (cmd = 0x%x, col = 0x%x, page = 0x%x)\n",
		  command, column, page_addr);

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	if (command == NAND_CMD_READ_REPEAT) {
		command = NAND_CMD_READ0;
		page_addr = page_addr_r;
	}

	if (command == NAND_CMD_READ0)
		page_addr_r = page_addr;

	/* Command latch cycle */
	chip->cmd_ctrl(mtd, command & 0xff, NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
//			if (chip->options & NAND_BUSWIDTH_16)
				column >>= 1;
			chip->cmd_ctrl(mtd, column & 0xff, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			chip->cmd_ctrl(mtd, (column >> 8) & 0xff, ctrl);
		}
		if (page_addr != -1) {
			chip->cmd_ctrl(mtd, page_addr & 0xff, ctrl);
			chip->cmd_ctrl(mtd, (page_addr >> 8) & 0xff,
					   NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			chip->cmd_ctrl(mtd, (page_addr >> 16) & 0xff,
						   NAND_NCE | NAND_ALE);
		}
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

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
	//case NAND_CMD_DEPLETE1://---lanbo---,this cmd has been removed
		return;

		/*
		 * read error status commands require only a short delay
		 */
	/*case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(chip->chip_delay);
		return;
	*/
	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS,
				   NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
				   NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY)) ;
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		chip->cmd_ctrl(mtd, NAND_CMD_RNDOUTSTART,
				   NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
				   NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		chip->cmd_ctrl(mtd, NAND_CMD_READSTART,
				   NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
				   NAND_NCE | NAND_CTRL_CHANGE);

		udelay(chip->chip_delay);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS,
				   NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
				   NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY)) ;

		chip->cmd_ctrl(mtd, NAND_CMD_READ0,
				   NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
				   NAND_NCE | NAND_CTRL_CHANGE);
		return;

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
//	ndelay(100);

//	nand_wait_ready(mtd);
}

/**
 * im98xx_nand_read_page_ecc_syndrome - hardware ecc syndrom based page read
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 *
 * The hw generator calculates the error syndrome automatically. Therefor
 * we need a special oob layout and handling.
 */
static int im98xx_nand_read_page_ecc_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
					      uint8_t *buf)
{
	int eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *oob = chip->oob_poi;
	uint8_t *oob_p;
	int pos = 0, oob_pos = 2048;
	int sndcmd = 0;

	for (; eccsteps; eccsteps--, p += eccsize, pos += eccsize,
				oob_pos += (chip->ecc.prepad + eccbytes)) {

		if (sndcmd == 1)
			chip->cmdfunc(mtd, NAND_CMD_READ_REPEAT, pos, -1);
		else
			sndcmd = 1;

		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		chip->read_buf(mtd, p, eccsize);

		oob_p = oob;
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, oob_pos, -1);

		chip->read_buf(mtd, oob, chip->ecc.prepad);
		oob += chip->ecc.prepad;

		chip->read_buf(mtd, oob, eccbytes);
		oob += eccbytes;
		chip->ecc.correct(mtd, p, oob_p, NULL);

	}
	return 0;

}

/**
 * im98xx_nand_write_page_ecc_syndrome - [REPLACABLE] hardware ecc syndrom based page write
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 *
 * The hw generator calculates the error syndrome automatically. Therefor
 * we need a special oob layout and handling.
 */
static int im98xx_nand_write_page_ecc_syndrome(struct mtd_info *mtd,
					struct nand_chip *chip, const uint8_t *buf)
{
	int eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	const uint8_t *p = buf;
	uint8_t *oob = chip->oob_poi;
	int pos = 0, oob_pos = 2048;
	int sndcmd = 0;
	uint8_t *oob_tmp;

	for (; eccsteps; eccsteps--, p += eccsize, pos += eccsize,
				oob_pos += (chip->ecc.prepad + eccbytes)) {

		if (sndcmd == 1)
			chip->cmdfunc(mtd, NAND_CMD_RNDIN, pos, -1);
		else
			sndcmd = 1;

		chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
		chip->write_buf(mtd, p, eccsize);

		chip->cmdfunc(mtd, NAND_CMD_RNDIN, oob_pos, -1);

		oob_tmp = oob;

		if (chip->ecc.prepad) {
			chip->write_buf(mtd, oob, chip->ecc.prepad);
			oob += chip->ecc.prepad;
		}

		chip->ecc.calculate(mtd, p, oob);
		chip->write_buf(mtd, oob, eccbytes);
		oob += eccbytes;
	}
	return 0;
}
#if 0
static int im98xx_ecc_read_oob(struct mtd_info *mtd, struct nand_chip *chip, int page, int sndcmd)
{
	register unsigned char *bufpoi = chip->oob_poi;
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->waitfunc(mtd, chip);
	chip->cmd_ctrl(mtd, NAND_CMD_READ0, (NAND_CTRL_CLE | NAND_CTRL_CHANGE));
	chip->read_buf(mtd, bufpoi, mtd->oobsize);

	return 0;
}

static int im98xx_ecc_write_oob(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
#if 0
	register unsigned char *bufpoi = chip->oob_poi;
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);
	chip->write_buf(mtd, bufpoi, mtd->oobsize);
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	return ( (chip->waitfunc(mtd, chip) & NAND_STATUS_FAIL)? -EIO:0 );
#else
	register unsigned long status, sndcmd, steps = chip->ecc.steps;
	register unsigned long pos, oob_pos;
	int len;
	register unsigned char *bufpoi = chip->oob_poi;
	int eccsize = chip->ecc.size;
	uint32_t fill = 0xFFFFFFFF;
	unsigned long chunk = chip->ecc.prepad + chip->ecc.bytes + chip->ecc.postpad;

	for (oob_pos = mtd->writesize, status = sndcmd = pos = 0; steps; steps--, bufpoi += chunk, pos += eccsize, oob_pos += chunk) {
		if (sndcmd)
			chip->cmdfunc(mtd, NAND_CMD_RNDIN, pos, -1);
		else
			sndcmd = 1;
		chip->ecc.hwctl(mtd, NAND_ECC_WRITE);

		len = eccsize;
		while (len > 0) {
			int num = min_t(int, len, 4);
			chip->write_buf(mtd, (uint8_t *)&fill, num);
			len -= num;
		}
		chip->cmdfunc(mtd, NAND_CMD_RNDIN, oob_pos, -1);
		chip->write_buf(mtd, bufpoi, chip->ecc.prepad);
		chip->write_buf(mtd, 0, chip->ecc.bytes);
		while(readl(RS_CTL_REG) & 0x00000001);
	}
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
#endif
}
#endif

static void im98xx_nand_read_block_mark(struct mtd_info *mtd, int page, uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	int marksize = 6;

	page &= ~(64 - 1);

	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->read_buf(mtd, buf, marksize);
}

static int im98xx_scan_boot_region(struct mtd_info *mtd)
{
	int i;
	int block = 0;
	u_char *buf, *p;

	buf = malloc(6);
	if (!buf) {
		printk(KERN_INFO, "No memory for page buffer!\n");
		return -1;
	}

	for (i = 0; i < 16; i++) {
//		im98xx_nand_read_block_mark(mtd, block / 2048, buf);
		im98xx_nand_read_block_mark(mtd, (block >> 11), buf);
		p = buf;
		strncpy(boot_region_info[i].mark_id, p + 2, 4);
		boot_region_info[i].offset = block;
		boot_region_info[i].size = 0x20000;

		block += 131072;	/* 64 * 2048 */
	}

	free(buf);

	return 0;
}

/**
 * nand_block_bad - [DEFAULT] Read bad block marker from the chip
 * @mtd:	MTD device structure
 * @ofs:	offset from device start
 * @getchip:	0, if the chip is already selected
 *
 * Check, if the block is bad.
 */
static int im98xx_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	int page, res = 0;
//	int chipnr;
	struct nand_chip *chip = mtd->priv;
	u16 bad;

	page = (int)(ofs >> chip->page_shift) & chip->pagemask;

	if (chip->options & NAND_BUSWIDTH_16) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, (chip->badblockpos & 0xFE), page);
		chip->waitfunc(mtd, chip);
		chip->cmd_ctrl(mtd, NAND_CMD_READ0, (NAND_CTRL_CLE | NAND_CTRL_CHANGE));
		bad = cpu_to_le16(chip->read_word(mtd));
		if (chip->badblockpos & 0x1)
			bad >>= 8;
		if ((bad & 0xFF) != 0xff)
			res = 1;
	} else {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos, page);
		chip->waitfunc(mtd, chip);
		chip->cmd_ctrl(mtd, NAND_CMD_READ0, (NAND_CTRL_CLE | NAND_CTRL_CHANGE));
		if (chip->read_byte(mtd) != 0xff)
			res = 1;
	}
#if 0
	if (getchip)
		nand_release_device(mtd);
#endif
	return res;
}


/*
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and
 *                remove functions
 *
 * @return  The function always returns 0.
 */

static int __init im98xx_nand_probe(struct device_d *dev)
{
	struct nand_chip *this;
	struct mtd_info *mtd;
	struct im98xx_nand_host *host;
	int err = 0;

	printk(KERN_INFO, "im98xx nand probe!\n");

	config_nand_controller();

	/* Allocate memory for MTD device structure and private data */
	host = kzalloc(sizeof(struct im98xx_nand_host) + NAND_MAX_PAGESIZE +
			NAND_MAX_OOBSIZE, GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	host->data_buf = (uint8_t *)(host + 1);
	//host->base = (void __iomem *)dev->map_base;

	//host->main_area0 = host->base;
	//host->main_area1 = host->base + 0x200;

	host->dev = dev;
	/* structures must be linked */
	this = &host->nand;
	mtd = &host->mtd;
	mtd->priv = this;

	/* 50 us command delay time */
	this->chip_delay = 5;

	this->priv = host;
	this->dev_ready = im98xx_nand_dev_ready;
	this->cmdfunc = im98xx_nand_command;
	this->cmd_ctrl = im98xx_nand_hwcontrol;
	this->read_byte = im98xx_nand_read_byte;
	this->read_word = im98xx_nand_read_word;
	this->write_buf = im98xx_nand_write_buf;
	this->read_buf = im98xx_nand_read_buf;
	//this->verify_buf = im98xx_nand_verify_buf;

	this->options |= NAND_BUSWIDTH_16;

	this->ecc.read_page = im98xx_nand_read_page_ecc_syndrome;
	this->ecc.write_page = im98xx_nand_write_page_ecc_syndrome;
	this->ecc.calculate = im98xx_nand_calculate_ecc;
	this->ecc.hwctl = im98xx_nand_enable_hwecc;
	this->ecc.correct = im98xx_nand_correct_data;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = 512;
	this->ecc.prepad = 6;
	this->ecc.bytes = 10;
	this->ecc.layout = &im98xx_hw_eccoob_page;
	this->block_bad	= im98xx_nand_block_bad;


	/* Reset NAND */
	this->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* first scan to find the device and get the page size */
	if (nand_scan(mtd, 1)) {
		err = -ENXIO;
		goto escan;
	}

	if (mtd->writesize == 4096)
		this->ecc.layout = &im98xx_hw_eccoob_page_4K;

	/* second phase scan */
	if (nand_scan_tail(mtd)) {
		err = -ENXIO;
		goto escan;
	}

	//add_mtd_device(mtd);
	add_mtd_nand_device(mtd, "nand");

	im98xx_scan_boot_region(mtd);

	dev->priv = host;

	return 0;

escan:
	kfree(host);

	return err;
}

static struct driver_d im98xx_nand_driver = {
	.name	= "im98xx_nand",
	.probe	= im98xx_nand_probe,
};

device_platform_driver(im98xx_nand_driver);


MODULE_AUTHOR("Infomax Communication, Inc.");
MODULE_DESCRIPTION("Infomax iM98XX NAND MTD driver");
MODULE_LICENSE("GPL");
