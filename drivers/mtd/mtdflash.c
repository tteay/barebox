/*
 * MTD flash device
 *
 * Copyright (C) 2011 Robert Jarzmik
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Adds a character devices :
 *  - mtdflash<N>
 *
 * Device mtd_raw<N> provides acces to the MTD "pages+OOB". For example if a MTD
 * has pages of 512 bytes and OOB of 16 bytes, mtd_oob<N> will be made of blocks
 * of 528 bytes, with page data being followed by OOB.
 * The layout will be: <page0> <oob0> <page1> <oob1> ... <pageN> <oobN>.
 * This means that a read at offset 516 of 20 bytes will give the 12 last bytes
 * of the OOB of page0, and the 8 first bytes of page1.
 * Same thing applies for writes, which have to be page+oob aligned (ie. offset
 * and size should be multiples of (mtd->writesize + mtd->oobsize)).
 */

#include <common.h>
#include <init.h>
#include <malloc.h>
#include <ioctl.h>
#include <errno.h>
#include <linux/mtd/mtd.h>

#include "mtd.h"

/* Must be a multiple of the largest NAND page size */
#define FLASH_WRITEBUF_SIZE	4096

/**
 * mtdflash - mtdflash device private data
 * @cdev: character device "mtdflash<N>"
 * @mtd: MTD device to handle read/writes/erases
 *
 * @writebuf: buffer to handle unaligned writes (ie. writes of sizes which are
 * not multiples of MTD (writesize+oobsize)
 * @write_fill: number of bytes in writebuf
 * @write_ofs: offset in character device (mtdflash) where last write(s) stored
 * bytes because of unaligned writes (ie. remain of writesize+oobsize write)
 *
 * The mtdflash device must allow unaligned writes. This is enabled by a write buffer which gathers data to issue mtd->write_oob() with full page+oob data.
 * Suppose writesize=512, oobsize=16.
 * A first write of 512 bytes triggers:
 *  - write_ofs = offset of write()
 *  - write_fill = 512
 *  - copy of the 512 provided bytes into writebuf
 *  - no actual mtd->write if done
 * A second write of 512 bytes triggers:
 *  - copy of the 16 first bytes into writebuf
 *  - a mtd_write_oob() from writebuf
 *  - empty writebuf
 *  - copy the remaining 496 bytes into writebuf
 *    => write_fill = 496, write_ofs = offset + 528
 * Etc ...
 */
struct mtdflash {
	struct cdev cdev;
	struct mtd_info *mtd;
	void *writebuf;
	int write_fill;
	int write_ofs;
	int skip_bad;
};

static struct mtdflash *to_mtdflash(struct cdev *cdev)
{
	return cdev->priv;
}

static struct mtd_info *to_mtd(struct cdev *cdev)
{
	struct mtdflash *mtdflash = to_mtdflash(cdev);
	return mtdflash->mtd;
}

static ssize_t mtdflash_read_unaligned(struct mtd_info *mtd, void *dst,
				     size_t count, int skip, ulong offset)
{
	struct mtd_oob_ops ops;
	ssize_t ret;
	int partial = 0;
	void *tmp = dst;

	if (skip || count < mtd->writesize + mtd->oobsize)
		partial = 1;
	if (partial)
		tmp = malloc(mtd->writesize + mtd->oobsize);
	if (!tmp)
		return -ENOMEM;
	ops.mode = MTD_OPS_AUTO_OOB;//MTD_OPS_RAW;
	ops.ooboffs = 0;
	ops.datbuf = tmp;
	ops.len = mtd->writesize;
	ops.oobbuf = tmp + mtd->writesize;
	ops.ooblen = mtd->oobsize;
	ret = mtd_read_oob(mtd, offset, &ops);
	if (ret)
		goto err;
	if (partial)
		memcpy(dst, tmp + skip, count);
	ret = count;
err:
	if (partial)
		free(tmp);

	return ret;
}

static ssize_t mtdflash_read(struct cdev *cdev, void *buf, size_t count,
			    loff_t _offset, ulong flags)
{
	struct mtd_info *mtd = to_mtd(cdev);
	ssize_t retlen = 0, ret = 1, toread;
	ulong numpage;
	int skip;
	unsigned long offset = _offset;

	numpage = offset / (mtd->writesize + mtd->oobsize);
	skip = offset % (mtd->writesize + mtd->oobsize);

	while (ret > 0 && count > 0) {
		toread = min_t(int, count,
				mtd->writesize + mtd->oobsize - skip);
		ret = mtdflash_read_unaligned(mtd, buf, toread,
					    skip, numpage++ * mtd->writesize);
		buf += ret;
		skip = 0;
		count -= ret;
		retlen += ret;
	}
	if (ret < 0)
		printf("err %zd\n", ret);
	else
		ret = retlen;
	return ret;
}

#ifdef CONFIG_MTD_WRITE
static ssize_t mtdflash_blkwrite(struct mtd_info *mtd, const void *buf,
			       ulong offset)
{
	struct mtd_oob_ops ops;
	int ret;

	ops.mode = MTD_OPS_AUTO_OOB;
	ops.ooboffs = 0;
	ops.datbuf = (void *)buf;
	ops.len = mtd->writesize;
	ops.oobbuf = (void *)buf + mtd->writesize;
	ops.ooblen = mtd->oobsize;
	ret = mtd_write_oob(mtd, offset, &ops);
	if (!ret)
		ret = ops.retlen + ops.oobretlen;
	return ret;
}

static void mtdflash_fillbuf(struct mtdflash *mtdflash, const void *src, int nbbytes)
{
	memcpy(mtdflash->writebuf + mtdflash->write_fill, src, nbbytes);
	mtdflash->write_fill += nbbytes;
}

static ssize_t mtdflash_write(struct cdev *cdev, const void *buf, size_t count,
			    loff_t _offset, ulong flags)
{
	struct mtdflash *mtdflash = to_mtdflash(cdev);
	struct mtd_info *mtd = to_mtd(cdev);
	int bsz = mtd->writesize + mtd->oobsize;
	ulong numpage;
	size_t retlen = 0, tofill;
	unsigned long offset = _offset;
	int ret = 0;
	int i=0;

	if(_offset == 0)
		mtdflash->skip_bad = 0;

	if(count + _offset + mtdflash->skip_bad > mtd->size){
		pr_err("%s write offset + count +badblock > mtd->size\n",__func__,_offset,count,mtdflash->skip_bad,mtd->size);
		return -EINVAL;
	}
	if (mtdflash->write_fill &&
	    mtdflash->write_ofs + mtdflash->write_fill != offset)
		return -EINVAL;
	if (mtdflash->write_fill == 0 && offset % bsz)
		return -EINVAL;

	if (mtdflash->write_fill) {
		tofill = min_t(size_t, count, bsz - mtdflash->write_fill);
		mtdflash_fillbuf(mtdflash, buf, tofill);
		offset += tofill;
		count -= tofill;
		retlen += tofill;
	}

	if (mtdflash->write_fill == bsz) {
		numpage = mtdflash->write_ofs / (mtd->writesize + mtd->oobsize);
#ifdef	CONFIG_MTD_FLASH_SKIP_BB
		do{
			ret = mtd_block_isbad(mtd,  mtd->writesize * numpage + mtdflash->skip_bad);

			if (ret > 0) {
				mtdflash->skip_bad += mtd->erasesize;//need check data + oob?
				printf("Skipping bad block at 0x%08x\n", mtd->writesize * numpage + mtdflash->skip_bad);
			}
		}while(ret>0);//skip bad blocks
#endif
		ret = mtdflash_blkwrite(mtd, mtdflash->writebuf,
				      mtdflash->skip_bad + mtd->writesize * numpage);
		mtdflash->write_fill = 0;

	}

	numpage = offset / (mtd->writesize + mtd->oobsize);
	while (ret >= 0 && count >= bsz) {
#ifdef	CONFIG_MTD_FLASH_SKIP_BB
		do{
			ret = mtd_block_isbad(mtd,  mtd->writesize * numpage + mtdflash->skip_bad);

			if (ret > 0) {
				mtdflash->skip_bad += mtd->erasesize;
				printf("Skipping bad block at 0x%08x\n", mtd->writesize * numpage + mtdflash->skip_bad);
			}
		}while(ret>0);//skip bad blocks
#endif
		ret = mtdflash_blkwrite(mtd, buf + retlen,
				   mtdflash->skip_bad + mtd->writesize * numpage++);
		count -= ret;
		retlen += ret;
		offset += ret;
	}

	if (ret >= 0 && count) {
		mtdflash->write_ofs = offset - mtdflash->write_fill;
		mtdflash_fillbuf(mtdflash, buf + retlen, count);
		retlen += count;
	}

	if (ret < 0) {
		printf("err %d\n", ret);
		return ret;
	} else {
		return retlen;
	}
}

#if 0
static int mtd_op_erase(struct cdev *cdev, size_t count, loff_t offset)
{
	struct mtd_info *mtd = cdev->priv;
	struct erase_info erase;
	uint32_t addr;
	int ret;

	ret = mtd_erase_align(mtd, &count, &offset);
	if (ret)
		return ret;

	memset(&erase, 0, sizeof(erase));
	erase.mtd = mtd;
	addr = offset;

	if (!mtd->block_isbad) {
		erase.addr = addr;
		erase.len = count;
		return mtd_erase(mtd, &erase);
	}

	erase.len = mtd->erasesize;

	while (count > 0) {
		dev_dbg(cdev->dev, "erase %d %d\n", addr, erase.len);

		if (!mtd->allow_erasebad)
			ret = mtd_block_isbad(mtd, addr);
		else
			ret = 0;

		erase.addr = addr;

		if (ret > 0) {
			printf("Skipping bad block at 0x%08x\n", addr);
		} else {
			ret = mtd_erase(mtd, &erase);
			if (ret)
				return ret;
		}

		addr += mtd->erasesize;
		count -= count > mtd->erasesize ? mtd->erasesize : count;
	}

	return 0;
}
#else
static int mtdflash_erase(struct cdev *cdev, size_t count, loff_t _offset)
{
	struct mtd_info *mtd = to_mtd(cdev);
	struct erase_info erase;
	unsigned long offset = _offset;
	int ret;
	pr_emerg("original offset %x count %x   ", offset, count);

	offset = offset / (mtd->writesize + mtd->oobsize) * mtd->writesize;
	count = count / (mtd->writesize + mtd->oobsize) * mtd->writesize;

	memset(&erase, 0, sizeof(erase));
	erase.mtd = mtd;
	erase.addr = offset;
	erase.len = mtd->erasesize;
	pr_emerg("calculate offset %x count %x\n", offset, count);

	while (count > 0) {
		//pr_emerg("erase %x %x count %x\n", erase.addr, erase.len,count);

		if (!mtd->allow_erasebad)
			ret = mtd_block_isbad(mtd, erase.addr);
		else
			ret = 0;

		if (ret > 0) {
			printf("Skipping bad block at 0x%08x\n", erase.addr);
		} else {
			ret = mtd_erase(mtd, &erase);
			if (ret)
				return ret;
		}

		offset += mtd->erasesize;
		erase.addr = offset;//erase.addr += mtd->erasesize;
		count -= count > mtd->erasesize ? mtd->erasesize : count;
	}

	return 0;
}
#endif
#else
static ssize_t mtdflash_write(struct cdev *cdev, const void *buf, size_t count,
			    loff_t offset, ulong flags)
{
	return 0;
}
static ssize_t mtdflash_erase(struct cdev *cdev, size_t count, loff_t offset)
{
	return 0;
}
#endif

static const struct file_operations mtd_flash_fops = {
	.read		= mtdflash_read,
	.write		= mtdflash_write,
	.erase		= mtdflash_erase,
	.lseek		= dev_lseek_default,
};

static int mtd_part_read_oob(struct mtd_info *mtd, loff_t from,
			struct mtd_oob_ops *ops)
{
	int res;

	if (from >= mtd->size)
		return -EIO;
		res = mtd->master->read_oob(mtd->master, from + mtd->master_offset,
				ops);
	return res;
}

static int mtd_part_write_oob(struct mtd_info *mtd, loff_t to,
			 struct mtd_oob_ops *ops)
{
	if (!(mtd->flags & MTD_WRITEABLE))
		return -EROFS;
	if (to >= mtd->size)
		return -EIO;
	return mtd->master->write_oob(mtd->master, to + mtd->master_offset,
					ops);
}


static int add_mtdflash_device(struct mtd_info *mtd, char *devname, void **priv)
{
	struct mtdflash *mtdflash;
	pr_emerg("%s mtd:%p,devname:%s, mtd->master %p, mtd->oobsize %d, mtd->size:0x%x,\n", __func__,mtd,devname,mtd->master,mtd->oobsize,(int)mtd->size);

	if ( mtd->oobsize == 0)
		return 0;
	if(mtd->master){
		if(!mtd->read_oob)
			mtd->read_oob = mtd_part_read_oob;
		if(!mtd->write_oob){
			mtd->write_oob = mtd_part_write_oob;
			pr_emerg("%s mtd:%p,devname:%s set write_oob!!\n", __func__,mtd,devname);

		}
	}
	mtdflash = xzalloc(sizeof(*mtdflash));
	mtdflash->writebuf = xmalloc(FLASH_WRITEBUF_SIZE);
	mtdflash->mtd = mtd;

	mtdflash->cdev.ops = (struct file_operations *)&mtd_flash_fops;
	mtdflash->cdev.size = mtd_div_by_wb(mtd->size, mtd) *
		(mtd->writesize + mtd->oobsize);
	mtdflash->cdev.name = asprintf("%s.fsh", mtd->cdev.name);
	mtdflash->cdev.priv = mtdflash;
	mtdflash->cdev.dev = &mtd->class_dev;
	mtdflash->cdev.mtd = mtd;
	*priv = mtdflash;
	pr_emerg("%s %s\n",__func__,mtdflash->cdev.name);
	devfs_create(&mtdflash->cdev);

	return 0;
}

static int del_mtdflash_device(struct mtd_info *mtd, void **priv)
{
	struct mtdflash *mtdflash;

	if (mtd->master || mtd->oobsize == 0)
		return 0;

	mtdflash = *priv;
	devfs_remove(&mtdflash->cdev);
	free(mtdflash);

	return 0;
}

static struct mtddev_hook mtdflash_hook = {
	.add_mtd_device = add_mtdflash_device,
	.del_mtd_device = del_mtdflash_device,
};

static int __init register_mtdflash(void)
{
	mtdcore_add_hook(&mtdflash_hook);
	return 0;
}

coredevice_initcall(register_mtdflash);
