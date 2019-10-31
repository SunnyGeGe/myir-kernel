/*
 * Copyright © 2004 Texas Instruments, Jian Zhang <jzhang@ti.com>
 * Copyright © 2004 Micron Technology Inc.
 * Copyright © 2004 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <plat/dma.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/elm.h>

#define	DRIVER_NAME	"omap2-nand"
#define	OMAP_NAND_TIMEOUT_MS	5000

#define NAND_Ecc_P1e		(1 << 0)
#define NAND_Ecc_P2e		(1 << 1)
#define NAND_Ecc_P4e		(1 << 2)
#define NAND_Ecc_P8e		(1 << 3)
#define NAND_Ecc_P16e		(1 << 4)
#define NAND_Ecc_P32e		(1 << 5)
#define NAND_Ecc_P64e		(1 << 6)
#define NAND_Ecc_P128e		(1 << 7)
#define NAND_Ecc_P256e		(1 << 8)
#define NAND_Ecc_P512e		(1 << 9)
#define NAND_Ecc_P1024e		(1 << 10)
#define NAND_Ecc_P2048e		(1 << 11)

#define NAND_Ecc_P1o		(1 << 16)
#define NAND_Ecc_P2o		(1 << 17)
#define NAND_Ecc_P4o		(1 << 18)
#define NAND_Ecc_P8o		(1 << 19)
#define NAND_Ecc_P16o		(1 << 20)
#define NAND_Ecc_P32o		(1 << 21)
#define NAND_Ecc_P64o		(1 << 22)
#define NAND_Ecc_P128o		(1 << 23)
#define NAND_Ecc_P256o		(1 << 24)
#define NAND_Ecc_P512o		(1 << 25)
#define NAND_Ecc_P1024o		(1 << 26)
#define NAND_Ecc_P2048o		(1 << 27)

#define TF(value)	(value ? 1 : 0)

#define P2048e(a)	(TF(a & NAND_Ecc_P2048e)	<< 0)
#define P2048o(a)	(TF(a & NAND_Ecc_P2048o)	<< 1)
#define P1e(a)		(TF(a & NAND_Ecc_P1e)		<< 2)
#define P1o(a)		(TF(a & NAND_Ecc_P1o)		<< 3)
#define P2e(a)		(TF(a & NAND_Ecc_P2e)		<< 4)
#define P2o(a)		(TF(a & NAND_Ecc_P2o)		<< 5)
#define P4e(a)		(TF(a & NAND_Ecc_P4e)		<< 6)
#define P4o(a)		(TF(a & NAND_Ecc_P4o)		<< 7)

#define P8e(a)		(TF(a & NAND_Ecc_P8e)		<< 0)
#define P8o(a)		(TF(a & NAND_Ecc_P8o)		<< 1)
#define P16e(a)		(TF(a & NAND_Ecc_P16e)		<< 2)
#define P16o(a)		(TF(a & NAND_Ecc_P16o)		<< 3)
#define P32e(a)		(TF(a & NAND_Ecc_P32e)		<< 4)
#define P32o(a)		(TF(a & NAND_Ecc_P32o)		<< 5)
#define P64e(a)		(TF(a & NAND_Ecc_P64e)		<< 6)
#define P64o(a)		(TF(a & NAND_Ecc_P64o)		<< 7)

#define P128e(a)	(TF(a & NAND_Ecc_P128e)		<< 0)
#define P128o(a)	(TF(a & NAND_Ecc_P128o)		<< 1)
#define P256e(a)	(TF(a & NAND_Ecc_P256e)		<< 2)
#define P256o(a)	(TF(a & NAND_Ecc_P256o)		<< 3)
#define P512e(a)	(TF(a & NAND_Ecc_P512e)		<< 4)
#define P512o(a)	(TF(a & NAND_Ecc_P512o)		<< 5)
#define P1024e(a)	(TF(a & NAND_Ecc_P1024e)	<< 6)
#define P1024o(a)	(TF(a & NAND_Ecc_P1024o)	<< 7)

#define P8e_s(a)	(TF(a & NAND_Ecc_P8e)		<< 0)
#define P8o_s(a)	(TF(a & NAND_Ecc_P8o)		<< 1)
#define P16e_s(a)	(TF(a & NAND_Ecc_P16e)		<< 2)
#define P16o_s(a)	(TF(a & NAND_Ecc_P16o)		<< 3)
#define P1e_s(a)	(TF(a & NAND_Ecc_P1e)		<< 4)
#define P1o_s(a)	(TF(a & NAND_Ecc_P1o)		<< 5)
#define P2e_s(a)	(TF(a & NAND_Ecc_P2e)		<< 6)
#define P2o_s(a)	(TF(a & NAND_Ecc_P2o)		<< 7)

#define P4e_s(a)	(TF(a & NAND_Ecc_P4e)		<< 0)
#define P4o_s(a)	(TF(a & NAND_Ecc_P4o)		<< 1)

#define MAX_HWECC_BYTES_OOB_64     24
#define JFFS2_CLEAN_MARKER_OFFSET  0x2

#define BCH_ECC_POS			0x2
#define BCH_JFFS2_CLEAN_MARKER_OFFSET	0x3a
#define OMAP_BCH8_ECC_SECT_BYTES	14

static u_char bch8_vector[] = {0xf3, 0xdb, 0x14, 0x16, 0x8b, 0xd2, 0xbe, 0xcc,
	0xac, 0x6b, 0xff, 0x99, 0x7b};
static u_char bch4_vector[] = {0x00, 0x6b, 0x31, 0xdd, 0x41, 0xbc, 0x10};

/* oob info generated runtime depending on ecc algorithm and layout selected */
static struct nand_ecclayout omap_oobinfo;
/* Define some generic bad / good block scan pattern which are used
 * while scanning a device for factory marked good / bad blocks
 */
static uint8_t scan_ff_pattern[] = { 0xff };
static struct nand_bbt_descr bb_descrip_flashbased = {
	.options = NAND_BBT_SCANEMPTY | NAND_BBT_SCANALLPAGES,
	.offs = 0,
	.len = 1,
	.pattern = scan_ff_pattern,
};


struct omap_nand_info {
	struct nand_hw_control		controller;
	struct omap_nand_platform_data	*pdata;
	struct mtd_info			mtd;
	struct nand_chip		nand;
	struct platform_device		*pdev;

	int				gpmc_cs;
	unsigned long			phys_base;
	struct completion		comp;
	int				dma_ch;
	int				gpmc_irq;
	enum {
		OMAP_NAND_IO_READ = 0,	/* read */
		OMAP_NAND_IO_WRITE,	/* write */
	} iomode;
	u_char				*buf;
	int				buf_len;
	int				ecc_opt;
	int (*ctrlr_suspend) (void);
	int (*ctrlr_resume) (void);
};

/**
 * omap_hwcontrol - hardware specific access to control-lines
 * @mtd: MTD device structure
 * @cmd: command to device
 * @ctrl:
 * NAND_NCE: bit 0 -> don't care
 * NAND_CLE: bit 1 -> Command Latch
 * NAND_ALE: bit 2 -> Address Latch
 *
 * NOTE: boards may use different bits for these!!
 */
static void omap_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);

	if (cmd != NAND_CMD_NONE) {
		if (ctrl & NAND_CLE)
			gpmc_nand_write(info->gpmc_cs, GPMC_NAND_COMMAND, cmd);

		else if (ctrl & NAND_ALE)
			gpmc_nand_write(info->gpmc_cs, GPMC_NAND_ADDRESS, cmd);

		else /* NAND_NCE */
			gpmc_nand_write(info->gpmc_cs, GPMC_NAND_DATA, cmd);
	}
}

/**
 * omap_read_buf8 - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf8(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand = mtd->priv;

	ioread8_rep(nand->IO_ADDR_R, buf, len);
}

/**
 * omap_write_buf8 - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf8(struct mtd_info *mtd, const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	u_char *p = (u_char *)buf;
	u32	status = 0;

	while (len--) {
		iowrite8(*p++, info->nand.IO_ADDR_W);
		/* wait until buffer is available for write */
		do {
			status = gpmc_read_status(GPMC_STATUS_BUFFER);
		} while (!status);
	}
}

/**
 * omap_read_buf16 - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf16(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand = mtd->priv;

	ioread16_rep(nand->IO_ADDR_R, buf, len / 2);
}

/**
 * omap_write_buf16 - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf16(struct mtd_info *mtd, const u_char * buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	u16 *p = (u16 *) buf;
	u32	status = 0;
	/* FIXME try bursts of writesw() or DMA ... */
	len >>= 1;

	while (len--) {
		iowrite16(*p++, info->nand.IO_ADDR_W);
		/* wait until buffer is available for write */
		do {
			status = gpmc_read_status(GPMC_STATUS_BUFFER);
		} while (!status);
	}
}

/**
 * omap_read_buf_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf_pref(struct mtd_info *mtd, u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	uint32_t r_count = 0;
	int ret = 0;
	u32 *p = (u32 *)buf;

	/* take care of subpage reads */
	if (len % 4) {
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_read_buf16(mtd, buf, len % 4);
		else
			omap_read_buf8(mtd, buf, len % 4);
		p = (u32 *) (buf + len % 4);
		len -= len % 4;
	}

	/* configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs,
			PREFETCH_FIFOTHRESHOLD_MAX, 0x0, len, 0x0);
	if (ret) {
		/* PFPW engine is busy, use cpu copy method */
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_read_buf16(mtd, (u_char *)p, len);
		else
			omap_read_buf8(mtd, (u_char *)p, len);
	} else {
		do {
			r_count = gpmc_read_status(GPMC_PREFETCH_FIFO_CNT);
			r_count = r_count >> 2;
			ioread32_rep(info->nand.IO_ADDR_R, p, r_count);
			p += r_count;
			len -= r_count << 2;
		} while (len);
		/* disable and stop the PFPW engine */
		gpmc_prefetch_reset(info->gpmc_cs);
	}
}

/**
 * omap_write_buf_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf_pref(struct mtd_info *mtd,
					const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	uint32_t w_count = 0;
	int i = 0, ret = 0;
	u16 *p = (u16 *)buf;
	unsigned long tim, limit;

	/* take care of subpage writes */
	if (len % 2 != 0) {
		writeb(*buf, info->nand.IO_ADDR_W);
		p = (u16 *)(buf + 1);
		len--;
	}

	/*  configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs,
			PREFETCH_FIFOTHRESHOLD_MAX, 0x0, len, 0x1);
	if (ret) {
		/* PFPW engine is busy, use cpu copy method */
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_write_buf16(mtd, (u_char *)p, len);
		else
			omap_write_buf8(mtd, (u_char *)p, len);
	} else {
		while (len) {
			w_count = gpmc_read_status(GPMC_PREFETCH_FIFO_CNT);
			w_count = w_count >> 1;
			for (i = 0; (i < w_count) && len; i++, len -= 2)
				iowrite16(*p++, info->nand.IO_ADDR_W);
		}
		/* wait for data to flushed-out before reset the prefetch */
		tim = 0;
		limit = (loops_per_jiffy *
					msecs_to_jiffies(OMAP_NAND_TIMEOUT_MS));
		while (gpmc_read_status(GPMC_PREFETCH_COUNT) && (tim++ < limit))
			cpu_relax();

		/* disable and stop the PFPW engine */
		gpmc_prefetch_reset(info->gpmc_cs);
	}
}

/*
 * omap_nand_dma_cb: callback on the completion of dma transfer
 * @lch: logical channel
 * @ch_satuts: channel status
 * @data: pointer to completion data structure
 */
static void omap_nand_dma_cb(int lch, u16 ch_status, void *data)
{
	complete((struct completion *) data);
}

/*
 * omap_nand_dma_transfer: configer and start dma transfer
 * @mtd: MTD device structure
 * @addr: virtual address in RAM of source/destination
 * @len: number of data bytes to be transferred
 * @is_write: flag for read/write operation
 */
static inline int omap_nand_dma_transfer(struct mtd_info *mtd, void *addr,
					unsigned int len, int is_write)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);
	enum dma_data_direction dir = is_write ? DMA_TO_DEVICE :
							DMA_FROM_DEVICE;
	dma_addr_t dma_addr;
	int ret;
	unsigned long tim, limit;

	/* The fifo depth is 64 bytes max.
	 * But configure the FIFO-threahold to 32 to get a sync at each frame
	 * and frame length is 32 bytes.
	 */
	int buf_len = len >> 6;

	if (addr >= high_memory) {
		struct page *p1;

		if (((size_t)addr & PAGE_MASK) !=
			((size_t)(addr + len - 1) & PAGE_MASK))
			goto out_copy;
		p1 = vmalloc_to_page(addr);
		if (!p1)
			goto out_copy;
		addr = page_address(p1) + ((size_t)addr & ~PAGE_MASK);
	}

	dma_addr = dma_map_single(&info->pdev->dev, addr, len, dir);
	if (dma_mapping_error(&info->pdev->dev, dma_addr)) {
		dev_err(&info->pdev->dev,
			"Couldn't DMA map a %d byte buffer\n", len);
		goto out_copy;
	}

	if (is_write) {
	    omap_set_dma_dest_params(info->dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
						info->phys_base, 0, 0);
	    omap_set_dma_src_params(info->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
							dma_addr, 0, 0);
	    omap_set_dma_transfer_params(info->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					0x10, buf_len, OMAP_DMA_SYNC_FRAME,
					OMAP24XX_DMA_GPMC, OMAP_DMA_DST_SYNC);
	} else {
	    omap_set_dma_src_params(info->dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
						info->phys_base, 0, 0);
	    omap_set_dma_dest_params(info->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
							dma_addr, 0, 0);
	    omap_set_dma_transfer_params(info->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					0x10, buf_len, OMAP_DMA_SYNC_FRAME,
					OMAP24XX_DMA_GPMC, OMAP_DMA_SRC_SYNC);
	}
	/*  configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs,
			PREFETCH_FIFOTHRESHOLD_MAX, 0x1, len, is_write);
	if (ret)
		/* PFPW engine is busy, use cpu copy method */
		goto out_copy;

	init_completion(&info->comp);

	omap_start_dma(info->dma_ch);

	/* setup and start DMA using dma_addr */
	wait_for_completion(&info->comp);
	tim = 0;
	limit = (loops_per_jiffy * msecs_to_jiffies(OMAP_NAND_TIMEOUT_MS));
	while (gpmc_read_status(GPMC_PREFETCH_COUNT) && (tim++ < limit))
		cpu_relax();

	/* disable and stop the PFPW engine */
	gpmc_prefetch_reset(info->gpmc_cs);

	dma_unmap_single(&info->pdev->dev, dma_addr, len, dir);
	return 0;

out_copy:
	if (info->nand.options & NAND_BUSWIDTH_16)
		is_write == 0 ? omap_read_buf16(mtd, (u_char *) addr, len)
			: omap_write_buf16(mtd, (u_char *) addr, len);
	else
		is_write == 0 ? omap_read_buf8(mtd, (u_char *) addr, len)
			: omap_write_buf8(mtd, (u_char *) addr, len);
	return 0;
}

/**
 * omap_read_buf_dma_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf_dma_pref(struct mtd_info *mtd, u_char *buf, int len)
{
	if (len <= mtd->oobsize)
		omap_read_buf_pref(mtd, buf, len);
	else
		/* start transfer in DMA mode */
		omap_nand_dma_transfer(mtd, buf, len, 0x0);
}

/**
 * omap_write_buf_dma_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf_dma_pref(struct mtd_info *mtd,
					const u_char *buf, int len)
{
	if (len <= mtd->oobsize)
		omap_write_buf_pref(mtd, buf, len);
	else
		/* start transfer in DMA mode */
		omap_nand_dma_transfer(mtd, (u_char *) buf, len, 0x1);
}

/*
 * omap_nand_irq - GMPC irq handler
 * @this_irq: gpmc irq number
 * @dev: omap_nand_info structure pointer is passed here
 */
static irqreturn_t omap_nand_irq(int this_irq, void *dev)
{
	struct omap_nand_info *info = (struct omap_nand_info *) dev;
	u32 bytes;
	u32 irq_stat;

	irq_stat = gpmc_read_status(GPMC_GET_IRQ_STATUS);
	bytes = gpmc_read_status(GPMC_PREFETCH_FIFO_CNT);
	bytes = bytes  & 0xFFFC; /* io in multiple of 4 bytes */
	if (info->iomode == OMAP_NAND_IO_WRITE) { /* checks for write io */
		if (irq_stat & 0x2)
			goto done;

		if (info->buf_len && (info->buf_len < bytes))
			bytes = info->buf_len;
		else if (!info->buf_len)
			bytes = 0;
		iowrite32_rep(info->nand.IO_ADDR_W,
						(u32 *)info->buf, bytes >> 2);
		info->buf = info->buf + bytes;
		info->buf_len -= bytes;

	} else {
		ioread32_rep(info->nand.IO_ADDR_R,
						(u32 *)info->buf, bytes >> 2);
		info->buf = info->buf + bytes;

		if (irq_stat & 0x2)
			goto done;
	}
	gpmc_cs_configure(info->gpmc_cs, GPMC_SET_IRQ_STATUS, irq_stat);

	return IRQ_HANDLED;

done:
	complete(&info->comp);
	/* disable irq */
	gpmc_cs_configure(info->gpmc_cs, GPMC_ENABLE_IRQ, 0);

	/* clear status */
	gpmc_cs_configure(info->gpmc_cs, GPMC_SET_IRQ_STATUS, irq_stat);

	return IRQ_HANDLED;
}

/*
 * omap_read_buf_irq_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf_irq_pref(struct mtd_info *mtd, u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	int ret = 0;

	if (len <= mtd->oobsize) {
		omap_read_buf_pref(mtd, buf, len);
		return;
	}

	info->iomode = OMAP_NAND_IO_READ;
	info->buf = buf;
	init_completion(&info->comp);

	/*  configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs,
			PREFETCH_FIFOTHRESHOLD_MAX/2, 0x0, len, 0x0);
	if (ret)
		/* PFPW engine is busy, use cpu copy method */
		goto out_copy;

	info->buf_len = len;
	/* enable irq */
	gpmc_cs_configure(info->gpmc_cs, GPMC_ENABLE_IRQ,
		(GPMC_IRQ_FIFOEVENTENABLE | GPMC_IRQ_COUNT_EVENT));

	/* waiting for read to complete */
	wait_for_completion(&info->comp);

	/* disable and stop the PFPW engine */
	gpmc_prefetch_reset(info->gpmc_cs);
	return;

out_copy:
	if (info->nand.options & NAND_BUSWIDTH_16)
		omap_read_buf16(mtd, buf, len);
	else
		omap_read_buf8(mtd, buf, len);
}

/*
 * omap_write_buf_irq_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf_irq_pref(struct mtd_info *mtd,
					const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	int ret = 0;
	unsigned long tim, limit;

	if (len <= mtd->oobsize) {
		omap_write_buf_pref(mtd, buf, len);
		return;
	}

	info->iomode = OMAP_NAND_IO_WRITE;
	info->buf = (u_char *) buf;
	init_completion(&info->comp);

	/* configure and start prefetch transfer : size=24 */
	ret = gpmc_prefetch_enable(info->gpmc_cs,
			(PREFETCH_FIFOTHRESHOLD_MAX * 3) / 8, 0x0, len, 0x1);
	if (ret)
		/* PFPW engine is busy, use cpu copy method */
		goto out_copy;

	info->buf_len = len;
	/* enable irq */
	gpmc_cs_configure(info->gpmc_cs, GPMC_ENABLE_IRQ,
			(GPMC_IRQ_FIFOEVENTENABLE | GPMC_IRQ_COUNT_EVENT));

	/* waiting for write to complete */
	wait_for_completion(&info->comp);
	/* wait for data to flushed-out before reset the prefetch */
	tim = 0;
	limit = (loops_per_jiffy *  msecs_to_jiffies(OMAP_NAND_TIMEOUT_MS));
	while (gpmc_read_status(GPMC_PREFETCH_COUNT) && (tim++ < limit))
		cpu_relax();

	/* disable and stop the PFPW engine */
	gpmc_prefetch_reset(info->gpmc_cs);
	return;

out_copy:
	if (info->nand.options & NAND_BUSWIDTH_16)
		omap_write_buf16(mtd, buf, len);
	else
		omap_write_buf8(mtd, buf, len);
}

/**
 * omap_verify_buf - Verify chip data against buffer
 * @mtd: MTD device structure
 * @buf: buffer containing the data to compare
 * @len: number of bytes to compare
 */
static int omap_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	u16 *p = (u16 *) buf;

	len >>= 1;
	while (len--) {
		if (*p++ != cpu_to_le16(readw(info->nand.IO_ADDR_R)))
			return -EFAULT;
	}

	return 0;
}

/**
 * gen_true_ecc - This function will generate true ECC value
 * @ecc_buf: buffer to store ecc code
 *
 * This generated true ECC value can be used when correcting
 * data read from NAND flash memory core
 */
static void gen_true_ecc(u8 *ecc_buf)
{
	u32 tmp = ecc_buf[0] | (ecc_buf[1] << 16) |
		((ecc_buf[2] & 0xF0) << 20) | ((ecc_buf[2] & 0x0F) << 8);

	ecc_buf[0] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) |
			P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp));
	ecc_buf[1] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) |
			P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
	ecc_buf[2] = ~(P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) |
			P1e(tmp) | P2048o(tmp) | P2048e(tmp));
}

/**
 * omap_compare_ecc - Detect (2 bits) and correct (1 bit) error in data
 * @ecc_data1:  ecc code from nand spare area
 * @ecc_data2:  ecc code from hardware register obtained from hardware ecc
 * @page_data:  page data
 *
 * This function compares two ECC's and indicates if there is an error.
 * If the error can be corrected it will be corrected to the buffer.
 * If there is no error, %0 is returned. If there is an error but it
 * was corrected, %1 is returned. Otherwise, %-1 is returned.
 */
static int omap_compare_ecc(u8 *ecc_data1,	/* read from NAND memory */
			    u8 *ecc_data2,	/* read from register */
			    u8 *page_data)
{
	uint	i;
	u8	tmp0_bit[8], tmp1_bit[8], tmp2_bit[8];
	u8	comp0_bit[8], comp1_bit[8], comp2_bit[8];
	u8	ecc_bit[24];
	u8	ecc_sum = 0;
	u8	find_bit = 0;
	uint	find_byte = 0;
	int	isEccFF;

	isEccFF = ((*(u32 *)ecc_data1 & 0xFFFFFF) == 0xFFFFFF);

	gen_true_ecc(ecc_data1);
	gen_true_ecc(ecc_data2);

	for (i = 0; i <= 2; i++) {
		*(ecc_data1 + i) = ~(*(ecc_data1 + i));
		*(ecc_data2 + i) = ~(*(ecc_data2 + i));
	}

	for (i = 0; i < 8; i++) {
		tmp0_bit[i]     = *ecc_data1 % 2;
		*ecc_data1	= *ecc_data1 / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp1_bit[i]	 = *(ecc_data1 + 1) % 2;
		*(ecc_data1 + 1) = *(ecc_data1 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp2_bit[i]	 = *(ecc_data1 + 2) % 2;
		*(ecc_data1 + 2) = *(ecc_data1 + 2) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp0_bit[i]     = *ecc_data2 % 2;
		*ecc_data2       = *ecc_data2 / 2;
	}

	for (i = 0; i < 8; i++) {
		comp1_bit[i]     = *(ecc_data2 + 1) % 2;
		*(ecc_data2 + 1) = *(ecc_data2 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp2_bit[i]     = *(ecc_data2 + 2) % 2;
		*(ecc_data2 + 2) = *(ecc_data2 + 2) / 2;
	}

	for (i = 0; i < 6; i++)
		ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];

	ecc_bit[22] = tmp2_bit[0] ^ comp2_bit[0];
	ecc_bit[23] = tmp2_bit[1] ^ comp2_bit[1];

	for (i = 0; i < 24; i++)
		ecc_sum += ecc_bit[i];

	switch (ecc_sum) {
	case 0:
		/* Not reached because this function is not called if
		 *  ECC values are equal
		 */
		return 0;

	case 1:
		/* Uncorrectable error */
		pr_debug("ECC UNCORRECTED_ERROR 1\n");
		return -1;

	case 11:
		/* UN-Correctable error */
		pr_debug("ECC UNCORRECTED_ERROR B\n");
		return -1;

	case 12:
		/* Correctable error */
		find_byte = (ecc_bit[23] << 8) +
			    (ecc_bit[21] << 7) +
			    (ecc_bit[19] << 6) +
			    (ecc_bit[17] << 5) +
			    (ecc_bit[15] << 4) +
			    (ecc_bit[13] << 3) +
			    (ecc_bit[11] << 2) +
			    (ecc_bit[9]  << 1) +
			    ecc_bit[7];

		find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];

		pr_debug("Correcting single bit ECC error at offset: "
				"%d, bit: %d\n", find_byte, find_bit);

		page_data[find_byte] ^= (1 << find_bit);

		return 1;
	default:
		if (isEccFF) {
			if (ecc_data2[0] == 0 &&
			    ecc_data2[1] == 0 &&
			    ecc_data2[2] == 0)
				return 0;
		}
		pr_debug("UNCORRECTED_ERROR default\n");
		return -1;
	}
}

/**
 * erased_sector_bitflips - count bit flips
 * @data:	data sector buffer
 * @oob:	oob buffer
 * @info:	omap_nand_info
 *
 * Check the bit flips in erased page falls below correctable level.
 * If falls below, report the page as erased with correctable bit
 * flip, else report as uncorrectable page.
 */
static int erased_sector_bitflips(u_char *data, u_char *oob,
		struct omap_nand_info *info)
{
	int flip_bits = 0, i;

	for (i = 0; i < info->nand.ecc.size; i++) {
		flip_bits += hweight8(~data[i]);
		if (flip_bits > 8)
			return 0;
	}

	for (i = 0; i < info->nand.ecc.bytes - 1; i++) {
		flip_bits += hweight8(~oob[i]);
		if (flip_bits > 8)
			return 0;
	}

	/*
	 * Bit flips falls in correctable level.
	 * Fill data area with 0xFF
	 */
	if (flip_bits) {
		memset(data, 0xFF, info->nand.ecc.size);
		memset(oob, 0xFF, info->nand.ecc.bytes);
	}

	return flip_bits;
}


/**
 * omap_read_page_bch - BCH ecc based page read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 * @page:	page number to read
 *
 * For BCH ECC scheme, GPMC used for syndrome calculation and ELM module
 * used for error correction.
 */
static int omap_read_page_bch(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int page)
{
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	uint8_t *oob = &chip->oob_poi[eccpos[0]];
	uint32_t oob_pos = mtd->writesize + chip->ecc.layout->eccpos[0];
	int stat;
	/* Enable GPMC ecc engine */
	chip->ecc.hwctl(mtd, NAND_ECC_READ);

	/* Read data */
	chip->read_buf(mtd, buf, mtd->writesize);

	/* Read oob bytes */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, oob_pos, -1);
	chip->read_buf(mtd, oob, chip->ecc.total);

	/* Calculate ecc bytes */
	chip->ecc.calculate(mtd, buf, ecc_calc);
	memcpy(ecc_code, &chip->oob_poi[eccpos[0]], chip->ecc.total);

	stat = chip->ecc.correct(mtd, buf, ecc_code, ecc_calc);

	if (stat < 0) {
		mtd->ecc_stats.failed++;
	} else {
		mtd->ecc_stats.corrected += stat;
	}
	return 0;
}

/**
 * omap_correct_data - Compares the ECC read with HW generated ECC
 * @mtd: MTD device structure
 * @dat: page data
 * @read_ecc: ecc read from nand flash
 * @calc_ecc: ecc read from HW ECC registers
 *
 * Compares the ecc read from nand spare area with ECC registers values
 * and if ECC's mismatched, it will call 'omap_compare_ecc' for error
 * detection and correction. If there are no errors, %0 is returned. If
 * there were errors and all of the errors were corrected, the number of
 * corrected errors is returned. If uncorrectable errors exist, %-1 is
 * returned.
 */
static int omap_correct_data(struct mtd_info *mtd, u_char *data,
				u_char *read_ecc, u_char *calc_ecc)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);

	struct nand_ecc_ctrl *ecc = &info->nand.ecc;
	int eccsteps = info->nand.ecc.steps;
	int i , j, stat = 0;
	int eccflag, actual_eccbytes;
	struct elm_errorvec err_vec[ERROR_VECTOR_MAX];
	u_char *ecc_vec = calc_ecc;
	u_char *spare_ecc = read_ecc;
	u_char *erased_ecc_vec;
	u_char *buf;
	int bitflip_count;
	bool is_error_reported = false;
	u32 bit_pos, byte_pos, error_max, pos;
	int bch_type;
	int err;
	switch (info->ecc_opt) {
	case OMAP_ECC_BCH4_CODE_HW:
		/* omit  7th ECC byte reserved for ROM code compatibility */
		actual_eccbytes = ecc->bytes - 1;
		erased_ecc_vec = bch4_vector;
		bch_type = OMAP_BCH4_ECC;
		break;
	case OMAP_ECC_BCH8_CODE_HW:
		/* omit 14th ECC byte reserved for ROM code compatibility */
		actual_eccbytes = ecc->bytes - 1;
		erased_ecc_vec = bch8_vector;
		bch_type = OMAP_BCH8_ECC;
		break;
	default:
		dev_err(&info->pdev->dev, "invalid driver configuration\n");
		return -EINVAL;
	}

	/* Initialize elm error vector to zero */
	memset(err_vec, 0, sizeof(err_vec));
	for (i = 0; i < eccsteps ; i++) {
		eccflag = 0;	/* initialize eccflag */

		/*
		 * Check any error reported,
		 * In case of error, non zero ecc reported.
		 */
		for (j = 0; j < actual_eccbytes; j++) {
			if (calc_ecc[j] != 0) {
				eccflag = 1; /* non zero ecc, error present */
				break;
			}
		}

		if (eccflag == 1) {
			if (memcmp(calc_ecc, erased_ecc_vec,
						actual_eccbytes) == 0) {
				/*
				 * calc_ecc[] matches pattern for ECC(all 0xff)
				 * so this is definitely an erased-page
				 */
			} else {
				buf = &data[info->nand.ecc.size * i];
				/*
				 * count number of 0-bits in read_buf.
				 * This check can be removed once a similar
				 * check is introduced in generic NAND driver
				 */
				bitflip_count = erased_sector_bitflips(
						buf, read_ecc, info);
				if (bitflip_count) {
					/*
					 * number of 0-bits within ECC limits
					 * So this may be an erased-page
					 */
					stat += bitflip_count;
				} else {
					/*
					 * Too many 0-bits. It may be a
					 * - programmed-page, OR
					 * - erased-page with many bit-flips
					 * So this page requires check by ELM
					 */
					err_vec[i].error_reported = true;
					is_error_reported = true;
				}
			}
		}


		/* Update the ecc vector */
		calc_ecc += ecc->bytes;
		read_ecc += ecc->bytes;
}
	
	/* Check if any error reported */
	if (!is_error_reported)
		return stat;

	/* Decode BCH error using ELM module */
	omap_elm_decode_bch_error(bch_type, ecc_vec, err_vec);

	err = 0;
	for (i = 0; i < eccsteps; i++) {
		if (err_vec[i].error_uncorrectable) {
			dev_err(&info->pdev->dev,
				"uncorrectable bit-flips found\n");
			err = -EBADMSG;
		} else if (err_vec[i].error_reported) {
			for (j = 0; j < err_vec[i].error_count; j++) {
				switch (info->ecc_opt) {

				case OMAP_ECC_BCH8_CODE_HW:
					pos = err_vec[i].error_loc[j];
					break;
				default:
					return -EINVAL;
				}
				error_max = (ecc->size + actual_eccbytes) * 8;
				/* Calculate bit position of error */
				bit_pos = pos % 8;

				/* Calculate byte position of error */
				byte_pos = (error_max - pos - 1) / 8;

				if (pos < error_max) {
					if (byte_pos < 512) {
						pr_debug("bitflip@dat[%d]=%x\n",
						     byte_pos, data[byte_pos]);
						data[byte_pos] ^= 1 << bit_pos;
					} else {
						pr_debug("bitflip@oob[%d]=%x\n",
							(byte_pos - 512),
						     spare_ecc[byte_pos - 512]);
						spare_ecc[byte_pos - 512] ^=
							1 << bit_pos;
					}
				} else {
					dev_err(&info->pdev->dev,
						"invalid bit-flip @ %d:%d\n",
						byte_pos, bit_pos);
					err = -EBADMSG;
				}
			}
		}

		/* Update number of correctable errors */
		stat += err_vec[i].error_count;

		/* Update page data with sector size */
		data += ecc->size;
		spare_ecc += ecc->bytes;
	}

	return (err) ? err : stat;
}

/**
 * omap_write_page_bch - BCH ecc based write page function for entire page
 * @mtd:		mtd info structure
 * @chip:		nand chip info structure
 * @buf:		data buffer
 * @oob_required:	must write chip->oob_poi to OOB
 *
 * Custom write page method evolved to support multi sector writing in one shot
 */
static int omap_write_page_bch(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf/*, int oob_required*/)
{
	int i;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	/* Enable GPMC ecc engine */
	chip->ecc.hwctl(mtd, NAND_ECC_WRITE);

	/* Write data */
	chip->write_buf(mtd, buf, mtd->writesize);

	/* Update ecc vector from GPMC result registers */
	chip->ecc.calculate(mtd, buf, &ecc_calc[0]);
	for (i = 0; i < chip->ecc.total; i++){
		chip->oob_poi[eccpos[i]] = ecc_calc[i];
		}

	/* Write ecc vector to OOB area */
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
	return 0;
}

/**
 * omap_calcuate_ecc - Generate non-inverted ECC bytes.
 * @mtd: MTD device structure
 * @dat: The pointer to data on which ecc is computed
 * @ecc_code: The ecc_code buffer
 *
 * Using noninverted ECC can be considered ugly since writing a blank
 * page ie. padding will clear the ECC bytes. This is no problem as long
 * nobody is trying to write data on the seemingly unused page. Reading
 * an erased page will produce an ECC mismatch between generated and read
 * ECC bytes that has to be dealt with separately.
 */
static int omap_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				u_char *ecc_code)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	return gpmc_calculate_ecc(info->ecc_opt, info->gpmc_cs, dat, ecc_code);
}

/**
 * omap_enable_hwecc - This function enables the hardware ecc functionality
 * @mtd: MTD device structure
 * @mode: Read/Write mode
 */
static void omap_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	struct nand_chip *chip = mtd->priv;
	unsigned int dev_width = (chip->options & NAND_BUSWIDTH_16) ? 1 : 0;

	gpmc_enable_hwecc(info->ecc_opt, info->gpmc_cs, mode,
				dev_width, info->nand.ecc.size);
}

/**
 * omap_wait - wait until the command is done
 * @mtd: MTD device structure
 * @chip: NAND Chip structure
 *
 * Wait function is called during Program and erase operations and
 * the way it is called from MTD layer, we should wait till the NAND
 * chip is ready after the programming/erase operation has completed.
 *
 * Erase can take up to 400ms and program up to 20ms according to
 * general NAND and SmartMedia specs
 */
static int omap_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct nand_chip *this = mtd->priv;
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	unsigned long timeo = jiffies;
	int status, state = this->state;

	if (state == FL_ERASING)
		timeo += (HZ * 400) / 1000;
	else
		timeo += (HZ * 20) / 1000;

	gpmc_nand_write(info->gpmc_cs,
			GPMC_NAND_COMMAND, (NAND_CMD_STATUS & 0xFF));
	while (time_before(jiffies, timeo)) {
		status = gpmc_nand_read(info->gpmc_cs, GPMC_NAND_DATA);
		if (status & NAND_STATUS_READY)
			break;
		cond_resched();
	}

	status = gpmc_nand_read(info->gpmc_cs, GPMC_NAND_DATA);
	return status;
}

/**
 * omap_dev_ready - calls the platform specific dev_ready function
 * @mtd: MTD device structure
 */
static int omap_dev_ready(struct mtd_info *mtd)
{
	unsigned int val = 0;
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);

	val = gpmc_read_status(GPMC_GET_IRQ_STATUS);
	if ((val & 0x100) == 0x100) {
		/* Clear IRQ Interrupt */
		val |= 0x100;
		val &= ~(0x0);
		gpmc_cs_configure(info->gpmc_cs, GPMC_SET_IRQ_STATUS, val);
	} else {
		unsigned int cnt = 0;
		while (cnt++ < 0x1FF) {
			if  ((val & 0x100) == 0x100)
				return 0;
			val = gpmc_read_status(GPMC_GET_IRQ_STATUS);
		}
	}

	return 1;
}

static int __devinit omap_nand_probe(struct platform_device *pdev)
{
	struct omap_nand_info		*info;
	struct omap_nand_platform_data	*pdata;
	int				err;
	int				i, offset;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct omap_nand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	platform_set_drvdata(pdev, info);

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	info->pdev = pdev;

	info->gpmc_cs		= pdata->cs;
	info->phys_base		= pdata->phys_base;

	info->mtd.priv		= &info->nand;
	info->mtd.name		= dev_name(&pdev->dev);
	info->mtd.owner		= THIS_MODULE;
	info->ecc_opt		= pdata->ecc_opt;

	info->nand.options	= pdata->devsize;
	info->nand.options	|= NAND_SKIP_BBTSCAN;

	if (pdata->ctrlr_suspend)
		info->ctrlr_suspend = pdata->ctrlr_suspend;
	if (pdata->ctrlr_resume)
		info->ctrlr_resume = pdata->ctrlr_resume;

	/* NAND write protect off */
	gpmc_cs_configure(info->gpmc_cs, GPMC_CONFIG_WP, 0);

	if (!request_mem_region(info->phys_base, NAND_IO_SIZE,
				pdev->dev.driver->name)) {
		err = -EBUSY;
		goto out_free_info;
	}

	info->nand.IO_ADDR_R = ioremap(info->phys_base, NAND_IO_SIZE);
	if (!info->nand.IO_ADDR_R) {
		err = -ENOMEM;
		goto out_release_mem_region;
	}

	info->nand.controller = &info->controller;

	info->nand.IO_ADDR_W = info->nand.IO_ADDR_R;
	info->nand.cmd_ctrl  = omap_hwcontrol;

	/*
	 * If RDY/BSY line is connected to OMAP then use the omap ready
	 * funcrtion and the generic nand_wait function which reads the status
	 * register after monitoring the RDY/BSY line.Otherwise use a standard
	 * chip delay which is slightly more than tR (AC Timing) of the NAND
	 * device and read status register until you get a failure or success
	 */
	if (pdata->dev_ready) {
		info->nand.dev_ready = omap_dev_ready;
		info->nand.chip_delay = 0;
	} else {
		info->nand.waitfunc = omap_wait;
		info->nand.chip_delay = 50;
	}

	switch (pdata->xfer_type) {
	case NAND_OMAP_PREFETCH_POLLED:
		info->nand.read_buf   = omap_read_buf_pref;
		info->nand.write_buf  = omap_write_buf_pref;
		break;

	case NAND_OMAP_POLLED:
		if (info->nand.options & NAND_BUSWIDTH_16) {
			info->nand.read_buf   = omap_read_buf16;
			info->nand.write_buf  = omap_write_buf16;
		} else {
			info->nand.read_buf   = omap_read_buf8;
			info->nand.write_buf  = omap_write_buf8;
		}
		break;

	case NAND_OMAP_PREFETCH_DMA:
		err = omap_request_dma(OMAP24XX_DMA_GPMC, "NAND",
				omap_nand_dma_cb, &info->comp, &info->dma_ch);
		if (err < 0) {
			info->dma_ch = -1;
			dev_err(&pdev->dev, "DMA request failed!\n");
			goto out_release_mem_region;
		} else {
			omap_set_dma_dest_burst_mode(info->dma_ch,
					OMAP_DMA_DATA_BURST_16);
			omap_set_dma_src_burst_mode(info->dma_ch,
					OMAP_DMA_DATA_BURST_16);

			info->nand.read_buf   = omap_read_buf_dma_pref;
			info->nand.write_buf  = omap_write_buf_dma_pref;
		}
		break;

	case NAND_OMAP_PREFETCH_IRQ:
		err = request_irq(pdata->gpmc_irq,
				omap_nand_irq, IRQF_SHARED, "gpmc-nand", info);
		if (err) {
			dev_err(&pdev->dev, "requesting irq(%d) error:%d",
							pdata->gpmc_irq, err);
			goto out_release_mem_region;
		} else {
			info->gpmc_irq	     = pdata->gpmc_irq;
			info->nand.read_buf  = omap_read_buf_irq_pref;
			info->nand.write_buf = omap_write_buf_irq_pref;
		}
		break;

	default:
		dev_err(&pdev->dev,
			"xfer_type(%d) not supported!\n", pdata->xfer_type);
		err = -EINVAL;
		goto out_release_mem_region;
	}

	info->nand.verify_buf = omap_verify_buf;

	/* selsect the ecc type */
	if (pdata->ecc_opt == OMAP_ECC_HAMMING_CODE_DEFAULT)
		info->nand.ecc.mode = NAND_ECC_SOFT;
	else {
		if (pdata->ecc_opt == OMAP_ECC_BCH4_CODE_HW) {
			info->nand.ecc.bytes    = 4*7;
			info->nand.ecc.size     = 4*512;
		} else if (pdata->ecc_opt == OMAP_ECC_BCH8_CODE_HW) {
			info->nand.ecc.bytes     = OMAP_BCH8_ECC_SECT_BYTES;
			info->nand.ecc.size      = 512;
			info->nand.ecc.read_page = omap_read_page_bch;
			info->nand.ecc.write_page	= omap_write_page_bch;
		} else {
			info->nand.ecc.bytes    = 3;
			info->nand.ecc.size     = 512;
		}

		info->nand.ecc.calculate        = omap_calculate_ecc;
		info->nand.ecc.hwctl            = omap_enable_hwecc;
		info->nand.ecc.correct          = omap_correct_data;
		info->nand.ecc.mode             = NAND_ECC_HW;
	}

	/* DIP switches on some boards change between 8 and 16 bit
	 * bus widths for flash.  Try the other width if the first try fails.
	 */
	if (nand_scan_ident(&info->mtd, 1, NULL)) {
		info->nand.options ^= NAND_BUSWIDTH_16;
		if (nand_scan_ident(&info->mtd, 1, NULL)) {
			err = -ENXIO;
			goto out_release_mem_region;
		}
	}
	/*
	 * If ELM feature is used in OMAP NAND driver, then configure it
	 */
	if(pdata->elm_used){
		if (pdata->ecc_opt == OMAP_ECC_BCH8_CODE_HW){
				omap_configure_elm(&info->mtd, OMAP_BCH8_ECC, \
				 	info->mtd.writesize / info->nand.ecc.size, \
				 	info->nand.ecc.size, info->nand.ecc.bytes);
				 }
	}

	/* select ecc lyout */
	if (info->nand.ecc.mode != NAND_ECC_SOFT) {

		if (!(info->nand.options & NAND_BUSWIDTH_16))
			info->nand.badblock_pattern = &bb_descrip_flashbased;

		offset = JFFS2_CLEAN_MARKER_OFFSET;

		omap_oobinfo.eccbytes = info->nand.ecc.bytes *
			info->mtd.writesize / info->nand.ecc.size;

		if (pdata->ecc_opt == OMAP_ECC_HAMMING_CODE_HW_ROMCODE) {
			omap_oobinfo.oobfree->offset =
						offset + omap_oobinfo.eccbytes;
			omap_oobinfo.oobfree->length = info->mtd.oobsize -
				(offset + omap_oobinfo.eccbytes);
		} else if (pdata->ecc_opt == OMAP_ECC_BCH8_CODE_HW) {
			offset = BCH_ECC_POS; /* Synchronize with U-boot */

			omap_oobinfo.oobfree->offset = offset +
				omap_oobinfo.eccbytes;

			omap_oobinfo.oobfree->length = info->mtd.oobsize -
						offset - omap_oobinfo.eccbytes;
		} else {
			omap_oobinfo.oobfree->offset = offset;
			omap_oobinfo.oobfree->length = info->mtd.oobsize -
						offset - omap_oobinfo.eccbytes;
			/*
			offset is calculated considering the following :
			1) 12 bytes ECC for 512 byte access and 24 bytes ECC for
			256 byte access in OOB_64 can be supported
			2)Ecc bytes lie to the end of OOB area.
			3)Ecc layout must match with u-boot's ECC layout.
			*/
			offset = info->mtd.oobsize - MAX_HWECC_BYTES_OOB_64;
		}

		for (i = 0; i < omap_oobinfo.eccbytes; i++)
			omap_oobinfo.eccpos[i] = i+offset;

		info->nand.ecc.layout = &omap_oobinfo;
	}

	/* second phase scan */
	if (nand_scan_tail(&info->mtd)) {
		err = -ENXIO;
		goto out_release_mem_region;
	}

	/* Fix sub page size to page size for HW ECC */
	if (info->nand.ecc.mode == NAND_ECC_HW) {
		/*
		 * For HW ECC, subpage size set to page size
		 * as subpage operations not supporting.
		 */
		info->mtd.subpage_sft = 0;
		info->nand.subpagesize = info->mtd.writesize >>
			info->mtd.subpage_sft;
	}

	mtd_device_parse_register(&info->mtd, NULL, 0,
			pdata->parts, pdata->nr_parts);

	platform_set_drvdata(pdev, &info->mtd);

	return 0;

out_release_mem_region:
	release_mem_region(info->phys_base, NAND_IO_SIZE);
out_free_info:
	kfree(info);

	return err;
}

static int omap_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);

	platform_set_drvdata(pdev, NULL);
	if (info->dma_ch != -1)
		omap_free_dma(info->dma_ch);

	if (info->gpmc_irq)
		free_irq(info->gpmc_irq, info);

	/* Release NAND device, its internal structures and partitions */
	nand_release(&info->mtd);
	iounmap(info->nand.IO_ADDR_R);
	release_mem_region(info->phys_base, NAND_IO_SIZE);
	kfree(info);
	return 0;
}

#ifdef CONFIG_PM
static int omap_nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);

	mtd->suspend(mtd);

	if (info->ctrlr_suspend)
		info->ctrlr_suspend();

	return 0;
}

static int omap_nand_resume(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);

	if (info->ctrlr_resume)
		info->ctrlr_resume();

	return 0;
}
#endif

static struct platform_driver omap_nand_driver = {
	.probe		= omap_nand_probe,
	.remove		= omap_nand_remove,
#ifdef CONFIG_PM
	.suspend	= omap_nand_suspend,
	.resume		= omap_nand_resume,
#endif
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init omap_nand_init(void)
{
	pr_info("%s driver initializing\n", DRIVER_NAME);

	return platform_driver_register(&omap_nand_driver);
}

static void __exit omap_nand_exit(void)
{
	platform_driver_unregister(&omap_nand_driver);
}

module_init(omap_nand_init);
module_exit(omap_nand_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Glue layer for NAND flash on TI OMAP boards");
