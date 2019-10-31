/*
 * OMAP2 Error Location Module
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#include <plat/elm.h>

#define ELM_SYSCONFIG			0x010
#define ELM_SYSSTATUS			0x014
#define ELM_IRQSTATUS			0x018
#define ELM_IRQENABLE			0x01c
#define ELM_LOCATION_CONFIG		0x020
#define ELM_PAGE_CTRL			0x080
#define ELM_SYNDROME_FRAGMENT_0		0x400
#define ELM_SYNDROME_FRAGMENT_1		0x404
#define ELM_SYNDROME_FRAGMENT_2		0x408
#define ELM_SYNDROME_FRAGMENT_3		0x40c
#define ELM_SYNDROME_FRAGMENT_4		0x410
#define ELM_SYNDROME_FRAGMENT_5		0x414
#define ELM_SYNDROME_FRAGMENT_6		0x418
#define ELM_LOCATION_STATUS		0x800
#define ELM_ERROR_LOCATION_0		0x880
#define ELM_ERROR_LOCATION_1		0x884
#define ELM_ERROR_LOCATION_2		0x888
#define ELM_ERROR_LOCATION_3		0x88c
#define ELM_ERROR_LOCATION_4		0x890
#define ELM_ERROR_LOCATION_5		0x894
#define ELM_ERROR_LOCATION_6		0x898
#define ELM_ERROR_LOCATION_7		0x89c
#define ELM_ERROR_LOCATION_8		0x8a0
#define ELM_ERROR_LOCATION_9		0x8a4
#define ELM_ERROR_LOCATION_10		0x8a8
#define ELM_ERROR_LOCATION_11		0x8ac
#define ELM_ERROR_LOCATION_12		0x8b0
#define ELM_ERROR_LOCATION_13		0x8b4
#define ELM_ERROR_LOCATION_14		0x8b8
#define ELM_ERROR_LOCATION_15		0x8bc

/* ELM System Configuration Register */
#define ELM_SYSCONFIG_SOFTRESET		BIT(1)
#define ELM_SYSCONFIG_SIDLE_MASK	(3 << 3)
#define ELM_SYSCONFIG_SMART_IDLE	(2 << 3)

/* ELM System Status Register */
#define ELM_SYSSTATUS_RESETDONE		BIT(0)

/* ELM Interrupt Status Register */
#define INTR_STATUS_PAGE_VALID		BIT(8)
#define INTR_STATUS_LOC_VALID_7		BIT(7)
#define INTR_STATUS_LOC_VALID_6		BIT(6)
#define INTR_STATUS_LOC_VALID_5		BIT(5)
#define INTR_STATUS_LOC_VALID_4		BIT(4)
#define INTR_STATUS_LOC_VALID_3		BIT(3)
#define INTR_STATUS_LOC_VALID_2		BIT(2)
#define INTR_STATUS_LOC_VALID_1		BIT(1)
#define INTR_STATUS_LOC_VALID_0		BIT(0)

/* ELM Interrupt Enable Register */
#define INTR_EN_PAGE_MASK		BIT(8)
#define INTR_EN_LOCATION_MASK_7		BIT(7)
#define INTR_EN_LOCATION_MASK_6		BIT(6)
#define INTR_EN_LOCATION_MASK_5		BIT(5)
#define INTR_EN_LOCATION_MASK_4		BIT(4)
#define INTR_EN_LOCATION_MASK_3		BIT(3)
#define INTR_EN_LOCATION_MASK_2		BIT(2)
#define INTR_EN_LOCATION_MASK_1		BIT(1)
#define INTR_EN_LOCATION_MASK_0		BIT(0)

/* ELM Location Configuration Register */
#define ECC_SIZE_MASK			(0x7ff << 16)
#define ECC_BCH_LEVEL_MASK		(0x3 << 0)
#define ECC_BCH4_LEVEL			(0x0 << 0)
#define ECC_BCH8_LEVEL			(0x1 << 0)
#define ECC_BCH16_LEVEL			(0x2 << 0)

/* ELM Page Definition Register */
#define PAGE_MODE_SECTOR_7		BIT(7)
#define PAGE_MODE_SECTOR_6		BIT(6)
#define PAGE_MODE_SECTOR_5		BIT(5)
#define PAGE_MODE_SECTOR_4		BIT(4)
#define PAGE_MODE_SECTOR_3		BIT(3)
#define PAGE_MODE_SECTOR_2		BIT(2)
#define PAGE_MODE_SECTOR_1		BIT(1)
#define PAGE_MODE_SECTOR_0		BIT(0)

/* ELM syndrome */
#define ELM_SYNDROME_VALID		BIT(16)

/* ELM_LOCATION_STATUS Register */
#define ECC_CORRECTABLE_MASK		BIT(8)
#define ECC_NB_ERRORS_MASK		(0x1f << 0)

/*  ELM_ERROR_LOCATION_0-15 Registers */
#define ECC_ERROR_LOCATION_MASK		(0x1fff << 0)

#define OMAP_ECC_SIZE			(0x7ff)

#define DRIVER_NAME	"omap2_elm"

#define SYNDROME_FRAGMENT_REG_SIZE	0x40
#define ERROR_LOCATION_SIZE		0x100

struct elm_registers {
	u32 elm_irqenable;
	u32 elm_sysconfig;
	u32 elm_location_config;
	u32 elm_page_ctrl;
	u32 elm_syndrome_fragment_6[ERROR_VECTOR_MAX];
	u32 elm_syndrome_fragment_5[ERROR_VECTOR_MAX];
	u32 elm_syndrome_fragment_4[ERROR_VECTOR_MAX];
	u32 elm_syndrome_fragment_3[ERROR_VECTOR_MAX];
	u32 elm_syndrome_fragment_2[ERROR_VECTOR_MAX];
	u32 elm_syndrome_fragment_1[ERROR_VECTOR_MAX];
	u32 elm_syndrome_fragment_0[ERROR_VECTOR_MAX];
};


static void  __iomem *elm_base;
static struct list_head list;
static struct completion elm_completion;
static struct mtd_info *mtd;
static	enum omap_bch_ecc g_bch_type;
static	struct elm_registers elm_regs;
static	int g_ecc_steps;
static	int g_ecc_syndrome_size;

static LIST_HEAD(elm_devices);

static void elm_write_reg(int idx, u32 val)
{
	writel(val, elm_base + idx);
}

static u32 elm_read_reg(int idx)
{
	return readl(elm_base + idx);
}

/**
 * omap_elm_config - Configure ELM for BCH ECC scheme
 * @bch_type:	type of BCH ECC scheme
 */
void omap_elm_config(int bch_type)
{
	u32 reg_val;
	u32 buffer_size = OMAP_ECC_SIZE;

	reg_val = (bch_type & ECC_BCH_LEVEL_MASK) | (buffer_size << 16);
	elm_write_reg(ELM_LOCATION_CONFIG, reg_val);


}

/**
 * omap_configure_elm - Configure ELM for BCH ECC scheme
 * @mtd_info:	mtd info structure
 * @bch_type:	type of BCH ECC scheme
 *
 * Configures the ELM module to support BCH error correction
 */
void omap_configure_elm(struct mtd_info *mtd_info, int bch_type,
	int ecc_steps, int ecc_step_size, int ecc_syndrome_size)
{
	/* ELM cannot detect ECC errors for chunks > 1KB */
	if (ecc_step_size > ((OMAP_ECC_SIZE + 1) / 2)) {
		printk("unsupported config ecc-size=%d\n", ecc_step_size);
	}
	/* ELM support 8 error syndrome process */
	if (ecc_steps > ERROR_VECTOR_MAX) {
		printk("unsupported config ecc-step=%d\n", ecc_steps);
	}
	omap_elm_config(bch_type);
	mtd = mtd_info;
	g_bch_type		= bch_type;
	g_ecc_steps		= ecc_steps;
	g_ecc_syndrome_size	= ecc_syndrome_size;
}
EXPORT_SYMBOL(omap_configure_elm);

/**
 * elm_configure_page_mode - Enable/Disable page mode
 * @info:	elm info
 * @index:	index number of syndrome fragment vector
 * @enable:	enable/disable flag for page mode
 *
 * Enable page mode for syndrome fragment index
 */
static void elm_configure_page_mode(int index,
		bool enable)
{
	u32 reg_val;

	reg_val = elm_read_reg(ELM_PAGE_CTRL);
	if (enable)
		reg_val |= BIT(index);	/* enable page mode */
	else
		reg_val &= ~BIT(index);	/* disable page mode */

	elm_write_reg(ELM_PAGE_CTRL, reg_val);
}

/**
 * elm_load_syndrome - Load ELM syndrome reg
 * @info:	elm info
 * @err_vec:	elm error vectors
 * @ecc:	buffer with calculated ecc
 *
 * Load syndrome fragment registers with calculated ecc in reverse order.
 */
static void elm_load_syndrome(struct elm_errorvec *err_vec, u8 *ecc)
{
	int i, offset;
	u32 val;

	for (i = 0; i < 4; i++) {

		/* Check error reported */
		if (err_vec[i].error_reported) {
			elm_configure_page_mode(i, true);
			offset = ELM_SYNDROME_FRAGMENT_0 +
				SYNDROME_FRAGMENT_REG_SIZE * i;
			switch (g_bch_type) {
			case OMAP_BCH8_ECC:
				/* syndrome fragment 0 = ecc[9-12B] */
				val = cpu_to_be32(*(u32 *) &ecc[9]);
				elm_write_reg(offset, val);
				/* syndrome fragment 1 = ecc[5-8B] */
				offset += 4;
				val = cpu_to_be32(*(u32 *) &ecc[5]);
				elm_write_reg(offset, val);
				/* syndrome fragment 2 = ecc[1-4B] */
				offset += 4;
				val = cpu_to_be32(*(u32 *) &ecc[1]);
				elm_write_reg(offset, val);
				/* syndrome fragment 3 = ecc[0B] */
				offset += 4;
				val = ecc[0];
				elm_write_reg(offset, val);
				break;
			case OMAP_BCH4_ECC:
				/* syndrome fragment 0 = ecc[20-52b] bits */
				val = (cpu_to_be32(*(u32 *) &ecc[3]) >> 4) |
					((ecc[2] & 0xf) << 28);
				elm_write_reg(offset, val);

				/* syndrome fragment 1 = ecc[0-20b] bits */
				offset += 4;
				val = cpu_to_be32(*(u32 *) &ecc[0]) >> 12;
				elm_write_reg(offset, val);
				break;
			default:
				pr_err("invalid config bch_type\n");
			}
		}

		/* Update ecc pointer with ecc byte size */
		ecc += g_ecc_syndrome_size;
	}
}

/**
 * elm_start_processing - start elm syndrome processing
 * @info:	elm info
 * @err_vec:	elm error vectors
 *
 * Set syndrome valid bit for syndrome fragment registers for which
 * elm syndrome fragment registers are loaded. This enables elm module
 * to start processing syndrome vectors.
 */
static void elm_start_processing(struct elm_errorvec *err_vec)
{
	int i, offset;
	u32 reg_val;

	/*
	 * Set syndrome vector valid, so that ELM module
	 * will process it for vectors error is reported
	 */
	for (i = 0; i < 4; i++) {
		if (err_vec[i].error_reported) {
			offset = ELM_SYNDROME_FRAGMENT_6 +
				SYNDROME_FRAGMENT_REG_SIZE * i;
			reg_val = elm_read_reg(offset);
			reg_val |= ELM_SYNDROME_VALID;
			elm_write_reg(offset, reg_val);
		}
	}
}

/**
 * elm_error_correction - locate correctable error position
 * @err_vec:	elm error vectors
 *
 * On completion of processing by elm module, error location status
 * register updated with correctable/uncorrectable error information.
 * In case of correctable errors, number of errors located from
 * elm location status register & read the positions from
 * elm error location register.
 */
static void elm_error_correction(struct elm_errorvec *err_vec)
{
	int i, j, errors = 0;
	int offset;
	u32 reg_val;

	for (i = 0; i < 4; i++) {

		/* Check error reported */
		if (err_vec[i].error_reported) {
			offset = ELM_LOCATION_STATUS + ERROR_LOCATION_SIZE * i;
			reg_val = elm_read_reg(offset);

			/* Check correctable error or not */
			if (reg_val & ECC_CORRECTABLE_MASK) {
				offset = ELM_ERROR_LOCATION_0 +
					ERROR_LOCATION_SIZE * i;

				/* Read count of correctable errors */
				err_vec[i].error_count = reg_val &
					ECC_NB_ERRORS_MASK;

				/* Update the error locations in error vector */
				for (j = 0; j < err_vec[i].error_count; j++) {

					reg_val = elm_read_reg(offset);
					err_vec[i].error_loc[j] = reg_val &
						ECC_ERROR_LOCATION_MASK;

					/* Update error location register */
					offset += 4;
				}

				errors += err_vec[i].error_count;
			} else {
				err_vec[i].error_uncorrectable = true;
			}

			/* Clearing interrupts for processed error vectors */
			elm_write_reg(ELM_IRQSTATUS, BIT(i));

			/* Disable page mode */
			elm_configure_page_mode(i, false);
		}
	}
}

/**
 * omap_elm_decode_bch_error - Locate error position
 * @ecc_calc:	calculated ECC bytes from GPMC
 * @err_vec:	elm error vectors
 *
 * Called with one or more error reported vectors & vectors with
 * error reported is updated in err_vec[].error_reported
 */
int omap_elm_decode_bch_error(int bch_type , char *ecc_calc,
		struct elm_errorvec *err_vec)
{
	u32 reg_val;

	/* Enable page mode interrupt */
	reg_val = elm_read_reg(ELM_IRQSTATUS);
	elm_write_reg(ELM_IRQSTATUS, reg_val & INTR_STATUS_PAGE_VALID);
	elm_write_reg(ELM_IRQENABLE, INTR_EN_PAGE_MASK);
	/* Load valid ecc byte to syndrome fragment register */
	elm_load_syndrome(err_vec, ecc_calc);

	/* Enable syndrome processing for which syndrome fragment is updated */
	elm_start_processing(err_vec);

	/* Wait for ELM module to finish locating error correction */
	wait_for_completion(&elm_completion);

	/* Disable page mode interrupt */
	reg_val = elm_read_reg(ELM_IRQENABLE);
	elm_write_reg(ELM_IRQENABLE, reg_val & ~INTR_EN_PAGE_MASK);
	elm_error_correction(err_vec);
	
	return 0;
}
EXPORT_SYMBOL(omap_elm_decode_bch_error);

static irqreturn_t omap_elm_isr(int this_irq, void *dev_id)
{
	u32 reg_val;

	reg_val = elm_read_reg(ELM_IRQSTATUS);

	/* All error vectors processed */
	if (reg_val & INTR_STATUS_PAGE_VALID) {
		elm_write_reg(ELM_IRQSTATUS,
				reg_val & INTR_STATUS_PAGE_VALID);
		complete(&elm_completion);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int omap_elm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res, *irq;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (irq == NULL)
	{
		dev_err(&pdev->dev, "no irq resource defined\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL)
	{
			dev_err(&pdev->dev, "no mem resource defined\n");
			return -EINVAL;
	}

	if (!request_mem_region(res->start, resource_size(res),
				dev_name(&pdev->dev)))
		return -EBUSY;

	elm_base =  ioremap(res->start, resource_size(res));

	if (!elm_base) {
		dev_dbg(&pdev->dev, "can't ioremap\n");
		ret = -ENOMEM;
		goto err_remap;
	}

	pm_runtime_enable(&pdev->dev);
	if (pm_runtime_get_sync(&pdev->dev)) {
		ret = -EINVAL;
		dev_dbg(&pdev->dev, "can't enable clock\n");
		goto err_clk;
	}

	ret = request_irq(irq->start, omap_elm_isr, 0, pdev->name,
			&pdev->dev);

	if (ret) {
		pr_err("failure requesting irq %i\n", irq->start);
		goto err_irq;
	}

	init_completion(&elm_completion);
	INIT_LIST_HEAD(&list);
	list_add(&list, &elm_devices);
	return ret;

err_irq:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
err_clk:
	iounmap(elm_base);
err_remap:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

static int omap_elm_remove(struct platform_device *pdev)
{
	struct resource *res = NULL;

	iounmap(elm_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	free_irq(res->start, &pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}


#ifdef CONFIG_PM
/**
 * elm_context_save
 * saves ELM configurations to preserve them across Hardware powered-down
 */
static int elm_context_save()
{

	u32 offset = 0, i;

	elm_regs.elm_irqenable       = elm_read_reg(ELM_IRQENABLE);
	elm_regs.elm_sysconfig       = elm_read_reg(ELM_SYSCONFIG);
	elm_regs.elm_location_config = elm_read_reg(ELM_LOCATION_CONFIG);
	elm_regs.elm_page_ctrl       = elm_read_reg(ELM_PAGE_CTRL);
	for (i = 0; i < ERROR_VECTOR_MAX; i++) {
		offset = i * SYNDROME_FRAGMENT_REG_SIZE;
		switch (g_bch_type) {
		case OMAP_BCH16_ECC:
			elm_regs.elm_syndrome_fragment_6[i] = elm_read_reg(ELM_SYNDROME_FRAGMENT_6 + offset);
			elm_regs.elm_syndrome_fragment_5[i] = elm_read_reg(ELM_SYNDROME_FRAGMENT_5 + offset);
			elm_regs.elm_syndrome_fragment_4[i] = elm_read_reg(ELM_SYNDROME_FRAGMENT_4 + offset);
		case OMAP_BCH8_ECC:
			elm_regs.elm_syndrome_fragment_3[i] = elm_read_reg(ELM_SYNDROME_FRAGMENT_3 + offset);
			elm_regs.elm_syndrome_fragment_2[i] = elm_read_reg(ELM_SYNDROME_FRAGMENT_2 + offset);
		case OMAP_BCH4_ECC:
			elm_regs.elm_syndrome_fragment_1[i] = elm_read_reg(ELM_SYNDROME_FRAGMENT_1 + offset);
			elm_regs.elm_syndrome_fragment_0[i] = elm_read_reg(ELM_SYNDROME_FRAGMENT_0 + offset);
			break;
		default:
			return -EINVAL;
		}
		/* ELM SYNDROME_VALID bit in SYNDROME_FRAGMENT_6[] needs
		 * to be saved for all BCH schemes*/
		elm_regs.elm_syndrome_fragment_6[i] = elm_read_reg(ELM_SYNDROME_FRAGMENT_6 + offset);
	}
	return 0;
}

/**
 * elm_context_restore
 * writes configurations saved duing power-down back into ELM registers
 */
static int elm_context_restore()
{
	u32 offset = 0, i;

	elm_write_reg(ELM_IRQENABLE,	 elm_regs.elm_irqenable);
	elm_write_reg(ELM_SYSCONFIG,	 elm_regs.elm_sysconfig);
	elm_write_reg(ELM_LOCATION_CONFIG, elm_regs.elm_location_config);
	elm_write_reg(ELM_PAGE_CTRL,	 elm_regs.elm_page_ctrl);
	for (i = 0; i < ERROR_VECTOR_MAX; i++) {
		offset = i * SYNDROME_FRAGMENT_REG_SIZE;
		switch (g_bch_type) {
		case OMAP_BCH16_ECC:
			elm_write_reg(ELM_SYNDROME_FRAGMENT_6 + offset,
					elm_regs.elm_syndrome_fragment_6[i]);
			elm_write_reg(ELM_SYNDROME_FRAGMENT_5 + offset,
					elm_regs.elm_syndrome_fragment_5[i]);
			elm_write_reg(ELM_SYNDROME_FRAGMENT_4 + offset,
					elm_regs.elm_syndrome_fragment_4[i]);
		case OMAP_BCH8_ECC:
			elm_write_reg(ELM_SYNDROME_FRAGMENT_3 + offset,
					elm_regs.elm_syndrome_fragment_3[i]);
			elm_write_reg(ELM_SYNDROME_FRAGMENT_2 + offset,
					elm_regs.elm_syndrome_fragment_2[i]);
		case OMAP_BCH4_ECC:
			elm_write_reg(ELM_SYNDROME_FRAGMENT_1 + offset,
					elm_regs.elm_syndrome_fragment_1[i]);
			elm_write_reg(ELM_SYNDROME_FRAGMENT_0 + offset,
					elm_regs.elm_syndrome_fragment_0[i]);
			break;
		default:
			return -EINVAL;
		}
		/* ELM_SYNDROME_VALID bit to be set in last to trigger FSM */
		elm_write_reg(ELM_SYNDROME_FRAGMENT_6 + offset,
					elm_regs.elm_syndrome_fragment_6[i] &
							 ELM_SYNDROME_VALID);
	}
	return 0;
}

static int omap_elm_suspend(struct platform_device *pdev)
{
	elm_context_save();
	pm_runtime_put_sync(&pdev->dev);
	return 0;
}

static int omap_elm_resume(struct platform_device *pdev)
{
	pm_runtime_get_sync(&pdev->dev);
	elm_context_restore();
	return 0;
}
#endif

static struct platform_driver omap_elm_driver = {
	.probe		= omap_elm_probe,
	.remove		= omap_elm_remove,
#ifdef CONFIG_PM
	.suspend	= omap_elm_suspend,
	.resume		= omap_elm_resume,
#endif
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init omap_elm_init(void)
{

	return platform_driver_register(&omap_elm_driver);
}

static void __exit omap_elm_exit(void)
{
	platform_driver_unregister(&omap_elm_driver);
}

module_init(omap_elm_init);
module_exit(omap_elm_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");
