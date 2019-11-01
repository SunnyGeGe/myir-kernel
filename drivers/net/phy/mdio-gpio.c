/*
 * GPIO based MDIO bitbang driver.
 * Supports OpenFirmware.
 *
 * Copyright (c) 2008 CSE Semaphore Belgium.
 *  by Laurent Pinchart <laurentp@cse-semaphore.com>
 *
 * Copyright (C) 2008, Paulius Zaleckas <paulius.zaleckas@teltonika.lt>
 *
 * Based on earlier work by
 *
 * Copyright (c) 2003 Intracom S.A.
 *  by Pantelis Antoniou <panto@intracom.gr>
 *
 * 2005 (c) MontaVista Software, Inc.
 * Vitaly Bordug <vbordug@ru.mvista.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mdio-gpio.h>

#include <linux/of_gpio.h>
#include <linux/of_mdio.h>

#define PHY_REG_SIZE 4096

struct mdio_gpio_info {
	struct mdiobb_ctrl ctrl;
	int mdc, mdio, mdo;
	int mdc_active_low, mdio_active_low, mdo_active_low;
};

static void *mdio_gpio_of_get_data(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mdio_gpio_platform_data *pdata;
	enum of_gpio_flags flags;
	int ret;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	ret = of_get_gpio_flags(np, 0, &flags);
	if (ret < 0)
		return NULL;

	pdata->mdc = ret;
	pdata->mdc_active_low = flags & OF_GPIO_ACTIVE_LOW;

	ret = of_get_gpio_flags(np, 1, &flags);
	if (ret < 0)
		return NULL;
	pdata->mdio = ret;
	pdata->mdio_active_low = flags & OF_GPIO_ACTIVE_LOW;

	ret = of_get_gpio_flags(np, 2, &flags);
	if (ret > 0) {
		pdata->mdo = ret;
		pdata->mdo_active_low = flags & OF_GPIO_ACTIVE_LOW;
	}

	return pdata;
}

static void mdio_dir(struct mdiobb_ctrl *ctrl, int dir)
{
	struct mdio_gpio_info *bitbang =
		container_of(ctrl, struct mdio_gpio_info, ctrl);

	if (bitbang->mdo) {
		/* Separate output pin. Always set its value to high
		 * when changing direction. If direction is input,
		 * assume the pin serves as pull-up. If direction is
		 * output, the default value is high.
		 */
		gpio_set_value_cansleep(bitbang->mdo,
					1 ^ bitbang->mdo_active_low);
		return;
	}

	if (dir)
		gpio_direction_output(bitbang->mdio,
				      1 ^ bitbang->mdio_active_low);
	else
		gpio_direction_input(bitbang->mdio);
}

static int mdio_get(struct mdiobb_ctrl *ctrl)
{
	struct mdio_gpio_info *bitbang =
		container_of(ctrl, struct mdio_gpio_info, ctrl);

	return gpio_get_value_cansleep(bitbang->mdio) ^
		bitbang->mdio_active_low;
}

static void mdio_set(struct mdiobb_ctrl *ctrl, int what)
{
	struct mdio_gpio_info *bitbang =
		container_of(ctrl, struct mdio_gpio_info, ctrl);

	if (bitbang->mdo)
		gpio_set_value_cansleep(bitbang->mdo,
					what ^ bitbang->mdo_active_low);
	else
		gpio_set_value_cansleep(bitbang->mdio,
					what ^ bitbang->mdio_active_low);
}

static void mdc_set(struct mdiobb_ctrl *ctrl, int what)
{
	struct mdio_gpio_info *bitbang =
		container_of(ctrl, struct mdio_gpio_info, ctrl);

	gpio_set_value_cansleep(bitbang->mdc, what ^ bitbang->mdc_active_low);
}

static struct mdiobb_ops mdio_gpio_ops = {
	.owner = THIS_MODULE,
	.set_mdc = mdc_set,
	.set_mdio_dir = mdio_dir,
	.set_mdio_data = mdio_set,
	.get_mdio_data = mdio_get,
};

static int gpio_mdio_reset(struct mii_bus *bus)
{
	//struct mdiobb_ctrl *ctrl = bus->priv;
	printk("=== gpio_mdio_reset===\n");
	//mdio_set(ctrl, 1);
	//mdc_set(ctrl, 1);

	//bus->phy_mask = 0xffffffee;
	return 0;
}

static struct mii_bus *mdio_gpio_bus_init(struct device *dev,
					  struct mdio_gpio_platform_data *pdata,
					  int bus_id)
{
	struct mii_bus *new_bus;
	struct mdio_gpio_info *bitbang;
	int i;

	bitbang = devm_kzalloc(dev, sizeof(*bitbang), GFP_KERNEL);
	if (!bitbang)
		goto out;

	bitbang->ctrl.ops = &mdio_gpio_ops;
	bitbang->ctrl.reset = &gpio_mdio_reset;
	bitbang->mdc = pdata->mdc;
	bitbang->mdc_active_low = pdata->mdc_active_low;
	bitbang->mdio = pdata->mdio;
	bitbang->mdio_active_low = pdata->mdio_active_low;
	bitbang->mdo = pdata->mdo;
	bitbang->mdo_active_low = pdata->mdo_active_low;

	new_bus = alloc_mdio_bitbang(&bitbang->ctrl);
	if (!new_bus)
		goto out;

	new_bus->name = "GPIO Bitbanged MDIO",

	new_bus->phy_mask = pdata->phy_mask;
	new_bus->irq = pdata->irqs;
	new_bus->parent = dev;

	if (new_bus->phy_mask == ~0)
		goto out_free_bus;

	for (i = 0; i < PHY_MAX_ADDR; i++)
		if (!new_bus->irq[i])
			new_bus->irq[i] = PHY_POLL;

//	if (bus_id != -1)
//		snprintf(new_bus->id, MII_BUS_ID_SIZE, "mdio", bus_id);
//	else
		strncpy(new_bus->id, "mdio", MII_BUS_ID_SIZE);

	if (devm_gpio_request(dev, bitbang->mdc, "mdc"))
		goto out_free_bus;

	if (devm_gpio_request(dev, bitbang->mdio, "mdio"))
		goto out_free_bus;

	if (bitbang->mdo) {
		if (devm_gpio_request(dev, bitbang->mdo, "mdo"))
			goto out_free_bus;
		gpio_direction_output(bitbang->mdo, 1);
		gpio_direction_input(bitbang->mdio);
	}

	gpio_direction_output(bitbang->mdc, 0);

	dev_set_drvdata(dev, new_bus);

	return new_bus;

out_free_bus:
	free_mdio_bitbang(new_bus);
out:
	return NULL;
}

static void mdio_gpio_bus_deinit(struct device *dev)
{
	struct mii_bus *bus = dev_get_drvdata(dev);

	free_mdio_bitbang(bus);
}

static void mdio_gpio_bus_destroy(struct device *dev)
{
	struct mii_bus *bus = dev_get_drvdata(dev);

	mdiobus_unregister(bus);
	mdio_gpio_bus_deinit(dev);
}

static ssize_t mdio_gpio_read(struct mii_bus *bus, char *buf, loff_t off)
{
	int value = 0;
	int phy = (int)(off/128);
	u32 regnum = (u32)((off - phy*128)/4);
	
	value = mdiobus_read(bus, phy, regnum);
	memcpy(buf, &value, sizeof(int));

	printk("read: phy=%d, regnum=%d, value= 0x%x\r\n", phy, regnum, value);
	return sizeof(value);
}

static ssize_t mdio_gpio_write(struct mii_bus *bus, char *buf, loff_t off)
{
	int ret = -1;
	int value = 0;
	int phy = (int)(off/128);
	u32 regnum = (u32)((off - phy*128)/4);
	
	memcpy(&value, buf, sizeof(int));

	printk("write: phy=%d, regnum=%d, 0x%x 0x%x 0x%x 0x%x\r\n", phy, regnum, buf[3], buf[2], buf[1], buf[0]);
	ret = mdiobus_write(bus, phy, regnum, value);
	printk("write: phy=%d, regnum=%d, value= 0x%x\r\n", phy, regnum, value);

	return sizeof(value);
}

 static ssize_t mdio_gpio_sysfs_read(struct file *filp,
						  struct kobject *kobj,
						  struct bin_attribute *bin,
						  char *buf, loff_t off,
						  size_t count)
 {
	 struct device *dev = container_of(kobj,struct device,kobj);
	 struct mii_bus *mii_bus = dev_get_drvdata(dev);
	if (off > PHY_REG_SIZE-4)
		return 0;
	if (off + count > PHY_REG_SIZE-4)
		count = PHY_REG_SIZE - off;

	return mdio_gpio_read(mii_bus, buf, off);
 }

 static ssize_t mdio_gpio_sysfs_write(struct file *filp,
						   struct kobject *kobj,
						   struct bin_attribute *bin,
						   char *buf, loff_t off,
						   size_t count)
 {
	 struct device *dev = container_of(kobj,struct device,kobj);
	 struct mii_bus *mii_bus = dev_get_drvdata(dev);
	 if (off > PHY_REG_SIZE-4)
		 return 0;
	 if (off + count > PHY_REG_SIZE-4)
		 count = PHY_REG_SIZE - off;
	 return mdio_gpio_write(mii_bus, buf, off);
 }
 
 static struct bin_attribute mdio_gpio_sysfs_info_attr = {
	 .attr = {
		 .name = "mdio",
		 .mode = S_IRUSR | S_IWUSR,
	 }, 
	 .size = PHY_REG_SIZE,
	 .read = mdio_gpio_sysfs_read,
	 .write = mdio_gpio_sysfs_write,
 };

static int mdio_gpio_probe(struct platform_device *pdev)
{
	struct mdio_gpio_platform_data *pdata;
	struct mii_bus *new_bus;
	int ret, bus_id, addr;
    struct phy_device *phy;


	if (pdev->dev.of_node) {
		pdata = mdio_gpio_of_get_data(pdev);
		bus_id = of_alias_get_id(pdev->dev.of_node, "mdio-gpio");
		if (bus_id < 0) {
			dev_warn(&pdev->dev, "failed to get alias id\n");
			bus_id = 0;
		}
	} else {
		pdata = dev_get_platdata(&pdev->dev);
		bus_id = pdev->id;
	}

	if (!pdata)
		return -ENODEV;
	pdata->phy_mask = 0xffffffae;
	new_bus = mdio_gpio_bus_init(&pdev->dev, pdata, bus_id);
	if (!new_bus)
		return -ENODEV;

	if (pdev->dev.of_node)
		ret = of_mdiobus_register(new_bus, pdev->dev.of_node);
	else
		ret = mdiobus_register(new_bus);

        /* scan and dump the bus */
    for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
        phy = new_bus->phy_map[addr];
        if (phy) {
            dev_err(&pdev->dev, "phy[%d]: device %s, driver %s\n",
                                                 phy->addr, dev_name(&phy->dev),
                                                 phy->drv ? phy->drv->name : "unknown");
                    
        }
            
    }

	if (ret)
		mdio_gpio_bus_deinit(&pdev->dev);
	else
	{
#if 0
		int value;
		unsigned short temp = 0;
	// Read CHIP ID	
		value = mdiobus_read(new_bus, 4, 2);
		printk("=== read chip id =  0x%x \r\n", value);
		value = mdiobus_read(new_bus, 4, 3);
		printk("=== read phy 20 reg 4 =  0x%x \r\n", value);
		value = mdiobus_read(new_bus, 0, 2);
		printk("=== read phy 20 reg 4 =  0x%x \r\n", value);
		value = mdiobus_read(new_bus, 0, 3);
		printk("=== read phy 21 reg 3 =  0x%x \r\n", value);
#endif
		ret = sysfs_create_bin_file(&pdev->dev.kobj, &mdio_gpio_sysfs_info_attr);
	}

	return ret;
}

static int mdio_gpio_remove(struct platform_device *pdev)
{
	mdio_gpio_bus_destroy(&pdev->dev);

	return 0;
}

static const struct of_device_id mdio_gpio_of_match[] = {
	{ .compatible = "virtual,mdio-gpio", },
	{ /* sentinel */ }
};

static struct platform_driver mdio_gpio_driver = {
	.probe = mdio_gpio_probe,
	.remove = mdio_gpio_remove,
	.driver		= {
		.name	= "mdio-gpio",
		.of_match_table = mdio_gpio_of_match,
	},
};

module_platform_driver(mdio_gpio_driver);

MODULE_ALIAS("platform:mdio-gpio");
MODULE_AUTHOR("Laurent Pinchart, Paulius Zaleckas");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Generic driver for MDIO bus emulation using GPIO");
