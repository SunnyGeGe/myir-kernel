/*
 * DaVinci MDIO Module driver
 *
 * Copyright (C) 2010 Texas Instruments.
 *
 * Shamelessly ripped out of davinci_emac.c, original copyrights follow:
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * ---------------------------------------------------------------------------
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ---------------------------------------------------------------------------
 */
#ifndef __DAVINCI_MDIO_H
#define __DAVINCI_MDIO_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/davinci_emac.h>
#include <linux/gpio.h>
#include <linux/reboot.h>

/*
 * This timeout definition is a worst-case ultra defensive measure against
 * unexpected controller lock ups.  Ideally, we should never ever hit this
 * scenario in practice.
 */
#define MDIO_TIMEOUT		100 /* msecs */

#define PHY_REG_MASK		0x1f
#define PHY_ID_MASK		0x1f

#define DEF_OUT_FREQ		2200000		/* 2.2 MHz */

#define CPGMAC_CLK_CTRL_REG	0x44E00014
#define CPGMAC_CLK_SYSC         0x4A101208
#define CPSW_NO_IDLE_NO_STDBY   0xA

struct davinci_mdio_regs {
	u32	version;
	u32	control;
#define CONTROL_IDLE		BIT(31)
#define CONTROL_ENABLE		BIT(30)
#define CONTROL_MAX_DIV		(0xff)

	u32	alive;
	u32	link;
	u32	linkintraw;
	u32	linkintmasked;
	u32	__reserved_0[2];
	u32	userintraw;
	u32	userintmasked;
	u32	userintmaskset;
	u32	userintmaskclr;
	u32	__reserved_1[20];

	struct {
		u32	access;
#define USERACCESS_GO		BIT(31)
#define USERACCESS_WRITE	BIT(30)
#define USERACCESS_ACK		BIT(29)
#define USERACCESS_READ		(0)
#define USERACCESS_DATA		(0xffff)

		u32	physel;
	}	user[0];
};

struct davinci_mdio_data {
	struct mdio_platform_data pdata;
	struct davinci_mdio_regs __iomem *regs;
	spinlock_t	lock;
	struct clk	*clk;
	struct device	*dev;
	struct mii_bus	*bus;
	bool		suspended;
	unsigned long	access_time; /* jiffies */
	/* Indicates that driver shouldn't modify phy_mask in case
	 * if MDIO bus is registered from DT.
	 */
	bool		skip_scan;
	struct gpio_desc **gpio_reset;
	int		num_gpios;
	int		reset_delay_us;
};


#endif // __DAVINCI_MDIO_H
