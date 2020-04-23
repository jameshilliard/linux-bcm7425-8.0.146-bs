/*
 * Copyright (C) 2017 BrightSign LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>

/***********************************************************************
 * Compatibility hack for Tiger devices. Although modern devices have
 * their USB device nodes directly under the rdb node, Tiger has to
 * have them under usb_0 and usb_1 nodes so that BOLT knows to remove
 * one of them on 7252 chips. But, since they are there they won't be
 * found automatically. This stub finds the usb_0 and usb_1 nodes
 * because they are compatible with brcm,usb-instance and then causes
 * their children to be evaluated by calling of_platform_populate.
 ***********************************************************************/

#ifdef CONFIG_OF

static int tiger_usb_instance_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	return of_platform_populate(dn, NULL, NULL, pdev->dev.parent);
}

static const struct of_device_id tiger_usb_instance_match[] = {
	{ .compatible = "brcm,usb-instance" },
	{},
};

static struct platform_driver tiger_usb_instance_driver = {
	.driver = {
		.name = "tiger-brcm",
		.bus = &platform_bus_type,
		.of_match_table = of_match_ptr(tiger_usb_instance_match),
	}
};

static int __init tiger_usb_instance_init(void)
{
	return platform_driver_probe(&tiger_usb_instance_driver,
				     tiger_usb_instance_probe);
}
module_init(tiger_usb_instance_init);

#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("BrightSign LLC");
MODULE_DESCRIPTION("Tiger USB unwrapper");
