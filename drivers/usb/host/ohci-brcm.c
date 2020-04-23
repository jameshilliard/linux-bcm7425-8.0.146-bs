/*
 * Copyright (C) 2009 - 2014 Broadcom Corporation
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
 */

#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/usb/ohci_pdriver.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/phy/phy.h>

#include "ohci.h"
#include "usb-brcm.h"

#define BRCM_DRIVER_DESC "OHCI BRCM driver"

static const char hcd_name[] = "ohci-brcm";

struct brcm_hcd {
	struct clk *hcd_clk;
	struct phy *hcd_phy;
#ifdef CONFIG_TIGER_DISABLE_WIFI_ON_POE
	bool allow_inhibit;
#endif
};


#ifdef CONFIG_TIGER_DISABLE_WIFI_ON_POE

/* BrightSign: special case for Tiger using PoE */
static unsigned int inhibit_internal_port = 0;
module_param(inhibit_internal_port, uint, S_IRUGO);

/* For OHCI, we only need these hooks when inhibiting */

static int (*org_hub_control)(struct usb_hcd *hcd,
                        u16 typeReq, u16 wValue, u16 wIndex,
                        char *buf, u16 wLength);
static int (*org_ohci_start)(struct usb_hcd *hcd);

static int ohci_brcm_start(struct usb_hcd *hcd)
{
        /* BrightSign: Disable WiFi port when using PoE. BS#29159 */
	/* This is only used on Tiger */
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
	struct brcm_hcd* brcm_hcd_ptr = (struct brcm_hcd *)ohci->priv;
        if (inhibit_internal_port && (brcm_hcd_ptr->allow_inhibit)) {
		printk("ohci-brcm: preventing startup on internal port\n");
	        ohci->rh_state = OHCI_RH_HALTED;
		ohci_writel(ohci, (u32) ~0, &ohci->regs->intrdisable);

		/* Software reset, after which the controller goes into SUSPEND */
		ohci_writel(ohci, OHCI_HCR, &ohci->regs->cmdstatus);
		ohci_readl(ohci, &ohci->regs->cmdstatus);       /* flush the writes */

		return 0;
	}

	return (*org_ohci_start)(hcd);
}

static int ohci_brcm_hub_control(
        struct usb_hcd  *hcd,
        u16             typeReq,
        u16             wValue,
        u16             wIndex,
        char            *buf,
        u16             wLength)
{
        /* BrightSign: Disable WiFi port when using PoE. BS#29159 */
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
	struct brcm_hcd* brcm_hcd_ptr = (struct brcm_hcd *)ohci->priv;
        if (inhibit_internal_port && (brcm_hcd_ptr->allow_inhibit)) {
                /* Force the power off every time an operation is attempted */
                /* printk("ohci-brcm: %hx %hx %hx\n", typeReq, wValue, wIndex); */
		ohci_writel(ohci, 0x00000201, &ohci->regs->roothub.portstatus[0x0]);
        }

        return (*org_hub_control)(hcd, typeReq, wValue, wIndex, buf, wLength);
}

#endif

static int ohci_brcm_reset(struct usb_hcd *hcd)
{
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);

	ohci->flags |= OHCI_QUIRK_BE_MMIO;

#ifdef CONFIG_TIGER_DISABLE_WIFI_ON_POE
	/* BrightSign: Special option for Tiger only to conditionally disable
	 * the first USB port on the second controller.  Needed for WiFi
	 * power disable when using PoE.  BS#29159
	 */
	{
		struct brcm_hcd* brcm_hcd_ptr = (struct brcm_hcd *)ohci->priv;
		if (inhibit_internal_port && brcm_hcd_ptr->allow_inhibit) {
			int rc;
			printk("ohci-brcm: power off internal USB port\n");
			rc = ohci_setup(hcd);   /* Setup can reset power state */

			ohci_writel(ohci, 0x02000101, &ohci->regs->roothub.a);
			ohci_writel(ohci, 0x00020000, &ohci->regs->roothub.b);
			ohci_writel(ohci, 0x00000201, &ohci->regs->roothub.portstatus[0x0]);
			return rc;
		}
        }
#endif
	return ohci_setup(hcd);
}

static struct hc_driver __read_mostly ohci_brcm_hc_driver;

static const struct ohci_driver_overrides brcm_overrides __initconst = {
	.product_desc =	"BRCM OHCI controller",
	.reset =	ohci_brcm_reset,
	.extra_priv_size = sizeof(struct brcm_hcd),
};

static void ohci_brcm_quirk(struct platform_device* dev)
{
#ifdef CONFIG_TIGER_DISABLE_WIFI_ON_POE
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct brcm_hcd *brcm_hcd_ptr = (struct brcm_hcd *)hcd_to_ohci(hcd)->priv;
	brcm_hcd_ptr->allow_inhibit = of_property_read_bool(dev->dev.of_node,
								"allow-internal-inhibit");
#endif
}

static int ohci_brcm_probe(struct platform_device *dev)
{
	int err;
	struct usb_hcd *hcd;
	struct brcm_hcd *brcm_hcd_ptr;
	struct clk *usb_clk;
	struct phy *phy;

	err = dma_coerce_mask_and_coherent(&dev->dev, DMA_BIT_MASK(32));
	if (err)
		return err;

#ifdef CONFIG_TIGER_DISABLE_WIFI_ON_POE
	{
		/* BrightSign: Intercept hub control so we can optionally inhibit port */
		if (org_hub_control == NULL) {
			org_hub_control = ohci_brcm_hc_driver.hub_control;
			ohci_brcm_hc_driver.hub_control = ohci_brcm_hub_control;
		}
		if (org_ohci_start == NULL) {
			org_ohci_start = ohci_brcm_hc_driver.start;
			ohci_brcm_hc_driver.start = ohci_brcm_start;
		}
	}
#endif
	err = brcm_usb_probe(dev, &ohci_brcm_hc_driver, &hcd, &usb_clk, &phy, ohci_brcm_quirk);
	if (err)
		return err;

	brcm_hcd_ptr = (struct brcm_hcd *)hcd_to_ohci(hcd)->priv;
	brcm_hcd_ptr->hcd_clk = usb_clk;
	brcm_hcd_ptr->hcd_phy = phy;

	return err;
}

static int ohci_brcm_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct brcm_hcd *brcm_hcd_ptr =
		(struct brcm_hcd *)hcd_to_ohci(hcd)->priv;

	brcm_usb_remove(dev, brcm_hcd_ptr->hcd_clk);
	phy_exit(brcm_hcd_ptr->hcd_phy);
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int ohci_brcm_suspend(struct device *dev)
{
	int ret;
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct brcm_hcd *brcm_hcd_ptr =
		(struct brcm_hcd *)hcd_to_ohci(hcd)->priv;
	bool do_wakeup = device_may_wakeup(dev);

	ret = ohci_suspend(hcd, do_wakeup);
	clk_disable(brcm_hcd_ptr->hcd_clk);
	return ret;
}

static int ohci_brcm_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct brcm_hcd *brcm_hcd_ptr =
		(struct brcm_hcd *)hcd_to_ohci(hcd)->priv;
	int err;

	err = clk_enable(brcm_hcd_ptr->hcd_clk);
	if (err)
		return err;
	ohci_resume(hcd, false);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(ohci_brcm_pm_ops, ohci_brcm_suspend,
		ohci_brcm_resume);

#ifdef CONFIG_OF
static const struct of_device_id brcm_ohci_of_match[] = {
	{ .compatible = "brcm,ohci-brcm", },
	{ .compatible = "brcm,ohci-brcm-v2", },
	{}
};

MODULE_DEVICE_TABLE(of, brcm_ohci_of_match);
#endif /* CONFIG_OF */

static struct platform_driver ohci_brcm_driver = {
	.probe		= ohci_brcm_probe,
	.remove		= ohci_brcm_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ohci-brcm",
		.pm	= &ohci_brcm_pm_ops,
		.of_match_table = of_match_ptr(brcm_ohci_of_match),
	}
};

static int __init ohci_brcm_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " BRCM_DRIVER_DESC "\n", hcd_name);

	ohci_init_driver(&ohci_brcm_hc_driver, &brcm_overrides);
	return platform_driver_register(&ohci_brcm_driver);
}
module_init(ohci_brcm_init);

static void __exit ohci_brcm_cleanup(void)
{
	platform_driver_unregister(&ohci_brcm_driver);
}
module_exit(ohci_brcm_cleanup);

MODULE_DESCRIPTION(BRCM_DRIVER_DESC);
MODULE_AUTHOR("Al Cooper");
MODULE_LICENSE("GPL");
