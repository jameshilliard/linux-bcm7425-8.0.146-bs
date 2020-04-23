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
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/ehci_pdriver.h>
#include <linux/clk.h>
#include <linux/phy/phy.h>

#include "ehci.h"
#include "usb-brcm.h"

#define BRCM_DRIVER_DESC "EHCI BRCM driver"

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
#endif

static const char brcm_hcd_name[] = "ehci-brcm";

static int (*org_hub_control)(struct usb_hcd *hcd,
			u16 typeReq, u16 wValue, u16 wIndex,
			char *buf, u16 wLength);

/* ehci_brcm_wait_for_sof
 * Wait for start of next microframe, then wait extra delay microseconds
 */
static inline void ehci_brcm_wait_for_sof(struct ehci_hcd *ehci, u32 delay)
{
	int frame_idx = ehci_readl(ehci, &ehci->regs->frame_index);

	while (frame_idx == ehci_readl(ehci, &ehci->regs->frame_index))
		;
	udelay(delay);
}

/* ehci_brcm_hub_control
 * Intercept echi-hcd request to complete RESUME and align it to the start
 * of the next microframe.
 * If RESUME is complete too late in the microframe, host controller
 * detects babble on suspended port and resets the port afterwards.
 * This s/w workaround allows to avoid this problem.
 * See http://jira.broadcom.com/browse/SWLINUX-1909 for more details
 */
static int ehci_brcm_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int		ports = HCS_N_PORTS(ehci->hcs_params);
	u32 __iomem	*status_reg = &ehci->regs->port_status[
				(wIndex & 0xff) - 1];
	unsigned long flags;
	int retval, irq_disabled = 0;

#ifdef CONFIG_TIGER_DISABLE_WIFI_ON_POE
	/* BrightSign: Disable WiFi port when using PoE.  BS#29159 */
	{
		struct brcm_hcd *brcm_hcd_ptr = (struct brcm_hcd *)hcd_to_ehci(hcd)->priv;
		if (inhibit_internal_port && (brcm_hcd_ptr->allow_inhibit)) {
			/* Turn the power off, unless this is a status request */
			/* printk("ehci-brcm: %hx %hx %hx\n", typeReq, wValue, wIndex); */
			if ((typeReq != GetHubDescriptor) && (typeReq != GetHubStatus) &&
								(typeReq != GetPortStatus)) {
				typeReq = ClearPortFeature;
				wValue = USB_PORT_FEAT_POWER;
			}
		}
	}
#endif

	/* RESUME is cleared when GetPortStatus() is called 20ms after start
	  of RESUME */
	if ((typeReq == GetPortStatus) &&
	    (wIndex && wIndex <= ports) &&
	    ehci->reset_done[wIndex-1] &&
	    time_after_eq(jiffies, ehci->reset_done[wIndex-1]) &&
	    (ehci_readl(ehci, status_reg) & PORT_RESUME)) {
		/* to make sure we are not interrupted until RESUME bit
		  is cleared, disable interrupts on current CPU */
		ehci_dbg(ehci, "SOF alignment workaround\n");
		irq_disabled = 1;
		local_irq_save(flags);
		ehci_brcm_wait_for_sof(ehci, 5);
	}
	retval = (*org_hub_control)(hcd, typeReq, wValue, wIndex, buf, wLength);
	if (irq_disabled)
		local_irq_restore(flags);
	return retval;
}

static int ehci_brcm_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	ehci->big_endian_mmio = 1;

	ehci->caps = (struct ehci_caps *) hcd->regs;
	ehci->regs = (struct ehci_regs *) (hcd->regs +
		HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase)));

	/* This fixes the lockup during reboot due to prior interrupts */
	ehci_writel(ehci, CMD_RESET, &ehci->regs->command);
	mdelay(10);

	/*
	 * SWLINUX-1705: Avoid OUT packet underflows during high memory
	 *   bus usage
	 * port_status[0x0f] = Broadcom-proprietary USB_EHCI_INSNREG00 @ 0x90
	 */
	ehci_writel(ehci, 0x00800040, &ehci->regs->port_status[0x10]);
	ehci_writel(ehci, 0x00000001, &ehci->regs->port_status[0x12]);

	/* BrightSign: Special option for Tiger only to conditionally disable
	 * the first USB port on the second controller.  Needed for WiFi
	 * power disable when using PoE.  BS#29159.
	 */
#ifdef CONFIG_TIGER_DISABLE_WIFI_ON_POE
	{
		struct brcm_hcd *brcm_hcd_ptr = (struct brcm_hcd *)hcd_to_ehci(hcd)->priv;
		if (inhibit_internal_port && (brcm_hcd_ptr->allow_inhibit)) {
			int rc;
			printk(KERN_INFO "ehci-brcm: power off internal USB port\n");
			rc = ehci_setup(hcd);	/* Setup will reset power state */

			ehci_writel(ehci, 0x02000101, &ehci->regs->port_status[0x1]);
			ehci_writel(ehci, 0x00020000, &ehci->regs->port_status[0x2]);
			ehci_writel(ehci, 0x00000000, &ehci->regs->port_status[0x4]);
			return rc;
		}
	}
#endif

	return ehci_setup(hcd);
}

static struct hc_driver __read_mostly ehci_brcm_hc_driver;

static const struct ehci_driver_overrides brcm_overrides __initconst = {

	.reset =	ehci_brcm_reset,
	.extra_priv_size = sizeof(struct brcm_hcd),
};

/* This is only used on Tiger */
static void ehci_brcm_quirk(struct platform_device *dev)
{
#ifdef CONFIG_TIGER_DISABLE_WIFI_ON_POE
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct brcm_hcd *brcm_hcd_ptr = (struct brcm_hcd *)hcd_to_ehci(hcd)->priv;
	brcm_hcd_ptr->allow_inhibit = of_property_read_bool(dev->dev.of_node,
							"allow-internal-inhibit");
	printk("ehci: allow_inhibit = %d\n", brcm_hcd_ptr->allow_inhibit);
#endif
}

static int ehci_brcm_probe(struct platform_device *dev)
{
	int err;
	struct brcm_hcd *brcm_hcd_ptr;
	struct clk *usb_clk;
	struct usb_hcd *hcd;
	struct phy *phy;

	err = dma_coerce_mask_and_coherent(&dev->dev, DMA_BIT_MASK(64));
	if (err)
		return err;

	/* Hook the hub control routine to work around a bug */
	if (org_hub_control == NULL)
		org_hub_control = ehci_brcm_hc_driver.hub_control;
	ehci_brcm_hc_driver.hub_control = ehci_brcm_hub_control;
	err = brcm_usb_probe(dev, &ehci_brcm_hc_driver, &hcd, &usb_clk, &phy, ehci_brcm_quirk);
	if (err)
		return err;
	brcm_hcd_ptr = (struct brcm_hcd *)hcd_to_ehci(hcd)->priv;
	brcm_hcd_ptr->hcd_clk = usb_clk;
	brcm_hcd_ptr->hcd_phy = phy;

	return err;
}

static int ehci_brcm_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct brcm_hcd *brcm_hcd_ptr =
		(struct brcm_hcd *)hcd_to_ehci(hcd)->priv;

	brcm_usb_remove(dev, brcm_hcd_ptr->hcd_clk);
	phy_exit(brcm_hcd_ptr->hcd_phy);
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int ehci_brcm_suspend(struct device *dev)
{
	int ret;
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct brcm_hcd *brcm_hcd_ptr =
		(struct brcm_hcd *)hcd_to_ehci(hcd)->priv;
	bool do_wakeup = device_may_wakeup(dev);

	ret = ehci_suspend(hcd, do_wakeup);
	clk_disable(brcm_hcd_ptr->hcd_clk);

	return ret;
}

static int ehci_brcm_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct brcm_hcd *brcm_hcd_ptr =
		(struct brcm_hcd *)hcd_to_ehci(hcd)->priv;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int err;

	err = clk_enable(brcm_hcd_ptr->hcd_clk);
	if (err)
		return err;

	/*
	 * SWLINUX-1705: Avoid OUT packet underflows during high memory
	 *   bus usage
	 * port_status[0x0f] = Broadcom-proprietary USB_EHCI_INSNREG00
	 * @ 0x90
	 */
	ehci_writel(ehci, 0x00800040, &ehci->regs->port_status[0x10]);
	ehci_writel(ehci, 0x00000001, &ehci->regs->port_status[0x12]);

	ehci_resume(hcd, false);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(ehci_brcm_pm_ops, ehci_brcm_suspend,
		ehci_brcm_resume);

#ifdef CONFIG_OF
static const struct of_device_id brcm_ehci_of_match[] = {
	{ .compatible = "brcm,ehci-brcm", },
	{ .compatible = "brcm,ehci-brcm-v2", },
	{}
};

MODULE_DEVICE_TABLE(of, brcm_ehci_of_match);
#endif /* CONFIG_OF */

static struct platform_driver ehci_brcm_driver = {
	.probe		= ehci_brcm_probe,
	.remove		= ehci_brcm_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ehci-brcm",
		.pm	= &ehci_brcm_pm_ops,
		.of_match_table = of_match_ptr(brcm_ehci_of_match),
	}
};

static int __init ehci_brcm_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " BRCM_DRIVER_DESC "\n", brcm_hcd_name);

	ehci_init_driver(&ehci_brcm_hc_driver, &brcm_overrides);
	return platform_driver_register(&ehci_brcm_driver);
}
module_init(ehci_brcm_init);

static void __exit ehci_brcm_cleanup(void)
{
	platform_driver_unregister(&ehci_brcm_driver);
}
module_exit(ehci_brcm_cleanup);

MODULE_DESCRIPTION(BRCM_DRIVER_DESC);
MODULE_AUTHOR("Al Cooper");
MODULE_LICENSE("GPL");
