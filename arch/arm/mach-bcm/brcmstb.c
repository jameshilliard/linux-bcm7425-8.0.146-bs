/*
 * Copyright (C) 2013-2016 Broadcom
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

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of_platform.h>

#include <linux/brcmstb/cma_driver.h>
#include <linux/clocksource.h>
#include <linux/console.h>
#include <linux/of_address.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

static const char *brightsign_match[] __initdata = {
	"brightsign,digital-signage-media-player",
	NULL
};

/*
 * Storage for debug-macro.S's state.
 *
 * This must be in .data not .bss so that it gets initialized each time the
 * kernel is loaded. The data is declared here rather than debug-macro.S so
 * that multiple inclusions of debug-macro.S point at the same data.
 */
u32 brcmstb_uart_config[3] = {
	/* Debug UART initialization required */
	1,
	/* Debug UART physical address */
	0,
	/* Debug UART virtual address */
	0,
};

static struct platform_device bcm74xx_wdt = {
	.name = "bcm74xx-wdt",
};

static void __init brcmstb_init_irq(void)
{
	gic_set_irqchip_flags(IRQCHIP_MASK_ON_SUSPEND);
	irqchip_init();
}

static const char *const brcmstb_match[] __initconst = {
	"brcm,bcm7445",
	"brcm,brcmstb",
	NULL
};

static void __init brcmstb_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	platform_device_register (&bcm74xx_wdt);
//	cma_register();
}

static void __init brcmstb_only_init_early(void)
{
	/* Serial ports aren't consoles by default in BrightSign's
	 * kernel, so if we're not running on a BrightSign let's just
	 * add the default serial port as the console. */
	add_preferred_console("ttyS", 0, "115200");
}

#ifdef CONFIG_CPU_V7
#include <asm/proc-fns.h>
#endif

static void __init brightsign_init_early(void)
{
	int console_enabled = 1;

#ifdef CONFIG_CPU_V7
	if (of_machine_is_compatible("brightsign,ranchero-revc")) {
		pr_info("BrightSign Ranchero revision C, D or E found. Idle workaround enabled.");
		cpu_do_idle = impala_revc_do_idle;
	}
	else if (of_machine_is_compatible("brightsign,impala-revc")) {
		pr_info("BrightSign Impala revision C, D or E found. Idle workaround enabled.");
		cpu_do_idle = impala_revc_do_idle;
	}
#endif // CONFIG_CPU_V7

	const struct device_node *bolt = of_find_node_by_path ("/bolt");
	if (bolt) {
		int len;
		const void *p;
		p = of_get_property (bolt, "brightsign-console-enabled", &len);
		if (p)
			console_enabled = be32_to_cpup (p);
	}

	if (console_enabled) {
		add_preferred_console("ttyS", 0, "115200");
		if (console_enabled > 1)
			console_loglevel = 8;
	} else {
		add_preferred_console("none", 0, "");
	}

	/* Pretend that the console was set on the command line to
	 * stop of_console_check from always adding the first serial
	 * port it sees as the console too. The correct way to solve
	 * this problem is to set the console correctly in the device
	 * tree, but we must remain compatible with older versions of
	 * the bootloader that didn't do that.
	 */
	console_set_on_cmdline = 1;
}

/* Now that we can't just write to arbitrary hardware addresses,
 * we need to do these fixups later than early_init.
 */
static void __init brightsign_init_machine(void)
{
	brcmstb_init_machine();

#define DDR34_PHY_PLL_FRAC_DIVIDER	(0x1C)
#define DDR34_PHY_PLL_SS_CONTROL	(0x20)
#define DDR34_PHY_PLL_SS_LIMIT		(0x24)

	if (of_machine_is_compatible("brightsign,tiger")) {

		const struct device_node *rdb = of_find_node_by_path ("/rdb");

		/* Early Tiger boot-code doesn't set the DDR PLL fractional part 
	 	* correctly, or enable spread spectrum, which leads to interference 
	 	* on the WiFi. See Bug #20007 and Broadcom case #862798 for details.
	 	*/
		u8* __iomem ddr_phy_regs;

		struct device_node *memory_controllers = of_get_child_by_name(rdb, "memory_controllers");
		struct device_node *memc = of_get_next_child(memory_controllers, NULL);
		while(memc) {
			struct device_node *ddr_phy = of_get_child_by_name(memc, "ddr-phy");
			struct resource res;
			if (of_address_to_resource(ddr_phy, 0, &res) == 0) {

				ddr_phy_regs = ioremap_nocache(res.start, resource_size(&res));

				/* No spread spectrum => boot code 5.0.22.5 */
				if (readl(ddr_phy_regs + DDR34_PHY_PLL_SS_CONTROL) == 0x00000000) {
					writel(0x00048174, ddr_phy_regs + DDR34_PHY_PLL_FRAC_DIVIDER);
					writel(0x00032470, ddr_phy_regs + DDR34_PHY_PLL_SS_LIMIT);
					writel(0x000019B1, ddr_phy_regs + DDR34_PHY_PLL_SS_CONTROL);
				}

				iounmap(ddr_phy_regs);
			}
			memc = of_get_next_child(memory_controllers, memc);
		}

#define SUN_TOP_CTRL_PIN_MUX_CTRL_8_offset		0x120
#define SUN_TOP_CTRL_PIN_MUX_CTRL_8_gpio_005_MASK       0x0000f000
#define SUN_TOP_CTRL_PIN_MUX_CTRL_8_gpio_004_MASK       0x00000f00
#define SUN_TOP_CTRL_PIN_MUX_CTRL_8_gpio_005_UART_RTS_0 2
#define SUN_TOP_CTRL_PIN_MUX_CTRL_8_gpio_004_UART_CTS_0 2

		{
			struct device_node *sun_top_ctrl = of_find_compatible_node(NULL, NULL, "brcm,brcmstb-sun-top-ctrl");
			struct resource res;
			if (of_address_to_resource(sun_top_ctrl, 0, &res) == 0) {
				// We only need to map one register
				u8* __iomem pin_mux_ctrl_8 = ioremap_nocache(res.start + SUN_TOP_CTRL_PIN_MUX_CTRL_8_offset, 4);

				// Enable pinmux for serial port RTS/CTS on Tiger
				uint32_t v = readl(pin_mux_ctrl_8);
				v &= ~(SUN_TOP_CTRL_PIN_MUX_CTRL_8_gpio_005_MASK | SUN_TOP_CTRL_PIN_MUX_CTRL_8_gpio_004_MASK);
				v |= SUN_TOP_CTRL_PIN_MUX_CTRL_8_gpio_005_UART_RTS_0;
				v |= SUN_TOP_CTRL_PIN_MUX_CTRL_8_gpio_004_UART_CTS_0;
				writel(v, pin_mux_ctrl_8);
				iounmap(pin_mux_ctrl_8);
			}
		}
	}
}

DT_MACHINE_START(BRIGHTSIGN, "BrightSign Digital Signage Media Player")
	.dt_compat	= brightsign_match,
	.init_irq	= brcmstb_init_irq,
	.init_machine	= brightsign_init_machine,
	.init_early	= brightsign_init_early,
MACHINE_END

DT_MACHINE_START(BRCMSTB, "Broadcom STB (Flattened Device Tree)")
	.dt_compat	= brcmstb_match,
	.init_irq	= brcmstb_init_irq,
	.init_machine	= brcmstb_init_machine,
	.init_early	= brcmstb_only_init_early,
MACHINE_END
