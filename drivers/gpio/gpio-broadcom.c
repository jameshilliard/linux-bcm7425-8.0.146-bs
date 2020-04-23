/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <linux/brcmstb/brcmstb.h>

static DEFINE_SPINLOCK(bcm_gpio_lock);

#define BCM_GPIO_DIR_OUT 0
#define BCM_GPIO_DIR_IN 1

struct bcm_gpiochip
{
	struct gpio_chip chip;
	u32 reg_base;
	u32 bit_offset;
};

static void bcm_gpio_set(struct gpio_chip *chip,
			     unsigned gpio, int val)
{
	struct bcm_gpiochip *bank = container_of(chip, struct bcm_gpiochip, chip);
	u32 reg;
	u32 mask;
	u32 tmp;
	unsigned long flags;

	if (gpio >= chip->ngpio)
		BUG();

	reg = bank->reg_base + (BCHP_GIO_DATA_LO - BCHP_GIO_ODEN_LO);
	mask = 1 << (gpio + bank->bit_offset);

	spin_lock_irqsave(&bcm_gpio_lock, flags);
	tmp = BDEV_RD (reg);
	if (val == 0)
		tmp &= ~mask;
	else
		tmp |= mask;
	BDEV_WR (reg, tmp);
	spin_unlock_irqrestore(&bcm_gpio_lock, flags);
}

static int bcm_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	struct bcm_gpiochip *bank = container_of(chip, struct bcm_gpiochip, chip);
	u32 reg;
	u32 mask;
	u32 tmp;

	if (gpio >= chip->ngpio)
		BUG();

	reg = bank->reg_base + (BCHP_GIO_DATA_LO - BCHP_GIO_ODEN_LO);
	mask = 1 << (gpio + bank->bit_offset);

	tmp = BDEV_RD (reg);

	return !!(tmp & mask);
}

static int bcm_gpio_set_direction(struct gpio_chip *chip,
				  unsigned gpio, int dir)
{
	struct bcm_gpiochip *bank = container_of(chip, struct bcm_gpiochip, chip);
	u32 reg;
	u32 mask;
	u32 tmp;
	unsigned long flags;

	if (gpio >= chip->ngpio)
		BUG();

	reg = bank->reg_base + (BCHP_GIO_IODIR_LO - BCHP_GIO_ODEN_LO);
	mask = 1 << (gpio + bank->bit_offset);

	spin_lock_irqsave(&bcm_gpio_lock, flags);
	tmp = BDEV_RD (reg);
	if (dir == BCM_GPIO_DIR_OUT)
		tmp &= ~mask;
	else
		tmp |= mask;
	BDEV_WR (reg, tmp);
	spin_unlock_irqrestore(&bcm_gpio_lock, flags);

	return 0;
}

static int bcm_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	return bcm_gpio_set_direction(chip, gpio, BCM_GPIO_DIR_IN);
}

static int bcm_gpio_direction_output(struct gpio_chip *chip,
					 unsigned gpio, int value)
{
	bcm_gpio_set(chip, gpio, value);
	return bcm_gpio_set_direction(chip, gpio, BCM_GPIO_DIR_OUT);
}

static int bcm_gpio_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err = -EIO;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't fetch device resource info\n");
		goto done;
	}

	struct device_node *of = pdev->dev.of_node;
	if (!of) {
		dev_err(&pdev->dev, "can't find OF node\n");
		goto done;
	}

#if 0
	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "can't request region\n");
		goto done;
	}
#endif

	struct bcm_gpiochip *bgc = devm_kzalloc (&pdev->dev, sizeof (*bgc), GFP_KERNEL);
	if (!bgc) {
		dev_err(&pdev->dev, "couldn't allocate bgc struct\n");
		err = -ENOMEM;
		goto release_region;
	}

	u32 gpios[2];
	err = of_property_read_u32_array (of, "gpio", &gpios[0], 2);
	if (err)
		goto release_region;

	err = of_property_read_u32 (of, "bit-offset", &bgc->bit_offset);
	if (err && err != -EINVAL)
		goto release_region;

	printk ("%s: gpio %d-%d (@ %d)\n", dev_name(&pdev->dev), gpios[0], gpios[0] + gpios[1] - 1, bgc->bit_offset);

	bgc->reg_base = res->start;
	bgc->chip.label = dev_name(&pdev->dev);
	bgc->chip.dev = &pdev->dev;

	bgc->chip.direction_input = bcm_gpio_direction_input;
	bgc->chip.direction_output = bcm_gpio_direction_output;
	bgc->chip.get = bcm_gpio_get;
	bgc->chip.set = bcm_gpio_set;
	bgc->chip.base = gpios[0];
	bgc->chip.ngpio = gpios[1];

	platform_set_drvdata(pdev, bgc);

	return gpiochip_add (&bgc->chip);

release_region:
#if 0
	release_region(res->start, resource_size(res));
#endif
done:
	return err;
}

static int bcm_gpio_remove(struct platform_device *pdev)
{
	struct bcm_gpiochip *bgc = platform_get_drvdata(pdev);

	gpiochip_remove (&bgc->chip);
	kfree (bgc);
	return 0;
}

static const struct of_device_id gpio_bcm_of_match[] = {
	{ .compatible = "brcm,gpio-broadcom" },
	{},
};

static struct platform_driver bcm_gpio_driver = {
	.driver = {
		.name = "gpio-broadcom",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_bcm_of_match),
	},
	.probe = bcm_gpio_probe,
	.remove = bcm_gpio_remove,
};

static int __init bcm_gpio_driver_init(void)
{
	return platform_driver_register(&bcm_gpio_driver);
}
module_init(bcm_gpio_driver_init);

static void __exit bcm_gpio_driver_exit(void)
{
	platform_driver_unregister(&bcm_gpio_driver);
}
module_exit(bcm_gpio_driver_exit);
