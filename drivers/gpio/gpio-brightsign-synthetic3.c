/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#define DRV_NAME	"gpio-brightsign-synthetic3"

struct bs_s3_gpio_pin 
{
	bool claimed;
	int in;
	int out;
	int oe;
};

struct bs_s3_gpio_chip
{
	struct gpio_chip chip;
	struct bs_s3_gpio_pin pin[];
};

static void bs_s3_gpio_set(struct gpio_chip *chip,
			     unsigned gpio, int val)
{
	struct bs_s3_gpio_chip *bs_ext3 = container_of (chip, struct bs_s3_gpio_chip, chip);
	BUG_ON (gpio >= chip->ngpio);
	if (bs_ext3->pin[gpio].out >= 0)
		gpio_set_value (bs_ext3->pin[gpio].out, val);
}

static int bs_s3_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	struct bs_s3_gpio_chip *bs_ext3 = container_of (chip, struct bs_s3_gpio_chip, chip);
	BUG_ON (gpio >= chip->ngpio);
	if (bs_ext3->pin[gpio].in >= 0)
		return gpio_get_value (bs_ext3->pin[gpio].in);
	else
		return 0;
}

static int bs_s3_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	struct bs_s3_gpio_chip *bs_ext3 = container_of (chip, struct bs_s3_gpio_chip, chip);
	BUG_ON (gpio >= chip->ngpio);
	if (bs_ext3->pin[gpio].in >= 0) {
		gpio_set_value (bs_ext3->pin[gpio].oe, 0);
		gpio_direction_input(bs_ext3->pin[gpio].in);
	}
	return 0;
}

static int bs_s3_gpio_direction_output(struct gpio_chip *chip,
					 unsigned gpio, int value)
{
	struct bs_s3_gpio_chip *bs_ext3 = container_of (chip, struct bs_s3_gpio_chip, chip);
	BUG_ON (gpio >= chip->ngpio);
	if (bs_ext3->pin[gpio].out >= 0) {
		gpio_direction_output (bs_ext3->pin[gpio].out, value);
		gpio_set_value (bs_ext3->pin[gpio].oe, 1);
	}
	return 0;
}

static int bs_s3_gpio_request(struct gpio_chip *chip, unsigned gpio)
{
	struct bs_s3_gpio_chip *bs_ext3 = container_of (chip, struct bs_s3_gpio_chip, chip);
	struct bs_s3_gpio_pin *pin = &bs_ext3->pin[gpio];
	if (!pin->claimed) {
		int err = 0;
		if (pin->oe >= 0)
			err = gpio_request_one(pin->oe, GPIOF_IN, DRV_NAME);
		if (err)
			return err;
		if (pin->in >= 0)
			err = gpio_request_one(pin->in, GPIOF_IN, DRV_NAME);
		if (err) {
			gpio_free (pin->oe);
			return err;
		}
		if (pin->in != pin->out) {
			if (pin->out >= 0)
				err = gpio_request_one(pin->out, GPIOF_IN, DRV_NAME);
			if (err) {
				gpio_free (pin->in);
				gpio_free (pin->oe);
				return err;
			}
		}
		if (pin->in >= 0)
			err = gpio_direction_input(pin->in);
		if (!err)
			if (pin->oe >= 0)
				err = gpio_direction_output(pin->oe, 0);
		if (err) {
			if (pin->in >= 0)
				gpio_free (pin->in);
			if (pin->oe >= 0)
				gpio_free (pin->oe);
			if (pin->in != pin->out && pin->out >= 0)
				gpio_free (pin->out);
			return err;
		}
		pin->claimed = true;
	}
	return 0;
}

static void bs_s3_gpio_free(struct gpio_chip *chip, unsigned gpio)
{
	struct bs_s3_gpio_chip *bs_ext3 = container_of (chip, struct bs_s3_gpio_chip, chip);
	struct bs_s3_gpio_pin *pin = &bs_ext3->pin[gpio];
	if (pin->claimed) {
		if (pin->oe >= 0)
			gpio_free (pin->oe);
		if (pin->in >= 0)
			gpio_free (pin->in);
		if (pin->in != pin->out) {
			if (pin->out >= 0)
				gpio_free (pin->out);
		}
		pin->claimed = false;
	}
}

static int bs_gpio_s3_probe(struct platform_device *pdev)
{
	int err = -EIO;
	u32 index;

	struct device_node *of = pdev->dev.of_node, *pp;
	if (!of) {
		dev_err(&pdev->dev, "can't find OF node\n");
		goto done;
	}

	// Work out how many GPIOs we're going to expose.
	u32 ngpio = 0;
	for_each_child_of_node(of, pp) {
		u32 index;
		if (of_property_read_u32(pp, "index", &index) == 0)
			if (index >= ngpio)
				ngpio = index + 1;
	}

	if (ngpio == 0) {
		dev_err(&pdev->dev, "no pins defined\n");
		goto done;
	}
	
	struct bs_s3_gpio_chip *bgc = devm_kzalloc (&pdev->dev, sizeof (*bgc) + ngpio * sizeof (struct bs_s3_gpio_pin), GFP_KERNEL);
	if (!bgc) {
		dev_err(&pdev->dev, "couldn't allocate bgc struct\n");
		err = -ENOMEM;
		goto done;
	}

	// No GPIO for each index by default
	for(index = 0; index < ngpio; ++index)
		bgc->pin[index].in = bgc->pin[index].out = bgc->pin[index].oe = -1;

	for_each_child_of_node(of, pp) {
		int in, out, oe;
		u32 index;

		if (of_property_read_u32(pp, "index", &index)) {
			dev_err(&pdev->dev, "failed to read index for '%s'\n", pp->full_name);
			continue;
		}

		BUG_ON(index >= ngpio);

		in = of_get_named_gpio_flags(pp, "in", 0, NULL);
		out = of_get_named_gpio_flags(pp, "out", 0, NULL);
		oe = of_get_named_gpio_flags(pp, "oe", 0, NULL);

		dev_info(&pdev->dev, "%u I=%d O=%d OE=%d\n", index, in, out, oe);

		if (in < 0 || out < 0 || oe < 0) {
			dev_err (&pdev->dev, "couldn't get gpios for %d: (%d, %d, %d)\n", index, in, out, oe);
			continue;
		}

		bgc->pin[index].in = in;
		bgc->pin[index].out = out;
		bgc->pin[index].oe = oe;
		bgc->pin[index].claimed = true;

		err = gpio_request_one(in, GPIOF_IN, DRV_NAME);
		if (err) {
			dev_err(&pdev->dev, "unable to claim gpio %u, err=%d\n",
				in, err);
			goto done;
		}

		err = gpio_direction_input(in);
		if (err) {
			dev_err(&pdev->dev, "unable to configure gpio %u as input, err=%d\n", in, err);
			goto done;
		}

		if (in != out) {
			err = gpio_request_one(out, GPIOF_IN, DRV_NAME);
			if (err) {
				dev_err(&pdev->dev, "unable to claim gpio %u, err=%d\n",
					out, err);
				goto done;
			}
		}

		err = gpio_request_one(oe, GPIOF_IN, DRV_NAME);
		if (err) {
			dev_err(&pdev->dev, "unable to claim gpio %u, err=%d\n",
				oe, err);
			goto done;
		}

		err = gpio_direction_output(oe, 0);
		if (err) {
			dev_err(&pdev->dev, "unable to configure gpio %u as output, err=%d\n", oe, err);
			goto done;
		}
	}

	bgc->chip.label = dev_name(&pdev->dev);
	bgc->chip.dev = &pdev->dev;

	bgc->chip.direction_input = bs_s3_gpio_direction_input;
	bgc->chip.direction_output = bs_s3_gpio_direction_output;
	bgc->chip.get = bs_s3_gpio_get;
	bgc->chip.set = bs_s3_gpio_set;
	bgc->chip.ngpio = ngpio;
	bgc->chip.base = -1;
	bgc->chip.request = bs_s3_gpio_request;
	bgc->chip.free = bs_s3_gpio_free;

	platform_set_drvdata(pdev, bgc);

	return gpiochip_add (&bgc->chip);

done:
	return err;
}

static int bs_gpio_s3_remove(struct platform_device *pdev)
{
	struct bs_s3_gpio_chip *bgc = platform_get_drvdata(pdev);
	gpiochip_remove (&bgc->chip);
	kfree (bgc);
	return 0;
}

static const struct of_device_id gpio_bs_synthetic3_of_match[] = {
	{ .compatible = "brightsign,gpio_synthetic3" },
	{},
};

static struct platform_driver bs_gpio_synthetic3_driver = {
	.driver = {
		.name = "gpio-brightsign-synthetic3",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_bs_synthetic3_of_match),
	},
	.probe = bs_gpio_s3_probe,
	.remove = bs_gpio_s3_remove,
};

module_platform_driver(bs_gpio_synthetic3_driver);
