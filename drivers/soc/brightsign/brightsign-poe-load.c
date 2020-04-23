/*
 * BrightSign POE load driver
 *
 * This driver exists in order to ensure that the bare minimum amount
 * of power is used from PoE in order to keep it alive, whilst the
 * main system runs on mains power. This means that PoE power can take
 * over immediately if mains power goes away.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pwm.h>
#include <linux/ratelimit.h>

#define POE_LOAD_ON_NS (80 * 1000 * 1000)
#define POE_LOAD_PERIOD_NS (300 * 1000 * 1000)
#define POLL_PERIOD (HZ/4)

struct poe_load_device {
	int poe_load_state;
	int gpio_poe_power_flag;
	int gpio_poe_present;
	int irq_poe_power_flag;
	int irq_poe_present;
	struct pwm_device *pwm;
};

static ssize_t poe_fitted_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct poe_load_device *poe_load_dev = dev_get_drvdata(dev);
	int poe_fitted = poe_load_dev->gpio_poe_present >= 0;
	return sprintf(buf, "%d\n", poe_fitted);
}

static DEVICE_ATTR(poe_fitted, S_IRUGO, poe_fitted_show, NULL);

static ssize_t poe_present_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct poe_load_device *poe_load_dev = dev_get_drvdata(dev);
	int poe_present = (poe_load_dev->gpio_poe_present >= 0)
		? gpio_get_value(poe_load_dev->gpio_poe_present) : 0;
	return sprintf(buf, "%d\n", poe_present);
}

static DEVICE_ATTR(poe_present, S_IRUGO, poe_present_show, NULL);

static ssize_t poe_power_flag_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct poe_load_device *poe_load_dev = dev_get_drvdata(dev);
	int poe_power_flag = (poe_load_dev->gpio_poe_power_flag >= 0)
		? gpio_get_value(poe_load_dev->gpio_poe_power_flag) : 0;
	return sprintf(buf, "%d\n", poe_power_flag);
}

static DEVICE_ATTR(poe_power_flag, S_IRUGO, poe_power_flag_show, NULL);

static irqreturn_t poe_load_interrupt(int irq, void *d)
{
	struct poe_load_device *poe_load_dev = d;

	int poe_power_flag = gpio_get_value(poe_load_dev->gpio_poe_power_flag);
	int poe_present = gpio_get_value(poe_load_dev->gpio_poe_present);

	int desired_poe_load_state = (poe_present && !poe_power_flag);
	if (poe_load_dev->poe_load_state != desired_poe_load_state)
	{
		if (desired_poe_load_state)
			pwm_enable(poe_load_dev->pwm);
		else
			pwm_disable(poe_load_dev->pwm);
		poe_load_dev->poe_load_state = desired_poe_load_state;
		pr_info_ratelimited("PoE change: POE_POWER_FLAG=%d POE_PRESENT=%d => POE_LOAD=%d\n", poe_power_flag, poe_present, desired_poe_load_state);
	}

	return IRQ_HANDLED;
}

static int is_poe_fitted(struct platform_device *pdev)
{
	struct device_node *of = pdev->dev.of_node;
	if (!of) {
		dev_err(&pdev->dev, "can't find OF node\n");
		return 0;
	}

	int gpio_poe_fitted = of_get_named_gpio_flags(of, "gpio-poe-fitted", 0, NULL);
	if (gpio_poe_fitted >= 0) {
		if (gpio_get_value(gpio_poe_fitted)) {
			dev_info(&pdev->dev, "PoE fitted\n");
			return 1;
		} else {
			dev_info(&pdev->dev, "PoE not fitted\n");
			return 0;
		}
	} else {
		/* If there's no GPIO then PoE must always be fitted. */
		return 1;
	}
}

static int poe_load_probe(struct platform_device *pdev)
{
	struct device_node *of = pdev->dev.of_node;
	if (!of) {
		dev_err(&pdev->dev, "can't find OF node\n");
		return -ENODEV;
	}

	int result = -ENODEV;

	struct poe_load_device *poe_load_dev = devm_kzalloc (&pdev->dev, sizeof(struct poe_load_device), GFP_KERNEL);
	if (!poe_load_dev) {
		dev_err(&pdev->dev, "couldn't allocate device data\n");
		result = -ENOMEM;
		return result;
	}
	dev_set_drvdata(&pdev->dev, poe_load_dev);

	// These values are used if PoE is not fitted so that the
	// attribute functions do the right thing, and to make cleanup
	// easier if things go wrong.
	poe_load_dev->gpio_poe_power_flag = -1;
	poe_load_dev->gpio_poe_present = -1;
	poe_load_dev->irq_poe_power_flag = -1;
	poe_load_dev->irq_poe_present = -1;

	if (is_poe_fitted(pdev)) {
		poe_load_dev->gpio_poe_power_flag = of_get_named_gpio_flags(of, "gpio-poe-power-flag", 0, NULL);
		poe_load_dev->gpio_poe_present = of_get_named_gpio_flags(of, "gpio-poe-present", 0, NULL);

		if (poe_load_dev->gpio_poe_power_flag < 0 || poe_load_dev->gpio_poe_present < 0) {
			dev_err(&pdev->dev, "Failed to determine GPIOs\n");
			goto fail;
		}

		if (gpio_request(poe_load_dev->gpio_poe_power_flag, "POE_POWER_FLAG") < 0) {
			dev_err(&pdev->dev, "Failed to acquire POE_POWER_FLAG GPIO\n");
			goto fail;
		}

		if (gpio_request(poe_load_dev->gpio_poe_present, "POE_PRESENT") < 0) {
			dev_err(&pdev->dev, "Failed to acquire POE_PRESENT GPIO\n");
			gpio_free(poe_load_dev->gpio_poe_power_flag);
			goto fail;
		}

		gpio_direction_input(poe_load_dev->gpio_poe_power_flag);
		gpio_direction_input(poe_load_dev->gpio_poe_present);

		// Pagani uses this driver to report the PoE state,
		// but it doesn't need to drive POE_LOAD so it has no
		// PWM. This means that it doesn't need the interrupt
		// handler either.
		poe_load_dev->pwm = of_pwm_get(of, NULL);
		if (!IS_ERR(poe_load_dev->pwm)) {
			pwm_config(poe_load_dev->pwm, POE_LOAD_ON_NS, POE_LOAD_PERIOD_NS);

			// Ensure everything is configured correctly initially by
			// faking an interrupt. We must do this before calling
			// request_irq in order to avoid races.
			poe_load_interrupt(0, poe_load_dev);

			/* We don't really mind what order the interrupts are provided
			 * in the device tree because we read the GPIOs in the
			 * interrupt handler. */
			poe_load_dev->irq_poe_power_flag = of_irq_get(of, 0);
			if (poe_load_dev->irq_poe_power_flag < 0) {
				result = poe_load_dev->irq_poe_power_flag;
				dev_err(&pdev->dev, "can't get poe_power_flag IRQ");
				goto fail;
			}

			poe_load_dev->irq_poe_present = of_irq_get(of, 1);
			if (poe_load_dev->irq_poe_present < 0) {
				result = poe_load_dev->irq_poe_present;
				dev_err(&pdev->dev, "can't get poe_present IRQ");
				goto fail;
			}

			result = request_irq(poe_load_dev->irq_poe_power_flag, poe_load_interrupt, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					     "brightsign-poe-load", poe_load_dev);
			if (result < 0) {
				dev_err(&pdev->dev, "Failed to request poe-load IRQ: %d (%d)\n", poe_load_dev->irq_poe_power_flag, result);
				goto fail;
			}

			result = request_irq(poe_load_dev->irq_poe_present, poe_load_interrupt, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					     "brightsign-poe-load", poe_load_dev);
			if (result < 0) {
				dev_err(&pdev->dev, "Failed to request poe-load IRQ: %d (%d)\n", poe_load_dev->irq_poe_present, result);
				free_irq(poe_load_dev->irq_poe_power_flag, poe_load_dev);
				goto fail;
			}
		}
	}

	result = device_create_file(&pdev->dev, &dev_attr_poe_fitted);
	if (result) {
		dev_err(&pdev->dev, "failed to add device attribute file\n");
		goto fail;
	}

	result = device_create_file(&pdev->dev, &dev_attr_poe_present);
	if (result) {
		dev_err(&pdev->dev, "failed to add device attribute file\n");
		goto fail;
	}

	result = device_create_file(&pdev->dev, &dev_attr_poe_power_flag);
	if (result) {
		dev_err(&pdev->dev, "failed to add device attribute file\n");
		goto fail;
	}


	return 0;

fail:
	/* Clearing out a file that we didn't add just causes a
	 * warning, so we don't need to track which ones we've
	 * added. */
	device_remove_file(&pdev->dev, &dev_attr_poe_power_flag);
	device_remove_file(&pdev->dev, &dev_attr_poe_present);
	device_remove_file(&pdev->dev, &dev_attr_poe_fitted);

	if (poe_load_dev->irq_poe_power_flag >= 0)
		free_irq(poe_load_dev->irq_poe_power_flag, poe_load_dev);
	if (poe_load_dev->irq_poe_present >= 0)
		free_irq(poe_load_dev->irq_poe_present, poe_load_dev);
	if (poe_load_dev->gpio_poe_present >= 0)
		gpio_free(poe_load_dev->gpio_poe_present);
	if (poe_load_dev->gpio_poe_power_flag >= 0)
		gpio_free(poe_load_dev->gpio_poe_power_flag);

	return result;
}

static int poe_load_remove(struct platform_device *pdev)
{
	struct poe_load_device *poe_load_dev = dev_get_drvdata(&pdev->dev);
	device_remove_file(&pdev->dev, &dev_attr_poe_fitted);
	device_remove_file(&pdev->dev, &dev_attr_poe_power_flag);
	device_remove_file(&pdev->dev, &dev_attr_poe_present);

	if (poe_load_dev->irq_poe_power_flag >= 0)
		free_irq(poe_load_dev->irq_poe_power_flag, poe_load_dev);
	if (poe_load_dev->irq_poe_present >= 0)
		free_irq(poe_load_dev->irq_poe_present, poe_load_dev);
	if (poe_load_dev->gpio_poe_present >= 0)
		gpio_free(poe_load_dev->gpio_poe_present);
	if (poe_load_dev->gpio_poe_power_flag >= 0)
		gpio_free(poe_load_dev->gpio_poe_power_flag);
	return 0;
}

static const struct of_device_id poe_load_of_match[] = {
	{ .compatible = "brightsign,poe-load" },
	{},
};

static struct platform_driver poe_load_driver = {
	.driver = {
		.name	= "brightsign-poe-load",
		.owner	= THIS_MODULE,
		.of_match_table = poe_load_of_match,
	},
	.probe	= poe_load_probe,
	.remove	= poe_load_remove,
};

module_platform_driver(poe_load_driver);

MODULE_DESCRIPTION("BrightSign POE load driver");
MODULE_AUTHOR("BrightSign Digital Ltd.");
MODULE_LICENSE("GPL v2");
