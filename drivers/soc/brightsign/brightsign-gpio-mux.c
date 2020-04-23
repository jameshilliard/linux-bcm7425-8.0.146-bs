/*
 * BrightSign GPIO alternate function multiplexing
 *
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
#include <linux/ratelimit.h>
#include <asm/io.h>

struct gpio_mux_device {
	struct device_node *gpio_functions;
	spinlock_t lock;
};

static void gpio_apply_mux(struct gpio_mux_device* gpio_dev, u32* mux_info, int len)
{
	if (len < 0)	/* Should not be possible. */
		return;

	/* The data is triples (addr, mask, data) */
	while(len > 2) {
		unsigned long flags;

		u32 addr = *mux_info++;
		u32 mask = *mux_info++;
		u32 data = *mux_info++;
		len -= 3;

		void *mapped_addr = ioremap(addr, sizeof(data));

		spin_lock_irqsave(&gpio_dev->lock, flags);
		writel((readl(mapped_addr) & ~mask) | data, mapped_addr);
		spin_unlock_irqrestore(&gpio_dev->lock, flags);

		iounmap(mapped_addr);
	}
}


/* Before complaining that this data structure is over-complicated,
 * please check the gerrit history to ensure your ire is
 * appropriately directed.  This is not my choice of implementation.
 */

/* Extract the GPIO multiplexing data from the device tree
 * Returns the number of bytes found, or an error.
 * If "output" is non NULL, the data is stored there
 */
static ssize_t gpio_mux_get_data(struct gpio_mux_device *gpio_mux_dev, const char *elem, u32* output)
{
	ssize_t result = 0;

	int lenp = 0;
	const void *p = of_get_property(gpio_mux_dev->gpio_functions, elem, &lenp);
	int num_u32 = lenp / sizeof(u32);
	u32* phandles = NULL;

	/* As device-tree doesn't support symbolic literals (yet), we have to use
	 * phandles for symbolic names.  The selected property then contains a
	 * list of phandles, and we look up the corresponding node.  If any node
	 * contains properties "addr", "mask", and "data", we add the triple to
	 * the operations we will perform to update the pin multiplexing.
	 *
	 * The "gpio_mux_input" and "gpio_mux_output" nodes are handled in user
	 * space, for consistency with other GPIO direction controls.
	 */

	if (p && lenp >= 4) {	/* We must have at least one node */
		phandles = kmalloc(lenp, GFP_KERNEL);
	}
	if (phandles) {
		if (of_property_read_u32_array(gpio_mux_dev->gpio_functions, elem, phandles, num_u32) == 0) {
			for(int i = 0; i < num_u32; i++) {
				struct device_node *child = of_find_node_by_phandle(phandles[i]);
				if (child) {
					u32 addr;
					u32 mask;
					u32 data;
					// We are only interested in properties with (addr, mask, data) triples
					if ( (of_property_read_u32_index(child, "addr", 0, &addr) == 0) &&
						(of_property_read_u32_index(child, "mask", 0, &mask) == 0) &&
						(of_property_read_u32_index(child, "data", 0, &data) == 0) ) {

						result += 3 * sizeof(u32);

						if (output) {
							*output++ = addr;
							*output++ = mask;
							*output++ = data;
						}
					}
				}
			}
		}
		kfree(phandles);
	}
	return result;
}

static ssize_t gpio_mux_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct gpio_mux_device *gpio_mux_dev = dev_get_drvdata(dev);
	if (gpio_mux_dev->gpio_functions) {
		/* Don't trust the input data to be NUL terminated */
		char* name = kmalloc(size + 1, GFP_KERNEL);
		if (!name)
			return -ENOMEM;
		memcpy(name, buf, size);
		name[size] = '\0';

		ssize_t len_data = gpio_mux_get_data(gpio_mux_dev, name, NULL);
		if (len_data > 0)
		{
			u32* mux_data = kmalloc(len_data, GFP_KERNEL);
			if (mux_data) {
				len_data = gpio_mux_get_data(gpio_mux_dev, name, mux_data);
				if (len_data > 0) 	/* Should not have changed */
					gpio_apply_mux(gpio_mux_dev, mux_data, len_data / sizeof(u32));
				kfree(mux_data);
			} else {
				dev_err(dev, "out of memory\n");
				return -ENOMEM;
			}
		} else
			dev_err(dev, "invalid mux request %s\n", buf);

		kfree(name);
	}
	return size;
}

static DEVICE_ATTR(gpio_mux, S_IWUSR, NULL, gpio_mux_store);

static int gpio_mux_probe(struct platform_device *pdev)
{
	int result = -ENODEV;

	struct gpio_mux_device *gpio_mux_dev = devm_kzalloc (&pdev->dev, sizeof(struct gpio_mux_device), GFP_KERNEL);
	if (!gpio_mux_dev) {
		dev_err(&pdev->dev, "couldn't allocate device data\n");
		result = -ENOMEM;
		return result;
	}
	dev_set_drvdata(&pdev->dev, gpio_mux_dev);
	spin_lock_init(&gpio_mux_dev->lock);

	result = device_create_file(&pdev->dev, &dev_attr_gpio_mux);
	if (result) {
		dev_err(&pdev->dev, "failed to add device attribute file\n");
		return result;
	}

	struct device_node* of = pdev->dev.of_node;
	if (!of) {
		dev_err(&pdev->dev, "can't find OF node\n");
		return -ENODEV;
	}

	gpio_mux_dev->gpio_functions = of_get_child_by_name(of, "gpio-functions");

	return 0;
}

static int gpio_mux_remove(struct platform_device *pdev)
{
	struct gpio_mux_device *gpio_mux_dev = dev_get_drvdata(&pdev->dev);
        device_remove_file(&pdev->dev, &dev_attr_gpio_mux);
	of_node_put(gpio_mux_dev->gpio_functions);
	return 0;
}

static const struct of_device_id gpio_mux_of_match[] = {
	{ .compatible = "brightsign,gpio-mux" },
	{},
};

static struct platform_driver gpio_mux_driver = {
	.driver = {
		.name	= "brightsign-gpio-mux",
		.owner	= THIS_MODULE,
		.of_match_table = gpio_mux_of_match,
	},
	.probe	= gpio_mux_probe,
	.remove	= gpio_mux_remove,
};

module_platform_driver(gpio_mux_driver);

MODULE_DESCRIPTION("BrightSign GPIO Mux driver");
MODULE_AUTHOR("BrightSign Digital Ltd.");
MODULE_LICENSE("GPL v2");
