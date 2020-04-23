/*
 * Null console driver.
 *
 * Can be used via the device tree chosen/stdout-path node to indicate
 * that no console should be used at all.
 *
 */

#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct null_console_device {
	struct console con;
};

static int null_console_probe(struct platform_device *pdev)
{
	int result = -ENODEV;

	struct null_console_device *null_console_dev = devm_kzalloc (&pdev->dev, sizeof(struct null_console_device), GFP_KERNEL);
	if (!null_console_dev) {
		dev_err(&pdev->dev, "couldn't allocate device data\n");
		result = -ENOMEM;
		return result;
	}
	dev_set_drvdata(&pdev->dev, null_console_dev);

	/* All the functions can just be left as NULL, but we must set
	 * the name and the flags. */
	strcpy(null_console_dev->con.name, "nullcon");
	null_console_dev->con.flags = CON_ENABLED;

	register_console(&null_console_dev->con);
	of_console_check(pdev->dev.of_node, "nullcon", 0);

	return 0;
}

static int null_console_remove(struct platform_device *pdev)
{
	struct null_console_device *null_console_dev = dev_get_drvdata(&pdev->dev);
	unregister_console(&null_console_dev->con);
	return 0;
}

static const struct of_device_id null_console_of_match[] = {
	{ .compatible = "brightsign,null-console" },
	{},
};

static struct platform_driver null_console_driver = {
	.driver = {
		.name	= "null-console",
		.owner	= THIS_MODULE,
		.of_match_table = null_console_of_match,
	},
	.probe	= null_console_probe,
	.remove	= null_console_remove,
};

module_platform_driver(null_console_driver);

MODULE_DESCRIPTION("Null console driver");
MODULE_AUTHOR("BrightSign Digital Ltd.");
MODULE_LICENSE("GPL v2");
