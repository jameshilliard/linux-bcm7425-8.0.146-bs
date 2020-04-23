#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <linux/dvb/dmx.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>

#include "dvbdev.h"
#include "xc5000.h"
#include "lgdt3305.h"

#define DRV_NAME "bs-tv"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

struct bs_tv {
	/* i2c i/o */
	struct i2c_adapter *i2c_adap;

	/* video for linux */
	struct dvb_adapter adapter;
	struct dvb_frontend *frontend;

	int fe_pwr_gpio;
	int tuner_reset_gpio;
	int demod_reset_gpio;
};

struct bs_tv_platform_priv {
	struct v4l2_subdev subdev;
};

static struct xc5000_config bs_xc5000_config =
{
	.i2c_address      = 0x61,
	.if_khz           = 6000,
	.chip_id	  = CTC701,
};

static struct lgdt3305_config bs_lgdt3305_config = {
	.i2c_addr           = 0x0e,
	.demod_chip         = LGDT3305,
	.mpeg_mode          = LGDT3305_MPEG_SERIAL,
	.tpclk_edge         = LGDT3305_TPCLK_FALLING_EDGE,
	.tpvalid_polarity   = LGDT3305_TP_VALID_HIGH,
	.deny_i2c_rptr      = 1,
	.spectral_inversion = 0,
	.qam_if_khz         = 6000,
	.vsb_if_khz         = 6000,
};

int bs_tv_callback(void *priv, int component, int command, int arg)
{
	struct bs_tv *dvb = priv;

	if (command == 0)
	{
		/* Tuner Reset Command from xc5000 */
		gpio_set_value(dvb->tuner_reset_gpio, 0);
		mdelay(10);
		gpio_set_value(dvb->tuner_reset_gpio, 1);
		mdelay(10);

		return 0;
	}

	return -EINVAL;
}

static int bs_tv_platform_probe(struct platform_device *pdev)
{
	int err = 0;
	struct bs_tv *dvb;
	struct device_node *of = pdev->dev.of_node;

	pr_info(DRV_NAME ": probe\n");

	dvb = kzalloc(sizeof(struct bs_tv), GFP_KERNEL);
	if (dvb == NULL) {
		printk(KERN_WARNING "bs_tv_dvb: memory allocation failed\n");
		return -ENOMEM;
	}

	if (!of) {
		dev_err(&pdev->dev, "can't find OF node\n");
		goto fail;
	}

	dvb->fe_pwr_gpio = of_get_named_gpio_flags(of, "fe-power-gpio", 0, NULL);
	dvb->demod_reset_gpio = of_get_named_gpio_flags(of, "demod-reset-gpio", 0, NULL);
	dvb->tuner_reset_gpio = of_get_named_gpio_flags(of, "tuner-reset-gpio", 0, NULL);

	if (dvb->fe_pwr_gpio == -EPROBE_DEFER || dvb->demod_reset_gpio == -EPROBE_DEFER || dvb->tuner_reset_gpio == -EPROBE_DEFER) {
		err = -EPROBE_DEFER;
		goto fail;
	}

	pr_info(DRV_NAME ": tuner_reset_gpio=%d\n", dvb->tuner_reset_gpio);
	pr_info(DRV_NAME ": demod_reset_gpio=%d\n", dvb->demod_reset_gpio);
	pr_info(DRV_NAME ": fe_pwr_gpio=%d\n", dvb->fe_pwr_gpio);

	/* Power everything up
	*/
	err = gpio_request_one(dvb->fe_pwr_gpio, GPIOF_OUT_INIT_LOW, DRV_NAME ":fe_pwr");
	if (err != 0)
		goto fail;
	err = gpio_request_one(dvb->demod_reset_gpio, GPIOF_OUT_INIT_LOW, DRV_NAME ":demod_reset");
	if (err != 0)
		goto fail;
	err = gpio_request_one(dvb->tuner_reset_gpio, GPIOF_OUT_INIT_LOW, DRV_NAME ":tuner_reset");
	if (err != 0)
		goto fail;

	gpio_set_value(dvb->fe_pwr_gpio, 1);
	msleep(30);

	msleep(10);
	gpio_set_value(dvb->demod_reset_gpio, 1);

	msleep(10);
	gpio_set_value(dvb->tuner_reset_gpio, 1);
	msleep(10);

	struct device_node *i2c_np = of_parse_phandle(of, "i2c-bus", 0);
	if (!i2c_np) {
		pr_err(DRV_NAME ": failed to locate I2C bus\n");
		err = -ENODEV;
		goto fail;
	}

	dvb->i2c_adap = of_find_i2c_adapter_by_node(i2c_np);
	if (!dvb->i2c_adap) {
		pr_err(DRV_NAME ": failed to locate I2C adapter\n");
		err = -ENODEV;
		goto fail;
	}

	dvb->frontend = dvb_attach(lgdt3305_attach, &bs_lgdt3305_config, dvb->i2c_adap);
	if (dvb->frontend == NULL) {
		bs_lgdt3305_config.i2c_addr = 0x59;
		lgdt3305_attach(&bs_lgdt3305_config, dvb->i2c_adap);
	}

	if (dvb->frontend != NULL) {
		if(dvb_attach(xc5000_attach, dvb->frontend, dvb->i2c_adap, &bs_xc5000_config) == 0)
		{
			printk(KERN_WARNING "bs_tv_dvb: Line %d\n", __LINE__);
			err = -EINVAL;
			goto fail;
		}
	} else {
		err = -EINVAL;
		goto fail;
	}
	/* define general-purpose callback pointer */
	dvb->frontend->callback = bs_tv_callback;

	/* register everything */
	err = dvb_register_adapter(&dvb->adapter, pdev->name, THIS_MODULE, &pdev->dev, adapter_nr);
	if (err < 0)  {
		printk(KERN_WARNING
		       "%s: dvb_register_adapter failed (errno = %d)\n",
		       pdev->name, err);
		goto fail;
	}

	dvb->adapter.priv = &pdev->dev;

	/* register frontend */
	err = dvb_register_frontend(&dvb->adapter, dvb->frontend);
	if (err < 0)  {
		printk(KERN_WARNING
		       "%s: dvb_register_frontend failed (errno = %d)\n",
		       pdev->name, err);
		goto fail;
	}

	return 0;

fail:
	/* We don't clean up very well here but this shouldn't really happen */
	kfree(dvb);
	return err;
}

static int bs_tv_platform_remove(struct platform_device *pdev)
{
	BUG();
	return 0;
}

static const struct of_device_id bs_tv_of_match[] = {
	{ .compatible = "brightsign,bs-tv" },
	{},
};

static struct platform_driver bs_tv_platform_driver = {
	.driver 	= {
		.name	= "bs_tv_platform",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(bs_tv_of_match),
	},
	.probe		= bs_tv_platform_probe,
	.remove		= bs_tv_platform_remove,
};

static int __init bs_tv_platform_module_init(void)
{
	return platform_driver_register(&bs_tv_platform_driver);
}
late_initcall(bs_tv_platform_module_init);

static void __exit bs_tv_platform_module_exit(void)
{
	platform_driver_unregister(&bs_tv_platform_driver);
}

module_exit(bs_tv_platform_module_exit);

MODULE_DESCRIPTION("BrightSign TV Platform driver");
MODULE_AUTHOR("BrightSign Digital Ltd.");
MODULE_LICENSE("GPL v2");
