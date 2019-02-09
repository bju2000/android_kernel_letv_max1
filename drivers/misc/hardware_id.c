/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/kobject.h>

struct msm8994_hardware_id {
	struct device *dev;
	unsigned int hardware_id1_gpio;
	unsigned int hardware_id2_gpio;
	unsigned int hardware_id3_gpio;
	int hardware_id1_value;
	int hardware_id2_value;
	int hardware_id3_value;
};
static struct msm8994_hardware_id *hardwaredata = NULL;
static struct kobject *hardware_obj = NULL;

static ssize_t hardware_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d%d%d", hardwaredata->hardware_id1_value, hardwaredata->hardware_id2_value, hardwaredata->hardware_id3_value);
}

static struct kobj_attribute hardware_id_attr = {
	.attr	= {
		.name = __stringify(hardware_id),
		.mode = 0444,
	},
	.show	= hardware_id_show,
	.store	= NULL,
};

static struct attribute * g[] = {
	&hardware_id_attr.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int msm8994_hardware_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct msm8994_hardware_id *hardware;
	int ret;

	hardware = devm_kzalloc(&pdev->dev, sizeof(*hardware), GFP_KERNEL);
	if (!hardware)
		return -ENOMEM;
	hardwaredata = hardware;
	hardware->dev = &pdev->dev;
	platform_set_drvdata(pdev, hardware);

	ret = of_get_named_gpio(node, "qcom,hardware-id1-gpio", 0);

	if (!gpio_is_valid(ret)) {
		dev_err(hardware->dev, "unable to get valid gpio\n");
		goto err_free;
	}

	hardware->hardware_id1_gpio = ret;

	ret = gpio_request(hardware->hardware_id1_gpio, "hardware_id1_gpio");

	if (ret) {
		dev_err(hardware->dev, "request gpio 11 failed\n");
		goto err_gpio1;
	}

	ret = gpio_direction_input(hardware->hardware_id1_gpio);

	if (ret) {
		dev_err(hardware->dev, "set gpio 11 input failed\n");
		goto err_gpio1;
	}

	hardware->hardware_id1_value = gpio_get_value(hardware->hardware_id1_gpio);

	ret = of_get_named_gpio(node, "qcom,hardware-id2-gpio", 0);

	if (!gpio_is_valid(ret)) {
		dev_err(hardware->dev, "unable to get valid gpio\n");
		goto err_free;
	}

	hardware->hardware_id2_gpio = ret;

	ret = gpio_request(hardware->hardware_id2_gpio, "hardware_id2_gpio");

	if (ret) {
		dev_err(hardware->dev, "request gpio 12 failed\n");
		goto err_gpio2;
	}

	ret = gpio_direction_input(hardware->hardware_id2_gpio);

	if (ret) {
		dev_err(hardware->dev, "set gpio 12 input failed\n");
		goto err_gpio2;
	}

	hardware->hardware_id2_value = gpio_get_value(hardware->hardware_id2_gpio);

	ret = of_get_named_gpio(node, "qcom,hardware-id3-gpio", 0);

	if (!gpio_is_valid(ret)) {
		dev_err(hardware->dev, "unable to get valid gpio\n");
		goto err_free;
	}

	hardware->hardware_id3_gpio = ret;

	ret = gpio_request(hardware->hardware_id3_gpio, "hardware_id3_gpio");

	if (ret) {
		dev_err(hardware->dev, "requset gpio 16 failed\n");
		goto err_gpio3;
	}

	ret = gpio_direction_input(hardware->hardware_id3_gpio);

	if (ret) {
		dev_err(hardware->dev, "set gpio 16 input failed\n");
		goto err_gpio3;
	}

	hardware->hardware_id3_value = gpio_get_value(hardware->hardware_id3_gpio);

	hardware_obj = kobject_create_and_add("hardware", NULL);

	if (!hardware_obj) {
		dev_err(hardware->dev, "create hardware kobject failed\n");
		goto err_gpio3;
	}

	ret = sysfs_create_group(hardware_obj, &attr_group);

	if (ret) {
		dev_err(hardware->dev, "create sys failed\n");
		goto err_kobj;
	}

	return 0;

err_kobj:
	kobject_put(hardware_obj);
err_gpio3:
	gpio_free(hardwaredata->hardware_id3_gpio);
err_gpio2:
	gpio_free(hardwaredata->hardware_id2_gpio);
err_gpio1:
	gpio_free(hardwaredata->hardware_id1_gpio);
err_free:
	devm_kfree(&pdev->dev, hardware);
	hardwaredata = NULL;

	return -EINVAL;
}

static int msm8994_hardware_remove(struct platform_device *pdev)
{
	struct msm8994_hardware_id *hardware = hardwaredata;
	sysfs_remove_group(hardware_obj, &attr_group);
	kobject_put(hardware_obj);
	gpio_free(hardware->hardware_id1_gpio);
	gpio_free(hardware->hardware_id2_gpio);
	gpio_free(hardware->hardware_id3_gpio);
	devm_kfree(&pdev->dev, hardware);
	hardwaredata = NULL;
	return 0;
}

static struct of_device_id of_match_table[] = {
	{ .compatible = "qcom,hardware-id", },
	{ },
};

static struct platform_driver msm8994_hardware_driver = {
	.driver         = {
		.name   = "hardware-id-driver",
		.of_match_table = of_match_table,
	},
	.probe          = msm8994_hardware_probe,
	.remove		= msm8994_hardware_remove,
};

module_platform_driver(msm8994_hardware_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM8994 hardware id driver");
