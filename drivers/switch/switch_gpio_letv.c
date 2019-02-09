/*
 *  drivers/switch/switch_gpio_letv.c
 *
 * Copyright (C) 2015 Letv, Inc.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned int gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
};

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);

	state = gpio_get_value(data->gpio);
	dev_notice(data->sdev.dev, "Switch state: %d\n", state);
	switch_set_state(&data->sdev, state);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;

	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return snprintf(buf, PAGE_SIZE, "%s\n", state);

	return -EINVAL;
}

struct gpio_sw_platform_data {
	struct gpio_switch_data sw_data;
	const char *name;
};

#ifdef CONFIG_OF
static struct gpio_sw_platform_data *gpio_sw_parse_dt(struct device *dev)
{
	struct device_node *node;
	struct gpio_sw_platform_data *pdata;
	struct gpio_switch_data *switches;
	int error;
	int gpio;
	enum of_gpio_flags flags;

	node = dev->of_node;
	if (!node) {
		error = -ENODEV;
		goto err_out;
	}

	pdata = kzalloc(sizeof(struct gpio_sw_platform_data), GFP_KERNEL);
	if (!pdata) {
		error = -ENOMEM;
		goto err_out;
	}

	pdata->name = of_get_property(node, "switch-name", NULL);

	if (!of_find_property(node, "gpios", NULL)) {
		dev_err(dev, "Switch without gpios\n");
		error = -EINVAL;
		goto err_free_pdata;
	}

	gpio = of_get_gpio_flags(node, 0, &flags);
	if (gpio < 0) {
		error = gpio;
		if (error != -EPROBE_DEFER)
			dev_err(dev, "Failed to get gpio flags, error: %d\n",
					error);
		goto err_free_pdata;
	}

	switches = &pdata->sw_data;

	switches->gpio = gpio;

	if (of_find_property(node, "state_on", NULL))
		switches->state_on = of_get_property(node, "state_on", NULL);
	else
		switches->state_on = NULL;

	if (of_find_property(node, "state_off", NULL))
		switches->state_off = of_get_property(node, "state_off", NULL);
	else
		switches->state_off = NULL;

	return pdata;

err_free_pdata:
	kfree(pdata);
err_out:
	return ERR_PTR(error);
}

static struct of_device_id gpio_switch_of_match[] = {
	{ .compatible = "gpio-switches", },
	{ },
};

MODULE_DEVICE_TABLE(of, gpio_switch_of_match);

#else

static inline struct gpio_sw_platform_data *gpio_sw_parse_dt(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_sw_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;

	if (!pdata) {
		pdata = gpio_sw_parse_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
		pdev->dev.platform_data = pdata;
	}

	switch_data = &pdata->sw_data;
	switch_data->sdev.name = pdata->name;
	switch_data->sdev.print_state = switch_gpio_print_state;

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, gpio_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	dev_dbg(dev, "%s: gpio: %d, irq: %d\n", __func__,
			switch_data->gpio, switch_data->irq);

	ret = request_irq(switch_data->irq, gpio_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		pdev->name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
	switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:

	return ret;
}

static int gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_sw_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data = &pdata->sw_data;

	free_irq(switch_data->irq, switch_data);
	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_switch_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

static int gpio_switch_resume(struct device *dev)
{
	struct gpio_sw_platform_data *pdata = dev->platform_data;
	struct gpio_switch_data	*data = &pdata->sw_data;
	int gpio_state, sw_state;

	dev_dbg(dev, "%s\n", __func__);

	gpio_state = gpio_get_value(data->gpio);
	sw_state = switch_get_state(&data->sdev);
	if (gpio_state != sw_state)
		schedule_work(&data->work);

	return 0;
}

static SIMPLE_DEV_PM_OPS(gpio_switch_pm_ops, gpio_switch_suspend,
			gpio_switch_resume);
#endif

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= gpio_switch_remove,
	.driver		= {
		.name	= "switch-gpio",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	= &gpio_switch_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(gpio_switch_of_match),
#endif
	},
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_DESCRIPTION("Letv GPIO Switch driver");
MODULE_LICENSE("GPL");
