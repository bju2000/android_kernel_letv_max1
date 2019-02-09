/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/usb/fusb300.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#define DRIVER_NAME "usb_typec_i2c"

#define REG_DEVICE_ID	0X01
#define REG_SWITCHES_0	0X02
#define REG_SWITCHES_1	0X03
#define REG_MEASURE		0X04
#define REG_SLICE		0X05
#define REG_CONTROL_0	0X06
#define REG_CONTROL_1	0X07
#define REG_MASK_0		0X0A
#define REG_POWER		0X0B
#define REG_SRESET		0X0C
#define REG_STATUS_0	0X40
#define REG_STATUS_1	0X41
#define REG_INTERRUPT	0X42
#define REG_FIFOS		0X43

/* register status_0 */
#define STS0_VBUS_OK	(1<<7)
#define STS0_COMP		(1<<5)
#define STS0_BC_LVL		0x3

/* register interrupt */
#define I_COMP		(1<<5)

struct fusb300_dev_info {
/*
0: fusb is working.
1: fusb is not working.
*/
	int irq_working;
	struct delayed_work delay_enable_irq;
	u8 reg;
	char *name;
	int irq_gpio;
	int irq;
	int dev_id;
	bool dwc3_host_mode;
	enum fusb_work_mode fusb_mode;
	struct i2c_client *i2c_client;
	struct regulator *regulator_vdd;
	struct pinctrl *pinctrl;
	struct pinctrl_state *intr_active;
	struct mutex di_mutex;
	struct notifier_block fusb_notifier;
};

static struct fusb300_dev_info *dev_info_g;

static int usb_typec_regulator_init(struct fusb300_dev_info *fdi)
{
	return 0;
}

static int usb_typec_power_on(struct fusb300_dev_info *fdi)
{
	return 0;
}

static int usb_typec_gpio_config(struct fusb300_dev_info *fdi)
{
	int ret = 0;

	if (gpio_is_valid(fdi->irq_gpio))
		ret = gpio_request(fdi->irq_gpio, "fusb300_irq_gpio");

	if (ret) {
		dev_err(&fdi->i2c_client->dev, "unable to request gpio [%d]\n",
			fdi->irq_gpio);
		goto err_irq_gpio_req;
	}
	ret = gpio_direction_input(fdi->irq_gpio);
	if (ret) {
		dev_err(&fdi->i2c_client->dev,
			"unable to set direction for gpio [%d]\n",
			fdi->irq_gpio);
		goto err_irq_gpio_dir;
	}

	return ret;

err_irq_gpio_dir:
	if (gpio_is_valid(fdi->irq_gpio))
		gpio_free(fdi->irq_gpio);
err_irq_gpio_req:
	return ret;
}

static int usb_typec_parse_dt(struct device *dev, struct fusb300_dev_info *fdi)
{
	if (NULL != dev->of_node) {
		fdi->irq_gpio = of_get_named_gpio_flags(dev->of_node,
			"irq-gpio", 0, NULL);
	}

	return 0;
}

static int fusb300_write_i2c(struct i2c_client *client, u8 reg, u8 value)
{
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	data[0] = reg;
	data[1] = value;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = sizeof(data);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return 0;
}

static int fusb300_read_i2c(struct i2c_client *client, u8 reg)
{
	struct i2c_msg msg[2];
	u8 data;
	int ret;

	if ((NULL == client) || (NULL == client->adapter))
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &data;
	msg[1].len = sizeof(data);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
		return ret;
	}

	return (int)data;
}

static int fusb300_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;

	ret = fusb300_write_i2c(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, "%s write err:%d.\n", __func__, ret);

	return ret;
}

static int get_dev_id(struct i2c_client *client)
{
	int ret;
	u8 reg = 0x01;

	ret = fusb300_read_i2c(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);

	dev_info(&client->dev, "%s reg0x%x=0x%x.\n", __func__, reg, ret);

	return ret;
}

#define INT_MASK 0xDA
static int fusb300_enable_irq(struct i2c_client *client, bool enable)
{
	int ret;
	u8 val;

	if (!enable) {
		dev_info(&client->dev, "%s disable irq.\n", __func__);
		ret = fusb300_read_i2c(client, REG_CONTROL_0);
		if (ret < 0)
			fusb300_write_i2c(client, REG_CONTROL_0, 0x28);
		else {
			ret |= (1 << 5);
			val = (u8)ret;
			ret = fusb300_write_i2c(client, REG_CONTROL_0, val);
		}
	} else {
		dev_info(&client->dev, "%s enable irq.\n", __func__);
		ret = fusb300_read_i2c(client, REG_CONTROL_0);
		if (ret < 0)
			fusb300_write_i2c(client, REG_CONTROL_0, 0x08);
		else {
			ret &= ~(1 << 5);
			val = (u8)ret;
			ret = fusb300_write_i2c(client, REG_CONTROL_0, val);
		}
	}

	fusb300_write_i2c(client, REG_MASK_0, INT_MASK);

	return ret;
}

static void fusb300_set_mode(struct fusb300_dev_info *fdi,
				enum fusb_work_mode fusb_mode)
{

	if (MD_DFP == fusb_mode) {
		fdi->fusb_mode = fusb_mode;
		fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0xcc);
		fusb300_write_reg(fdi->i2c_client, REG_MEASURE, 0x26);
		fusb300_write_reg(fdi->i2c_client, REG_CONTROL_0, 0x08);
		fusb300_write_reg(fdi->i2c_client, REG_POWER, 0x07);
		fusb300_read_i2c(fdi->i2c_client, REG_INTERRUPT);
		dev_info(&fdi->i2c_client->dev, "%s=MD_DFP.\n", __func__);
	} else if (MD_LPW == fusb_mode) {
		fdi->fusb_mode = fusb_mode;
		fusb300_write_reg(fdi->i2c_client, REG_CONTROL_0, 0x28);
		fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0x03);
		fusb300_write_reg(fdi->i2c_client, REG_POWER, 0x01);
		fusb300_read_i2c(fdi->i2c_client, REG_INTERRUPT);
		dev_info(&fdi->i2c_client->dev, "%s=MD_LPW.\n", __func__);
	} else if (MD_HALT == fusb_mode) {
		fdi->fusb_mode = fusb_mode;
		disable_irq(fdi->irq);
		fdi->irq_working = 0;
		fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0x00);
		dev_info(&fdi->i2c_client->dev, "%s=halt mode.\n", __func__);
	} else
		dev_err(&fdi->i2c_client->dev, "%s=unknow mode.\n", __func__);

	return;
}

void set_fusb300_mode(enum fusb_work_mode fusb_mode)
{
	if (NULL == dev_info_g)
		return;

	dev_err(&dev_info_g->i2c_client->dev, "%s=%d.\n", __func__, fusb_mode);
	fusb300_set_mode(dev_info_g, fusb_mode);

	return;
}

static int fusb_reboot(struct notifier_block *notify_block,
		unsigned long val, void *unused)
{
	struct fusb300_dev_info *fdi = container_of(
		notify_block, struct fusb300_dev_info, fusb_notifier);

	if (fdi) {
		fusb300_set_mode(fdi, MD_HALT);
		if (fdi->dwc3_host_mode)
			fusb300_set_msm_usb_host_mode(false);
	}

	return NOTIFY_OK;
}

static int is_valid_reg(u8 reg)
{
	if (reg >= 0x1 && reg <= 0xc)
		return 0;
	else if (reg >= 0x40 && reg <= 0x43)
		return 0;
	else if (0xff == reg)
		return 0;

	return -EINVAL;
}

static int fusb300_dump_regs(struct i2c_client *client)
{
	int ret, i;

	for (i = 1; i <= 0x7; i++) {
		ret = fusb300_read_i2c(client, (u8)i);
		dev_err(&client->dev, "REG0x%x=0x%x.\n", i, ret);
	}

	ret = fusb300_read_i2c(client, 0x0A);
	dev_err(&client->dev, "REG0x0a=0x%x.\n", ret);
	ret = fusb300_read_i2c(client, 0x0B);
	dev_err(&client->dev, "REG0x0b=0x%x.\n", ret);

	for (i = 0x40; i <= 0x43; i++) {
		ret = fusb300_read_i2c(client, (u8)i);
		dev_err(&client->dev, "REG0x%x=0x%x.\n", i, ret);
	}

	return ret;
}

static ssize_t fusb300_sysfs_show_votg(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	/* struct i2c_client *client = to_i2c_client(dev); */
	/* struct fusb300_dev_info *fdi = i2c_get_clientdata(client); */

	return snprintf(buf, 6, "votg.\n");
}

static ssize_t fusb300_sysfs_set_votg(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret, val;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	fusb300_set_msm_usb_host_mode(val);

	return count;
}

static ssize_t fusb300_sysfs_show_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct fusb300_dev_info *fdi = i2c_get_clientdata(client);

	return snprintf(buf, 4, "0x%x\n", fdi->fusb_mode);
}

static ssize_t fusb300_sysfs_set_mode(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static ssize_t fusb300_sysfs_show_regs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb300_dev_info *fdi = i2c_get_clientdata(client);

	if (NULL == fdi || NULL == client)
		return 0;

	fusb300_dump_regs(client);

	return snprintf(buf, 7, "value.\n");
}

static ssize_t fusb300_sysfs_set_regs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	u8 reg, val;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb300_dev_info *fdi = i2c_get_clientdata(client);

	if (NULL == fdi || NULL == client)
		return count;

	if (sscanf(buf, "%hhx %hhx", &reg, &val) != 2) {
		dev_err(&client->dev,
				"%s: get regs values: Invaild regs\n",
				__func__);
		return -EINVAL;
	}

	dev_err(&client->dev, "%s reg=0x%hhx,val=0x%hhx\n", __func__,
				reg, val);

	ret = is_valid_reg(reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s invalid reg:%d.\n", __func__,
					fdi->reg);
		return count;
	}

	ret = fusb300_write_i2c(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s write err:%d.\n", __func__, ret);
		return count;
	}

	return count;
}

static ssize_t fusb300_sysfs_show_chip_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb300_dev_info *fdi = i2c_get_clientdata(client);

	if (NULL == fdi)
		return -ENODEV;

	if (fdi->dev_id == 0x50 || fdi->dev_id == 0x60)
		return snprintf(buf, 10, "fusb300\n");
	else
		return snprintf(buf, 10, "unknow\n");
}

static DEVICE_ATTR(regs, 0644,
		   fusb300_sysfs_show_regs, fusb300_sysfs_set_regs);
static DEVICE_ATTR(mode, 0644,
		   fusb300_sysfs_show_mode, fusb300_sysfs_set_mode);
static DEVICE_ATTR(votg, 0644,
		   fusb300_sysfs_show_votg, fusb300_sysfs_set_votg);
static DEVICE_ATTR(chip_id, 0644,
		   fusb300_sysfs_show_chip_id, NULL);

static struct attribute *attrs[] = {
	&dev_attr_regs.attr,
	&dev_attr_mode.attr,
	&dev_attr_votg.attr,
	&dev_attr_chip_id.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static void fusb300_dfp_detect(struct fusb300_dev_info *info)
{
	struct fusb300_dev_info *fdi = info;
	int reg_sts0;
	enum ccpin_state cc1, cc2;

	fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0x44);
	reg_sts0 = fusb300_read_i2c(fdi->i2c_client, REG_STATUS_0);
	dev_info(&fdi->i2c_client->dev, "reg_status0=0x%x.\n", reg_sts0);
	if (reg_sts0 < 0)
		dev_err(&fdi->i2c_client->dev, "reg_status0=0x%x.\n", reg_sts0);
	else {
		if ((reg_sts0 & 0x20) == 0x20)
			cc1 = OPEN;
		else {
			if ((reg_sts0 & 0x03) == 0x00)
				cc1 = RA;
			else
				cc1 = RD;
		}
	}

	fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0x88);
	reg_sts0 = fusb300_read_i2c(fdi->i2c_client, REG_STATUS_0);
	dev_info(&fdi->i2c_client->dev, "reg_status0=0x%x.\n", reg_sts0);
	if (reg_sts0 < 0)
		dev_err(&fdi->i2c_client->dev, "reg_status0=0x%x.\n", reg_sts0);
	else {
		if ((reg_sts0 & 0x20) == 0x20)
			cc2 = OPEN;
		else {
			if ((reg_sts0 & 0x03) == 0x00)
				cc2 = RA;
			else
				cc2 = RD;
		}
	}

	dev_info(&fdi->i2c_client->dev,
			"%s:cc1=0x%x,cc2=0x%x.\n", __func__, cc1, cc2);
	if (((cc1 | cc2) == (RA | RD)) || (cc1 | cc2) == (RD | OPEN)) {
		if (!fdi->dwc3_host_mode) {
			fusb300_set_msm_usb_host_mode(true);
			fdi->dwc3_host_mode = true;
		}
	} else {
		if (fdi->dwc3_host_mode) {
			fusb300_set_msm_usb_host_mode(false);
			fdi->dwc3_host_mode = false;
		}
	}

	fusb300_set_mode(fdi, MD_DFP);

	return;
}

static irqreturn_t fusb300_irq_handler(int irq, void *info)
{
	int int_status, reg_sts0;
	struct fusb300_dev_info *fdi = info;

	if (NULL == fdi->i2c_client)
		return IRQ_NONE;

	if (time_before(jiffies, fusb300_get_checktime())) {
		dev_err(&fdi->i2c_client->dev, "%s: work round for usb charger",
				__func__);
		return IRQ_NONE;
	}

	/* must clear interrupt register */
	int_status = fusb300_read_i2c(fdi->i2c_client, REG_INTERRUPT);
	reg_sts0 = fusb300_read_i2c(fdi->i2c_client, REG_STATUS_0);
	dev_err(&fdi->i2c_client->dev, "reg_status0=0x%x, int_status=0x%x",
			reg_sts0, int_status);

	/* if not I_COMP_CHNG interrupt occured, just ignore it */
	if (!(int_status & 0x20) || (int_status < 0) || (reg_sts0 < 0)) {
		dev_info(&fdi->i2c_client->dev,
				"%s unhandled inttrrupt.\n", __func__);
		return IRQ_HANDLED;
	}

	fusb300_dfp_detect(fdi);

	return IRQ_HANDLED;
}

static void fusb300_delay_enable_irq(struct work_struct *w)
{
	struct fusb300_dev_info *fdi = dev_info_g;
	if ((fdi == NULL) || (fdi->irq == 0)) {
		dev_info(&fdi->i2c_client->dev,
				"%s err:fdi is NULL\n", __func__);
		return;
	}

	if (fdi->irq_working == 0) {
		enable_irq(fdi->irq);
		fdi->irq_working = 1;
	}

	return;
}

static int usb_typec_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct fusb300_dev_info *fdi = NULL;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}


	fdi = kzalloc(sizeof(struct fusb300_dev_info), GFP_KERNEL);
	if (!fdi) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for fusb300 device info\n",
				__func__);
		return -ENOMEM;
	}

	fdi->i2c_client = client;

	usb_typec_parse_dt(&client->dev, fdi);

	fdi->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(fdi->pinctrl)) {
		pr_err("%s: Unable to get pinctrl handle\n", __func__);
		goto err_gpio_config;
	}

	fdi->intr_active = pinctrl_lookup_state(fdi->pinctrl,
					"fusb300_active");
	if (IS_ERR(fdi->intr_active)) {
		pr_err("%s: could not get intr_active pinstate\n", __func__);
		goto err_gpio_config;
	}

	ret = pinctrl_select_state(fdi->pinctrl,
					fdi->intr_active);
	if (ret != 0) {
		pr_err("%s: Disable TLMM pins failed with %d\n",
			__func__, ret);
		ret = -EIO;
		goto err_gpio_config;
	}

	ret = usb_typec_gpio_config(fdi);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to configure gpios\n");
		goto err_gpio_config;
	}

	fdi->irq = gpio_to_irq(fdi->irq_gpio);

	ret = request_threaded_irq(fdi->irq, NULL,
		fusb300_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
		DRIVER_NAME, fdi);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request irq.\n");
		goto err_request_irq;
	}

	fdi->irq_working = 1;
	INIT_DELAYED_WORK(&fdi->delay_enable_irq, fusb300_delay_enable_irq);

	/* need request L21A_3P3 LDO */
	ret = usb_typec_regulator_init(fdi);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to init regulator.\n");
		goto err_request_regulator;
	}
	ret = usb_typec_power_on(fdi);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to power on.\n");
		goto err_power_on;
	}

	/* Register reboot notifier here */
	fdi->fusb_notifier.notifier_call = fusb_reboot;
	ret = register_reboot_notifier(&fdi->fusb_notifier);
	if (ret) {
		dev_err(&client->dev,
			"%s: cannot register reboot notifier(err = %d)\n",
			__func__, ret);
		goto err_power_on;
	}

	i2c_set_clientdata(client, fdi);

	fdi->dev_id = get_dev_id(client);
	if (fdi->dev_id != 0x50 && fdi->dev_id != 0x60) {
		dev_err(&client->dev, "%s unexpect device dev_id=%d,abort!\n",
					__func__, fdi->dev_id);
		goto err_unknow_dev;
	}

	fusb300_set_mode(fdi, MD_DFP);

	fusb300_enable_irq(client, true);

	ret = sysfs_create_group(&(&client->dev)->kobj, &attr_group);
	if (ret) {
		dev_err(&client->dev, "%s sysfs err:%d.\n", __func__, ret);
		goto err_creat_sysfs;
	}

	dev_info_g = fdi;

	return 0;

err_creat_sysfs:
err_unknow_dev:
	unregister_reboot_notifier(&fdi->fusb_notifier);
err_power_on:
	regulator_put(fdi->regulator_vdd);
err_request_regulator:
	free_irq(fdi->irq, fdi);
	fdi->irq_working = 0;
err_request_irq:
	if (gpio_is_valid(fdi->irq_gpio))
		gpio_free(fdi->irq_gpio);
err_gpio_config:
	kfree(fdi);
	fdi = NULL;
	return 0;
}

static int usb_typec_i2c_remove(struct i2c_client *client)
{
	struct fusb300_dev_info *fdi = i2c_get_clientdata(client);

	if (fdi) {
		unregister_reboot_notifier(&fdi->fusb_notifier);
		sysfs_remove_group(&(&client->dev)->kobj, &attr_group);
	}

	return 0;
}


static const struct i2c_device_id usb_typec_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, usb_typec_id_table);

#ifdef CONFIG_OF
static struct of_device_id typec_match_table[] = {
	{ .compatible = "fairchild,usb_type_c",},
	{ },
};
#else
#define typec_match_table NULL
#endif

#ifdef CONFIG_PM_SLEEP
static int fusb300_pm_suspend(struct device *dev)
{
	struct fusb300_dev_info *fdi = dev_info_g;
	int ret = 0;

	if (fdi == NULL) {
		dev_err(dev, "%s sysfs err: dev_info_g is NULL\n", __func__);
		return -EPERM;
	}

	cancel_delayed_work_sync(&fdi->delay_enable_irq);

	if (fdi->irq_working) {
		disable_irq(fdi->irq);
		fdi->irq_working = 0;
	}

	return ret;
}

static int fusb300_pm_resume(struct device *dev)
{
	struct fusb300_dev_info *fdi = dev_info_g;
	int ret = 0;

	if (fdi == NULL) {
		dev_err(dev, "%s err: dev_info_g is NULL\n", __func__);
		return -EPERM;
	}

	if (!fdi->irq_working)
		queue_delayed_work(system_nrt_wq, &fdi->delay_enable_irq,
							msecs_to_jiffies(10));

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fusb300_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fusb300_pm_suspend, fusb300_pm_resume)
};

static struct i2c_driver usb_typec_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &fusb300_pm_ops,
		.of_match_table = typec_match_table,
	},
	.probe = usb_typec_i2c_probe,
	.remove = usb_typec_i2c_remove,
	.id_table = usb_typec_id_table,
};

int usb_typec_init(void)
{
	return i2c_add_driver(&usb_typec_i2c_driver);
}

void usb_typec_exit(void)
{
	i2c_del_driver(&usb_typec_i2c_driver);

	return;
}

late_initcall(usb_typec_init);
module_exit(usb_typec_exit);

MODULE_AUTHOR("Letv, Inc.");
MODULE_DESCRIPTION("Letv usb-type-c I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
