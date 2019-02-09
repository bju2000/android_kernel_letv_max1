/*
 * Copyright (C) 2013 NXP Semicoductors N.V.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include "pn547.h"
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/miscdevice.h>


#define MAX_BUFFER_SIZE 512
#define NFC_DEBUG 0

/* enable LDO */
/*struct pvdd_info {
        const char * const name;
        struct regulator *regulator;
};
struct pvdd_info regulators = {"pvdd", NULL}; */


struct pn547_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct device		*dev;
	struct i2c_client	*client;
	struct miscdevice	pn547_device;
	unsigned int		ven_gpio;
	unsigned int		firm_gpio;
	unsigned int		irq_gpio;
	unsigned int		clk_req_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	bool			do_reading;
	bool			cancel_read;
};

char reset_os_cmd[] = {0x20, 0x00, 0x01, 0x00};

//static struct clk *clk_rf;
static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;
#if NFC_DEBUG
	pr_info("%s ++ \n", __func__);
#endif
	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (pn547_dev->irq_enabled) {
		disable_irq_nosync(pn547_dev->client->irq);
		pn547_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;
#if NFC_DEBUG
	pr_info("%s ++ \n", __func__);
#endif

	if (gpio_get_value(pn547_dev->irq_gpio) != 1)
		return IRQ_HANDLED;

	pn547_disable_irq(pn547_dev);

	pn547_dev->do_reading = 1;

	/* Wake up waiting readers */
	wake_up(&pn547_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

#if NFC_DEBUG
	pr_info("%s ++ \n", __func__);
#endif

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

#if NFC_DEBUG
	pr_info("%s : read request for %zu bytes.\n", __func__, count);
#endif

	mutex_lock(&pn547_dev->read_mutex);
	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

#if NFC_DEBUG
		pr_info("Waiting for PN547 IRQ.\n");
#endif
		pn547_dev->irq_enabled = true;
		pn547_dev->do_reading = 0;
		enable_irq(pn547_dev->client->irq);
		ret = wait_event_interruptible(pn547_dev->read_wq,
				pn547_dev->do_reading);

		pn547_disable_irq(pn547_dev);
#if NFC_DEBUG
		pr_info("PN547 IRQ high.\n");
#endif

		if (pn547_dev->cancel_read) {
			pn547_dev->cancel_read = false;
			ret = -1;
			goto fail;
		}

		if (ret)
			goto fail;

	}

	/* Read data */
	ret = i2c_master_recv(pn547_dev->client, tmp, count);
	mutex_unlock(&pn547_dev->read_mutex);

#if NFC_DEBUG
	pr_info("%s : i2c read %zu bytes. status : %d\n", __func__, count, ret);
#endif
	if (ret < 0) {
		pr_err("%s: PN547 i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}

	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}

	if (copy_to_user(buf, tmp, ret)) {
#if NFC_DEBUG
		pr_warning("%s : failed to copy to user space\n", __func__);
#endif
		return -EFAULT;
	}

	printk("%s :",__func__);
	for(i = 0; i < count; i++) 
	{
		printk(" %02X\n", tmp[i]);
	}
	printk("\n");

	return ret;

fail:
	mutex_unlock(&pn547_dev->read_mutex);
	return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev  *pn547_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

#if NFC_DEBUG
	pr_info("%s ++ \n", __func__);
#endif

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

#if NFC_DEBUG
	pr_info("%s : writing %zu bytes.\n", __func__, count);
#endif
	/* Write data */
	ret = i2c_master_send(pn547_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	printk("%s :",__func__);
	for(i = 0; i < count; i++) 
	{
		printk(" %02X\n", tmp[i]);
	}
	printk("\n");

	return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = container_of(filp->private_data,
						struct pn547_dev,
						pn547_device);

	filp->private_data = pn547_dev;
#if NFC_DEBUG
	pr_err("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));
#endif

	return 0;
}

static long pn547_dev_ioctl(struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	struct pn547_dev *pn547_dev = filp->private_data;

#if NFC_DEBUG
	pr_err("%s ++: firm = %d; ven= %d; irq= %d \n", __func__, pn547_dev->firm_gpio,pn547_dev->ven_gpio,pn547_dev->irq_gpio);
#endif

	switch (cmd) {
	case PN547_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
#if NFC_DEBUG
			pr_err("%s power on with firmware\n", __func__);
#endif
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
			gpio_set_value_cansleep(pn547_dev->firm_gpio, 1);
			msleep(60);
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 0);
			msleep(60);
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
			msleep(60);
		} else if (arg == 1) {
			/* power on */
#if NFC_DEBUG
			pr_err("%s power on\n", __func__);
#endif
			gpio_set_value_cansleep(pn547_dev->firm_gpio, 0);
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
			irq_set_irq_wake(pn547_dev->client->irq,1);
			msleep(20);
		} else  if (arg == 0) {
			/* power off */
#if NFC_DEBUG
			pr_err("%s power off\n", __func__);
#endif
			gpio_set_value_cansleep(pn547_dev->firm_gpio, 0);
			gpio_set_value_cansleep(pn547_dev->ven_gpio, 0);
                        irq_set_irq_wake(pn547_dev->client->irq,0);
			msleep(60);
		} else if (arg == 3) {
#if NFC_DEBUG
			pr_info("%s Read Cancel\n", __func__);
#endif
			pn547_dev->cancel_read = true;
			pn547_dev->do_reading = 1;
			wake_up(&pn547_dev->read_wq);
		} else {
#if NFC_DEBUG
			pr_err("%s bad arg %lu\n", __func__, arg);
#endif
			return -EINVAL;
		}
		break;
	default:
#if NFC_DEBUG
		pr_err("%s bad ioctl %u\n", __func__, cmd);
#endif
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn547_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn547_dev_read,
	.write	= pn547_dev_write,
	.open	= pn547_dev_open,
	.unlocked_ioctl	= pn547_dev_ioctl,
	.compat_ioctl = pn547_dev_ioctl,
};

static int pn547_parse_dt(struct device *dev,
			 struct pn547_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = of_get_named_gpio(np, "nxp,irq-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"nxp,irq-gpio\"\n");
		goto err;
	}
	pdata->irq_gpio = ret;

	ret = of_get_named_gpio(np, "nxp,dwld-en", 0);
	if (ret < 0) {
		pr_err("failed to get \"nxp,dwld-en\"\n");
		goto err;
	}
	pdata->fwdl_en_gpio = ret;

	ret = of_get_named_gpio(np, "nxp,ven", 0);
	if (ret < 0) {
		pr_err("failed to get \"nxp,ven\"\n");
		goto err;
	}
	pdata->ven_gpio = ret;

	ret = of_get_named_gpio(np, "nxp,clk-req", 0);
	if (ret < 0) {
		pr_err("failed to get \"nxp,clk-req\"\n");
		goto err;
	}
	pdata->clk_req_gpio = ret;

err:
	return ret;
}

static int pn547_gpio_request(struct device *dev,
				struct pn547_i2c_platform_data *pdata)
{
	int ret;
	ret = gpio_request(pdata->irq_gpio, "pn547_irq");
	if (ret)
		goto err_irq;
	ret = gpio_direction_input(pdata->irq_gpio);
	if (ret)
		goto err_irq_dir;
	ret = gpio_request(pdata->fwdl_en_gpio, "pn547_fw");
	if (ret)
		goto err_fwdl_en;
        ret = gpio_direction_output(pdata->fwdl_en_gpio, 1);
        if (ret)
                goto err_fwdl_dir;
	ret = gpio_request(pdata->ven_gpio, "pn547_ven");
	if (ret)
		goto err_ven;
        ret = gpio_direction_output(pdata->ven_gpio, 1);
        if (ret)
                goto err_ven_dir;
	ret = gpio_request(pdata->clk_req_gpio, "pn547_clk_req");
	if (ret)
		goto err_clk_req;

	ret = gpio_direction_input(pdata->clk_req_gpio);
	if (ret)
		goto err_clk_dir;

	return 0;

err_clk_dir:
        gpio_free(pdata->clk_req_gpio);
err_clk_req:
err_ven_dir:
	gpio_free(pdata->ven_gpio);
err_ven:
err_fwdl_dir:
	gpio_free(pdata->fwdl_en_gpio);
err_fwdl_en:
err_irq_dir:
	gpio_free(pdata->irq_gpio);
err_irq:

	pr_err("%s: gpio request err %d\n", __func__, ret);
	return ret;
}

static void pn547_gpio_release(struct pn547_i2c_platform_data *pdata)
{
	gpio_free(pdata->ven_gpio);
	gpio_free(pdata->irq_gpio);
	gpio_free(pdata->fwdl_en_gpio);
	gpio_free(pdata->clk_req_gpio);
}

static int pn547_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
        int irqn = 0;
	struct clk *nfc_clk;
	struct pn547_i2c_platform_data *platform_data;
	struct pn547_dev *pn547_dev;

#if NFC_DEBUG
	pr_info("%s ++\n", __func__);
#endif
	platform_data = kzalloc(sizeof(struct pn547_i2c_platform_data),
				GFP_KERNEL);

	if (platform_data == NULL) {
#if NFC_DEBUG
		pr_err("%s : nfc probe failed\n", __func__);
#endif
		ret = -ENOMEM;
		goto err_platform_data;
	}

	ret = pn547_parse_dt(&client->dev, platform_data);
	if (ret < 0) {
#if NFC_DEBUG
		pr_err("failed to parse device tree: %d\n", ret);
#endif
		goto err_parse_dt;
	}

	nfc_clk  = clk_get(&client->dev, "ref_clk");
	if (nfc_clk == NULL) {
#if NFC_DEBUG
		pr_err("failed to get clk: %d\n", ret);
#endif
		goto err_parse_dt;
	}

	ret = clk_prepare_enable(nfc_clk);
	if (ret) {
#if NFC_DEBUG
		pr_err("failed to enable clk: %d\n", ret);
#endif
		goto err_parse_dt;
	}


	ret = pn547_gpio_request(&client->dev, platform_data);
	if (ret) {
#if NFC_DEBUG
		pr_err("failed to request gpio\n");
#endif
		goto err_gpio_request;
	}
	dev_dbg(&client->dev, "%s:\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
#if NFC_DEBUG
		pr_err("%s: i2c check failed\n", __func__);
#endif
		ret = -ENODEV;
		goto err_i2c;
	}

	pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
	if (pn547_dev == NULL) {
#if NFC_DEBUG
		pr_err("failed to allocate memory for module data\n");
#endif
		ret = -ENOMEM;
		goto err_exit;
	}

	pn547_dev->irq_gpio = platform_data->irq_gpio;
	pn547_dev->ven_gpio  = platform_data->ven_gpio;
	pn547_dev->firm_gpio  = platform_data->fwdl_en_gpio;
	pn547_dev->clk_req_gpio = platform_data->clk_req_gpio;
	pn547_dev->client   = client;
	pn547_dev->dev = &client->dev;
	pn547_dev->do_reading = 0;

	gpio_set_value_cansleep(pn547_dev->firm_gpio, 0);
	gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
	msleep(60);
	gpio_set_value_cansleep(pn547_dev->ven_gpio, 0);
	msleep(60);
	gpio_set_value_cansleep(pn547_dev->ven_gpio, 1);
	msleep(60);

	i2c_master_send(client, reset_os_cmd, sizeof(reset_os_cmd));
	msleep(100);

	gpio_set_value_cansleep(pn547_dev->ven_gpio, 0);

	/* Initialise mutex and work queue */
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);

	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
	pn547_dev->pn547_device.name = "pn547";
	pn547_dev->pn547_device.fops = &pn547_dev_fops;

	ret = misc_register(&pn547_dev->pn547_device);
	if (ret) {
#if NFC_DEBUG
		pr_err("%s : misc_register failed\n", __FILE__);
#endif
		goto err_misc_register;
	}

/* BEGIN, LeTV yuyan@letv.com, MOBILEP-27, 20140820 */
#if 0
	regulators.regulator = regulator_get(&client->dev, regulators.name);
	if (IS_ERR(regulators.regulator)) {
		ret = PTR_ERR(regulators.regulator);
		pr_err("regulator get of %s failed (%d)\n", regulators.name, ret);
	} else {
		/* Enable the regulator */
		ret = regulator_enable(regulators.regulator);
		if (ret) {
			pr_err("pvdd %s enable failed (%d)\n",
				regulators.name, ret);
		}
	}
#endif
/* END, LeTV yuyan@letv.com, MOBILEP-27, 20140820 */

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */

	pn547_dev->irq_enabled = true;

	irqn = gpio_to_irq(platform_data->irq_gpio);
	if (irqn < 0) {
		goto err_to_irq;
	}
	client->irq = irqn;

#if NFC_DEBUG
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
#endif

	ret = request_irq(client->irq, pn547_dev_irq_handler,
			IRQF_TRIGGER_RISING /*| IRQF_ONESHOT*/,
			client->name, pn547_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	pn547_disable_irq(pn547_dev);
	i2c_set_clientdata(client, pn547_dev);

	return 0;

err_request_irq_failed:
	misc_deregister(&pn547_dev->pn547_device);

err_misc_register:
	mutex_destroy(&pn547_dev->read_mutex);
	kfree(pn547_dev);
err_to_irq:
err_exit:
err_i2c:
	pn547_gpio_release(platform_data);
err_gpio_request:
err_parse_dt:
	kfree(platform_data);
err_platform_data:
#if NFC_DEBUG
	pr_info("%s: err %d\n", __func__, ret);
#endif
	return ret;
}

static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev;

#if NFC_DEBUG
	pr_info("%s ++ \n", __func__);
#endif
	pn547_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn547_dev);
	misc_deregister(&pn547_dev->pn547_device);
	mutex_destroy(&pn547_dev->read_mutex);
	gpio_free(pn547_dev->irq_gpio);
	gpio_free(pn547_dev->ven_gpio);
	gpio_free(pn547_dev->firm_gpio);
	gpio_free(pn547_dev->clk_req_gpio);
	kfree(pn547_dev);

	return 0;
}

static const struct i2c_device_id pn547_id[] = {
	{ "pn547", 0 },
	{ }
};

static struct of_device_id pn547_match_table[] = {
	{ .compatible = "nxp,pn547", },
	{ },
};

static struct i2c_driver pn547_driver = {
	.id_table	= pn547_id,
	.probe		= pn547_probe,
	.remove		= pn547_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn547",
		.of_match_table	= pn547_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init pn547_dev_init(void)
{
	return i2c_add_driver(&pn547_driver);
}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
#if NFC_DEBUG
	pr_info("Unloading pn547 driver\n");
#endif
	i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_AUTHOR("SERI");
MODULE_DESCRIPTION("NFC pn547 driver");
MODULE_LICENSE("GPL");
