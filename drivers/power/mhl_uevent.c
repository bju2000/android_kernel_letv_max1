/*
 * linux/drivers/power/mhl_uevent.c
 *
 * Copyright 2004 LeTV <www.letv.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */
#include <linux/init.h>

#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include <linux/string.h>

#include <linux/export.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/user_namespace.h>
#include <linux/switch.h>

#include <linux/power/mhl_uevent.h>

static struct switch_dev *mhldev = NULL;

int mhl_kobj_init(void)
{
	int err;

	if (mhldev != NULL) {
		pr_err("mhl device have registered.\n");
		return 0;
	}

	mhldev = kzalloc(sizeof(*mhldev), GFP_KERNEL);
	if (!mhldev) {
		pr_err("mhl device memory allocation failed.\n");
		return -ENOMEM;
	}

	mhldev->name = "mhl";

	err = switch_dev_register(mhldev);
	if (err) {
		kfree(mhldev);
		mhldev = NULL;

		pr_err("mhldev switch registration failed\n");
	}

	return 0;
}

int mhl_kobj_exit(void)
{
	if (mhldev) {
		switch_dev_unregister(mhldev);

                kfree(mhldev);
                mhldev = NULL;
        }
	return 0;
}

bool mhl_state_uevent(u16 mhl_adc, u8 mhl_disconnected)
{
	static int mhl_plug_in;
	int mhl_detect_flag = 0;

	pr_err("mhl_plug_in = %d, mhl_adc = 0x%x, mhl_disconnected = %d\n",
		mhl_plug_in, mhl_adc, mhl_disconnected);

	if ((mhl_adc <= 0x120) && (mhl_disconnected == 0)) {
		mhl_plug_in = 1;
		mhl_detect_flag = 1;

		if (mhldev) {
			pr_err("the mhl state is set\n");
			switch_set_state(mhldev, 1);
		}
	}

	if (mhl_disconnected == 1 && mhl_plug_in == 1) {
		mhl_plug_in = 0;
		if (mhldev)
		{
			pr_err("the mhl state is clear\n");
			switch_set_state(mhldev, 0);
		}
	}

	return mhl_detect_flag == 1;
}
EXPORT_SYMBOL(mhl_state_uevent);
