#include <linux/ratelimit.h>
#include <linux/timekeeping.h>
/*
 * KTZ Semiconductor KTZ8866 LED Driver
 *
 * Copyright (C) 2013 Ideas on board SPRL
 *
 * Contact: Zhang Teng <zhangteng3@xiaomi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt)	"ktz8866:[%s:%d] " fmt, __func__, __LINE__

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include "sheng_ktz8866.h"

static LIST_HEAD(ktz8866_dev_list);
static DEFINE_MUTEX(ktz8866_dev_list_mutex);
#define NORMAL_MAX_DVB 1630

static const struct regmap_config ktz8866_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ktz8866_write(struct ktz8866_led *ktz, unsigned int reg, u8 data)
{
	int ret;

	if(NULL != ktz) {
		ret = regmap_write(ktz->regmap, reg, data);
		if(ret < 0)
			pr_err("ktz write failed to access reg = %d, data = %d \n", reg, data);
	} else {
		pr_err("write missing ktz8866 \n ");
		ret = -EINVAL;
	}
	return ret;
}

static int ktz8866_read(struct ktz8866_led *ktz, unsigned int reg, unsigned int *data)
{
	int ret;

	if(NULL != ktz) {
		ret = regmap_read(ktz->regmap, reg, data);
		if(ret < 0)
			pr_err("ktz read failed to access reg = %d, data = %d", reg, *data);
	} else {
		pr_err("read missing ktz8866 \n ");
		ret = -EINVAL;
	}
	return ret;
}

static int ktz_update_status(struct ktz8866_led *ktz, unsigned int level)
{
	unsigned int scaled, brightness;
	int ret;

	if (level > BL_LEVEL_MAX_HBM)
		return -EINVAL;
	if (level <= BL_LEVEL_MAX)
		scaled = level * NORMAL_MAX_DVB / BL_LEVEL_MAX;
	else
		scaled = (level - 2048) * (2047 - NORMAL_MAX_DVB) / 2047 + NORMAL_MAX_DVB;
	brightness = mi_bl_level_remap[scaled];
	if (brightness == ktz->level)
		return 0;
	/* Keep the sheng register values, remap and bias-preserving enable bits. */
	if (!brightness) {
		ret = ktz8866_write(ktz, KTZ8866_DISP_BL_ENABLE, 0x3f);
		if (ret)
			return ret;
		ktz->ktz8866_status = false;
		usleep_range(10000, 10010);
	}
	ret = ktz8866_write(ktz, KTZ8866_DISP_BB_LSB, brightness & 7);
	if (ret)
		return ret;
	ret = ktz8866_write(ktz, KTZ8866_DISP_BB_MSB, (brightness >> 3) & 0xff);
	if (ret)
		return ret;
	if (brightness && !ktz->ktz8866_status) {
		ret = ktz8866_write(ktz, KTZ8866_DISP_BL_ENABLE, 0x7f);
		if (ret)
			return ret;
		ktz->ktz8866_status = true;
	}
	ktz->level = brightness;
	return 0;
}

static int ktz_get_brightness(struct ktz8866_led *ktz, unsigned int reg, unsigned int *data)
{
	return ktz8866_read(ktz, reg, data);
}

static const struct ktz_ops ops = {
	.update_status	= ktz_update_status,
	.get_brightness	= ktz_get_brightness,
};

int ktz8866_backlight_update_status(unsigned int level)
{
	struct ktz8866_led *ktz;
	int ret = 0, err;
	static DEFINE_RATELIMIT_STATE(trace_limit, HZ, 4);
	bool trace = ktime_get_boottime_seconds() < 120 && __ratelimit(&trace_limit);

	mutex_lock(&ktz8866_dev_list_mutex);
	if (list_empty(&ktz8866_dev_list))
		ret = -ENODEV;
	list_for_each_entry(ktz, &ktz8866_dev_list, entry) {
		err = ktz->ops->update_status(ktz, level);
		if (err && !ret)
			ret = err;
		if (trace) {
			unsigned int lsb, msb, enable;
			int readback = regmap_read(ktz->regmap, KTZ8866_DISP_BB_LSB, &lsb);
			if (!readback)
				readback = regmap_read(ktz->regmap, KTZ8866_DISP_BB_MSB, &msb);
			if (!readback)
				readback = regmap_read(ktz->regmap, KTZ8866_DISP_BL_ENABLE, &enable);
			if (!readback)
				do { } while (0);
		}
	}
	mutex_unlock(&ktz8866_dev_list_mutex);
	return ret;
}

/* Read actual I2C registers without providing a raw write interface. */
static ssize_t register_brightness_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	struct ktz8866_led *ktz = dev_get_drvdata(dev);
	unsigned int lsb, msb, enable;
	int ret;

	mutex_lock(&ktz8866_dev_list_mutex);
	ret = regmap_read(ktz->regmap, KTZ8866_DISP_BB_LSB, &lsb);
	if (!ret)
		ret = regmap_read(ktz->regmap, KTZ8866_DISP_BB_MSB, &msb);
	if (!ret)
		ret = regmap_read(ktz->regmap, KTZ8866_DISP_BL_ENABLE, &enable);
	if (!ret)
		ret = sysfs_emit(buf, "cached=%d actual=%u enable=0x%02x\n",
			ktz->level, ((msb & 0xff) << 3) | (lsb & 7), enable);
	mutex_unlock(&ktz8866_dev_list_mutex);
	return ret;
}
static DEVICE_ATTR_RO(register_brightness);
static struct attribute *ktz8866_attrs[] = {
	&dev_attr_register_brightness.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ktz8866);

static int ktz8866_probe(struct i2c_client *i2c)
{
	struct ktz8866_led *pdata;
	unsigned int enable;
	int ret;

	if (!i2c_check_functionality(i2c->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&i2c->dev, "ktz8866 I2C adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
		return -EIO;
	}
	pdata = devm_kzalloc(&i2c->dev,
			     sizeof(struct ktz8866_led), GFP_KERNEL);
	if (!pdata){
		pr_err("failed: out of memory \n");
		return -ENOMEM;
	}
	pdata->regmap = devm_regmap_init_i2c(i2c, &ktz8866_i2c_regmap_config);
	if (IS_ERR(pdata->regmap))
		return PTR_ERR(pdata->regmap);
	ret = regmap_read(pdata->regmap, KTZ8866_DISP_BL_ENABLE, &enable);
	if (ret)
		return ret;
	pdata->ktz8866_status = !!(enable & KTZ8866_backlight_EN_MASK);
	pdata->level = -1;
	pdata->dev = &i2c->dev;
	i2c_set_clientdata(i2c, pdata);
	pdata->ops = &ops;
	/* Probe must preserve the bootloader image. The actual panel reset
	 * path darkens these LEDs before invalidating scanout. */

	mutex_lock(&ktz8866_dev_list_mutex);
	list_add(&pdata->entry, &ktz8866_dev_list);
	mutex_unlock(&ktz8866_dev_list_mutex);
	dev_info(&i2c->dev, "probe sucess end\n");
	return ret;
}

static void ktz8866_remove(struct i2c_client *i2c)
{
	struct ktz8866_led *pdata = i2c_get_clientdata(i2c);

	mutex_lock(&ktz8866_dev_list_mutex);
	list_del(&pdata->entry);
	mutex_unlock(&ktz8866_dev_list_mutex);
}

static struct of_device_id ktz8866_match_table[] = {
	{ .compatible = "ktz,ktz8866",},
	{ },
};

static struct i2c_driver ktz8866_driver = {
	.driver = {
		.name = "ktz8866",
		.of_match_table = ktz8866_match_table,
		.dev_groups = ktz8866_groups,
	},
	.probe = ktz8866_probe,
	.remove = ktz8866_remove,
};

int sheng_ktz8866_init(void)
{
	return i2c_add_driver(&ktz8866_driver);
}

void sheng_ktz8866_exit(void)
{
	i2c_del_driver(&ktz8866_driver);
}
