// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/*
 * TM1637 7-segment display driver
 *
 * Copyright (C) 2025 Siratul Islam <email@sirat.me>
 *
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/map_to_7segment.h>

/* TM1637 protocol commands */
#define TM1637_CMD_DATA_AUTO_INC 0x40
#define TM1637_CMD_ADDR_BASE 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x80

/* Display control register bits */
#define TM1637_DISPLAY_ON BIT(3)
#define TM1637_BRIGHTNESS_MASK GENMASK(2, 0)
#define TM1637_BRIGHTNESS_MAX 7

/* Timing parameters (microseconds) */
#define TM1637_BIT_DELAY_US 100

/* Number of digits in display */
#define TM1637_DIGITS 4

struct tm1637
{
	struct device *dev;
	struct gpio_desc *clk;
	struct gpio_desc *dio;
	u8 brightness;
	bool enabled;
	u8 buf[TM1637_DIGITS];
	struct seg7_conversion_map map;
};

static const SEG7_DEFAULT_MAP(initial_map);

static void tm1637_udelay(void)
{
	udelay(TM1637_BIT_DELAY_US);
}

static void tm1637_start(struct tm1637 *tm)
{
	gpiod_direction_output(tm->dio, 1);
	gpiod_set_value(tm->clk, 1);
	tm1637_udelay();
	gpiod_set_value(tm->dio, 0);
	tm1637_udelay();
	gpiod_set_value(tm->clk, 0);
	tm1637_udelay();
}

static void tm1637_stop(struct tm1637 *tm)
{
	gpiod_direction_output(tm->dio, 0);
	gpiod_set_value(tm->clk, 1);
	tm1637_udelay();
	gpiod_set_value(tm->dio, 1);
	tm1637_udelay();
}

static bool tm1637_write_byte(struct tm1637 *tm, u8 data)
{
	bool ack;
	int i;

	for (i = 0; i < 8; i++)
	{
		gpiod_set_value(tm->clk, 0);
		tm1637_udelay();

		if (data & BIT(i))
			gpiod_direction_input(tm->dio);
		else
			gpiod_direction_output(tm->dio, 0);

		tm1637_udelay();
		gpiod_set_value(tm->clk, 1);
		tm1637_udelay();
	}

	gpiod_set_value(tm->clk, 0);
	gpiod_direction_input(tm->dio);
	tm1637_udelay();

	gpiod_set_value(tm->clk, 1);
	tm1637_udelay();

	ack = !gpiod_get_value(tm->dio);

	if (!ack)
		gpiod_direction_output(tm->dio, 0);

	tm1637_udelay();
	gpiod_set_value(tm->clk, 0);

	return ack;
}

static void tm1637_update_display(struct tm1637 *tm)
{
	u8 ctrl_cmd;
	int i;

	tm1637_start(tm);
	tm1637_write_byte(tm, TM1637_CMD_DATA_AUTO_INC);
	tm1637_stop(tm);

	tm1637_start(tm);
	tm1637_write_byte(tm, TM1637_CMD_ADDR_BASE);
	for (i = 0; i < TM1637_DIGITS; i++)
		tm1637_write_byte(tm, tm->buf[i]);
	tm1637_stop(tm);

	ctrl_cmd = TM1637_CMD_DISPLAY_CTRL | (tm->brightness & TM1637_BRIGHTNESS_MASK);
	if (tm->enabled)
		ctrl_cmd |= TM1637_DISPLAY_ON;

	tm1637_start(tm);
	tm1637_write_byte(tm, ctrl_cmd);
	tm1637_stop(tm);
}

/* Sysfs attributes */

static ssize_t message_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	return sprintf(buf, "\n");
}

static ssize_t message_store(struct device *dev, struct device_attribute *attr,
							 const char *buf, size_t count)
{
	struct tm1637 *tm = dev_get_drvdata(dev);
	size_t i, pos = 0;
	size_t len;
	u8 segment_data[TM1637_DIGITS] = {0};

	len = count;
	if (len > 0 && buf[len - 1] == '\n')
		len--;

	for (i = 0; i < len && pos < TM1637_DIGITS; i++)
	{
		char c = buf[i];

		if (c == '.')
			continue;

		segment_data[pos] = map_to_seg7(&tm->map, c);

		if (i + 1 < len && buf[i + 1] == '.')
			segment_data[pos] |= BIT(7);

		pos++;
	}

	memcpy(tm->buf, segment_data, sizeof(tm->buf));
	tm1637_update_display(tm);

	return count;
}
static DEVICE_ATTR_RW(message);

static ssize_t brightness_show(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	struct tm1637 *tm = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", tm->brightness);
}

static ssize_t brightness_store(struct device *dev,
								struct device_attribute *attr,
								const char *buf, size_t count)
{
	struct tm1637 *tm = dev_get_drvdata(dev);
	unsigned int brightness;
	int ret;

	ret = kstrtouint(buf, 10, &brightness);
	if (ret)
		return ret;

	if (brightness > TM1637_BRIGHTNESS_MAX)
		brightness = TM1637_BRIGHTNESS_MAX;

	if (tm->brightness != brightness)
	{
		tm->brightness = brightness;
		tm1637_update_display(tm);
	}

	return count;
}
static DEVICE_ATTR_RW(brightness);

static ssize_t enabled_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct tm1637 *tm = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", tm->enabled);
}

static ssize_t enabled_store(struct device *dev,
							 struct device_attribute *attr,
							 const char *buf, size_t count)
{
	struct tm1637 *tm = dev_get_drvdata(dev);
	bool enabled;
	int ret;

	ret = kstrtobool(buf, &enabled);
	if (ret)
		return ret;

	if (tm->enabled != enabled)
	{
		tm->enabled = enabled;
		tm1637_update_display(tm);
	}

	return count;
}
static DEVICE_ATTR_RW(enabled);

static ssize_t map_seg7_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct tm1637 *tm = dev_get_drvdata(dev);

	memcpy(buf, &tm->map, sizeof(tm->map));
	return sizeof(tm->map);
}

static ssize_t map_seg7_store(struct device *dev,
							  struct device_attribute *attr,
							  const char *buf, size_t count)
{
	struct tm1637 *tm = dev_get_drvdata(dev);

	if (count != sizeof(tm->map))
		return -EINVAL;

	memcpy(&tm->map, buf, sizeof(tm->map));
	return count;
}
static DEVICE_ATTR_RW(map_seg7);

static struct attribute *tm1637_attrs[] = {
	&dev_attr_message.attr,
	&dev_attr_brightness.attr,
	&dev_attr_enabled.attr,
	&dev_attr_map_seg7.attr,
	NULL};
ATTRIBUTE_GROUPS(tm1637);

static int tm1637_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tm1637 *tm;
	int ret;

	tm = devm_kzalloc(dev, sizeof(*tm), GFP_KERNEL);
	if (!tm)
		return -ENOMEM;

	tm->dev = dev;

	tm->clk = devm_gpiod_get(dev, "clk", GPIOD_OUT_LOW);
	if (IS_ERR(tm->clk))
		return dev_err_probe(dev, PTR_ERR(tm->clk),
							 "Failed to get clk GPIO\n");

	tm->dio = devm_gpiod_get(dev, "dio", GPIOD_OUT_LOW);
	if (IS_ERR(tm->dio))
		return dev_err_probe(dev, PTR_ERR(tm->dio),
							 "Failed to get dio GPIO\n");

	tm->brightness = TM1637_BRIGHTNESS_MAX;
	tm->enabled = true;
	tm->map = initial_map;

	ret = sysfs_create_groups(&dev->kobj, tm1637_groups);
	if (ret)
	{
		dev_err(dev, "Failed to create sysfs groups: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, tm);

	tm1637_update_display(tm);

	return 0;
}

static void tm1637_remove(struct platform_device *pdev)
{
	struct tm1637 *tm = platform_get_drvdata(pdev);

	sysfs_remove_groups(&tm->dev->kobj, tm1637_groups);

	tm->enabled = false;
	memset(tm->buf, 0, sizeof(tm->buf));
	tm1637_update_display(tm);
}

static const struct of_device_id tm1637_of_match[] = {
	{.compatible = "titanmec,tm1637"},
	{}};
MODULE_DEVICE_TABLE(of, tm1637_of_match);

static struct platform_driver tm1637_driver = {
	.probe = tm1637_probe,
	.remove = tm1637_remove,
	.driver = {
		.name = "tm1637",
		.of_match_table = tm1637_of_match,
	},
};
module_platform_driver(tm1637_driver);

MODULE_DESCRIPTION("TM1637 7-segment display driver");
MODULE_AUTHOR("Siratul Islam");
MODULE_LICENSE("GPL");