/*
 * rtc-ds1307.c - RTC driver for some mostly-compatible I2C chips.
 *
 *  Copyright (C) 2005 James Chapman (ds1337 core)
 *  Copyright (C) 2006 David Brownell
 *  Copyright (C) 2009 Matthias Fuchs (rx8025 support)
 *  Copyright (C) 2012 Bertrand Achard (nvram access fixes)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc/ds1307.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

/* RTC registers don't differ much, except for the century flag */
#define DS1307_REG_SECS		0x00	/* 00-59 */
#define DS1307_BIT_CH		0x80
#define DS1307_REG_MIN		0x01	/* 00-59 */
#define DS1307_REG_HOUR		0x02	/* 00-23, or 1-12{am,pm} */
#define DS1307_BIT_12HR		0x40	/* in REG_HOUR */
#define DS1307_BIT_PM		0x20	/* in REG_HOUR */
#define DS1307_REG_WDAY		0x03	/* 01-07 */
#define DS1307_REG_MDAY		0x04	/* 01-31 */
#define DS1307_REG_MONTH	0x05	/* 01-12 */
#define DS1337_BIT_CENTURY	0x80	/* in REG_MONTH */
#define DS1307_REG_YEAR		0x06	/* 00-99 */
#define DS1339_REG_ALARM1_SECS  0x07

/*
 * Other registers (control, status, alarms, trickle charge, NVRAM, etc)
 * start at 7, and they differ a LOT. Only control and status matter for
 * basic RTC date and time functionality; be careful using them.
 */
#define DS1307_REG_CONTROL	0x07		/* or ds1338 */
#define DS1307_BIT_OUT		0x80
#define DS1338_BIT_OSF		0x20
#define DS1307_BIT_SQWE		0x10
#define DS1307_BIT_RS1		0x02
#define DS1307_BIT_RS0		0x01
#define DS1337_REG_CONTROL	0x0e
#define DS1337_BIT_nEOSC		0x80
#define DS1337_BIT_RS2		0x10
#define DS1337_BIT_RS1		0x08
#define DS1337_BIT_INTCN		0x04
#define DS1337_BIT_A2IE		0x02
#define DS1337_BIT_A1IE		0x01
#define DS1337_REG_STATUS	0x0f
#define DS1337_BIT_OSF		0x80
#define DS3231_BIT_EN32KHZ	0x08
#define DS1337_BIT_A2I		0x02
#define DS1337_BIT_A1I		0x01

#define DS13XX_TRICKLE_CHARGER_MAGIC	0xa0
#define HAS_ALARM	1		/* bit 1 == irq claimed */

struct ds1307 {
	struct nvmem_config	nvmem_cfg;
	unsigned long		flags;
	struct device		*dev;
	struct regmap		*regmap;
	const char		*name;
	struct rtc_device	*rtc;
};

struct chip_desc {
	unsigned		alarm:1;
	u16			nvram_offset;
	u16			nvram_size;
	u8			offset; /* register's offset */
	u8			century_reg;
	u8			century_enable_bit;
	u8			century_bit;
	u8			bbsqi_bit;
	irq_handler_t		irq_handler;
	const struct rtc_class_ops *rtc_ops;
	u16			trickle_charger_reg;
	u8			(*do_trickle_setup)(struct ds1307 *, u32,
						    bool);
};

static int ds1307_get_time(struct device *dev, struct rtc_time *t);
static int ds1307_set_time(struct device *dev, struct rtc_time *t);

static const struct chip_desc chips = {
	.nvram_offset	= 8,
	.nvram_size	= 56,
};

static const struct i2c_device_id ds1307_id[] = {
	{ "ds1307", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds1307_id);

#ifdef CONFIG_OF
static const struct of_device_id ds1307_of_match[] = {
	{ .compatible = "dallas,ds1307", },
	{ }
};
MODULE_DEVICE_TABLE(of, ds1307_of_match);
#endif

/*
 * The ds1337 and ds1339 both have two alarms, but we only use the first
 * one (with a "seconds" field).  For ds1337 we expect nINTA is our alarm
 * signal; ds1339 chips have only one alarm signal.
 */
static irqreturn_t ds1307_irq(int irq, void *dev_id)
{
	struct ds1307		*ds1307 = dev_id;
	struct mutex		*lock = &ds1307->rtc->ops_lock;
	int			stat, ret;

	mutex_lock(lock);
	ret = regmap_read(ds1307->regmap, DS1337_REG_STATUS, &stat);
	if (ret)
		goto out;

	if (stat & DS1337_BIT_A1I) {
		stat &= ~DS1337_BIT_A1I;
		regmap_write(ds1307->regmap, DS1337_REG_STATUS, stat);

		ret = regmap_update_bits(ds1307->regmap, DS1337_REG_CONTROL,
					 DS1337_BIT_A1IE, 0);
		if (ret)
			goto out;

		rtc_update_irq(ds1307->rtc, 1, RTC_AF | RTC_IRQF);
	}

out:
	mutex_unlock(lock);

	return IRQ_HANDLED;
}

static int ds1307_get_time(struct device *dev, struct rtc_time *t)
{
	struct ds1307	*ds1307 = dev_get_drvdata(dev);
	int		tmp, ret;
	const struct chip_desc *chip = &chips;
	u8 regs[7];

	/* read the RTC date and time registers all at once */
	ret = regmap_bulk_read(ds1307->regmap, chip->offset, regs,
			       sizeof(regs));
	if (ret) {
		dev_err(dev, "%s error %d\n", "read", ret);
		return ret;
	}

	t->tm_sec = bcd2bin(regs[DS1307_REG_SECS] & 0x7f);
	t->tm_min = bcd2bin(regs[DS1307_REG_MIN] & 0x7f);
	tmp = regs[DS1307_REG_HOUR] & 0x3f;
	t->tm_hour = bcd2bin(tmp);
	t->tm_wday = bcd2bin(regs[DS1307_REG_WDAY] & 0x07) - 1;
	t->tm_mday = bcd2bin(regs[DS1307_REG_MDAY] & 0x3f);
	tmp = regs[DS1307_REG_MONTH] & 0x1f;
	t->tm_mon = bcd2bin(tmp) - 1;
	t->tm_year = bcd2bin(regs[DS1307_REG_YEAR]) + 100;

	if (regs[chip->century_reg] & chip->century_bit &&
	    IS_ENABLED(CONFIG_RTC_DRV_DS1307_CENTURY))
		t->tm_year += 100;

	/* initial clock setting can be undefined */
	return rtc_valid_tm(t);
}

static int ds1307_set_time(struct device *dev, struct rtc_time *t)
{
	struct ds1307	*ds1307 = dev_get_drvdata(dev);
	const struct chip_desc *chip = &chips;
	int		result;
	int		tmp;
	u8		regs[7];

	if (t->tm_year < 100)
		return -EINVAL;

#ifdef CONFIG_RTC_DRV_DS1307_CENTURY
	if (t->tm_year > (chip->century_bit ? 299 : 199))
		return -EINVAL;
#else
	if (t->tm_year > 199)
		return -EINVAL;
#endif

	regs[DS1307_REG_SECS] = bin2bcd(t->tm_sec);
	regs[DS1307_REG_MIN] = bin2bcd(t->tm_min);
	regs[DS1307_REG_HOUR] = bin2bcd(t->tm_hour);
	regs[DS1307_REG_WDAY] = bin2bcd(t->tm_wday + 1);
	regs[DS1307_REG_MDAY] = bin2bcd(t->tm_mday);
	regs[DS1307_REG_MONTH] = bin2bcd(t->tm_mon + 1);

	/* assume 20YY not 19YY */
	tmp = t->tm_year - 100;
	regs[DS1307_REG_YEAR] = bin2bcd(tmp);

	if (chip->century_enable_bit)
		regs[chip->century_reg] |= chip->century_enable_bit;
	if (t->tm_year > 199 && chip->century_bit)
		regs[chip->century_reg] |= chip->century_bit;

	result = regmap_bulk_write(ds1307->regmap, chip->offset, regs,
				   sizeof(regs));
	if (result) {
		dev_err(dev, "%s error %d\n", "write", result);
		return result;
	}
	return 0;
}

static int ds1337_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct ds1307		*ds1307 = dev_get_drvdata(dev);
	int			ret;
	u8			regs[9];

	if (!test_bit(HAS_ALARM, &ds1307->flags))
		return -EINVAL;

	/* read all ALARM1, ALARM2, and status registers at once */
	ret = regmap_bulk_read(ds1307->regmap, DS1339_REG_ALARM1_SECS,
			       regs, sizeof(regs));
	if (ret) {
		dev_err(dev, "%s error %d\n", "alarm read", ret);
		return ret;
	}

	t->time.tm_sec = bcd2bin(regs[0] & 0x7f);
	t->time.tm_min = bcd2bin(regs[1] & 0x7f);
	t->time.tm_hour = bcd2bin(regs[2] & 0x3f);
	t->time.tm_mday = bcd2bin(regs[3] & 0x3f);

	/* ... and status */
	t->enabled = !!(regs[7] & DS1337_BIT_A1IE);
	t->pending = !!(regs[8] & DS1337_BIT_A1I);

	return 0;
}

static int ds1337_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct ds1307		*ds1307 = dev_get_drvdata(dev);
	unsigned char		regs[9];
	u8			control, status;
	int			ret;

	if (!test_bit(HAS_ALARM, &ds1307->flags))
		return -EINVAL;

	/* read current status of both alarms and the chip */
	ret = regmap_bulk_read(ds1307->regmap, DS1339_REG_ALARM1_SECS, regs,
			       sizeof(regs));
	if (ret) {
		dev_err(dev, "%s error %d\n", "alarm write", ret);
		return ret;
	}
	control = regs[7];
	status = regs[8];

	/* set ALARM1, using 24 hour and day-of-month modes */
	regs[0] = bin2bcd(t->time.tm_sec);
	regs[1] = bin2bcd(t->time.tm_min);
	regs[2] = bin2bcd(t->time.tm_hour);
	regs[3] = bin2bcd(t->time.tm_mday);

	/* set ALARM2 to non-garbage */
	regs[4] = 0;
	regs[5] = 0;
	regs[6] = 0;

	/* disable alarms */
	regs[7] = control & ~(DS1337_BIT_A1IE | DS1337_BIT_A2IE);
	regs[8] = status & ~(DS1337_BIT_A1I | DS1337_BIT_A2I);

	ret = regmap_bulk_write(ds1307->regmap, DS1339_REG_ALARM1_SECS, regs,
				sizeof(regs));
	if (ret) {
		dev_err(dev, "can't set alarm time\n");
		return ret;
	}

	/* optionally enable ALARM1 */
	if (t->enabled) {
		dev_dbg(dev, "alarm IRQ armed\n");
		regs[7] |= DS1337_BIT_A1IE;	/* only ALARM1 is used */
		regmap_write(ds1307->regmap, DS1337_REG_CONTROL, regs[7]);
	}

	return 0;
}

static int ds1307_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct ds1307		*ds1307 = dev_get_drvdata(dev);

	if (!test_bit(HAS_ALARM, &ds1307->flags))
		return -ENOTTY;

	return regmap_update_bits(ds1307->regmap, DS1337_REG_CONTROL,
				  DS1337_BIT_A1IE,
				  enabled ? DS1337_BIT_A1IE : 0);
}

static const struct rtc_class_ops ds13xx_rtc_ops = {
	.read_time	= ds1307_get_time,
	.set_time	= ds1307_set_time,
	.read_alarm	= ds1337_read_alarm,
	.set_alarm	= ds1337_set_alarm,
	.alarm_irq_enable = ds1307_alarm_irq_enable,
};

static int ds1307_nvram_read(void *priv, unsigned int offset, void *val,
			     size_t bytes)
{
	struct ds1307 *ds1307 = priv;
	const struct chip_desc *chip = &chips;

	return regmap_bulk_read(ds1307->regmap, chip->nvram_offset + offset,
				val, bytes);
}

static int ds1307_nvram_write(void *priv, unsigned int offset, void *val,
			      size_t bytes)
{
	struct ds1307 *ds1307 = priv;
	const struct chip_desc *chip = &chips;

	return regmap_bulk_write(ds1307->regmap, chip->nvram_offset + offset,
				 val, bytes);
}

static u8 ds1307_trickle_init(struct ds1307 *ds1307,
			      const struct chip_desc *chip)
{
	u32 ohms;
	bool diode = true;

	if (!chip->do_trickle_setup)
		return 0;

	if (device_property_read_u32(ds1307->dev, "trickle-resistor-ohms",
				     &ohms))
		return 0;

	if (device_property_read_bool(ds1307->dev, "trickle-diode-disable"))
		diode = false;

	return chip->do_trickle_setup(ds1307, ohms, diode);
}

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ds1307_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ds1307_platform_data *pdata = dev_get_platdata(&client->dev);
	const struct chip_desc	*chip;
	struct rtc_time	tm;
	struct ds1307 *ds1307;
	int err = -ENODEV;
	int tmp, wday;
	bool want_irq;
	bool ds1307_can_wakeup_device = false;
	unsigned char regs[8];
	unsigned long timestamp;
	u8 trickle_charger_setup = 0;

	ds1307 = devm_kzalloc(&client->dev, sizeof(struct ds1307), GFP_KERNEL);
	if (!ds1307)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, ds1307);
	ds1307->dev = &client->dev;
	ds1307->name = client->name;

	ds1307->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(ds1307->regmap)) {
		dev_err(ds1307->dev, "regmap allocation failed\n");
		return PTR_ERR(ds1307->regmap);
	}

	i2c_set_clientdata(client, ds1307);

	chip = &chips;
	want_irq = client->irq > 0 && chip->alarm;

	if (!pdata)
		trickle_charger_setup = ds1307_trickle_init(ds1307, chip);
	else if (pdata->trickle_charger_setup)
		trickle_charger_setup = pdata->trickle_charger_setup;

	if (trickle_charger_setup && chip->trickle_charger_reg) {
		trickle_charger_setup |= DS13XX_TRICKLE_CHARGER_MAGIC;
		dev_dbg(ds1307->dev,
			"writing trickle charger info 0x%x to 0x%x\n",
			trickle_charger_setup, chip->trickle_charger_reg);
		regmap_write(ds1307->regmap, chip->trickle_charger_reg,
			     trickle_charger_setup);
	}

#ifdef CONFIG_OF
	if (chip->alarm && of_property_read_bool(client->dev.of_node,
						 "wakeup-source"))
		ds1307_can_wakeup_device = true;
#endif

read_rtc:
	/* read RTC registers */
	err = regmap_bulk_read(ds1307->regmap, chip->offset, regs,
			       sizeof(regs));
	if (err) {
		dev_dbg(ds1307->dev, "read error %d\n", err);
		goto exit;
	}

	/*
	 * minimal sanity checking; some chips (like DS1340) don't
	 * specify the extra bits as must-be-zero, but there are
	 * still a few values that are clearly out-of-range.
	 */
	tmp = regs[DS1307_REG_SECS];
		/* clock halted?  turn it on, so clock can tick. */
		if (tmp & DS1307_BIT_CH) {
			regmap_write(ds1307->regmap, DS1307_REG_SECS, 0);
			dev_warn(ds1307->dev, "SET TIME!\n");
			goto read_rtc;
		}

	tmp = regs[DS1307_REG_HOUR];
	if (tmp & DS1307_BIT_12HR)
	{
		tmp = bcd2bin(tmp & 0x1f);
		if (tmp == 12)
			tmp = 0;
		if (regs[DS1307_REG_HOUR] & DS1307_BIT_PM)
			tmp += 12;
		regmap_write(ds1307->regmap, chip->offset + DS1307_REG_HOUR,
			     bin2bcd(tmp));
	}
	/*
	 * Some IPs have weekday reset value = 0x1 which might not correct
	 * hence compute the wday using the current date/month/year values
	 */
	ds1307_get_time(ds1307->dev, &tm);
	wday = tm.tm_wday;
	timestamp = rtc_tm_to_time64(&tm);
	rtc_time64_to_tm(timestamp, &tm);

	/*
	 * Check if reset wday is different from the computed wday
	 * If different then set the wday which we computed using
	 * timestamp
	 */
	if (want_irq || ds1307_can_wakeup_device) {
		device_set_wakeup_capable(ds1307->dev, true);
		set_bit(HAS_ALARM, &ds1307->flags);
	}

	ds1307->rtc = devm_rtc_allocate_device(ds1307->dev);
	if (IS_ERR(ds1307->rtc))
		return PTR_ERR(ds1307->rtc);

	if (ds1307_can_wakeup_device && !want_irq) {
		dev_info(ds1307->dev,
			 "'wakeup-source' is set, request for an IRQ is disabled!\n");
		/* We cannot support UIE mode if we do not have an IRQ line */
		ds1307->rtc->uie_unsupported = 1;
	}

	if (want_irq) {
		err = devm_request_threaded_irq(ds1307->dev, client->irq, NULL,
						chip->irq_handler ?: ds1307_irq,
						IRQF_SHARED | IRQF_ONESHOT,
						ds1307->name, ds1307);
		if (err) {
			client->irq = 0;
			device_set_wakeup_capable(ds1307->dev, false);
			clear_bit(HAS_ALARM, &ds1307->flags);
			dev_err(ds1307->dev, "unable to request IRQ!\n");
		} else {
			dev_dbg(ds1307->dev, "got IRQ %d\n", client->irq);
		}
	}

	if (chip->nvram_size) {
		ds1307->nvmem_cfg.name = "ds1307_nvram";
		ds1307->nvmem_cfg.word_size = 1;
		ds1307->nvmem_cfg.stride = 1;
		ds1307->nvmem_cfg.size = chip->nvram_size;
		ds1307->nvmem_cfg.reg_read = ds1307_nvram_read;
		ds1307->nvmem_cfg.reg_write = ds1307_nvram_write;
		ds1307->nvmem_cfg.priv = ds1307;

		ds1307->rtc->nvmem_config = &ds1307->nvmem_cfg;
		ds1307->rtc->nvram_old_abi = true;
	}

	ds1307->rtc->ops = chip->rtc_ops ?: &ds13xx_rtc_ops;
	err = rtc_register_device(ds1307->rtc);
	if (err)
		return err;

	return 0;

exit:
	return err;
}

static struct i2c_driver ds1307_driver = {
	.driver = {
		.name	= "rtc-ds1307",
		.of_match_table = of_match_ptr(ds1307_of_match),
		.acpi_match_table = ACPI_PTR(ds1307_acpi_ids),
	},
	.probe		= ds1307_probe,
	.id_table	= ds1307_id,
};

module_i2c_driver(ds1307_driver);

MODULE_DESCRIPTION("RTC driver for DS1307 and similar chips");
MODULE_LICENSE("GPL");
