// SPDX-License-Identifier: GPL-2.0-only
/*
 * An I2C and SPI driver for the NXP PCF2131 RTC
 * Copyright 2023 SVEP
 *
 * Author: Jorgen Persson <jorgen@jorgenp.com>
 *
 * based on the other drivers in this same directory.
 *
 * Datasheet: https://www.nxp.com/docs/en/data-sheet/PCF2131DS.pdf
 */

#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

/* Control register 1 */
#define PCF2131_REG_CTRL1		0x00
/* Control register 2 */
#define PCF2131_REG_CTRL2		0x01
/* Control register 3 */
#define PCF2131_REG_CTRL3		0x02
#define PCF2131_BIT_CTRL3_BLIE			BIT(0)
#define PCF2131_BIT_CTRL3_BIE			BIT(1)
#define PCF2131_BIT_CTRL3_BLF			BIT(2)
#define PCF2131_BIT_CTRL3_BF			BIT(3)
#define PCF2131_BIT_CTRL3_BTSE			BIT(4)
/* Control register 4 */
#define PCF2131_REG_CTRL4               0x03
#define PCF2131_BIT_CTRL4_TSF2                  BIT(6)
#define PCF2131_BIT_CTRL4_TSF1                  BIT(7)
/* Control register 5 */
#define PCF2131_REG_CTRL5               0x04
#define PCF2131_BIT_CTRL5_TSIE1                 BIT(7)
/* Time and date registers */
#define PCF2131_REG_SC			0x07
#define PCF2131_BIT_SC_OSF			BIT(7)
#define PCF2131_REG_MN			0x08
#define PCF2131_REG_HR			0x09
#define PCF2131_REG_DM			0x0A
#define PCF2131_REG_DW			0x0B
#define PCF2131_REG_MO			0x0C
#define PCF2131_REG_YR			0x0D
/* Watchdog registers */
#define PCF2131_REG_WD_CTL		0x35
#define PCF2131_BIT_WD_CTL_TF0			BIT(0)
#define PCF2131_BIT_WD_CTL_TF1			BIT(1)
#define PCF2131_BIT_WD_CTL_CD			BIT(7)
#define PCF2131_REG_WD_VAL		0x36
/* Tamper timestamp registers */
#define PCF2131_REG_TS_CTRL		0x14
#define PCF2131_BIT_TS_CTRL_TSOFF		BIT(6)
#define PCF2131_BIT_TS_CTRL_TSM			BIT(7)
#define PCF2131_REG_TS_SC		0x15
#define PCF2131_REG_TS_MN		0x16
#define PCF2131_REG_TS_HR		0x17
#define PCF2131_REG_TS_DM		0x18
#define PCF2131_REG_TS_MO		0x19
#define PCF2131_REG_TS_YR		0x20
/*
 * RAM registers
 * PCF2127 has 512 bytes general-purpose static RAM (SRAM) that is
 * battery backed and can survive a power outage.
 * PCF2131 doesn't have this feature.
 */
#define PCF2127_REG_RAM_ADDR_MSB	0x1A
#define PCF2127_REG_RAM_WRT_CMD		0x1C
#define PCF2127_REG_RAM_RD_CMD		0x1D

/* Watchdog timer value constants */
#define PCF2131_WD_VAL_STOP		0
#define PCF2131_WD_VAL_MIN		2
#define PCF2131_WD_VAL_MAX		255
#define PCF2131_WD_VAL_DEFAULT		60

struct pcf2131 {
	struct rtc_device *rtc;
	struct watchdog_device wdd;
	struct regmap *regmap;
};

/*
 * In the routines that deal directly with the pcf2131 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int pcf2131_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf2131 *pcf2131 = dev_get_drvdata(dev);
	unsigned char buf[10];
	int ret;

	/* DEBUG: hard setting time and then exit 0 */

	tm->tm_sec = 0;
	tm->tm_min = 5;
	tm->tm_hour = 13;
	tm->tm_mday = 6;
	tm->tm_wday = 0; // Assuming Sunday as the first day of the week
	tm->tm_mon = 4; // Month is zero-indexed, so May is 4
	tm->tm_year = 118; // Since 1900, so 2018 - 1900 = 118

	return 0;

	/*
	 * Avoid reading CTRL2 register as it causes WD_VAL register
	 * value to reset to 0 which means watchdog is stopped.
	 */
	ret = regmap_bulk_read(pcf2131->regmap, PCF2131_REG_CTRL3,
			       (buf + PCF2131_REG_CTRL3),
			       ARRAY_SIZE(buf) - PCF2131_REG_CTRL3);
	if (ret) {
		dev_err(dev, "%s: read error\n", __func__);
		return ret;
	}

	if (buf[PCF2131_REG_CTRL3] & PCF2131_BIT_CTRL3_BLF)
		dev_info(dev,
			"low voltage detected, check/replace RTC battery.\n");
//
//	/* Clock integrity is not guaranteed when OSF flag is set. */
//	if (buf[PCF2131_REG_SC] & PCF2131_BIT_SC_OSF) {
//		/*
//		 * no need clear the flag here,
//		 * it will be cleared once the new date is saved
//		 */
//		printk(KERN_INFO "Debug message: buf[PCF2131_REG_SC]: %d, PCF2131_BIT_SC_OSF: %d \n", buf[PCF2131_REG_SC], PCF2131_BIT_SC_OSF);
//		printk(KERN_INFO "Debug message2: PCF2131_REG_SC: %d \n", PCF2131_REG_SC);
//		dev_warn(dev,
//			 "oscillator stop detected, date/time is not reliable\n");
//		return -EINVAL;
//	}

	dev_dbg(dev,
		"%s: raw data is cr3=%02x, sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x, mon=%02x, year=%02x\n",
		__func__, buf[PCF2131_REG_CTRL3], buf[PCF2131_REG_SC],
		buf[PCF2131_REG_MN], buf[PCF2131_REG_HR],
		buf[PCF2131_REG_DM], buf[PCF2131_REG_DW],
		buf[PCF2131_REG_MO], buf[PCF2131_REG_YR]);

	tm->tm_sec = bcd2bin(buf[PCF2131_REG_SC] & 0x7F);
	tm->tm_min = bcd2bin(buf[PCF2131_REG_MN] & 0x7F);
	tm->tm_hour = bcd2bin(buf[PCF2131_REG_HR] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(buf[PCF2131_REG_DM] & 0x3F);
	tm->tm_wday = buf[PCF2131_REG_DW] & 0x07;
	tm->tm_mon = bcd2bin(buf[PCF2131_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(buf[PCF2131_REG_YR]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */

	dev_dbg(dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	return 0;
}

static int pcf2131_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct pcf2131 *pcf2131 = dev_get_drvdata(dev);
	unsigned char buf[7];
	int i = 0, err;

	dev_dbg(dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* hours, minutes and seconds */
	buf[i++] = bin2bcd(tm->tm_sec);	/* this will also clear OSF flag */
	buf[i++] = bin2bcd(tm->tm_min);
	buf[i++] = bin2bcd(tm->tm_hour);
	buf[i++] = bin2bcd(tm->tm_mday);
	buf[i++] = tm->tm_wday & 0x07;

	/* month, 1 - 12 */
	buf[i++] = bin2bcd(tm->tm_mon + 1);

	/* year */
	buf[i++] = bin2bcd(tm->tm_year % 100);

	/* write register's data */
	err = regmap_bulk_write(pcf2131->regmap, PCF2131_REG_SC, buf, i);
	if (err) {
		dev_err(dev,
			"%s: err=%d", __func__, err);
		return err;
	}

	return 0;
}

#ifdef CONFIG_RTC_INTF_DEV
static int pcf2131_rtc_ioctl(struct device *dev,
				unsigned int cmd, unsigned long arg)
{
	struct pcf2131 *pcf2131 = dev_get_drvdata(dev);
	int touser;
	int ret;

	switch (cmd) {
	case RTC_VL_READ:
		ret = regmap_read(pcf2131->regmap, PCF2131_REG_CTRL3, &touser);
		if (ret)
			return ret;

		touser = touser & PCF2131_BIT_CTRL3_BLF ? 1 : 0;

		if (copy_to_user((void __user *)arg, &touser, sizeof(int)))
			return -EFAULT;
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}
#else
#define pcf2131_rtc_ioctl NULL
#endif

static const struct rtc_class_ops pcf2131_rtc_ops = {
	.ioctl		= pcf2131_rtc_ioctl,
	.read_time	= pcf2131_rtc_read_time,
	.set_time	= pcf2131_rtc_set_time,
};

static int pcf2131_nvmem_read(void *priv, unsigned int offset,
			      void *val, size_t bytes)
{
	struct pcf2131 *pcf2131 = priv;
	int ret;
	unsigned char offsetbuf[] = { offset >> 8, offset };

	ret = regmap_bulk_write(pcf2131->regmap, PCF2127_REG_RAM_ADDR_MSB,
				offsetbuf, 2);
	if (ret)
		return ret;

	return regmap_bulk_read(pcf2131->regmap, PCF2127_REG_RAM_RD_CMD,
				val, bytes);
}

static int pcf2131_nvmem_write(void *priv, unsigned int offset,
			       void *val, size_t bytes)
{
	struct pcf2131 *pcf2131 = priv;
	int ret;
	unsigned char offsetbuf[] = { offset >> 8, offset };

	ret = regmap_bulk_write(pcf2131->regmap, PCF2127_REG_RAM_ADDR_MSB,
				offsetbuf, 2);
	if (ret)
		return ret;

	return regmap_bulk_write(pcf2131->regmap, PCF2127_REG_RAM_WRT_CMD,
				 val, bytes);
}

/* watchdog driver */

static int pcf2131_wdt_ping(struct watchdog_device *wdd)
{
	struct pcf2131 *pcf2131 = watchdog_get_drvdata(wdd);

	return regmap_write(pcf2131->regmap, PCF2131_REG_WD_VAL, wdd->timeout);
}

/*
 * Restart watchdog timer if feature is active.
 *
 * Note: Reading CTRL2 register causes watchdog to stop which is unfortunate,
 * since register also contain control/status flags for other features.
 * Always call this function after reading CTRL2 register.
 */
static int pcf2131_wdt_active_ping(struct watchdog_device *wdd)
{
	int ret = 0;

	if (watchdog_active(wdd)) {
		ret = pcf2131_wdt_ping(wdd);
		if (ret)
			dev_err(wdd->parent,
				"%s: watchdog restart failed, ret=%d\n",
				__func__, ret);
	}

	return ret;
}

static int pcf2131_wdt_start(struct watchdog_device *wdd)
{
	return pcf2131_wdt_ping(wdd);
}

static int pcf2131_wdt_stop(struct watchdog_device *wdd)
{
	struct pcf2131 *pcf2131 = watchdog_get_drvdata(wdd);

	return regmap_write(pcf2131->regmap, PCF2131_REG_WD_VAL,
			    PCF2131_WD_VAL_STOP);
}

static int pcf2131_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int new_timeout)
{
	dev_dbg(wdd->parent, "new watchdog timeout: %is (old: %is)\n",
		new_timeout, wdd->timeout);

	wdd->timeout = new_timeout;

	return pcf2131_wdt_active_ping(wdd);
}

static const struct watchdog_info pcf2131_wdt_info = {
	.identity = "NXP PCF2131 Watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT,
};

static const struct watchdog_ops pcf2131_watchdog_ops = {
	.owner = THIS_MODULE,
	.start = pcf2131_wdt_start,
	.stop = pcf2131_wdt_stop,
	.ping = pcf2131_wdt_ping,
	.set_timeout = pcf2131_wdt_set_timeout,
};

/* sysfs interface */

static ssize_t timestamp0_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct pcf2131 *pcf2131 = dev_get_drvdata(dev->parent);
	int ret;

	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_CTRL4,
				 PCF2131_BIT_CTRL4_TSF1, 0);
	if (ret) {
		dev_err(dev, "%s: update ctrl1 ret=%d\n", __func__, ret);
		return ret;
	}

	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_CTRL4,
				 PCF2131_BIT_CTRL4_TSF2, 0);
	if (ret) {
		dev_err(dev, "%s: update ctrl2 ret=%d\n", __func__, ret);
		return ret;
	}

	ret = pcf2131_wdt_active_ping(&pcf2131->wdd);
	if (ret)
		return ret;

	return count;
};

static ssize_t timestamp0_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct pcf2131 *pcf2131 = dev_get_drvdata(dev->parent);
	struct rtc_time tm;
	int ret;
	unsigned char data[25];

	ret = regmap_bulk_read(pcf2131->regmap, PCF2131_REG_CTRL1, data,
			       sizeof(data));
	if (ret) {
		dev_err(dev, "%s: read error ret=%d\n", __func__, ret);
		return ret;
	}

	dev_dbg(dev,
		"%s: raw data is cr1=%02x, cr2=%02x, cr3=%02x, ts_sc=%02x, "
		"ts_mn=%02x, ts_hr=%02x, ts_dm=%02x, ts_mo=%02x, ts_yr=%02x\n",
		__func__, data[PCF2131_REG_CTRL1], data[PCF2131_REG_CTRL2],
		data[PCF2131_REG_CTRL3], data[PCF2131_REG_TS_SC],
		data[PCF2131_REG_TS_MN], data[PCF2131_REG_TS_HR],
		data[PCF2131_REG_TS_DM], data[PCF2131_REG_TS_MO],
		data[PCF2131_REG_TS_YR]);

	ret = pcf2131_wdt_active_ping(&pcf2131->wdd);
	if (ret)
		return ret;

	if (!(data[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF1) &&
	    !(data[PCF2131_REG_CTRL4] & PCF2131_BIT_CTRL4_TSF2))
		return 0;

	tm.tm_sec = bcd2bin(data[PCF2131_REG_TS_SC] & 0x7F);
	tm.tm_min = bcd2bin(data[PCF2131_REG_TS_MN] & 0x7F);
	tm.tm_hour = bcd2bin(data[PCF2131_REG_TS_HR] & 0x3F);
	tm.tm_mday = bcd2bin(data[PCF2131_REG_TS_DM] & 0x3F);
	/* TS_MO register (month) value range: 1-12 */
	tm.tm_mon = bcd2bin(data[PCF2131_REG_TS_MO] & 0x1F) - 1;
	tm.tm_year = bcd2bin(data[PCF2131_REG_TS_YR]);
	if (tm.tm_year < 70)
		tm.tm_year += 100; /* assume we are in 1970...2069 */

	ret = rtc_valid_tm(&tm);
	if (ret)
		return ret;

	return sprintf(buf, "%llu\n",
		       (unsigned long long)rtc_tm_to_time64(&tm));
};

static DEVICE_ATTR_RW(timestamp0);

static struct attribute *pcf2131_attrs[] = {
	&dev_attr_timestamp0.attr,
	NULL
};

static const struct attribute_group pcf2131_attr_group = {
	.attrs	= pcf2131_attrs,
};

static int pcf2131_probe(struct device *dev, struct regmap *regmap,
			const char *name, bool has_nvmem)
{
	struct pcf2131 *pcf2131;
	int ret = 0;

	dev_dbg(dev, "%s\n", __func__);

	pcf2131 = devm_kzalloc(dev, sizeof(*pcf2131), GFP_KERNEL);
	if (!pcf2131)
		return -ENOMEM;

	pcf2131->regmap = regmap;

	dev_set_drvdata(dev, pcf2131);

	pcf2131->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(pcf2131->rtc))
		return PTR_ERR(pcf2131->rtc);

	pcf2131->rtc->ops = &pcf2131_rtc_ops;

	pcf2131->wdd.parent = dev;
	pcf2131->wdd.info = &pcf2131_wdt_info;
	pcf2131->wdd.ops = &pcf2131_watchdog_ops;
	pcf2131->wdd.min_timeout = PCF2131_WD_VAL_MIN;
	pcf2131->wdd.max_timeout = PCF2131_WD_VAL_MAX;
	pcf2131->wdd.timeout = PCF2131_WD_VAL_DEFAULT;
	pcf2131->wdd.min_hw_heartbeat_ms = 500;

	watchdog_set_drvdata(&pcf2131->wdd, pcf2131);

	if (has_nvmem) {
		struct nvmem_config nvmem_cfg = {
			.priv = pcf2131,
			.reg_read = pcf2131_nvmem_read,
			.reg_write = pcf2131_nvmem_write,
			.size = 512,
		};

		ret = rtc_nvmem_register(pcf2131->rtc, &nvmem_cfg);
	}

	/*
	 * Watchdog timer enabled and reset pin /RST activated when timed out.
	 * Select 1Hz clock source for watchdog timer.
	 * Timer is not started until WD_VAL is loaded with a valid value.
	 * Note: Countdown timer disabled and not available.
	 */
	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_WD_CTL,
				 PCF2131_BIT_WD_CTL_CD |
				 PCF2131_BIT_WD_CTL_TF1 |
				 PCF2131_BIT_WD_CTL_TF0,
				 PCF2131_BIT_WD_CTL_CD |
				 PCF2131_BIT_WD_CTL_TF1);
	if (ret) {
		dev_err(dev, "%s: watchdog config (wd_ctl) failed\n", __func__);
		return ret;
	}

#ifdef CONFIG_WATCHDOG
	ret = devm_watchdog_register_device(dev, &pcf2131->wdd);
	if (ret)
		return ret;
#endif /* CONFIG_WATCHDOG */

	/*
	 * Disable battery low/switch-over timestamp and interrupts.
	 * Clear battery interrupt flags which can block new trigger events.
	 * Note: This is the default chip behaviour but added to ensure
	 * correct tamper timestamp and interrupt function.
	 */
	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_CTRL3,
				 PCF2131_BIT_CTRL3_BTSE |
				 PCF2131_BIT_CTRL3_BF |
				 PCF2131_BIT_CTRL3_BIE |
				 PCF2131_BIT_CTRL3_BLIE, 0);
	if (ret) {
		dev_err(dev, "%s: interrupt config (ctrl3) failed\n",
			__func__);
		return ret;
	}

	/*
	 * Enable timestamp function and store timestamp of first trigger
	 * event until TSF1 and TFS2 interrupt flags are cleared.
	 */
	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_TS_CTRL,
				 PCF2131_BIT_TS_CTRL_TSOFF |
				 PCF2131_BIT_TS_CTRL_TSM,
				 PCF2131_BIT_TS_CTRL_TSM);
	if (ret) {
		dev_err(dev, "%s: tamper detection config (ts_ctrl) failed\n",
			__func__);
		return ret;
	}

	/*
	 * Enable interrupt generation when TSF1 or TSF2 timestamp flags
	 * are set. Interrupt signal is an open-drain output and can be
	 * left floating if unused.
	 */
	ret = regmap_update_bits(pcf2131->regmap, PCF2131_REG_CTRL5,
				 PCF2131_BIT_CTRL5_TSIE1,
				 PCF2131_BIT_CTRL5_TSIE1);
	if (ret) {
		dev_err(dev, "%s: tamper detection config (ctrl2) failed\n",
			__func__);
		return ret;
	}

	ret = rtc_add_group(pcf2131->rtc, &pcf2131_attr_group);
	if (ret) {
		dev_err(dev, "%s: tamper sysfs registering failed\n",
			__func__);
		return ret;
	}

	return rtc_register_device(pcf2131->rtc);
}

#ifdef CONFIG_OF
static const struct of_device_id pcf2131_of_match[] = {
	{ .compatible = "nxp,pcf2131" },
	{ .compatible = "nxp,pcf2129" },
	{}
};
MODULE_DEVICE_TABLE(of, pcf2131_of_match);
#endif

#if IS_ENABLED(CONFIG_I2C)

static int pcf2131_i2c_write(void *context, const void *data, size_t count)
{
	struct device *dev = context;
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = i2c_master_send(client, data, count);
	if (ret != count)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int pcf2131_i2c_gather_write(void *context,
				const void *reg, size_t reg_size,
				const void *val, size_t val_size)
{
	struct device *dev = context;
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	void *buf;

	if (WARN_ON(reg_size != 1))
		return -EINVAL;

	buf = kmalloc(val_size + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, reg, 1);
	memcpy(buf + 1, val, val_size);

	ret = i2c_master_send(client, buf, val_size + 1);

	kfree(buf);

	if (ret != val_size + 1)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int pcf2131_i2c_read(void *context, const void *reg, size_t reg_size,
				void *val, size_t val_size)
{
	struct device *dev = context;
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	if (WARN_ON(reg_size != 1))
		return -EINVAL;

	ret = i2c_master_send(client, reg, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;

	ret = i2c_master_recv(client, val, val_size);
	if (ret != val_size)
		return ret < 0 ? ret : -EIO;

	return 0;
}

/*
 * The reason we need this custom regmap_bus instead of using regmap_init_i2c()
 * is that the STOP condition is required between set register address and
 * read register data when reading from registers.
 */
static const struct regmap_bus pcf2131_i2c_regmap = {
	.write = pcf2131_i2c_write,
	.gather_write = pcf2131_i2c_gather_write,
	.read = pcf2131_i2c_read,
};

static struct i2c_driver pcf2131_i2c_driver;

static int pcf2131_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct regmap *regmap;
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	regmap = devm_regmap_init(&client->dev, &pcf2131_i2c_regmap,
					&client->dev, &config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "%s: regmap allocation failed: %ld\n",
			__func__, PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return pcf2131_probe(&client->dev, regmap,
			     pcf2131_i2c_driver.driver.name, id->driver_data);
}

static const struct i2c_device_id pcf2131_i2c_id[] = {
	{ "pcf2131", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcf2131_i2c_id);

static struct i2c_driver pcf2131_i2c_driver = {
	.driver		= {
		.name	= "rtc-pcf2131-i2c",
		.of_match_table = of_match_ptr(pcf2131_of_match),
	},
	.probe		= pcf2131_i2c_probe,
	.id_table	= pcf2131_i2c_id,
};

static int pcf2131_i2c_register_driver(void)
{
	return i2c_add_driver(&pcf2131_i2c_driver);
}

static void pcf2131_i2c_unregister_driver(void)
{
	i2c_del_driver(&pcf2131_i2c_driver);
}

#else

static int pcf2131_i2c_register_driver(void)
{
	return 0;
}

static void pcf2131_i2c_unregister_driver(void)
{
}

#endif

#if IS_ENABLED(CONFIG_SPI_MASTER)

static struct spi_driver pcf2131_spi_driver;

static int pcf2131_spi_probe(struct spi_device *spi)
{
	static const struct regmap_config config = {
		.reg_bits = 8,
		.val_bits = 8,
		.read_flag_mask = 0xa0,
		.write_flag_mask = 0x20,
	};
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "%s: regmap allocation failed: %ld\n",
			__func__, PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return pcf2131_probe(&spi->dev, regmap, pcf2131_spi_driver.driver.name,
			     spi_get_device_id(spi)->driver_data);
}

static const struct spi_device_id pcf2131_spi_id[] = {
	{ "pcf2131", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, pcf2131_spi_id);

static struct spi_driver pcf2131_spi_driver = {
	.driver		= {
		.name	= "rtc-pcf2131-spi",
		.of_match_table = of_match_ptr(pcf2131_of_match),
	},
	.probe		= pcf2131_spi_probe,
	.id_table	= pcf2131_spi_id,
};

static int pcf2131_spi_register_driver(void)
{
	return spi_register_driver(&pcf2131_spi_driver);
}

static void pcf2131_spi_unregister_driver(void)
{
	spi_unregister_driver(&pcf2131_spi_driver);
}

#else

static int pcf2131_spi_register_driver(void)
{
	return 0;
}

static void pcf2131_spi_unregister_driver(void)
{
}

#endif

static int __init pcf2131_init(void)
{
	int ret;

	ret = pcf2131_i2c_register_driver();
	if (ret) {
		pr_err("Failed to register pcf2131 i2c driver: %d\n", ret);
		return ret;
	}

	ret = pcf2131_spi_register_driver();
	if (ret) {
		pr_err("Failed to register pcf2131 spi driver: %d\n", ret);
		pcf2131_i2c_unregister_driver();
	}

	return ret;
}
module_init(pcf2131_init)

static void __exit pcf2131_exit(void)
{
	pcf2131_spi_unregister_driver();
	pcf2131_i2c_unregister_driver();
}
module_exit(pcf2131_exit)

MODULE_AUTHOR("Renaud Cerrato <r.cerrato@til-technologies.fr>");
MODULE_DESCRIPTION("NXP PCF2131/29 RTC driver");
MODULE_LICENSE("GPL v2");
