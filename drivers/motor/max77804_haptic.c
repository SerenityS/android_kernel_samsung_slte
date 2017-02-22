/*
 * haptic motor driver for max77804 - max77673_haptic.c
 *
 * Copyright (C) 2011 ByungChang Cha <bc.cha@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/max77804.h>
#include <linux/mfd/max77804-private.h>
#include <plat/devs.h>

#define TEST_MODE_TIME 10000

#define PWM_MIN 0
#define PWM_DEFAULT 50
#define PWM_THRESH 75
#define PWM_MAX 100

/* duty in percent */
static unsigned pwm_val = PWM_DEFAULT;
/* duty value, 37050=100%, 27787=50%, 18525=0% */
static int pwm_duty = 27787;

struct max77804_haptic_data {
	struct max77804_dev *max77804;
	struct i2c_client *i2c;
	struct i2c_client *pmic_i2c;
	struct max77804_haptic_platform_data *pdata;

	struct pwm_device *pwm;
	struct regulator *regulator;
	struct timed_output_dev tout_dev;
	struct hrtimer timer;
	unsigned int timeout;

	struct workqueue_struct *workqueue;
	struct work_struct work;
	spinlock_t lock;
	bool running;
};

struct max77804_haptic_data *g_hap_data;
static int prev_duty;

static void max77804_haptic_i2c(struct max77804_haptic_data *hap_data, bool en)
{
	int ret;
	u8 value = hap_data->pdata->reg2;
	u8 lscnfg_val = 0x00;

	pr_debug("[VIB] %s %d\n", __func__, en);

	if (en) {
		value |= MOTOR_EN;
		lscnfg_val = 0x80;
	}

	ret = max77804_update_reg(hap_data->pmic_i2c, MAX77804_PMIC_REG_LSCNFG,
				lscnfg_val, 0x80);
	if (ret)
		pr_err("[VIB] i2c update error %d\n", ret);

	ret = max77804_write_reg(hap_data->i2c,
				 MAX77804_HAPTIC_REG_CONFIG2, value);
	if (ret)
		pr_err("[VIB] i2c write error %d\n", ret);
}

static int haptic_get_time(struct timed_output_dev *tout_dev)
{
	struct max77804_haptic_data *hap_data
		= container_of(tout_dev, struct max77804_haptic_data, tout_dev);

	struct hrtimer *timer = &hap_data->timer;
	if (hrtimer_active(timer)) {
		ktime_t remain = hrtimer_get_remaining(timer);
		struct timeval t = ktime_to_timeval(remain);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	return 0;
}

static void haptic_enable(struct timed_output_dev *tout_dev, int value)
{
	struct max77804_haptic_data *hap_data
		= container_of(tout_dev, struct max77804_haptic_data, tout_dev);

	struct hrtimer *timer = &hap_data->timer;
	unsigned long flags;


	cancel_work_sync(&hap_data->work);
	hrtimer_cancel(timer);
	hap_data->timeout = value;
	queue_work(hap_data->workqueue, &hap_data->work);
	spin_lock_irqsave(&hap_data->lock, flags);
	if (value > 0 && value != TEST_MODE_TIME) {
		pr_debug("%s value %d\n", __func__, value);
		value = min(value, (int)hap_data->pdata->max_timeout);
		hrtimer_start(timer, ns_to_ktime((u64)value * NSEC_PER_MSEC),
			HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&hap_data->lock, flags);
}

static enum hrtimer_restart haptic_timer_func(struct hrtimer *timer)
{
	struct max77804_haptic_data *hap_data
		= container_of(timer, struct max77804_haptic_data, timer);

	hap_data->timeout = 0;
	queue_work(hap_data->workqueue, &hap_data->work);
	return HRTIMER_NORESTART;
}

static int vibetonz_clk_on(struct device *dev, bool en)
{
	struct clk *vibetonz_clk = NULL;

#if defined(CONFIG_OF)
	struct device_node *np;

	np = of_find_node_by_name(NULL,"pwm");
	if (np == NULL) {
		printk("%s : pwm error to get dt node\n", __func__);
		return -EINVAL;
	}

	vibetonz_clk = of_clk_get_by_name(np, "gate_timers");
	if (!vibetonz_clk) {
		pr_info("%s fail to get the vibetonz_clk\n", __func__);
		return -EINVAL;
	}
#else
	vibetonz_clk = clk_get(dev, "timers");
#endif
	pr_debug("[VIB] DEV NAME %s %lu\n",
		 dev_name(dev), clk_get_rate(vibetonz_clk));
	pr_info("[VIB] DEV NAME %s %lu\n",
		 dev_name(dev), clk_get_rate(vibetonz_clk));

	if (IS_ERR(vibetonz_clk)) {
		pr_err("[VIB] failed to get clock for the motor\n");
		goto err_clk_get;
	}

	if (en)
		clk_enable(vibetonz_clk);
	else
		clk_disable(vibetonz_clk);

	clk_put(vibetonz_clk);
	return 0;

err_clk_get:
	clk_put(vibetonz_clk);
	return -EINVAL;
}

static void haptic_work(struct work_struct *work)
{
	struct max77804_haptic_data *hap_data
		= container_of(work, struct max77804_haptic_data, work);
	int ret;

	pr_debug("[VIB] %s\n", __func__);
	if (hap_data->timeout > 0) {
		if (hap_data->running)
			return;

		max77804_haptic_i2c(hap_data, true);

		pwm_config(hap_data->pwm, pwm_duty,
			   hap_data->pdata->period);
		pr_debug("[VIB] %s: pwm_config duty=%d",
			 __func__, pwm_duty);

		pwm_enable(hap_data->pwm);
		if (hap_data->pdata->motor_en)
			hap_data->pdata->motor_en(true);
		else
			ret = regulator_enable(hap_data->regulator);
		hap_data->running = true;
	} else {
		if (!hap_data->running)
			return;
		if (hap_data->pdata->motor_en)
			hap_data->pdata->motor_en(false);
		else
			regulator_disable(hap_data->regulator);
		pwm_disable(hap_data->pwm);

		max77804_haptic_i2c(hap_data, false);

		hap_data->running = false;
	}
	return;
}

#ifdef CONFIG_VIBETONZ
void vibtonz_en(bool en)
{
	int ret;

	if (g_hap_data == NULL) {
		printk(KERN_ERR "[VIB] the motor is not ready!!!");
		return ;
	}

	if (en) {
		if (g_hap_data->running)
			return;

		max77804_haptic_i2c(g_hap_data, true);

		pwm_config(g_hap_data->pwm, prev_duty, g_hap_data->pdata->period);
		pwm_enable(g_hap_data->pwm);
		if (g_hap_data->pdata->motor_en)
			g_hap_data->pdata->motor_en(true);
		else
			ret = regulator_enable(g_hap_data->regulator);
		g_hap_data->running = true;
	} else {
		if (!g_hap_data->running)
			return;
		if (g_hap_data->pdata->motor_en)
			g_hap_data->pdata->motor_en(false);
		else
			regulator_disable(g_hap_data->regulator);
		pwm_disable(g_hap_data->pwm);

		max77804_haptic_i2c(g_hap_data, false);

		g_hap_data->running = false;
	}
}
EXPORT_SYMBOL(vibtonz_en);

void vibtonz_pwm(int nForce)
{
	int pwm_period = 0;

	if (g_hap_data == NULL) {
		pr_err("[VIB] %s: the motor is not ready!!!", __func__);
		return ;
	}

	/* add to avoid the glitch issue */
	if (prev_duty != pwm_duty) {
		prev_duty = pwm_duty;

		pr_debug("[VIB] %s: setting pwm_duty=%d", __func__, pwm_duty);
		pwm_config(g_hap_data->pwm, pwm_duty, pwm_period);
	}
}
EXPORT_SYMBOL(vibtonz_pwm);
#endif

#if defined(CONFIG_OF)
static int of_max77804_haptic_dt(struct max77804_haptic_platform_data *pdata)
{
	struct device_node *np_haptic;
	u32 temp;
	const char *temp_str;

	printk("%s : start dt parsing\n", __func__);

	np_haptic = of_find_node_by_path("/haptic");
	if (np_haptic == NULL) {
		printk("%s : error to get dt node\n", __func__);
		return -EINVAL;
	}

	of_property_read_u32(np_haptic, "haptic,max_timeout", &temp);
	pdata->max_timeout = (u16)temp;

	of_property_read_u32(np_haptic, "haptic,duty", &temp);
	pdata->duty = (u16)temp;

	of_property_read_u32(np_haptic, "haptic,period", &temp);
	pdata->period = (u16)temp;

	of_property_read_u32(np_haptic, "haptic,reg2", &temp);
	pdata->reg2 = (u16)temp;

	of_property_read_u32(np_haptic, "haptic,pwm_id", &temp);
	pdata->pwm_id = (u16)temp;

	of_property_read_string(np_haptic, "haptic,regulator_name", &temp_str);
	pdata->regulator_name = (char *)temp_str;

/* debugging */
	printk("%s : max_timeout = %d\n", __func__, pdata->max_timeout);
	printk("%s : duty = %d\n", __func__, pdata->duty);
	printk("%s : period = %d\n", __func__, pdata->period);
	printk("%s : reg2 = %d\n", __func__, pdata->reg2);
	printk("%s : pwm_id = %d\n", __func__, pdata->pwm_id);
	printk("%s : regulator_name = %s\n", __func__, pdata->regulator_name);

	pdata->init_hw = NULL;
	pdata->motor_en = NULL;

	return 0;
}
#endif /* CONFIG_OF */

static ssize_t pwm_value_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
	int count;

	pwm_val = ((pwm_duty - 18525) * PWM_MAX) / 18525;

	count = sprintf(buf, "%u\n", pwm_val);
	pr_debug("[VIB] pwm_val: %u\n", pwm_val);

	return count;
}

ssize_t pwm_value_store(struct device *dev,
                       struct device_attribute *attr,
                       const char *buf, size_t size)
{
	unsigned new_pwm_val;

	if (sscanf(buf, "%u", &new_pwm_val)) {
		pr_err("[VIB] %s: error on storing pwm_val\n", __func__);
	}

	if (new_pwm_val < PWM_MIN || new_pwm_val > PWM_MAX) {
		pr_info("[VIB] %s: new pwm_val %u is out of [%d, %d] range\n",
			__func__, pwm_val, PWM_MIN, PWM_MAX);
		return -EINVAL;
	}
	pwm_val = new_pwm_val;

	pr_info("[VIB] %s: pwm_val=%u\n", __func__, pwm_val);

	pwm_duty = (pwm_val * 18525) / PWM_MAX + 18525;

	/* make sure new pwm duty is in range */
	if (pwm_duty > 37050) {
		pwm_duty = 37050;
	} else if (pwm_duty < 18525) {
		pwm_duty = 18525;
	}

	pr_info("[VIB] %s: pwm_duty=%d\n", __func__, pwm_duty);

	return size;
}

static DEVICE_ATTR(pwm_value,
		   S_IRUGO | S_IWUSR,
		   pwm_value_show,
		   pwm_value_store);

static ssize_t pwm_default_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", PWM_DEFAULT);
}

static DEVICE_ATTR(pwm_default,
		   S_IRUGO,
		   pwm_default_show,
		   NULL);

static ssize_t pwm_max_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", PWM_MAX);
}

static DEVICE_ATTR(pwm_max,
		   S_IRUGO,
		   pwm_max_show,
		   NULL);

static ssize_t pwm_min_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", PWM_MIN);
}

static DEVICE_ATTR(pwm_min,
		   S_IRUGO,
		   pwm_min_show,
		   NULL);

static ssize_t pwm_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", PWM_THRESH);
}

static DEVICE_ATTR(pwm_threshold,
		   S_IRUGO,
		   pwm_threshold_show,
		   NULL);

static int max77804_haptic_probe(struct platform_device *pdev)
{
	int ret, error = 0;
	struct max77804_dev *max77804 = dev_get_drvdata(pdev->dev.parent);
#if !defined(CONFIG_OF)
	struct max77804_platform_data *max77804_pdata
		= dev_get_platdata(max77804->dev);
#endif
	struct max77804_haptic_data *hap_data;

	pr_info("[VIB] ++ %s\n", __func__);

	hap_data = kzalloc(sizeof(struct max77804_haptic_data), GFP_KERNEL);
	if (!hap_data)
		return -ENOMEM;

#if defined(CONFIG_OF)
	hap_data->pdata = kzalloc(sizeof(struct max77804_haptic_data), GFP_KERNEL);
	if (!hap_data->pdata) {
		kfree(hap_data);
		return -ENOMEM;
	}

	ret = of_max77804_haptic_dt(hap_data->pdata);
	if (ret < 0) {
		pr_err("max77804-haptic : %s not found haptic dt! ret[%d]\n",
				 __func__, ret);
		kfree(hap_data->pdata);
		kfree(hap_data);
		return -1;
	}
#else
	pdata = max77804_pdata->haptic_data;
	if (pdata == NULL) {
		pr_err("%s: no pdata\n", __func__);
		kfree(hap_data);
		return -ENODEV;
	}
#endif /* CONFIG_OF */

	platform_set_drvdata(pdev, hap_data);
	g_hap_data = hap_data;
	hap_data->max77804 = max77804;
	hap_data->i2c = max77804->haptic;
	hap_data->pmic_i2c = max77804->i2c;

	hap_data->workqueue = create_singlethread_workqueue("hap_work");
	INIT_WORK(&(hap_data->work), haptic_work);
	spin_lock_init(&(hap_data->lock));

	hap_data->pwm = pwm_request(hap_data->pdata->pwm_id, "vibrator");
	if (IS_ERR(hap_data->pwm)) {
		pr_err("[VIB] Failed to request pwm\n");
		error = -EFAULT;
		goto err_pwm_request;
	}

	pwm_config(hap_data->pwm, hap_data->pdata->period / 2, hap_data->pdata->period);
	prev_duty = hap_data->pdata->period / 2;

	vibetonz_clk_on(&pdev->dev, true);
	if (hap_data->pdata->init_hw)
		hap_data->pdata->init_hw();
	else
		hap_data->regulator
			= regulator_get(NULL, hap_data->pdata->regulator_name);

	if (IS_ERR(hap_data->regulator)) {
		pr_err("[VIB] Failed to get vmoter regulator.\n");
		error = -EFAULT;
		goto err_regulator_get;
	}
	/* hrtimer init */
	hrtimer_init(&hap_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hap_data->timer.function = haptic_timer_func;

	/* timed_output_dev init*/
	hap_data->tout_dev.name = "vibrator";
	hap_data->tout_dev.get_time = haptic_get_time;
	hap_data->tout_dev.enable = haptic_enable;

#ifdef CONFIG_ANDROID_TIMED_OUTPUT
	error = timed_output_dev_register(&hap_data->tout_dev);
	if (error < 0) {
		pr_err("[VIB] Failed to register timed_output : %d\n", error);
		error = -EFAULT;
		goto err_timed_output_register;
	}

	/* User controllable pwm level */
	error = device_create_file(hap_data->tout_dev.dev, &dev_attr_pwm_value);
	if (error > 0) {
		pr_err("[VIB] create sysfs fail: pwm_value\n");
	}
	error = device_create_file(hap_data->tout_dev.dev, &dev_attr_pwm_max);
	if (error > 0) {
		pr_err("[VIB] create sysfs fail: pwm_max\n");
	}
	error = device_create_file(hap_data->tout_dev.dev, &dev_attr_pwm_min);
	if (error > 0) {
		pr_err("[VIB] create sysfs fail: pwm_min\n");
	}
	error = device_create_file(hap_data->tout_dev.dev, &dev_attr_pwm_default);
	if (error > 0) {
		pr_err("[VIB] create sysfs fail: pwm_default\n");
	}
	error = device_create_file(hap_data->tout_dev.dev, &dev_attr_pwm_threshold);
	if (error > 0) {
		pr_err("[VIB] create sysfs fail: pwm_threshold\n");
	}
#endif

	pr_debug("[VIB] -- %s\n", __func__);

	return error;

err_timed_output_register:
	regulator_put(hap_data->regulator);
err_regulator_get:
	pwm_free(hap_data->pwm);
err_pwm_request:
	kfree(hap_data->pdata);
	kfree(hap_data);
	g_hap_data = NULL;
	return error;
}

static int __devexit max77804_haptic_remove(struct platform_device *pdev)
{
	struct max77804_haptic_data *data = platform_get_drvdata(pdev);
#ifdef CONFIG_ANDROID_TIMED_OUTPUT
	timed_output_dev_unregister(&data->tout_dev);
#endif

	regulator_put(data->regulator);
	pwm_free(data->pwm);
	destroy_workqueue(data->workqueue);
	kfree(data->pdata);
	kfree(data);
	g_hap_data = NULL;

	return 0;
}

static int max77804_haptic_suspend(struct platform_device *pdev,
			pm_message_t state)
{
	pr_info("[VIB] %s\n", __func__);
	if (g_hap_data != NULL) {
		cancel_work_sync(&g_hap_data->work);
		hrtimer_cancel(&g_hap_data->timer);
	}
	vibetonz_clk_on(&pdev->dev, false);
	return 0;
}
static int max77804_haptic_resume(struct platform_device *pdev)
{
	pr_info("[VIB] %s\n", __func__);
	vibetonz_clk_on(&pdev->dev, true);
	return 0;
}

static struct platform_driver max77804_haptic_driver = {
	.probe		= max77804_haptic_probe,
	.remove		= max77804_haptic_remove,
	.suspend	= max77804_haptic_suspend,
	.resume		= max77804_haptic_resume,
	.driver = {
		.name	= "max77804-haptic",
		.owner	= THIS_MODULE,
	},
};

static int __init max77804_haptic_init(void)
{
	pr_debug("[VIB] %s\n", __func__);
	return platform_driver_register(&max77804_haptic_driver);
}
module_init(max77804_haptic_init);

static void __exit max77804_haptic_exit(void)
{
	platform_driver_unregister(&max77804_haptic_driver);
}
module_exit(max77804_haptic_exit);

MODULE_AUTHOR("ByungChang Cha <bc.cha@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX77804 haptic driver");
