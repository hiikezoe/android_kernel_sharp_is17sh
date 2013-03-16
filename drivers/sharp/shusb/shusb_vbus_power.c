/* drivers/sharp/shboosten/shusb_vbus_power.c
 *
 * Copyright (C) 2012 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mutex.h>
#include <sharp/shusb_vbus_power.h>

#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define D_MSM_HSUSB_VBUS_5V_EN	(25 - 1)

void set_pm8058_5v_power_en(int kind, int on)
{
	volatile static unsigned long vbus_5v_en = 0;
	static int mutex_inited = 0;
	static struct mutex lock;
	struct pm_gpio gpio_config ={
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.vin_sel        = 2,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
	unsigned long vbus_5v_en_old = vbus_5v_en;

	if (!mutex_inited) {
		mutex_init(&lock);
		mutex_inited = 1;
	}

	mutex_lock(&lock);
	if (kind == PM8058_5V_POWER_EN_GEIGER) {
		if (on)
			set_bit(PM8058_5V_POWER_EN_GEIGER, &vbus_5v_en);
		else
			clear_bit(PM8058_5V_POWER_EN_GEIGER, &vbus_5v_en);
	} else if (kind == PM8058_5V_POWER_EN_USBHOST) {
		if (on)
			set_bit(PM8058_5V_POWER_EN_USBHOST, &vbus_5v_en);
		else
			clear_bit(PM8058_5V_POWER_EN_USBHOST, &vbus_5v_en);
	}

	/* If VBUS is already on (or off), do nothing. */
	if ((vbus_5v_en && vbus_5v_en_old) ||
	    (!vbus_5v_en && !vbus_5v_en_old))
		goto unlock;

	if (vbus_5v_en) {
		static int initialize = 0;

		if (initialize) {
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(D_MSM_HSUSB_VBUS_5V_EN), 1);
		} else {
			int rc;

			rc = pm8xxx_gpio_config(
				PM8058_GPIO_PM_TO_SYS(D_MSM_HSUSB_VBUS_5V_EN),
				&gpio_config);
			if (rc) {
				pr_err("%s VBUS ERR:PMIC GPIO write failed\n", __func__);
				gpio_set_value_cansleep(
					PM8058_GPIO_PM_TO_SYS(D_MSM_HSUSB_VBUS_5V_EN), 0);
				goto unlock;
			}
			initialize = 1;
		}
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(D_MSM_HSUSB_VBUS_5V_EN), 0);
	}
unlock:
	mutex_unlock(&lock);
}
EXPORT_SYMBOL(set_pm8058_5v_power_en);
