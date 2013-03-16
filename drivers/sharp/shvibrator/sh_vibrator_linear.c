/* drivers/sharp/shvibrator/sh_vibrator.c
 *
 * Copyright (C) 2010 SHARP CORPORATION
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
/* CONFIG_SH_AUDIO_DRIVER newly created */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include "../../staging/android/timed_output.h"
#include <linux/sched.h>

/* SH_AUDIO_DRIVER ADD 2011/06/06 --> */
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <linux/delay.h>
/* SH_AUDIO_DRIVER ADD 2011/06/06 <-- */

#include <mach/pmic.h>
#include <mach/msm_rpcrouter.h>

/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
#include <linux/slab.h>
#include <linux/i2c.h>
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */

#if defined( CONFIG_SENSORS_AMI603 ) || defined( CONFIG_SH_YAS530 )
#define SHVIB_PAUSE_PEDOMETER
#endif
#if defined( SHVIB_PAUSE_PEDOMETER )
#if defined( CONFIG_SENSORS_AMI603 )
#include <sharp/shmds_driver.h>
#elif defined( CONFIG_SH_YAS530 )
#include <sharp/shpem_kerl.h>
#endif
#endif

/* [BatteryTemperatureLog] [start] */
#include <sharp/shterm_k.h>
/* [BatteryTemperatureLog] [end] */
/* [stop geiger when using speaker/vibration] -> */
#if defined( CONFIG_SENSORS_SHGEIGER )
#include <sharp/shgeiger.h>
#endif
/* [stop geiger when using speaker/vibration] <- */

/* SH_AUDIO_DRIVER ADD 2011/06/06 --> */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)

#define SHVIB_RST_PDN				21
#define SHVIB_RST_PDN_MSG			"gpio_linearVIB_reset"
#define SHVIB_PDN					22
#define SHVIB_PDN_MSG				"gpio_linearVIB"
/* SH_AUDIO_DRIVER ADD 2011/06/06 <-- */

/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
#define SHVIB_I2C_DRIVER_NAME		"sh_vib_i2c"
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */

static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
/* add work queue for hrtimer -> */
static struct work_struct work_hrtimer;
/* add work queue for hrtimer <- */
static struct hrtimer vibe_timer;

#if defined( SHVIB_PAUSE_PEDOMETER )
static int pause_pedometer = 0; /* Whether pedometer would be paused. */
static int paused_pedometer = 0; /* Whether pedometer was already paused. */
#endif

/* [BatteryTemperatureLog] [start] */
static bool sh_vibrator_shterm_flg = false;
/* [BatteryTemperatureLog] [end] */

static int sh_vibrator_reqtime = 0;
static struct mutex sh_vibrator_pmic_mutex;
/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
static int shvibrator_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);
static int shvibrator_i2c_remove(struct i2c_client *client);

int __init shvibrator_i2c_init(void);
void shvibrator_i2c_exit(void);
int shvibrator_i2c_read(struct i2c_client *client,
									int reg, u8 *data, int size);
int shvibrator_i2c_write(struct i2c_client *client,
									int reg, u8 data);
static int vibrator_init = 0;
struct shvib_i2c_data {
	struct i2c_client *client_p;
};
struct shvib_i2c_data *vib_data = NULL;
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */

static void set_pmic_vibrator(int on)
{
	mutex_lock(&sh_vibrator_pmic_mutex);
	if (on){
#if 0 /* Warning fix [start] */
/* SH_AUDIO_DRIVER ADD 2011/06/08 --> */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN - 1), 1);
		usleep(300);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN - 1), 1);
/* SH_AUDIO_DRIVER ADD 2011/06/08 <-- */

		pr_debug("[shvibrator] GPIO vibrator On.\n");
	}else{
		pr_debug("[shvibrator] GPIO vibrator Off.\n");

/* SH_AUDIO_DRIVER ADD 2011/06/08 --> */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN - 1), 0);
		usleep(300);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN - 1), 0);
/* SH_AUDIO_DRIVER ADD 2011/06/08 <-- */
#else /* Warning fix */
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN - 1), 1);
		usleep(300);
/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
		if(vib_data){
			u8 getval;
			shvibrator_i2c_write(vib_data->client_p, 0x03, 0x01);   /* STTIME 0 */
			shvibrator_i2c_write(vib_data->client_p, 0x01, 0x0b);   /* HBPW OFF */
			shvibrator_i2c_write(vib_data->client_p, 0x02, 0x0f); 	/* FRQ 200Hz */
			shvibrator_i2c_read( vib_data->client_p, 0x02, &getval ,1);

		}
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN - 1), 1);
		pr_debug("[shvibrator] GPIO vibrator On.\n");
	}else{
		pr_debug("[shvibrator] GPIO vibrator Off.\n");
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN - 1), 0);
		usleep(300);
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN - 1), 0);
#endif /* Warning fix [end] */
	}
	mutex_unlock(&sh_vibrator_pmic_mutex);
}

static void pmic_vibrator_on(struct work_struct *work)
{
#if defined( SHVIB_PAUSE_PEDOMETER )
	if (!paused_pedometer && pause_pedometer) {
#if defined( CONFIG_SENSORS_AMI603 )
		pr_debug("[shvibrator] Pause a pedometer.\n");
		AMI602Pedometer_Pause();
#elif defined( CONFIG_SH_YAS530 )
		pr_debug("[shvibrator] Pause a acclerometer.\n");
		SHMDS_Acclerometer_Control( SHMDS_ACC_PAUSE );
#endif
		paused_pedometer = 1;
	} else {
#if defined( CONFIG_SENSORS_AMI603 )
		pr_debug("[shvibrator] Don't pause a pedometer.\n");
#elif defined( CONFIG_SH_YAS530 )
		pr_debug("[shvibrator] Don't pause a acclerometer.\n");
#endif
	}
#endif

/* [BatteryTemperatureLog] [start] */
    if (sh_vibrator_shterm_flg == false) {
        pr_debug("%s() shterm_k_set_info( SHTERM_INFO_VIB, 1 ) \n", __func__);
        shterm_k_set_info( SHTERM_INFO_VIB, 1 );
        sh_vibrator_shterm_flg = true;
    }
/* [BatteryTemperatureLog] [end] */
/* [stop geiger when using speaker/vibration] -> */
#if defined( CONFIG_SENSORS_SHGEIGER )
	ShGeigerPause_Vib();
#endif
/* [stop geiger when using speaker/vibration] <- */

	set_pmic_vibrator(1);
	
	hrtimer_start(&vibe_timer,
		      ktime_set(sh_vibrator_reqtime / 1000, (sh_vibrator_reqtime % 1000) * 1000000),
		      HRTIMER_MODE_REL);
	pr_debug("[shvibrator] timer start. %d \n", sh_vibrator_reqtime);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);

/* [stop geiger when using speaker/vibration] -> */
#if defined( CONFIG_SENSORS_SHGEIGER )
	ShGeigerReStart_Vib();
#endif
/* [stop geiger when using speaker/vibration] <- */
/* [BatteryTemperatureLog] [start] */
    if (sh_vibrator_shterm_flg == true) {
        pr_debug("%s() shterm_k_set_info( SHTERM_INFO_VIB, 0 ) \n", __func__);
        shterm_k_set_info( SHTERM_INFO_VIB, 0 );
        sh_vibrator_shterm_flg = false;
    }
/* [BatteryTemperatureLog] [end] */

#if defined( SHVIB_PAUSE_PEDOMETER )
	if (paused_pedometer) {
#if defined( CONFIG_SENSORS_AMI603 )
		pr_debug("[shvibrator] Restart a pedometer.\n");
		AMI602Pedometer_ReStart();
#elif defined( CONFIG_SH_YAS530 )
		pr_debug("[shvibrator] Restart a acclerometer.\n");
		SHMDS_Acclerometer_Control( SHMDS_ACC_RESTART );
#endif
		paused_pedometer = 0;
	} else {
#if defined( CONFIG_SENSORS_AMI603 )
		pr_debug("[shvibrator] Don't restart a pedometer.\n");
#elif defined( CONFIG_SH_YAS530 )
		pr_debug("[shvibrator] Don't restart a acclerometer.\n");
#endif
	}
#endif
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	if(schedule_work(&work_vibrator_on) != 1){
		pr_debug("[shvibrator] update vibrator on workqueue\n");
		cancel_work_sync(&work_vibrator_on);
		schedule_work(&work_vibrator_on);
	}
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	if(schedule_work(&work_vibrator_off) != 1){
		pr_debug("[shvibrator] update vibrator off workqueue\n");
		cancel_work_sync(&work_vibrator_off);
		schedule_work(&work_vibrator_off);
	}
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	pr_debug("[shvibrator] value=%d.\n", value);
	
	hrtimer_cancel(&vibe_timer);
	
	sh_vibrator_reqtime = value;
	cancel_work_sync(&work_hrtimer);
	cancel_work_sync(&work_vibrator_on);
	cancel_work_sync(&work_vibrator_off);
	
	if (value == 0)
		timed_vibrator_off(dev);
	else {
		sh_vibrator_reqtime = (value > 15000 ? 15000 : value);

#if defined( SHVIB_PAUSE_PEDOMETER )
#if defined( CONFIG_SENSORS_AMI603 )
		if (value >= 700) {
			pause_pedometer = 1;
		} else {
			pause_pedometer = 0;
		}
#elif defined( CONFIG_SH_YAS530 )
		pause_pedometer = 1;
#endif
#endif
		
		timed_vibrator_on(dev);
		
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	pr_debug("[shvibrator] timer stop.\n");
	/* add work queue for hrtimer -> */
#if 0
	timed_vibrator_off(NULL);
#else
	schedule_work(&work_hrtimer);
#endif
	/* add work queue for hrtimer <- */
	
	return HRTIMER_NORESTART;
}

/* add work queue for hrtimer -> */
static void hrtimer_work_func(struct work_struct *work)
{
#if 0
	timed_vibrator_off(NULL);
#else
	pmic_vibrator_off(NULL);
#endif
}
/* add work queue for hrtimer <- */

/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
int shvibrator_i2c_read(struct i2c_client *client,
									int reg, u8 *data, int size)
{
	int rc;
	u8 buf[2];
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags= 0,
			.len  = 1,
			.buf  = buf,
		},
		{
			.addr = client->addr,
			.flags= I2C_M_RD,
			.len  = size,
			.buf  = data,
		}
	};

	buf[0] = reg;
	rc = i2c_transfer(client->adapter, msg, 2);
	if(rc != 2){
		dev_err(&client->dev,
		       "shvibrator_i2c_read FAILED: read of register %d\n", reg);
		rc = -1;
		goto i2c_rd_exit;
	}
	
i2c_rd_exit:
	return rc;
}

int shvibrator_i2c_write(struct i2c_client *client,
									int reg, u8 data)
{
	int rc;
	u8 buf[2];
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags= 0,
		.len  = 2,
		.buf  = buf,
	};
	
	buf[0] = reg;
	buf[1] = data;
	rc = i2c_transfer(client->adapter, &msg, 1);
	if(rc != 1){
		dev_err(&client->dev,
		       "shvibrator_i2c_write FAILED: writing to reg %d\n", reg);
		rc = -1;
	}
	
	return rc;
}

static const struct i2c_device_id shvibrator_i2c_id[] = {
	{ SHVIB_I2C_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver shvibrator_i2c_driver = {
	.driver		= {
		.name = SHVIB_I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= shvibrator_i2c_probe,
	.remove		= __devexit_p(shvibrator_i2c_remove),
	.id_table	= shvibrator_i2c_id
};

static int shvibrator_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rc;
	struct shvib_i2c_data *sd;
	
	if(vib_data){
		rc = -EPERM;
		goto probe_exit;
	}
	
	sd = (struct shvib_i2c_data*)kzalloc(sizeof *sd, GFP_KERNEL);
	if(!sd){
		rc = -ENOMEM;
		goto probe_exit;
	}
	vib_data = sd;
	i2c_set_clientdata(client, sd);
	sd->client_p = client;
	
	return 0;
	
probe_exit:
	return rc;
}

static int shvibrator_i2c_remove(struct i2c_client *client)
{
	return 0;
}

int __init shvibrator_i2c_init(void)
{
	int ret;
	if (vibrator_init) return 0;
	ret = i2c_add_driver(&shvibrator_i2c_driver);
	vibrator_init = 1;
	return ret;
}

void shvibrator_i2c_exit(void)
{
	if (vibrator_init)
		i2c_del_driver(&shvibrator_i2c_driver);
}
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __init msm_init_pmic_vibrator(void)
{
/* SH_AUDIO_DRIVER ADD 2011/06/08 --> */
	int rc;

	struct pm_gpio vib_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L17,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.out_strength   = PM_GPIO_STRENGTH_LOW,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0

	};/* SH_AUDIO_DRIVER ADD 2011/06/08 <-- */

	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);
	/* add work queue for hrtimer -> */
	INIT_WORK(&work_hrtimer, hrtimer_work_func);
	/* add work queue for hrtimer <- */

/* SH_AUDIO_DRIVER ADD 2011/06/08 --> */

	rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN - 1),&vib_config);

	if(rc < 0){
		pr_err("linearVIB reset :gpio_config error\n");
	}

	rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN - 1),&vib_config);

	if(rc < 0){
		pr_err("linearVIB:gpio_config error\n");
	}

/* SH_AUDIO_DRIVER ADD 2011/06/08 <-- */
/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
	rc = shvibrator_i2c_init();
	if(rc < 0){
		printk(KERN_WARNING "linearVIB:shvibrator_i2c_init error\n");
	}
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	mutex_init(&sh_vibrator_pmic_mutex);
	
	return timed_output_dev_register(&pmic_vibrator);

}
module_init( msm_init_pmic_vibrator );

static void __exit msm_exit_pmic_vibrator(void)
{
/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
    shvibrator_i2c_exit();
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */
	timed_output_dev_unregister(&pmic_vibrator);
#if 0 /* Warning fix [start] */
/* SH_AUDIO_DRIVER ADD 20110607 -->*/
	gpio_free(SHVIB_RST_PDN);
	gpio_free(SHVIB_PDN);
/* SH_AUDIO_DRIVER ADD 20110607 <--*/
#endif /* Warning Fix [end] */
}
module_exit( msm_exit_pmic_vibrator );

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");
