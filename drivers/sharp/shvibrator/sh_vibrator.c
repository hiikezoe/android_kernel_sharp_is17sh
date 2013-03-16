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

#include <mach/pmic.h>
#include <mach/msm_rpcrouter.h>

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

#if (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 203)
#define PMIC_VIBRATOR_ON_LEV	(3100)
#elif (CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER == 105)
#define PMIC_VIBRATOR_ON_LEV	(2000)
#else /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER */
#define PMIC_VIBRATOR_ON_LEV	(3000)
#endif /* CONFIG_SH_AUDIO_DRIVER_MODEL_NUMBER */
#define PMIC_VIBRATOR_OFF_LEV	(   0)

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

static void set_pmic_vibrator(int on)
{
	mutex_lock(&sh_vibrator_pmic_mutex);
	if (on){
		pmic_vib_mot_set_polarity(PM_VIB_MOT_POL__ACTIVE_LOW);
		pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__DBUS2);
		pmic_vib_mot_set_volt(PMIC_VIBRATOR_ON_LEV);
		pr_debug("[shvibrator] PMIC vibrator On.\n");
	}else{
		pr_debug("[shvibrator] PMIC vibrator Off.\n");
		pmic_vib_mot_set_volt(0);
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

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __init msm_init_pmic_vibrator(void)
{
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);
	/* add work queue for hrtimer -> */
	INIT_WORK(&work_hrtimer, hrtimer_work_func);
	/* add work queue for hrtimer <- */

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;
	
	mutex_init(&sh_vibrator_pmic_mutex);
	
	return timed_output_dev_register(&pmic_vibrator);
}

static void __exit msm_exit_pmic_vibrator(void)
{
	timed_output_dev_unregister(&pmic_vibrator);
}
module_init( msm_init_pmic_vibrator );
module_exit( msm_exit_pmic_vibrator );

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");
