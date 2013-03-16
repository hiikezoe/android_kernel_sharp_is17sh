/*
 * MSM MDDI Transport
 *
 * Copyright (C) 2007 Google Incorporated
 * Copyright (c) 2007-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <asm/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#ifdef CONFIG_SHLCDC_BOARD
#include <sharp/shlcdc_kerl.h>
#include <linux/hrtimer.h>
#endif /* CONFIG_SHLCDC_BOARD */
#include "msm_fb.h"
#include "mddihosti.h"
#include "mddihost.h"
#include <mach/gpio.h>
#include <mach/clk.h>

#ifdef	CONFIG_SHLCDC_BOARD
#include <sharp/shlcdc_eventlog.h>
#include <linux/kmsg_dump.h>
#include <linux/notifier.h>
#include "../mach-msm/modem_notifier.h"
#endif	/* CONFIG_SHLCDC_BOARD */

static int mddi_probe(struct platform_device *pdev);
static int mddi_remove(struct platform_device *pdev);

static int mddi_off(struct platform_device *pdev);
static int mddi_on(struct platform_device *pdev);

#ifdef CONFIG_PM
static int mddi_suspend(struct platform_device *pdev, pm_message_t state);
static int mddi_resume(struct platform_device *pdev);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mddi_early_suspend(struct early_suspend *h);
static void mddi_early_resume(struct early_suspend *h);
#endif

static void pmdh_clk_disable(void);
static void pmdh_clk_enable(void);
static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;
static struct clk *mddi_clk;
static struct clk *mddi_pclk;
static struct mddi_platform_data *mddi_pdata;

DEFINE_MUTEX(mddi_timer_lock);

#ifdef	CONFIG_SHLCDC_BOARD
static int mddi_klog=0;
static int mddi_early_suspend_resume=0;
static int mddi_suspend_resume=0;
#ifdef CONFIG_ANDROID_ENGINEERING
module_param(mddi_klog, int, 0600);
module_param(mddi_early_suspend_resume, int, 0400);
module_param(mddi_suspend_resume, int, 0400);
#endif
#define pr_info_lv(fmt, arg...)	\
	if(mddi_klog>0) pr_info(fmt, ## arg);

#ifndef SH_BUILD_ID
#define	SH_BUILD_ID	"S0000"
#endif

enum enum_mddi_event {
	EEVENT_MDDI_EARLY_SUSPEND,
	EEVENT_MDDI_EARLY_RESUME,
	EEVENT_MDDI_SUSPEND,
	EEVENT_MDDI_RESUME,
	EEVENT_MDDI_ON,
	EEVENT_MDDI_OFF,
};

#define	EVENTLOG_MDDI	0x100
static void mddi_eventlog_rec(unsigned long event, unsigned long param)
{
	shlcdc_eventlog_rec(SHLCDC_EVENTLOG_FB + EVENTLOG_MDDI + event, param);
}

static void mddi_kmsg_dumper(struct kmsg_dumper *dumper,
		enum kmsg_dump_reason reason, const char *s1, unsigned long l1,
		const char *s2, unsigned long l2)
{
	if(reason == KMSG_DUMP_PANIC) return;
	pr_info("[####]%s:reason=%d, mddi_early_suspend_resume=%d, mddi_suspend_resume=%d\n", __func__, reason, mddi_early_suspend_resume, mddi_suspend_resume);
}
static struct kmsg_dumper kmsg_dumper_mddi = {
	.dump = mddi_kmsg_dumper,
};

static int mddi_notifier_call(struct notifier_block *this,
				  unsigned long code,
				  void *_cmd)
{
	pr_info("[####]%s:mddi_early_suspend_resume=%d, mddi_suspend_resume=%d\n", __func__, mddi_early_suspend_resume, mddi_suspend_resume);
	return NOTIFY_DONE;
}

static struct notifier_block nb_mddi = {
	.notifier_call = mddi_notifier_call,
};
#endif	/* CONFIG_SHLCDC_BOARD */


static int mddi_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int mddi_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static int mddi_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: idling...\n");
	return 0;
}

static struct dev_pm_ops mddi_dev_pm_ops = {
	.runtime_suspend = mddi_runtime_suspend,
	.runtime_resume = mddi_runtime_resume,
	.runtime_idle = mddi_runtime_idle,
};

static int pmdh_clk_status;
int irq_enabled;
unsigned char mddi_timer_shutdown_flag;

static struct platform_driver mddi_driver = {
	.probe = mddi_probe,
	.remove = mddi_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_PM
	.suspend = mddi_suspend,
	.resume = mddi_resume,
#endif
#endif
	.shutdown = NULL,
	.driver = {
		.name = "mddi",
		.pm = &mddi_dev_pm_ops,
		   },
};

extern int int_mddi_pri_flag;
DEFINE_MUTEX(pmdh_clk_lock);

#ifdef CONFIG_SHLCDC_BOARD
extern struct semaphore shdisp_vdlink_mutex;
static volatile int no_set_power_flag = FALSE;

void mddi_suspend_shdisp(int sw)
{
    pm_message_t state;

#ifdef	CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s:sw=%d\n", __func__, sw);
#endif	/* CONFIG_SHLCDC_BOARD */
	state.event = PM_EVENT_SUSPEND;

    no_set_power_flag = TRUE;
    if (sw == TRUE) {
        msleep(40);
        mddi_suspend(NULL, state);
    } else {
        mddi_resume(NULL);
    }
    no_set_power_flag = FALSE;
    return;
}
#endif

int pmdh_clk_func(int value)
{
	int ret = 0;

	switch (value) {
	case 0:
		pmdh_clk_disable();
		break;
	case 1:
		pmdh_clk_enable();
		break;
	case 2:
	default:
		mutex_lock(&pmdh_clk_lock);
		ret = pmdh_clk_status;
		mutex_unlock(&pmdh_clk_lock);
		break;
	}
	return ret;
}

static void pmdh_clk_disable()
{
	mutex_lock(&pmdh_clk_lock);
	if (pmdh_clk_status == 0) {
		mutex_unlock(&pmdh_clk_lock);
		return;
	}

	if (mddi_host_timer.function) {
		mutex_lock(&mddi_timer_lock);
		mddi_timer_shutdown_flag = 1;
		mutex_unlock(&mddi_timer_lock);
		del_timer_sync(&mddi_host_timer);
		mutex_lock(&mddi_timer_lock);
		mddi_timer_shutdown_flag = 0;
		mutex_unlock(&mddi_timer_lock);
	}
	if (int_mddi_pri_flag && irq_enabled) {
		disable_irq(INT_MDDI_PRI);
		irq_enabled = 0;
	}

	if (mddi_clk) {
		clk_disable(mddi_clk);
		pmdh_clk_status = 0;
	}
	if (mddi_pclk)
		clk_disable(mddi_pclk);
	mutex_unlock(&pmdh_clk_lock);
}

static void pmdh_clk_enable()
{
	mutex_lock(&pmdh_clk_lock);
	if (pmdh_clk_status == 1) {
		mutex_unlock(&pmdh_clk_lock);
		return;
	}

	if (mddi_clk) {
		clk_enable(mddi_clk);
		pmdh_clk_status = 1;
	}
	if (mddi_pclk)
		clk_enable(mddi_pclk);

	if (int_mddi_pri_flag && !irq_enabled) {
		enable_irq(INT_MDDI_PRI);
		irq_enabled = 1;
	}

	if (mddi_host_timer.function)
		mddi_host_timer_service(0);

	mutex_unlock(&pmdh_clk_lock);
}

static int mddi_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	boolean dma_pending, dma_update_flag;
	int ret, i;
#ifdef	CONFIG_SHLCDC_BOARD
	struct timespec tu;
#endif	/* CONFIG_SHLCDC_BOARD */

	mfd = platform_get_drvdata(pdev);

#ifdef	CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s\n", __func__);
	mddi_eventlog_rec(EEVENT_MDDI_OFF, 0);
#endif	/* CONFIG_SHLCDC_BOARD */
	for (i = 0; i < 6; i++) {
		dma_update_flag = mfd->dma_update_flag;
		dma_pending = mfd->dma->busy;
		if (dma_update_flag && !dma_pending)
			break;
#ifdef	CONFIG_SHLCDC_BOARD
		tu.tv_sec = 0;
		tu.tv_nsec = 5 * 1000000;
		hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
#else
		msleep(5);
#endif	/* CONFIG_SHLCDC_BOARD */
	}

	pmdh_clk_enable();
	ret = panel_next_off(pdev);
	pmdh_clk_disable();

	if (mddi_pdata && mddi_pdata->mddi_power_save)
		mddi_pdata->mddi_power_save(0);
#ifdef CONFIG_MSM_BUS_SCALING
	mdp_bus_scale_update_request(0);
#else
	if (mfd->ebi1_clk)
		clk_disable(mfd->ebi1_clk);
#endif
	pm_runtime_put(&pdev->dev);
	return ret;
}

static int mddi_on(struct platform_device *pdev)
{
	int ret = 0;
	u32 clk_rate;
	struct msm_fb_data_type *mfd;
#ifdef ENABLE_FWD_LINK_SKEW_CALIBRATION
	mddi_host_type host_idx = MDDI_HOST_PRIM;
	u32 stat_reg;
#endif

#ifdef	CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s\n", __func__);
	mddi_eventlog_rec(EEVENT_MDDI_ON, 0);
#endif	/* CONFIG_SHLCDC_BOARD */
	mfd = platform_get_drvdata(pdev);
	pm_runtime_get(&pdev->dev);
	if (mddi_pdata && mddi_pdata->mddi_power_save)
		mddi_pdata->mddi_power_save(1);

	pmdh_clk_enable();
#ifdef ENABLE_FWD_LINK_SKEW_CALIBRATION
	if (mddi_client_type < 2) {
		/* For skew calibration, clock should be less than 50MHz */
		clk_rate = clk_round_rate(mddi_clk, 49000000);
		if (!clk_set_rate(mddi_clk, clk_rate)) {
			stat_reg = mddi_host_reg_in(STAT);
			printk(KERN_DEBUG "\n stat_reg = 0x%x", stat_reg);
			mddi_host_reg_out(CMD, MDDI_CMD_HIBERNATE);
			if (stat_reg & (0x1 << 4))
				mddi_host_reg_out(CMD, MDDI_CMD_LINK_ACTIVE);

			mddi_host_reg_out(CMD, MDDI_CMD_SEND_RTD);
			mddi_send_fw_link_skew_cal(host_idx);
			mddi_host_reg_out(CMD, MDDI_CMD_SEND_RTD);
			mddi_host_reg_out(CMD, MDDI_CMD_HIBERNATE | 1);
		} else {
			printk(KERN_ERR "%s: clk_set_rate failed\n",
				__func__);
		}
	}
#endif

	clk_rate = mfd->fbi->var.pixclock;
	clk_rate = min(clk_rate, mfd->panel_info.clk_max);

	if (mddi_pdata &&
	    mddi_pdata->mddi_sel_clk &&
	    mddi_pdata->mddi_sel_clk(&clk_rate))
			printk(KERN_ERR
			  "%s: can't select mddi io clk targate rate = %d\n",
			  __func__, clk_rate);

	clk_rate = clk_round_rate(mddi_clk, clk_rate);
	if (clk_set_rate(mddi_clk, clk_rate) < 0)
		printk(KERN_ERR "%s: clk_set_rate failed\n",
			__func__);

#ifdef CONFIG_MSM_BUS_SCALING
	mdp_bus_scale_update_request(2);
#else
	if (mfd->ebi1_clk)
		clk_enable(mfd->ebi1_clk);
#endif
	ret = panel_next_on(pdev);

	return ret;
}

static int mddi_resource_initialized;

static int mddi_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct platform_device *mdp_dev = NULL;
	struct msm_fb_panel_data *pdata = NULL;
	int rc;
	resource_size_t size ;
	u32 clk_rate;

	if ((pdev->id == 0) && (pdev->num_resources >= 0)) {
		mddi_pdata = pdev->dev.platform_data;

		size =  resource_size(&pdev->resource[0]);
		msm_pmdh_base =  ioremap(pdev->resource[0].start, size);

		MSM_FB_INFO("primary mddi base phy_addr = 0x%x virt = 0x%x\n",
				pdev->resource[0].start, (int) msm_pmdh_base);

		if (unlikely(!msm_pmdh_base))
			return -ENOMEM;

		if (mddi_pdata && mddi_pdata->mddi_power_save)
			mddi_pdata->mddi_power_save(1);

		mddi_resource_initialized = 1;
		return 0;
	}

	if (!mddi_resource_initialized)
		return -EPERM;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;

	mdp_dev = platform_device_alloc("mdp", pdev->id);
	if (!mdp_dev)
		return -ENOMEM;

	/*
	 * link to the latest pdev
	 */
	mfd->pdev = mdp_dev;
	mfd->dest = DISPLAY_LCD;

	/*
	 * alloc panel device data
	 */
	if (platform_device_add_data
	    (mdp_dev, pdev->dev.platform_data,
	     sizeof(struct msm_fb_panel_data))) {
		printk(KERN_ERR "mddi_probe: platform_device_add_data failed!\n");
		platform_device_put(mdp_dev);
		return -ENOMEM;
	}
	/*
	 * data chain
	 */
	pdata = mdp_dev->dev.platform_data;
	pdata->on = mddi_on;
	pdata->off = mddi_off;
	pdata->next = pdev;
	pdata->clk_func = pmdh_clk_func;
	/*
	 * get/set panel specific fb info
	 */
	mfd->panel_info = pdata->panel_info;

	if (mfd->index == 0)
		mfd->fb_imgType = MSMFB_DEFAULT_TYPE;
	else
		mfd->fb_imgType = MDP_RGB_565;

	clk_rate = mfd->panel_info.clk_max;
	if (mddi_pdata &&
	    mddi_pdata->mddi_sel_clk &&
	    mddi_pdata->mddi_sel_clk(&clk_rate))
			printk(KERN_ERR
			  "%s: can't select mddi io clk targate rate = %d\n",
			  __func__, clk_rate);

	if (clk_set_max_rate(mddi_clk, clk_rate) < 0)
		printk(KERN_ERR "%s: clk_set_max_rate failed\n", __func__);
	mfd->panel_info.clk_rate = mfd->panel_info.clk_min;

	if (!mddi_client_type)
		mddi_client_type = mfd->panel_info.lcd.rev;
	else if (!mfd->panel_info.lcd.rev)
		printk(KERN_ERR
		"%s: mddi client is trying to revert back to type 1	!!!\n",
		__func__);

	/*
	 * set driver data
	 */
	platform_set_drvdata(mdp_dev, mfd);
	rc = pm_runtime_set_active(&pdev->dev);
	if (rc < 0)
		printk(KERN_ERR "pm_runtime: fail to set active\n");

	rc = 0;
	pm_runtime_enable(&pdev->dev);
#ifndef CONFIG_MSM_BUS_SCALING
	mfd->ebi1_clk = clk_get(NULL, "ebi1_mddi_clk");
	if (IS_ERR(mfd->ebi1_clk))
		return PTR_ERR(mfd->ebi1_clk);
	clk_set_rate(mfd->ebi1_clk, 65000000);
#endif
	/*
	 * register in mdp driver
	 */
	rc = platform_device_add(mdp_dev);
	if (rc)
		goto mddi_probe_err;

	pdev_list[pdev_list_cnt++] = pdev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	mfd->mddi_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	mfd->mddi_early_suspend.suspend = mddi_early_suspend;
	mfd->mddi_early_suspend.resume = mddi_early_resume;
	register_early_suspend(&mfd->mddi_early_suspend);
#endif

	return 0;

mddi_probe_err:
	platform_device_put(mdp_dev);
	return rc;
}

static int mddi_pad_ctrl;
static int mddi_power_locked;

int mddi_client_power(unsigned int client_id)
{
	int ret = 0;
#ifdef	CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s:client_id=%d\n", __func__, client_id);
#endif	/* CONFIG_SHLCDC_BOARD */
	if (mddi_pdata && mddi_pdata->mddi_client_power)
		ret = mddi_pdata->mddi_client_power(client_id);
	return ret;
}

void mddi_disable(int lock)
{
	mddi_host_type host_idx = MDDI_HOST_PRIM;

#ifdef	CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s\n", __func__);
#endif	/* CONFIG_SHLCDC_BOARD */
	if (mddi_power_locked)
		return;

	if (lock)
		mddi_power_locked = 1;
	pmdh_clk_enable();

	mddi_pad_ctrl = mddi_host_reg_in(PAD_CTL);
	mddi_host_reg_out(PAD_CTL, 0x0);

	pmdh_clk_disable();

	if (mddi_pdata && mddi_pdata->mddi_power_save)
		mddi_pdata->mddi_power_save(0);
}

#ifdef CONFIG_PM
static int mddi_is_in_suspend;
#ifdef	CONFIG_SHLCDC_BOARD
#ifdef CONFIG_ANDROID_ENGINEERING
module_param(mddi_is_in_suspend, int, 0400);
#endif
#endif	/* CONFIG_SHLCDC_BOARD */

static int mddi_suspend(struct platform_device *pdev, pm_message_t state)
{
	mddi_host_type host_idx = MDDI_HOST_PRIM;
#ifdef	CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s\n", __func__);
	mddi_suspend_resume=1;
	mddi_eventlog_rec(EEVENT_MDDI_SUSPEND, 0);
#endif	/* CONFIG_SHLCDC_BOARD */
	if (mddi_is_in_suspend)
		return 0;

	mddi_is_in_suspend = 1;

	if (mddi_power_locked)
		return 0;

	pmdh_clk_enable();

	mddi_pad_ctrl = mddi_host_reg_in(PAD_CTL);
	mddi_host_reg_out(PAD_CTL, 0x0);

	pmdh_clk_disable();

#ifdef CONFIG_SHLCDC_BOARD
    if (no_set_power_flag == FALSE) {
        shlcdc_api_set_power_mode(SHLCDC_DEV_TYPE_MDDI, SHLCDC_DEV_PWR_OFF);
    }
#endif

	return 0;
}

static int mddi_resume(struct platform_device *pdev)
{
	mddi_host_type host_idx = MDDI_HOST_PRIM;

#ifdef	CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s\n", __func__);
	mddi_suspend_resume=0;
	mddi_eventlog_rec(EEVENT_MDDI_RESUME, 0);
#endif	/* CONFIG_SHLCDC_BOARD */
	if (!mddi_is_in_suspend)
		return 0;

	mddi_is_in_suspend = 0;

	if (mddi_power_locked)
		return 0;

#ifdef CONFIG_SHLCDC_BOARD
    if (no_set_power_flag == FALSE) {
        shlcdc_api_set_power_mode(SHLCDC_DEV_TYPE_MDDI, SHLCDC_DEV_PWR_ON);
    }
#endif

	pmdh_clk_enable();

	mddi_host_reg_out(PAD_CTL, mddi_pad_ctrl);


	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mddi_early_suspend(struct early_suspend *h)
{
	pm_message_t state;
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
							mddi_early_suspend);
#ifdef CONFIG_SHLCDC_BOARD
	struct timespec tu;
#endif

#ifdef	CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s\n", __func__);
	mddi_early_suspend_resume=1;
	mddi_eventlog_rec(EEVENT_MDDI_EARLY_SUSPEND, 0);
#endif	/* CONFIG_SHLCDC_BOARD */
	state.event = PM_EVENT_SUSPEND;
#ifdef CONFIG_SHLCDC_BOARD
	tu.tv_sec = 0;
	tu.tv_nsec = 40 * 1000000;
	hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
#endif
	mddi_suspend(mfd->pdev, state);

#ifdef CONFIG_SHLCDC_BOARD
    up(&shdisp_vdlink_mutex);
#endif
}

static void mddi_early_resume(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
							mddi_early_suspend);
#ifdef CONFIG_SHLCDC_BOARD
	pr_info_lv("[####]%s\n", __func__);
	mddi_early_suspend_resume=0;
	mddi_eventlog_rec(EEVENT_MDDI_EARLY_RESUME, 0);
    down(&shdisp_vdlink_mutex);
#endif
	mddi_resume(mfd->pdev);
}
#endif

static int mddi_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (mddi_host_timer.function) {
		mutex_lock(&mddi_timer_lock);
		mddi_timer_shutdown_flag = 1;
		mutex_unlock(&mddi_timer_lock);
		del_timer_sync(&mddi_host_timer);
		mutex_lock(&mddi_timer_lock);
		mddi_timer_shutdown_flag = 0;
		mutex_unlock(&mddi_timer_lock);
	}

	iounmap(msm_pmdh_base);

	return 0;
}

static int mddi_register_driver(void)
{
	return platform_driver_register(&mddi_driver);
}

static int __init mddi_driver_init(void)
{
	int ret;
	unsigned long rate;
	pmdh_clk_status = 0;

	mddi_clk = clk_get(NULL, "mddi_clk");
	if (IS_ERR(mddi_clk)) {
		printk(KERN_ERR "can't find mddi_clk\n");
		return PTR_ERR(mddi_clk);
	}
	rate = clk_round_rate(mddi_clk, 49000000);
	ret = clk_set_rate(mddi_clk, rate);
	if (ret)
		printk(KERN_ERR "Can't set mddi_clk min rate to %lu\n", rate);

	printk(KERN_INFO "mddi_clk init rate is %lu\n",
		clk_get_rate(mddi_clk));
	mddi_pclk = clk_get(NULL, "mddi_pclk");
	if (IS_ERR(mddi_pclk))
		mddi_pclk = NULL;
	pmdh_clk_enable();

	ret = mddi_register_driver();
	if (ret) {
		pmdh_clk_disable();
		clk_put(mddi_clk);
		if (mddi_pclk)
			clk_put(mddi_pclk);
		printk(KERN_ERR "mddi_register_driver() failed!\n");
		return ret;
	}

#ifdef	CONFIG_SHLCDC_BOARD
	if(strncmp(SH_BUILD_ID, "S", 1) != 0){
		ret = kmsg_dump_register(&kmsg_dumper_mddi);
		if (ret) {
			printk(KERN_ERR "%s: registering kmsg dumper failed", __func__);
		}
		modem_register_notifier(&nb_mddi);
	}
#endif	/* CONFIG_SHLCDC_BOARD */

	mddi_init();

	return ret;
}

module_init(mddi_driver_init);
