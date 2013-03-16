/* drivers/media/video/msm/sh_ov7692.c
 *
 * Copyright (C) 2009-2011 SHARP CORPORATION
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/board.h>

#include <mach/gpio.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/slab.h>

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <media/msm_camera.h>
/* SH_OV7692 -> */
#include <media/sh_camera.h>
/* SH_OV7692 <- */
#include <mach/camera.h>

#include "sh_ov7692.h"

#include <sharp/shlcdc_kerl.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <asm/pgtable.h>

#define SH_OV7692_GPIO_CVREF_A          2
#define SH_OV7692_DEVICE_NAME           "sh_ov7692"

static int sh_ov7692_irq_port = 0;

struct shlcdc_subscribe sh_ov7692_gol_csiint_subscribe;

struct sh_ov7692_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
};

static struct sh_ov7692_ctrl_t *sh_ov7692_ctrl;

struct sh_ov7692_sync_head_t {
    struct file         *file;
    struct cdev         cdev;
    spinlock_t          irq_event_q_lock;
    struct list_head    irq_event_q;
    uint32              irq_event_count;
    wait_queue_head_t   irq_event_wait;
    struct wake_lock    wake_lock;
    struct mutex        mut_lock;
};
static struct sh_ov7692_sync_head_t *sh_ov7692_sync_head = 0;

struct sh_ov7692_irq_msg_t {
    struct list_head    list;
    uint16              irq_number;
    struct shcam_ctrl_cmd   ctrl_data;
};

static irqreturn_t sh_ov7692_drv_gpio_vsync_isr(int irq_num, void *data);
static int sh_ov7692_drv_get_irq_num( struct sensor_cfg_data *cfg_data_p );
static int sh_ov7692_drv_enable_irq( struct sensor_cfg_data *cfg_data_p );
static int sh_ov7692_drv_disable_irq( struct sensor_cfg_data *cfg_data_p );
static int sh_ov7692_drv_camif_pad_reg_reset( struct sensor_cfg_data *cfg_data_p );
static int sh_ov7692_drv_sensor_init(void);
static int sh_ov7692_drv_sensor_off(void);
static int sh_ov7692_sys_open(void);
static void sh_ov7692_irq_msg_free(struct sh_ov7692_sync_head_t *sync);

static void sh_ov7692_mipi_err_isr(void);
static int sh_ov7692_CSI_request_irq(void);
static int sh_ov7692_CSI_free_irq(void);

static int sh_ov7692_request_irq(void);
static int sh_ov7692_free_irq(void);
static int sh_ov7692_probe_init_sensor(const struct msm_camera_sensor_info *data);

static int sh_ov7692_timeout_irq_num( struct sensor_cfg_data *cfg_data_p );
static int sh_ov7692_event_isr( struct sensor_cfg_data *cfg_data_p );
static int sh_ov7692_frameevent_isr( struct sensor_cfg_data *cfg_data_p );

static irqreturn_t sh_ov7692_drv_gpio_vsync_isr(int irq_num, void *data)
{
    irqreturn_t rc = IRQ_HANDLED;
    unsigned long flags = 0;
    struct sh_ov7692_irq_msg_t *qmsg = NULL;

    do {
        qmsg = kzalloc(sizeof(struct sh_ov7692_irq_msg_t), GFP_ATOMIC);
        if (qmsg == NULL) {
            break;
        }

        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        qmsg->irq_number = (uint16)CAM_INT_TYPE_VS;

        list_add_tail(&qmsg->list, &sh_ov7692_sync_head->irq_event_q);

        sh_ov7692_sync_head->irq_event_count++;

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        wake_up(&sh_ov7692_sync_head->irq_event_wait);

    } while (0);

    return rc;
}

static void sh_ov7692_mipi_err_isr(void)
{
    unsigned long flags = 0;
    struct sh_ov7692_irq_msg_t *qmsg = NULL;

    do {
        qmsg = kzalloc(sizeof(struct sh_ov7692_irq_msg_t), GFP_ATOMIC);
        if (qmsg == NULL) {
            break;
        }

        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        qmsg->irq_number = (uint16)CAM_INT_TYPE_MIPI_ERR;

        list_add_tail(&qmsg->list, &sh_ov7692_sync_head->irq_event_q);

        sh_ov7692_sync_head->irq_event_count++;

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        wake_up(&sh_ov7692_sync_head->irq_event_wait);

    } while (0);

}
static int sh_ov7692_event_isr( struct sensor_cfg_data *cfg_data_p )
{
    int ret = 0;
    unsigned long flags = 0;
    struct sh_ov7692_irq_msg_t *qmsg = NULL;

    do {
        cfg_data_p->cfg.sh_ctrl_data.status = 0;

        qmsg = kzalloc(sizeof(struct sh_ov7692_irq_msg_t), GFP_ATOMIC);
        if (qmsg == NULL) {
            ret = -EFAULT;
            break;
        }

        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        qmsg->irq_number = (uint16)CAM_INT_TYPE_EVENT_ISR;

        qmsg->ctrl_data.event_data = (struct shcam_event_ctrl_cmd)cfg_data_p->cfg.sh_ctrl_data.event_data;

        list_add_tail(&qmsg->list, &sh_ov7692_sync_head->irq_event_q);

        sh_ov7692_sync_head->irq_event_count++;

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        wake_up(&sh_ov7692_sync_head->irq_event_wait);

    } while (0);

    return ret;
}

static int sh_ov7692_frameevent_isr( struct sensor_cfg_data *cfg_data_p )
{
    int ret = 0;
    unsigned long flags = 0;
    struct sh_ov7692_irq_msg_t *qmsg = NULL;

    do {
        cfg_data_p->cfg.sh_ctrl_data.status = 0;

        qmsg = kzalloc(sizeof(struct sh_ov7692_irq_msg_t ), GFP_ATOMIC);
        if (qmsg == NULL) {
            ret = -EFAULT;
            break;
        }

        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        qmsg->irq_number = (uint16)CAM_INT_TYPE_FRAMEEVENT_ISR;

        qmsg->ctrl_data.event_data = (struct shcam_event_ctrl_cmd)cfg_data_p->cfg.sh_ctrl_data.event_data;

        list_add_tail(&qmsg->list, &sh_ov7692_sync_head->irq_event_q);

        sh_ov7692_sync_head->irq_event_count++;

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        wake_up(&sh_ov7692_sync_head->irq_event_wait);

    } while (0);

    return ret;
}

static int sh_ov7692_drv_get_irq_num( struct sensor_cfg_data *cfg_data_p )
{
    int ret = 0;
    unsigned long flags = 0;
    struct sh_ov7692_irq_msg_t *qmsg = NULL;

    do {
        cfg_data_p->cfg.sh_ctrl_data.status = 0;
        cfg_data_p->cfg.sh_ctrl_data.irq_number  = 0;

        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        if ( !list_empty( &sh_ov7692_sync_head->irq_event_q ) ){
            qmsg = list_first_entry(&sh_ov7692_sync_head->irq_event_q, struct sh_ov7692_irq_msg_t, list);
            list_del(&qmsg->list);
            if(qmsg->irq_number == CAM_INT_TYPE_EVENT_ISR ||
               qmsg->irq_number == CAM_INT_TYPE_FRAMEEVENT_ISR) {
                cfg_data_p->cfg.sh_ctrl_data.event_data = (struct shcam_event_ctrl_cmd)qmsg->ctrl_data.event_data;
            }
            cfg_data_p->cfg.sh_ctrl_data.irq_number = qmsg->irq_number;
            sh_ov7692_sync_head->irq_event_count--;
            kfree(qmsg);
        }
        else{
            cfg_data_p->cfg.sh_ctrl_data.status = -1;
        }

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);
    } while (0);

    return ret;
}

static int sh_ov7692_timeout_irq_num( struct sensor_cfg_data *cfg_data_p )
{
    int ret = 0;

    do {
        cfg_data_p->cfg.sh_ctrl_data.status = 0;
        cfg_data_p->cfg.sh_ctrl_data.irq_number  = (uint16)CAM_INT_TYPE_TIMEOUT;
    } while (0);

    return ret;
}

static int sh_ov7692_drv_enable_irq( struct sensor_cfg_data *cfg_data_p )
{
    int ret = 0;
    unsigned long flags = 0;

    do {
        cfg_data_p->cfg.sh_ctrl_data.status = 0;

        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        enable_irq(sh_ov7692_irq_port);

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);

    } while (0);

    return ret;
}

static int sh_ov7692_drv_disable_irq( struct sensor_cfg_data *cfg_data_p )
{
    int ret = 0;
    unsigned long flags = 0;

    do {
        cfg_data_p->cfg.sh_ctrl_data.status = 0;

        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        disable_irq(sh_ov7692_irq_port);

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);

    } while (0);

    return ret;
}

static int sh_ov7692_drv_camif_pad_reg_reset( struct sensor_cfg_data *cfg_data_p )
{
    int ret = 0;

    do {
        cfg_data_p->cfg.sh_ctrl_data.status = 0;

        msm_camio_camif_pad_reg_reset();
        udelay(1500);
    } while (0);

    return ret;
}

static int sh_ov7692_CSI_request_irq(void)
{
    uint16 ret = 0;

    mutex_lock(&sh_ov7692_sync_head->mut_lock);

    do {

        sh_ov7692_gol_csiint_subscribe.event_type = SHLCDC_EVENT_TYPE_CSI;
        sh_ov7692_gol_csiint_subscribe.callback   = sh_ov7692_mipi_err_isr;
        ret = shlcdc_api_event_subscribe(&sh_ov7692_gol_csiint_subscribe);
        if(ret != SHLCDC_RESULT_SUCCESS) {
            break;
        }

    } while (0);

    mutex_unlock(&sh_ov7692_sync_head->mut_lock);

    return ret;
}

static int sh_ov7692_CSI_free_irq(void)
{
    int ret = 0;

    mutex_lock(&sh_ov7692_sync_head->mut_lock);

    do {
        ret = shlcdc_api_event_unsubscribe(SHLCDC_EVENT_TYPE_CSI);
        if(ret != SHLCDC_RESULT_SUCCESS) {
        }
    } while (0);
    
    mutex_unlock(&sh_ov7692_sync_head->mut_lock);

    return ret;
}

static int sh_ov7692_request_irq(void)
{
    int ret = 0;
    unsigned long flags = 0;

    mutex_lock(&sh_ov7692_sync_head->mut_lock);

    do {
        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        sh_ov7692_irq_port = MSM_GPIO_TO_INT(SH_OV7692_GPIO_CVREF_A);
        ret = request_irq(sh_ov7692_irq_port, sh_ov7692_drv_gpio_vsync_isr, IRQF_TRIGGER_FALLING, "sh_ov7692_vsync", 0);
        disable_irq(sh_ov7692_irq_port);

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);

    } while (0);

    mutex_unlock(&sh_ov7692_sync_head->mut_lock);

    return ret;
}

static int sh_ov7692_free_irq(void)
{
    int ret = 0;
    unsigned long flags = 0;

    mutex_lock(&sh_ov7692_sync_head->mut_lock);

    do {
        spin_lock_irqsave(&sh_ov7692_sync_head->irq_event_q_lock, flags);

        free_irq(sh_ov7692_irq_port, 0);

        spin_unlock_irqrestore(&sh_ov7692_sync_head->irq_event_q_lock, flags);
    }while(0);
    
    mutex_unlock(&sh_ov7692_sync_head->mut_lock);

    return ret;
}

static int sh_ov7692_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	CDBG("init entry \n");

	return rc;
}

static int sh_ov7692_drv_sensor_init(void)
{
    int ret = 0;

    gpio_tlmm_config( GPIO_CFG(  2, 0,  GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,    GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(  4, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(  5, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(  6, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(  7, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(  8, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(  9, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG( 10, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG( 11, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG( 12, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG( 13, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG( 14, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_ENABLE );

#if 0
    gpio_tlmm_config( GPIO_CFG(124, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(126, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(127, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(128, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(129, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(130, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(131, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(132, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(160, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(161, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(162, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_ENABLE );
#endif

    return ret;
}

static int sh_ov7692_drv_sensor_off(void)
{
    int ret = 0;

    gpio_tlmm_config( GPIO_CFG(  2, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(  4, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(  5, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(  6, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(  7, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(  8, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(  9, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG( 10, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG( 11, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG( 12, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG( 13, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG( 14, 1,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA),  GPIO_CFG_DISABLE );

#if 0
    gpio_tlmm_config( GPIO_CFG(124, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(126, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(127, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(128, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(129, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(130, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(131, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(132, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(160, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(161, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
    gpio_tlmm_config( GPIO_CFG(162, 0,  GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_10MA), GPIO_CFG_DISABLE );
#endif

    return ret;
}

static int sh_ov7692_sys_open(void)
{
    int ret = 0;

    wake_lock(&sh_ov7692_sync_head->wake_lock);
    mutex_lock(&sh_ov7692_sync_head->mut_lock);

    do {
        ret = sh_ov7692_drv_sensor_init();
        if (ret < 0) {
            break;
        }

    } while (0);

    mutex_unlock(&sh_ov7692_sync_head->mut_lock);

    return ret;
}

static void sh_ov7692_irq_msg_free(struct sh_ov7692_sync_head_t *sync)
{
    struct sh_ov7692_irq_msg_t *qmsg = NULL;

    for (;;) {
        if (!list_empty(&sync->irq_event_q)) {
            qmsg = list_first_entry(&sync->irq_event_q, struct sh_ov7692_irq_msg_t, list);
            list_del(&qmsg->list);
            sync->irq_event_count--;
            kfree(qmsg);
        }
        else {
            break;
        }
    }

    return;
}

int sh_ov7692_sensor_init(const struct msm_camera_sensor_info *data)
{
    int rc = 0;

    do{
        sh_ov7692_ctrl = kzalloc(sizeof(struct sh_ov7692_ctrl_t), GFP_ATOMIC);
        if (!sh_ov7692_ctrl) {
            CDBG("[%s][L:%d] kzalloc sh_ov7692_ctrl area failed\n", __func__, __LINE__ );
            rc = -ENOMEM;
            break;
        }

        if (data)
            sh_ov7692_ctrl->sensordata = data;

        sh_ov7692_sync_head = kzalloc(sizeof(struct sh_ov7692_sync_head_t), GFP_ATOMIC);
        if (!sh_ov7692_sync_head) {
            CDBG("[%s][L:%d] kzalloc sh_ov7692_sync_head area failed\n", __func__, __LINE__ );
            kfree(sh_ov7692_ctrl);
            rc = -ENOMEM;
            break;
        }

        memset(sh_ov7692_sync_head, 0x00, sizeof(struct sh_ov7692_sync_head_t ));

        spin_lock_init(&sh_ov7692_sync_head->irq_event_q_lock);
        INIT_LIST_HEAD(&sh_ov7692_sync_head->irq_event_q);
        init_waitqueue_head(&sh_ov7692_sync_head->irq_event_wait);
        wake_lock_init(&sh_ov7692_sync_head->wake_lock, WAKE_LOCK_SUSPEND, SH_OV7692_DEVICE_NAME);
        sh_ov7692_sync_head->irq_event_count = 0;
        mutex_init(&sh_ov7692_sync_head->mut_lock);

        rc = sh_ov7692_sys_open();
        if (rc != 0) {
            CDBG("[%s][L:%d] sh_ov7692_sys_open func failed\n", __func__, __LINE__ );
            kfree(sh_ov7692_ctrl);
            kfree(sh_ov7692_sync_head);
            rc = -ENOMEM;
            break;
        }
    }while(0);

    return rc;
}

int sh_ov7692_sensor_release(void)
{
    int ret = 0;

    mutex_lock(&sh_ov7692_sync_head->mut_lock);

    do {
        ret = sh_ov7692_drv_sensor_off();
        if (ret < 0) {
            CDBG("[%s][L:%d] sh_ov7692_drv_sensor_off failed\n", __func__, __LINE__ );
//            break;
        }

        sh_ov7692_irq_msg_free(sh_ov7692_sync_head);
        wake_unlock(&sh_ov7692_sync_head->wake_lock);

        wake_lock_destroy(&sh_ov7692_sync_head->wake_lock);

        mutex_unlock(&sh_ov7692_sync_head->mut_lock);

        kfree(sh_ov7692_sync_head);
        kfree(sh_ov7692_ctrl);
    } while (0);

    return ret;
}

int sh_ov7692_sensor_config(void __user *argp)
{
    int ret = 0;
    struct sensor_cfg_data cfg_data_p;

    do {
        if (copy_from_user(&cfg_data_p, (void *)argp, sizeof(struct sensor_cfg_data))){
            CDBG( "[%s][L:%d] copy_from_user err \n", __func__, __LINE__ );
            ret =  -EFAULT;
            break;
        }

        CDBG("sh_ov7692_ioctl, cfgtype = %d\n", cfg_data_p.cfgtype);

        switch (cfg_data_p.cfgtype) {
        case SH_CFG_READ_IRQ_KIND:
            ret = wait_event_interruptible_timeout(
                        sh_ov7692_sync_head->irq_event_wait,
                        !list_empty(&sh_ov7692_sync_head->irq_event_q),
                        msecs_to_jiffies(500));
            if (ret == 0) {
                sh_ov7692_timeout_irq_num( &cfg_data_p );
                return -ETIMEDOUT;
            }
            else {
                ret = sh_ov7692_drv_get_irq_num( &cfg_data_p );
            }
            break;
        case SH_CFG_ENABLE_IRQ:
            ret = sh_ov7692_drv_enable_irq( &cfg_data_p );
            break;
        case SH_CFG_DISABLE_IRQ:
            ret = sh_ov7692_drv_disable_irq( &cfg_data_p );
            break;
        case SH_CFG_CAMIF_PAD_RESET:
            ret = sh_ov7692_drv_camif_pad_reg_reset( &cfg_data_p );
            break;
        case SH_CFG_CSI_REQUEST_IRQ:
            ret = sh_ov7692_CSI_request_irq();
            break;
        case SH_CFG_CSI_FREE_IRQ:
            ret = sh_ov7692_CSI_free_irq();
            break;
        case SH_CFG_REQUEST_IRQ:
            ret =sh_ov7692_request_irq();
            break;
        case SH_CFG_FREE_IRQ:
            ret = sh_ov7692_free_irq();
            break;
        case SH_CFG_EVENT_REQUEST_ISR:
            ret = sh_ov7692_event_isr( &cfg_data_p );
            break;
        case SH_CFG_FRAMEEVENT_REQUEST_ISR:
            ret = sh_ov7692_frameevent_isr( &cfg_data_p );
            break;
        default:
            ret = -EFAULT;
            break;
        }

        if (copy_to_user((void *)argp, &cfg_data_p, sizeof(struct sensor_cfg_data))) {
            CDBG( "[%s][L:%d] copy_to_user err \n", __func__, __LINE__ );
            ret = -EFAULT;
        }
    } while (0);

    return ret;
}

static int sh_ov7692_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = sh_ov7692_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	memset(s, 0, sizeof(struct msm_sensor_ctrl));

	s->s_init    = sh_ov7692_sensor_init;
	s->s_release = sh_ov7692_sensor_release;
	s->s_config  = sh_ov7692_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = 0;

	return rc;

probe_fail:
	CDBG("[%s][L:%d]: SENSOR PROBE FAILS!\n", __func__, __LINE__);
	return rc;
}

static int __sh_ov7692_probe(struct platform_device *pdev)
{

	return msm_camera_drv_start(pdev, sh_ov7692_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sh_ov7692_probe,
	.driver = {
		.name = "msm_camera_sh_ov7692",
		.owner = THIS_MODULE,
	},
};

static int __init sh_ov7692_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(sh_ov7692_init);

MODULE_DESCRIPTION("SHARP CAMERA DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.01");
