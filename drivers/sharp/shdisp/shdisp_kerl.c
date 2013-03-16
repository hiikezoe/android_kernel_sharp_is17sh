/*
 * Copyright (C) 2009 SHARP CORPORATION All rights reserved.
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

/*
 * SHARP DISPLAY DRIVER FOR KERNEL
 */

/*
 * INCLUDE FILES
 */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shlcdc_kerl.h>
#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
//#include <mach/msm_i2ctps.h>
//#include <mach/msm_i2ckbd.h>
#ifdef CONFIG_TOUCHSCREEN_SHTPS
#include <sharp/shtps_dev.h>
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
#include <asm/io.h>
#include <asm/pgtable.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#if defined(CONFIG_SHLCDC_CAMISP) && defined(CONFIG_SHLCDC_3D)
#include "../shcamsensor/sh_maincam_power_ctrl.h"
#endif /* CONFIG_SHLCDC_CAMISP && CONFIG_SHLCDC_3D */
#include <linux/proc_fs.h>
#include <linux/fb.h>

/*
 * MACROS
 */

//#define KERN_DBG_ENABLE_A
//#define KERN_DBG_ENABLE_B
//#define KERN_DBG_ENABLE_C

#ifdef CONFIG_ANDROID_ENGINEERING
#define SHDISP_DEBUG
#endif

#define SHDISP_FILE "shdisp_kerl.c"

#ifdef KERN_DBG_ENABLE_A
#define KERN_DBG_A(fmt, args...)    \
        printk(KERN_INFO "[%s][%s] " fmt, SHDISP_FILE, __func__, ## args)
#ifdef KERN_DBG_ENABLE_B
#define KERN_DBG_B(fmt, args...)    \
        printk(KERN_INFO "[%s][%s] " fmt, SHDISP_FILE, __func__, ## args)
#ifdef KERN_DBG_ENABLE_C
#define KERN_DBG_C(fmt, args...)     \
        printk(KERN_INFO "[%s][%s] " fmt, SHDISP_FILE, __func__, ## args)
#else /* !KERN_DBG_ENABLE_C */
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_C */
#else /* !KERN_DBG_ENABLE_B */
#define KERN_DBG_B(fmt, args...)
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_B */
#else /* !KERN_DBG_ENABLE_A */
#define KERN_DBG_A(fmt, args...)
#define KERN_DBG_B(fmt, args...)
#define KERN_DBG_C(fmt, args...)
#endif /* !KERN_DBG_ENABLE_A */

#define SHDISP_NAME "shdisp"

//#define SHDISP_AREA_SIZE PAGE_SIZE
#define SHDISP_AREA_SIZE 0x1000

#define SHDISP_BASE_ADDR_ES0 0x94000000
#define SHDISP_VRAM_BASE_ES0 0x94000800
#define SHDISP_BASE_ADDR_ES1 0x60000000
#define SHDISP_VRAM_BASE_ES1 0x60000800
#define SHDISP_BASE_ADDR_SBM 0x8B000000
#define SHDISP_VRAM_BASE_SBM 0x8B000800

#define SHDISP_INT_IRQ MSM_GPIO_TO_INT(26)
#define SHDISP_INT_FLAGS IRQF_TRIGGER_LOW

#define SHDISP_MASK 0x0006
#define SHDISP_MASK2 0x0008
#define SHDISP_MASK3 0x0002
#define SHDISP_MASK4 0x0256

struct shdisp_qsd_gpio {
    int num;
    int value;
};

#define SHDISP_IOC_MAGIC 's'
#define SHDISP_IOCTL_GET_CONTEXT \
        _IOR(SHDISP_IOC_MAGIC, 0, struct shdisp_boot_context)
#define SHDISP_IOCTL_SET_QSD_GPIO \
        _IOW(SHDISP_IOC_MAGIC, 1, struct shdisp_qsd_gpio)
#define SHDISP_IOCTL_SUSPEND _IOW(SHDISP_IOC_MAGIC, 2, int)
#define SHDISP_IOCTL_SET_ESD_ERR_DETECT _IOW(SHDISP_IOC_MAGIC, 3, int)
#define SHDISP_IOCTL_SET_EVENT _IO(SHDISP_IOC_MAGIC, 4)
#define SHDISP_IOCTL_SET_ESD_EVENT _IO(SHDISP_IOC_MAGIC, 5)
#define SHDISP_IOCTL_MAIN_DISP_ON _IOW(SHDISP_IOC_MAGIC, 6, int)
#define SHDISP_IOCTL_MAIN_DISP_OFF _IOW(SHDISP_IOC_MAGIC, 7, int)
#define SHDISP_IOCTL_MAIN_BKL_ON _IOW(SHDISP_IOC_MAGIC, 8, int)
#define SHDISP_IOCTL_MAIN_BKL_OFF _IOW(SHDISP_IOC_MAGIC, 9, int)
#define SHDISP_IOCTL_SET_VREG_WLAN _IOW(SHDISP_IOC_MAGIC, 10, int)
#define SHDISP_IOCTL_SET_POWER_MODE _IOW(SHDISP_IOC_MAGIC, 11, int)
#define SHDISP_IOCTL_EVENT_SUBSCRIBE \
        _IOW(SHDISP_IOC_MAGIC, 12, struct shlcdc_subscribe)
#define SHDISP_IOCTL_EVENT_UNSUBSCRIBE _IOW(SHDISP_IOC_MAGIC, 13, int)
#define SHDISP_IOCTL_IR_SET_IRSD_MODE _IOW(SHDISP_IOC_MAGIC, 14, int)
#define SHDISP_IOCTL_IR_SET_IRSEL_MODE _IOW(SHDISP_IOC_MAGIC, 15, int)
#define SHDISP_IOCTL_TP_SET_PSOC_STBY_MODE _IOW(SHDISP_IOC_MAGIC, 16, int)
#define SHDISP_IOCTL_TP_SET_PSOC_RESET_MODE _IOW(SHDISP_IOC_MAGIC, 17, int)
#define SHDISP_IOCTL_SET_POWER _IOW(SHDISP_IOC_MAGIC, 18, int)
#define SHDISP_IOCTL_SHUTDOWN _IOW(SHDISP_IOC_MAGIC, 19, int)
#define SHDISP_IOCTL_GET_HW_REVISION _IOR(SHDISP_IOC_MAGIC, 20, int)
#define SHDISP_IOCTL_SET_QSD_GPIO_CFG _IOR(SHDISP_IOC_MAGIC, 21, int)
#define SHDISP_IOCTL_CAMERA_POWER _IOR(SHDISP_IOC_MAGIC, 22, int)
#define SHDISP_IOCTL_DEBUGINFO _IOW(SHDISP_IOC_MAGIC, 23, int)
#define SHDISP_IOCTL_LAYER0_ON _IOW(SHDISP_IOC_MAGIC, 24, int)
#define SHDISP_IOCTL_GET_LAYER0_STATUS _IOR(SHDISP_IOC_MAGIC, 25, int)

/*
 * TYPES
 */

enum {
    SHDISP_DEV_PWR_STATUS_OFF,
    SHDISP_DEV_PWR_STATUS_STANDBY,
    SHDISP_DEV_PWR_STATUS_RUN,
    SHDISP_DEV_PWR_STATUS_RUNIR,
    SHDISP_DEV_PWR_STATUS_RUNSD,
    SHDISP_DEV_PWR_STATUS_RUNFULL,
    NUM_SHDISP_DEV_PWR_STATUS
};

enum {
    SHDISP_CMD_CALLER_USER,
    SHDISP_CMD_CALLER_KERNEL,
    NUM_SHDISP_CMD_CALLER
};

enum {
	SHDISP_HW_TYPE_1ES,
	SHDISP_HW_TYPE_1PP,
	NUM_SHDISP_HW_TYPE
};

struct shdisp_power {
    int dev_type;
    int dev_pwr_req;
};

struct shdisp_procfs {
    int id;
    int par[4];
};

struct shdisp_event {
    int cmd;
    struct shdisp_main_bkl_ctl bkl;
    int event_type;
    struct shdisp_power power;
    int ir_irsel_mode;
    struct shdisp_procfs shdisp_pfs;
};

/*
 * VARIABLES
 */

static dev_t shdisp_dev;
static dev_t shdisp_major = 0;
static dev_t shdisp_minor = 0;

static wait_queue_head_t shdisp_wq_for_read;
static atomic_t shdisp_atomic_for_read;
static signed long shdisp_event_counter_for_read;

static wait_queue_head_t shdisp_wq_for_ioctl;
static atomic_t shdisp_atomic_for_ioctl;
static signed long shdisp_event_counter_for_ioctl;

static int shdisp_driver_is_initialized = 0;
static int shdisp_upper_init_is_connected = 0;

static struct cdev shdisp_cdev;
static struct class *shdisp_class;

static struct shdisp_boot_context shdisp_boot_ctx;

static unsigned char *shdisp_base_addr = NULL;
static unsigned char *shdisp_vram_base = NULL;
static void *shdisp_addr = NULL;

struct shdisp_event shdisp_event_info; 

static struct vreg *shdisp_vreg_wlan = NULL;

static int shdisp_power_status = SHDISP_DEV_PWR_STATUS_RUN;
static int shdisp_irq_regist_status = 0;

static int shdisp_result;

static struct semaphore shdisp_sem;
static struct semaphore shdisp_int_sem;

static int shdisp_fbcon_status = SHDISP_FBCON_DEACTIVATED;

static void (*shdisp_int_callback_table[NUM_SHLCDC_EVENT_TYPE])(void) = {
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

static int shdisp_cmd_caller[NUM_SHLCDC_EVENT_TYPE] = {
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL,
    SHDISP_CMD_CALLER_KERNEL
};

struct hrtimer dma_hrtimer;

static int shdisp_layer0_on = 0;

static int shdisp_kerl_klog=0;
static int shdisp_kerl_klog_layer0=0;

#ifdef CONFIG_ANDROID_ENGINEERING
module_param(shdisp_kerl_klog, int, 0600);
module_param(shdisp_kerl_klog_layer0, int, 0600);
module_param(shdisp_layer0_on, int, 0400);
module_param(shdisp_driver_is_initialized, int, 0400);
module_param(shdisp_upper_init_is_connected, int, 0400);
#endif
#define pr_info_klog(fmt, arg...)	\
	if(shdisp_kerl_klog>0) pr_info(fmt, ## arg);
#define pr_info_klog_layer0(fmt, arg...)	\
	if(shdisp_kerl_klog_layer0>0) pr_info(fmt, ## arg);

/*
 * PROTOTYPES
 */

static int shdisp_open(struct inode *inode, struct file *filp);
static int shdisp_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos);
static int shdisp_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos);
static int shdisp_mmap(struct file *filp, struct vm_area_struct *vma);

//static int shdisp_ioctl(struct inode *inode, struct file *filp,
//                         unsigned int cmd, unsigned long arg);
static long shdisp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int shdisp_release(struct inode *inode, struct file *filp);
static int shdisp_ioctl_get_context(void __user *argp);
static int shdisp_ioctl_set_qsd_gpio(void __user *argp);
static int shdisp_ioctl_suspend(void __user *argp);
static int shdisp_ioctl_main_disp_on(void __user *argp);
static int shdisp_ioctl_main_disp_off(void __user *argp);
static int shdisp_ioctl_main_bkl_on(void __user *argp);
static int shdisp_ioctl_main_bkl_off(void __user *argp);
static int shdisp_ioctl_set_vreg_wlan(void __user *argp);
static int shdisp_ioctl_set_power_mode(void __user *argp);
static int shdisp_ioctl_event_subscribe(void __user *argp);
static int shdisp_ioctl_event_unsubscribe(void __user *argp);
static int shdisp_ioctl_ir_set_irsel_mode(void __user *argp);
static int shdisp_ioctl_set_power(void __user *argp);
static int shdisp_ioctl_shutdown(void __user *argp);
static int shdisp_ioctl_get_hw_revision(void __user *argp);
static int shdisp_ioctl_set_qsd_gpio_cfg(void __user *argp);
static int shdisp_ioctl_camera_power(void __user *argp);
static int shdisp_ioctl_layer0_on(void __user *argp);
static int shdisp_ioctl_get_layer0_status(void __user *argp);
static int shdisp_sqe_event_subscribe(struct shlcdc_subscribe *subscribe,
                                       int caller);
static int shdisp_sqe_event_unsubscribe(int event_type, int caller);
static void shdisp_wait_result(void);
static irqreturn_t shdisp_sqe_int_isr(int irq, void *dev_id);
static void shdisp_sqe_dummy_isr(void);
static void shdisp_io_write_reg(unsigned short reg, unsigned short val);
static void shdisp_io_read_reg(unsigned short reg, unsigned short *val);
static void shdisp_io_set_bit_reg(unsigned short reg, unsigned short val);
static void shdisp_io_clr_bit_reg(unsigned short reg, unsigned short val);
static void shdisp_io_msk_bit_reg(unsigned short reg,
                                   unsigned short val, unsigned short msk);
#ifdef SHDISP_DEBUG
static int shdisp_ioctl_debuginfo(void __user *argp);
static int shdisp_proc_write(struct file *file, const char *buffer, unsigned long count, void *data);
#endif
static void shdisp_fb_open( void );
static void shdisp_fb_close( void );

extern void msm_fb_suspend_shdisp(int sw);
extern void mdp_suspend_shdisp(int sw);
extern void mddi_suspend_shdisp(int sw);
extern void shlcdc_wake_up(int *event_type);

static struct file_operations shdisp_fops = {
    .owner   = THIS_MODULE,
    .open    = shdisp_open,
    .write   = shdisp_write,
    .read    = shdisp_read,
    .mmap    = shdisp_mmap,
//  .ioctl   = shdisp_ioctl,
    .unlocked_ioctl   = shdisp_ioctl,
    .release = shdisp_release,
};

/*
 * FUNCTIONS
 */

/*
 * shdisp_api_get_fbcon_status
 */

int shdisp_api_get_fbcon_status(void)
{
    return shdisp_fbcon_status;
}

/*
 * shdisp_api_main_disp_on
 */

int shdisp_api_main_disp_on(void)
{
    int ret;

    pr_info_klog("[====]%s:start\n", __func__);

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_upper_init_is_connected == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    down(&shdisp_sem);

    shdisp_event_info.cmd = SHDISP_IOCTL_MAIN_DISP_ON;

    atomic_inc(&shdisp_atomic_for_read);
    wake_up_interruptible(&shdisp_wq_for_read);

    shdisp_wait_result();

    ret = shdisp_result;

    up(&shdisp_sem);

#if 0
    msm_i2ckbd_setsleep(0);
    msm_i2ctps_setsleep(0);
#endif
    /* Touch panel driver are turned on. */
#ifdef CONFIG_TOUCHSCREEN_SHTPS
    msm_tps_setsleep(0);
#endif /* CONFIG_TOUCHSCREEN_SHTPS */
    pr_info_klog("[====]%s:end\n", __func__);

    return ret;
}

/*
 * shdisp_api_main_disp_off
 */

int shdisp_api_main_disp_off(void)
{
    int ret;
    pr_info_klog("[====]%s:start\n", __func__);

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

#if 0
    msm_i2ctps_setsleep(1);
    msm_i2ckbd_setsleep(1);
#endif

    /* Touch panel driver are turned off. */
#ifdef CONFIG_TOUCHSCREEN_SHTPS
    msm_tps_setsleep(1);
#endif /* CONFIG_TOUCHSCREEN_SHTPS */

    if (shdisp_upper_init_is_connected == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    down(&shdisp_sem);

    shdisp_event_info.cmd = SHDISP_IOCTL_MAIN_DISP_OFF;

    atomic_inc(&shdisp_atomic_for_read);
    wake_up_interruptible(&shdisp_wq_for_read);

    shdisp_wait_result();

    ret = shdisp_result;

    up(&shdisp_sem);

    pr_info_klog("[====]%s:end\n", __func__);

    return ret;
}

/*
 * shdisp_api_main_bkl_on
 */

int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl)
{
    int ret;

    pr_info_klog("[====]%s:\n", __func__);

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_upper_init_is_connected == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (bkl == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

    if ((bkl->mode == SHDISP_MAIN_BKL_MODE_OFF) ||
        (bkl->mode >= SHDISP_MAIN_BKL_MODE_DTV_OFF)) {
        return SHDISP_RESULT_FAILURE;
    }

    if (bkl->mode == SHDISP_MAIN_BKL_MODE_FIX) {
        if (bkl->param < SHDISP_MAIN_BKL_PARAM_1) {
            return SHDISP_RESULT_FAILURE;
        }
        if (bkl->param > SHDISP_MAIN_BKL_PARAM_22) {
            bkl->param = SHDISP_MAIN_BKL_PARAM_22;
        }
    }

    if (bkl->mode == SHDISP_MAIN_BKL_MODE_AUTO) {
        if (bkl->param != SHDISP_MAIN_BKL_PARAM_1) {
            return SHDISP_RESULT_FAILURE;
        }
    }

    down(&shdisp_sem);

    shdisp_event_info.cmd = SHDISP_IOCTL_MAIN_BKL_ON;
    shdisp_event_info.bkl.mode = bkl->mode;
    shdisp_event_info.bkl.param = bkl->param;

    atomic_inc(&shdisp_atomic_for_read);
    wake_up_interruptible(&shdisp_wq_for_read);

    shdisp_wait_result();

    ret = shdisp_result;

    up(&shdisp_sem);

    return ret;
}

/*
 * shdisp_api_main_bkl_off
 */

int shdisp_api_main_bkl_off(void)
{
    int ret;

    pr_info_klog("[====]%s:\n", __func__);

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_upper_init_is_connected == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    down(&shdisp_sem);

    shdisp_event_info.cmd = SHDISP_IOCTL_MAIN_BKL_OFF;

    atomic_inc(&shdisp_atomic_for_read);
    wake_up_interruptible(&shdisp_wq_for_read);

    shdisp_wait_result();

    ret = shdisp_result;

    up(&shdisp_sem);

    return ret;
}

/*
 * shdisp_api_shutdown
 */

int shdisp_api_shutdown(void)
{
    int ret;

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (shdisp_upper_init_is_connected == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    down(&shdisp_sem);

    shdisp_event_info.cmd = SHDISP_IOCTL_SHUTDOWN;

    atomic_inc(&shdisp_atomic_for_read);
    wake_up_interruptible(&shdisp_wq_for_read);

    shdisp_wait_result();

    ret = shdisp_result;

    up(&shdisp_sem);

    return ret;
}

/*
 * shdisp_api_layer0_on
 */

int shdisp_api_layer0_on(void)
{
    int ret = 0; 
    unsigned short data;

    pr_info_klog_layer0("[====]%s:\n", __func__);

    if(0 != shdisp_layer0_on)
    {
		pr_info_klog_layer0("[====]LayerON%s:\n", __func__);
        down(&shdisp_sem);
        shdisp_io_read_reg(0x0100, &data);
        data |= 0x0010;
        shdisp_io_write_reg(0x0100, data);
        shdisp_io_write_reg(0x01C0, 0x0001);
        shdisp_layer0_on = 0;

        up(&shdisp_sem);
    }

    return ret;
}

/*
 * shlcdc_api_set_power_mode
 */

int shlcdc_api_set_power_mode(int dev_type, int dev_pwr_req)
{
    int ret;

    pr_info_klog("[====]%s:type:%d, req:%d\n", __func__, dev_type, dev_pwr_req);

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if ((dev_type < 0) || (dev_type >= NUM_SHLCDC_DEV_TYPE)) {
        return SHDISP_RESULT_FAILURE;
    }

    if ((dev_pwr_req < 0) || (dev_pwr_req >= NUM_SHLCDC_DEV_PWR)) {
        return SHDISP_RESULT_FAILURE;
    }

    down(&shdisp_sem);

    shdisp_event_info.cmd = SHDISP_IOCTL_SET_POWER_MODE;
    shdisp_event_info.power.dev_type = dev_type;
    shdisp_event_info.power.dev_pwr_req = dev_pwr_req;

    atomic_inc(&shdisp_atomic_for_read);
    wake_up_interruptible(&shdisp_wq_for_read);

    shdisp_wait_result();

    ret = shdisp_result;

    up(&shdisp_sem);

    return ret;
}

/*
 * shlcdc_api_ir_set_irsd_mode
 */

int shlcdc_api_ir_set_irsd_mode(int irsd_mode)
{
    return SHDISP_RESULT_SUCCESS;
}

/*
 * shlcdc_api_ir_set_irsel_mode
 */

int shlcdc_api_ir_set_irsel_mode(int irsel_mode)
{

    int ret;

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if ((irsel_mode < 0) || (irsel_mode >= NUM_SHLCDC_IR_IRSD_MODE)) {
        return SHDISP_RESULT_FAILURE;
    }

    down(&shdisp_sem);

    shdisp_event_info.cmd = SHDISP_IOCTL_IR_SET_IRSEL_MODE;
    shdisp_event_info.ir_irsel_mode = irsel_mode;

    atomic_inc(&shdisp_atomic_for_read);
    wake_up_interruptible(&shdisp_wq_for_read);

    shdisp_wait_result();

    ret = shdisp_result;

    up(&shdisp_sem);

    return ret;
}

/*
 * shlcdc_api_tp_set_psoc_stby_mode
 */

int shlcdc_api_tp_set_psoc_stby_mode(int stby_mode)
{
    return SHDISP_RESULT_SUCCESS;
}

/*
 * shlcdc_api_tp_set_psoc_reset_mode
 */

int shlcdc_api_tp_set_psoc_reset_mode(int reset_mode)
{
    return SHDISP_RESULT_SUCCESS;
}

/*
 * shlcdc_api_event_subscribe
 */

int shlcdc_api_event_subscribe(struct shlcdc_subscribe *subscribe)
{
    int ret;

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if (subscribe == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

    if ((subscribe->event_type < 0) || (subscribe->event_type >= NUM_SHLCDC_EVENT_TYPE)) {
        return SHDISP_RESULT_FAILURE;
    }

    if (subscribe->callback == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

    down(&shdisp_sem);

    ret = shdisp_sqe_event_subscribe(subscribe, SHDISP_CMD_CALLER_KERNEL);

    up(&shdisp_sem);

    return ret;
}

/*
 * shlcdc_api_event_unsubscribe
 */

int shlcdc_api_event_unsubscribe(int event_type)
{
    int ret;

    if (shdisp_driver_is_initialized == 0) {
        return SHDISP_RESULT_SUCCESS;
    }

    if ((event_type < 0) || (event_type >= NUM_SHLCDC_EVENT_TYPE)) {
        return SHDISP_RESULT_FAILURE;
    }

    down(&shdisp_sem);

    ret = shdisp_sqe_event_unsubscribe(event_type, SHDISP_CMD_CALLER_KERNEL);

    up(&shdisp_sem);

    return ret;
}

enum hrtimer_restart mdp4_dtv_pvsync_hrtimer(struct hrtimer *ht){
    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);
    
	return HRTIMER_NORESTART;
}


static void mdp4_dtv_pvsync_isr(void)
{
    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);
}

void shdisp_api_wait_vsync( void )
{
	struct shlcdc_subscribe subscribe;
	unsigned short data;

	subscribe.event_type = SHLCDC_EVENT_TYPE_PVSYNC;
	subscribe.callback = mdp4_dtv_pvsync_isr;

    shdisp_io_read_reg(0x0100, &data);

	if( (data & 0x0004) == 0 ) return;

	down(&shdisp_sem);

	shdisp_io_write_reg(SHDISP_MASK3, 0xFFFE);
	shdisp_io_write_reg(SHDISP_MASK4, 0x2001);
	
	shdisp_sqe_event_subscribe( &subscribe, SHDISP_CMD_CALLER_KERNEL );
	{
		ktime_t wait_time;
		
		wait_time.tv64 = (s64)(100 * 1000 * 1000);
		dma_hrtimer.function = mdp4_dtv_pvsync_hrtimer;
		hrtimer_start(&dma_hrtimer, wait_time, HRTIMER_MODE_REL);
	}
    shdisp_wait_result();
    
    shdisp_sqe_event_unsubscribe( subscribe.event_type, SHDISP_CMD_CALLER_KERNEL );
    
    hrtimer_cancel(&dma_hrtimer);
    
	up(&shdisp_sem);
}

/*
 * shdisp_open
 */

static int shdisp_open(struct inode *inode, struct file *filp)
{
    if(shdisp_driver_is_initialized >= INT_MAX) {
        return -1;
    }

    if (shdisp_driver_is_initialized == 0) {
        shdisp_event_counter_for_read = atomic_read(&shdisp_atomic_for_read);
        shdisp_event_counter_for_ioctl = atomic_read(&shdisp_atomic_for_ioctl);
    }

    shdisp_driver_is_initialized++;

    return 0;
}

/*
 * shdisp_write
 */

static int shdisp_write(struct file *filp, const char __user *buf,
                         size_t count, loff_t *ppos)
{
    return 0;
}

/*
 * shdisp_read
 */

static int shdisp_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos)
{
    DECLARE_WAITQUEUE(wait, current);
    int ret = 0;
    int event_count;

    if (count != sizeof(shdisp_event_info)) {
        return -EPERM;
    }

    add_wait_queue(&shdisp_wq_for_read, &wait);

    do {
        set_current_state(TASK_INTERRUPTIBLE);

        event_count = atomic_read(&shdisp_atomic_for_read);

        if (event_count != shdisp_event_counter_for_read) {
            if (copy_to_user(buf, &shdisp_event_info, count) != 0) {
                ret = -EFAULT;
            } else {
                shdisp_event_counter_for_read = event_count;
                ret = count;
            }
            break;
        }

        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            break;
        }

        if (signal_pending(current)) {
            ret = -ERESTARTSYS;
            break;
        }

        schedule();
    } while (1);

    __set_current_state(TASK_RUNNING);
    remove_wait_queue(&shdisp_wq_for_read, &wait);

    return ret;
}

/*
 * shdisp_mmap
 */

static int shdisp_mmap(struct file *filp, struct vm_area_struct *vma)
{
    int ret;

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    ret = io_remap_pfn_range(vma, vma->vm_start,
                       (unsigned long)shdisp_base_addr >> PAGE_SHIFT,
                       vma->vm_end - vma->vm_start, vma->vm_page_prot);

    return ret;
}

/*
 * shdisp_ioctl
 */

//static int shdisp_ioctl(struct inode *inode, struct file *filp,
//                         unsigned int cmd, unsigned long arg)
static long shdisp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret;
    void __user *argp = (void __user*)arg;

    switch (cmd) {
    case SHDISP_IOCTL_GET_CONTEXT:
        ret = shdisp_ioctl_get_context(argp);
        break;
    case SHDISP_IOCTL_SET_QSD_GPIO:
        ret = shdisp_ioctl_set_qsd_gpio(argp);
        break;
    case SHDISP_IOCTL_SUSPEND:
        ret = shdisp_ioctl_suspend(argp);
        break;
    case SHDISP_IOCTL_MAIN_DISP_ON:
        ret = shdisp_ioctl_main_disp_on(argp);
        break;
    case SHDISP_IOCTL_MAIN_DISP_OFF:
        ret = shdisp_ioctl_main_disp_off(argp);
        break;
    case SHDISP_IOCTL_MAIN_BKL_ON:
        ret = shdisp_ioctl_main_bkl_on(argp);
        break;
    case SHDISP_IOCTL_MAIN_BKL_OFF:
        ret = shdisp_ioctl_main_bkl_off(argp);
        break;
    case SHDISP_IOCTL_SET_VREG_WLAN:
        ret = shdisp_ioctl_set_vreg_wlan(argp);
        break;
    case SHDISP_IOCTL_SET_POWER_MODE:
        ret = shdisp_ioctl_set_power_mode(argp);
        break;
    case SHDISP_IOCTL_EVENT_SUBSCRIBE:
        ret = shdisp_ioctl_event_subscribe(argp);
        break;
    case SHDISP_IOCTL_EVENT_UNSUBSCRIBE:
        ret = shdisp_ioctl_event_unsubscribe(argp);
        break;
    case SHDISP_IOCTL_IR_SET_IRSEL_MODE:
        ret = shdisp_ioctl_ir_set_irsel_mode(argp);
        break;
    case SHDISP_IOCTL_SET_POWER:
        ret = shdisp_ioctl_set_power(argp);
        break;
    case SHDISP_IOCTL_SHUTDOWN:
        ret = shdisp_ioctl_shutdown(argp);
        break;
    case SHDISP_IOCTL_GET_HW_REVISION:
    	ret = shdisp_ioctl_get_hw_revision(argp);
    	break;
    case SHDISP_IOCTL_SET_QSD_GPIO_CFG:
        ret = shdisp_ioctl_set_qsd_gpio_cfg(argp);
        break;
	case SHDISP_IOCTL_CAMERA_POWER:
		ret = shdisp_ioctl_camera_power(argp);
		break;
#ifdef SHDISP_DEBUG
    case SHDISP_IOCTL_DEBUGINFO:
        ret = shdisp_ioctl_debuginfo(argp);
        break;
#endif
    case SHDISP_IOCTL_LAYER0_ON:
        ret = shdisp_ioctl_layer0_on(argp);
        break;
    case SHDISP_IOCTL_GET_LAYER0_STATUS:
        ret = shdisp_ioctl_get_layer0_status(argp);
        break;
    default:
        ret = -EFAULT;
        break;
    }

    return ret;
}

/*
 * shdisp_release
 */

static int shdisp_release(struct inode *inode, struct file *filp)
{
    if (shdisp_driver_is_initialized > 0) {
        shdisp_driver_is_initialized--;

        if (shdisp_driver_is_initialized == 0) {
            atomic_set(&shdisp_atomic_for_read, 0);
            atomic_set(&shdisp_atomic_for_ioctl, 0);
        }
    }

    return 0;
}

/*
 * shdisp_ioctl_get_context
 */

static int shdisp_ioctl_get_context(void __user *argp)
{
    int ret;

    ret = copy_to_user(argp, &shdisp_boot_ctx,
                             sizeof(struct shdisp_boot_context));

    if (ret != 0) {
        return ret;
    }
    return 0;
}

/*
 * shdisp_ioctl_set_qsd_gpio
 */

static int shdisp_ioctl_set_qsd_gpio(void __user *argp)
{
    int ret;
    struct shdisp_qsd_gpio gpio;

    ret = copy_from_user(&gpio, argp, sizeof(struct shdisp_qsd_gpio));

    if (ret != 0) {
        return ret;
    }

    if(gpio.num < 0) {
        return -1;
    }

    gpio_set_value(gpio.num, gpio.value);

    return 0;
}

/*
 * shdisp_ioctl_suspend
 */

static int shdisp_ioctl_suspend(void __user *argp)
{
    int ret, sw;

    ret = copy_from_user(&sw, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    if (sw == 0) {
        mddi_suspend_shdisp(sw);
        mdp_suspend_shdisp(sw);
        msm_fb_suspend_shdisp(sw);
    } else {
        msm_fb_suspend_shdisp(sw);
        mdp_suspend_shdisp(sw);
        mddi_suspend_shdisp(sw);
    }

    return 0;
}

/*
 * shdisp_ioctl_main_disp_on
 */

static int shdisp_ioctl_main_disp_on(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    shdisp_layer0_on = 1;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}

/*
 * shdisp_ioctl_main_disp_off
 */

static int shdisp_ioctl_main_disp_off(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}

/*
 * shdisp_ioctl_main_bkl_on
 */

static int shdisp_ioctl_main_bkl_on(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}

/*
 * shdisp_ioctl_main_bkl_off
 */

static int shdisp_ioctl_main_bkl_off(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}

/*
 * shdisp_ioctl_set_vreg_wlan
 */

static int shdisp_ioctl_set_vreg_wlan(void __user *argp)
{
    int ret, sw;

    ret = copy_from_user(&sw, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    if (sw == 0) {
        vreg_disable(shdisp_vreg_wlan);
    } else {
        vreg_set_level(shdisp_vreg_wlan, 2850);
        vreg_enable(shdisp_vreg_wlan);
    }

    return 0;
}

/*
 * shdisp_ioctl_set_power_mode
 */

static int shdisp_ioctl_set_power_mode(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}

/*
 * shdisp_ioctl_shutdown
 */

static int shdisp_ioctl_shutdown(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}

/*
 * shdisp_ioctl_get_hw_revision
 */

static int shdisp_ioctl_get_hw_revision(void __user *argp)
{
    int ret;
    unsigned short hw_check;

	hw_check = sh_boot_get_hw_revision();

    ret = copy_to_user(argp, &hw_check, sizeof(unsigned short));

    if (ret != 0) {
        return ret;
    }
    return 0;
}

/*
 * shdisp_ioctl_set_qsd_gpio_cfg
 */

static int shdisp_ioctl_set_qsd_gpio_cfg(void __user *argp)
{
    int ret;
    int gpio;

    ret = copy_from_user(&gpio, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    if(gpio < 0) {
        return -1;
    }

    gpio_tlmm_config(gpio, GPIO_CFG_ENABLE);

    return 0;
}

/*
 * shdisp_ioctl_event_subscribe
 */

static int shdisp_ioctl_event_subscribe(void __user *argp)
{
    int ret;
    struct shlcdc_subscribe subscribe;

    ret = copy_from_user(&subscribe, argp,
                                     sizeof(struct shlcdc_subscribe));

    if (ret != 0) {
        return ret;
    }

    if ((subscribe.event_type < 0) || (subscribe.event_type >= NUM_SHLCDC_EVENT_TYPE)) {
        return SHDISP_RESULT_FAILURE;
    }

    return shdisp_sqe_event_subscribe(&subscribe, SHDISP_CMD_CALLER_USER);
}

/*
 * shdisp_ioctl_event_unsubscribe
 */

static int shdisp_ioctl_event_unsubscribe(void __user *argp)
{
    int ret, event_type;

    ret = copy_from_user(&event_type, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    if ((event_type < 0) || (event_type >= NUM_SHLCDC_EVENT_TYPE)) {
        return SHDISP_RESULT_FAILURE;
    }

    return shdisp_sqe_event_unsubscribe(event_type, SHDISP_CMD_CALLER_USER);
}

/*
 * shdisp_ioctl_ir_set_irsel_mode
 */

static int shdisp_ioctl_ir_set_irsel_mode(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}

/*
 * shdisp_ioctl_set_power
 */

static int shdisp_ioctl_set_power(void __user *argp)
{
	static unsigned char irq_wake_flag = 0;
    int ret, power;

    ret = copy_from_user(&power, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    if((power < 0) || (power >= NUM_SHDISP_DEV_PWR_STATUS)) {
        return -1;
    }

    shdisp_power_status = power;

	if(irq_wake_flag == 0)
	{
		if(shdisp_power_status == SHDISP_DEV_PWR_STATUS_STANDBY)
		{
//			set_irq_type(SHDISP_INT_IRQ, IRQF_TRIGGER_LOW );
			irq_set_irq_type(SHDISP_INT_IRQ, IRQF_TRIGGER_LOW );
			enable_irq_wake(SHDISP_INT_IRQ);

			irq_wake_flag = 1;
		}
		else
		{}
	}
	else
	{
		if(shdisp_power_status != SHDISP_DEV_PWR_STATUS_STANDBY)
		{
			disable_irq_wake(SHDISP_INT_IRQ);

			irq_wake_flag = 0;
		}
		else
		{}
	}

    return 0;
}

/*
 * shdisp_ioctl_camera_power
 */

static int shdisp_ioctl_camera_power(void __user *argp)
{
#if defined(CONFIG_SHLCDC_CAMISP) && defined(CONFIG_SHLCDC_3D)
	int ret, OnOff;
	
	ret = copy_from_user(&OnOff, argp, sizeof(int));
	
	if (ret != 0) {
        return ret;
    }
    
    if (OnOff == 1) {
		ret = sh_maincam_drv_power_ctrl(SH_MAINCAMDRV_POWER_ON, SH_MAINCAMDRV_USE_SASUKE_DRV);
	} else {
		ret = sh_maincam_drv_power_ctrl(SH_MAINCAMDRV_POWER_OFF, SH_MAINCAMDRV_USE_SASUKE_DRV);
    }
#endif /* CONFIG_SHLCDC_CAMISP && CONFIG_SHLCDC_3D */
    return 0;
}

/*
 * shdisp_ioctl_layer0_on
 */

static int shdisp_ioctl_layer0_on(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}

/*
 * shdisp_ioctl_get_layer0_status
 */

static int shdisp_ioctl_get_layer0_status(void __user *argp)
{
    int ret;

    ret = copy_to_user(argp, &shdisp_layer0_on, sizeof(int));

    if (ret != 0) {
        return ret;
    }
    return 0;
}

/*
 * shdisp_sqe_event_subscribe
 */

static int shdisp_sqe_event_subscribe(struct shlcdc_subscribe *subscribe,
                                       int caller)
{
    int idx, regist_status = 0;

    if (subscribe == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

    if (subscribe->event_type < 0 || subscribe->event_type >= NUM_SHLCDC_EVENT_TYPE) {
        return SHDISP_RESULT_FAILURE;
    }

    if (caller < 0 || caller >= NUM_SHDISP_CMD_CALLER) {
        return SHDISP_RESULT_FAILURE;
    }

    down(&shdisp_int_sem);

    for (idx = 0; idx < NUM_SHLCDC_EVENT_TYPE; idx++) {
        if (shdisp_int_callback_table[idx] != NULL) {
            regist_status = 1;
        }
    }

    shdisp_cmd_caller[subscribe->event_type] = caller;

    if (caller == SHDISP_CMD_CALLER_KERNEL) {
        shdisp_int_callback_table[subscribe->event_type] =
                                                        subscribe->callback;
    } else {
        shdisp_int_callback_table[subscribe->event_type] =
                                                        shdisp_sqe_dummy_isr;
    }

    if (subscribe->event_type == SHLCDC_EVENT_TYPE_IRRC) {
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x0004);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_SDHC) {
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x0100);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_GPIO) {
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x0002);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_PVSYNC) {
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x0001);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_I2C) {
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x0020);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_CAMVIEW) {
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x0800);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_SDDET_H) {
        shdisp_io_msk_bit_reg(0x0488, 0x0004, 0xFFF3);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_SDDET_L) {
        shdisp_io_msk_bit_reg(0x0488, 0x0008, 0xFFF3);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_CSTM) {
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x0080);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_CSI) {
        shdisp_io_set_bit_reg(SHDISP_MASK2, 0x0008);
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x1000);
    } else if (subscribe->event_type == SHLCDC_EVENT_TYPE_IRRC2) {
        shdisp_io_set_bit_reg(SHDISP_MASK, 0x0008);
    }

    if (regist_status == 0) {
        if (shdisp_irq_regist_status == 0) {
            enable_irq(SHDISP_INT_IRQ);
            shdisp_irq_regist_status = 1;
        }
    }

    up(&shdisp_int_sem);

    return SHDISP_RESULT_SUCCESS;
}

/*
 * shdisp_sqe_event_unsubscribe
 */

static int shdisp_sqe_event_unsubscribe(int event_type, int caller)
{
    int idx, regist_status = 0;

    if (event_type < 0 || event_type >= NUM_SHLCDC_EVENT_TYPE) {
        return SHDISP_RESULT_FAILURE;
    }

    if (caller < 0 || caller >= NUM_SHDISP_CMD_CALLER) {
        return SHDISP_RESULT_FAILURE;
    }

    down(&shdisp_int_sem);

    shdisp_cmd_caller[event_type] = caller;

    if (event_type == SHLCDC_EVENT_TYPE_IRRC) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0004);
    } else if (event_type == SHLCDC_EVENT_TYPE_SDHC) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0100);
    } else if (event_type == SHLCDC_EVENT_TYPE_GPIO) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0002);
    } else if (event_type == SHLCDC_EVENT_TYPE_PVSYNC) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0001);
    } else if (event_type == SHLCDC_EVENT_TYPE_I2C) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0020);
    } else if (event_type == SHLCDC_EVENT_TYPE_CAMVIEW) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0800);
    } else if (event_type == SHLCDC_EVENT_TYPE_SDDET_H) {
        shdisp_io_clr_bit_reg(0x0488, 0x0004);
    } else if (event_type == SHLCDC_EVENT_TYPE_SDDET_L) {
        shdisp_io_clr_bit_reg(0x0488, 0x0008);
    } else if (event_type == SHLCDC_EVENT_TYPE_CSTM) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0080);
    } else if (event_type == SHLCDC_EVENT_TYPE_CSI) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x1000);
        shdisp_io_clr_bit_reg(SHDISP_MASK2, 0x0008);
    } else if (event_type == SHLCDC_EVENT_TYPE_IRRC2) {
        shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0008);
    }

    shdisp_int_callback_table[event_type] = NULL;

    for (idx = 0; idx < NUM_SHLCDC_EVENT_TYPE; idx++) {
        if (shdisp_int_callback_table[idx] != NULL) {
            regist_status = 1;
        }
    }

    if (regist_status == 0) {
        if (shdisp_irq_regist_status == 1) {
            disable_irq(SHDISP_INT_IRQ);
            shdisp_irq_regist_status = 0;
        }
    }

    up(&shdisp_int_sem);

    return SHDISP_RESULT_SUCCESS;
}

/*
 * shdisp_wait_result
 */

static void shdisp_wait_result(void)
{
    DECLARE_WAITQUEUE(wait, current);
    int event_count;

    add_wait_queue(&shdisp_wq_for_ioctl, &wait);

    do {
        set_current_state(TASK_INTERRUPTIBLE);

        event_count = atomic_read(&shdisp_atomic_for_ioctl);

        if (event_count != shdisp_event_counter_for_ioctl) {
            shdisp_event_counter_for_ioctl = event_count;
            break;
        }

        schedule();
    } while (1);

    __set_current_state(TASK_RUNNING);
    remove_wait_queue(&shdisp_wq_for_ioctl, &wait);

    return;
}

/*
 * shdisp_sqe_int_isr
 */

static irqreturn_t shdisp_sqe_int_isr(int irq, void *dev_id)
{
    unsigned short int_status;
    unsigned short sddet_status;
    int event_type_for_user[NUM_SHLCDC_EVENT_TYPE];
    int is_wake_up = 0;

    memset(&event_type_for_user[0], 0, sizeof(event_type_for_user));

    if (shdisp_driver_is_initialized == 0) {
        return IRQ_HANDLED;
    }

    if (shdisp_power_status == SHDISP_DEV_PWR_STATUS_STANDBY) {
        disable_irq_nosync(SHDISP_INT_IRQ);
        if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_H] != NULL) {
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_SDDET_H] !=
                                                    SHDISP_CMD_CALLER_USER) {
                (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_H])();
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_SDDET_H] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_H] = NULL;
        }

        if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_L] != NULL) {
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_SDDET_L] !=
                                                    SHDISP_CMD_CALLER_USER) {
                (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_L])();
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_SDDET_L] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_L] = NULL;
        }
        shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_H] = NULL;
        shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_L] = NULL;
        shdisp_irq_regist_status = 0;
    } else {
        shdisp_io_read_reg(0x0002, &int_status);
        shdisp_io_read_reg(0x0488, &sddet_status);

        if (int_status & 0x0004) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0004);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_IRRC] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_IRRC]
                                                                   != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_IRRC])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_IRRC] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_IRRC] = NULL;
        }

        if (int_status & 0x0100) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0100);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_SDHC] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDHC]
                                                                   != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDHC])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_SDHC] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDHC] = NULL;
        }

        if (int_status & 0x0001) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0001);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_PVSYNC] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_PVSYNC]
                                                                   != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_PVSYNC])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_PVSYNC] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_PVSYNC] = NULL;
        }

        if (int_status & 0x0080) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0080);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_CSTM] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CSTM]
                                                                   != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CSTM])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_CSTM] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CSTM] = NULL;
        }

        if (int_status & 0x0002) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0002);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_GPIO] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_GPIO]
                                                                   != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_GPIO])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_GPIO] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_GPIO] = NULL;
        }

        if (int_status & 0x0020) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0020);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_I2C] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_I2C]
                                                                  != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_I2C])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_I2C] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_I2C] = NULL;
        }

        if (int_status & 0x0800) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0800);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_CAMVIEW] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CAMVIEW]
                                                                   != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CAMVIEW])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_CAMVIEW] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CAMVIEW] = NULL;
        }

        if (int_status & 0x1000) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x1000);
            shdisp_io_clr_bit_reg(SHDISP_MASK2, 0x0008);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_CSI] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CSI]
                                                                   != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CSI])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_CSI] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_CSI] = NULL;
        }

        if (int_status & 0x0008) {
            shdisp_io_clr_bit_reg(SHDISP_MASK, 0x0008);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_IRRC2] !=
                                                    SHDISP_CMD_CALLER_USER) {
                if (shdisp_int_callback_table[SHLCDC_EVENT_TYPE_IRRC2]
                                                                   != NULL) {
                    (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_IRRC2])();
                }
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_IRRC2] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_IRRC2] = NULL;
        }

        if ((shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_H] != NULL) &&
            ((sddet_status & 0x0002) != 0)) {
            shdisp_io_clr_bit_reg(0x0488, 0x000C);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_SDDET_H] !=
                                                     SHDISP_CMD_CALLER_USER) {
                (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_H])();
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_SDDET_H] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_H] = NULL;
        }

        if ((shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_L] != NULL) &&
            ((sddet_status & 0x0002) == 0)) {
            shdisp_io_clr_bit_reg(0x0488, 0x000C);
            if (shdisp_cmd_caller[SHLCDC_EVENT_TYPE_SDDET_L] !=
                                                     SHDISP_CMD_CALLER_USER) {
                (*shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_L])();
            } else {
                is_wake_up = 1;
                event_type_for_user[SHLCDC_EVENT_TYPE_SDDET_L] = 1;
            }
            shdisp_int_callback_table[SHLCDC_EVENT_TYPE_SDDET_L] = NULL;
        }
    }

    if (is_wake_up == 1) {
        shlcdc_wake_up(&event_type_for_user[0]);
    }

    return IRQ_HANDLED;
}

/*
 * shdisp_sqe_dummy_isr
 */

static void shdisp_sqe_dummy_isr(void)
{
    return;
}

/*
 * shdisp_io_write_reg
 */

static void shdisp_io_write_reg(unsigned short reg, unsigned short val)
{
    *((volatile unsigned short*)(shdisp_addr + reg)) = val;

    return;
}

/*
 * shdisp_io_read_reg
 */

static void shdisp_io_read_reg(unsigned short reg, unsigned short *val)
{
    *val = *((volatile unsigned short*)(shdisp_addr + reg));

    return;
}

/*
 * shdisp_io_set_bit_reg
 */

static void shdisp_io_set_bit_reg(unsigned short reg, unsigned short val)
{
    unsigned short set_bit = 0;

    shdisp_io_read_reg(reg, &set_bit);
    set_bit |= val;
    shdisp_io_write_reg(reg, set_bit);

    return;
}

/*
 * shdisp_io_clr_bit_reg
 */

static void shdisp_io_clr_bit_reg(unsigned short reg, unsigned short val)
{
    unsigned short clr_bit = 0;

    shdisp_io_read_reg(reg, &clr_bit);
    clr_bit &= (unsigned short)~val;
    shdisp_io_write_reg(reg, clr_bit);

    return;
}

/*
 * shdisp_io_msk_bit_reg
 */

static void shdisp_io_msk_bit_reg(unsigned short reg,
                                   unsigned short val, unsigned short msk)
{
    unsigned short src_bit = 0;
    unsigned short dst_bit = 0;

    shdisp_io_read_reg(reg, &src_bit);
    dst_bit = (src_bit & msk) | val;
    shdisp_io_write_reg(reg, dst_bit);

    return;
}

#ifdef SHDISP_DEBUG
/*
 * shdisp_proc_write
 */

static int shdisp_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
#define SHDISP_LEN_ID    (2)
#define SHDISP_LEN_PARAM (4)
#define SHDISP_PARAM_MAX (4)

    unsigned long len = count;
    struct shdisp_procfs shdisp_pfs;
    char buf[SHDISP_LEN_PARAM + 1];
    char kbuf[SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM];
    int i;

    if(count == 0) {
        return 0;
    }

    down(&shdisp_sem);
    
    len--;
    if (len < SHDISP_LEN_ID) return count;
    
    if (len > (SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM)){
       len = SHDISP_LEN_ID + SHDISP_PARAM_MAX * SHDISP_LEN_PARAM;
    }

    if (copy_from_user(kbuf, buffer, len)) return -EFAULT;

    memcpy(buf, kbuf, SHDISP_LEN_ID);
    buf[SHDISP_LEN_ID] = '\0';
    shdisp_pfs.id = simple_strtol(buf, NULL, 10);
    shdisp_pfs.par[0] = 0;
    shdisp_pfs.par[1] = 0;
    shdisp_pfs.par[2] = 0;
    shdisp_pfs.par[3] = 0;
    
    for(i = 0; (i + 1) * SHDISP_LEN_PARAM <= (len - SHDISP_LEN_ID); i++){
        memcpy(buf, &(kbuf[SHDISP_LEN_ID + i * SHDISP_LEN_PARAM]), SHDISP_LEN_PARAM);
        buf[SHDISP_LEN_PARAM] = '\0';
        shdisp_pfs.par[i] = simple_strtol(buf, NULL, 16);
    }

    shdisp_event_info.cmd = SHDISP_IOCTL_DEBUGINFO;
    memcpy(&(shdisp_event_info.shdisp_pfs), &shdisp_pfs, sizeof(shdisp_pfs));

    atomic_inc(&shdisp_atomic_for_read);
    wake_up_interruptible(&shdisp_wq_for_read);

    shdisp_wait_result();

    up(&shdisp_sem);

    return count;
}

/*
 *  shdisp_ioctl_debuginfo
 */

static int shdisp_ioctl_debuginfo(void __user *argp)
{
    int ret, result;

    ret = copy_from_user(&result, argp, sizeof(int));

    if (ret != 0) {
        return ret;
    }

    shdisp_result = result;

    atomic_inc(&shdisp_atomic_for_ioctl);
    wake_up_interruptible(&shdisp_wq_for_ioctl);

    return 0;
}
#endif

/*
 *  shdisp_fb_open
 */
static void shdisp_fb_open( void )
{
    struct fb_info *info = NULL;

    if (!num_registered_fb){
        return;
    }
    info = registered_fb[0];
    if (!info){
        return;
    }
    if (!try_module_get(info->fbops->owner)){
        return;
    }
    if (info->fbops->fb_open && info->fbops->fb_open(info, 0)) {
        module_put(info->fbops->owner);
        return;
    }
    return;
}

/*
 *  shdisp_fb_close
 */

static void shdisp_fb_close( void )
{
    struct fb_info *info = NULL;

    info = registered_fb[0];
    if (!info){
        return;
    }
    if (info->fbops->fb_release){
        info->fbops->fb_release(info, 0);
    }
    module_put(info->fbops->owner);
    return;
}

/*
 *  shdisp_init
 */

static int __init shdisp_init(void)
{
    int ret;

    sharp_smem_common_type *sh_smem_common;

#ifdef SHDISP_DEBUG
    struct proc_dir_entry *entry;
#endif

    sh_smem_common = sh_smem_get_common_address();

    if(sh_smem_common == NULL) {
        shdisp_base_addr = (unsigned char*)SHDISP_BASE_ADDR_SBM;
        shdisp_vram_base = (unsigned char*)SHDISP_VRAM_BASE_SBM;
        return -1;
    }

    memcpy(&shdisp_boot_ctx,
           &(sh_smem_common->shdisp_boot_ctx),
           sizeof(struct shdisp_boot_context));

    shdisp_upper_init_is_connected = shdisp_boot_ctx.upper_unit_is_connected;
#if 0
    if ((shdisp_boot_ctx.hw_revision & 0x000F) == 0x0000) {
        shdisp_base_addr = (unsigned char*)SHDISP_BASE_ADDR_ES0;
        shdisp_vram_base = (unsigned char*)SHDISP_VRAM_BASE_ES0;
    } else {
        shdisp_base_addr = (unsigned char*)SHDISP_BASE_ADDR_ES1;
        shdisp_vram_base = (unsigned char*)SHDISP_VRAM_BASE_ES1;
    }
#endif

    shdisp_base_addr = (unsigned char*)SHDISP_BASE_ADDR_SBM;
    shdisp_vram_base = (unsigned char*)SHDISP_VRAM_BASE_SBM;

    gpio_tlmm_config(GPIO_CFG(23,	0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL  , GPIO_CFG_6MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(93,	0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL  , GPIO_CFG_6MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(26,	0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL  , GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(19,	0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL  , GPIO_CFG_6MA), GPIO_CFG_ENABLE);

    ret = alloc_chrdev_region(&shdisp_dev, 0, 1, SHDISP_NAME);

    if (!ret) {
        shdisp_major = MAJOR(shdisp_dev);
        shdisp_minor = MINOR(shdisp_dev);
    } else {
        goto shdisp_err_1;
    }

    cdev_init(&shdisp_cdev, &shdisp_fops);

    shdisp_cdev.owner = THIS_MODULE;
    shdisp_cdev.ops = &shdisp_fops;

    ret = cdev_add(&shdisp_cdev, shdisp_dev, 1);

    if (ret) {
        goto shdisp_err_2;
    }

    shdisp_class = class_create(THIS_MODULE, SHDISP_NAME);

    if (IS_ERR(shdisp_class)) {
        goto shdisp_err_2;
    }

    device_create(shdisp_class, NULL,
                  shdisp_dev, &shdisp_cdev, SHDISP_NAME);

    shdisp_addr = ioremap((unsigned long)shdisp_base_addr, SHDISP_AREA_SIZE);

    if (!shdisp_addr) {
        goto shdisp_err_3;
    }

    init_waitqueue_head(&shdisp_wq_for_read);
    init_waitqueue_head(&shdisp_wq_for_ioctl);

    atomic_set(&shdisp_atomic_for_read, 0);
    atomic_set(&shdisp_atomic_for_ioctl, 0);

    ret = request_irq(SHDISP_INT_IRQ, shdisp_sqe_int_isr,
                      SHDISP_INT_FLAGS, SHDISP_NAME, NULL);

    if (ret) {
        goto shdisp_err_4;
    }

    disable_irq(SHDISP_INT_IRQ);

    shdisp_vreg_wlan = vreg_get(NULL, "wlan");

    if (shdisp_vreg_wlan == NULL) {
        goto shdisp_err_5;
    }

#ifdef SHDISP_DEBUG
    entry = create_proc_entry("driver/SHDISP", 0666, NULL); 
    
    if (entry == NULL) {
        goto shdisp_err_5;
    }
    
    entry->write_proc = shdisp_proc_write;
#endif

	hrtimer_init(&dma_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

#if 0
    init_MUTEX(&shdisp_sem);
    init_MUTEX(&shdisp_int_sem);
#else
	sema_init(&shdisp_sem, 1);
	sema_init(&shdisp_int_sem, 1);
#endif


    shdisp_fbcon_status = SHDISP_FBCON_ACTIVATED;

    shdisp_fb_open();

    return 0;

shdisp_err_5:
    free_irq(SHDISP_INT_IRQ, NULL);

shdisp_err_4:
    shdisp_addr = NULL;

shdisp_err_3:
    device_destroy(shdisp_class, shdisp_dev);
    class_destroy(shdisp_class);

shdisp_err_2:
    cdev_del(&shdisp_cdev);

shdisp_err_1:
    return -1;
}
module_init(shdisp_init);

/*
 *  shdisp_exit
 */

static void shdisp_exit(void)
{
    shdisp_vreg_wlan = NULL;
    free_irq(SHDISP_INT_IRQ, NULL);
    shdisp_addr = NULL;
    shdisp_vram_base = NULL;
    device_destroy(shdisp_class, shdisp_dev);
    class_destroy(shdisp_class);
    cdev_del(&shdisp_cdev);
    shdisp_fb_close();

    return;
}
module_exit(shdisp_exit);

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
