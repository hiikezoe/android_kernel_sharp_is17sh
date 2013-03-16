/* drivers/sharp/shvibrator/sh_vibrator_diag.c
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
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <mach/pmic.h>

/* SH_AUDIO_DRIVER ADD 2011/06/06 --> */
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <linux/delay.h>
/* SH_AUDIO_DRIVER ADD 2011/06/06 <-- */

/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
#include <linux/slab.h>
#include <linux/i2c.h>
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */
#define SHVIB_DRIVER_NAME		"shvibrator"

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

static dev_t 				shvib_devid;
static struct class*		shvib_class;
static struct device*		shvib_device;
struct cdev 				shvib_cdev;

/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
extern int __init shvibrator_i2c_init(void);
extern void shvibrator_i2c_exit(void);
extern int shvibrator_i2c_read(struct i2c_client *client,
									int reg, u8 *data, int size);
extern int shvibrator_i2c_write(struct i2c_client *client,
									int reg, u8 data);
struct shvib_i2c_data {
	struct i2c_client *client_p;
};
extern struct shvib_i2c_data *vib_data;

/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */

static void set_pmic_vibrator(int on)
{
	if (on){
#if 0 /* Warning fix [start] */
/* SH_AUDIO_DRIVER ADD 2011/06/08 --> */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN -1), 1);
		usleep(300);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN -1), 1);
/* SH_AUDIO_DRIVER ADD 2011/06/08 <-- */

	}else{

/* SH_AUDIO_DRIVER ADD 2011/06/08 --> */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN -1), 0);
		usleep(300);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN -1), 0);
/* SH_AUDIO_DRIVER ADD 2011/06/08 <-- */
#else /* Warning fix */
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN -1), 1);
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
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN -1), 1);
	}else{
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN -1), 0);
		usleep(300);
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN -1), 0);
#endif /* Warning fix [end] */
	}
}

static int shvib_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t shvib_write(struct file *filp, 
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char cmd;

	pr_debug("[shvib]shvib_write(%s) start\n", ubuf);
	
	if(get_user(cmd, ubuf)){
		return -EFAULT;
	}

	pr_debug("[shvib]shvib_write() cmd = %d\n", cmd);
	
	if(cmd == '0'){
		set_pmic_vibrator(0);
	}else{
		set_pmic_vibrator(1);
	}
	
	return cnt;
}

static int shvib_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations shvib_fileops = {
	.owner   = THIS_MODULE,
	.open    = shvib_open,
	.write   = shvib_write,
	.release = shvib_release,
};

static int __init init_pmic_shvibrator(void)
{
	int rc;

/* SH_AUDIO_DRIVER ADD 2011/06/08 --> */


	struct pm_gpio vib_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L17,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.out_strength   = PM_GPIO_STRENGTH_LOW,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0
	};

	rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(SHVIB_RST_PDN - 1),&vib_config);

	if(rc < 0){
		pr_err("linearVIB reset :gpio_config error\n");
		goto err_gpio_request;
	}

	rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(SHVIB_PDN - 1),&vib_config);

	if(rc < 0){
		pr_err("linearVIB:gpio_config error\n");
		goto err_gpio_request;
	}

/* SH_AUDIO_DRIVER ADD 2011/06/08 <-- */

	rc = alloc_chrdev_region(&shvib_devid, 0, 1, SHVIB_DRIVER_NAME);
	if(rc < 0){
		pr_err("shvib:alloc_chrdev_region error\n");
		return rc;
	}
/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
	rc = shvibrator_i2c_init();
	if(rc < 0){
		printk(KERN_WARNING "linearVIB:shvibrator_i2c_init error\n");
	}
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */

	shvib_class = class_create(THIS_MODULE, SHVIB_DRIVER_NAME);
	if (IS_ERR(shvib_class)) {
		rc = PTR_ERR(shvib_class);
		pr_err("shvib:class_create error\n");
		goto error_vid_class_create;
	}

	shvib_device = device_create(shvib_class, NULL, 
								 shvib_devid, &shvib_cdev, 
								 SHVIB_DRIVER_NAME);
	if (IS_ERR(shvib_device)) {
		rc = PTR_ERR(shvib_device);
		pr_err("shvib:device_create error\n");
		goto error_vid_class_device_create;
	}
	
	cdev_init(&shvib_cdev, &shvib_fileops);
	shvib_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shvib_cdev, shvib_devid, 1);
	if(rc < 0){
		pr_err("shvib:cdev_add error\n");
		goto err_via_cdev_add;
	}

	pr_debug("[shvib]shvib_init() done\n");
	
	return 0;
	
err_gpio_request:
	gpio_free(SHVIB_RST_PDN -1);
	gpio_free(SHVIB_PDN -1);
err_via_cdev_add:
	cdev_del(&shvib_cdev);
error_vid_class_device_create:
	class_destroy(shvib_class);
error_vid_class_create:
	unregister_chrdev_region(shvib_devid, 1);

	return rc;
}
module_init( init_pmic_shvibrator );

static void __exit exit_pmic_shvibrator(void)
{
/* SH_AUDIO_DRIVER ADD 2011/10/17 --> */
    shvibrator_i2c_exit();
/* SH_AUDIO_DRIVER ADD 2011/10/17 <-- */
#if 0 /* Warning fix [start] */
/* SH_AUDIO_DRIVER ADD 20110607 -->*/
	gpio_free(SHVIB_RST_PDN);
	gpio_free(SHVIB_PDN);
/* SH_AUDIO_DRIVER ADD 20110607 <--*/
#endif /* Warning fix [end] */

	cdev_del(&shvib_cdev);
	class_destroy(shvib_class);
	unregister_chrdev_region(shvib_devid, 1);
}
module_exit( exit_pmic_shvibrator );

