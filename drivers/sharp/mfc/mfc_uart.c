/* drivers/sharp/mfc/mfc_uart.c (MFC UART Wrapper)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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

#ifdef CONFIG_SHFELICA

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/major.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <asm/termios.h>
#include <asm/uaccess.h>

#include<linux/gpio.h>
#include "mfc_log.h"

/* Common UART */
#define D_UART_DEV_FILE_PATH	("/dev/ttyMSM1")

/* FeliCa UART */
#define D_FELICA_UART_DEVS		(1)
#define D_FELICA_UART_DEV_NAME	("felica")

/*
 * prototype
 */
static __init signed int mfcuart_init(void);
static void __exit mfcuart_exit(void);

/*
 * static variable
 */
static struct class *mfcuart_class = NULL;

static struct cdev felica_uart_cdev;

/*
 * function_common
 */
static inline struct tty_struct *file_tty(struct file *filp)
{
	return ((struct tty_file_private *)filp->private_data)->tty;
}

/*
 * function_common_uart
 */
static ssize_t uart_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	struct file *tty;
	ssize_t result = -ENOSYS;

	MFCUART_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->read) {
		MFCUART_DRV_DBG_LOG("tty read len = %d", len);
		result = tty->f_op->read(tty, buf, len, ppos);
	}

	MFCUART_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static ssize_t uart_write(struct file *filp, const char __user *data, size_t len, loff_t *ppos)
{
	struct file *tty;
	ssize_t result = -ENOSYS;

	MFCUART_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->write) {
		MFCUART_DRV_DBG_LOG("tty write len = %d", len);
		result = tty->f_op->write(tty, data, len, ppos);
	}

	MFCUART_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static unsigned int uart_poll(struct file *filp, poll_table *wait)
{
	struct file *tty;
	unsigned int result = 0;

	MFCUART_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->poll) {
		MFCUART_DRV_DBG_LOG("tty poll");
		result = tty->f_op->poll(tty, wait);
	}

	MFCUART_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static signed long uart_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct file *tty;
	long result = -ENOSYS;

	MFCUART_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->unlocked_ioctl) {
		MFCUART_DRV_DBG_LOG("tty unlocked_ioctl cmd = %d", cmd);
		result = tty->f_op->unlocked_ioctl(tty, cmd, arg);
	}

	MFCUART_DRV_DBG_LOG("END result = %ld", result);

	return result;
}

#ifdef CONFIG_COMPAT
static signed long uart_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct file *tty;
	long result = -ENOSYS;

	MFCUART_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->compat_ioctl) {
		MFCUART_DRV_DBG_LOG("tty compat_ioctl cmd = %d", cmd);
		result = tty->f_op->compat_ioctl(tty, cmd, arg);
	}

	MFCUART_DRV_DBG_LOG("END result = %ld", result);

	return result;
}
#else
#define uart_compat_ioctl NULL
#endif

static signed int uart_open(struct inode *inode, struct file *filp)
{
	signed int result = -ENOSYS;
	struct file *tty;
	struct termios termios;

	MFCUART_DRV_DBG_LOG("START");

	gpio_tlmm_config(GPIO_CFG(51, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
					 GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(52, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA),
					 GPIO_CFG_ENABLE);

	/* open tty driver */
	tty = filp_open(D_UART_DEV_FILE_PATH, filp->f_flags, 0);
	if (IS_ERR(tty)) {
		filp->private_data = NULL;
		result = PTR_ERR(tty);
		MFCUART_DRV_ERR_LOG("filp_open result = %d", result);
		return result;
	}

	if (tty->f_op->unlocked_ioctl) {
		mm_segment_t oldfs = get_fs();
		set_fs(KERNEL_DS);

		/* set UART speed */
		tty->f_op->unlocked_ioctl(tty, TCGETS, (unsigned long)&termios);

		termios.c_cflag = (termios.c_cflag & ~CBAUD) | (B115200 & CBAUD);
		termios.c_cflag &= ~0xD;
		termios.c_cflag |= (CLOCAL | CREAD);
		termios.c_cflag &= ~PARENB;
		termios.c_cflag &= ~CSTOPB;
		termios.c_cflag &= ~CSIZE;
		termios.c_cflag &= ~PARODD;
		termios.c_cflag |= CS8;
		termios.c_cflag |= CRTSCTS;
		termios.c_lflag &= ~(ICANON | IEXTEN | ISIG | ECHO);
		termios.c_oflag &= ~OPOST;
		termios.c_iflag &= ~(ICRNL | INPCK | ISTRIP | IXON | BRKINT);
		termios.c_cc[VMIN] = 0;
		termios.c_cc[VTIME] = 0;

		tty->f_op->unlocked_ioctl(tty, TCSETS, (unsigned long)&termios);

		filp->private_data = tty;
		result = 0;

		set_fs(oldfs);
	} else  {
		filp_close(tty, 0);
		filp->private_data = NULL;
		result = -ENOSYS;
	}

	MFCUART_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static signed int uart_release(struct inode *inode, struct file *filp)
{
	struct file *tty;

	MFCUART_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty) {
		MFCUART_DRV_DBG_LOG("tty close");
		filp_close(tty, 0);
		filp->private_data = NULL;
	}

	gpio_tlmm_config(GPIO_CFG(51, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
					 GPIO_CFG_DISABLE);
	gpio_tlmm_config(GPIO_CFG(52, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA),
					 GPIO_CFG_DISABLE);

	MFCUART_DRV_DBG_LOG("END");

	return 0;
}

static signed int uart_fasync(signed int fd, struct file *filp, signed int on)
{
	struct file *tty;
	signed int result = -ENOSYS;

	MFCUART_DRV_DBG_LOG("START");

	tty = (struct file *)filp->private_data;
	if (tty && tty->f_op->fasync) {
		MFCUART_DRV_DBG_LOG("tty fasync");
		result = tty->f_op->fasync(fd, tty, on);
	}

	MFCUART_DRV_DBG_LOG("END result = %d", result);

	return result;
}

static signed int uart_fsync(struct file *filp, signed int datasync)
{
	MFCUART_DRV_DBG_LOG("ENTER");
	return 0;
}

/*
 * function_felica_uart
 */
static ssize_t felica_uart_write(struct file *filp, const char __user *data, size_t len, loff_t *ppos)
{
	ssize_t result;
	struct file *tty;

	result = uart_write(filp, data, len, ppos);
	if (result > 0) {
		MFCUART_DRV_DBG_LOG("tty_wait_until_sent");
		tty = (struct file *)filp->private_data;
		if (tty) {
			tty_wait_until_sent(file_tty(tty), 0);
		}
	}

	return result;
}

static signed int felica_uart_open(struct inode *inode, struct file *filp)
{
	signed int result;
	result = uart_open(inode, filp);
	return result;
}

static signed int felica_uart_release(struct inode *inode, struct file *filp)
{
	signed int result;
	result = uart_release(inode, filp);
	return result;
}

static const struct file_operations felica_uart_fileops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.read			= uart_read,
	.write			= felica_uart_write,
	.poll			= uart_poll,
	.unlocked_ioctl	= uart_ioctl,
	.compat_ioctl	= uart_compat_ioctl,
	.open			= felica_uart_open,
	.release		= felica_uart_release,
	.fasync			= uart_fasync,
	.fsync			= uart_fsync,
};

static signed int felica_uart_init(void)
{
	signed int result = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFCUART_DRV_DBG_LOG("START");

	result = alloc_chrdev_region(&dev , 0 , D_FELICA_UART_DEVS, D_FELICA_UART_DEV_NAME);
	if (result) {
		MFCUART_DRV_ERR_LOG("alloc_chrdev_region result = %d", result);
		return result;
	}

	cdev_init(&felica_uart_cdev, &felica_uart_fileops);
	felica_uart_cdev.owner = THIS_MODULE;

	result = cdev_add(&felica_uart_cdev, dev, D_FELICA_UART_DEVS);
	if (result) {
		unregister_chrdev_region(dev, D_FELICA_UART_DEVS);
		MFCUART_DRV_ERR_LOG("cdev_add result = %d", result);
		return result;
	}

	class_dev = device_create(mfcuart_class, NULL, dev, NULL, D_FELICA_UART_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&felica_uart_cdev);
		unregister_chrdev_region(dev, D_FELICA_UART_DEVS);
		result = PTR_ERR(class_dev);
		MFCUART_DRV_ERR_LOG("device_create result = %d", result);
		return result;
	}

	MFCUART_DRV_DBG_LOG("END");

	return result;
}

static void felica_uart_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	MFCUART_DRV_DBG_LOG("START");

	cdev_del(&felica_uart_cdev);
	unregister_chrdev_region(dev, D_FELICA_UART_DEVS);

	MFCUART_DRV_DBG_LOG("END");
}

/*
 * mfcuart_init
 */
static __init signed int mfcuart_init(void)
{
	signed int ret;

	mfcuart_class = class_create(THIS_MODULE, "mfcuart");
	if (IS_ERR(mfcuart_class)) {
		return PTR_ERR(mfcuart_class);
	}

	ret = felica_uart_init();
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/*
 * mfcuart_exit
 */
static void __exit mfcuart_exit(void)
{
	class_destroy(mfcuart_class);

	felica_uart_exit();
}

MODULE_LICENSE("GPL v2");

module_init(mfcuart_init);
module_exit(mfcuart_exit);

#endif /* CONFIG_SHFELICA */

