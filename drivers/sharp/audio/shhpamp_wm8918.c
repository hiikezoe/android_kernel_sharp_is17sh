/* drivers/sharp/audio/shhpamp_wm8918.c
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
/* CONFIG_SH_AUDIO_DRIVER newly created */
#include <asm/uaccess.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/kernel.h>

#include <sharp/shhpamp.h>
#include <sharp/shspamp.h>

#define SHHPAMP_I2C_DRIVER_NAME	"sh_hpamp_i2c"
#define SHHPAMP_MASTER_CLOCK	"mi2s_codec_rx_m_clk"

#ifdef CONFIG_DEBUG_FS
static int shhpamp_i2c_read(u8 reg, u16 *data);
#endif /* CONFIG_DEBUG_FS */
static int shhpamp_i2c_write(u8 reg, u16 data);

void shhpamp_calibrate(void);
void shhpamp_set_dac_boost(u16 data);
void shhpamp_set_dac_vol(u16 data);
void shhpamp_set_hpout_vol(u16 data);
void shhpamp_set_volume(int path);
void shhpamp_set_hp_volume(void);
void shhpamp_set_hp_and_sp_volume(void);

static struct i2c_client *shhpamp_client;
static struct clk *shhpamp_mclk;

#ifdef CONFIG_DEBUG_FS
static int shhpamp_i2c_read(u8 reg, u16 *data)
{
	int rc;

    if (shhpamp_client) {
        u8 buff[2];
        struct i2c_msg msg[2] = {
            {
                .addr = shhpamp_client->addr,
                .flags= 0,
                .len  = 1,
                .buf  = &reg,
            },
            {
                .addr = shhpamp_client->addr,
                .flags= I2C_M_RD,
                .len  = 2,
                .buf  = buff,
            }
        };

        rc = i2c_transfer(shhpamp_client->adapter, msg, 2);
        if (rc != 2) {
            dev_err(&shhpamp_client->dev, "%s: read of register %d\n", __func__, reg);
            rc = -1;
            goto i2c_rd_exit;
        }

        *data = (buff[0] << 8) + (buff[1] & 0xFF);

        pr_debug("%s: addr=0x%02x, value(MSByte)=0x%02x, value(LSByte)=0x%02x, value=0x%04x", __func__, reg, buff[0], buff[1], *data);
    } else {
        pr_err("%s: shhpamp_client is NULL\n", __func__);
        rc = -1;
    }

i2c_rd_exit:
	return rc;
}
#endif /* CONFIG_DEBUG_FS */

static int shhpamp_i2c_write(u8 reg, u16 data)
{
	int rc;

    if (shhpamp_client) {
        u8 buff[3];
        struct i2c_msg msg = {
            .addr = shhpamp_client->addr,
            .flags= 0,
            .len  = 3,
            .buf  = buff,
        };

        buff[0] = reg;
        buff[1] = (data >> 8);
        buff[2] = (data & 0xFF);

        pr_debug("%s: addr=0x%02x, value(MSByte)=0x%02x, value(LSByte)=0x%02x, value=0x%04x", __func__, buff[0], buff[1], buff[2], data);

        rc = i2c_transfer(shhpamp_client->adapter, &msg, 1);
        if (rc != 1) {
            dev_err(&shhpamp_client->dev, "%s: writing to reg %d, rc=%d\n", __func__, reg, rc);
            rc = -1;
        }
    } else {
        pr_err("%s: shhpamp_client is NULL\n", __func__);
        rc = -1;
    }
	
	return rc;
}

void shhpamp_calibrate()
{
    pr_debug("shhpamp_calibrate!");

    clk_enable(shhpamp_mclk);

    /* 1b */
    shhpamp_i2c_write(0x04, 0x0019);
    shhpamp_i2c_write(0x05, 0x0007);
    msleep(5);
    shhpamp_i2c_write(0x05, 0x0003);
    shhpamp_i2c_write(0x16, 0x0006);

    /* 2 */
    shhpamp_i2c_write(0x0E, 0x0003);
    shhpamp_i2c_write(0x12, 0x000C);
    msleep(3);
    shhpamp_i2c_write(0x62, 0x0001);
    msleep(5);
    shhpamp_i2c_write(0x5A, 0x0011);
    shhpamp_i2c_write(0x5A, 0x0033);
    shhpamp_i2c_write(0x43, 0x0003);
    shhpamp_i2c_write(0x44, 0x0030);
    msleep(129);

    /* 3 */
    shhpamp_i2c_write(0x21, 0x0608);
    shhpamp_i2c_write(0x5A, 0x0077);
    shhpamp_i2c_write(0x5A, 0x0000);
    shhpamp_i2c_write(0x68, 0x0004);
    shhpamp_i2c_write(0x62, 0x0000);
    shhpamp_i2c_write(0x12, 0x0000);
    shhpamp_i2c_write(0x0E, 0x0000);

    /* 5 */
    shhpamp_i2c_write(0x16, 0x0000);
    shhpamp_i2c_write(0x05, 0x0000);
    shhpamp_i2c_write(0x04, 0x0018);

    clk_disable(shhpamp_mclk);
}

void shhpamp_set_dac_boost(u16 data)
{
    shhpamp_i2c_write(SHHPAMP_AUDIO_INTERFACE_0, SHHPAMP_AUDIO_INTERFACE_0_OTHER | data);
}

void shhpamp_set_dac_vol(u16 data)
{
    shhpamp_i2c_write(SHHPAMP_DAC_DIGITAL_VOLUME_LEFT, data);
    shhpamp_i2c_write(SHHPAMP_DAC_DIGITAL_VOLUME_RIGHT, SHHPAMP_DAC_VU | data);
}

void shhpamp_set_hpout_vol(u16 data)
{
    shhpamp_i2c_write(SHHPAMP_ANALOGUE_OUT1_LEFT, data);
    shhpamp_i2c_write(SHHPAMP_ANALOGUE_OUT1_RIGHT, SHHPAMP_HPOUT_VU | data);
}

void shhpamp_set_hp_volume()
{
    pr_debug("shhpamp_set_hp_volume!");

//    shhpamp_set_dac_boost(SHHPAMP_DAC_BOOST_0DB);
//    shhpamp_set_dac_vol(SHHPAMP_DAC_VOL_0DB);

    shhpamp_set_hpout_vol(SHHPAMP_HPOUT_VOL_MINUS5DB);
}

void shhpamp_set_hp_and_sp_volume()
{
    pr_debug("shhpamp_set_hp_and_sp_volume!");

//    shhpamp_set_dac_boost(SHHPAMP_DAC_BOOST_0DB);
//    shhpamp_set_dac_vol(SHHPAMP_DAC_VOL_0DB);

    shhpamp_set_hpout_vol(SHHPAMP_HPOUT_VOL_MINUS5DB);
}

void shhpamp_set_volume(int path)
{
    if (path == SHHPAMP_PATH_HP_AND_SP) {
        shhpamp_set_hp_and_sp_volume();
    } else {
        shhpamp_set_hp_volume();
    }
}

void shhpamp_poweron_internal(int path)
{
    pr_debug("shhpamp_poweron_internal! path=%d", path);

    /* 4 */
    shhpamp_i2c_write(0x04, 0x0019);
    shhpamp_i2c_write(0x05, 0x0007);
    msleep(5);
    shhpamp_i2c_write(0x05, 0x0003);
    shhpamp_i2c_write(0x16, 0x0000);
    shhpamp_i2c_write(0x16, 0x0006);
    shhpamp_i2c_write(0x74, 0x0000);
    shhpamp_i2c_write(0x19, 0x0002);
    shhpamp_i2c_write(0x0E, 0x0003);
    shhpamp_i2c_write(0x12, 0x000C);
    msleep(2);
    shhpamp_i2c_write(0x62, 0x0001);
    msleep(5);
    shhpamp_i2c_write(0x5A, 0x0011);
    shhpamp_i2c_write(0x5A, 0x0033);
    shhpamp_i2c_write(0x5A, 0x0077);
    shhpamp_i2c_write(0x5A, 0x00FF);
    shhpamp_i2c_write(0x68, 0x0005);
    shhpamp_set_volume(path);
    shhpamp_i2c_write(0x21, 0x0200);
}

void shhpamp_poweron()
{
    shhpamp_poweron_internal(SHHPAMP_PATH_HP);
}
EXPORT_SYMBOL(shhpamp_poweron);

void shhpamp_poweroff()
{
    pr_debug("shhpamp_poweroff!");

    /* 3 */
    shhpamp_i2c_write(0x21, 0x0608);
    shhpamp_i2c_write(0x5A, 0x0077);
    shhpamp_i2c_write(0x5A, 0x0000);
    shhpamp_i2c_write(0x68, 0x0004);
    shhpamp_i2c_write(0x62, 0x0000);
    shhpamp_i2c_write(0x12, 0x0000);
    shhpamp_i2c_write(0x0E, 0x0000);

    /* 5 */
    shhpamp_i2c_write(0x16, 0x0000);
    shhpamp_i2c_write(0x05, 0x0000);
    shhpamp_i2c_write(0x04, 0x0018);
}
EXPORT_SYMBOL(shhpamp_poweroff);

void shhpamp_and_shspamp_poweron()
{
    shhpamp_poweron_internal(SHHPAMP_PATH_HP_AND_SP);
    shspamp_poweron();
}
EXPORT_SYMBOL(shhpamp_and_shspamp_poweron);

void shhpamp_and_shspamp_poweroff()
{
    shhpamp_poweroff();
    shspamp_poweroff();
}
EXPORT_SYMBOL(shhpamp_and_shspamp_poweroff);

#ifdef CONFIG_DEBUG_FS

static int shhpamp_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, long int *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");

	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token != NULL) {
			if ((token[1] == 'x') || (token[1] == 'X')) {
				base = 16;
			} else {
				base = 10;
            }

			if (strict_strtoul(token, base, &param1[cnt]) != 0) {
				return -EINVAL;
            }

			token = strsep(&buf, " ");
        } else {
			return -EINVAL;
        }
	}
	return 0;
}

static u16 read_data;

static ssize_t shhpamp_debug_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	char lbuf[8];

	snprintf(lbuf, sizeof(lbuf), "0x%x\n", read_data);
	return simple_read_from_buffer(ubuf, count, ppos, lbuf, strlen(lbuf));
}

static ssize_t shhpamp_debug_write(struct file *filp, const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *access_str = filp->private_data;
	char lbuf[32];
	int rc;
	long int param[5];

	if (cnt > sizeof(lbuf) - 1) {
		return -EINVAL;
    }

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc) {
		return -EFAULT;
    }

	lbuf[cnt] = '\0';

	if (!strcmp(access_str, "power")) {
		if (get_parameters(lbuf, param, 1) == 0) {
			switch (param[0]) {
			case 1:
				shhpamp_poweron();
				break;
			case 0:
				shhpamp_poweroff();
				break;
			default:
				rc = -EINVAL;
				break;
			}
		} else {
			rc = -EINVAL;
        }
	} else if (!strcmp(access_str, "poke")) {
		/* write */
		rc = get_parameters(lbuf, param, 2);
		if ((param[0] <= 0xFF) && (param[1] <= 0xFFFF) && (rc == 0)) {
			shhpamp_i2c_write(param[0], param[1]);
		} else {
			rc = -EINVAL;
        }
	} else if (!strcmp(access_str, "peek")) {
		/* read */
		rc = get_parameters(lbuf, param, 1);
		if ((param[0] <= 0xFF) && (rc == 0)) {
			shhpamp_i2c_read(param[0], &read_data);
		} else {
			rc = -EINVAL;
        }
	}

	if (rc == 0) {
		rc = cnt;
	} else {
		pr_err("%s: rc = %d\n", __func__, rc);
    }

	return rc;
}

static const struct file_operations shhpamp_debug_ops = {
	.open = shhpamp_debug_open,
	.write = shhpamp_debug_write,
	.read = shhpamp_debug_read
};

#define DEBUGFS_MODE (S_IFREG | S_IRUGO)
#define DEBUGFS_CREATE_FILE(name) \
    debugfs_create_file(name, DEBUGFS_MODE, dent, (void *)name, &shhpamp_debug_ops)

#endif /* CONFIG_DEBUG_FS */

static int shhpamp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef CONFIG_DEBUG_FS
    struct dentry *dent = debugfs_create_dir("shhpamp", 0);
	if (!IS_ERR(dent)) {
        DEBUGFS_CREATE_FILE("peek");
        DEBUGFS_CREATE_FILE("poke");
        DEBUGFS_CREATE_FILE("power");
	}
#endif /* CONFIG_DEBUG_FS */

	shhpamp_client = client;

	return 0;
}

static const struct i2c_device_id shhpamp_i2c_id[] = {
	{ SHHPAMP_I2C_DRIVER_NAME, 0 },
	{}
};

static struct i2c_driver shhpamp_i2c_driver = {
	.driver		= {
		.name = SHHPAMP_I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= shhpamp_i2c_probe,
	.id_table	= shhpamp_i2c_id
};

static int __init shhpamp_init(void)
{
	int rc;

    shhpamp_mclk = clk_get(NULL, SHHPAMP_MASTER_CLOCK);
    if (IS_ERR(shhpamp_mclk)) {
        pr_err("%s: can't get shhpamp_mclk\n", __func__);
        return IS_ERR(shhpamp_mclk);
    }

	rc = i2c_add_driver(&shhpamp_i2c_driver);
    if (rc) {
        pr_err("%s: i2c_add_driver rc=%d\n", __func__, rc);
        goto init_exit;
    }

    shhpamp_calibrate();

	return 0;
	
init_exit:
    if (shhpamp_mclk) {
        clk_put(shhpamp_mclk);
    }
	return rc;
}

static void __exit shhpamp_exit(void)
{
    if (shhpamp_mclk) {
        clk_put(shhpamp_mclk);
    }
	i2c_del_driver(&shhpamp_i2c_driver);
}

MODULE_LICENSE("GPL");

module_init(shhpamp_init);
module_exit(shhpamp_exit);
