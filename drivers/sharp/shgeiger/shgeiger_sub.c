/*
  *Copyright (C) 2012 SHARP CORPORATION All rights reserved.
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
 * SHARP Geiger Sub DRIVER
*/

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>

#include <sharp/shgeiger_sub.h>

#define DEBUG 1 
#define SHGEIGERSUB_DEBUG_MSG			1
#define SHGEIGERSUB_DEBUG_FUNC			1
#define SHGEIGERSUB_DEBUG_FUNC_FIN		1

#if SHGEIGERSUB_DEBUG_FUNC
#define FUNC_LOG() printk(KERN_DEBUG "[SHGEIGERSUB] %s is called\n", __func__)
#else
#define FUNC_LOG()
#endif

#if SHGEIGERSUB_DEBUG_FUNC_FIN
#define FUNC_LOG_FIN() printk(KERN_DEBUG "[SHGEIGERSUB] %s is finished\n", __func__)
#else
#define FUNC_LOG_FIN()
#endif

#if SHGEIGERSUB_DEBUG_MSG
#define DEBUG_LOG(format, ...) printk(KERN_DEBUG "[SHGEIGERSUB] " format "\n", ## __VA_ARGS__)
#else
#define DEBUG_LOG(format, ...)
#endif

#define DEBUG_BUILD

/*+-------------------------------------------------------------------------+*/
/*|																			|*/
/*+-------------------------------------------------------------------------+*/
typedef struct i2c_client       I2cClt;
typedef struct i2c_device_id    I2cDevID;

#define	I2C_RETRY			3

typedef struct
{
	uint8_t	mbRegAdr;					/* Register */
	uint8_t mbData;						/* Data */
} I2cWriteData;

static I2cClt		*this_client;

static int SHGEIGERSUB_Probe(I2cClt *client, const I2cDevID *poDevId);

int SHGEIGER_EEPROM_I2cRead(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY; loop_i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= I2C_RETRY) {
		printk(KERN_ERR "I2cRead: error\n");

		return -EIO;
	}

	return 0;
}

static int SHGEIGERSUB_Remove(I2cClt *client)
{

	FUNC_LOG();

	DEBUG_LOG("Remove()");

	return 0;	
}

static const I2cDevID gI2cDevIdTableAcc[] =
{
   { SH_GEIGERSUB_I2C_DEVNAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTableAcc);

static struct i2c_driver goI2cAccDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = SH_GEIGERSUB_I2C_DEVNAME,
	},
	.probe	  = SHGEIGERSUB_Probe,
	.remove	  = SHGEIGERSUB_Remove,
	.id_table = gI2cDevIdTableAcc,
};

static int SHGEIGERSUB_Probe(I2cClt *client, const I2cDevID *poDevId)
{
	int nResult;

	FUNC_LOG();

	/* CLIENT CHECKING */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DEBUG_LOG("check_functionality failed.");
		nResult = -ENODEV;
		goto exit;
	}

	/* Copy to global variable */
	this_client = client;

	return 0;
	
exit:
	return nResult;

}

static int __init SHGEIGERSUB_Init(void)
{
	FUNC_LOG();

	/* I2C driver Use beginning */
	return i2c_add_driver(&goI2cAccDriver);

	return 0;
}

static void __exit SHGEIGERSUB_Exit(void)
{
	FUNC_LOG();

	/* I2C driver Use end */
	i2c_del_driver(&goI2cAccDriver);
}


module_init(SHGEIGERSUB_Init);
module_exit(SHGEIGERSUB_Exit);


MODULE_DESCRIPTION("shgeiger sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");



