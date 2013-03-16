/* drivers/sharp/btpm/btpm.c  (BT Power Management)
 *
 * Copyright (C) 2009 SHARP CORPORATION
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
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <linux/hrtimer.h>

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Definition

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
#define _BTPM_NAME_	"btpm"

#define disp_err( format, args... ) \
	printk( KERN_ERR "[%s] " format, _BTPM_NAME_ , ##args )
#define disp_war( format, args... ) \
	printk( KERN_WARNING "[%s] " format, _BTPM_NAME_ , ##args )

#ifdef BTPM_DEBUG
  #define disp_inf( format, args... ) \
	printk( KERN_ERR "[%s] " format, _BTPM_NAME_ , ##args )
  #define disp_dbg( format, args... ) \
	printk( KERN_ERR "[%s] " format, _BTPM_NAME_ , ##args )
  #define disp_trc( format, args... ) \
	printk( KERN_ERR "[%s] trace:%s " format, _BTPM_NAME_ , __func__ , ##args )
#else
  #define disp_inf( format, args... ) 
  #define disp_dbg( format, args... ) 
  #define disp_trc( format, args... )
#endif /* BTPM_DEBUG */

/* private data */
typedef struct {
	int bluetooth;								/* power status of bluetooth */
} btpm_data_t;

typedef enum {
	BTPM_STATE_OFF,
	BTPM_STATE_ON,
	BTPM_STATE_RESET = 9
} eBTPM_STATE;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Configuration

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* GPIO mapping for Wireless LAN device */
#define BTPM_PORT_RF_LNA_EN			105			/* BT_POW       */
#define BTPM_PORT_BT_RESET_N		108			/* BTRST_N      */
#define BTPM_WAKE_OUT				86			/* BT_WAKEOUT_N */
#define BTPM_WAKE_IN				142			/* BT_WAKE_IN   */

#ifdef BTPM_DRVSTR_6MA
    #undef BTPM_DRVSTR_6MA
#endif
#ifdef BTPM_DRVSTR_8MA
    #undef BTPM_DRVSTR_8MA
#endif
#ifdef BTPM_BCM_4329
    #undef BTPM_BCM_4329
#endif
#ifdef BTPM_BCM_4330
    #undef BTPM_BCM_4330
#endif
#if defined(CONFIG_MACH_DECKARD_AF20)
    #define BTPM_DRVSTR_8MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_DECKARD_AF21)
    #define BTPM_DRVSTR_8MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_DECKARD_AS35)
    #define BTPM_DRVSTR_8MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_DECKARD_AS36)
    #define BTPM_DRVSTR_8MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_DECKARD_AS46)
    #define BTPM_DRVSTR_8MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_DECKARD_AS50)
    #define BTPM_DRVSTR_8MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_DECKARD_AF30)
    #define BTPM_DRVSTR_8MA
    #define BTPM_BCM_4330
#elif defined(CONFIG_MACH_DECKARD_AF33)
    #define BTPM_DRVSTR_8MA
    #define BTPM_BCM_4330
#elif defined(CONFIG_MACH_STR)
    #define BTPM_DRVSTR_6MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_RYK)
    #define BTPM_DRVSTR_6MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_NSK)
    #define BTPM_DRVSTR_6MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_LYNX_DC40)
    #define BTPM_DRVSTR_6MA
    #define BTPM_BCM_4329
#elif defined(CONFIG_MACH_LYNX_DC45)
    #define BTPM_DRVSTR_6MA
    #define BTPM_BCM_4329
#else
    #define BTPM_DRVSTR_6MA
    #define BTPM_BCM_4330
#endif

/* Port Configuration Table */
static unsigned btpm_gpio_config[] = {
	/*        gpio,                 func, dir,             pull,             drvstr */
#ifdef BTPM_DRVSTR_8MA
	GPIO_CFG( BTPM_PORT_RF_LNA_EN,  0,    GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG( BTPM_PORT_BT_RESET_N, 0,    GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
#else  /* BTPM_DRVSTR_6MA */
	GPIO_CFG( BTPM_PORT_RF_LNA_EN,  0,    GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
	GPIO_CFG( BTPM_PORT_BT_RESET_N, 0,    GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
#endif
};

#ifndef BTPM_INI_BT
  #define BTPM_INI_BT 0
#endif

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Resource

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
struct platform_device *p_btpm_dev = NULL;

static int wifi;								/* power status of WiFi */
static int bluetooth;							/* power status of Bluetooth */


/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Local

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static int btpm_reset( struct device *pdev )
{
	btpm_data_t *p_sts;
    struct timespec tu;

	if ( pdev == NULL ){
		disp_err( "device not found\n" );
		return -1;
	}

	p_sts = (btpm_data_t *)dev_get_drvdata(pdev);
	if ( p_sts == NULL ){
		disp_err( "driver infomation not found\n" );
		return -1;
	}

	gpio_set_value( BTPM_PORT_BT_RESET_N, 0 );          /* BTRST_N LOW */
	/* BC7 is always turned on */
    tu.tv_sec = (time_t)0;
    tu.tv_nsec = 1000000;                               /* over 1ms */
    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
    /* RF OFF -Low power mode- */
	gpio_set_value( BTPM_PORT_RF_LNA_EN, 0 );           /* BT_POW LOW */

    /* init status */
	p_sts->bluetooth   = 0;
	bluetooth          = 0;
	wifi               = 0;

	return 0;
}

static int btpm_bluetooth_on( struct device *pdev , int on )
{
	btpm_data_t *p_sts;
    struct timespec tu;
#ifdef BTPM_BCM_4330
	int ret;
#endif

	if ( pdev == NULL ){
		disp_err( "device not found\n" );
		return -1;
	}

	p_sts = (btpm_data_t *)dev_get_drvdata(pdev);
	if ( p_sts == NULL ){
		disp_err( "driver infomation not found\n" );
		return -1;
	}

	if ( p_sts->bluetooth == on ){
		disp_dbg( "%s: no need to change status (%d->%d)\n" , __func__, p_sts->bluetooth , on );
		return 0;
	}

	if ( on ){
#ifdef BTPM_BCM_4330
		/* BT WAKE Configuration */
		ret = gpio_tlmm_config(GPIO_CFG( BTPM_WAKE_OUT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_6MA), GPIO_CFG_ENABLE);
		if (ret) {
			disp_err( "uart gpio_tlmm_config(BTPM_WAKE_OUT) : %d\n", ret);
			return -EIO;
		}
		ret = gpio_tlmm_config(GPIO_CFG( BTPM_WAKE_IN,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP,   GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			disp_err( "uart gpio_tlmm_config(BTPM_WAKE_IN ) : %d\n", ret);
			return -EIO;
		}
#endif
		/* Turn ON RF/WLAN_IO-3.0V */
		gpio_set_value( BTPM_PORT_BT_RESET_N, 0 );              /* BTRST_N LOW */
		if( wifi <= 0 ){
			gpio_set_value( BTPM_PORT_RF_LNA_EN, 1 );		    /* BT_POW HIGH */
    		disp_dbg( "%s: RF ON\n" , __func__);
			/* BC7 is always turned on */
		}
        tu.tv_sec = (time_t)0;
        tu.tv_nsec = 20000000;                                  /* over 20ms */
        hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
		gpio_set_value( BTPM_PORT_BT_RESET_N, 1 );              /* BTRST_N HIGH */
        tu.tv_sec = (time_t)0;
        tu.tv_nsec = 200000000;                                  /* over 200ms */
        hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
		gpio_set_value( BTPM_WAKE_OUT, 0 );                     /* BT_WAKEOUT_N LOW(WAKE) */
	} else {
		gpio_set_value( BTPM_WAKE_OUT, 1 );                     /* BT_WAKEOUT_N HIGH(SLEEP) */
		gpio_set_value( BTPM_PORT_BT_RESET_N, 0 );              /* BTRST_N LOW */
		if( wifi <= 0 ){
            tu.tv_sec = (time_t)0;
            tu.tv_nsec = 1000000;                               /* over 1ms */
            hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
			/* BC7 is always turned on */
			gpio_set_value( BTPM_PORT_RF_LNA_EN, 0 );		    /* BT_POW LOW */
    		disp_dbg( "%s: RF OFF\n" , __func__);
		}
#ifdef BTPM_BCM_4330
		/* BT WAKE Configuration */
		ret = gpio_tlmm_config(GPIO_CFG( BTPM_WAKE_OUT, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
		if (ret) {
			disp_err( "gpio_tlmm_config(BTPM_WAKE_OUT) : %d\n", ret);
			return -EIO;
		}
		ret = gpio_tlmm_config(GPIO_CFG( BTPM_WAKE_IN,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			disp_err( "gpio_tlmm_config(BTPM_WAKE_IN) : %d\n", ret);
			return -EIO;
		}
#endif
	}

	if ( p_sts->bluetooth < 0 ){
		disp_inf( "Bluetooth power on reset\n" );
	} else {
		disp_dbg( "%s: change status (%d->%d)\n" , __func__, p_sts->bluetooth , on );
	}
	p_sts->bluetooth = on;
	bluetooth = on;

	return 0;
}

static int btpm_bluetooth_reset( struct device *pdev , int on )
{
	btpm_data_t *p_sts;
    struct timespec tu;

	if ( pdev == NULL ){
		disp_err( "device not found\n" );
		return -1;
	}

	p_sts = (btpm_data_t *)dev_get_drvdata(pdev);
	if ( p_sts == NULL ){
		disp_err( "driver infomation not found\n" );
		return -1;
	}

	gpio_set_value( BTPM_PORT_BT_RESET_N, 0 );          /* BTRST_N LOW */
    tu.tv_sec = (time_t)0;
    tu.tv_nsec = 20000000;                              /* over 20ms */
    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
	gpio_set_value( BTPM_PORT_BT_RESET_N, 1 );          /* BTRST_N HIGH */
    tu.tv_sec = (time_t)0;
    tu.tv_nsec = 200000000;                             /* over 200ms */
    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

	return 0;
}

static int btpm_power_off( struct device *pdev )
{
	btpm_data_t *p_sts;
    struct timespec tu;

	if ( pdev == NULL ){
		disp_err( "device not found\n" );
		return -1;
	}
	p_sts = (btpm_data_t *)dev_get_drvdata(pdev);
	if ( p_sts == NULL ){
		disp_err( "driver infomation not found\n" );
		return -1;
	}

	p_sts->bluetooth = 0;
	bluetooth = 0;

	if( wifi <= 0 ){
        tu.tv_sec = (time_t)0;
        tu.tv_nsec = 1000000;                           /* over 1ms */
        hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
		/* BC7 is always turned on */
		gpio_set_value( BTPM_PORT_RF_LNA_EN, 0 );       /* BT_POW LOW */
	}

	gpio_set_value( BTPM_PORT_BT_RESET_N, 1 );          /* BTRST_N HIGH */
	gpio_set_value( BTPM_PORT_BT_RESET_N, 0 );          /* BTRST_N LOW  */

    tu.tv_sec = (time_t)0;
    tu.tv_nsec = 10000000;                              /* over 10ms */
    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

	return 0;
}

void btpm_wifi_reg( int on )
{
	if( on ){                                           /* WLAN ON */
		if( bluetooth <= 0 ){
			gpio_set_value( BTPM_PORT_RF_LNA_EN, 1 );   /* BT_POW HIGH */
    		disp_dbg( "%s: RF ON\n" , __func__);
		}
	} else {                                            /* WLAN OFF */
		if( bluetooth <= 0 ){
			gpio_set_value( BTPM_PORT_RF_LNA_EN, 0 );   /* BT_POW LOW */
    		disp_dbg( "%s: RF OFF\n" , __func__);
		}
	}
	wifi = on;
}
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Device attribute

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static
ssize_t show_bluetooth_power(struct device *pdev, struct device_attribute *pattr, char *buf)
{
	btpm_data_t *p_priv = (btpm_data_t *)dev_get_drvdata(pdev);

	buf[0] = (char)(p_priv->bluetooth);
	
	return( 1 );
}

static
ssize_t set_bluetooth_power(struct device *pdev, struct device_attribute *pattr, const char *buf, size_t count)
{
	if ( (buf[0] == BTPM_STATE_OFF) || (buf[0] == BTPM_STATE_ON) ){
		btpm_bluetooth_on( pdev, (int)buf[0] );
        return( count );
	} else if ( (buf[0] == BTPM_STATE_RESET) ){
		btpm_bluetooth_reset( pdev, (int)buf[0] );
        return( count );
    }

	return( 0 );
}


/* device attribute structure */
static DEVICE_ATTR(
	bluetooth,
	((S_IRUGO | S_IWUGO) & (~S_IWOTH)),			/* R:user/group/other W:user/group */
	show_bluetooth_power,		/* read */
	set_bluetooth_power			/* write */
);


static struct attribute *btpm_device_attributes[] = {
	&dev_attr_bluetooth.attr,
	NULL,
};

static struct attribute_group btpm_device_attributes_gourp = {
	.attrs = btpm_device_attributes,
};

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Driver Description

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static int __init btpm_driver_probe( struct platform_device *pdev )
{
	int pin;
	int ret;
	btpm_data_t *p_priv;

    int ini;

	/* Port Configuration */
	for (pin = 0; pin < ARRAY_SIZE(btpm_gpio_config); pin++) {
		ret = gpio_tlmm_config( btpm_gpio_config[pin], GPIO_CFG_ENABLE );
		if (ret) {
			disp_err( "gpio_tlmm_config(%d)<-%#x : %d\n", pin,btpm_gpio_config[pin], ret);
			return -EIO;
		}
	}

	/* Initialize private data */
	p_priv = kmalloc( sizeof(*p_priv) , GFP_KERNEL );
	if ( p_priv == NULL ){
		disp_err( "memory allocation for private data failed\n" );
		return -ENOMEM;
	}
	platform_set_drvdata( pdev , p_priv );

	/* power on reset */
    btpm_reset( &(pdev->dev) );

#ifdef BTPM_BCM_4330
	/* BT WAKE Configuration */
	ret = gpio_tlmm_config(GPIO_CFG( BTPM_WAKE_OUT, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
	if (ret) {
		disp_err( "gpio_tlmm_config(BTPM_WAKE_OUT) : %d\n", ret);
		return -EIO;
	}
	ret = gpio_tlmm_config(GPIO_CFG( BTPM_WAKE_IN,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret) {
		disp_err( "gpio_tlmm_config(BTPM_WAKE_IN) : %d\n", ret);
		return -EIO;
	}
#endif

	/* power on */
	ini = BTPM_INI_BT;
	if ( ini ){
		btpm_bluetooth_on( &(pdev->dev) , 1 );
	}

	/* create sysfs interface */
	ret = sysfs_create_group( &(pdev->dev.kobj), &btpm_device_attributes_gourp);
	if ( ret ){
		disp_err( "Sysfs attribute export failed with error %d.\n" , ret );
	}

	return ret;
}

static int btpm_driver_remove( struct platform_device *pdev )
{
	btpm_data_t *p_priv;

	sysfs_remove_group( &(pdev->dev.kobj), &btpm_device_attributes_gourp);
	
	p_priv = platform_get_drvdata( pdev );
	platform_set_drvdata( pdev , NULL );
	if ( p_priv != NULL ){
		kfree( p_priv );
	}
	
	return 0;
}

static void btpm_driver_shutdown( struct platform_device *pdev )
{
	btpm_power_off( &(pdev->dev) );

	return;
}

/* driver structure */
static struct platform_driver btpm_driver = {
	.remove = __devexit_p(btpm_driver_remove),
	.shutdown = __devexit_p(btpm_driver_shutdown),
	.driver = {
		.name = _BTPM_NAME_,
		.owner = THIS_MODULE,
	},
};

static int btpm_driver_init( void )
{
	int ret;
	
	/* regist driver */
	ret = platform_driver_probe( &btpm_driver, btpm_driver_probe );
	if ( ret != 0 ){
		disp_err( "driver register failed (%d)\n" , ret );
	}

	return ret;
}

static void btpm_driver_exit( void )
{
	platform_driver_unregister( &btpm_driver );
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Device Description

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static int btpm_device_init( void )
{
	int ret;
	
	/* allocate device structure */
	p_btpm_dev = platform_device_alloc( _BTPM_NAME_ , -1 );
	if ( p_btpm_dev == NULL ){
		disp_err( "device allocation failed\n" );
		return -ENOMEM;
	}

	/* regist device */
	ret = platform_device_add( p_btpm_dev );
	if ( ret != 0 ){
		disp_err( "device register failed (%d)\n" , ret );
	}

	return ret;
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	Module Description

 *:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static int __init btpm_module_init( void )
{
	int ret;
	
	disp_inf( "Bluetooth Power Management\n" );

	ret = btpm_device_init();
	if ( ret == 0 ){
		ret = btpm_driver_init();
	}

	return ret;
}

static void __exit btpm_module_exit( void )
{
	btpm_driver_exit();
}


EXPORT_SYMBOL(btpm_bluetooth_on);
EXPORT_SYMBOL(btpm_wifi_reg);

MODULE_DESCRIPTION("Bluetooth Power Management");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("0.10");

module_init(btpm_module_init);
module_exit(btpm_module_exit);

