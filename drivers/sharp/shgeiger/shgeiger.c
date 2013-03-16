/*
  *Copyright (C) 2010 SHARP CORPORATION All rights reserved.
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
 * SHARP Geiger DRIVER
*/

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <../../../arch/arm/mach-msm/timer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <sharp/sh_boot_manager.h>
#include <linux/hrtimer.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>

#include <sharp/shgeiger.h>
#include <sharp/shgeiger_sub.h>
#include <sharp/shusb_vbus_power.h>

#define DEBUG 1 
#define SHGEIGER_DEBUG_MSG			1
#define SHGEIGER_DEBUG_FUNC			1
#define SHGEIGER_DEBUG_FUNC_FIN		1
#define SHGEIGER_DEBUG_COUNTPRINT		1

#if SHGEIGER_DEBUG_FUNC
#define FUNC_LOG() printk(KERN_DEBUG "[SHGEIGER] %s is called\n", __func__)
#else
#define FUNC_LOG()
#endif

#if SHGEIGER_DEBUG_FUNC_FIN
#define FUNC_LOG_FIN() printk(KERN_DEBUG "[SHGEIGER] %s is finished\n", __func__)
#else
#define FUNC_LOG_FIN()
#endif

#if SHGEIGER_DEBUG_MSG
#define DEBUG_LOG(format, ...) printk(KERN_DEBUG "[SHGEIGER] " format "\n", ## __VA_ARGS__)
#else
#define DEBUG_LOG(format, ...)
#endif

#if SHGEIGER_DEBUG_COUNTPRINT
#define DEBUG_LOG_COUNTPRINT(format, ...) printk(KERN_DEBUG "[SHGEIGER] " format "\n", ## __VA_ARGS__)
#else
#define DEBUG_LOG_COUNTPRINT(format, ...)
#endif

//#define DEBUG_BUILD

/*+-------------------------------------------------------------------------+*/
/*|																			|*/
/*+-------------------------------------------------------------------------+*/
typedef struct drv_data_tag     drv_data;
typedef struct i2c_client       I2cClt;
typedef struct i2c_device_id    I2cDevID;
typedef struct work_struct      WorkStruct;
typedef struct device           Device;

#define	I2C_RETRY			3
#define REGISTER_MAX 		21

struct count_data {
	uint8_t time_slot_msb;
	uint8_t time_slot_lsb;
	uint8_t num_atom_msb;
	uint8_t num_atom_lsb;
	uint8_t num_dc_msb;
	uint8_t num_dc_lsb;
};

struct drv_data_tag
{
	int			irq_gpio;
	WorkStruct	IrqWork;
};

typedef struct
{
	uint8_t	mbRegAdr;					/* Register */
	uint8_t mbData;						/* Data */
} I2cWriteData;


static I2cClt		*this_client;

static atomic_t		open_flag = ATOMIC_INIT(0);
struct semaphore geiger_mutex;
static atomic_t		active_mode = ATOMIC_INIT(0);
//static atomic_t		start_mode = ATOMIC_INIT(0);
static uint8_t		measure_state = 0x80;
static uint8_t		state_mask = 0x8F;
static atomic_t		first_start = ATOMIC_INIT(0);
static atomic_t		thermalstart_mode = ATOMIC_INIT(0);
static atomic_t		chipver_enable = ATOMIC_INIT(1);
static uint8_t		chipver = 0xFF;
static int  		pvddid = 1; /*0 = module es1, 1 = module pp1*/ 
static int 			alarm_wait = ALARM_NOT_LOCKED;
static int 			thermal_wait = THERMAL_NOT_LOCKED;
static wait_queue_head_t alarmwait_t;
static wait_queue_head_t thermalwait_t;
static uint8_t		atom_threshold_MSB = 0xFF;
static uint8_t		atom_threshold_LSB = 0xFF;
static uint16_t		atom_threshold = 0xFFFF;
static uint8_t 	thermal_threshold_HIGH = 55;
static uint8_t 	thermal_threshold_LOW = 50;
static uint8_t Threshold_high;
static uint8_t Threshold_low;

/*Timer*/
static unsigned long long int total_count_time = 0;
static unsigned long long int total_stop_time = 0;
static int64_t count_start_time = 0;
static int64_t count_end_time = 0;
static int64_t stop_start_time = 0;
static int64_t stop_end_time = 0;
static int64_t period = 0;

/********/
static struct shgeiger_count_data driver_count_data;
static struct shgeiger_count_data clear_count_data;
static struct shgeiger_calibration_data driver_calibration_data;

/*Slot_time = 200ms*/
static char ragdata_es1[REGISTER_MAX] = {0x00,0x00,0x00,0xFF,0xFF,0xFF,0x0F,0xC8,0x40,0xFF,0xC7,0x05,0x50,0x44,0x80,0xD8,0x00,0x18,0x42,0x11,0x00};
static char ragdata_pp1[REGISTER_MAX] = {0x00,0x00,0x00,0xFC,0xFF,0xFF,0x3E,0xC8,0x50,0xFF,0x47,0x05,0x50,0x44,0x65,0xCC,0xF0,0x16,0x62,0x11,0x00};
static char ragdata_pp2[REGISTER_MAX] = {0x00,0x00,0x00,0xFF,0xFF,0xFF,0x0F,0xC8,0x50,0xFF,0x47,0x05,0x4C,0x44,0x65,0xCC,0xF0,0x16,0x62,0x11,0x00};
static char ragdata_other[REGISTER_MAX]={0x00,0x00,0x00,0xFF,0xFF,0xFF,0x0F,0xC8,0x50,0xFF,0x47,0x05,0x4C,0x44,0x65,0xCC,0xF0,0x16,0x62,0x11,0x00};

static int SHGEIGER_Probe(I2cClt *client, const I2cDevID *poDevId);
static int IOECS_Start( void );
static int IOECS_Stop( void );
static int SHGEIGER_RegisterInitialize( void );
static int IOECS_Thermal_Start( void );
static int IOECS_Thermal_Stop( void );

static void SHGEIGER_MutexDown(void)
{
	down(&geiger_mutex);
}

static void SHGEIGER_MutexUP(void)
{
	up(&geiger_mutex);
}

static void SHGEIGER_DBUGPRINT(struct shgeiger_count_data *data)
{
	DEBUG_LOG_COUNTPRINT("time_slot %d num_atom %d num_dc %d thermal %d,total_time_nsec %llu,total_stop_time_nsec %llu",data->num_time_slot,data->num_atom,data->num_dc,data->thermal_data,data->total_time_nsec,data->total_stoptime_nsec);
}

//static unsigned long int gettimeofday_msec( void )
//{
//    struct timeval tv;
//    do_gettimeofday(&tv);
//    return ((tv.tv_sec*1000) + (tv.tv_usec/1000));
//}

static int64_t SHGEIGER_Gettime(int64_t start_time,int64_t end_time)
{
	int64_t time;

	if (end_time != 0) {
		time = end_time - start_time;
		if(time < 0){
			time += period;
		}
	} else {
		time = 0;	
	}

	DEBUG_LOG("start = %lld,end = %lld,time = %lld",start_time,end_time,time);
	DEBUG_LOG("period = %lld",period);

	return time;
}


static void shsys_hrtimer_msleep(unsigned int msec)
{
    struct timespec tu;

    if(msec >= 1000){
        tu.tv_sec  = msec / 1000;
        tu.tv_nsec = (msec % 1000) * 1000000;
    }else{
    tu.tv_sec = 0;
        tu.tv_nsec = msec * 1000000;
    }
    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
}


static int SHGEIGER_I2cRead(I2cClt *client, char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY; loop_i++) {
		if (i2c_transfer(client->adapter, msgs, 2) > 0) {
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

static int SHGEIGER_I2cWrite(I2cClt *client, uint8_t bRegAdr, uint8_t bData)
{
	int nResult;
	int nI;
	uint8_t bBuff[2];
	struct i2c_msg oMsgs[] =
		{
			[0] =
				{
					.addr	= client->addr,
					.flags	= 0,
					.buf	= (void *)bBuff,
					.len	= 2
				}
		};

	FUNC_LOG();

	bBuff[0] = bRegAdr;
	bBuff[1] = bData;
	for(nI = 0; nI < I2C_RETRY; nI++)
	{
		nResult = i2c_transfer(client->adapter, oMsgs, 1);
		if(nResult == 1)
		{
            DEBUG_LOG("I2cWrite success(%02X,reg:%02X,Data:%02X)=%d", client->addr, bRegAdr, bData, nResult);
			return 0;
		}
	}

	printk(KERN_ERR "I2cWrite: error\n");
	return -1;
}

static int SHGEIGER_PowerON(void)
{
	DEBUG_LOG("PVDDID = %d",pvddid);
	if(pvddid == 1){
		/*PVDD ON*/
		set_pm8058_5v_power_en(PM8058_5V_POWER_EN_GEIGER, 1);
		shsys_hrtimer_msleep(1);
	}
	/*AVDD ON*/
//		vreg_enable(vreg_wlan2);
//		shsys_hrtimer_msleep(1);
	/*EN High*/
	gpio_set_value(SH_GEIGER_EN,1);
	shsys_hrtimer_msleep(100);
	
	return 0;
}




static int SHGEIGER_PowerOFF(void)
{
	gpio_direction_output(SH_GEIGER_EN,0);
	//vreg_disable(vreg_wlan2);
	if(pvddid == 1){
		set_pm8058_5v_power_en(PM8058_5V_POWER_EN_GEIGER, 0);
	}
	return 0;
}

static int SHGEIGER_Change(uint8_t MSB, uint8_t LSB)
{
	int result;

	result = MSB;
	result = ((result << 8) & 0xFF00);
	result = result + LSB;

	DEBUG_LOG("RESULT = %d:%04X", result,result);

	return result;
}

static int SHGEIGER_GetCountData(struct count_data *get_data)
{
	int nResult = 0;
	uint8_t rbuffer[5];

	DEBUG_LOG("GetCountData_MUX_Down_start");
	SHGEIGER_MutexDown();
	DEBUG_LOG("GetCountData_MUX_Down_end");


	rbuffer[0] = NUM_TIME_SLOT_LSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("I2cRead Error");
	}
	get_data->time_slot_lsb  = rbuffer[0];

	rbuffer[0] = NUM_TIME_SLOT_MSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("I2cRead Error");
	}
	get_data->time_slot_msb  = rbuffer[0];

	rbuffer[0] = NUM_ATOM_LSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("I2cRead Error");
	}
	get_data->num_atom_lsb  = rbuffer[0];

	rbuffer[0] = NUM_ATOM_MSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("I2cRead Error");
	}
	get_data->num_atom_msb  = rbuffer[0];

	rbuffer[0] = NUM_DOC_LSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("I2cRead Error");
	}
	get_data->num_dc_lsb  = rbuffer[0];

	rbuffer[0] = NUM_DOC_MSB;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("I2cRead Error");
	}
	get_data->num_dc_msb  = rbuffer[0];

	DEBUG_LOG("GetCountData_MUX_UP_start");
	SHGEIGER_MutexUP();
	DEBUG_LOG("GetCountData_MUX_UP_end");

	return nResult;

}

static int SHGEIGER_BitWrite1(uint8_t addr, uint8_t mask)
{
	int nResult = 0;
	uint8_t rbuffer[5];
	uint8_t rData;
	uint8_t wData;

	FUNC_LOG();

	rbuffer[0] = addr;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("I2cRead Error");
		nResult = -1;
		goto i2c_error;
	}
	rData = rbuffer[0];
	DEBUG_LOG("Read Data:%02Xh",rData);

	wData = rData | mask;
	DEBUG_LOG("Write Data:%02Xh",wData);

	nResult = SHGEIGER_I2cWrite(this_client,addr,wData);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
		nResult = -1;
		goto i2c_error;
	}

	return nResult;

i2c_error:
	return nResult;
}

static int SHGEIGER_BitWrite0(uint8_t addr, uint8_t mask)
{
	int nResult = 0;
	uint8_t rbuffer[5];
	uint8_t rData;
	uint8_t wData;
	uint8_t wmask;

	FUNC_LOG();

	rbuffer[0] = addr;
	nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("I2cRead Error");
		nResult = -1;
		goto i2c_error;
	}
	rData = rbuffer[0];
	DEBUG_LOG("Read Data:%02Xh",rData);

	wmask = (~mask);

	wData = rData & wmask;
	DEBUG_LOG("Write Data:%02Xh",wData);

	nResult = SHGEIGER_I2cWrite(this_client,addr,wData);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
		nResult = -1;
		goto i2c_error;
	}

	return nResult;

i2c_error:
	return nResult;
}

static int SHGEIGER_CountManager( int flag )
{
	int64_t time = 0;
	int nResult = 0;
	
	FUNC_LOG();

	if(atomic_read(&active_mode) == 1){
		/*Start*/
		if(flag == 0){
			if((measure_state & state_mask) == 0x00){
				count_start_time = msm_timer_get_sclk_time(&period);
				if(stop_start_time != 0){
					stop_end_time = msm_timer_get_sclk_time(NULL);
					time = SHGEIGER_Gettime(stop_start_time,stop_end_time);
					total_stop_time += time;
				}
				nResult = SHGEIGER_BitWrite1(COMMAND1,0x10);
				if (nResult != 0){
					DEBUG_LOG("SHGEIGER Start Error");
					nResult = -1;
				}
			}
		}else{	/*Stop*/
			if((measure_state & state_mask) == 0x00){
				stop_start_time = msm_timer_get_sclk_time(&period);
				if(count_start_time != 0){
					count_end_time = msm_timer_get_sclk_time(NULL);
					time = SHGEIGER_Gettime(count_start_time,count_end_time);
					total_count_time += time;
				}
				nResult = SHGEIGER_BitWrite0(COMMAND1,0x10);
				if (nResult != 0){
					DEBUG_LOG("SHGEIGER Stop Error");
					nResult = -1;
				}
			}
		}
	}
	return nResult;
}



static int SHGEIGER_ThermalRead( unsigned int *thermal )
{
	int nResult = 0;
	uint8_t rbuffer[5];
	unsigned int thermaldata = 0;
	
	FUNC_LOG();

	if(atomic_read(&thermalstart_mode) == 1){
		rbuffer[0] = THERMAL_VALUE;
		nResult = SHGEIGER_I2cRead(this_client,rbuffer,1);
		if(nResult != 0){
			DEBUG_LOG("I2cRead Error");
			nResult = -1;
			return nResult;
		}
		thermaldata = rbuffer[0];
		DEBUG_LOG("thermaldata = %d",thermaldata);
	}else{
		nResult = -1;
		return nResult;
	}

	*thermal = thermaldata;

	return nResult;
}

static int SHGEIGER_IrqEnable( void )
{

	enable_irq_wake(this_client->irq);
	enable_irq(this_client->irq);

	return 0;
}

static int SHGIEGER_IrqDisable( void )
{

	disable_irq_wake(this_client->irq);
	disable_irq(this_client->irq);
	
	return 0;
}

void ShGeigerPause_Vib( void )
{
	FUNC_LOG();
}

void ShGeigerReStart_Vib( void )
{
	FUNC_LOG();
}

void ShGeigerPause_Spk( void )
{
	FUNC_LOG();
}

void ShGeigerReStart_Spk( void )
{
	FUNC_LOG();
}

void ShGeigerShutdown_Dtv( void )
{
	FUNC_LOG();
}

void ShGeigerActive_Dtv( void )
{
	FUNC_LOG();
}


static int IOECS_Clear( void )
{
	struct shgeiger_count_data count_data;
	struct count_data copy_data;

	FUNC_LOG();

	SHGEIGER_GetCountData(&copy_data);
	
	count_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
	count_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
	count_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);

	clear_count_data.num_time_slot = count_data.num_time_slot + driver_count_data.num_time_slot;
	clear_count_data.num_atom = count_data.num_atom + driver_count_data.num_atom;
	clear_count_data.num_dc = count_data.num_dc + driver_count_data.num_dc;

	/*Counter clear*/
	total_count_time = 0;
	total_stop_time = 0;

	if((measure_state & state_mask) == 0x00){
		count_start_time = msm_timer_get_sclk_time(&period);
		count_end_time = 0;
		stop_start_time = 0;
		stop_end_time = 0;
	} else {
		stop_start_time = msm_timer_get_sclk_time(&period);
		stop_end_time = 0;
		count_start_time = 0;
		count_end_time = 0;
	}

	return 0;

}

static int IOECS_Start( void )
{
	int nResult = 0;

	FUNC_LOG();

	if((measure_state & 0x80) == 0x80){
		/*ThermalSensor ON*/
		nResult = SHGEIGER_BitWrite1(COMMAND1,0x40);
		if (nResult != 0){
			DEBUG_LOG("SHGEIGER Start Error");
			nResult = -1;
			goto error;
		}

		/*measure_state = 0xxx xxxx*/
		measure_state = (measure_state & 0x7F);
		SHGEIGER_CountManager( 0 );
		
		if(atomic_read(&first_start) == 0){
			if((measure_state & state_mask) == 0x00){
				stop_start_time = msm_timer_get_sclk_time(&period);
			}
			atomic_set(&first_start,1);
		}
		
		if(atomic_read(&thermalstart_mode) == 0){
			atomic_set(&thermalstart_mode,1);
		}
		SHGEIGER_IrqEnable();
	}

	FUNC_LOG_FIN();	
	return nResult;

error:
	return nResult;
}

static int IOECS_Stop( void )
{
	int nResult = 0;

	FUNC_LOG();

	if((measure_state & 0x80) == 0x00){
		/*ThermalSensor OFF*/
		nResult = SHGEIGER_BitWrite0(COMMAND1,0x40);
		if (nResult != 0){
			DEBUG_LOG("SHGEIGER Stop Error");
			nResult = -1;
			goto error;
		}
		
		SHGEIGER_CountManager( 1 );
		/*measure_state = 1xxx xxxx*/
		measure_state = measure_state | 0x80;
		
		if(atomic_read(&thermalstart_mode) == 1){
			atomic_set(&thermalstart_mode,0);
		}
		SHGIEGER_IrqDisable();
	}

	FUNC_LOG_FIN();	
	return nResult;

error:
	return nResult;
}

static int IOECS_Active( void )
{
	int nResult = 0;
	FUNC_LOG();

	if(atomic_read(&active_mode) == 0){
		SHGEIGER_PowerON();

		nResult = SHGEIGER_RegisterInitialize();
		if(nResult < 0){
			DEBUG_LOG("RegisterInitialize Error");
			nResult = -1;
			goto error;
		}

		driver_count_data.num_time_slot = 0;
		driver_count_data.num_atom = 0;
		driver_count_data.num_dc = 0;
		driver_count_data.thermal_data = 0;

		clear_count_data.num_time_slot = 0;
		clear_count_data.num_atom = 0;
		clear_count_data.num_dc = 0;
		clear_count_data.thermal_data = 0;	

		/*Timer Clear*/
		total_count_time = 0;
		total_stop_time = 0;
		count_start_time = 0;
		count_end_time = 0;
		stop_start_time = 0;
		stop_end_time = 0;
		period = 0;
		/*state initialize*/
		measure_state = 0x80;
		atomic_set(&first_start, 0);
		atomic_set(&active_mode, 1);
	}

	FUNC_LOG_FIN();	
	return nResult;
	
error:
	SHGEIGER_PowerOFF();
	return nResult;
}

static int IOECS_Shutdown( void )
{
	FUNC_LOG();

	if(atomic_read(&active_mode) == 1)
	{
		IOECS_Stop();

		if(alarm_wait == ALARM_ALREADY_LOCKED){
			wake_up( &alarmwait_t );
			alarm_wait = ALARM_NOT_LOCKED;
		}
		if(thermal_wait == THERMAL_ALREADY_LOCKED){
			wake_up( &thermalwait_t );
			thermal_wait = THERMAL_NOT_LOCKED;
		}		
		irq_set_irq_type(this_client->irq,IRQ_TYPE_LEVEL_LOW);

		SHGEIGER_PowerOFF();
		
		/*state initialize*/
		measure_state = 0x80;
		atomic_set(&active_mode, 0);
	}

	FUNC_LOG_FIN();	
	return 0;
}

static int IOECS_CalibrationRead( void __user *argp )
{
	int nResult = 0;

	FUNC_LOG();

	nResult = copy_to_user(argp,&driver_calibration_data,sizeof(struct shgeiger_calibration_data));

	FUNC_LOG_FIN();	
	return nResult;
}

static int IOECS_TimeSlot( void __user *argp )
{
	FUNC_LOG();


	FUNC_LOG_FIN();	
	return 0;
}

static int IOECS_ReadCountData( void __user *argp )
{
	struct shgeiger_count_data count_data;
	struct count_data copy_data;
	unsigned int thermal_data;
	int nResult = 0;
	int64_t count_time = 0;
	int64_t stop_time = 0;

	FUNC_LOG();

	SHGEIGER_GetCountData(&copy_data);
	/*Count time*/
	if((measure_state & state_mask) == 0x00){
		count_end_time = msm_timer_get_sclk_time(NULL);
		count_time = SHGEIGER_Gettime(count_start_time,count_end_time);
	}else{
		stop_end_time = msm_timer_get_sclk_time(NULL);
		stop_time = SHGEIGER_Gettime(stop_start_time,stop_end_time);
	}
	count_data.total_time_nsec = total_count_time + count_time;
	count_data.total_stoptime_nsec = total_stop_time + stop_time;

	count_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
	count_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
	count_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);
	SHGEIGER_ThermalRead( &thermal_data );
	count_data.thermal_data = thermal_data;
	DEBUG_LOG("ChipReadData");
	/*Debug*/
	SHGEIGER_DBUGPRINT(&count_data);
	/*Debug end*/
	
	DEBUG_LOG("DriverData");
	/*Debug*/
	SHGEIGER_DBUGPRINT(&driver_count_data);
	/*Debug end*/

	count_data.num_time_slot += driver_count_data.num_time_slot;
	count_data.num_atom += driver_count_data.num_atom;
	count_data.num_dc += driver_count_data.num_dc;

	DEBUG_LOG("ChipReadData + DriverData");
	/*Debug*/
	SHGEIGER_DBUGPRINT(&count_data);
	/*Debug end*/

	DEBUG_LOG("ClearData");
	/*Debug*/
	SHGEIGER_DBUGPRINT(&clear_count_data);
	/*Debug end*/

	count_data.num_time_slot -= clear_count_data.num_time_slot;
	count_data.num_atom -= clear_count_data.num_atom;
	count_data.num_dc -= clear_count_data.num_dc;	
	
	DEBUG_LOG("ChipReadData + DriverData - ClearData");
	/*Debug*/
	SHGEIGER_DBUGPRINT(&count_data);
	/*Debug end*/

	nResult = copy_to_user(argp,&count_data,sizeof(struct shgeiger_count_data));
	if(nResult != 0){
		return nResult;
	}

	FUNC_LOG_FIN();	
	return nResult;
}

static int IOECS_Thermal_Start( void )
{
	int nResult = 0;

	FUNC_LOG();

//	if(atomic_read(&thermalstart_mode) == 0){
//		nResult = SHGEIGER_BitWrite1(COMMAND1,0x40);
//		if (nResult != 0){
//			DEBUG_LOG("SHGEIGER ThermalStart Error");
//			nResult = -1;
//			goto error;
//		}
//		atomic_set(&thermalstart_mode, 1);
//	}

	FUNC_LOG_FIN();	
	return nResult;

//error:
//	return nResult;
}

static int IOECS_Thermal_Stop( void )
{
	int nResult = 0;

	FUNC_LOG();

//	if(atomic_read(&thermalstart_mode) == 1){
//		nResult = SHGEIGER_BitWrite0(COMMAND1,0x40);
//		if (nResult != 0){
//			DEBUG_LOG("SHGEIGER ThermalStop Error");
//			nResult = -1;
//			goto error;
//		}
//		atomic_set(&thermalstart_mode, 0);
//	}

	FUNC_LOG_FIN();	
	return nResult;

//error:
//	return nResult;
}

static int IOECS_ThermalRead( void __user *argp )
{
	unsigned int thermal;
	int nResult = 0;
	
	FUNC_LOG();

	SHGEIGER_ThermalRead( &thermal );

	nResult = copy_to_user(argp, &thermal, sizeof(thermal));
	if(nResult != 0){
		return -1;
	}

	return nResult;

}

static int IOECS_CountThreshold( void __user *argp )
{
	int nResult = 0;
	unsigned int user_threshold;
	struct count_data copy_data;
	struct shgeiger_count_data geiger_copy_data;

	FUNC_LOG();

	nResult = copy_from_user(&user_threshold,argp,sizeof(user_threshold));
	if(nResult != 0){
		return -1;
	}
	if(user_threshold > 65535 || user_threshold == 0 ){
		user_threshold = 65535;
	}

	/* Chip data is hold and Chip data clear  */
	SHGEIGER_GetCountData(&copy_data);
	geiger_copy_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
	geiger_copy_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
	geiger_copy_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);

	driver_count_data.num_time_slot += geiger_copy_data.num_time_slot;
	driver_count_data.num_atom += geiger_copy_data.num_atom;
	driver_count_data.num_dc += geiger_copy_data.num_dc;

	nResult = SHGEIGER_BitWrite1(COUNT_CLEAL,0x01);
	if (nResult != 0){
		DEBUG_LOG("IOECS_CountThreshold Error");
	}
	nResult = SHGEIGER_BitWrite0(COUNT_CLEAL,0x01);
	if (nResult != 0){
			DEBUG_LOG("IOECS_CountThreshold Error");
	}
	/**************************************/
	
	/*Count Threshold Setting*/
	DEBUG_LOG("CountThreshold = %d",user_threshold);
	atom_threshold = user_threshold;
	atom_threshold_LSB = (user_threshold & 0x00FF);
	atom_threshold_MSB = ((user_threshold >> 8) & 0x00FF);
	DEBUG_LOG("CountThreshold MSB = %d,LSB = %d",atom_threshold_MSB,atom_threshold_LSB);

	nResult = SHGEIGER_I2cWrite(this_client,ATOM_THRESHOLD_LSB,atom_threshold_LSB);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
	}
	nResult = SHGEIGER_I2cWrite(this_client,ATOM_THRESHOLD_MSB,atom_threshold_MSB);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
	}
	/*******************************************/

	return nResult;

}

static int IOECS_ThermalThreshold( void __user *argp )
{
	int nResult = 0;

	FUNC_LOG();

	return nResult;

}

static int IOECS_ChipVerRead( void __user *argp )
{
	int nResult = 0;

	FUNC_LOG();

	if(atomic_read(&chipver_enable) == 1){
		nResult = copy_to_user(argp, &chipver, sizeof(chipver));
		if(nResult != 0){
			nResult = -1;
		}
	}else{
		nResult = -1;
	}

	return nResult;

}

static int IOECS_AlarmWait( void __user *argp )
{
	int nResult = 0;

	FUNC_LOG();

	if(alarm_wait == ALARM_ALREADY_LOCKED){
		nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
		return 0;
	}

	alarm_wait = ALARM_ALREADY_LOCKED;

	DEBUG_LOG("Alarm Lock Start");
	wait_event(alarmwait_t,(alarm_wait != ALARM_ALREADY_LOCKED));
	DEBUG_LOG("Alarm Lock End");

	nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
	if(nResult != 0){
		DEBUG_LOG("copy_to_user Error");
		alarm_wait = ALARM_NOT_LOCKED;
		return -1;
	}
	alarm_wait = ALARM_NOT_LOCKED;
	FUNC_LOG_FIN();
	return 0;

}

static int IOECS_AlarmWaitUnlock( void __user *argp )
{
	int nResult = 0;
	FUNC_LOG();
	
	if(alarm_wait == ALARM_NOT_LOCKED){
		nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
		return -1;
	}
	
	alarm_wait = ALARM_UNLOCK_SUCCESS;
	wake_up( &alarmwait_t );
	nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
	if(nResult != 0){
		DEBUG_LOG("copy_to_user Error");
		return -1;
	}
	FUNC_LOG_FIN();
	return 0;
}

static int IOECS_ThermalWait( void __user *argp )
{
	int nResult = 0;
	FUNC_LOG();

	if(thermal_wait == THERMAL_ALREADY_LOCKED){
		nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
		return 0;
	}

	thermal_wait =  THERMAL_ALREADY_LOCKED;

	DEBUG_LOG("Thermal Lock Start");
	wait_event(thermalwait_t,(thermal_wait != THERMAL_ALREADY_LOCKED));
	DEBUG_LOG("Thermal Lock End");

	nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
	if(nResult != 0){
		DEBUG_LOG("copy_to_user Error");
		thermal_wait = THERMAL_NOT_LOCKED;
		return -1;
	}
	thermal_wait = THERMAL_NOT_LOCKED;

	FUNC_LOG_FIN();
	return 0;

}

static int IOECS_ThermalWaitUnlock( void __user *argp )
{
	int nResult = 0;
	FUNC_LOG();
	
	if(thermal_wait == THERMAL_NOT_LOCKED){
		nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
		return -1;
	}

	thermal_wait = THERMAL_UNLOCK_SUCCESS;
	wake_up( &thermalwait_t );
	nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
	if(nResult != 0){
		DEBUG_LOG("copy_to_user Error");
		return -1;
	}

	FUNC_LOG_FIN();
	return 0;
}

static int IOECS_GMBIST( void __user *argp )
{
	int nResult;
	unsigned int atom = 0;
	struct count_data copy_data;

	FUNC_LOG();
	
	/*TimeSlot:1ms(0x08h:0x01)*/
	nResult = SHGEIGER_I2cWrite(this_client,SLOT_TIME,0x01);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
		nResult = -1;
		goto error;
	}

	/*BYPASS Mode(0x02h:0x77)*/
	nResult = SHGEIGER_I2cWrite(this_client,COMMAND1,0x77);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
		nResult = -1;
		goto error;
	}

	/*Chip data clear*/
	nResult = SHGEIGER_BitWrite1(COUNT_CLEAL,0x01);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
	}
	nResult = SHGEIGER_BitWrite0(COUNT_CLEAL,0x01);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
	}

	/*Comparison threshold(0x0D:0x7F)*/
	nResult = SHGEIGER_I2cWrite(this_client,0x0D,0x7F);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
		nResult = -1;
		goto error;
	}

	/*Test Start(0x03:0x80)*/
	nResult = SHGEIGER_I2cWrite(this_client,0x03,0x80);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
		nResult = -1;
		goto error;
	}
	/*Wait(1s)*/
	shsys_hrtimer_msleep(1000);

	/*Atom Read*/
	nResult = SHGEIGER_GetCountData(&copy_data);
	atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
	if (nResult != 0){
		DEBUG_LOG("Atom Read Failed");
		atom = 0;
		nResult = -1;
		goto error;
	}

	nResult = copy_to_user(argp, &atom, sizeof(atom));
	if(nResult != 0){
		nResult = -1;
		DEBUG_LOG("copy to user Failed");
		goto error;
	}

	/*Chip data clear*/
	nResult = SHGEIGER_BitWrite1(COUNT_CLEAL,0x01);
	if (nResult != 0){
		DEBUG_LOG("I2cWrite Error");
	}

	/*Register Initialize*/
	nResult = SHGEIGER_RegisterInitialize();
	if(nResult < 0){
		DEBUG_LOG("RegisterInitialize Error");
	}
	
	return nResult;

error:
	return nResult;	
}



static int IOECS_AlarmWaitUnlockCmd( void __user *argp )
{
	int nResult = 0;
	FUNC_LOG();
	
	if(alarm_wait == ALARM_NOT_LOCKED){
		nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
		return -1;
	}
	
	alarm_wait = ALARM_UNLOCK;
	wake_up( &alarmwait_t );
	nResult = copy_to_user(argp, &alarm_wait, sizeof(alarm_wait));
	if(nResult != 0){
		DEBUG_LOG("copy_to_user Error");
		return -1;
	}
	FUNC_LOG_FIN();
	return alarm_wait;
}

static int IOECS_ThermalHighUnlockCmd( void __user *argp )
{
	int nResult = 0;
	FUNC_LOG();
	
	if(thermal_wait == THERMAL_NOT_LOCKED){
		nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
		return -1;
	}

	thermal_wait = THERMAL_HIGH_UNLOCK;
	wake_up( &thermalwait_t );
	nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
	if(nResult != 0){
		DEBUG_LOG("copy_to_user Error");
		return -1;
	}

	FUNC_LOG_FIN();
	return thermal_wait;
}

static int IOECS_ThermalNolmalUnlockCmd( void __user *argp )
{
	int nResult = 0;
	FUNC_LOG();
	
	if(thermal_wait == THERMAL_NOT_LOCKED){
		nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
		return -1;
	}

	thermal_wait = THERMAL_NOLMAL_UNLOCK;
	wake_up( &thermalwait_t );
	nResult = copy_to_user(argp, &thermal_wait, sizeof(thermal_wait));
	if(nResult != 0){
		DEBUG_LOG("copy_to_user Error");
		return -1;
	}

	FUNC_LOG_FIN();
	return thermal_wait;
}



static int SHGEIGER_open(struct inode *inode, struct file *filp)
{
	FUNC_LOG();

	atomic_set(&open_flag, 1);
		
	return 0;
}

static int SHGEIGER_release(struct inode *inode, struct file *filp)
{
	FUNC_LOG();

	atomic_set(&open_flag, 0);

	return 0;
}

static long SHGEIGER_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user*)arg;

	FUNC_LOG();

	switch (cmd) {
		case SHGEIGER_IOCTL_ACTIVE:
			DEBUG_LOG("SHGEIGER_IOCTL_ACTIVE");
			if(IOECS_Active() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_SHUTDOWN:
			DEBUG_LOG("SHGEIGER_IOCTL_SHUTDOWN");
			if(IOECS_Shutdown() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_START:
			DEBUG_LOG("SHGEIGER_IOCTL_START");
			if(IOECS_Start() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_STOP:
			DEBUG_LOG("SHGEIGER_IOCTL_STOP");
			if(IOECS_Stop() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_CALIBRATIONREAD:
			DEBUG_LOG("SHGEIGER_IOCTL_CALIBRATIONREAD");
			if(IOECS_CalibrationRead( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_TIMESLOT:
			DEBUG_LOG("SHGEIGER_IOCTL_TIMESLOT");
			if(IOECS_TimeSlot( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_READCOUNTDATA:
			DEBUG_LOG("SHGEIGER_IOCTL_READCOUNTDATA");
			if(IOECS_ReadCountData( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_CLEAR:
			DEBUG_LOG("SHGEIGER_IOCTL_CLEAR");
			if(IOECS_Clear() < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMAL_START:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMAL_START");
			if(IOECS_Thermal_Start( ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMAL_STOP:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMAL_STOP");
			if(IOECS_Thermal_Stop( ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALREAD:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALREAD");
			if(IOECS_ThermalRead( argp ) < 0)
			{
				return -EIO;
			}
			break;		
		case SHGEIGER_IOCTL_COUNTTHRESHOLD:
			DEBUG_LOG("SHGEIGER_IOCTL_COUNTTHRESHOLD");
			if(IOECS_CountThreshold( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALTHRESHOLD:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALTHRESHOLD");
			if(IOECS_ThermalThreshold( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_CHIPVERREAD:
			DEBUG_LOG("SHGEIGER_IOCTL_CHIPVERREAD");
			if(IOECS_ChipVerRead( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_ALARMWAIT:
			DEBUG_LOG("SHGEIGER_IOCTL_ALARMWAIT");
			if(IOECS_AlarmWait( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_ALARMWAIT_UNLOCK:
			DEBUG_LOG("SHGEIGER_IOCTL_ALARMWAIT_UNLOCK");
			if(IOECS_AlarmWaitUnlock( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALWAIT:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALWAIT");
			if(IOECS_ThermalWait( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALWAIT_UNLOCK:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALWAIT_UNLOCK");
			if(IOECS_ThermalWaitUnlock( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_GMBIST:
			DEBUG_LOG("SHGEIGER_IOCTL_GMBIST");
			if(IOECS_GMBIST( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALHIGHCMD:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALHIGHCMD");
			if(IOECS_ThermalHighUnlockCmd( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_THERMALNOLMALCMD:
			DEBUG_LOG("SHGEIGER_IOCTL_THERMALNOLMALUNLOCKCMD");
			if(IOECS_ThermalNolmalUnlockCmd( argp ) < 0)
			{
				return -EIO;
			}
			break;
		case SHGEIGER_IOCTL_ALARMWAITCMD:
			DEBUG_LOG("SHGEIGER_IOCTL_ALARMWAITCMD");
			if(IOECS_AlarmWaitUnlockCmd( argp ) < 0)
			{
				return -EIO;
			}			
			break;
		default:
			break;
	}
	return 0;
}

static struct file_operations SHGEIGER_ctl_fops = {
	.owner		= THIS_MODULE,
	.open		= SHGEIGER_open,
	.release	= SHGEIGER_release,
	.unlocked_ioctl		= SHGEIGER_ioctl,
};

static struct miscdevice SHGEIGER_device = {
 .minor = MISC_DYNAMIC_MINOR,
	.name = "shgeiger_dev",
	.fops = &SHGEIGER_ctl_fops,
};

static irqreturn_t SHGEIGER_interrupt(int irq, void *dev_id)
{
	drv_data *poShGeigerRec = dev_id;

	FUNC_LOG();

	disable_irq_wake(this_client->irq);
	disable_irq_nosync(this_client->irq);
	schedule_work(&poShGeigerRec->IrqWork);
	
	return IRQ_HANDLED;

}

static void SHGEIGER_Irq_workfunc(WorkStruct *work)
{
	int nResult = 0;
	struct count_data copy_data;
	struct shgeiger_count_data geiger_copy_data;
	unsigned int thermal_data;

	FUNC_LOG();

	/* Chip data is hold.  */
	SHGEIGER_GetCountData(&copy_data);
	geiger_copy_data.num_time_slot = SHGEIGER_Change(copy_data.time_slot_msb,copy_data.time_slot_lsb);
	geiger_copy_data.num_atom = SHGEIGER_Change(copy_data.num_atom_msb,copy_data.num_atom_lsb);
	geiger_copy_data.num_dc = SHGEIGER_Change(copy_data.num_dc_msb,copy_data.num_dc_lsb);
	SHGEIGER_ThermalRead( &thermal_data );
	geiger_copy_data.thermal_data = thermal_data;

	//driver_count_data.num_time_slot += geiger_copy_data.num_time_slot;
	//driver_count_data.num_atom += geiger_copy_data.num_atom;
	//driver_count_data.num_dc += geiger_copy_data.num_dc;
	/*********************/

	/*AlarmWait*/
	if(geiger_copy_data.num_atom >= atom_threshold){
		DEBUG_LOG("Count Over");
		driver_count_data.num_time_slot += geiger_copy_data.num_time_slot;
		driver_count_data.num_atom += geiger_copy_data.num_atom;
		driver_count_data.num_dc += geiger_copy_data.num_dc;

		/*Chip data clear*/
		nResult = SHGEIGER_BitWrite1(COUNT_CLEAL,0x01);
		if (nResult != 0){
			DEBUG_LOG("SHGEIGER_Irq_workfunc Error");
		}
		SHGEIGER_IrqEnable();
		nResult = SHGEIGER_BitWrite0(COUNT_CLEAL,0x01);
		if (nResult != 0){
			DEBUG_LOG("SHGEIGER_Irq_workfunc Error");
		}
		/*Chip data clear End*/
		
		if(alarm_wait == ALARM_ALREADY_LOCKED){
			alarm_wait = ALARM_UNLOCK;
			wake_up( &alarmwait_t );
		}
		return;
	}
	
	/*TimeSlot Max*/
	if(geiger_copy_data.num_time_slot >= 65535){
		DEBUG_LOG("TimeSlot Max");
		
		driver_count_data.num_time_slot += geiger_copy_data.num_time_slot;
		driver_count_data.num_atom += geiger_copy_data.num_atom;
		driver_count_data.num_dc += geiger_copy_data.num_dc;
		
		/*Chip data clear*/
		nResult = SHGEIGER_BitWrite1(COUNT_CLEAL,0x01);
		if (nResult != 0){
			DEBUG_LOG("SHGEIGER_Irq_workfunc Error");
		}
		SHGEIGER_IrqEnable();
		nResult = SHGEIGER_BitWrite0(COUNT_CLEAL,0x01);
		if (nResult != 0){
			DEBUG_LOG("SHGEIGER_Irq_workfunc Error");
		}
		/*Chip data clear End*/
		return;
	}

	/*thermal wait*/
	if(geiger_copy_data.thermal_data >= Threshold_high){
		DEBUG_LOG("Thermal HIGH");
		irq_set_irq_type(this_client->irq,IRQ_TYPE_LEVEL_HIGH);
		/**Count Pause**/
		if((measure_state & 0x02) == 0x00){
			SHGEIGER_CountManager( 1 );
			/*measure_state = xxxx xx1x*/
			measure_state = (measure_state | 0x02);
		}
		/*********************/
		if(thermal_wait == THERMAL_ALREADY_LOCKED){
			thermal_wait = THERMAL_HIGH_UNLOCK;
			wake_up( &thermalwait_t );
		}
		SHGEIGER_IrqEnable();
		return;
	}
	if(geiger_copy_data.thermal_data <= Threshold_low){
		DEBUG_LOG("Thermal Normal");
		irq_set_irq_type(this_client->irq,IRQ_TYPE_LEVEL_LOW);
		/**Count ReStart**/
		if((measure_state & 0x02) == 0x02){
			/*measure_state = xxxx xx0x*/
			measure_state = (measure_state & 0xFD);
			SHGEIGER_CountManager( 0 );
		}
		/*****************/
		if(thermal_wait == THERMAL_ALREADY_LOCKED){
			thermal_wait = THERMAL_NOLMAL_UNLOCK;
			wake_up( &thermalwait_t );
		}
		SHGEIGER_IrqEnable();
		return;
	}
	
//	/*Chip data clear*/
//	nResult = SHGEIGER_BitWrite1(COUNT_CLEAL,0x01);
//	if (nResult != 0){
//		DEBUG_LOG("SHGEIGER_Irq_workfunc Error");
//	}
//
//	enable_irq_wake(this_client->irq);
//	enable_irq(this_client->irq);
//	nResult = SHGEIGER_BitWrite0(COUNT_CLEAL,0x01);
//	if (nResult != 0){
//		DEBUG_LOG("SHGEIGER_Irq_workfunc Error");
//	}
//	/*Chip data clear End*/
	
//	FUNC_LOG_FIN();
}


static int SHGEIGER_ReleaseGPIO(drv_data *poShGeigerRec)
{
	FUNC_LOG();

	gpio_free(poShGeigerRec->irq_gpio);

	return 0;
}

static int SHGEIGER_ConfigGPIO(drv_data *poShGeigerRec)
{
	FUNC_LOG();
	
	gpio_tlmm_config(GPIO_CFG(SH_GEIGER_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), GPIO_CFG_ENABLE);

	poShGeigerRec->irq_gpio = 147;
	this_client->irq = MSM_GPIO_TO_INT(poShGeigerRec->irq_gpio);

	return 0;
}

static int SHGEIGER_RegisterInitialize( void )
{
	int nResult =0;
	uint8_t adder = 0x01;
	int i = 0;

	FUNC_LOG();

	if( chipver == 0xFF ){	/*ES1*/
		ragdata_es1[5] = Threshold_high;
		ragdata_es1[6] = Threshold_low;
		for(i=0;i < REGISTER_MAX;i++){
			nResult = SHGEIGER_I2cWrite(this_client,adder,ragdata_es1[i]);
			if (nResult != 0){
				DEBUG_LOG("I2cWrite Error");
				nResult = -1;
				goto i2c_error;
			}
			adder++;
		}
	}else if( chipver == 0x01 ){	/*PP1*/
		ragdata_pp1[5] = Threshold_high;
		ragdata_pp1[6] = Threshold_low;
		for(i=0;i < REGISTER_MAX;i++){
			nResult = SHGEIGER_I2cWrite(this_client,adder,ragdata_pp1[i]);
			if (nResult != 0){
				DEBUG_LOG("I2cWrite Error");
				nResult = -1;
				goto i2c_error;
			}
			adder++;
		}
	}else if( chipver == 0x02 ){	/*PP2*/
		ragdata_pp2[5] = Threshold_high;
		ragdata_pp2[6] = Threshold_low;		
		for(i=0;i < REGISTER_MAX;i++){
			nResult = SHGEIGER_I2cWrite(this_client,adder,ragdata_pp2[i]);
			if (nResult != 0){
				DEBUG_LOG("I2cWrite Error");
				nResult = -1;
				goto i2c_error;
			}
			adder++;
		}
	}else{
		for(i=0;i < REGISTER_MAX;i++){
			ragdata_other[5] = Threshold_high;
			ragdata_other[6] = Threshold_low;				
			nResult = SHGEIGER_I2cWrite(this_client,adder,ragdata_other[i]);
			if (nResult != 0){
				DEBUG_LOG("I2cWrite Error");
				nResult = -1;
				goto i2c_error;
			}
			adder++;
		}
	}

i2c_error:
	FUNC_LOG_FIN();
	return nResult;
}

static int SHGEIGER_CalilbrationRead( void )
{
	int nResult = 0;
	uint8_t rbuffer[5];
	uint8_t multi_lsb;
	uint8_t multi_msb;

	FUNC_LOG();

	SHGEIGER_PowerON();

	rbuffer[0] = BG_LSB;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("EEPROMRead Error");
		nResult = -1;
		goto i2c_error;
	}
	DEBUG_LOG("BG_LSB = 0x%02X",rbuffer[0]);
	driver_calibration_data.bg_lsb = rbuffer[0];

	rbuffer[0] = BG_MSB;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("EEPROMRead Error");
		nResult = -1;
		goto i2c_error;
	}
	DEBUG_LOG("BG_MSB = 0x%02X",rbuffer[0]);
	driver_calibration_data.bg_msb = rbuffer[0];

	rbuffer[0] = MULTI_LSB;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("EEPROMRead Error");
		nResult = -1;
		goto i2c_error;
	}
	DEBUG_LOG("MULTI_LSB = 0x%02X",rbuffer[0]);
	multi_lsb = rbuffer[0];

	rbuffer[0] = MULTI_MSB;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("EEPROMRead Error");
		nResult = -1;
		goto i2c_error;
	}
	DEBUG_LOG("MULTI_MSB = 0x%02X",rbuffer[0]);
	multi_msb = rbuffer[0];

	rbuffer[0] = MDULE_VER;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("EEPROMRead Error");
		atomic_set(&chipver_enable,0);
		nResult = -1;
		goto i2c_error;
	}
	DEBUG_LOG("MDULE_VER = 0x%02X",rbuffer[0]);
	chipver = rbuffer[0];

	rbuffer[0] = THERMAL_A;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("EEPROMRead Error");
		nResult = -1;
		goto i2c_error;
	}
	DEBUG_LOG("THERMAL_A = 0x%02X",rbuffer[0]);
	driver_calibration_data.thermal_a = rbuffer[0];

	rbuffer[0] = THERMAL_B;
	nResult = SHGEIGER_EEPROM_I2cRead(rbuffer,1);
	if(nResult != 0){
		DEBUG_LOG("EEPROMRead Error");
		nResult = -1;
		goto i2c_error;
	}
	DEBUG_LOG("THERMAL_B = 0x%02X",rbuffer[0]);
	driver_calibration_data.thermal_b = rbuffer[0];
	
	
i2c_error:
	/*Calibration_default_data*/
	driver_calibration_data.bg = 0.0;
	driver_calibration_data.multi = 0.14343;
	driver_calibration_data.multi_data = SHGEIGER_Change(multi_msb,multi_lsb);
	SHGEIGER_PowerOFF();
	return nResult;
}

/*Undecided*/
static int SHGEIGER_ThermalThreshold( void )
{
	int nResult = 0;

	FUNC_LOG();

	/*default*/
	Threshold_high = ((thermal_threshold_HIGH+driver_calibration_data.thermal_b
)*128)/driver_calibration_data.thermal_a;

	Threshold_low = ((thermal_threshold_LOW+driver_calibration_data.thermal_b
)*128)/driver_calibration_data.thermal_a;

	DEBUG_LOG("THERMAL_HIGH : %d,0x%02X", Threshold_high,Threshold_high);
	DEBUG_LOG("THERMAL_LOW : %d,0x%02X", Threshold_low,Threshold_low);
	
//	nResult = SHGEIGER_I2cWrite(this_client,THERMAL_THRESHOLD_HITH,Threshold_high);
//	if (nResult != 0){
//		DEBUG_LOG("I2cWrite Error");
//		nResult = -1;
//		goto i2c_error;
//	}
	
//	nResult = SHGEIGER_I2cWrite(this_client,THERMAL_THRESHOLD_LOW,Threshold_low);
//	if (nResult != 0){
//		DEBUG_LOG("I2cWrite Error");
//		nResult = -1;
//		goto i2c_error;
//	}
	/*****/

//i2c_error:
	return nResult;
}


static int SHGEIGER_Remove(I2cClt *client)
{
	drv_data *poShGeigerRec = i2c_get_clientdata(client);

	FUNC_LOG();

	DEBUG_LOG("Remove()");

	dev_info(&client->dev, "removing driver\n");
	device_init_wakeup(&client->dev, 0);
	SHGEIGER_ReleaseGPIO(poShGeigerRec);
	kfree(poShGeigerRec);
	return 0;	
}

static const I2cDevID gI2cDevIdTableAcc[] =
{
   { SH_GEIGER_I2C_DEVNAME, 0 },
   { }
};

MODULE_DEVICE_TABLE(i2c, gI2cDevIdTableAcc);

static struct i2c_driver goI2cAccDriver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name  = SH_GEIGER_I2C_DEVNAME,
	},
	.probe	  = SHGEIGER_Probe,
	.remove	  = SHGEIGER_Remove,
	.id_table = gI2cDevIdTableAcc,
};

static int SHGEIGER_Probe(I2cClt *client, const I2cDevID *poDevId)
{
	drv_data *poShGeigerRec = NULL;
	int nResult;

	FUNC_LOG();

	DEBUG_LOG("ini_MUX_start");
	sema_init(&geiger_mutex,1);
    DEBUG_LOG("ini_MUX_end");

	/* CLIENT CHECKING */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DEBUG_LOG("check_functionality failed.");
		nResult = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	poShGeigerRec = kzalloc(sizeof(drv_data), GFP_KERNEL);
	if (!poShGeigerRec){
		DEBUG_LOG("memory allocation failed.");
		nResult = -ENOMEM;
		goto exit1;
	}

	INIT_WORK(&poShGeigerRec->IrqWork, SHGEIGER_Irq_workfunc);
	i2c_set_clientdata(client, poShGeigerRec);

	/* Copy to global variable */
	this_client = client;

	/* GPIO setting */
	nResult = SHGEIGER_ConfigGPIO(poShGeigerRec);
	if(nResult < 0)
	{
		DEBUG_LOG("ConfigGPIO is Err.");
		goto exit2;
	}

//	vreg_wlan2 = vreg_get(NULL, "wlan2");
//	vreg_set_level(vreg_wlan2, 2750);

	/*PVDDID Check*/
//	gpio_direction_input(SH_GEIGER_PVDDID);
	nResult = gpio_tlmm_config(GPIO_CFG(SH_GEIGER_PVDDID, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(nResult == 0){
		pvddid = gpio_get_value(SH_GEIGER_PVDDID);
	}
	nResult = gpio_tlmm_config(GPIO_CFG(SH_GEIGER_PVDDID, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(nResult < 0){
		DEBUG_LOG("pvdd gpio config failed.");
	}

	/*CalibrationData Read*/
	nResult = SHGEIGER_CalilbrationRead();
	if(nResult < 0){
		DEBUG_LOG("SHGEIGER_CalilbrationRead Error");
		nResult = -1;
		goto exit2;
	}

	/*ThermalThreshold Setting*/
	nResult = SHGEIGER_ThermalThreshold();
	if(nResult < 0){
		DEBUG_LOG("SHGEIGER_ThermalThreshold Error");
		nResult = -1;
		goto exit2;
	}

	nResult = misc_register(&SHGEIGER_device);
	if (nResult)
	{
		DEBUG_LOG("misc_register failed.");
		goto exit4;
	}

	/* wakeup set */
	irq_set_irq_type(client->irq, IRQ_TYPE_LEVEL_LOW);

	/* irq will not be enabled on request irq */
	irq_set_status_flags(client->irq, IRQ_NOAUTOEN);

	nResult = request_irq(client->irq, SHGEIGER_interrupt, IRQF_TRIGGER_LOW | IRQF_DISABLED,
					  "GEIGER_INT", poShGeigerRec);
	if (nResult < 0) {
		DEBUG_LOG("request irq failed.");
		goto exit5;
	}

	init_waitqueue_head( &alarmwait_t );
	init_waitqueue_head( &thermalwait_t );

	return 0;

exit5:
	misc_deregister(&SHGEIGER_device);
exit4:
exit2:
	kfree(poShGeigerRec);
exit1:
exit0:
	return nResult;

}

static int __init SHGEIGER_Init(void)
{
	FUNC_LOG();

	/* I2C driver Use beginning */
	return i2c_add_driver(&goI2cAccDriver);

	return 0;
}

static void __exit SHGEIGER_Exit(void)
{
	FUNC_LOG();

	/* I2C driver Use end */
	i2c_del_driver(&goI2cAccDriver);
}


module_init(SHGEIGER_Init);
module_exit(SHGEIGER_Exit);


MODULE_DESCRIPTION("shgeiger sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
