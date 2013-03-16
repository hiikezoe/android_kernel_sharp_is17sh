/* shtps_tma_spi.c
 *
 * Copyright (c) 2010, Sharp. All rights reserved.
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

#define	TPS_SETPARAM			/* Firm parameter setting */
#define	TPS_PRNERR				/* Log output(Error Log) */
/* #define	TPS_PRNLOG */		/* Log output(Normal Log) */
/* #define	TPS_PRNDEB */		/* Log output(Debug Log) */
/* #define	TPS_EVENTLOG */		/* Log output(event Log) */
#define	TPS_SETCALIBRATION
#define	TPS_TPCT_COMMAND		/* Diag TPCT command */
/* #define	TPS_ISSP_ENABLE */
/* #define	TPS_STBY_SOFTCONTROL */

#if defined( CONFIG_SHTPS_TMA3XX_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_TMA3XX_FACETOUCH_OFF_DETECT )
	#define	TPS_IRQ_WAKE		/* Wake Up IRQ setting */
	#define	FACETOUCH_DETECT_ENABLE	/* facetouch Detect On */
#endif	/* defined( CONFIG_SHTPS_TMA3XX_FACETOUCH_DETECT ) || defined( CONFIG_SHTPS_TMA3XX_FACETOUCH_OFF_DETECT ) */

/*+-------------------------------------------------------------------------+*/
/*|	Include files															|*/
/*+-------------------------------------------------------------------------+*/
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <mach/gpio.h>
#include <sharp/shtps_dev.h>
//#include <sharp/shlcdc_kerl.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#ifdef TPS_TPCT_COMMAND
#include <linux/poll.h>
#endif	/* TPS_TPCT_COMMAND */

#include <linux/spi/spi.h>
#include <sharp/sh_smem.h>		/* HW Rev */

#include <linux/jiffies.h>
#include <linux/hrtimer.h>
#ifdef CONFIG_USB_SWIC
#include <sharp/shswic_kerl.h>
#endif /* CONFIG_USB_SWIC */
#ifdef FACETOUCH_DETECT_ENABLE
#include <linux/wakelock.h>
#endif	/* FACETOUCH_DETECT_ENABLE */

/*+-------------------------------------------------------------------------+*/
/*|	Constant declaration													|*/
/*+-------------------------------------------------------------------------+*/

#define	KPD_KEYPRESS			1
#define	KPD_KEYRELEASE			0

/* Command */
enum
{
	QPHYSLEN			= 128,
	QCVENDOR_ID			= 0x5143,
	QCPRODUCT_ID		= 2,
	QCVERSION_ID		= 1
};

#ifdef TPS_IRQ_WAKE
enum{
	TPS_IRQ_WAKE_DISABLE,
	TPS_IRQ_WAKE_ENABLE,
};
#endif	/* TPS_IRQ_WAKE */

#define	INITDELAY_TIMER			600
#define	RECOVER_TIMER			1000
#ifdef TPS_TPCT_COMMAND
#define DIAGPOLL_TIME			100
#endif	/* TPS_TPCT_COMMAND */

#define	RETRY_MAX			3
#define	RETRY_WAIT			15

typedef enum
{
	TPS_STATE_HOVER = 0,
	TPS_STATE_DOWN,
	TPS_STATE_MAX,
} TpsState;

/* Adjustment Parameters */
#define	POS_X0				0
#define	POS_X1				135
#define	POS_X2				405
#define	POS_X3				539
#define	POS_Y0				0
#define	POS_Y1				185
#define	POS_Y2				515
#define	POS_Y3				844
#define	POS_Y4				1030
#define	POS_LIMIT			100

#define	ADJUST_POINT		6					/* Number of Adjustment points */
#define	AREA_COUNT			(ADJUST_POINT*2)	/* Number of Adjustment Area */
#define DOUBLE_ACCURACY		10000

#define	TPS_DISABLE_FLIP	0x01
#define	TPS_DISABLE_SLEEP	0x02
#define	TPS_DISABLE_API		0x04

#define	TPS_DISABLE_ON		0xFF
#define	TPS_DISABLE_OFF		0x00

#define	TPS_CHECK_ON		0x01
#define	TPS_CHECK_OFF		0x00

#define	TPS_RETURN_ON		0x01
#define	TPS_RETURN_OFF		0x00

#define	TPS_RESET_INTERVAL	2000

#define	TPS_CALIB_ON		0x01
#define	TPS_CALIB_OFF		0x00
#define	TPS_CALIB_WAIT		1400
#define	TPS_CALIB_MIN		70
#define	TPS_CALIB_MAX		150

#define TPS_DRAG_THRESH_VAL_1ST_VALUE		3
#define TPS_DRAG_THRESH_VAL_1ST			TPS_DRAG_THRESH_VAL_1ST_VALUE
#define TPS_DRAG_THRESH_VAL_2ND			1
#define TPS_DRAG_THRESH_VAL_1ST_MULTI		8
#define TPS_DRAG_THRESH_VAL_2ND_MULTI		1
#define TPS_DRAG_THRESH_RETURN_TIME		250
#define TPS_CHECK_POLL_INTERVAL			500		/* (500ms) */

#define	TPS_TESTMODE_START_WAIT	150

#define TPS_SENSOR_SIZE		(SHTPS_TMA_TXNUM_MAX*SHTPS_TMA_RXNUM_MAX)
#define TPS_SENSOR_READ_VAL	(SHTPS_TMA_TXNUM_MAX*SHTPS_TMA_RXNUM_MAX)+SHTPS_TMA_OFFSET_MAX

#ifdef TPS_ISSP_ENABLE
	#define	SH_TOUCH_SPI_HSSP_CLK	SH_TOUCH_SPI_MOSI
	#define	SH_TOUCH_SPI_HSSP_DATA	SH_TOUCH_SPI_CLK
#else
	#define	SH_TOUCH_SPI_HSSP_CLK	SH_TOUCH_HSSP_CLK
	#define	SH_TOUCH_SPI_HSSP_DATA	SH_TOUCH_HSSP_DATA
#endif	/* TPS_ISSP_ENABLE */

#ifdef FACETOUCH_DETECT_ENABLE
	#define SHTPS_FINGER_WIDTH_PALMDET			255
	#define SHTPS_FINGER_WIDTH_MIN				1
	#define SHTPS_FINGER_WIDTH_MAX				128
#endif	/* FACETOUCH_DETECT_ENABLE */

/*+-------------------------------------------------------------------------+*/
/*|	Type declaration														|*/
/*+-------------------------------------------------------------------------+*/
typedef struct spitps_record	SpiTpsRec;
typedef struct spi_device       SpiDev;

typedef struct work_struct		WorkStruct;
typedef struct input_dev		InputDev;
typedef struct device			Device;

#ifdef TPS_TPCT_COMMAND
struct Tps_diag_info{
	uint8_t					pos_mode;
	int						event;
	wait_queue_head_t		wait;
	int						enable;
};
#endif	/* TPS_TPCT_COMMAND */

#ifdef FACETOUCH_DETECT_ENABLE
struct shtps_facetouch_info{
	int							mode;
	int							state;
	int							off_detect;
	int							wake_sig;
	wait_queue_head_t			wait_off;
	struct wake_lock			wake_lock;
	int							sleepfacetouch;
	int							multitouch;
};
#endif	/* FACETOUCH_DETECT_ENABLE */

struct spitps_record
{
	SpiDev  *mpoSpidev;
	InputDev *mpoInDev;
	int		mnProductInfo;
	char	mcPhysInfo[QPHYSLEN];
	int		mnIrqPin;
	int		(*mpfPinSetupFunc)(void);
	void	(*mpfPinShutdownFunc)(void);
	uint8_t	mbIsActive;					/* Driver Status(1:Active/0:Inactive) */
	struct delayed_work moCmdQ;
	WorkStruct moIrqWork;
	int		mnReset;
	int		mnVcpin;
	int		mnHsspClkPin;
	int		mnHsspDataPin;
	TpsState mnState;
	uint8_t	mbIsEnable;					/* Status(1:Enable/0:Disable) */
	uint8_t	mbIsTestMode;				/* Test mode Status(1:Test mode/0:Normal mode) */
	uint8_t	mbAdjustEnable;				/* Adjustment(1:Validity 0:Invalid) */
	uint8_t	mbAccessState;				/* Access state */
	uint8_t	mbIsFirst;					/* flag in First start */
	uint8_t	mbIsUpdate;
	uint8_t	mbIsChargerArmor;
	struct hrtimer	polling_timer;
	struct work_struct	polling_work;
#ifdef TPS_IRQ_WAKE
	int		mnIrqWake;
#endif	/* TPS_IRQ_WAKE */
#ifdef TPS_TPCT_COMMAND
	struct Tps_diag_info	diag;
#endif	/* TPS_TPCT_COMMAND */
#ifdef FACETOUCH_DETECT_ENABLE
	struct	shtps_facetouch_info	facetouch;
#endif	/* FACETOUCH_DETECT_ENABLE */
};

typedef struct
{
	short x;
	short y;
} TpsPoint_t;

typedef struct
{
	TpsPoint_t p;		/* Upper left */
	TpsPoint_t q;		/* Upper right */
	TpsPoint_t r;		/* Lower left */
	TpsPoint_t s;		/* Lower right */
} TpsArea_t;

typedef struct
{
	short mValue;
	short mNo;
} Qsort_t;

typedef struct
{
	uint8_t mbID;
	uint16_t mwPosX;
	uint16_t mwPosY;
	uint16_t mwPosZ;
} TpsEvent;

/* Defined type dispatch table */
typedef struct
{
	uint8_t mbValid;							/* 0:Invalid */
												/* Event Handling Functions */
	void (*mpReportInit)(void);
	void (*mpReportPos)(InputDev *, int nCnt, TpsEvent *poEv);
} TpsDispatch;

static struct semaphore sem;
static uint8_t gSense[TPS_SENSOR_READ_VAL];

static uint16_t sh_sensor_max_x;
static uint16_t sh_sensor_max_y;

/*+-------------------------------------------------------------------------+*/
/*|	Defining Global Variables												|*/
/*+-------------------------------------------------------------------------+*/
#ifdef TPS_SETPARAM
static uint8_t gbSetParam = 0x01;
#endif	/* TPS_SETPARAM */

/* Coordinates of six criteria points for adjusting */
static const TpsPoint_t gBasePt[ADJUST_POINT] =
		{
			{POS_X1, POS_Y1}, {POS_X2, POS_Y1},
			{POS_X1, POS_Y2}, {POS_X2, POS_Y2},
			{POS_X1, POS_Y3}, {POS_X2, POS_Y3},
		};
/* Coordinates of the split area for adjusting */
static const TpsArea_t gAreaRect[AREA_COUNT] =
		{
			{{POS_X0, POS_Y0}, {POS_X1, POS_Y0}, {POS_X0, POS_Y1}, {POS_X1, POS_Y1}},
			{{POS_X1, POS_Y0}, {POS_X2, POS_Y0}, {POS_X1, POS_Y1}, {POS_X2, POS_Y1}},
			{{POS_X2, POS_Y0}, {POS_X3, POS_Y0}, {POS_X2, POS_Y1}, {POS_X3, POS_Y1}},
			{{POS_X0, POS_Y1}, {POS_X1, POS_Y1}, {POS_X0, POS_Y2}, {POS_X1, POS_Y2}},
			{{POS_X1, POS_Y1}, {POS_X2, POS_Y1}, {POS_X1, POS_Y2}, {POS_X2, POS_Y2}},
			{{POS_X2, POS_Y1}, {POS_X3, POS_Y1}, {POS_X2, POS_Y2}, {POS_X3, POS_Y2}},
			{{POS_X0, POS_Y2}, {POS_X1, POS_Y2}, {POS_X0, POS_Y3}, {POS_X1, POS_Y3}},
			{{POS_X1, POS_Y2}, {POS_X2, POS_Y2}, {POS_X1, POS_Y3}, {POS_X2, POS_Y3}},
			{{POS_X2, POS_Y2}, {POS_X3, POS_Y2}, {POS_X2, POS_Y3}, {POS_X3, POS_Y3}},
			{{POS_X0, POS_Y3}, {POS_X1, POS_Y3}, {POS_X0, POS_Y4}, {POS_X1, POS_Y4}},
			{{POS_X1, POS_Y3}, {POS_X2, POS_Y3}, {POS_X1, POS_Y4}, {POS_X2, POS_Y4}},
			{{POS_X2, POS_Y3}, {POS_X3, POS_Y3}, {POS_X2, POS_Y4}, {POS_X3, POS_Y4}},

		};
static TpsArea_t gAreaDiff[AREA_COUNT];
static TpsPoint_t gAdjustPrm[ADJUST_POINT];

static TpsSensorData gSensData;
static TpsEvent gPrev[4];
static sharp_smem_common_type * sh_smem_common_ptr = NULL;
static int gnHwRev = 0;

static uint32_t gdwWait = 1;
static int gnResult = 0;

static unsigned long drag_timeout[4];
static int gnDragStep[4];
#ifdef TPS_TPCT_COMMAND
static struct shtps_touch_info diaginfo;
#endif	/* TPS_TPCT_COMMAND */

/*+-------------------------------------------------------------------------+*/
/*|	Prototype declaration													|*/
/*+-------------------------------------------------------------------------+*/
/* SPI access */
static int ShSpiTps_SpiWriteOne(uint8_t bReg, uint8_t Data, uint8_t bLen);
static int ShSpiTps_SpiRead(uint8_t bReg, uint8_t *Data_p, uint8_t bLen);
static int ShSpiTps_Command(SpiDev *poSpidev, unsigned int wCmd, void *pArg);
static int __init ShSpiTps_Init(void);
static void __exit ShSpiTps_Exit(void);
static int __devinit ShSpiTps_Probe(struct spi_device *spi);
static int __devexit ShSpiTps_Remove(struct spi_device *spi);
static void ShSpiTps_SpiShutdown(struct spi_device *spi);
static void ShSpiTps_Connect2InputSys(WorkStruct *poWork);
static InputDev *ShSpiTps_CreateInputDev(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_ConfigGPIO(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_ReleaseGPIO(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_OpenCB(InputDev *poInDev);
static void ShSpiTps_CloseCB(InputDev *pInDev);
static void ShSpiTps_Shutdown(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_Start(SpiTpsRec *poSpiTpsRec, int nCalib);
static void ShSpiTps_Stop(SpiTpsRec *poSpiTpsRec);
static void ShSpiTps_Reset(int nOnOff);
static void ShSpiTps_Standby(int nOnOff);
static void ShSpiTps_Vcpin(int nOnOff);
static void ShSpiTps_HsspClk(int nOnOff);
static void ShSpiTps_HsspData(int nOnOff);
static void ShSpiTps_PowerOn(void);
static void ShSpiTps_PowerOff(void);
#ifdef TPS_IRQ_WAKE
static void ShSpiTps_irq_wake_disable(SpiTpsRec *poSpiTpsRec);
static void ShSpiTps_irq_wake_enable(SpiTpsRec *poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
static int ShSpiTps_ReStart(void *poPt);
static void ShSpiTps_Polling_Start(SpiTpsRec *poSpiTpsRec);
static void ShSpiTps_Polling_Stop(SpiTpsRec *poSpiTpsRec);
static void ShSpiTps_TouchCheck(SpiTpsRec *poSpiTpsRec);
static void ShSpiTps_Work_Pollf(struct work_struct *work);
static enum hrtimer_restart ShSpiTps_Polling_Timer_Function(struct hrtimer *timer);
static void ShtpsTps_Event_Complement(SpiTpsRec *poSpiTpsRec);
static irqreturn_t ShSpiTps_IrqHandler(int nIrq, void *pvDevId);
static void ShSpiTps_FetchInt(WorkStruct *poWork);
static void ShSpiTps_Recover(WorkStruct *poWork);
static int ShSpiTps_DelayEnable(void *poPt);
static int ShSpiTps_SetState(SpiTpsRec *poSpiTpsRec, uint8_t bMask, uint8_t bValue, uint8_t bCheck, uint8_t bResult);
static int ShSpiTps_Enable_Phase1(SpiTpsRec *poSpiTpsRec, uint8_t bResult);
static int ShSpiTps_Enable_Phase2(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_Disable(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_GetFwVer(SpiTpsRec *poSpiTpsRec);
#ifdef TPS_SETPARAM
static int ShSpiTps_WriteFirmParam(SpiDev *poSpidev);
#endif	/* TPS_SETPARAM */
static int ShSpiTps_TestMode_Start(SpiTpsRec *poSpiTpsRec, int nMode);
static int ShSpiTps_TestMode_Stop(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_ParamSetting(SpiTpsRec *poSpiTpsRec, int nParam);
static int ShSpiTps_Calibration(void);
static int ShSpiTps_SetReCalibration(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_FirstCalibration(void *poPt);
static int ShSpiTps_SensorCheck(void);
static int ShSpiTps_Reg_Read(SpiTpsRec *poSpiTpsRec, unsigned long arg);
static int ShSpiTps_Reg_Write(SpiTpsRec *poSpiTpsRec, unsigned long arg);
static int ShSpiTps_ChargerArmorEn(void *poPt);
static void ShSpiTps_SetChargerArmor(SpiTpsRec *poSpiTpsRec);
#ifdef TPS_TPCT_COMMAND
static int ShSpiTps_SetSoftReset(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_Get_Touchinfo(SpiTpsRec *poSpiTpsRec, unsigned long arg);
static int ShSpiTps_Get_Touchinfo_untrans(SpiTpsRec *poSpiTpsRec, unsigned long arg);
static int ShSpiTps_Set_Touchinfo_mode(SpiTpsRec *poSpiTpsRec, unsigned long arg);
#endif	/* TPS_TPCT_COMMAND */
static int ShSpiTps_FirmUp_Set(SpiTpsRec *poSpiTpsRec, int nMode, int nCheck);
static int ShSpiTps_Gpio_Reset(int nMode);
static int ShSpiTps_Gpio_HsspClk(int nOnOff);
static int ShSpiTps_Gpio_HsspData(int nOnOff);
static int ShSpiTps_Gpio_HsspClkCh(int nInOut);
static int ShSpiTps_Gpio_HsspDataCh(int nInOut);
static int ShSpiTps_SDATACheck(void);
static int ShSpiTps_RunClock(int nNumCycles);
static uint8_t ShSpiTps_ReceiveBit(void);
static uint8_t ShSpiTps_ReceiveByte(void);
static int ShSpiTps_SendByte(uint8_t bData, int nNumBits);
static int ShSpiTps_DetectHiLoTransition(void);
/* Adjustment of coordinates */
static void ShSpiTps_Qsort(Qsort_t *pTable, int nTop, int nEnd);
static void ShSpiTps_RoundValue(short *pValue);
static int ShSpiTps_SetAdjustParam(SpiTpsRec *poSpiTpsRec, uint16_t *pParam);
static void ShSpiTps_AdjustPt(short *pX, short *pY);
static void ShSpiTps_PosInit(void);
static void ShSpiTps_PosSet(InputDev *pInDev, int nCnt, TpsEvent *poEv);

static void ShSpiTps_GetSensorSize(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_Get_Dragstep(int type, int fingers);
static int ShSpiTps_Get_Diff(unsigned short pos1, unsigned short pos2);
static void ShSpiTps_Rec_Notify_Time(int index);
static int ShSpiTps_Chk_Notify_Time(int index);
#ifdef FACETOUCH_DETECT_ENABLE
static int ShSpiTps_Start_Facetouchmode(SpiTpsRec *poSpiTpsRec, int nParam);
static int ShSpiTps_Poll_Facetouchoff(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_Ack_Facetouchoff(SpiTpsRec *poSpiTpsRec);
static int ShSpiTps_Stop_Facetouchmode(SpiTpsRec *poSpiTpsRec, int nParam);
static void shtps_notify_facetouchoff(SpiTpsRec *poSpiTpsRec, int force);
static void ShSpiTps_Wake_Lock(SpiTpsRec *poSpiTpsRec);
static void ShSpiTps_Wake_Unlock(SpiTpsRec *poSpiTpsRec);
static void ShSpiTps_Get_Fingerwidth(int nFingerNum, TpsEvent *poEv);
#endif	/* FACETOUCH_DETECT_ENABLE */

static DEFINE_MUTEX(goTpsAccessMutex);

/*+-----------------------------------------------------------------------------+*/
/*|	Macro definition															|*/
/*+-----------------------------------------------------------------------------+*/
#define	MINMAX(min,max,val)	((min)>(val) ? (min) : ((max)<(val) ? (max) : (val)))
#define	SET_POINT(val,x1,y1)	val.x=(x1);val.y=(y1)
#define	SET_AREA(val,x1,y1,x2,y2,x3,y3,x4,y4)	\
								val.p.x=x1;val.p.y=y1;val.q.x=x2;val.q.y=y2;	\
								val.r.x=x3;val.r.y=y3;val.s.x=x4;val.s.y=y4;

#define	SCLKHigh()			ShSpiTps_HsspClk(1);
#define	SCLKLow()			ShSpiTps_HsspClk(0);
#define	SetSDATAHigh()		ShSpiTps_HsspData(1);
#define	SetSDATALow()		ShSpiTps_HsspData(0);
#define	AssertXRES()		ShSpiTps_Reset(1);
#define	DeassertXRES()		ShSpiTps_Reset(0);
#define	SetSDATAHiZ()		ShSpiTps_HsspData(0);	\
							gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_DATA, 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
#define	SetSDATAStrong()	gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_DATA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);

static const TpsDispatch gTpsDispatch[TPS_STATE_MAX][TPS_STATE_MAX] =
{
	/* [EVENT] HOVER */
	{
		{	0,	NULL,				NULL			},	/* [STATE] HOVER */
		{	1,	NULL,				ShSpiTps_PosSet	},	/* [STATE] DOWN */
	},
	/* [EVENT] DOWN */
	{
		{	1,	ShSpiTps_PosInit,	ShSpiTps_PosSet	},	/* [STATE] HOVER */
		{	1,	NULL,				ShSpiTps_PosSet	},	/* [STATE] DOWN */
	},
};

static SpiTpsRec *gpoSpiTpsRec = NULL;
static struct cdev goTpsCDev;
static struct class *gpoTpsClass;
static dev_t gnTpsDev;

/*+-------------------------------------------------------------------------+*/
/*|	SPI access																|*/
/*+-------------------------------------------------------------------------+*/
static int ShSpiTps_SpiWriteOne(uint8_t bReg, uint8_t Data, uint8_t bLen)
{
	int err;
	char tx_buf[40];
	char rx_buf[40];
	struct spi_message  m;
	struct spi_transfer t;
	int nI;

	memset(&t, 0, sizeof t);
	memset(tx_buf, 0, 40);
	memset(rx_buf, 0, 40);

	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;

	t.len = 4+bLen;
	tx_buf[0] = 0x00;
	tx_buf[1] = 0xFF;
	tx_buf[2] = bReg;
	tx_buf[3] = 0x00;
	tx_buf[4] = Data;

	for(nI = 0; nI < RETRY_MAX; nI++)
	{
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		err = spi_sync(gpoSpiTpsRec->mpoSpidev, &m);

		if (err < 0)
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Write spi sync Error %d\n", err);
#endif /* TPS_PRNERR */
			return err;
		}

		/* ACK completion */
		if(rx_buf[2] == 0x62)
			return 0;

#ifdef TPS_PRNLOG
		printk(KERN_DEBUG "[ShSpiTps]Write Retry %d\n", nI);
#endif	/* TPS_PRNLOG */

		/* Waiting and retry */
		udelay(RETRY_WAIT);

	}
	return -1;
}

static int ShSpiTps_SpiRead(uint8_t bReg, uint8_t *Data_p, uint8_t bLen)
{
	int err;
	char tx_buf[40];
	struct spi_message  m;
	struct spi_transfer t;
	int nI;

	memset(&t, 0, sizeof t);
	memset(tx_buf, 0, 40);

	t.tx_buf = tx_buf;
	t.rx_buf = Data_p;

	t.len = bLen;
	tx_buf[0] = 0;
	tx_buf[1] = 0xFF;
	tx_buf[2] = bReg;
	tx_buf[3] = 0x01;

	for(nI = 0; nI < RETRY_MAX; nI++)
	{
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		err = spi_sync(gpoSpiTpsRec->mpoSpidev, &m);

		if (err < 0)
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Read spi sync Error %d\n", err);
#endif /* TPS_PRNERR */
			return err;
		}

		/* ACK completion */
		if(Data_p[2] == 0x62)
			return 0;

#ifdef TPS_PRNLOG
		printk(KERN_DEBUG "[ShSpiTps]Read Retry %d\n", nI);
#endif	/* TPS_PRNLOG */

		/* Waiting and retry */
		udelay(RETRY_WAIT);
	}
	return -1;
}

static int TpsIf_Open(struct inode *pINode, struct file *poFile)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]Open\n");
#endif	/* TPS_PRNLOG */
	return 0;
}

#ifdef TPS_TPCT_COMMAND
static ssize_t TpsIf_Read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	struct shtps_touch_info info;
//	int i;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]TpsIf_Read\n");
#endif	/* TPS_PRNLOG */

	if(gpoSpiTpsRec == NULL){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]TpsIf_Read NULL\n");
#endif	/* TPS_PRNERR */
		return -EFAULT;
	}
	wait_event_interruptible(gpoSpiTpsRec->diag.wait, gpoSpiTpsRec->diag.event == 1);

	memcpy(&info, &diaginfo, sizeof(diaginfo));

//	if(gpoSpiTpsRec->diag.pos_mode == TPSDEV_TOUCHINFO_MODE_LCDSIZE){
//		for(i = 0;i < 4;i++){
//			info.fingers[i].x = info.fingers[i].x * (((SH_TOUCH_LCD_V_MAX_X + 1) * 10000) / sh_sensor_max_x) / 10000;
//			info.fingers[i].y = info.fingers[i].y * (((SH_TOUCH_LCD_V_MAX_Y + 1) * 10000) / sh_sensor_max_y) / 10000;
//		}
//	}
	if(copy_to_user((uint8_t*)buf, (uint8_t*)&info, sizeof(info))){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]TpsIf_Read Error\n");
#endif	/* TPS_PRNERR */
		return -EFAULT;
	}

	gpoSpiTpsRec->diag.event = 0;

	return sizeof(diaginfo);
}

static unsigned int TpsIf_Poll(struct file *file, poll_table *wait)
{
	int ret;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]TpsIf_Poll\n");
#endif	/* TPS_PRNLOG */

	if(gpoSpiTpsRec == NULL){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]TpsIf_Poll NULL\n");
#endif	/* TPS_PRNERR */
		return -EFAULT;
	}
	ret = wait_event_interruptible_timeout(gpoSpiTpsRec->diag.wait, 
			gpoSpiTpsRec->diag.event == 1,
			msecs_to_jiffies(DIAGPOLL_TIME));

	if(0 != ret){
		return POLLIN | POLLRDNORM;
	}
	return 0;
}
#endif	/* TPS_TPCT_COMMAND */

static int TpsIf_Ioctl(struct inode *pINode, struct file *poFile, unsigned int wCmd, unsigned long dwArg)
{
	int nResult = -EINVAL;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]Ioctl(CMD:%d,ARG:%lx)\n", wCmd, dwArg);
#endif	/* TPS_PRNLOG */
#ifdef FACETOUCH_DETECT_ENABLE
	if(wCmd != TPSDEV_POLL_FACETOUCHOFF)
	{
		mutex_lock(&goTpsAccessMutex);
	}
#else
	mutex_lock(&goTpsAccessMutex);
#endif	/* FACETOUCH_DETECT_ENABLE */
	if(gpoSpiTpsRec != NULL)
	{
		nResult = ShSpiTps_Command(gpoSpiTpsRec->mpoSpidev, wCmd, (void *)dwArg);
		switch(nResult)
		{
		case -1:
			nResult = -EIO;
			break;
		case -2:
			nResult = -EINVAL;
			break;
		case -3:
			nResult = -EFAULT;
			break;
		}
	}
#ifdef FACETOUCH_DETECT_ENABLE
	if(wCmd != TPSDEV_POLL_FACETOUCHOFF)
	{
		mutex_unlock(&goTpsAccessMutex);
	}
#else
	mutex_unlock(&goTpsAccessMutex);
#endif	/* FACETOUCH_DETECT_ENABLE */
	return nResult;
}

static const struct file_operations goTpsIf_Fops =
{
	.owner	= THIS_MODULE,
	.open	= TpsIf_Open,
	.ioctl	= TpsIf_Ioctl,
#ifdef TPS_TPCT_COMMAND
	.read	= TpsIf_Read,
	.poll	= TpsIf_Poll,
#endif	/* TPS_TPCT_COMMAND */
};

int __init TpsIf_Setup(void)
{
	dev_t nMajor = 0;
	dev_t nMinor = 0;
	int nResult;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]Setup\n");
#endif	/* TPS_PRNLOG */
	nResult = alloc_chrdev_region(&gnTpsDev, 0, 1, TPSIF_DEV_NAME);
	if(!nResult)
	{
		nMajor = MAJOR(gnTpsDev);
		nMinor = MINOR(gnTpsDev);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]alloc_chrdev_region %d:%d\n", nMajor, nMinor);
#endif	/* TPS_PRNLOG */
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]alloc_chrdev_region error\n");
#endif	/* TPS_PRNERR */
		return -1;
	}

	cdev_init(&goTpsCDev, &goTpsIf_Fops);

	goTpsCDev.owner = THIS_MODULE;
	goTpsCDev.ops = &goTpsIf_Fops;

	nResult = cdev_add(&goTpsCDev, gnTpsDev, 1);
	if(nResult)
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]cdev_add error\n");
#endif	/* TPS_PRNERR */
		cdev_del(&goTpsCDev);
		return -1;
	}

	gpoTpsClass = class_create(THIS_MODULE, TPSIF_DEV_NAME);
	if(IS_ERR(gpoTpsClass))
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTpsIF]class_create error\n");
#endif	/* TPS_PRNERR */
		cdev_del(&goTpsCDev);
		return -1;
	}
	device_create(gpoTpsClass, NULL, gnTpsDev, &goTpsCDev, TPSIF_DEV_NAME);
	return 0;
}

void __exit TpsIf_Cleanup(void)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTpsIF]Cleanup\n");
#endif	/* TPS_PRNLOG */
	device_destroy(gpoTpsClass, gnTpsDev);
	class_destroy(gpoTpsClass);
	cdev_del(&goTpsCDev);
}

module_init(TpsIf_Setup);
module_exit(TpsIf_Cleanup);

/*+-------------------------------------------------------------------------+*/
/*|	Touch panel Driver														|*/
/*+-------------------------------------------------------------------------+*/
#ifdef FACETOUCH_DETECT_ENABLE
static void ShSpiTps_Wake_Lock(SpiTpsRec *poSpiTpsRec)
{
	wake_lock(&poSpiTpsRec->facetouch.wake_lock);
}

static void ShSpiTps_Wake_Unlock(SpiTpsRec *poSpiTpsRec)
{
	wake_unlock(&poSpiTpsRec->facetouch.wake_lock);
}

static void ShSpiTps_Get_Fingerwidth(int nFingerNum, TpsEvent *poEv)
{
	int nCnt;
	
	if((gpoSpiTpsRec->facetouch.mode == 1) && nFingerNum >= 2){
		gpoSpiTpsRec->facetouch.state = 1;
		for(nCnt = 0; nCnt <= (nFingerNum-1); nCnt++)
		{
			poEv[nCnt].mwPosZ = SHTPS_FINGER_WIDTH_PALMDET;
#ifdef	TPS_PRNDEB
			printk(KERN_DEBUG "[ShSpiTps]poEv[%d].mwPosZ = %d\n", nCnt, poEv[nCnt].mwPosZ);
#endif	/* TPS_PRNDEB */
		}
	}
	else
	{
		for(nCnt = 0; nCnt <= (nFingerNum-1); nCnt++)
		{
			if(poEv[nCnt].mwPosZ < SHTPS_FINGER_WIDTH_MIN){
				poEv[nCnt].mwPosZ = SHTPS_FINGER_WIDTH_MIN;
			}else if(poEv[nCnt].mwPosZ > SHTPS_FINGER_WIDTH_MAX){
				poEv[nCnt].mwPosZ = SHTPS_FINGER_WIDTH_MAX;
			}
#ifdef	TPS_PRNDEB
			printk(KERN_DEBUG "[ShSpiTps]poEv[%d].mwPosZ = %d\n", nCnt, poEv[nCnt].mwPosZ);
#endif	/* TPS_PRNDEB */
		}
	}
}

static void shtps_notify_facetouchoff(SpiTpsRec *poSpiTpsRec, int force)
{
	poSpiTpsRec->facetouch.state = 0;
	poSpiTpsRec->facetouch.off_detect = 1;
	wake_up_interruptible(&poSpiTpsRec->facetouch.wait_off);
#ifdef TPS_PRNDEB
	printk(KERN_DEBUG "face touch off detect. wake_up()\n");
#endif	/* TPS_PRNDEB */
}
#endif	/* FACETOUCH_DETECT_ENABLE */

static int ShSpiTps_Command(SpiDev *poSpidev, unsigned int wCmd, void *pArg)
{
	int nResult;
	uint8_t bData;
	int nResponse;
	int nHwRev;
	int ret = 0;
	Tps_send_data data;

#ifdef TPS_PRNLOG
	int nCnt;
#endif	/* TPS_PRNLOG */

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Command(CMD:%d,ARG:%lx)\n", wCmd, (long)pArg);
#endif	/* TPS_PRNLOG */
	/* IOCTL that can be executed without device running */
	switch(wCmd)
	{
	case TPSDEV_FW_VERSION:
		return ShSpiTps_GetFwVer(gpoSpiTpsRec);
	case TPSDEV_FW_DOWNLOAD:
		return -2;
	case TPSDEV_FW_UPDATE:
		return -2;
	case TPSDEV_FW_UPDATE2:
		return -2;
	case TPSDEV_UPDATE_START:
		nResult = 0;
		nHwRev = ShSpiTps_FirmUp_Set(gpoSpiTpsRec, 1, 0);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]FirmUp_Set(%d)\n", nHwRev);
#endif	/* TPS_PRNLOG */

		if(copy_to_user((int*)pArg, &nHwRev, sizeof(nHwRev)))
		{
			nResult = -3;
		}
		return nResult;
	case TPSDEV_UPDATE_STOP:
		nResult = 0;
		ShSpiTps_FirmUp_Set(gpoSpiTpsRec, 0, (int)pArg);
		return nResult;
	case TPSDEV_GPIO_RESET:
		/* If not beginning to rewriting firmware */
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* controlling GPIO */
		return ShSpiTps_Gpio_Reset(1);
	case TPSDEV_GPIO_HSSPCLK:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* controlling GPIO */
		return ShSpiTps_Gpio_HsspClk((int)pArg);
	case TPSDEV_GPIO_HSSPDATA:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* controlling GPIO */
		return ShSpiTps_Gpio_HsspData((int)pArg);
	case TPSDEV_GPIO_CLK_INOUT:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* Switching I/O */
		return ShSpiTps_Gpio_HsspClkCh((int)pArg);
	case TPSDEV_GPIO_DATA_INOUT:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		/* Switching I/O */
		return ShSpiTps_Gpio_HsspDataCh((int)pArg);
	case TPSDEV_RUN_CLOCK:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		return ShSpiTps_RunClock((int)pArg);
	case TPSDEV_RECV_BYTE:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		nResult = 0;
		bData = ShSpiTps_ReceiveByte();
		if(copy_to_user((uint8_t*)pArg, &bData, sizeof(bData)))
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]copy_to_user Error(ReceiveByte)\n");
#endif	/* TPS_PRNERR */
			nResult = -3;
		}
		return nResult;
	case TPSDEV_SEND_BYTE:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		ret = copy_from_user(&data, (Tps_send_data *)pArg, sizeof(Tps_send_data));
		if (ret != 0) {
			return -2;
	    }
	    if(data.bReset != 0)
	    {
			/* reset */
			ShSpiTps_Gpio_Reset(0);
		}
		return ShSpiTps_SendByte(data.bData, data.nNumBits);
	case TPSDEV_DETECT_HILO:
		if(gpoSpiTpsRec->mbIsUpdate == 0)
			return -2;
		nResult = 0;
		nResponse = ShSpiTps_DetectHiLoTransition();
		if(copy_to_user((int*)pArg, &nResponse, sizeof(nResponse)))
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]copy_to_user Error(ReceiveByte)\n");
#endif	/* TPS_PRNERR */
			nResult = -3;
		}
		return nResult;
	}

	/* If Device is not open or Being written */
	if(gpoSpiTpsRec->mbIsActive == 0 || gpoSpiTpsRec->mbIsUpdate != 0)
		return -2;
	switch(wCmd)
	{
	case TPSDEV_ENABLE:
		/* Setting the touchpanel operating state */
		return ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_OFF, TPS_CHECK_ON, TPS_RETURN_ON);
	case TPSDEV_DISABLE:
		/* Setting the touchpanel standby mode */
		return ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_ON, TPS_CHECK_ON, TPS_RETURN_ON);
	case TPSDEV_START_TESTMODE:
		/* Starting the Test mode */
		return ShSpiTps_TestMode_Start(gpoSpiTpsRec, (int)pArg);
	case TPSDEV_STOP_TESTMODE:
		/* Stopping the Test mode */
		return ShSpiTps_TestMode_Stop(gpoSpiTpsRec);
	case TPSDEV_GET_SENSOR:
		nResult = -2;
		if(gpoSpiTpsRec->mbIsTestMode)
		{
			nResult = 0;
			/* copy acquisition data in the user space */
			if(copy_to_user((TpsSensorData*)pArg, &gSensData, sizeof(TpsSensorData)))
			{
#ifdef TPS_PRNERR
				printk(KERN_DEBUG "[ShSpiTps]copy_to_user Error\n");
#endif	/* TPS_PRNERR */
				nResult = -3;
			}
#ifdef TPS_PRNLOG
			printk(KERN_DEBUG "[ShSpiTps]PosX %3d / PosY %3d / Counter %3d\n", gSensData.wPosX, gSensData.wPosY, gSensData.bCounter);
			for(nCnt = 0; nCnt < SHTPS_TMA_RXNUM_MAX; nCnt++)
			{
printk(KERN_DEBUG "[ShSpiTps]RX%02d %3d %3d %3d %3d %3d %3d %3d %3d\n", nCnt+1,
																gSensData.bSensData[SHTPS_TMA_RXNUM_MAX*0+nCnt],
																gSensData.bSensData[SHTPS_TMA_RXNUM_MAX*1+nCnt],
																gSensData.bSensData[SHTPS_TMA_RXNUM_MAX*2+nCnt],
																gSensData.bSensData[SHTPS_TMA_RXNUM_MAX*3+nCnt],
																gSensData.bSensData[SHTPS_TMA_RXNUM_MAX*4+nCnt],
																gSensData.bSensData[SHTPS_TMA_RXNUM_MAX*5+nCnt],
																gSensData.bSensData[SHTPS_TMA_RXNUM_MAX*6+nCnt],
																gSensData.bSensData[SHTPS_TMA_RXNUM_MAX*7+nCnt]);
			}
#endif	/* TPS_PRNLOG */
		}
		return nResult;
	case TPSDEV_SET_FIRMPARAM:
		return ShSpiTps_ParamSetting(gpoSpiTpsRec, (int)pArg);
	case TPSDEV_CALIBRATION_PARAM:
		return ShSpiTps_SetAdjustParam(gpoSpiTpsRec, (uint16_t*)pArg);
	case TPSDEV_SLEEP_ON:
		return ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_ON, TPS_CHECK_OFF, TPS_RETURN_OFF);
	case TPSDEV_SLEEP_OFF:
		return ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_OFF, TPS_CHECK_OFF, TPS_RETURN_OFF);
	case TPSDEV_RECALIBRATION_IDAC:
		return ShSpiTps_SetReCalibration(gpoSpiTpsRec);

	case TPSDEV_READ_REG:
		return ShSpiTps_Reg_Read(gpoSpiTpsRec, (unsigned long)pArg);
	case TPSDEV_WRITE_REG:
		return ShSpiTps_Reg_Write(gpoSpiTpsRec, (unsigned long)pArg);
#ifdef TPS_TPCT_COMMAND
	case TPSDEV_HWRESET:
		/* controlling GPIO */
		return ShSpiTps_Gpio_Reset(0);
	case TPSDEV_SOFTRESET:
		return ShSpiTps_SetSoftReset(gpoSpiTpsRec);
	case TPSDEV_GET_TOUCHINFO:
		return ShSpiTps_Get_Touchinfo(gpoSpiTpsRec, (unsigned long)pArg);
	case TPSDEV_GET_TOUCHINFO_UNTRANS:
		return ShSpiTps_Get_Touchinfo_untrans(gpoSpiTpsRec, (unsigned long)pArg);
	case TPSDEV_SET_TOUCHMONITOR_MODE:
		return ShSpiTps_Set_Touchinfo_mode(gpoSpiTpsRec, (unsigned long)pArg);
#endif	/* TPS_TPCT_COMMAND */
#ifdef FACETOUCH_DETECT_ENABLE
	case TPSDEV_START_FACETOUCHMODE:
		return ShSpiTps_Start_Facetouchmode(gpoSpiTpsRec, (int)pArg);
	case TPSDEV_POLL_FACETOUCHOFF:
		return ShSpiTps_Poll_Facetouchoff(gpoSpiTpsRec);
	case TPSDEV_ACK_FACETOUCHOFF:
		return ShSpiTps_Ack_Facetouchoff(gpoSpiTpsRec);
	case TPSDEV_STOP_FACETOUCHMODE:
		return ShSpiTps_Stop_Facetouchmode(gpoSpiTpsRec, (int)pArg);
#endif	/* FACETOUCH_DETECT_ENABLE */
	}
	return -2;
}

module_init(ShSpiTps_Init);
module_exit(ShSpiTps_Exit);
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("SPI TOUCH sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:SH_touchpanel");


/* Structure for SPI driver call */
static struct spi_driver goSpiTpsDriver =
{
	.driver =
	{
		.name  = SH_TOUCH_SPI_DEVNAME,
		.owner = THIS_MODULE,
		.bus	= &spi_bus_type,
	},
	.probe	  = ShSpiTps_Probe,
	.remove	  = __devexit_p(ShSpiTps_Remove),
	.shutdown = ShSpiTps_SpiShutdown,
};

static int __init ShSpiTps_Init(void)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Init\n");
#endif	/* TPS_PRNLOG */

	/* SPI driver use start */
	return spi_register_driver(&goSpiTpsDriver);
}

static void __exit ShSpiTps_Exit(void)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Exit\n");
#endif	/* TPS_PRNLOG */

	/* SPI driver use end */
	spi_unregister_driver(&goSpiTpsDriver);
}

static int __devinit ShSpiTps_Probe(struct spi_device *spi)
{
	struct msm_sh_spitps_platform_data *poSetupData;
	SpiTpsRec *poSpiTpsRec = NULL;
	int nResult;
#ifdef CONFIG_USB_SWIC
	uint8_t device  = 0;
	shswic_result_t swic_ret;
#endif /* CONFIG_USB_SWIC */

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Probe\n");
#endif	/* TPS_PRNLOG */

	if(!spi->dev.platform_data)
	{
		dev_err(&spi->dev, "platform device data is required\n");
		return -ENODEV;
	}

	/* The memory for touch panel driver information is secured.  */
	poSpiTpsRec = kzalloc(sizeof(SpiTpsRec), GFP_KERNEL);
	if(!poSpiTpsRec)
	{
		return -ENOMEM;
	}

	gpoSpiTpsRec = poSpiTpsRec;

	poSpiTpsRec->mpoSpidev			 = spi;
	poSetupData						 = spi->dev.platform_data;
	/* Setup information is obtained.(It defined by "board-xxxx.c") */
	poSpiTpsRec->mnIrqPin			 = poSetupData->gpio_irq;
	poSpiTpsRec->mnHsspClkPin		 = poSetupData->gpio_hssp_clk;
	poSpiTpsRec->mnHsspDataPin		 = poSetupData->gpio_hssp_data;
	poSpiTpsRec->mnReset			 = poSetupData->gpio_reset;
	poSpiTpsRec->mnVcpin			 = poSetupData->gpio_vcpin;
	poSpiTpsRec->mpfPinSetupFunc	 = poSetupData->gpio_setup;
	poSpiTpsRec->mpfPinShutdownFunc  = poSetupData->gpio_teardown;

	poSpiTpsRec->mnState			 = TPS_STATE_HOVER;
	poSpiTpsRec->mbAccessState		 = 0;
	poSpiTpsRec->mbIsActive			 = 0;
	poSpiTpsRec->mbIsUpdate			 = 0;
#ifdef TPS_TPCT_COMMAND
	poSpiTpsRec->diag.event			 = 0;
	poSpiTpsRec->diag.enable		 = 0;
	init_waitqueue_head(&poSpiTpsRec->diag.wait);
#endif	/* TPS_TPCT_COMMAND */
#ifdef FACETOUCH_DETECT_ENABLE
	poSpiTpsRec->facetouch.mode = 0;
	poSpiTpsRec->facetouch.state = 0;
	poSpiTpsRec->facetouch.off_detect = 0;
	poSpiTpsRec->facetouch.wake_sig = 0;
	poSpiTpsRec->facetouch.sleepfacetouch = 0;
	poSpiTpsRec->facetouch.multitouch = 0;
	init_waitqueue_head(&poSpiTpsRec->facetouch.wait_off);
	wake_lock_init(&poSpiTpsRec->facetouch.wake_lock, WAKE_LOCK_SUSPEND, "shspitps_wake_lock");
#endif	/* FACETOUCH_DETECT_ENABLE */
#ifdef CONFIG_USB_SWIC
	swic_ret = shswic_get_usb_port_status(&device);
	if (swic_ret == SHSWIC_SUCCESS) {
		switch(device)
		{
		case SHSWIC_ID_USB_CABLE:
		case SHSWIC_ID_AC_ADAPTER:
			poSpiTpsRec->mbIsChargerArmor = 1;
			break;
		default:
			poSpiTpsRec->mbIsChargerArmor = 0;
			break;
		}
	}else{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]shswic_get_usb_port_status err\n");
#endif	/* TPS_PRNERR */
		poSpiTpsRec->mbIsChargerArmor = 0;
	}
#else
	poSpiTpsRec->mbIsChargerArmor	 = 0;
#endif	/* CONFIG_USB_SWIC */
	hrtimer_init(&poSpiTpsRec->polling_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	poSpiTpsRec->polling_timer.function = ShSpiTps_Polling_Timer_Function;
	INIT_WORK(&poSpiTpsRec->polling_work, ShSpiTps_Work_Pollf);
	/* Semaphore initialization */
	init_MUTEX(&sem);

	/* getting the top address that structs of common SMEM */
	sh_smem_common_ptr = sh_smem_get_common_address();
	if( sh_smem_common_ptr != NULL)
	{
		/* getting the hardware revision */
		gnHwRev = sh_smem_common_ptr->sh_hw_revision;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]hw_revision(%d)\n", gnHwRev);
#endif	/* TPS_PRNLOG */
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]sh_smem_get_common_address is NULL\n");
#endif	/* TPS_PRNERR */
	}

	/* setting the GPIO */
	if(0 == (nResult = ShSpiTps_ConfigGPIO(poSpiTpsRec)))
	{
		INIT_WORK(&poSpiTpsRec->moIrqWork, ShSpiTps_FetchInt);
		dev_info(&spi->dev, "Detected %s, attempting to initialize\n", SH_TOUCH_SPI_DEVNAME);
		INIT_DELAYED_WORK(&poSpiTpsRec->moCmdQ, ShSpiTps_Connect2InputSys);
		schedule_delayed_work(&poSpiTpsRec->moCmdQ, msecs_to_jiffies(INITDELAY_TIMER));
		device_init_wakeup(&spi->dev, 1);
		return 0;
	}
	/* release the GPIO */
	ShSpiTps_ReleaseGPIO(poSpiTpsRec);
#ifdef FACETOUCH_DETECT_ENABLE
	wake_lock_destroy(&poSpiTpsRec->facetouch.wake_lock);
#endif	/* FACETOUCH_DETECT_ENABLE */
	kfree(poSpiTpsRec);
	return nResult;
}

static int __devexit ShSpiTps_Remove(struct spi_device *spi)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Remove\n");
#endif	/* TPS_PRNLOG */
	dev_info(&spi->dev, "removing driver\n");
	device_init_wakeup(&spi->dev, 0);
#ifdef FACETOUCH_DETECT_ENABLE
	wake_lock_destroy(&gpoSpiTpsRec->facetouch.wake_lock);
#endif	/* FACETOUCH_DETECT_ENABLE */
	if(gpoSpiTpsRec->mpoInDev)
	{
		dev_dbg(&spi->dev, "deregister from input system\n");
		input_unregister_device(gpoSpiTpsRec->mpoInDev);
		gpoSpiTpsRec->mpoInDev = NULL;
	}
	ShSpiTps_Shutdown(gpoSpiTpsRec);
	ShSpiTps_ReleaseGPIO(gpoSpiTpsRec);
	kfree(gpoSpiTpsRec);
	gpoSpiTpsRec = NULL;
	return 0;
}

static void ShSpiTps_SpiShutdown(struct spi_device *spi)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]SpiShutdown\n");
#endif	/* TPS_PRNLOG */

	msm_tps_shutdown();
}

static void ShSpiTps_Connect2InputSys(WorkStruct *poWork)
{
	SpiTpsRec *poSpiTpsRec = container_of(poWork, SpiTpsRec, moCmdQ.work);
	Device *poDev = &poSpiTpsRec->mpoSpidev->dev;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Connect2InputSys\n");
#endif	/* TPS_PRNLOG */
	poSpiTpsRec->mpoInDev = ShSpiTps_CreateInputDev(poSpiTpsRec);
	if(poSpiTpsRec->mpoInDev)
	{
		if(input_register_device(poSpiTpsRec->mpoInDev) != 0)
		{
			dev_err(poDev, "Failed to register with input system\n");
			input_free_device(poSpiTpsRec->mpoInDev);
		}
	}
}

static InputDev *ShSpiTps_CreateInputDev(SpiTpsRec *poSpiTpsRec)
{
	Device *poDev = &poSpiTpsRec->mpoSpidev->dev;
	InputDev *pInDev = input_allocate_device();

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]CreateInputDev\n");
#endif	/* TPS_PRNLOG */
	if(pInDev)
	{
		pInDev->name = SH_TOUCH_SPI_DEVNAME;
		pInDev->phys = poSpiTpsRec->mcPhysInfo;
		pInDev->id.vendor  = QCVENDOR_ID;
		pInDev->id.product = QCPRODUCT_ID;
		pInDev->id.version = QCVERSION_ID;
		pInDev->open = ShSpiTps_OpenCB;
		pInDev->close = ShSpiTps_CloseCB;
		/* registering valid event */
		__set_bit(EV_KEY, pInDev->evbit);
		__set_bit(EV_ABS, pInDev->evbit);
		/* Vertual key */
		__set_bit(KEY_MENU,   pInDev->keybit);
		__set_bit(KEY_HOME,   pInDev->keybit);
		__set_bit(KEY_BACK,   pInDev->keybit);
#if 0
		__set_bit(KEY_SEARCH, pInDev->keybit);
#endif
		__set_bit(ABS_MT_TOUCH_MAJOR, pInDev->absbit);
		__set_bit(ABS_MT_POSITION_X, pInDev->absbit);
		__set_bit(ABS_MT_POSITION_Y, pInDev->absbit);
		__set_bit(ABS_MT_WIDTH_MAJOR, pInDev->absbit);
		input_set_drvdata(pInDev, poSpiTpsRec);
		/* setting the range of Event parameters */
		input_set_abs_params(pInDev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
#if 0
		input_set_abs_params(pInDev, ABS_MT_POSITION_X,  0, SH_TOUCH_MAX_X, 0, 0);
		input_set_abs_params(pInDev, ABS_MT_POSITION_Y,  0, SH_TOUCH_MAX_Y, 0, 0);
#else
		/* Vertual key */
		input_set_abs_params(pInDev, ABS_MT_POSITION_X,  0, SH_TOUCH_LCD_MAX_X, 0, 0);
		input_set_abs_params(pInDev, ABS_MT_POSITION_Y,  0, SH_TOUCH_LCD_MAX_Y, 0, 0);
#endif
#ifdef FACETOUCH_DETECT_ENABLE
		input_set_abs_params(pInDev, ABS_MT_WIDTH_MAJOR, 0, SHTPS_FINGER_WIDTH_PALMDET, 0, 0);
#else
		input_set_abs_params(pInDev, ABS_MT_WIDTH_MAJOR, 0, SH_TOUCH_MAX_DIAGONAL, 0, 0);
#endif	/* FACETOUCH_DETECT_ENABLE */
	}
	else
	{
		dev_err(poDev, "Failed to allocate input device for %s\n", SH_TOUCH_SPI_DEVNAME);
	}
	return pInDev;
	
}

static int ShSpiTps_ConfigGPIO(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_ConfigGPIO\n");
#endif	/* TPS_PRNLOG */
	if(poSpiTpsRec == NULL)
		return -EINVAL;
	return poSpiTpsRec->mpfPinSetupFunc();
}

static int ShSpiTps_ReleaseGPIO(SpiTpsRec *poSpiTpsRec)
{
	if(poSpiTpsRec == NULL)
		return -EINVAL;
	/* release the GPIO */
	dev_info(&poSpiTpsRec->mpoSpidev->dev, "releasing gpio pins %d,%d,%d,%d,%d\n",
			 poSpiTpsRec->mnIrqPin, poSpiTpsRec->mnHsspClkPin, poSpiTpsRec->mnHsspDataPin,
			 poSpiTpsRec->mnReset, poSpiTpsRec->mnVcpin);
	poSpiTpsRec->mpfPinShutdownFunc();
	return 0;
}

static int ShSpiTps_OpenCB(InputDev *poInDev)
{
	SpiTpsRec *poSpiTpsRec = input_get_drvdata(poInDev);

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]OpenCB\n");
#endif	/* TPS_PRNLOG */

	dev_dbg(&poSpiTpsRec->mpoSpidev->dev, "ENTRY: input_dev open callback\n");
	return ShSpiTps_Start(poSpiTpsRec, TPS_CALIB_ON);
}

static void ShSpiTps_CloseCB(InputDev *pInDev)
{
	SpiTpsRec *poSpiTpsRec = input_get_drvdata(pInDev);
	Device *poDev = &poSpiTpsRec->mpoSpidev->dev;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]CloseCB\n");
#endif	/* TPS_PRNLOG */

	dev_dbg(poDev, "ENTRY: close callback\n");
	ShSpiTps_Shutdown(poSpiTpsRec);
}

static void ShSpiTps_Shutdown(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Shutdown\n");
#endif	/* TPS_PRNLOG */

	/* If touchpanel driver is active */
	if(poSpiTpsRec->mbIsActive)
	{
		mutex_lock(&goTpsAccessMutex);
		ShSpiTps_Stop(poSpiTpsRec);
		mutex_unlock(&goTpsAccessMutex);
		/* release the work-memory */
		flush_work(&poSpiTpsRec->moIrqWork);
	}
}

//volatile uint32_t dwSpeed = 2000000;
volatile uint32_t dwSpeed = 1000000;	/* 1.0Mbps*/

static int ShSpiTps_Start(SpiTpsRec *poSpiTpsRec, int nCalib)
{
	int err;
	struct task_struct *p;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Start\n");
#endif	/* TPS_PRNLOG */
	mutex_lock(&goTpsAccessMutex);
	/* disabling a touchpanel */
	poSpiTpsRec->mbIsEnable = 0;
	poSpiTpsRec->mbAccessState |= TPS_DISABLE_API;
	poSpiTpsRec->mbIsActive = 1;
	/* Setting the Normal mode */
	poSpiTpsRec->mbIsTestMode = 0;
	poSpiTpsRec->mbAdjustEnable = 0;
	poSpiTpsRec->mbIsFirst = 1;
#ifdef TPS_IRQ_WAKE
	poSpiTpsRec->mnIrqWake = TPS_IRQ_WAKE_DISABLE;
#endif	/* TPS_IRQ_WAKE */

	/* Setting the SPI */
	poSpiTpsRec->mpoSpidev->bits_per_word = 8;
	poSpiTpsRec->mpoSpidev->mode          = SPI_MODE_1;

	err = spi_setup(poSpiTpsRec->mpoSpidev);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]spi_setup = %d\n", err);
#endif	/* TPS_PRNLOG */
	if (err < 0)
	{
		poSpiTpsRec->mbIsActive = 0;
		mutex_unlock(&goTpsAccessMutex);
		return -EIO;
	}

	/* VCPIN(5V)ON (100msWait) */
	ShSpiTps_Vcpin(1);
	/* Power ON */
	if(0 != ShSpiTps_SetState(poSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_OFF, TPS_CHECK_ON, TPS_RETURN_ON))
	{
		poSpiTpsRec->mbIsActive = 0;
		mutex_unlock(&goTpsAccessMutex);
		return -EIO;
	}

	ShSpiTps_GetSensorSize(poSpiTpsRec);

	if(nCalib != TPS_CALIB_OFF)
	{
		/* executes it by the thread */
		p = kthread_run(ShSpiTps_FirstCalibration, poSpiTpsRec, "shspitps_firstcalibration");
		/* If it cannot start a thread, doing the synchronous execution */
		if(IS_ERR(p))
			ShSpiTps_FirstCalibration(poSpiTpsRec);
		/* perform mutex_unlock at the time of the calibration end */
		return 0;
	}

	mutex_unlock(&goTpsAccessMutex);
	return 0;
}

static void ShSpiTps_Stop(SpiTpsRec *poSpiTpsRec)
{
	/* disabling a touchpanel */
	ShSpiTps_SetState(poSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_ON, TPS_CHECK_ON, TPS_RETURN_OFF);
	poSpiTpsRec->mbIsActive = 0;
}

static void ShSpiTps_Reset(int nOnOff)
{
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_RESET , 1);
	} else {
		gpio_direction_output(SH_TOUCH_RESET , 0);
	}
}

static void ShSpiTps_Standby(int nOnOff)
{
#ifdef TPS_STBY_SOFTCONTROL
	uint8_t bStby = 0;
	uint8_t rx_buf[6];

	if(nOnOff != 0)
	{
		if(ShSpiTps_SpiRead(0x00, rx_buf, 5) == 0)
		{
			bStby = rx_buf[4];
			bStby |= 0x02;

			if(ShSpiTps_SpiWriteOne(0x00, bStby, 1) == 0)
			{
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Standby OK\n");
				return;
			}
		}
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Standby NG\n");
	}
#else
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_STBY , 1);
	} else {
		gpio_direction_output(SH_TOUCH_STBY , 0);
	}
#endif	/* TPS_STBY_SOFTCONTROL */
}

static void ShSpiTps_Vcpin(int nOnOff)
{
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_VCPIN , 0);
	} else {
		gpio_direction_output(SH_TOUCH_VCPIN , 1);
		/* 100ms Wait */
		msleep(100);
	}
}

static void ShSpiTps_HsspClk(int nOnOff)
{
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_SPI_HSSP_CLK , 0);
	} else {
		gpio_direction_output(SH_TOUCH_SPI_HSSP_CLK , 1);
	}
}

static void ShSpiTps_HsspData(int nOnOff)
{
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_SPI_HSSP_DATA, 0);
	} else {
		gpio_direction_output(SH_TOUCH_SPI_HSSP_DATA, 1);
	}
}

static void ShSpiTps_PowerOn(void)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_PowerOn\n");
#endif	/* TPS_PRNDEB */
	ShSpiTps_Reset(1);
	usleep(TPS_RESET_INTERVAL);
	ShSpiTps_Reset(0);
	msleep(5);
	ShSpiTps_Standby(0);
	msleep(300);
}

static void ShSpiTps_PowerOff(void)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_PowerOff\n");
#endif	/* TPS_PRNDEB */
	/* setting to the standby state */
	ShSpiTps_Standby(1);
}

#ifdef TPS_ISSP_ENABLE
static int ShSpiTps_ISSPport_Change(int nMode)
{
	int result;

	if(nMode == 1)
	{
printk(KERN_DEBUG "[ShSpiTps]GPIO 45/47 SPI -> ISSP\n");
		/* SPI -> ISSP */
		result = gpio_request(SH_TOUCH_SPI_HSSP_DATA, "spi_clk");
		if(result < 0){
			printk(KERN_DEBUG "[shtps]spi_clk gpio_request() error : %d\n", result);
		}
		result = gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_DATA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if(result < 0){
			printk(KERN_DEBUG "[shtps]spi_clk gpio_tlmm_config() error : %d\n", result);
		}
		result = gpio_request(SH_TOUCH_SPI_HSSP_CLK, "spi_mosi");
		if(result < 0){
			printk(KERN_DEBUG "[shtps]spi_mosi gpio_request() error : %d\n", result);
		}
		result = gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if(result < 0){
			printk(KERN_DEBUG "[shtps]spi_mosi gpio_tlmm_config() error : %d\n", result);
		}
		gpio_direction_output(SH_TOUCH_SPI_HSSP_DATA, 0);
		gpio_direction_output(SH_TOUCH_SPI_HSSP_CLK,  0);
	}
	else
	{
printk(KERN_DEBUG "[ShSpiTps]GPIO 45/47 ISSP -> SPI\n");
		msleep(100);
		gpio_direction_output(SH_TOUCH_SPI_HSSP_DATA , 1);
		gpio_direction_output(SH_TOUCH_SPI_HSSP_CLK , 0);
		/* ISSP -> SPI */
		gpio_free(SH_TOUCH_SPI_HSSP_DATA);
		result = gpio_request(SH_TOUCH_SPI_HSSP_DATA, "spi_clk");
		if(result < 0){
			printk(KERN_DEBUG "[shtps]spi_clk gpio_request() error : %d\n", result);
		}
		result = gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_DATA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if(result < 0){
			printk(KERN_DEBUG "[shtps]spi_clk gpio_tlmm_config() error : %d\n", result);
		}
		gpio_free(SH_TOUCH_SPI_HSSP_CLK);
		result = gpio_request(SH_TOUCH_SPI_HSSP_CLK, "spi_mosi");
		if(result < 0){
			printk(KERN_DEBUG "[shtps]spi_mosi gpio_request() error : %d\n", result);
		}
		result = gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if(result < 0){
			printk(KERN_DEBUG "[shtps]spi_mosi gpio_tlmm_config() error : %d\n", result);
		}
	}
	msleep(50);
	return result;
}
#endif	/* TPS_ISSP_ENABLE */

#ifdef TPS_IRQ_WAKE
static void ShSpiTps_irq_wake_disable(SpiTpsRec *poSpiTpsRec)
{
	if(poSpiTpsRec->mnIrqWake != TPS_IRQ_WAKE_DISABLE)
	{
		disable_irq_wake(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
		poSpiTpsRec->mnIrqWake = TPS_IRQ_WAKE_DISABLE;
	}
}

static void ShSpiTps_irq_wake_enable(SpiTpsRec *poSpiTpsRec)
{
	if(poSpiTpsRec->mnIrqWake != TPS_IRQ_WAKE_ENABLE)
	{
		enable_irq_wake(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
		poSpiTpsRec->mnIrqWake = TPS_IRQ_WAKE_ENABLE;
	}
}
#endif	/* TPS_IRQ_WAKE */

static int ShSpiTps_ReStart(void *poPt)
{
	SpiTpsRec *poSpiTpsRec = (SpiTpsRec *)poPt;
	int nI;
#ifdef TPS_SETPARAM
	int nResult = -1;
#endif	/* TPS_SETPARAM */
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_ReStart\n");
#endif	/* TPS_PRNDEB */
	/* to the hover-state */
	if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mbValid != 0)
	{
		/* Report */
		if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportInit != NULL)
		{
			gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportInit();
		}
		for(nI = 0; nI < 4; nI++)
		{
			/* Report[ABS_MT_TOUCH_MAJOR][ABS_MT_POSITION_X][ABS_MT_POSITION_Y][ABS_MT_WIDTH_MAJOR] */
			if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportPos != NULL)
			{
				gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportPos(poSpiTpsRec->mpoInDev, 0, NULL);
			}
		}
		input_sync(poSpiTpsRec->mpoInDev);
	}
	poSpiTpsRec->mnState = TPS_STATE_HOVER;
	/* setting to the standby state */
	ShSpiTps_PowerOff();
	ShSpiTps_PowerOn();
#ifdef TPS_SETPARAM
	/* setting parameter */
	nResult = ShSpiTps_WriteFirmParam(poSpiTpsRec->mpoSpidev);
	if(nResult < 0)
	{
#ifdef TPS_PRNERR
		printk(KERN_ERR "WriteFirmParam Error %d\n", nResult);
#endif	/* TPS_PRNERR */
	}
#endif	/* TPS_SETPARAM */
	/* allowing the next interrupt */
#ifdef TPS_IRQ_WAKE
	ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
	enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	mutex_unlock(&goTpsAccessMutex);
	return 0;
}

static void ShSpiTps_Polling_Start(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
	printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Polling_Start\n");
#endif	/* TPS_PRNLOG */
	hrtimer_cancel(&poSpiTpsRec->polling_timer);
	hrtimer_start(&poSpiTpsRec->polling_timer, ktime_set(0, TPS_CHECK_POLL_INTERVAL * 1000 * 1000), HRTIMER_MODE_REL);
}

static void ShSpiTps_Polling_Stop(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
	printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Polling_Stop\n");
#endif	/* TPS_PRNLOG */
	hrtimer_try_to_cancel(&poSpiTpsRec->polling_timer);
}

static void ShSpiTps_TouchCheck(SpiTpsRec *poSpiTpsRec)
{
	int bTt_s;
	uint8_t rx_buf[40];
	InputDev *pInDev = poSpiTpsRec->mpoInDev;

	mutex_lock(&goTpsAccessMutex);

	/* Reading register */
	if(0 != ShSpiTps_SpiRead(0x01, rx_buf, 32))
	{
		printk(KERN_DEBUG "[ShSpiTps]spi read error\n");
		return;
	}

	bTt_s  = rx_buf[5];
#ifdef TPS_PRNLOG
	printk(KERN_DEBUG "[Tps]Toutch Check (TT_STAT 0x%02x)\n", bTt_s);
#endif	/* TPS_PRNLOG */

	/* Ignoring LargeObjectDetect */
	bTt_s &= 0x0f;
	/* if not touched  */
	if(bTt_s == 0)
	{
		if((gPrev[0].mbID != 0xff) || (gPrev[0].mwPosX != 0xffff) || (gPrev[0].mwPosY != 0xffff) || (gPrev[0].mwPosZ != 0xffff))
		{
#ifdef TPS_EVENTLOG
			printk(KERN_DEBUG "[Tps]Force Toutch Up  (%d,%4d,%4d,%4d)\n", gPrev[0].mbID, gPrev[0].mwPosX, gPrev[0].mwPosY, gPrev[0].mwPosZ);
#endif	/* TPS_EVENTLOG */
			/* notice of the previous Information */
			input_report_abs(pInDev, ABS_MT_TOUCH_MAJOR, 0); 
			input_report_abs(pInDev, ABS_MT_POSITION_X,  gPrev[0].mwPosX);
			input_report_abs(pInDev, ABS_MT_POSITION_Y,  gPrev[0].mwPosY);
			input_report_abs(pInDev, ABS_MT_WIDTH_MAJOR, gPrev[0].mwPosZ);
			input_mt_sync(pInDev);
			input_sync(pInDev);
			/* clearing management information */
			gPrev[0].mbID = 0xff;
			gPrev[0].mwPosX = 0xffff;
			gPrev[0].mwPosY = 0xffff;
			gPrev[0].mwPosZ = 0xffff;
		}
		ShSpiTps_Polling_Stop(poSpiTpsRec);
		mutex_unlock(&goTpsAccessMutex);
		return;
	}
	ShSpiTps_Polling_Stop(poSpiTpsRec);
	ShSpiTps_Polling_Start(poSpiTpsRec);
	mutex_unlock(&goTpsAccessMutex);
}

static void ShSpiTps_Work_Pollf(struct work_struct *work)
{
	SpiTpsRec *poSpiTpsRec = container_of(work, SpiTpsRec, polling_work);
#ifdef TPS_PRNLOG
	printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Work_Pollf\n");
#endif	/* TPS_PRNLOG */
	ShSpiTps_TouchCheck(poSpiTpsRec);

}

static enum hrtimer_restart ShSpiTps_Polling_Timer_Function(struct hrtimer *timer)
{
	SpiTpsRec *poSpiTpsRec = container_of(timer,SpiTpsRec, polling_timer);
#ifdef TPS_PRNLOG
	printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Polling_Timer_Function\n");
#endif	/* TPS_PRNLOG */
	schedule_work(&poSpiTpsRec->polling_work);
	return HRTIMER_NORESTART;
}

static void ShtpsTps_Event_Complement(SpiTpsRec *poSpiTpsRec)
{
	int nI;

	/* to the hover-state */
	if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mbValid != 0)
	{
		/* Report */
		if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportInit != NULL)
		{
			gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportInit();
		}
		for(nI = 0; nI < 4; nI++)
		{
			/* Report[ABS_MT_TOUCH_MAJOR][ABS_MT_POSITION_X][ABS_MT_POSITION_Y][ABS_MT_WIDTH_MAJOR] */
			if(gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportPos != NULL)
			{
				gTpsDispatch[TPS_STATE_HOVER][poSpiTpsRec->mnState].mpReportPos(poSpiTpsRec->mpoInDev, 0, NULL);
			}
		}
		input_sync(poSpiTpsRec->mpoInDev);
	}
	poSpiTpsRec->mnState = TPS_STATE_HOVER;
}

static irqreturn_t ShSpiTps_IrqHandler(int nIrq, void *pvDevId)
{
	SpiTpsRec *poSpiTpsRec = pvDevId;

#ifdef TPS_IRQ_WAKE
	ShSpiTps_irq_wake_disable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
	disable_irq_nosync(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	schedule_work(&poSpiTpsRec->moIrqWork);
	return IRQ_HANDLED;
}

static void ShSpiTps_FetchInt(WorkStruct *poWork)
{
	struct task_struct *p;
	SpiTpsRec *poSpiTpsRec = container_of(poWork, SpiTpsRec, moIrqWork);
//	SpiDev *poSpidev = poSpiTpsRec->mpoSpidev;
	InputDev *pInDev = poSpiTpsRec->mpoInDev;
	uint8_t rx_buf[40];
	uint8_t bTt_m, bTt_s;
	static uint8_t prev_bTt_s = 0;
	int nI, nJ;
	int nNextState;
	int diff_x;
	int diff_y;
	int dragStepCur;
	int dragStep1st;
	int nReport[4];
	uint8_t bNotify = 0;
	
#ifdef TPS_PRNLOG
	const char *StaName[] = { "HOVER", "DOWN "};
#endif	/* TPS_PRNLOG */
	TpsEvent oEv[4];
	uint8_t bInt = 0;
	uint8_t bVer = 0x00;

#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]FetchInt\n");
#endif	/* TPS_PRNDEB */
	mutex_lock(&goTpsAccessMutex);
#ifdef FACETOUCH_DETECT_ENABLE
	if(poSpiTpsRec->facetouch.sleepfacetouch)
	{
		poSpiTpsRec->facetouch.wake_sig = 1;
	}
#endif	/* FACETOUCH_DETECT_ENABLE */
	/* If Test mode */
	if(poSpiTpsRec->mbIsTestMode)
	{
		/* acquire a sensor value */
		if(0 == ShSpiTps_SpiRead(0x00, gSense, TPS_SENSOR_READ_VAL))
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps] HST_MODE [0x%2X]\n", gSense[4]);
printk(KERN_DEBUG "[ShSpiTps] MD[0x%2X] ST[0x%2X]\n", gSense[5], gSense[6]);
printk(KERN_DEBUG "[ShSpiTps] NewData[0x%2X]\n", (gSense[5] >> 6));
printk(KERN_DEBUG "[ShSpiTps] X1[%4d] Y1[%4d]\n",(uint16_t)(gSense[7] << 8) + ((uint16_t)gSense[8]),(uint16_t)(gSense[9] << 8) + ((uint16_t)gSense[10]));
#endif	/* TPS_PRNLOG */
			/* coordinate */
			gSensData.wPosX = (uint16_t)(gSense[7] << 8) + ((uint16_t)gSense[8]);
			gSensData.wPosY = (uint16_t)(gSense[9] << 8) + ((uint16_t)gSense[10]);
			gSensData.bCounter = (uint8_t)(gSense[5] >> 6);
			memcpy(&gSensData.bSensData[0], &gSense[11], TPS_SENSOR_SIZE);

			/* interrupt cancellation */
			bInt = gSense[4];
			if(bInt & 0x80){
				bInt &= ~0x80;
			}else{
				bInt |= 0x80;
			}
			if(0 == ShSpiTps_SpiWriteOne(0x00, bInt, 1))
			{
				/* wait */
				msleep(20);
			}
		}
		mutex_unlock(&goTpsAccessMutex);

		/* wait */
		msleep(15);
		/* allowing the next interrupt */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
		return;
	}
	if(poSpiTpsRec->mbIsEnable == 0)
	{
#ifdef TPS_PRNLOG
		printk(KERN_DEBUG "[ShSpiTps]Already disable\n");
#endif	/* TPS_PRNLOG */
		mutex_unlock(&goTpsAccessMutex);
		return;
	}
	/* Reading register */
	if(0 != ShSpiTps_SpiRead(0x01, rx_buf, 32))
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]spi read error\n");
#endif	/* TPS_PRNERR */
#ifdef FACETOUCH_DETECT_ENABLE
		if(poSpiTpsRec->mbIsUpdate != 1 || poSpiTpsRec->facetouch.sleepfacetouch != 1)
#else
		if(poSpiTpsRec->mbIsUpdate != 1)
#endif	/* FACETOUCH_DETECT_ENABLE */
		{
			ShSpiTps_Stop(poSpiTpsRec);
			INIT_DELAYED_WORK(&poSpiTpsRec->moCmdQ, ShSpiTps_Recover);
			schedule_delayed_work(&poSpiTpsRec->moCmdQ, msecs_to_jiffies(RECOVER_TIMER));
		}
		else
		{
#ifdef TPS_IRQ_WAKE
			ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
			enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
		}
		mutex_unlock(&goTpsAccessMutex);
		return;
	}

	/* Negative_finger bit check */
	if(rx_buf[31] & 0x40)
	{
		/* executes it by the thread */
		p = kthread_run(ShSpiTps_ReStart, poSpiTpsRec, "shspitps_restart");
		if(IS_ERR(p)){
			/* allowing the next interrupt */
#ifdef TPS_IRQ_WAKE
			ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
			enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
			mutex_unlock(&goTpsAccessMutex);
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]ReStart error\n");
#endif	/* TPS_PRNERR */
		}
		/* perform mutex_unlock after having restarted */
		return;
	}

	bTt_m   = rx_buf[4];
	bTt_s   = rx_buf[5];
	/* getting the Coordinate */
	oEv[0].mbID = rx_buf[11] >> 4;
	oEv[0].mwPosX = (uint16_t)(rx_buf[ 6] << 8) + ((uint16_t)rx_buf[ 7]);
	oEv[0].mwPosY = (uint16_t)(rx_buf[ 8] << 8) + ((uint16_t)rx_buf[ 9]);
	oEv[0].mwPosZ = (uint16_t)rx_buf[10];

	oEv[1].mbID = rx_buf[11] & 0x0f;
	oEv[1].mwPosX = (uint16_t)(rx_buf[12] << 8) + ((uint16_t)rx_buf[13]);
	oEv[1].mwPosY = (uint16_t)(rx_buf[14] << 8) + ((uint16_t)rx_buf[15]);
	oEv[1].mwPosZ = (uint16_t)rx_buf[16];

	oEv[2].mbID = rx_buf[24] >> 4;
	oEv[2].mwPosX = (uint16_t)(rx_buf[19] << 8) + ((uint16_t)rx_buf[20]);
	oEv[2].mwPosY = (uint16_t)(rx_buf[21] << 8) + ((uint16_t)rx_buf[22]);
	oEv[2].mwPosZ = (uint16_t)rx_buf[23];

	oEv[3].mbID = rx_buf[24] & 0x0f;
	oEv[3].mwPosX = (uint16_t)(rx_buf[25] << 8) + ((uint16_t)rx_buf[26]);
	oEv[3].mwPosY = (uint16_t)(rx_buf[27] << 8) + ((uint16_t)rx_buf[28]);
	oEv[3].mwPosZ = (uint16_t)rx_buf[29];
	bVer = rx_buf[30];

#ifdef TPS_PRNLOG
//	printk(KERN_DEBUG "[ShSpiTps] MD[0x%2X] ST[0x%2X]\n", bTt_m, bTt_s);
//	printk(KERN_DEBUG "[ShSpiTps] X1[%4d] Y1[%4d] Z1[%4d]\n", oEv[0].mwPosX, oEv[0].mwPosY, oEv[0].mwPosZ);
//	printk(KERN_DEBUG "[ShSpiTps] X2[%4d] Y2[%4d] Z2[%4d]\n", oEv[1].mwPosX, oEv[1].mwPosY, oEv[1].mwPosZ);
//	printk(KERN_DEBUG "[ShSpiTps] X3[%4d] Y3[%4d] Z3[%4d]\n", oEv[2].mwPosX, oEv[2].mwPosY, oEv[2].mwPosZ);
//	printk(KERN_DEBUG "[ShSpiTps] X4[%4d] Y4[%4d] Z4[%4d]\n", oEv[3].mwPosX, oEv[3].mwPosY, oEv[3].mwPosZ);
//	printk(KERN_DEBUG "[ShSpiTps] 12ID[0x%2X] 34ID[0x%2X] GCn[%4d] GID[%4d]\n", rx_buf[11], rx_buf[24], rx_buf[17], rx_buf[18]);

	printk(KERN_DEBUG "-------------------------------------\n");
	printk(KERN_DEBUG "MD[0x%2X] ST[0x%2X] GCn[%4d] GID[%4d]\n", bTt_m, bTt_s, rx_buf[17], rx_buf[18]);
	printk(KERN_DEBUG "1 %2d(%4d,%4d,%4d)\n", oEv[0].mbID, oEv[0].mwPosX, oEv[0].mwPosY, oEv[0].mwPosZ);
	printk(KERN_DEBUG "2 %2d(%4d,%4d,%4d)\n", oEv[1].mbID, oEv[1].mwPosX, oEv[1].mwPosY, oEv[1].mwPosZ);
	printk(KERN_DEBUG "3 %2d(%4d,%4d,%4d)\n", oEv[2].mbID, oEv[2].mwPosX, oEv[2].mwPosY, oEv[2].mwPosZ);
	printk(KERN_DEBUG "4 %2d(%4d,%4d,%4d)\n", oEv[3].mbID, oEv[3].mwPosX, oEv[3].mwPosY, oEv[3].mwPosZ);
	printk(KERN_DEBUG "Ver %2x\n"            , rx_buf[30]);
#endif	/* TPS_PRNLOG */
	/* Ignoring LargeObjectDetect */
	bTt_s &= 0x0f;
	/* if touched number is 4 or more */
	if(bTt_s > 4)
		bTt_s = 4;
#ifdef FACETOUCH_DETECT_ENABLE
	if(bTt_s >= 2)
		gpoSpiTpsRec->facetouch.multitouch = 1;
	ShSpiTps_Get_Fingerwidth(bTt_s, oEv);
	if(bTt_s == 0)
	{
		if((gpoSpiTpsRec->facetouch.mode == 1 && gpoSpiTpsRec->facetouch.multitouch == 1) ||(poSpiTpsRec->facetouch.sleepfacetouch == 1))
		{
			gpoSpiTpsRec->facetouch.off_detect = 1;
			gpoSpiTpsRec->facetouch.multitouch = 0;
			shtps_notify_facetouchoff(gpoSpiTpsRec, 0);
		}
		nNextState = TPS_STATE_HOVER;
	}
#else
	if(bTt_s == 0)
	{
		nNextState = TPS_STATE_HOVER;
	}
#endif	/* FACETOUCH_DETECT_ENABLE */
	else
	{
		nNextState = TPS_STATE_DOWN;
	}
	/* If state can transition */
	if(gTpsDispatch[nNextState][poSpiTpsRec->mnState].mbValid != 0)
	{
		/* Report */
		if(gTpsDispatch[nNextState][poSpiTpsRec->mnState].mpReportInit != NULL)
		{
			gTpsDispatch[nNextState][poSpiTpsRec->mnState].mpReportInit();
		}
		for(nI = 0; nI < bTt_s; nI++)
		{
			if(oEv[nI].mwPosX > sh_sensor_max_x)
			{
				oEv[nI].mwPosX = sh_sensor_max_x;
			}
			if(oEv[nI].mwPosY > sh_sensor_max_y)
			{
				oEv[nI].mwPosY = sh_sensor_max_y;
			}
#if 0
			if(bVer < 0x03)
			{
//				oEv[nI].mwPosX = (sh_sensor_max_x - oEv[nI].mwPosX);
				oEv[nI].mwPosY = (sh_sensor_max_y - oEv[nI].mwPosY);
			}
#endif
			oEv[nI].mwPosX = oEv[nI].mwPosX * (((SH_TOUCH_LCD_V_MAX_X + 1) * 10000) / sh_sensor_max_x) / 10000;
			oEv[nI].mwPosY = oEv[nI].mwPosY * (((SH_TOUCH_LCD_V_MAX_Y + 1) * 10000) / sh_sensor_max_y) / 10000;

			/* if Adjustment is enabled */
			if(poSpiTpsRec->mbAdjustEnable != 0)
			{
				/* Adjusting the coordinates */
				ShSpiTps_AdjustPt(&oEv[nI].mwPosX, &oEv[nI].mwPosY);
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]Adjust (%4d,%4d)\n", oEv[nI].mwPosX, oEv[nI].mwPosY);
#endif	/* TPS_PRNDEB */
			}

			/* Because a high rank layer does not react when notify of the same coordinate, do not return the same coordinate */
			for(nJ = 0; nJ < nI; nJ++)
			{
				/* The length and breadth same coordinate moves one pixel of coordinate */
				if(oEv[nI].mwPosX == oEv[nJ].mwPosX && oEv[nI].mwPosY == oEv[nJ].mwPosY)
				{
					if(oEv[nI].mwPosX != 0)
						oEv[nI].mwPosX--;
					else
						oEv[nI].mwPosX++;
					if(oEv[nI].mwPosY != 0)
						oEv[nI].mwPosY--;
					else
						oEv[nI].mwPosY++;
				}
			}
		}

		if(nNextState != TPS_STATE_HOVER)
		{
			dragStep1st = ShSpiTps_Get_Dragstep(TPS_DRAG_THRESHOLD_1ST, bTt_s);
			for(nI= 0; nI < bTt_s; nI++)
			{
				dragStepCur = ShSpiTps_Get_Dragstep(gnDragStep[nI], bTt_s);
				diff_x = ShSpiTps_Get_Diff(oEv[nI].mwPosX, gPrev[nI].mwPosX);
				diff_y = ShSpiTps_Get_Diff(oEv[nI].mwPosY, gPrev[nI].mwPosY);

				if((gPrev[nI].mwPosX == 0xffff) || (gPrev[nI].mwPosY == 0xffff))
				{
					ShSpiTps_Rec_Notify_Time(nI);
					nReport[nI] = 0x01;
				}
				else if(diff_x >= dragStepCur || diff_y >= dragStepCur)
				{
					if(diff_x >= dragStep1st || diff_y >= dragStep1st)
					{
						gnDragStep[nI] = TPS_DRAG_THRESHOLD_2ND;
						ShSpiTps_Rec_Notify_Time(nI);
						nReport[nI] = 0x01;
					}
					else if(ShSpiTps_Chk_Notify_Time(nI) == 0)
					{
						if(diff_x >= dragStepCur || diff_y >= dragStepCur)
						{
							gnDragStep[nI] = TPS_DRAG_THRESHOLD_2ND;
							ShSpiTps_Rec_Notify_Time(nI);
							nReport[nI] = 0x01;
						}
						else
						{
							gnDragStep[nI] = TPS_DRAG_THRESHOLD_2ND;
							nReport[nI] = 0xFF;
						}
					}
					else
					{
						gnDragStep[nI] = TPS_DRAG_THRESHOLD_1ST;
						nReport[nI] = 0x01;
						if(!(diff_x >= dragStep1st || diff_y >= dragStep1st))
						{
							gnDragStep[nI] = TPS_DRAG_THRESHOLD_1ST;
							nReport[nI] = 0xFF;
						}
					}
				}
				else
				{
					nReport[nI] = 0xFF;
				}
			}
			for(nI= 0; nI < bTt_s; nI++){
				if (nReport[nI] != 0xFF)
					bNotify++;
			}
			if(prev_bTt_s > bTt_s){
				bNotify ++;
			}
#ifdef FACETOUCH_DETECT_ENABLE
			if((bNotify == 0) || (poSpiTpsRec->facetouch.sleepfacetouch == 1)){
#else
			if(bNotify == 0){
#endif	/* FACETOUCH_DETECT_ENABLE */
#ifdef TPS_EVENTLOG
				printk(KERN_DEBUG "[ShSpiTps](event cancel)\n");
#endif	/* TPS_EVENTLOG */
				prev_bTt_s = bTt_s;
#ifdef TPS_IRQ_WAKE
				ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
				enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
				mutex_unlock(&goTpsAccessMutex);
				return;
			}
			prev_bTt_s = bTt_s;
		}
		/* Report[ABS_MT_TOUCH_MAJOR][ABS_MT_POSITION_X][ABS_MT_POSITION_Y][ABS_MT_WIDTH_MAJOR] */
		if(gTpsDispatch[nNextState][poSpiTpsRec->mnState].mpReportPos != NULL)
		{
			gTpsDispatch[nNextState][poSpiTpsRec->mnState].mpReportPos(pInDev, bTt_s, oEv);
		}
		input_sync(pInDev);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]State[%s]->[%s] OK\n", StaName[poSpiTpsRec->mnState], StaName[nNextState]);
#endif	/* TPS_PRNLOG */
		poSpiTpsRec->mnState = nNextState;
	}
	else
	{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]State[%s]->[%s] NG\n", StaName[poSpiTpsRec->mnState], StaName[nNextState]);
#endif	/* TPS_PRNLOG */
	}
	prev_bTt_s = bTt_s;
#ifdef TPS_TPCT_COMMAND
	if(poSpiTpsRec->diag.enable){
		poSpiTpsRec->diag.event = 1;
		wake_up_interruptible(&poSpiTpsRec->diag.wait);
	}
#endif	/* TPS_TPCT_COMMAND */
	/* allowing the next interrupt */
#ifdef TPS_IRQ_WAKE
	ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
	enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	mutex_unlock(&goTpsAccessMutex);
}

static int ShSpiTps_Get_Dragstep(int type, int fingers)
{
	int dragStep;
	
	if(type == TPS_DRAG_THRESHOLD_1ST){
		dragStep = (fingers <= 1)? TPS_DRAG_THRESH_VAL_1ST : TPS_DRAG_THRESH_VAL_1ST_MULTI;
	}else{
		dragStep = (fingers <= 1)? TPS_DRAG_THRESH_VAL_2ND : TPS_DRAG_THRESH_VAL_2ND_MULTI;
	}

	return dragStep;
}
static int ShSpiTps_Get_Diff(unsigned short pos1, unsigned short pos2)
{
	int diff = pos1 - pos2;
	return (diff >= 0)? diff : -diff;
}

static void ShSpiTps_Rec_Notify_Time(int index)
{
	drag_timeout[index] = jiffies + msecs_to_jiffies(TPS_DRAG_THRESH_RETURN_TIME);
}

static int ShSpiTps_Chk_Notify_Time(int index)
{
	if(time_after(jiffies, drag_timeout[index])){
		return -1;
	}
	return 0;
}

static void ShSpiTps_Recover(WorkStruct *poWork)
{
	int nResult;
	SpiTpsRec *poSpiTpsRec;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Recover\n");
#endif	/* TPS_PRNLOG */
	poSpiTpsRec = container_of(poWork, SpiTpsRec, moCmdQ.work);

	dev_info(&poSpiTpsRec->mpoSpidev->dev, "touchpanel recovery requested\n");

	nResult = ShSpiTps_Start(poSpiTpsRec, TPS_CALIB_OFF);
	if(nResult != 0)
	{
		dev_err(&poSpiTpsRec->mpoSpidev->dev, "recovery failed with (nResult=%d)\n", nResult);
	}
}

static int ShSpiTps_DelayEnable(void *poPt)
{
	SpiTpsRec *poSpiTpsRec = (SpiTpsRec *)poPt;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_DelayEnable\n");
#endif	/* TPS_PRNLOG */
	/* Power ON */
	ShSpiTps_PowerOn();
	gnResult = ShSpiTps_Enable_Phase2(poSpiTpsRec);
	up(&sem);
	return 0;
}

static int ShSpiTps_SetState(SpiTpsRec *poSpiTpsRec, uint8_t bMask, uint8_t bValue, uint8_t bCheck, uint8_t bResult)
{
	uint8_t bNew;
	int nResult = 0;

	/* get semaphore */
	down(&sem);

	bValue &= bMask;
	bNew = (poSpiTpsRec->mbAccessState & ~bMask) | bValue;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]SetState[%02X]->[%02X]\n", poSpiTpsRec->mbAccessState, bNew);
#endif	/* TPS_PRNLOG */
	/* If doing the double-check */
	if(bCheck)
	{
		if((poSpiTpsRec->mbAccessState & bMask) == bValue)
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]StateCheck NG\n");
#endif	/* TPS_PRNERR */
			/* release semaphore */
			up(&sem);
			return -2;
		}
	}
	/* recording the state */
	poSpiTpsRec->mbAccessState = bNew;
	if(poSpiTpsRec->mbIsActive)
	{
		/* if ready to operate */
		if(bNew == 0x00)
		{
#ifdef FACETOUCH_DETECT_ENABLE
			if((bMask == TPS_DISABLE_SLEEP) && (poSpiTpsRec->facetouch.mode == 1))
			{
				poSpiTpsRec->facetouch.sleepfacetouch = 0;
				ShSpiTps_Wake_Unlock(poSpiTpsRec);
			}
#endif	/* FACETOUCH_DETECT_ENABLE */
			nResult = ShSpiTps_Enable_Phase1(poSpiTpsRec, bResult);
			return nResult;
		}
		else
		{
#ifdef FACETOUCH_DETECT_ENABLE
			if((bMask == TPS_DISABLE_SLEEP) && (poSpiTpsRec->facetouch.mode == 1))
			{
				poSpiTpsRec->facetouch.sleepfacetouch = 1;
				ShSpiTps_Wake_Lock(poSpiTpsRec);
			}
			else
			{
				nResult = ShSpiTps_Disable(poSpiTpsRec);
			}
#else
			nResult = ShSpiTps_Disable(poSpiTpsRec);
#endif	/* FACETOUCH_DETECT_ENABLE */
		}
	}
	/* release semaphore */
	up(&sem);

	return nResult;
}

static int ShSpiTps_Enable_Phase1(SpiTpsRec *poSpiTpsRec, uint8_t bResult)
{
	struct task_struct *p;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Enable\n");
#endif	/* TPS_PRNLOG */
	/* Initialize */
	gnResult = 0;

	/* If there is no disincentive with the off state now */
	if(poSpiTpsRec->mbIsEnable == 0 && poSpiTpsRec->mbAccessState == 0x00)
	{
		if(poSpiTpsRec->mbIsTestMode == 0)
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Enable ON\n");
#endif	/* TPS_PRNLOG */
			/* executes it by the thread */
			p = kthread_run(ShSpiTps_DelayEnable, poSpiTpsRec, "shspitps_delayenable");
			/* If it cannot start a thread, doing the synchronous execution */
			if(IS_ERR(p))
				ShSpiTps_DelayEnable(poSpiTpsRec);
			/* If there is the return value */
			if(bResult)
			{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Result Wait... \n");
#endif	/* TPS_PRNLOG */
				down(&sem);
				up(&sem);
				return gnResult;
			}
			return 0;
		}
		/* enabling a touchpanel */
		poSpiTpsRec->mbIsEnable = 1;
	}

	/* release semaphore */
	up(&sem);

	return 0;
}

static int ShSpiTps_Enable_Phase2(SpiTpsRec *poSpiTpsRec)
{
	int nResult;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Enable_Phase2(%d:%d)\n", poSpiTpsRec->mbIsActive, poSpiTpsRec->mbAccessState);
#endif	/* TPS_PRNLOG */

	/* Initialize the state */
	poSpiTpsRec->mnState = TPS_STATE_HOVER;

#ifdef TPS_SETPARAM
	/* setting parameter */
	nResult = ShSpiTps_WriteFirmParam(poSpiTpsRec->mpoSpidev);
	if(nResult < 0)
	{
		ShSpiTps_PowerOff();
		return -1;
	}
#endif	/* TPS_SETPARAM */


#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Set ChargerArmor(%d)\n", poSpiTpsRec->mbIsChargerArmor);
#endif	/* TPS_PRNLOG */
	if(poSpiTpsRec->mbIsChargerArmor != 0)
	{
		ShSpiTps_SetChargerArmor(poSpiTpsRec);
	}

	/* wakeup set */
	set_irq_type(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), IRQF_TRIGGER_FALLING);
	/* Registering interrupt handler */
	nResult = request_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), &ShSpiTps_IrqHandler,
					 IRQF_TRIGGER_FALLING | IRQF_DISABLED,
				     SH_TOUCH_SPI_DEVNAME, poSpiTpsRec);
	if(nResult < 0)
	{
#ifdef TPS_PRNERR
		printk(KERN_ERR "Could not register for  %s interrupt nResult = %d)\n", SH_TOUCH_SPI_DEVNAME, nResult);
#endif	/* TPS_PRNERR */
		ShSpiTps_PowerOff();
		return -3;
	}
#ifdef FACETOUCH_DETECT_ENABLE
	if(poSpiTpsRec->facetouch.mode == 1)
	{
		ShSpiTps_Wake_Unlock(poSpiTpsRec);
	}
#endif	/* FACETOUCH_DETECT_ENABLE */
#ifdef TPS_IRQ_WAKE
	ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */

	poSpiTpsRec->mbIsFirst= 0;
	/* enabling a touchpanel */
	poSpiTpsRec->mbIsEnable = 1;

	return 0;
}

static int ShSpiTps_Disable(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Disable\n");
#endif	/* TPS_PRNLOG */
	/* If there is a disincentive in ON now */
	if(poSpiTpsRec->mbIsEnable != 0 && poSpiTpsRec->mbAccessState != 0x00)
	{
		if(poSpiTpsRec->mbIsTestMode == 0)
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Disable OFF\n");
#endif	/* TPS_PRNLOG */
			/* Interrupt Unsubscribe */
#ifdef TPS_IRQ_WAKE
			ShSpiTps_irq_wake_disable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
			disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
			free_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin), poSpiTpsRec);

			ShtpsTps_Event_Complement(poSpiTpsRec);
			/* setting to the standby state */
			ShSpiTps_PowerOff();
		}
		/* disabling a touchpanel */
		poSpiTpsRec->mbIsEnable = 0;
	}
	/* If the first start, do Power On and Off */
	else if(poSpiTpsRec->mbIsFirst)
	{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]First\n");
#endif	/* TPS_PRNLOG */
		ShSpiTps_PowerOn();
		ShSpiTps_PowerOff();
		poSpiTpsRec->mbIsFirst = 0;
	}
	return 0;
}

static int ShSpiTps_GetFwVer(SpiTpsRec *poSpiTpsRec)
{
	int nResult = -1;
	uint8_t rx_buf[8];

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}
	else
	{
		/* interrupt disabled */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_disable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}

	if(0 == ShSpiTps_SpiRead(0x1B, rx_buf, 5))
		nResult = rx_buf[4];
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Fw Version %02X\n", nResult);
#endif	/* TPS_PRNLOG */

	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}
	else
	{
		/* Resuming interrupt */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	return nResult;
}

#ifdef TPS_SETPARAM
static int ShSpiTps_WriteFirmParam(SpiDev *poSpidev)
{
	uint8_t bGest = 0;
	uint8_t rx_buf[5];

	if(0 != ShSpiTps_SpiRead(0x1E, rx_buf, 5))
		return -1;

	bGest = rx_buf[4];
	bGest &= ~0x0f;
	bGest |= gbSetParam;

	if(0 != ShSpiTps_SpiWriteOne(0x1E, bGest, 1))
		return -1;
	/* check */
	if(0 != ShSpiTps_SpiRead(0x1E, rx_buf, 5))
		return -1;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Active distance %02X\n", rx_buf[4] & 0x0f);
#endif	/* TPS_PRNLOG */
	return 0;
}
#endif	/* TPS_SETPARAM */

static int ShSpiTps_TestMode_Start(SpiTpsRec *poSpiTpsRec, int nMode)
{
	int nResult = -1;
	uint8_t bSt = 0;
	uint8_t rx_buf[6];

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_TestMode_Start(%02X)\n", nMode);
#endif	/* TPS_PRNLOG */
	if(poSpiTpsRec->mbIsTestMode)
	{
		/* if already in test mode, return an error */
		return -2;
	}

	/* parameter check*/
	if(nMode < 0 || nMode > 3)
		return -3;

	/* if touchpanel is Enabled */
	if(poSpiTpsRec->mbIsEnable)
	{
		/* interrupt disabled */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_disable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	else
	{
		/* Power ON */
		ShSpiTps_PowerOn();
	}

	/*+-------------------------------------------------------------------------+*/
	/*|	Test mode selection														|*/
	/*+-------------------------------------------------------------------------+*/
	if(ShSpiTps_SpiRead(0x00, rx_buf, 5) == 0)
	{
		bSt = rx_buf[4];

		/* Test mode */
		switch(nMode)
		{
			case 0:
				/* Test mode 0 */
				bSt &= ~0x70;
				bSt |= 0x40;
				break;
			case 1:
				/* Test mode 1 */
				bSt &= ~0x70;
				bSt |= 0x50;
				break;
			case 2:
				/* Test mode 2 */
				bSt &= ~0x70;
				bSt |= 0x60;
				break;
			case 3:
				/* Test mode 3 */
				bSt &= ~0x70;
				bSt |= 0x70;
				break;
		}
		if(ShSpiTps_SpiWriteOne(0x00, bSt, 1) == 0)
		{
			/* Clear a buffer */
			memset(gSense, 0x00, TPS_SENSOR_READ_VAL);
			/* Wait */
			msleep(TPS_TESTMODE_START_WAIT);
			/* Resuming interrupt */
#ifdef TPS_IRQ_WAKE
			ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
			enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
			/* Setting the Test mode */
			poSpiTpsRec->mbIsTestMode = 1;
			return 0;
		}
	}

	/* When it was an error, return it to a state */
	/* It resets and power supply ON */
	ShSpiTps_PowerOn();

	if(poSpiTpsRec->mbIsEnable)
	{
		/* Resuming interrupt */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	else
	{
		/* setting to the standby state */
		ShSpiTps_PowerOff();
	}
	return nResult;
}

static int ShSpiTps_TestMode_Stop(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_TestMode_Stop\n");
#endif	/* TPS_PRNLOG */
	if(!poSpiTpsRec->mbIsTestMode)
	{
		/* if not in test mode, return an error */
		return -2;
	}

	/* interrupt disabled */
#ifdef TPS_IRQ_WAKE
	ShSpiTps_irq_wake_disable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
	disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));

	/* It resets and power supply ON */
	ShSpiTps_PowerOn();

	/* if touchpanel is Enabled */
	if(poSpiTpsRec->mbIsEnable)
	{
		/* Resuming interrupt */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	else
	{
		/* setting to the standby state */
		ShSpiTps_PowerOff();
	}
	/* to Normal mode */
	poSpiTpsRec->mbIsTestMode = 0;

	return 0;
}

static int ShSpiTps_ParamSetting(SpiTpsRec *poSpiTpsRec, int nParam)
{
#ifdef TPS_SETPARAM
	int nResult = 0;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_ParamSetting %d\n", nParam);
#endif	/* TPS_PRNLOG */
	/* check a parameter */
	if(nParam < 1 || nParam > 15)
		return -2;

	/* store a parameter */
	gbSetParam = (uint8_t)nParam;
	gbSetParam &= 0x0f;

	/* perform the off state in next Enable */
	if(!poSpiTpsRec->mbIsEnable)
		return 0;

	nResult = ShSpiTps_WriteFirmParam(poSpiTpsRec->mpoSpidev);

	return nResult;
#else
	return -3;
#endif	/* TPS_SETPARAM */
}

static int ShSpiTps_Calibration(void)
{
	uint8_t rx_buf[5];
	int nResult = -1;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Calibration\n");
#endif	/* TPS_PRNLOG */

	/* carry out calibration */
	if(0 == ShSpiTps_SpiWriteOne(0x1C, 0x02, 1))
	{
		/* Waiting is necessary(1.35sec) */
		msleep(TPS_CALIB_WAIT);
		/* calibration end check */
		if(0 == ShSpiTps_SpiRead(0x1C, rx_buf, 5))
		{
			/* If calibration is not over, wait more */
			if(rx_buf[4] & 0x02)
			{
				/* Waiting is necessary(1.35sec) */
				msleep(TPS_CALIB_WAIT);
			}
			nResult = 0;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Calibration OK\n");
#endif	/* TPS_PRNLOG */
		}
		else
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Calibration Read Error\n");
#endif /* TPS_PRNERR */
		}
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Calibration Write Error\n");
#endif /* TPS_PRNERR */
	}
	return nResult;
}

static int ShSpiTps_SetReCalibration(SpiTpsRec *poSpiTpsRec)
{
	int nResult = -1;

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}
	else
	{
		/* interrupt disabled */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_disable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	/*+-------------------------------------------------------------------------+*/
	/*|	IDAC Calibration														|*/
	/*+-------------------------------------------------------------------------+*/
	nResult = ShSpiTps_Calibration();

	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}
	else
	{
		/* Resuming interrupt */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	return nResult;
}

static int ShSpiTps_FirstCalibration(void *poPt)
{
	SpiTpsRec *poSpiTpsRec = (SpiTpsRec *)poPt;
	int nI;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_FirstCalibration\n");
#endif	/* TPS_PRNLOG */
	memset(gSense, 0x00, TPS_SENSOR_READ_VAL);

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}
	else
	{
		/* interrupt disabled */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_disable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}

	for(nI = 0; nI < RETRY_MAX; nI++)
	{
		if(ShSpiTps_SensorCheck() == 0)
		{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]Calibration Sensor Check OK \n");
#endif	/* TPS_PRNLOG */
			break;
		}

		/* carry out calibration */
		if(ShSpiTps_Calibration() == 0)
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Calibration enforcement \n");
#endif	/* TPS_PRNERR */
		}
		else
		{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Calibration Error \n");
#endif	/* TPS_PRNERR */
		}
	}
	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}
	else
	{
		/* Resuming interrupt */
#ifdef TPS_IRQ_WAKE
		ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	mutex_unlock(&goTpsAccessMutex);
	return 0;
}

static int ShSpiTps_SensorCheck(void)
{
	uint8_t bSt = 0;
	uint8_t rx_buf[6];
	int nCnt;
	int nResult = 0;
	int bInt;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_SensorCheck\n");
#endif	/* TPS_PRNLOG */
	if(0 == ShSpiTps_SpiRead(0x00, rx_buf, 5))
	{
		bSt = rx_buf[4];
		/* Test mode 0 */
		bSt &= ~0x70;
		bSt |= 0x40;
		if(0 == ShSpiTps_SpiWriteOne(0x00, bSt, 1))
		{
			/* awaiting 150ms and get the first interrupt */
			msleep(150);

			for(nCnt = 0; nCnt < 500; nCnt++)
			{
				if(gpio_get_value(SH_TOUCH_IRQ) == 0)
				{
					/* acquire a sensor value */
					if(0 == ShSpiTps_SpiRead(0x00, gSense, TPS_SENSOR_READ_VAL))
					{
#ifdef TPS_PRNLOG
						for(nCnt = 0; nCnt < SHTPS_TMA_RXNUM_MAX; nCnt++)
						{
printk(KERN_DEBUG "[ShSpiTps]RX%02d %3d %3d %3d %3d %3d %3d %3d %3d\n", nCnt+1,
																gSense[SHTPS_TMA_OFFSET_MAX+(SHTPS_TMA_RXNUM_MAX*0+nCnt)],
																gSense[SHTPS_TMA_OFFSET_MAX+(SHTPS_TMA_RXNUM_MAX*1+nCnt)],
																gSense[SHTPS_TMA_OFFSET_MAX+(SHTPS_TMA_RXNUM_MAX*2+nCnt)],
																gSense[SHTPS_TMA_OFFSET_MAX+(SHTPS_TMA_RXNUM_MAX*3+nCnt)],
																gSense[SHTPS_TMA_OFFSET_MAX+(SHTPS_TMA_RXNUM_MAX*4+nCnt)],
																gSense[SHTPS_TMA_OFFSET_MAX+(SHTPS_TMA_RXNUM_MAX*5+nCnt)],
																gSense[SHTPS_TMA_OFFSET_MAX+(SHTPS_TMA_RXNUM_MAX*6+nCnt)],
																gSense[SHTPS_TMA_OFFSET_MAX+(SHTPS_TMA_RXNUM_MAX*7+nCnt)]);
						}
#endif	/* TPS_PRNLOG */
						/* interrupt cancellation */
						bInt = gSense[4];
						if(bInt & 0x80){
							bInt &= ~0x80;
						}else{
							bInt |= 0x80;
						}
						/* Normal mode */
						bInt &= ~0x70;
						if(0 == ShSpiTps_SpiWriteOne(0x00, bInt, 1))
						{
							msleep(40);

							/* check a sensor value */
							for(nCnt = 0; nCnt < TPS_SENSOR_SIZE; nCnt++)
							{
								if(gSense[SHTPS_TMA_OFFSET_MAX+nCnt] < TPS_CALIB_MIN || gSense[SHTPS_TMA_OFFSET_MAX+nCnt] > TPS_CALIB_MAX)
								{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Outside Range(Calibration) SNS[%d] = %d \n", nCnt, gSense[11+nCnt]);
#endif	/* TPS_PRNERR */
									nResult = 1;
								}
							}
							return nResult;
						}
					}
					break;
				}
			}
		}
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]SensorCheck Error\n");
#endif /* TPS_PRNERR */
		/* reboot it without calibration */
		ShSpiTps_PowerOn();
	}
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Forced Calibration!\n");
#endif /* TPS_PRNERR */
	return 1;
}

static int ShSpiTps_Reg_Read(SpiTpsRec *poSpiTpsRec, unsigned long arg)
{
	uint8_t rx_buf[5];
	struct shtps_ioctl_param param;
	
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Reg_Read\n");
#endif	/* TPS_PRNLOG */
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}
	if(0 == ShSpiTps_SpiRead(param.data[0], rx_buf, 5))
	{
		if(copy_to_user(((struct shtps_ioctl_param*)arg)->data, (uint8_t*)&rx_buf[4], 1)){
			return -EFAULT;
		}
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Reg_Read Err addr:%02x\n", param.data[0]);
#endif	/* TPS_PRNERR */
	}

	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}

	return 0;
}

static int ShSpiTps_Reg_Write(SpiTpsRec *poSpiTpsRec, unsigned long arg)
{
	struct shtps_ioctl_param param;

#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Reg_Write\n");
#endif	/* TPS_PRNLOG */
	if(0 == arg || 0 != copy_from_user(&param, (void __user *)arg, sizeof(param))){
		return -EINVAL;
	}

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}

	if(0 == ShSpiTps_SpiWriteOne(param.data[0],  param.data[1], 1))
	{
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Reg_Write Err addr:%02x\n", param.data[0]);
#endif	/* TPS_PRNERR */
	}

	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}

	return 0;
}

static int ShSpiTps_ChargerArmorEn(void *poPt)
{
	SpiTpsRec *poSpiTpsRec = (SpiTpsRec *)poPt;

	mutex_lock(&goTpsAccessMutex);
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_ChargerArmorEn\n");
#endif	/* TPS_PRNLOG */
	if(poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_SetChargerArmor(poSpiTpsRec);
	}
	mutex_unlock(&goTpsAccessMutex);
	return 0;
}
static void ShSpiTps_SetChargerArmor(SpiTpsRec *poSpiTpsRec)
{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_SetChargerArmor %d\n", poSpiTpsRec->mbIsChargerArmor);
#endif	/* TPS_PRNLOG */
	ShSpiTps_SpiWriteOne(0x1F, poSpiTpsRec->mbIsChargerArmor, 1);
}

#ifdef TPS_TPCT_COMMAND
static int ShSpiTps_SetSoftReset(SpiTpsRec *poSpiTpsRec)
{
	int nResult = -1;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_SetSoftReset(PID:%ld)\n", sys_getpid());
#endif	/* TPS_PRNLOG */

	/* The off state does power on */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOn();
	}
	else
	{
		/* interrupt disabled */
		disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}

	/*+-------------------------------------------------------------------------+*/
	/*|	Soft Reset																|*/
	/*+-------------------------------------------------------------------------+*/
	if(0 == ShSpiTps_SpiWriteOne(0x00, 0x01, 1))
	{
		nResult = 0;
		msleep(500);
	}
	else
	{
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Soft Reset Error\n");
#endif /* TPS_PRNERR */
	}

	/* return it to the off state */
	if(!poSpiTpsRec->mbIsEnable)
	{
		ShSpiTps_PowerOff();
	}
	else
	{
		/* Resuming interrupt */
		enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
	}
	return nResult;
}

static int ShSpiTps_Get_Touchinfo(SpiTpsRec *poSpiTpsRec, unsigned long arg)
{
	int i;
	struct shtps_touch_info info;
	
	for(i = 0; i < 4; i++)
	{
		if((gPrev[i].mbID == 0xFF) ||
		   (gPrev[i].mwPosX == 0xFFFF) || (gPrev[i].mwPosY == 0xFFFF) || (gPrev[i].mwPosZ == 0xFFFF)){
			diaginfo.fingers[i].id = 0;
			diaginfo.fingers[i].x  = 0;
			diaginfo.fingers[i].y  = 0;
			diaginfo.fingers[i].z  = 0;
		}else{
			diaginfo.fingers[i].id= gPrev[i].mbID;
			diaginfo.fingers[i].x = gPrev[i].mwPosX;
			diaginfo.fingers[i].y = gPrev[i].mwPosY;
			diaginfo.fingers[i].z = gPrev[i].mwPosZ;
		}
	}	
	
	memcpy(&info, &diaginfo, sizeof(diaginfo));
//	for(i = 0;i < 4;i++){
//		info.fingers[i].x = info.fingers[i].x * (((SH_TOUCH_LCD_V_MAX_X + 1) * 10000) / sh_sensor_max_x) / 10000;
//		info.fingers[i].y = info.fingers[i].y * (((SH_TOUCH_LCD_V_MAX_Y + 1) * 10000) / sh_sensor_max_y) / 10000;
//	}
	if(copy_to_user((uint8_t*)arg, (uint8_t*)&info, sizeof(info))){
		return -EFAULT;
	}
	
	return 0;
}

static int ShSpiTps_Get_Touchinfo_untrans(SpiTpsRec *poSpiTpsRec, unsigned long arg)
{
	if(copy_to_user((uint8_t*)arg, (uint8_t*)&diaginfo, sizeof(diaginfo))){
		return -EFAULT;
	}
	
	return 0;
}

static int ShSpiTps_Set_Touchinfo_mode(SpiTpsRec *poSpiTpsRec, unsigned long arg)
{
	poSpiTpsRec->diag.pos_mode = arg;

printk(KERN_DEBUG "[ShSpiTps]Touchinfo Mode = %02x\n", poSpiTpsRec->diag.pos_mode);

	if(poSpiTpsRec->diag.pos_mode & 0x80)
	{
printk(KERN_DEBUG "[ShSpiTps]monitor ON\n");
		poSpiTpsRec->diag.enable = 1;
	}
	else
	{
printk(KERN_DEBUG "[ShSpiTps]monitor OFF\n");
		poSpiTpsRec->diag.enable = 0;
	}

	return 0;
}
#endif	/* TPS_TPCT_COMMAND */

static int ShSpiTps_FirmUp_Set(SpiTpsRec *poSpiTpsRec, int nMode ,int nCheck)
{
#ifdef TPS_ISSP_ENABLE
	int nRetry;
	int nResult;
#endif	/* TPS_ISSP_ENABLE */

	if(nMode == 1)
	{
		poSpiTpsRec->mbIsUpdate = 1;
		if(poSpiTpsRec->mbIsEnable)
		{
			/* interrupt disabled */
#ifdef TPS_IRQ_WAKE
			ShSpiTps_irq_wake_disable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
			disable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
			ShtpsTps_Event_Complement(poSpiTpsRec);
			msleep(50);
		}
#ifdef TPS_ISSP_ENABLE
		for(nRetry = 0; nRetry <= 3; nRetry++)
		{
			nResult = ShSpiTps_ISSPport_Change(1);
			if(nResult == 0)
				break;
			printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_ISSPport_Change Error Retry\n");
		}
#endif	/* TPS_ISSP_ENABLE */
	}
	else
	{
		poSpiTpsRec->mbIsUpdate = 0;

		if(nCheck == 1 && 	poSpiTpsRec->mbIsActive == 0)
		{
			poSpiTpsRec->mbIsActive = 1;
			ShSpiTps_SetState(poSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_OFF, TPS_CHECK_OFF, TPS_RETURN_ON);
		}
		else
		{
			if(poSpiTpsRec->mbIsEnable)
			{
			/* Resuming interrupt */
#ifdef TPS_IRQ_WAKE
				ShSpiTps_irq_wake_enable(poSpiTpsRec);
#endif	/* TPS_IRQ_WAKE */
				enable_irq(MSM_GPIO_TO_INT(poSpiTpsRec->mnIrqPin));
			}
			else
			{
				/* Power Off */
				ShSpiTps_PowerOff();
			}
		}
		ShSpiTps_GetSensorSize(poSpiTpsRec);
	}
	return gnHwRev;
}

static int ShSpiTps_Gpio_Reset(int nMode)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]Gpio_Reset\n");
#endif	/* TPS_PRNDEB */

#ifdef TPS_ISSP_ENABLE
	if(nMode == 1)
	{
		ShSpiTps_ISSPport_Change(0);
	}
#endif	/* TPS_ISSP_ENABLE */
	gpio_direction_output(SH_TOUCH_RESET , 0);
	usleep(TPS_RESET_INTERVAL);
	gpio_direction_output(SH_TOUCH_RESET , 1);

	return 0;
}

static int ShSpiTps_Gpio_HsspClk(int nOnOff)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]Gpio_HsspClk (%d)\n", nOnOff);
#endif	/* TPS_PRNDEB */
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_SPI_HSSP_CLK , 0);
	} else {
		gpio_direction_output(SH_TOUCH_SPI_HSSP_CLK , 1);
	}
	return 0;
}

static int ShSpiTps_Gpio_HsspData(int nOnOff)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]Gpio_HsspData (%d)\n", nOnOff);
#endif	/* TPS_PRNDEB */
	/* if nOnOff = 0, Turn Off */
	if(nOnOff == 0) {
		gpio_direction_output(SH_TOUCH_SPI_HSSP_DATA, 0);
	} else {
		gpio_direction_output(SH_TOUCH_SPI_HSSP_DATA, 1);
	}
    return 0;
}

static int ShSpiTps_Gpio_HsspClkCh(int nInOut)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]HsspClkCh (%d)\n", nInOut);
#endif	/* TPS_PRNDEB */
	if(nInOut == 0)
	{
		gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_CLK, 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
	}
	else
	{
		gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
	}
    return 0;
}

static int ShSpiTps_Gpio_HsspDataCh(int nInOut)
{
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]HsspDataCh (%d)\n", nInOut);
#endif	/* TPS_PRNDEB */
	if(nInOut == 0)
	{
		gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_DATA, 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
	}
	else
	{
		gpio_tlmm_config(GPIO_CFG(SH_TOUCH_SPI_HSSP_DATA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
	}
    return 0;
}

static int ShSpiTps_SDATACheck(void)
{
	if(gpio_get_value(SH_TOUCH_SPI_HSSP_DATA) == 1) {
		return 1;
	}
	return 0;
}

static int ShSpiTps_RunClock(int nNumCycles)
{
	int nI;

	for(nI = 0; nI < nNumCycles; nI++)
	{
		SCLKLow();
		udelay(gdwWait);	/* Waiting */
		SCLKHigh();
		udelay(gdwWait);	/* Waiting */
	}
	return 0;
}

static uint8_t ShSpiTps_ReceiveBit(void)
{
	SCLKLow();
	udelay(gdwWait);	/* Waiting */
	SCLKHigh();
	udelay(gdwWait);	/* Waiting */
	return ShSpiTps_SDATACheck();
}

static uint8_t ShSpiTps_ReceiveByte(void)
{
	int nI;
	uint8_t bData = 0x00;

	for(nI = 0; nI < 8; nI++)
	{
		bData = (bData << 1) + ShSpiTps_ReceiveBit();
	}
#ifdef TPS_PRNDEB
printk(KERN_DEBUG "[ShSpiTps]ReceiveByte %x\n", bData);
#endif	/* TPS_PRNDEB */
	return bData;
}

static int ShSpiTps_SendByte(uint8_t bData, int nNumBits)
{
	int nI = 0;

	for(nI = 0; nI < nNumBits; nI++)
	{
		if(bData & 0x80)
		{
			/* Send a '1' */
			SetSDATAHigh();
			SCLKHigh();
			SCLKLow();
		}
		else
		{
			/* Send a '0' */
			SetSDATALow();
			SCLKHigh();
			SCLKLow();
		}
		bData = bData << 1;
	}
	return 0;
}

static int ShSpiTps_DetectHiLoTransition(void)
{
	int nCnt;
	int nIs_Response = -1;

	SetSDATAHiZ();

	for(nCnt = 0; (nCnt < 20000) && (nIs_Response == -1); nCnt++)
	{
		udelay(10);

		if(ShSpiTps_SDATACheck() == 0)
		{
			nIs_Response = 0;
		}
	}

	if(nIs_Response == -1){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]First Low Wait\n");
#endif	/* TPS_PRNERR */
	}

	nIs_Response = -1;

	for(nCnt = 0; (nCnt < 20000) && (nIs_Response == -1); nCnt++)
	{
		SCLKHigh();
		udelay(1);

		SCLKLow();
		udelay(10);

		if(ShSpiTps_SDATACheck() != 0)
		{
			nIs_Response = 0;
		}
	}

	if(nIs_Response == -1){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]High Wait\n");
#endif	/* TPS_PRNERR */
	}

	nIs_Response = -1;
	msleep(10);

	for(nCnt = 0; (nCnt < 20000) && (nIs_Response == -1); nCnt++)
	{
		udelay(10);

		if(ShSpiTps_SDATACheck() == 0)
		{
			nIs_Response = 0;
		}
	}
	if(nIs_Response == -1){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]Low Wait\n");
#endif	/* TPS_PRNERR */
	}

	SetSDATAStrong();
//	ShSpiTps_SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

	return nIs_Response;
}

/*+-----------------------------------------------------------------------------+*/
/*|	adjustment of Coordinate													|*/
/*+-----------------------------------------------------------------------------+*/
static void ShSpiTps_Qsort(Qsort_t *pTable, int nTop, int nEnd)
{
    int i, j;
    int nCenter;
	Qsort_t Swap;

    i = nTop;
    j = nEnd;

    nCenter = pTable[(nTop + nEnd) / 2].mValue;

	while(1)
	{
		while (pTable[i].mValue < nCenter)
			i++;
		while (nCenter < pTable[j].mValue)
			j--;
		if(i >= j)
			break;
		memcpy(&Swap, &pTable[i], sizeof(Qsort_t));
		memcpy(&pTable[i], &pTable[j], sizeof(Qsort_t));
		memcpy(&pTable[j], &Swap, sizeof(Qsort_t));
		i++;
		j--;
	}
	if(nTop < i - 1)
		ShSpiTps_Qsort(pTable, nTop, i - 1);
	if(j + 1 <  nEnd)
		ShSpiTps_Qsort(pTable, j + 1, nEnd);
}

static void ShSpiTps_RoundValue(short *pValue)
{
	Qsort_t pTable[6];
	int nI;

	for(nI = 0; nI < 6; nI++)
	{
		pTable[nI].mNo = nI;
		pTable[nI].mValue = pValue[nI];
	}
	ShSpiTps_Qsort(pTable, 0, 5);
	pValue[pTable[0].mNo] = pValue[pTable[1].mNo];
	pValue[pTable[5].mNo] = pValue[pTable[4].mNo];
}

static int ShSpiTps_SetAdjustParam(SpiTpsRec *poSpiTpsRec, uint16_t *pParam)
{
	int nI;
	TpsPoint_t sD[ADJUST_POINT];
	short nDiff[2][6];
	int nResult = 0;

	/* adjustment of Coordinate is Invalid */
	if(pParam == NULL)
	{
		poSpiTpsRec->mbAdjustEnable = 0;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_SetAdjustParam(NULL)\n");
#endif	/* TPS_PRNLOG */
#ifdef TPS_SETCALIBRATION
		nResult = ShSpiTps_SetReCalibration(poSpiTpsRec);
#endif	/* TPS_SETCALIBRATION */
		return nResult;
	}
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_SetAdjustParam(%4d,%4d)(%4d,%4d)\n", pParam[ 0], pParam[ 1], pParam[ 2], pParam[ 3]);
printk(KERN_DEBUG "                                 (%4d,%4d)(%4d,%4d)\n", pParam[ 4], pParam[ 5], pParam[ 6], pParam[ 7]);
printk(KERN_DEBUG "                                 (%4d,%4d)(%4d,%4d)\n", pParam[ 8], pParam[ 9], pParam[10], pParam[11]);
#endif	/* TPS_PRNLOG */

	/* Parameter checking :Effective range(+-100) */
	for(nI = 0; nI < ADJUST_POINT; nI++)
	{
		if(pParam[nI*2+0] > gBasePt[nI].x + POS_LIMIT ||
		   pParam[nI*2+0] < gBasePt[nI].x - POS_LIMIT)
			return -2;
		if(pParam[nI*2+1] > gBasePt[nI].y + POS_LIMIT ||
		   pParam[nI*2+1] < gBasePt[nI].y - POS_LIMIT)
			return -2;
	}

	/* Save Parameters */
	SET_POINT(gAdjustPrm[0], pParam[ 0], pParam[ 1]);
	SET_POINT(gAdjustPrm[1], pParam[ 2], pParam[ 3]);
	SET_POINT(gAdjustPrm[2], pParam[ 4], pParam[ 5]);
	SET_POINT(gAdjustPrm[3], pParam[ 6], pParam[ 7]);
	SET_POINT(gAdjustPrm[4], pParam[ 8], pParam[ 9]);
	SET_POINT(gAdjustPrm[5], pParam[10], pParam[11]);
#if 0	/* Changing the calculation method of diff */
	/* calculate diff value */
	for(nI = 0; nI < ADJUST_POINT; nI++)
	{
		sD[nI].x = (gAdjustPrm[nI].x - gBasePt[nI].x) * 3 / 4;
		sD[nI].y = (gAdjustPrm[nI].y - gBasePt[nI].y) * 3 / 4;
	}
#else
	for(nI = 0; nI < ADJUST_POINT; nI++)
	{
		nDiff[0][nI] = (gAdjustPrm[nI].x - gBasePt[nI].x);
		nDiff[1][nI] = (gAdjustPrm[nI].y - gBasePt[nI].y);
	}
	/* truncate the maximum and minimum */
	ShSpiTps_RoundValue(nDiff[0]);			/* X */
	ShSpiTps_RoundValue(nDiff[1]);			/* Y */
	for(nI = 0; nI < ADJUST_POINT; nI++)
	{
		sD[nI].x = nDiff[0][nI] * 75 / 100;
		sD[nI].y = nDiff[1][nI] * 75 / 100;
	}
#endif	/* Changing the calculation method of diff */
	/* store the blurring value of each four area corners */
	/*                     |-------p-------| |-------q-------| |-------r-------| |-------s-------|*/
	SET_AREA(gAreaDiff[ 0], 0      , 0      , sD[0].x, 0      , 0      , sD[0].y, sD[0].x, sD[0].y);
	SET_AREA(gAreaDiff[ 1], sD[0].x, 0      , sD[1].x, 0      , sD[0].x, sD[0].y, sD[1].x, sD[1].y);
	SET_AREA(gAreaDiff[ 2], sD[1].x, 0      , 0      , 0      , sD[1].x, sD[1].y, 0      , sD[1].y);
	SET_AREA(gAreaDiff[ 3], 0      , sD[0].y, sD[0].x, sD[0].y, 0      , sD[2].y, sD[2].x, sD[2].y);
	SET_AREA(gAreaDiff[ 4], sD[0].x, sD[0].y, sD[1].x, sD[1].y, sD[2].x, sD[2].y, sD[3].x, sD[3].y);
	SET_AREA(gAreaDiff[ 5], sD[1].x, sD[1].y, 0      , sD[1].y, sD[3].x, sD[3].y, 0      , sD[3].y);
	SET_AREA(gAreaDiff[ 6], 0      , sD[2].y, sD[2].x, sD[2].y, 0      , sD[4].y, sD[4].x, sD[4].y);
	SET_AREA(gAreaDiff[ 7], sD[2].x, sD[2].y, sD[3].x, sD[3].y, sD[4].x, sD[4].y, sD[5].x, sD[5].y);
	SET_AREA(gAreaDiff[ 8], sD[3].x, sD[3].y, 0      , sD[3].y, sD[5].x, sD[5].y, 0      , sD[5].y);
	SET_AREA(gAreaDiff[ 9], 0      , sD[4].y, sD[4].x, sD[4].y, 0      , 0      , sD[4].x, 0      );
	SET_AREA(gAreaDiff[10], sD[4].x, sD[4].y, sD[5].x, sD[5].y, sD[4].x, 0      , sD[5].x, 0      );
	SET_AREA(gAreaDiff[11], sD[5].x, sD[5].y, 0      , sD[5].y, sD[5].x, 0      , 0      , 0      );
	/* to valid an adjustment of Coordinate */
	poSpiTpsRec->mbAdjustEnable = 1;
	return 0;
}

static void ShSpiTps_AdjustPt(short *pX, short *pY)
{
	int nI;
	int32_t lXPQ;
	int32_t lXRS;
	int32_t lX;
	int32_t lYPR;
	int32_t lYQS;
	int32_t lY;

	/* divide the area */
	for(nI = 0; nI < AREA_COUNT; nI++)
	{
		if(gAreaRect[nI].p.x <= *pX && gAreaRect[nI].s.x > *pX &&
		   gAreaRect[nI].p.y <= *pY && gAreaRect[nI].s.y > *pY)
		{
			break;
		}
	}
	/* If not belong to any area, do not adjust */
	if(nI != AREA_COUNT)
	{
		/* do an adjustment of Coordinate */
		lXPQ = (((gAreaDiff[nI].q.x*DOUBLE_ACCURACY) - (gAreaDiff[nI].p.x*DOUBLE_ACCURACY)) /
				(gAreaRect[nI].q.x - gAreaRect[nI].p.x)) * (*pX - gAreaRect[nI].p.x) + (gAreaDiff[nI].p.x*DOUBLE_ACCURACY);
		lXRS = (((gAreaDiff[nI].s.x*DOUBLE_ACCURACY) - (gAreaDiff[nI].r.x*DOUBLE_ACCURACY)) /
				(gAreaRect[nI].s.x - gAreaRect[nI].r.x)) * (*pX - gAreaRect[nI].r.x) + (gAreaDiff[nI].r.x*DOUBLE_ACCURACY);
		lX   = ((lXRS - lXPQ) / (gAreaRect[nI].r.y - gAreaRect[nI].p.y)) * (*pY - gAreaRect[nI].p.y) + lXPQ;
		lYPR = (((gAreaDiff[nI].r.y*DOUBLE_ACCURACY) - (gAreaDiff[nI].p.y*DOUBLE_ACCURACY)) /
				(gAreaRect[nI].r.y - gAreaRect[nI].p.y)) * (*pY - gAreaRect[nI].p.y) + (gAreaDiff[nI].p.y*DOUBLE_ACCURACY);
		lYQS = (((gAreaDiff[nI].s.y*DOUBLE_ACCURACY) - (gAreaDiff[nI].q.y*DOUBLE_ACCURACY)) /
				(gAreaRect[nI].s.y - gAreaRect[nI].q.y)) * (*pY - gAreaRect[nI].q.y) + (gAreaDiff[nI].q.y*DOUBLE_ACCURACY);
		lY   = ((lYQS - lYPR) / (gAreaRect[nI].q.x - gAreaRect[nI].p.x)) * (*pX - gAreaRect[nI].p.x) + lYPR;
		*pX = *pX - (short)(lX / DOUBLE_ACCURACY);
		*pY = *pY - (short)(lY / DOUBLE_ACCURACY);
	}
	/* to be adjusted inside the range */
#if 0
	*pX = MINMAX(0, SH_TOUCH_LCD_V_MAX_X, *pX);
	*pY = MINMAX(0, SH_TOUCH_LCD_V_MAX_Y, *pY);
#else
	*pX = MINMAX(0, sh_sensor_max_x, *pX);
	*pY = MINMAX(0, sh_sensor_max_y, *pY);
#endif
}

static void ShSpiTps_PosInit(void)
{
	int nI;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_PosInit\n");
#endif	/* TPS_PRNLOG */
	/* clearing the Last Info */
	for(nI = 0; nI < 4; nI++)
	{
		gPrev[nI].mbID = 0xff;
		gPrev[nI].mwPosX = 0xffff;
		gPrev[nI].mwPosY = 0xffff;
		gPrev[nI].mwPosZ = 0xffff;
	}
	for(nI = 0; nI < 4; nI++)
	{
		gnDragStep[nI] = TPS_DRAG_THRESHOLD_1ST;
	}
}
static void ShSpiTps_PosSet(InputDev *pInDev, int nCnt, TpsEvent *poEv)
{
	int nI, nJ;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_PosSet(%d)\n", nCnt);
#endif	/* TPS_PRNLOG */

	/* update a finger notifying you of earlier */
	for(nI = 0; nI < 4; nI++)
	{
		/* Previously notified */
		if(gPrev[nI].mbID != 0xff)
		{
			/* do loop in count of event */
			for(nJ = 0; nJ < nCnt; nJ++)
			{
				/* Found the same number(Drag) */
				if(gPrev[nI].mbID == poEv[nJ].mbID)
				{
#ifdef TPS_EVENTLOG
printk(KERN_DEBUG "[Tps]DRAG(%d:%d %d,%4d,%4d,%4d)\n", nI, nJ, poEv[nJ].mbID, poEv[nJ].mwPosX, poEv[nJ].mwPosY, poEv[nJ].mwPosZ);
#endif	/* TPS_EVENTLOG */
					/* updating the coordinate */
					input_report_abs(pInDev, ABS_MT_TOUCH_MAJOR, 100); 
					input_report_abs(pInDev, ABS_MT_POSITION_X,  poEv[nJ].mwPosX);
					input_report_abs(pInDev, ABS_MT_POSITION_Y,  poEv[nJ].mwPosY);
					input_report_abs(pInDev, ABS_MT_WIDTH_MAJOR, poEv[nJ].mwPosZ);
					input_mt_sync(pInDev);
					/* updating the management information */
					gPrev[nI].mbID = poEv[nJ].mbID;
					gPrev[nI].mwPosX = poEv[nJ].mwPosX;
					gPrev[nI].mwPosY = poEv[nJ].mwPosY;
					gPrev[nI].mwPosZ = poEv[nJ].mwPosZ;
					/* handled flag */
					poEv[nJ].mbID = 0xff;
					if(nJ == nCnt-1)
					{
						ShSpiTps_Polling_Stop(gpoSpiTpsRec);
						ShSpiTps_Polling_Start(gpoSpiTpsRec);
					}
					break;
				}
			}
			/* event of do not touch(Touch up) */
			if(nJ == nCnt)
			{
#ifdef TPS_EVENTLOG
printk(KERN_DEBUG "[Tps]UP  (%d:- %d,%4d,%4d,%4d)\n", nI, gPrev[nI].mbID, gPrev[nI].mwPosX, gPrev[nI].mwPosY, gPrev[nI].mwPosZ);
#endif	/* TPS_EVENTLOG */
				/* notice of the previous Information */
				input_report_abs(pInDev, ABS_MT_TOUCH_MAJOR, 0); 
				input_report_abs(pInDev, ABS_MT_POSITION_X,  gPrev[nI].mwPosX);
				input_report_abs(pInDev, ABS_MT_POSITION_Y,  gPrev[nI].mwPosY);
				input_report_abs(pInDev, ABS_MT_WIDTH_MAJOR, gPrev[nI].mwPosZ);
				input_mt_sync(pInDev);
				/* clearing management information */
				gPrev[nI].mbID = 0xff;
				gPrev[nI].mwPosX = 0xffff;
				gPrev[nI].mwPosY = 0xffff;
				gPrev[nI].mwPosZ = 0xffff;
				ShSpiTps_Polling_Stop(gpoSpiTpsRec);
			}
		}
	}
/* packing before */
	for(nI = 0; nI < 3; nI++)
	{
		/* If vacant */
		if(gPrev[nI].mbID == 0xff)
		{
			for(nJ = nI + 1; nJ < 4; nJ++)
			{
				/* Searching in gap */
				if(gPrev[nJ].mbID != 0xff)
				{
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[Tps](%d<-%d)\n", nI, nJ);
#endif	/* TPS_PRNLOG */
					gPrev[nI].mbID = gPrev[nJ].mbID;
					gPrev[nI].mwPosX = gPrev[nJ].mwPosX;
					gPrev[nI].mwPosY = gPrev[nJ].mwPosY;
					gPrev[nI].mwPosZ = gPrev[nJ].mwPosZ;
					gPrev[nJ].mbID = 0xff;
					gPrev[nJ].mwPosX = 0xffff;
					gPrev[nJ].mwPosY = 0xffff;
					gPrev[nJ].mwPosZ = 0xffff;
					break;
				}
			}
		}
	}
	/* handling the remaining events */
	for(nJ = 0; nJ < nCnt; nJ++)
	{
		/* found the not handled event */
		if(poEv[nJ].mbID != 0xff)
		{
			for(nI = 0; nI < 4; nI++)
			{
				/* found the space */
				if(gPrev[nI].mbID == 0xff)
				{
#ifdef TPS_EVENTLOG
printk(KERN_DEBUG "[Tps]DOWN(%d:%d %d,%4d,%4d,%4d)\n", nI, nJ, poEv[nJ].mbID, poEv[nJ].mwPosX, poEv[nJ].mwPosY, poEv[nJ].mwPosZ);
#endif	/* TPS_EVENTLOG */
					/* updating the coordinate */
					input_report_abs(pInDev, ABS_MT_TOUCH_MAJOR, 100); 
					input_report_abs(pInDev, ABS_MT_POSITION_X,  poEv[nJ].mwPosX);
					input_report_abs(pInDev, ABS_MT_POSITION_Y,  poEv[nJ].mwPosY);
					input_report_abs(pInDev, ABS_MT_WIDTH_MAJOR, poEv[nJ].mwPosZ);
					input_mt_sync(pInDev);
					/* updating the management information */
					gPrev[nI].mbID = poEv[nJ].mbID;
					gPrev[nI].mwPosX = poEv[nJ].mwPosX;
					gPrev[nI].mwPosY = poEv[nJ].mwPosY;
					gPrev[nI].mwPosZ = poEv[nJ].mwPosZ;
					/* handled flag */
					poEv[nJ].mbID = 0xff;
					if(nJ == nCnt-1)
					{
						ShSpiTps_Polling_Stop(gpoSpiTpsRec);
						ShSpiTps_Polling_Start(gpoSpiTpsRec);
					}
					break;
				}
			}
		}
	}
#ifdef TPS_PRNLOG
	for(nI = 0; nI < 4; nI++)
	{
printk(KERN_DEBUG "[Tps][%d] %2X,%5d,%5d,%5d)\n", nI, gPrev[nI].mbID, gPrev[nI].mwPosX, gPrev[nI].mwPosY, gPrev[nI].mwPosZ);
	}
#endif	/* TPS_PRNLOG */
#ifdef TPS_TPCT_COMMAND
	if(gpoSpiTpsRec->diag.enable){
		for(nI = 0; nI < 4; nI++)
		{
			if((gPrev[nI].mbID == 0xFF) ||
			   (gPrev[nI].mwPosX == 0xFFFF) || (gPrev[nI].mwPosY == 0xFFFF) || (gPrev[nI].mwPosZ == 0xFFFF)){
				diaginfo.fingers[nI].id = 0;
				diaginfo.fingers[nI].x  = 0;
				diaginfo.fingers[nI].y  = 0;
				diaginfo.fingers[nI].z  = 0;
			}else{
				diaginfo.fingers[nI].id= gPrev[nI].mbID;
				diaginfo.fingers[nI].x = gPrev[nI].mwPosX;
				diaginfo.fingers[nI].y = gPrev[nI].mwPosY;
				diaginfo.fingers[nI].z = gPrev[nI].mwPosZ;
			}
		}
	}
#endif	/* TPS_TPCT_COMMAND */
}

static void ShSpiTps_GetSensorSize(SpiTpsRec *poSpiTpsRec)
{
#if 0
	uint8_t bVer;

	bVer = ShSpiTps_GetFwVer(poSpiTpsRec);
	if(bVer < 0x06)
	{
		sh_sensor_max_x = SH_SENSOR_MAX_X_V03;
		sh_sensor_max_y = SH_SENSOR_MAX_Y_V03;
	}
	else
	{
		sh_sensor_max_x = SH_SENSOR_MAX_X;
		sh_sensor_max_y = SH_SENSOR_MAX_Y;
	}
#else
	sh_sensor_max_x = SH_SENSOR_MAX_X;
	sh_sensor_max_y = SH_SENSOR_MAX_Y;
#endif
}
#ifdef FACETOUCH_DETECT_ENABLE
static int ShSpiTps_Start_Facetouchmode(SpiTpsRec *poSpiTpsRec, int nParam)
{
#ifdef	TPS_PRNDEB
	printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Start_Facetouchmode(%d)\n", nParam);
#endif	/* TPS_PRNDEB */
	if(nParam == 0)
	{
		poSpiTpsRec->facetouch.mode = 1;
		poSpiTpsRec->facetouch.off_detect = 0;
	}
	return 0;
}

static int ShSpiTps_Poll_Facetouchoff(SpiTpsRec *poSpiTpsRec)
{
	int rc;
#ifdef	TPS_PRNDEB
	printk(KERN_DEBUG "ShSpiTps_Poll_Facetouchoff Wait Event int\n");
	printk(KERN_DEBUG "facetouch.off_detect(%d), facetouch.mode(%d), facetouch.wake_sig(%d)\n",poSpiTpsRec->facetouch.off_detect, poSpiTpsRec->facetouch.mode, poSpiTpsRec->facetouch.wake_sig);
#endif	/* TPS_PRNDEB */
	rc = wait_event_interruptible(poSpiTpsRec->facetouch.wait_off, 
		(poSpiTpsRec->facetouch.off_detect == 1) || (poSpiTpsRec->facetouch.mode == 0) ||
		(poSpiTpsRec->facetouch.wake_sig == 1));
#ifdef	TPS_PRNDEB
	printk(KERN_DEBUG "%d = wait_event_interruptible\n",rc);
#endif	/* TPS_PRNDEB */
	poSpiTpsRec->facetouch.wake_sig = 0;
	
	if(poSpiTpsRec->facetouch.off_detect){
//#ifdef	TPS_PRNDEB
		printk(KERN_DEBUG "face touch off detect.\n");
//#endif	/* TPS_PRNDEB */
		rc = TPSDEV_FACETOUCHOFF_DETECT;
		poSpiTpsRec->facetouch.off_detect = 0;
	}else{
		rc = TPSDEV_FACETOUCHOFF_NOCHG;
	}
	
	return rc;
}

static int ShSpiTps_Ack_Facetouchoff(SpiTpsRec *poSpiTpsRec)
{
#ifdef	TPS_PRNDEB
	printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Ack_Facetouchoff()\n");
#endif	/* TPS_PRNDEB */
	ShSpiTps_Wake_Unlock(poSpiTpsRec);
	return 0;
}

static int ShSpiTps_Stop_Facetouchmode(SpiTpsRec *poSpiTpsRec, int nParam)
{
#ifdef	TPS_PRNDEB
	printk(KERN_DEBUG "[ShSpiTps]ShSpiTps_Stop_Facetouchmode(%d)\n", nParam);
#endif	/* TPS_PRNDEB */
	poSpiTpsRec->facetouch.mode = 0;
	if(poSpiTpsRec->facetouch.sleepfacetouch)
	{
		poSpiTpsRec->facetouch.sleepfacetouch = 0;
		ShSpiTps_SetState(poSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_ON, TPS_CHECK_OFF, TPS_RETURN_OFF);
	}
	wake_up_interruptible(&poSpiTpsRec->facetouch.wait_off);
	ShSpiTps_Wake_Unlock(poSpiTpsRec);
	return 0;
}
#endif	/* FACETOUCH_DETECT_ENABLE */

/*+-------------------------------------------------------------------------+*/
/*|	External public I/F														|*/
/*+-------------------------------------------------------------------------+*/
void msm_spitps_flipchange(int nFlipState)
{
	mutex_lock(&goTpsAccessMutex);
	if(gpoSpiTpsRec != NULL)
	{
		/* if a flip was opening */
		if(nFlipState == 0x00)
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_FLIP, TPS_DISABLE_OFF, TPS_CHECK_OFF, TPS_RETURN_OFF);
		else
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_FLIP, TPS_DISABLE_ON, TPS_CHECK_OFF, TPS_RETURN_OFF);
	}
	mutex_unlock(&goTpsAccessMutex);
}

void msm_tps_setsleep(int nIsSleep)
{
	mutex_lock(&goTpsAccessMutex);
	if(gpoSpiTpsRec != NULL)
	{
		/* if sleep has been lifted */
		if(nIsSleep == 0x00)
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_OFF, TPS_CHECK_OFF, TPS_RETURN_OFF);
		else
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_SLEEP, TPS_DISABLE_ON, TPS_CHECK_OFF, TPS_RETURN_OFF);
	}
	mutex_unlock(&goTpsAccessMutex);
}

void msm_tps_shutdown(void)
{
	int nIsActive = 0;
#ifdef TPS_PRNLOG
printk(KERN_DEBUG "[ShSpiTps]msm_tps_shutdown\n");
#endif	/* TPS_PRNLOG */
	mutex_lock(&goTpsAccessMutex);
	if(gpoSpiTpsRec != NULL)
	{
		/* If touchpanel driver is active */
		if(gpoSpiTpsRec->mbIsActive)
		{
			/* disabling a touchpanel */
			ShSpiTps_SetState(gpoSpiTpsRec, TPS_DISABLE_API, TPS_DISABLE_ON, TPS_CHECK_ON, TPS_RETURN_OFF);
			gpoSpiTpsRec->mbIsActive = 0;
			nIsActive = 1;
		}
		/* Power Off Sequence */
		ShSpiTps_Standby(0);
		ShSpiTps_Reset(1);
		usleep(TPS_RESET_INTERVAL);
		ShSpiTps_Reset(0);
		msleep(300);
		ShSpiTps_SpiWriteOne(0x1C, 0x01, 1);
		/* VCPIN(5V)OFF */
		ShSpiTps_Vcpin(0);
	}
	mutex_unlock(&goTpsAccessMutex);

	if(nIsActive == 1)
	{
		/* release the work-memory */
		flush_work(&gpoSpiTpsRec->moIrqWork);
	}
}

void msm_tps_set_chargerarmor(int nMode)
{
	struct task_struct *p;

	if(gpoSpiTpsRec != NULL)
	{
		gpoSpiTpsRec->mbIsChargerArmor = (uint8_t)nMode;
		p = kthread_run(ShSpiTps_ChargerArmorEn, gpoSpiTpsRec, "shspitps_chargerarmoren");
		/* If it cannot start a thread, doing the synchronous execution */
		if(IS_ERR(p)){
#ifdef TPS_PRNERR
printk(KERN_DEBUG "[ShSpiTps]msm_tps_set_chargerarmor Error\n");
#endif /* TPS_PRNERR */
		}
	}
}

