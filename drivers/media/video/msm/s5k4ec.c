/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* #define DEBUG */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>

#include <linux/regulator/consumer.h>
#include <linux/gpio.h> /* Individual support ICS */
#include <mach/vreg.h>
#include <mach/clk.h>
#include <linux/proc_fs.h>

#include "s5k4ec.h"

#if defined(CONFIG_SHCAM_TOOLS)
#include <sharp/sh_camera_tools.h>
#endif /* defined(CONFIG_SHCAM_TOOLS) */

#undef CDBG
#define CDBG(fmt, args...) do {} while (0)
#define CDBG_I2C(fmt, args...) do {} while (0)

#define S5K4EC_DEFAULT_MASTER_CLK_RATE      24000000
#define S5K4EC_I2C_RETRY_TIMES              10

#define S5K4EC_FOCUS_STEPS_MAX              0x20

#define S5K4EC_FOCUS_LOW_LIGHT              0x002E86F1

#define S5K4EC_FOCUS_SEARCH_IDLE            0x0000
#define S5K4EC_FOCUS_SEARCH_PROGRESS        0x0001
#define S5K4EC_FOCUS_SEARCH_SUCCESS         0x0002
#define S5K4EC_FOCUS_SEARCH_LOWCONF         0x0003
#define S5K4EC_FOCUS_SEARCH_CANCELED        0x0004
#define S5K4EC_FOCUS_SEARCH_RESTART_AE      0x0006
#define S5K4EC_FOCUS_SEARCH_RESTART_SCENE   0x0007
#define S5K4EC_FOCUS_SEARCH_RESET           0x0008

#define S5K4EC_FOCUS_SEARCH_FINISHED        0x00
#define S5K4EC_FOCUS_SEARCH_NOT_FINISHED    0x01

#define S5K4EC_FOCUS_POSITION_MIN           0x0000
#define S5K4EC_FOCUS_POSITION_MAX           0x00FF

#define S5K4EC_ALGORITHMSBIT_AE             0x0002
#define S5K4EC_ALGORITHMSBIT_AWB            0x0008
#define S5K4EC_ALGORITHMSBIT_FLICKER        0x0020

#define S5K4EC_READWRITE                    0x0F12
#define S5K4EC_FACTORY                      0xFCFC
#define S5K4EC_SETPAGE                      0x0028
#define S5K4EC_SETADDR                      0x002A

#define S5K4EC_WRITEALGO(a,b,c)  s5k4ec_autoalgbits_ctrl(a,b,c,\
									sizeof(c)/sizeof(c[0]))

struct s5k4ec_work {
	struct work_struct work;
};

struct __s5k4ec_ctrl {
	const struct msm_camera_sensor_info *sensordata;
	int sensormode;
	uint fps_divider; /* init to 1 * 0x00000400 */
	uint pict_fps_divider; /* init to 1 * 0x00000400 */
	u16 curr_step_pos;
	u16 curr_lens_pos;
	u16 init_curr_lens_pos;
	u16 my_reg_gain;
	u16 my_reg_line_count;
	enum msm_s_resolution prev_res;
	enum msm_s_resolution pict_res;
	enum msm_s_resolution curr_res;
	enum msm_s_test_mode  set_test;
	struct dimension_cfg dim_set;
	u16 zoom_ratio;
};

struct sensor_get_stat_info {
	uint8_t  mon_dbg[64];
	uint8_t  mon_skl[256];
	uint8_t  mon_setot[488];
	uint8_t  mon_afc[140];
	uint8_t  mon_awb[1628];
	uint8_t  mon_aaio[552];
	uint8_t  mon_ae[122];
	uint8_t  mon_af[232];
	uint16_t otp_func;
	uint16_t otp_data;
	uint16_t otp_awb;
	uint16_t otp_gas;
};

static DECLARE_WAIT_QUEUE_HEAD(s5k4ec_wait_queue);
DEFINE_MUTEX(s5k4ec_mutex);

static int S5K4EC_CSI_CONFIG;
static struct s5k4ec_work *s5k4ec_sensorw;
static struct i2c_client    *s5k4ec_client;
static u8 s5k4ec_counter;

static struct __s5k4ec_ctrl *s5k4ec_ctrl;

static unsigned short s5k4ec_af_macro_position = S5K4EC_FOCUS_POSITION_MACRO;

struct rw_semaphore ov_leds_list_lock;
struct list_head ov_leds_list;

static int s5k4ec_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id);

static int32_t s5k4ec_i2c_txdata(unsigned short saddr,
								 unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr	= saddr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	if (i2c_transfer(s5k4ec_client->adapter, msg, 1) < 0)
		return -EIO;
	else
		return 0;
}

static int32_t _s5k4ec_i2c_write(unsigned short waddr, unsigned short wdata,
		int trytimes)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4] = {0};

	s5k4ec_counter = 0;
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);

	while ((s5k4ec_counter < trytimes) && (rc != 0)) {
		rc = s5k4ec_i2c_txdata(s5k4ec_client->addr, buf, 4);
		if (rc < 0) {
			s5k4ec_counter++;
			CDBG("***--CAMERA i2c_write_w failed,i2c addr=0x%x,"
				"command addr = 0x%x, val = 0x%x,s=%d,"
					"rc=%d!\n", s5k4ec_client->addr, waddr, wdata,
					s5k4ec_counter, rc);
			msleep(20);
		}
	}
	return rc;
}

static int32_t s5k4ec_i2c_write(unsigned short waddr, unsigned short wdata,
		int trytimes)
{
	int32_t rc = 0;

	rc = _s5k4ec_i2c_write(waddr, wdata, trytimes);
	if (rc == 0) {
		CDBG_I2C("i2c_write addr = 0x%04x, val = 0x%04x\n", waddr, wdata);
	}
	return rc;
}

static int s5k4ec_i2c_rxdata(unsigned short saddr, unsigned char *rxdata,
		int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr	= saddr,
			.flags	= 0,
			.len	= 2,
			.buf	= rxdata,
		},
		{
			.addr	= saddr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	if (i2c_transfer(s5k4ec_client->adapter, msgs, 2) < 0) {
		pr_err("s5k4ec:[%d] i2c_rxdata failed!.\n", __LINE__);
		return -EIO;
	}

	return 0;
}

static int32_t _s5k4ec_i2c_read(unsigned short raddr,
							   unsigned short *rdata, int rlen)
{
	int32_t rc = -EIO;
	unsigned char buf[2] = {0};

	if (!rdata)
		return rc;

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k4ec_i2c_rxdata(s5k4ec_client->addr, buf, rlen);
	if (rc < 0) {
		pr_err("s5k4ec:[%d] i2c_read 0x%x failed!.\n", __LINE__, raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	return rc;
}

static int32_t s5k4ec_i2c_read(unsigned short raddr,
							   unsigned short *rdata, int rlen)
{
	int32_t rc = 0;

	rc = _s5k4ec_i2c_read(raddr, rdata, rlen);
	if (rc == 0) {
		CDBG_I2C("s5k4ec the i2c read is successful rdata = 0x%x\n",*rdata);
	}
	return rc;
}

static int32_t s5k4ec_i2c_reg_read(unsigned short raddr,
							   unsigned short *rdata)
{
	int ret = 0;
	unsigned short work = 0;

	ret = s5k4ec_i2c_write(0xFCFC, 0xD000, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = s5k4ec_i2c_write(0x002C, 0x7000, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = s5k4ec_i2c_write(0x002E, raddr, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = s5k4ec_i2c_read(0x0F12, &work, sizeof(unsigned short));
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_read failed!.\n", __LINE__);
		return -EIO;
	}
	*rdata = work;

	return 0;
}

static int32_t s5k4ec_i2c_reg_readsilent(unsigned short raddr,
										 unsigned short *rdata)
{
	int ret = 0;
	unsigned short work = 0;

	ret = _s5k4ec_i2c_write(0xFCFC, 0xD000, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = _s5k4ec_i2c_write(0x002C, 0x7000, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = _s5k4ec_i2c_write(0x002E, raddr, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = _s5k4ec_i2c_read(0x0F12, &work, sizeof(unsigned short));
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_read failed!.\n", __LINE__);
		return -EIO;
	}
	*rdata = work;

	return 0;
}

static int32_t s5k4ec_i2c_reg_readbyte(unsigned short raddr,
									   unsigned char *rdata, int rlen)
{
	int ret = 0, i, rdsiz;
	unsigned short work = 0, *wptr;

	if (!rdata)
		return -EINVAL;

	ret = s5k4ec_i2c_write(0xFCFC, 0xD000, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = s5k4ec_i2c_write(0x002C, 0x7000, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = s5k4ec_i2c_write(0x002E, raddr, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}

	for(i = 0; i < rlen; i += 2) {
		work = 0;
		rdsiz = (rlen - i) == 1 ? 1 : 2;
		ret = _s5k4ec_i2c_read(0x0F12, &work, rdsiz);
		if(0 != ret) {
			pr_err("s5k4ec:[%d] i2c_read failed!.\n", __LINE__);
			return -EIO;
		}
		if (rdsiz == 2) {
			wptr = (unsigned short *)rdata;
			*wptr = work;
		}
		else {
			*rdata = (unsigned char)(work & 0x00FF);
			if (rlen == 1) {
				CDBG_I2C("s5k4ec the i2c read is successful rdata = 0x%02x\n",
						 *rdata);
			}
		}
		rdata += 2;
	}

	return 0;
}

static int32_t s5k4ec_i2c_reg_multiread(unsigned short raddr,
							   unsigned short *rdata, int rlen)
{
	int ret = 0, i;
	unsigned short work = 0;

	ret = s5k4ec_i2c_write(0xFCFC, 0xD000, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = s5k4ec_i2c_write(0x002C, 0x7000, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	ret = s5k4ec_i2c_write(0x002E, raddr, S5K4EC_I2C_RETRY_TIMES);
	if(0 != ret) {
		pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
		return -EIO;
	}
	for(i = 0; i < rlen; i++) {
		ret = s5k4ec_i2c_read(0x0F12, &work, sizeof(unsigned short));
		if(0 != ret) {
			pr_err("s5k4ec:[%d] i2c_read failed!.\n", __LINE__);
			return -EIO;
		}
		*rdata = work;
		rdata++;
	}

	return 0;
}

static int32_t _s5k4ec_writepregs(struct s5k4ec_sensor *ptb, int32_t len)
{
	int32_t i, ret = 0;

	for (i = 0; i < len; i++) {
		ret = _s5k4ec_i2c_write(ptb[i].addr, ptb[i].data, S5K4EC_I2C_RETRY_TIMES);
		if(0 != ret) {
			pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
			break;
		}
	}
	return ret;
}

static int32_t s5k4ec_writepregs(struct s5k4ec_sensor *ptb, int32_t len)
{
	int32_t i, ret = 0;

	for (i = 0; i < len; i++) {
		ret = s5k4ec_i2c_write(ptb[i].addr, ptb[i].data, S5K4EC_I2C_RETRY_TIMES);
		if(0 != ret) {
			pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
			break;
		}
	}
	return ret;
}

static int s5k4ec_reg_srch(unsigned short sreg, 
						   struct s5k4ec_sensor *ptb, int32_t len)
{
	int i;
	unsigned short setadr = 0;

	for(i = 0; i < len; i++) {
		switch(ptb[i].addr) {
		case S5K4EC_SETADDR:
			setadr = ptb[i].data;
			break;

		case S5K4EC_READWRITE:
			if (setadr != 0) {
				if (setadr == sreg) {
					return i;
				}
				setadr += sizeof(unsigned short);
			}
			break;

		default:
			setadr = 0;
			break;
		}
	}
	return -1;
}

static int s5k4ec_autoalgbits_ctrl(int bctl, unsigned short reqbit,
								   struct s5k4ec_sensor *ptb, int32_t len)
{
	int rc = 0, idx;
	unsigned short rreg = 0, maskbit = 0;

	idx = s5k4ec_reg_srch(REG_TC_DBG_AutoAlgEnBits, ptb, len);

	if (idx >= 0) {
		rc = s5k4ec_i2c_reg_read(REG_TC_DBG_AutoAlgEnBits, &rreg);
		if (rc != 0) {
			pr_err("s5k4ec:[%d] reg read error!!\n", __LINE__);
			return rc;
		}

		if (bctl) {
			maskbit = rreg | reqbit;		/* on */
		}
		else {
			maskbit = rreg & (~reqbit);		/* off */
		}

		maskbit |= 0x0100;	/* bit8 on */

		ptb[idx].data = maskbit;
	}

	rc = s5k4ec_writepregs(ptb, len);
	return rc;
}

static int s5k4ec_mode_upadate_chk(unsigned short raddr)
{
	int rc = 0;
	int cnt = 0;
	unsigned short work = 0;

	CDBG("%s Check start. addr=0x%04x\n", __func__, raddr);
	while(1) {
		work = 0;
		rc = s5k4ec_i2c_reg_readsilent(raddr, &work);
		if(0 != rc) {
			return rc;
		}
		else {
			if(work != 0x0000)
			{
				if(cnt == 2000){
					pr_err("s5k4ec update timeout error!! reg=0x%x\n", work);
					return -ETIMEDOUT;
				}
				cnt++;
				usleep(200);
				continue;
			}
			else
			{
				CDBG("%s Check OK. cnt=%d\n", __func__, cnt);
				break;
			}
		}
	}
	return 0;
}

static int sensor_set_zoom(int setreq)
{
	int rc = 0;

	if (setreq || s5k4ec_ctrl->zoom_ratio != 0x0100) {
		s5k4ec_sensor_zoom_tbl[ZOOM_RATIO_IDX].data =
						(unsigned short)s5k4ec_ctrl->zoom_ratio;

		rc = S5K4ECCORE_WRITEPREG(s5k4ec_sensor_zoom_tbl);
		if (rc != 0) {
			return rc;
		}

		rc = S5K4ECCORE_WRITEPREG(s5k4ec_sensor_zoom_change_tbl);
		if (rc != 0) {
			return rc;
		}
		usleep(500);

		rc = s5k4ec_mode_upadate_chk(REG_TC_PZOOM_ePzoomState);
		if (rc != 0) {
			pr_err("s5k4ec:[%d] zoom error!!\n", __LINE__);
			return rc;
		}
	}

	return rc;
}

static int sensor_input_set(enum msm_s_setting rt)
{
	unsigned short pre_w = 0, pre_h = 0, cap_w = 0, cap_h = 0;
	unsigned short in_w, in_h, ofs_w, ofs_h;
	struct input_request *in_set;
	int mode = 0;

	if (S_RES_PREVIEW == rt) {
		in_set = &s5k4ec_ctrl->dim_set.pre_input;
		pre_w = (unsigned short)s5k4ec_ctrl->dim_set.preview_width;
		pre_h = (unsigned short)s5k4ec_ctrl->dim_set.preview_height;
		CDBG("%s:set preview w:%d h:%d\n", __func__, pre_w, pre_h);

		/* preview size */
		s5k4ec_preview_tbl[PREVIEW_SIZE_W_IDX].data = pre_w;
		s5k4ec_preview_tbl[PREVIEW_SIZE_H_IDX].data = pre_h;

		in_w  = (unsigned short)in_set->ins.in_width;
		in_h  = (unsigned short)in_set->ins.in_height;
		ofs_w = (unsigned short)in_set->ins.ofs_width;
		ofs_h = (unsigned short)in_set->ins.ofs_height;
		CDBG("%s:preview input w:%d h:%d, offset w:%d h:%d\n", __func__,
			 in_w, in_h, ofs_w, ofs_h);
		/* preview input */
		s5k4ec_sensor_preview_input_tbl[PRE_IN_W_IDX].data	= in_w;
		s5k4ec_sensor_preview_input_tbl[PRE_IN_H_IDX].data	= in_h;
		s5k4ec_sensor_preview_input_tbl[PRE_OFS_W_IDX].data = ofs_w;
		s5k4ec_sensor_preview_input_tbl[PRE_OFS_H_IDX].data = ofs_h;

		in_w  = (unsigned short)in_set->zms.in_width;
		in_h  = (unsigned short)in_set->zms.in_height;
		ofs_w = (unsigned short)in_set->zms.ofs_width;
		ofs_h = (unsigned short)in_set->zms.ofs_height;
		CDBG("%s:preview zoom  w:%d h:%d, offset w:%d h:%d\n", __func__,
			 in_w, in_h, ofs_w, ofs_h);
		/* preview zoom input */
		s5k4ec_sensor_preview_zoom_tbl[PZOOM_IN_W_IDX].data  = in_w;
		s5k4ec_sensor_preview_zoom_tbl[PZOOM_IN_H_IDX].data  = in_h;
		s5k4ec_sensor_preview_zoom_tbl[PZOOM_OFS_W_IDX].data = ofs_w;
		s5k4ec_sensor_preview_zoom_tbl[PZOOM_OFS_H_IDX].data = ofs_h;

		if (pre_w == SENSOR_QTR_W) {
			mode = 1;
		}
	}
	else {
		in_set = &s5k4ec_ctrl->dim_set.cap_input;
		cap_w = (unsigned short)s5k4ec_ctrl->dim_set.capture_width;
		cap_h = (unsigned short)s5k4ec_ctrl->dim_set.capture_height;
		CDBG("%s:set capture w:%d h:%d\n", __func__, cap_w, cap_h);

		/* capture size */
		s5k4ec_capture_tbl[CAPTURE_SIZE_W_IDX].data = cap_w;
		s5k4ec_capture_tbl[CAPTURE_SIZE_H_IDX].data = cap_h;

		in_w  = (unsigned short)in_set->ins.in_width;
		in_h  = (unsigned short)in_set->ins.in_height;
		ofs_w = (unsigned short)in_set->ins.ofs_width;
		ofs_h = (unsigned short)in_set->ins.ofs_height;
		CDBG("%s:capture input w:%d h:%d, offset w:%d h:%d\n", __func__,
			 in_w, in_h, ofs_w, ofs_h);
		/* capture input */
		s5k4ec_sensor_capture_input_tbl[CAP_IN_W_IDX].data  = in_w;
		s5k4ec_sensor_capture_input_tbl[CAP_IN_H_IDX].data  = in_h;
		s5k4ec_sensor_capture_input_tbl[CAP_OFS_W_IDX].data = ofs_w;
		s5k4ec_sensor_capture_input_tbl[CAP_OFS_H_IDX].data = ofs_h;

		in_w  = (unsigned short)in_set->zms.in_width;
		in_h  = (unsigned short)in_set->zms.in_height;
		ofs_w = (unsigned short)in_set->zms.ofs_width;
		ofs_h = (unsigned short)in_set->zms.ofs_height;
		CDBG("%s:capture zoom  w:%d h:%d, offset w:%d h:%d\n", __func__,
			 in_w, in_h, ofs_w, ofs_h);
		/* capture zoom input */
		s5k4ec_sensor_capture_zoom_tbl[CZOOM_IN_W_IDX].data  = in_w;
		s5k4ec_sensor_capture_zoom_tbl[CZOOM_IN_H_IDX].data  = in_h;
		s5k4ec_sensor_capture_zoom_tbl[CZOOM_OFS_W_IDX].data = ofs_w;
		s5k4ec_sensor_capture_zoom_tbl[CZOOM_OFS_H_IDX].data = ofs_h;
	}

	return mode;
}

static int sensor_preview_set(void)
{
	int rc = 0, mode;
	unsigned short rreg = 0;

	/* preview stop */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_preview_stop_tbl);
	if (rc != 0) {
		pr_err("s5k4ec:[%d] stop preview error!!\n", __LINE__);
		return rc;
	}
	msleep(1);

	/* input setting */
	mode = sensor_input_set(S_RES_PREVIEW);

	/* clos */
	if (mode) {
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_cols_qtr_tbl);
		if (rc != 0) {
			return rc;
		}
	}
	else {
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_cols_normal_tbl);
		if (rc != 0) {
			return rc;
		}
	}

	/* preview input */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_sensor_preview_input_tbl);
	if (rc != 0) {
		return rc;
	}

	/* preview zoom input */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_sensor_preview_zoom_tbl);
	if (rc != 0) {
		return rc;
	}

	/* preview setting */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_preview_tbl);
	if (rc != 0) {
		return rc;
	}

	/* init params update */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_init_params_tbl);
	if (rc != 0) {
		return rc;
	}
	msleep(10);

	/* preview changed */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_preview_changed_tbl);
	if (rc != 0) {
		return rc;
	}

	rc = s5k4ec_i2c_reg_read(REG_TC_GP_ErrorPrevConfig, &rreg);
	if (rc != 0) {
		return rc;
	}
	if (rreg != 0) {
		pr_err("s5k4ec:[%d] preview set error!! status reg=0x%x\n",
				__LINE__, rreg);
		return -EINVAL;
	}

	rc = s5k4ec_mode_upadate_chk(REG_TC_GP_NewConfigSync);
	if (rc != 0) {
		pr_err("s5k4ec:[%d] preview set sync error!!\n", __LINE__);
		return rc;
	}

	/* preview start */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_preview_start_tbl);
	if (rc != 0) {
		pr_err("s5k4ec:[%d] start preview error!!\n", __LINE__);
		return rc;
	}
	msleep(1);

	return rc;
}

static int sensor_capture_set(void)
{
	int rc = 0, mode;
	unsigned short rreg = 0;

	/* preview stop */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_preview_stop_tbl);
	if (rc != 0) {
		pr_err("s5k4ec:[%d] stop preview error!!\n", __LINE__);
		return rc;
	}
	msleep(1);

	/* input setting */
	mode = sensor_input_set(S_RES_CAPTURE);

	/* clos */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_cols_normal_tbl);
	if (rc != 0) {
		return rc;
	}

	/* capture input */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_sensor_capture_input_tbl);
	if (rc != 0) {
		return rc;
	}

	/* capture zoom input */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_sensor_capture_zoom_tbl);
	if (rc != 0) {
		return rc;
	}

	/* capture setting */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_capture_tbl);
	if (rc != 0) {
		return rc;
	}

	/* init params update */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_init_params_tbl);
	if (rc != 0) {
		return rc;
	}
	msleep(10);

	/* capture changed */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_capture_changed_tbl);
	if (rc != 0) {
		return rc;
	}
	msleep(1);

	rc = s5k4ec_i2c_reg_read(REG_TC_GP_ErrorCapConfig, &rreg);
	if (rc != 0) {
		return rc;
	}
	if (rreg != 0) {
		pr_err("s5k4ec:[%d] capture set error!! status reg=0x%x\n",
				__LINE__, rreg);
		return -EINVAL;
	}

	rc = s5k4ec_mode_upadate_chk(REG_TC_GP_NewConfigSync);
	if (rc != 0) {
		pr_err("s5k4ec:[%d] capture set sync error!!\n", __LINE__);
		return rc;
	}

	return rc;
}

static int s5k4ec_video_config(int csi_config)
{
	int rc = 0;
	unsigned short rreg = 0;

	CDBG("--CAMERA-- s5k4ec_video_config\n");

	if (csi_config) {
		/* capture stop */
		rc = s5k4ec_i2c_reg_read(REG_TC_GP_EnableCapture, &rreg);
		if (rc != 0) {
			return rc;
		}
		if (rreg != 0) {
			rc = S5K4ECCORE_WRITEPREG(s5k4ec_capture_stop_tbl);
			if (rc != 0) {
				return rc;
			}
			usleep(500);
		}
	}
	else {
		usleep(500);
	}

	/* preview setting */
	rc = sensor_preview_set();
	if (rc != 0) {
		return rc;
	}

	/* zoom set */
	rc = sensor_set_zoom(0);
	if (rc != 0) {
		return rc;
	}

	return rc;
}

static int s5k4ec_snapshot_config(void)
{
	int rc = 0;

	CDBG("--CAMERA-- SENSOR_SNAPSHOT_MODE\n");

	/* capture setting */
	rc = sensor_capture_set();
	if (rc != 0) {
		return rc;
	}
	msleep(1);

	/* zoom set */
	rc = sensor_set_zoom(0);
	if (rc != 0) {
		return rc;
	}

	/* capture start */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_capture_start_tbl);
	if (rc != 0) {
		return rc;
	}
	msleep(1);

	rc = s5k4ec_mode_upadate_chk(REG_TC_GP_NewConfigSync);
	if (rc != 0) {
		pr_err("s5k4ec:[%d] capture start error!!\n", __LINE__);
		return rc;
	}
	msleep(100);

	return rc;
}

static int s5k4ec_setting(enum msm_s_reg_update rupdate,
		enum msm_s_setting rt)
{
	int rc = -EINVAL;
	struct msm_camera_csi_params s5k4ec_csi_params;
	int csi_config = S5K4EC_CSI_CONFIG;

#if defined(S5K4EC_IIC_BURSTMODE_WRITE)
	int i = 0, retryend = 0;
#endif /* defined(S5K4EC_IIC_BURSTMODE_WRITE) */

	CDBG("--CAMERA-- %s (Start...), rupdate=%d\n", __func__, rupdate);

	switch (rupdate) {
	case S_UPDATE_PERIODIC:
		if (!S5K4EC_CSI_CONFIG) {
			s5k4ec_csi_params.lane_cnt = 2;
			s5k4ec_csi_params.data_format = CSI_8BIT;
			s5k4ec_csi_params.lane_assign = 0xe4;
			s5k4ec_csi_params.dpcm_scheme = 0;
			s5k4ec_csi_params.settle_cnt = 0x6;

			CDBG("%s: msm_camio_csi_config\n", __func__);

			rc = msm_camio_csi_config(&s5k4ec_csi_params);
			msleep(20);

			S5K4EC_CSI_CONFIG = 1;

		} else {
			rc = 0;
		}

		if (S_RES_PREVIEW == rt)
			rc = s5k4ec_video_config(csi_config);
		else if (S_RES_CAPTURE == rt)
			rc = s5k4ec_snapshot_config();

		break; /* UPDATE_PERIODIC */

	case S_REG_INIT:
		CDBG("--CAMERA-- S_REG_INIT (Start)\n");

#if defined(S5K4EC_IIC_BURSTMODE_WRITE)
		do {
			/* set sensor init Part1 setting */
			CDBG("set sensor init Part1 setting\n");
			rc = S5K4ECCORE_WRITEPREGSILENT(s5k4ec_init_1_tbl);
			if (rc < 0) {
				pr_err("s5k4ec: sensor init_1_tbl setting failed!.\n");
				retryend = 1;
				break;
			}
			msleep(10);

			/* set sensor init Part2 setting */
			CDBG("set sensor init Part2 setting\n");
			rc = s5k4ec_i2c_multipleWrite(s5k4ec_init_2_tbl,
					sizeof(s5k4ec_init_2_tbl) /
						sizeof(s5k4ec_init_2_tbl[0]));
			if (rc < 0) {
				pr_err("s5k4ec: sensor init_2_tbl setting failed!.\n");
				retryend = 1;
				break;
			}
			usleep(50);

			/* set sensor init PartOtp setting */
			if(1 == shcam_get_smem_otp_flg()) {
				CDBG("set sensor init OTP used setting\n");
				rc = S5K4ECCORE_WRITEPREGSILENT(s5k4ec_init_useOtp_tbl);
			}
			else {
				CDBG("set sensor init OTP not used setting\n");
				rc = S5K4ECCORE_WRITEPREGSILENT(s5k4ec_init_notUseOtp_tbl);
			}
			if (rc < 0) {
				pr_err("s5k4ec: sensor init_otp_tbl setting failed!.\n");
				retryend = 1;
				break;
			}
			usleep(50);

			/* set sensor init Part3 setting */
			CDBG("set sensor init Part3 setting\n");
			rc = s5k4ec_i2c_multipleWrite(s5k4ec_init_3_tbl,
					sizeof(s5k4ec_init_3_tbl) /
						sizeof(s5k4ec_init_3_tbl[0]));
			if (rc < 0) {
				pr_err("s5k4ec: sensor init_3_tbl setting failed!.\n");
				retryend = 1;
				break;
			}
			usleep(1000);

			rc = s5k4ec_mode_upadate_chk(REG_TC_GP_NewConfigSync);
			if(rc == -ETIMEDOUT) {
				pr_err("s5k4ec:[%d] s5k4ec_mode_upadate_chk retry %d\n",
						__LINE__, i);
				i++;
			}
			else if(rc != 0) {
				pr_err("s5k4ec:[%d] s5k4ec_mode_upadate_chk err\n",__LINE__);
				retryend = 1;
				break;
			}
		} while(rc < 0 && i < 5 && retryend == 0);
#else
		/* set sensor init Part1 setting */
		CDBG("set sensor init setting\n");
		rc = S5K4ECCORE_WRITEPREGSILENT(s5k4ec_init_1_tbl);
		if (rc < 0) {
			pr_err("s5k4ec: sensor init_1_tbl setting failed!.\n");
			break;
		}
		msleep(10);
		/* set sensor init Part2 setting */
		rc = S5K4ECCORE_WRITEPREGSILENT(s5k4ec_init_2_tbl);
		if (rc < 0) {
			pr_err("s5k4ec: sensor init_2_tbl setting failed!.\n");
			break;
		}
		usleep(50);

		/* set sensor init PartOtp setting */
		if(1 == shcam_get_smem_otp_flg()) {
			rc = S5K4ECCORE_WRITEPREGSILENT(s5k4ec_init_useOtp_tbl);
		}
		else {
			rc = S5K4ECCORE_WRITEPREGSILENT(s5k4ec_init_notUseOtp_tbl);
		}
		if (rc < 0) {
			pr_err("s5k4ec: sensor init_otp_tbl setting failed!.\n");
			break;
		}
		usleep(50);

		/* set sensor init Part3 setting */
		rc = S5K4ECCORE_WRITEPREGSILENT(s5k4ec_init_3_tbl);
		if (rc < 0) {
			pr_err("s5k4ec: sensor init_3_tbl setting failed!.\n");
			break;
		}
		usleep(500);

		rc = s5k4ec_mode_upadate_chk(REG_TC_GP_NewConfigSync);
		if(rc != 0) {
			pr_err("s5k4ec:[%d] s5k4ec_mode_upadate_chk err\n", __LINE__);
		}
#endif /* defined(S5K4EC_IIC_BURSTMODE_WRITE) */

		CDBG("--CAMERA-- S_REG_INIT (End)\n");
		break; /* case REG_INIT: */

	default:
		break;
	} /* switch (rupdate) */

	CDBG("--CAMERA-- %s (End), rupdate=%d\n", __func__, rupdate);

	return rc;
}

static int s5k4ec_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = -ENOMEM;

	CDBG("--CAMERA-- %s\n", __func__);
	s5k4ec_ctrl = kzalloc(sizeof(struct __s5k4ec_ctrl), GFP_KERNEL);
	if (!s5k4ec_ctrl) {
		pr_err("s5k4ec:[%d] kzalloc s5k4ec_ctrl error !!\n", __LINE__);
		kfree(s5k4ec_ctrl);
		return rc;
	}

	s5k4ec_ctrl->fps_divider = 1 * 0x00000400;
	s5k4ec_ctrl->pict_fps_divider = 1 * 0x00000400;
	s5k4ec_ctrl->set_test = S_TEST_OFF;
	s5k4ec_ctrl->prev_res = S_QTR_SIZE;
	s5k4ec_ctrl->pict_res = S_FULL_SIZE;
	s5k4ec_ctrl->dim_set.preview_width  = 640;
	s5k4ec_ctrl->dim_set.preview_height = 480;
	s5k4ec_ctrl->zoom_ratio = 0x0100;

	if (data)
		s5k4ec_ctrl->sensordata = data;

	rc = s5k4ec_setting(S_REG_INIT, S_RES_PREVIEW);
	if (rc < 0) {
		pr_err("s5k4ec:[%d] s5k4ec_setting failed!. rc = %d\n",
				__LINE__, rc);
		kfree(s5k4ec_ctrl);
		return rc;
	}

	S5K4EC_CSI_CONFIG = 0;

	CDBG("--CAMERA--re_init_sensor ok!!\n");
	return rc;
}

static int32_t s5k4ec_power_down(void)
{
	int32_t rc;
	do {
		rc = (int32_t)S5K4ECCORE_WRITEPREG(s5k4ec_preview_stop_tbl);
		if(rc != 0) {
			pr_err("s5k4ec:[%d] s5k4ec_preview_stop_tbl write error.\n",
							 __LINE__);
			break;
		}
		usleep(50);

		rc = S5K4ECCORE_WRITEPREG(s5k4ec_capture_stop_tbl);
		if(rc != 0) {
			pr_err("s5k4ec:[%d] s5k4ec_capture_stop_tbl write error.\n",
							 __LINE__);
			break;
		}
		usleep(500);

		rc = s5k4ec_mode_upadate_chk(REG_TC_GP_NewConfigSync);
		if (rc != 0) {
			pr_err("s5k4ec:[%d] power down config sync error!!\n", __LINE__);
		}

	} while(0);

	return rc;
}

static int s5k4ec_sensor_release(void)
{
	int32_t rc;

	CDBG("--CAMERA--s5k4ec_sensor_release!!\n");

	mutex_lock(&s5k4ec_mutex);

	rc = s5k4ec_power_down();

	if(s5k4ec_ctrl != NULL) {
		kfree(s5k4ec_ctrl);
		s5k4ec_ctrl = NULL;
	}

	S5K4EC_CSI_CONFIG = 0;

	mutex_unlock(&s5k4ec_mutex);
	return 0;
}

static const struct i2c_device_id s5k4ec_i2c_id[] = {
	{"s5k4ec",  0}, {}
};

static int s5k4ec_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k4ec_wait_queue);
	return 0;
}

static long s5k4ec_set_effect(int mode, int effect)
{
	int rc = 0;

	CDBG("--CAMERA-- %s ...effect = %d\n", __func__ , effect);
	if (mode != SENSOR_PREVIEW_MODE) {
		CDBG("%s:not preview. mode=%d\n", __func__, mode);
		return 0;
	}

	s5k4ec_effect_tbl[EFFECT_IDX].data = (unsigned short)effect;

	/* effect set */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_effect_tbl);

	return rc;
}

static int s5k4ec_set_brightness(int8_t brightness)
{
	int rc = 0;

	CDBG("--CAMERA-- %s ...brightness = %d\n", __func__ , brightness);

	switch(brightness) {
	case 0:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_brightness_lvm2_tbl);
		break;
	case 1:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_brightness_lvm1_tbl);
		break;
	case 2:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_brightness_lv0_tbl);
		break;
	case 3:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_brightness_lvp1_tbl);
		break;
	case 4:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_brightness_lvp2_tbl);
		break;
	default:
		pr_err("s5k4ec:[%d] brightness is invalid!! %d\n", brightness,
				__LINE__);
		return -EINVAL;
	}

	return rc;
}

static int s5k4ec_set_gamma(int gamma)
{
	int rc = 0;

	CDBG("--CAMERA-- %s ...gamma = %d\n", __func__ , gamma);

	switch(gamma) {
	case CAMERA_GAMMA_WIDE:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_gamma_wide_tbl);
		break;
	case CAMERA_GAMMA_LIMITED:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_gamma_limited_tbl);
		break;
	default:
		pr_err("s5k4ec:[%d] gamma is invalid!! %d\n", gamma,
				__LINE__);
		return -EINVAL;
	}

	return rc;
}

static int s5k4ec_set_antibanding(uint16_t antibanding)
{
	int rc = 0;
	unsigned short bit = S5K4EC_ALGORITHMSBIT_FLICKER;

	CDBG("--CAMERA-- %s ...antibanding = %d\n",  __func__, antibanding);

	switch (antibanding) {
	case CAMERA_ANTIBANDING_60HZ:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_antibanding_60z_tbl);
		break;
	case CAMERA_ANTIBANDING_50HZ:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_antibanding_50z_tbl);
		break;
	case CAMERA_ANTIBANDING_OFF:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_antibanding_off_tbl);
		break;
	case CAMERA_ANTIBANDING_AUTO:
		rc = S5K4EC_WRITEALGO(1, bit, s5k4ec_antibanding_auto_tbl);
		break;
	default:
		pr_err("s5k4ec:[%d] antibanding is invalid!! %d\n", __LINE__, antibanding);
		return -EINVAL;
	}

	return rc;
}

static int s5k4ec_set_sensor_mode(int mode, int res)
{
	int rc = 0;

	CDBG("--CAMERA-- s5k4ec_set_sensor_mode mode = %d, res = %d\n",
			mode, res);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		CDBG("--CAMERA-- SENSOR_PREVIEW_MODE\n");
		rc = s5k4ec_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
		break;

	case SENSOR_SNAPSHOT_MODE:
		CDBG("--CAMERA-- SENSOR_SNAPSHOT_MODE\n");
		rc = s5k4ec_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		CDBG("--CAMERA-- SENSOR_RAW_SNAPSHOT_MODE\n");
		rc = s5k4ec_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
		break;

	default:
		CDBG("--CAMERA--s5k4ec_set_sensor_mode no support\n");
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int s5k4ec_set_wb(uint8_t param)
{
	int rc = 0;
	unsigned short bit = S5K4EC_ALGORITHMSBIT_AWB;

	CDBG("--CAMERA-- %s ...wb = %d\n", __func__ , param);

	switch (param) {
	case CAMERA_WB_AUTO:
		rc = S5K4EC_WRITEALGO(1, bit, s5k4ec_wb_auto);
		break;
	case CAMERA_WB_INCANDESCENT:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_wb_incandescent);
		break;
	case CAMERA_WB_FLUORESCENT:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_wb_fluorescent);
		break;
	case CAMERA_WB_DAYLIGHT:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_wb_daylight);
		break;
	case CAMERA_WB_CLOUDY_DAYLIGHT:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_wb_cloudy);
		break;
	default:
		pr_err("s5k4ec:[%d] wb is invalid!! %d\n", __LINE__, param);
		return -EINVAL;
	}

	return rc;
}

static int32_t s5k4ec_set_af_default_code(struct focus_default_cfg *data)
{
    int32_t rc = 0;

    CDBG("--CAMERA-- %s (Start...)\n", __func__);

    s5k4ec_af_macro_position = data->macro_default_position;
    CDBG("%s default_code:macro=%d\n", __func__, s5k4ec_af_macro_position);

    s5k4ec_af_area_tbl[2].data = (s5k4ec_af_macro_position << 8) & 0xFF00;
    rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_low_conf_pos_tbl);

    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int32_t s5k4ec_set_af_table(struct sensor_cfg_data *cdata)
{
    int32_t rc = 0;

	int cnt = 0;

    int16_t full_step   = cdata->cfg.focus_table_info.full_step;
    int16_t full_start  = cdata->cfg.focus_table_info.full_start;
    int16_t full_stop   = cdata->cfg.focus_table_info.full_stop;
    int16_t macro_step  = cdata->cfg.focus_table_info.macro_step;
    int16_t macro_start = cdata->cfg.focus_table_info.macro_start;
    int16_t macro_stop  = cdata->cfg.focus_table_info.macro_stop;

    unsigned short lens_position = 0;
	int table_index = 0;

    CDBG("--CAMERA-- %s (Start...)\n", __func__);

    if((full_step >= 0) && (full_step <= S5K4EC_FOCUS_STEPS_MAX)
        && (full_start >= 0) && (full_start < full_stop) ) {

        table_index = 2;
        s5k4ec_af_lens_position_normal_tbl[table_index].data = full_step - 1;
        s5k4ec_af_lens_position_caf_tbl[table_index].data = full_step - 1;
        CDBG("[%s] s5k4ec_af_lens_position_normal_tbl[%d].data = 0x%04x !!\n",
              __func__, table_index, s5k4ec_af_lens_position_normal_tbl[table_index].data);

        for(cnt = 0; cnt < S5K4EC_FOCUS_STEPS_MAX; cnt++) {
            table_index++;
            if (cnt < full_step) {
                lens_position = (full_start * 10) + (cnt * (full_stop - full_start) * 10 / (full_step - 1));
                lens_position = (lens_position + 5) / 10;
                s5k4ec_af_lens_position_normal_tbl[table_index].data = lens_position;
                s5k4ec_af_lens_position_caf_tbl[table_index].data = lens_position;
            }
            else {
                s5k4ec_af_lens_position_normal_tbl[table_index].data = 0x00;
                s5k4ec_af_lens_position_caf_tbl[table_index].data =0x00;
            }
            CDBG("[%s] s5k4ec_af_lens_position_normal_tbl[%d].data = 0x%04x !!\n",
                  __func__, table_index, s5k4ec_af_lens_position_normal_tbl[table_index].data);
        }
    }
    else {
        CDBG("[%s] Used Default s5k4ec_af_lens_position_normal_tbl !!\n", __func__);
    }

    if((macro_step >= 0) && (macro_step <= S5K4EC_FOCUS_STEPS_MAX)
        && (macro_start >= 0) && (macro_start < macro_stop) ) {

        table_index = 2;
        s5k4ec_af_lens_position_macro_tbl[table_index].data = macro_step - 1;
        s5k4ec_af_mode_macro_tbl[10].data = macro_step - 1;
        CDBG("[%s] s5k4ec_af_lens_position_macro_tbl[%d].data = 0x%04x !!\n",
              __func__, table_index, s5k4ec_af_lens_position_macro_tbl[table_index].data);

        for(cnt = 0; cnt < S5K4EC_FOCUS_STEPS_MAX; cnt++) {
            table_index++;
            if (cnt < macro_step) {
                lens_position = (macro_start * 10) + (cnt * (macro_stop - macro_start) * 10 / (macro_step - 1));
                lens_position = (lens_position + 5) / 10;
                s5k4ec_af_lens_position_macro_tbl[table_index].data = lens_position;
            }
            else {
                s5k4ec_af_lens_position_macro_tbl[table_index].data = 0x00;
            }
            CDBG("[%s] s5k4ec_af_lens_position_macro_tbl[%d].data = 0x%04x !!\n",
                  __func__, table_index, s5k4ec_af_lens_position_macro_tbl[table_index].data);
        }
    }
    else {
        CDBG("[%s] Used Default s5k4ec_af_lens_position_macro_tbl !!\n", __func__);
    }

    rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_lens_position_normal_tbl);
    if (rc != 0) {
        pr_err("[%s] lens position error !!\n", __func__);
    }

    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int32_t s5k4ec_set_focus_mode(uint8_t focus_mode)
{
    int32_t rc = 0;
    CDBG("--CAMERA-- %s (Start...)\n", __func__);
    switch (focus_mode) {
    case CAMERA_FOCUS_NORMAL:
        rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_lens_position_normal_tbl);
        if (rc != 0) {
            pr_err("[%s] lens position error !!\n", __func__);
            return rc;
        }
        rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_mode_normal_tbl);
        break;
    case CAMERA_FOCUS_MACRO:
        rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_lens_position_macro_tbl);
        if (rc != 0) {
            pr_err("[%s] lens position error !!\n", __func__);
            return rc;
        }
        s5k4ec_af_mode_macro_tbl[2].data = s5k4ec_af_macro_position;
        rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_mode_macro_tbl);
        break;
    case CAMERA_FOCUS_PRODUCT:
        rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_mode_product_tbl);
        break;
    case CAMERA_FOCUS_CAF:
        rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_lens_position_caf_tbl);
        if (rc != 0) {
            pr_err("[%s] lens position error !!\n", __func__);
            return rc;
        }
        rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_mode_continuous_tbl);
        break;
    default:
        pr_err("[%s] Invalid focus mode !!\n", __func__);
        return -EINVAL;
    }
    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int s5k4ec_sensor_start_af(uint8_t focus_mode)
{
	int rc = 0;
	unsigned short read_data  = 0;
	uint32_t lei = 0;
	unsigned short SingleAfFlags = 0;


	CDBG("--CAMERA-- %s (Start...) focus_mode = %d\n", __func__, focus_mode);

	read_data  = 0;
	rc = s5k4ec_i2c_reg_read(Mon_AAIO_ulOptimalLei_0_, &read_data);
	if(rc == 0) {
		CDBG("%s:read_data = 0x%04x\n", __func__, read_data);
		lei = (read_data & 0x0000FFFF);
	} else {
		pr_err("[%d]:s5k4ec i2c_read failed!. addr=0x%04x\n",
				__LINE__, Mon_AF_stat_ulCurStat_0_);
		return rc;
	}

	read_data  = 0;
	rc = s5k4ec_i2c_read(0x0F12, &read_data, sizeof(unsigned short));
	if(rc == 0) {
		lei |= ((read_data << 16) & 0xFFFF0000);
		CDBG("%s:lei = 0x%08lx\n", __func__, (unsigned long)lei);
	} else {
		pr_err("[%s]:s5k4ec i2c_read failed!.\n", __func__);
		return rc;
	}

	read_data  = 0;
	rc = s5k4ec_i2c_reg_read(af_search_usSingleAfFlags, &read_data);
	if(rc == 0) {
		CDBG("%s:read_data = 0x%04x\n", __func__, read_data);
		SingleAfFlags = read_data;
	} else {
		pr_err("[%d]:s5k4ec i2c_read failed!. addr=0x%04x\n",
				__LINE__, Mon_AF_stat_ulCurStat_0_);
		return rc;
	}
	if (lei >= S5K4EC_FOCUS_LOW_LIGHT) {
		if ((SingleAfFlags & 0x000F) != 0x0000) {
			SingleAfFlags &= 0xFFF0;
		}
	} else {
		if ((SingleAfFlags & 0x0002) != 0x0002) {
			SingleAfFlags |= 0x0002;
		}
	}
	if (SingleAfFlags != read_data) {
		CDBG("%s:Write SingleAfFlags = 0x%04x\n", __func__, SingleAfFlags);
		s5k4ec_pre_single_af_tbl[2].data = (unsigned short)SingleAfFlags;
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_pre_single_af_tbl);
		if(rc != 0) {
			pr_err("[%d]:write s5k4ec_pre_single_af_tbl failed!.\n", __LINE__);
			return rc;
		}
	}

	rc = S5K4ECCORE_WRITEPREG(s5k4ec_single_af_tbl);
	CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
	return rc;
}

static int s5k4ec_sensor_cancel_af(void)
{
	int rc = 0;
	CDBG("--CAMERA-- %s (Start...)\n", __func__);
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_cancel_single_af_tbl);
	if(rc != 0) {
		pr_err("[%d]:write s5k4ec_cancel_single_af_tbl failed!.\n", __LINE__);
	}
	CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
	return rc;
}

static int s5k4ec_sensor_start_manual_focus(int16_t af_position)
{
    int rc = 0;

    CDBG("--CAMERA-- %s af_position(=%d) (Start...)\n", __func__, af_position);
    if ((af_position >= S5K4EC_FOCUS_POSITION_MIN)
        || (af_position <= S5K4EC_FOCUS_POSITION_MAX)) {
        s5k4ec_manual_af_tbl[2].data = (unsigned short)af_position;
    } else {
        pr_err("%s af_position(=%d) invalid \n", __func__, af_position);
        return -EINVAL;
    }

    rc = S5K4ECCORE_WRITEPREG(s5k4ec_manual_af_tbl);

    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int s5k4ec_get_af_1st_search_status(uint32_t *af_search_status)
{
    int rc = 0;
    unsigned short af_status  = 0;

    CDBG("--CAMERA-- %s (Start...)\n", __func__);

    rc = s5k4ec_i2c_reg_read(Mon_AF_search_usStatus, &af_status);
    if(0 != rc) {
        pr_err("[%d]:s5k4ec i2c_read failed!. addr=0x%04x\n",
                __LINE__, Mon_AF_search_usStatus);
        return rc;
    }
    else {
        CDBG("af_status=0x%04x\n", af_status);
        switch(af_status) {
            case S5K4EC_FOCUS_SEARCH_SUCCESS:
                *af_search_status = FOCUS_SUCCESS;
                break;
            case S5K4EC_FOCUS_SEARCH_PROGRESS:
                *af_search_status = FOCUS_MOVE;
                break;
            case S5K4EC_FOCUS_SEARCH_CANCELED:
                *af_search_status = FOCUS_CANCEL;
                break;
            case S5K4EC_FOCUS_SEARCH_IDLE:
            case S5K4EC_FOCUS_SEARCH_LOWCONF:
            case S5K4EC_FOCUS_SEARCH_RESTART_AE:
            case S5K4EC_FOCUS_SEARCH_RESTART_SCENE:
            case S5K4EC_FOCUS_SEARCH_RESET:
                *af_search_status = FOCUS_FAILED;
                break;
            default:
                pr_err("%s unmatch af_status =%d\n", __func__, af_status);
                *af_search_status = FOCUS_FAILED;
                break;
        }
    }
    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int s5k4ec_get_af_2nd_search_status(uint32_t *af_search_status)
{
    int rc = 0;
    unsigned char af_status  = 0;

    CDBG("--CAMERA-- %s (Start...)\n", __func__);
    rc = s5k4ec_i2c_reg_readbyte(Mon_SKL_AfOutput_bChangeCfgeCfgDisable,
                                 &af_status, sizeof(uint8_t));
    if(0 != rc) {
        pr_err("[%d]:s5k4ec i2c_read failed!. addr=0x%04x\n",
                __LINE__, Mon_SKL_AfOutput_bChangeCfgeCfgDisable);
        return rc;
    }
    else {
        CDBG("af_search_status=0x%02x\n", af_status);
        switch(af_status) {
            case S5K4EC_FOCUS_SEARCH_FINISHED:
                *af_search_status = FOCUS_SUCCESS;
                break;
            case S5K4EC_FOCUS_SEARCH_NOT_FINISHED:
                *af_search_status = FOCUS_MOVE;
                break;
            default:
                pr_err("%s unmatch af_status =%d\n", __func__, af_status);
                *af_search_status = FOCUS_FAILED;
                break;
        }
    }

    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int s5k4ec_get_af_manual_search_status(int16_t af_position, uint32_t *af_search_status)
{
    int rc = 0;
    unsigned short read_data  = 0;
    unsigned long af_status  = 0;

    CDBG("--CAMERA-- %s (Start...)\n", __func__);

    rc = s5k4ec_i2c_reg_read(Mon_AF_stat_ulCurStat_0_, &read_data);
    if(rc == 0) {
        CDBG("%s:read_data = 0x%04x\n", __func__, read_data);
        af_status = (read_data & 0x0000FFFF);
    } else {
        pr_err("[%d]:s5k4ec i2c_read failed!. addr=0x%04x\n",
                __LINE__, Mon_AF_stat_ulCurStat_0_);
        return rc;
    }

    rc = s5k4ec_i2c_read(0x0F12, &read_data, sizeof(unsigned short));
    if(rc == 0) {
        af_status |= ((read_data << 16) & 0xFFFF0000);
        *af_search_status = af_status;
        CDBG("%s:af_status = 0x%08lx\n", __func__, af_status);
    } else {
        pr_err("[%s]:s5k4ec i2c_read failed!.\n", __func__);
        return rc;
    }

    rc = s5k4ec_i2c_reg_read(Mon_AF_pos_usCurPos, &read_data);
    if(rc == 0) {
        CDBG("%s:raddr = 0x%04x rdata = 0x%04x\n",
                __func__, Mon_AF_pos_usCurPos, read_data);
    }

    rc = s5k4ec_sensor_start_manual_focus(af_position);

    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int s5k4ec_get_af_search_status(struct sensor_cfg_data *cdata)
{
    int rc = 0;
    uint32_t af_search_status  = 0;
    unsigned short rreg = 0;
    uint8_t breg;
    uint32_t dwreg;

    CDBG("--CAMERA-- %s (Start...)\n", __func__);
    if (cdata->cfg.focus_info.af_stage == FOCUS_SEARCH_1ST) {
        rc = s5k4ec_get_af_1st_search_status(&af_search_status);
        if(rc == 0) {
            cdata->cfg.focus_info.af_search_status = af_search_status;
        } else {
            pr_err("%s get 1st_search_status error(%d).\n", __func__, rc);
        }
    } else if (cdata->cfg.focus_info.af_stage == FOCUS_SEARCH_2ND) {
        rc = s5k4ec_get_af_2nd_search_status(&af_search_status);
        if(rc == 0) {
            cdata->cfg.focus_info.af_search_status = af_search_status;
        } else {
            pr_err("%s get 2nd_search_status error(%d).\n", __func__, rc);
        }
    } else if (cdata->cfg.focus_info.af_stage == FOCUS_SEARCH_MANUAL) {
        rc = s5k4ec_get_af_manual_search_status(
                cdata->cfg.focus_info.af_position, &af_search_status);
        if(rc == 0) {
            cdata->cfg.focus_info.af_search_status = af_search_status;
        } else {
            pr_err("%s get manual_search_status error(%d).\n", __func__, rc);
        }
    } else {
        pr_err("%s failed (af_stage=%d)\n",
                __func__, cdata->cfg.focus_info.af_stage);
        cdata->cfg.focus_info.af_search_status = FOCUS_FAILED;
    }

    /* status read */
    rreg = 0;
    rc = s5k4ec_i2c_reg_read(Mon_AF_search_usStatus, &rreg);
    if(0 != rc) {
        pr_err("%s get 1st status error(%d).\n", __func__, rc);
    }
    cdata->cfg.focus_info.af_1st_status = (uint16_t)rreg;

    breg = 0;
    rc = s5k4ec_i2c_reg_readbyte(Mon_SKL_AfOutput_bChangeCfgeCfgDisable,
                                 (unsigned char *)&breg, sizeof(uint8_t));
    if(0 != rc) {
        pr_err("%s get 2nd status error(%d).\n", __func__, rc);
    }
    cdata->cfg.focus_info.af_2nd_status = breg;

    rreg = 0;
    rc = s5k4ec_i2c_reg_read(Mon_AF_pos_usCurPos, &rreg);
    if (rc != 0) {
        pr_err("%s get lens postion error(%d).\n", __func__, rc);
    }
    cdata->cfg.focus_info.af_position = rreg;

    dwreg = 0;
    rc = s5k4ec_i2c_reg_readbyte(Mon_AF_stat_ulCurStat_0_,
                                 (unsigned char *)&dwreg, sizeof(uint32_t));
    if (rc != 0) {
        pr_err("%s get integrated value error(%d).\n", __func__, rc);
    }
    cdata->cfg.focus_info.af_integrated_value = dwreg;

    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int s5k4ec_get_af_lens_position(struct sensor_cfg_data *cdata)
{
	int rc = 0;
    unsigned short rreg = 0;
	CDBG("--CAMERA-- %s (Start...)\n", __func__);

    rc = s5k4ec_i2c_reg_read(Mon_AF_pos_usCurPos, &rreg);
    if (rc != 0) {
        pr_err("%s get lens postion error(%d).\n", __func__, rc);
    }
    cdata->cfg.lens_position = rreg;

    CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
    return rc;
}

static int s5k4ec_set_focus_area(struct sensor_cfg_data *cdata)
{
	int rc = 0;

	CDBG("--CAMERA-- %s (Start...)\n", __func__);
	if (cdata->cfg.focus_area_info.x != 0 && cdata->cfg.focus_area_info.y != 0 &&
	    cdata->cfg.focus_area_info.dx != 0 && cdata->cfg.focus_area_info.dy != 0)
	{
		s5k4ec_af_area_tbl[2].data = (unsigned short)cdata->cfg.focus_area_info.x;
		s5k4ec_af_area_tbl[3].data = (unsigned short)cdata->cfg.focus_area_info.y;
		s5k4ec_af_area_tbl[4].data = (unsigned short)cdata->cfg.focus_area_info.dx;
		s5k4ec_af_area_tbl[5].data = (unsigned short)cdata->cfg.focus_area_info.dy;
		s5k4ec_af_area_tbl[6].data = (unsigned short)cdata->cfg.focus_area_info.x;
		s5k4ec_af_area_tbl[7].data = (unsigned short)cdata->cfg.focus_area_info.y;
		s5k4ec_af_area_tbl[8].data = (unsigned short)cdata->cfg.focus_area_info.dx;
		s5k4ec_af_area_tbl[9].data = (unsigned short)cdata->cfg.focus_area_info.dy;
	}
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_af_area_tbl);
	CDBG("--CAMERA-- %s rc = %d(End...)\n", __func__, rc);
	return rc;
}

static int s5k4ec_set_dimension(struct sensor_cfg_data *cdata)
{
	int rc = 0;

	CDBG("%s:preview width:%d height:%d\n", __func__,
		   cdata->cfg.dimension_info.preview_width,
		   cdata->cfg.dimension_info.preview_height);
	CDBG("%s:capture width:%d height:%d\n", __func__,
		   cdata->cfg.dimension_info.capture_width,
		   cdata->cfg.dimension_info.capture_height);

	s5k4ec_ctrl->dim_set = cdata->cfg.dimension_info;

	return rc;
}

static int s5k4ec_set_zoom(struct sensor_cfg_data *cdata)
{
	int rc = 0;

	CDBG("%s:zoom = %d\n", __func__, cdata->cfg.zoom_info.zoom_value);

	if (s5k4ec_ctrl->zoom_ratio == 0x0100 &&
		cdata->cfg.zoom_info.zoom_value == 0x0100) {
		CDBG("%s: zoom of zero to zero.\n", __func__);
		return rc;
	}
	s5k4ec_ctrl->zoom_ratio = (u16)cdata->cfg.zoom_info.zoom_value;

	rc = sensor_set_zoom(1);

	return rc;
}

static int s5k4ec_set_iso(uint8_t param)
{
	int rc = 0;
	unsigned short bit = S5K4EC_ALGORITHMSBIT_FLICKER;

	CDBG("--CAMERA-- %s ...iso = %d\n", __func__ , param);

	switch(param) {
	case MSM_V4L2_ISO_AUTO:
		rc = S5K4EC_WRITEALGO(1, bit, s5k4ec_iso_auto_tbl);
		break;
	case MSM_V4L2_ISO_100:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_iso_100_tbl);
		break;
	case MSM_V4L2_ISO_200:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_iso_200_tbl);
		break;
	case MSM_V4L2_ISO_400:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_iso_400_tbl);
		break;
	case MSM_V4L2_ISO_800:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_iso_800_tbl);
		break;
	case MSM_V4L2_ISO_1600:
		rc = S5K4EC_WRITEALGO(0, bit, s5k4ec_iso_1600_tbl);
		break;
	default:
		pr_err("s5k4ec:[%d] iso is invalid!! %d\n", __LINE__, param);
		return -EINVAL;
	}

	return rc;
}

static int s5k4ec_set_scene(uint8_t param)
{
	int rc = 0;

	CDBG("--CAMERA-- %s ...scene = %d\n", __func__ , param);

	switch(param) {
	case CAMERA_SCENE_OFF:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_off_tbl);
		break;
	case CAMERA_SCENE_POTRAIT:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_portrait_tbl);
		break;
	case CAMERA_SCENE_POTRAITNIGHT:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_portraitnight_tbl);
		break;
	case CAMERA_SCENE_LANDSCAPE:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_landscape_tbl);
		break;
	case CAMERA_SCENE_NIGHTSHOT:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_nightshot_tbl);
		break;
	case CAMERA_SCENE_FOOD:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_food_tbl);
		break;
	case CAMERA_SCENE_TEXT:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_text_tbl);
		break;
	case CAMERA_SCENE_SPORTS:
		rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_sports_tbl);
		break;
	default:
		pr_err("s5k4ec:[%d] scene is invalid!! %d\n", __LINE__, param);
		return -EINVAL;
	}

	/* scene changed */
	rc = S5K4ECCORE_WRITEPREG(s5k4ec_scene_changed_tbl);
	if (rc != 0) {
		return rc;
	}
	msleep(1);

	rc = s5k4ec_mode_upadate_chk(REG_TC_GP_NewConfigSync);
	if (rc != 0) {
		pr_err("s5k4ec:[%d] scene set sync error!!\n", __LINE__);
		return rc;
	}

	return rc;
}

static int s5k4ec_set_lock_unlock(struct sensor_cfg_data *cdata)
{
	int rc = 0, bctl;
	unsigned short bit = 0;

	CDBG("--CAMERA-- %s ...type = %d, lock/unlock = %d\n", __func__,
		 cdata->cfg.lock_unlock_info.lock_type,
		 cdata->cfg.lock_unlock_info.lock_unlock);

	switch(cdata->cfg.lock_unlock_info.lock_type) {
	case CAMERA_LOCK_UNLOCK_AEC:
		bit = S5K4EC_ALGORITHMSBIT_AE;
		break;
	case CAMERA_LOCK_UNLOCK_AWB:
		bit = S5K4EC_ALGORITHMSBIT_AWB;
		break;
	default:
		pr_err("s5k4ec:[%d] lock/unlock type is invalid!! %d\n",
				__LINE__, cdata->cfg.lock_unlock_info.lock_type);
		return -EINVAL;
	}

	if (cdata->cfg.lock_unlock_info.lock_unlock) {
		bctl = 0;	/* lock */
	}
	else {
		bctl = 1;	/* unlock */
	}

	rc = S5K4EC_WRITEALGO(bctl, bit, s5k4ec_auto_algorithms_bit);

	return rc;
}

static int s5k4ec_set_fps(struct sensor_cfg_data *cdata)
{
    int rc = 0;
    struct s5k4ec_sensor fps_table[] = {{ 0xFCFC, 0xD000 },
                                        { 0x0028, 0x7000 },
                                        { 0x002A, REG_0TC_PCFG_usMaxFrTimeMsecMult10 },
                                        { 0x0F12, 0x0000 },
                                        { 0x0F12, 0x0000 },
                                        { 0x002A, REG_TC_GP_PrevConfigChanged        },
                                        { 0x0F12, 0x0001 }
                                       };

    unsigned short min_fps = (unsigned short)(10000 / cdata->cfg.fps.min_fps);
    unsigned short max_fps = (unsigned short)(10000 / cdata->cfg.fps.max_fps);

    fps_table[3].data = (min_fps ? min_fps : max_fps);
    fps_table[4].data = max_fps;
    
    CDBG("%s:min_fps = 0x%x, max_fps = 0x%x\n", __func__, min_fps, max_fps);

    rc = S5K4ECCORE_WRITEPREG(fps_table);
    return rc;
}

static int s5k4ec_get_camera_info(struct sensor_cfg_data *cdata)
{
    int rc = 0;
    unsigned short read_data  = 0;
    uint16_t lens_pos = 0;
    uint32_t bv = 0;

    rc = s5k4ec_i2c_reg_readsilent(Mon_AF_pos_usCurPos, &read_data);
    if(rc == 0) {
        lens_pos = read_data;
    } else {
        pr_err("[%d]:s5k4ec i2c_read failed!. addr=0x%04x\n", __LINE__,
               Mon_AF_pos_usCurPos);
        return rc;
    }

    read_data  = 0;
    rc = s5k4ec_i2c_reg_readsilent(Mon_AAIO_ulOptimalLei_0_, &read_data);
    if(rc == 0) {
        bv = (read_data & 0x0000FFFF);
    } else {
        pr_err("[%d]:s5k4ec i2c_read failed!. addr=0x%04x\n", __LINE__,
               Mon_AF_stat_ulCurStat_0_);
        return rc;
    }

    read_data  = 0;
    rc = _s5k4ec_i2c_read(0x0F12, &read_data, sizeof(unsigned short));
    if(rc == 0) {
        bv |= ((read_data << 16) & 0xFFFF0000);
    } else {
        pr_err("[%d]:s5k4ec i2c_read failed!.\n", __LINE__);
        return rc;
    }

    cdata->cfg.camera_info.lens_pos =lens_pos;
    cdata->cfg.camera_info.bv = bv;
    CDBG("%s:lens=0x%04x bv=0x%08x\n", __func__, lens_pos, bv);

    return rc;
}

static int s5k4ec_get_snapshot_info(struct sensor_cfg_data *cdata)
{
	int rc = 0;
	unsigned short rreg = 0;
	unsigned short dreg[2];

	CDBG("--CAMERA-- %s\n", __func__);

	rc = s5k4ec_i2c_reg_multiread(Mon_AAIO_PrevAcqCtxt_ME_LEI_Exp, dreg, 2);
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. addr=0x%04x\n", __LINE__,
				Mon_AAIO_PrevAcqCtxt_ME_LEI_Exp);
		return rc;
	}
	cdata->cfg.sensor_snapshot_info.exp_value = dreg[0] | dreg[1] << 16;

	rc = s5k4ec_i2c_reg_read(Mon_AAIO_PrevAcqCtxt_ME_AGain, &rreg);
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. addr=0x%04x\n", __LINE__,
				Mon_AAIO_PrevAcqCtxt_ME_AGain);
		return rc;
	}
	cdata->cfg.sensor_snapshot_info.a_gain = rreg;

	rc = s5k4ec_i2c_reg_read(Mon_AAIO_PrevAcqCtxt_ME_DGain, &rreg);
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. addr=0x%04x\n", __LINE__,
				Mon_AAIO_PrevAcqCtxt_ME_DGain);
		return rc;
	}
	cdata->cfg.sensor_snapshot_info.d_gain = rreg;

	return rc;
}

static int s5k4ec_get_sensor_stat(struct sensor_cfg_data *cdata)
{
	int rc = 0;
	unsigned short rreg = 0;
	struct sensor_get_stat_info *stat_info = NULL;

	CDBG("--CAMERA-- %s ...get stat size = %d\n", __func__ ,
		 cdata->cfg.sensor_get_stat.statsize);

	if (cdata->cfg.sensor_get_stat.statsize !=
		sizeof(struct sensor_get_stat_info)) {
		pr_err("s5k4ec:[%d] stat info size is invalid!!\n", __LINE__);
		return -EINVAL;
	}

	stat_info = kmalloc(sizeof(struct sensor_get_stat_info), GFP_KERNEL);
	if (!stat_info) {
		pr_err("s5k4ec:[%d] stat info kmalloc error!!\n", __LINE__);
		return -ENOMEM;
	}

	rc = s5k4ec_i2c_reg_readbyte(Mon_DBG_xx, stat_info->mon_dbg,
								sizeof(stat_info->mon_dbg));
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. Mon_DBG_xx\n", __LINE__);
		goto get_sensor_err;
	}

	rc = s5k4ec_i2c_reg_readbyte(Mon_SKL_xx, stat_info->mon_skl,
								sizeof(stat_info->mon_skl));
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. Mon_SKL_xx\n", __LINE__);
		goto get_sensor_err;
	}

	rc = s5k4ec_i2c_reg_readbyte(Mon_SETOT_xx, stat_info->mon_setot,
								sizeof(stat_info->mon_setot));
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. Mon_SETOT_xx\n", __LINE__);
		goto get_sensor_err;
	}

	rc = s5k4ec_i2c_reg_readbyte(Mon_AFC_xx, stat_info->mon_afc,
								sizeof(stat_info->mon_afc));
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. Mon_AFC_xx\n", __LINE__);
		goto get_sensor_err;
	}

	rc = s5k4ec_i2c_reg_readbyte(Mon_AWB_xx, stat_info->mon_awb,
								sizeof(stat_info->mon_awb));
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. Mon_AWB_xx\n", __LINE__);
		goto get_sensor_err;
	}

	rc = s5k4ec_i2c_reg_readbyte(Mon_AAIO_xx, stat_info->mon_aaio,
								sizeof(stat_info->mon_aaio));
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. Mon_AAIO_xx\n", __LINE__);
		goto get_sensor_err;
	}

	rc = s5k4ec_i2c_reg_readbyte(Mon_AE_xx, stat_info->mon_ae,
								sizeof(stat_info->mon_ae));
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. Mon_AE_xx\n", __LINE__);
		goto get_sensor_err;
	}

	rc = s5k4ec_i2c_reg_readbyte(Mon_AF_xx, stat_info->mon_af,
								sizeof(stat_info->mon_af));
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. Mon_AF_xx\n", __LINE__);
		goto get_sensor_err;
	}

	rc = s5k4ec_i2c_reg_read(REG_skl_bUseOTPfunc, &rreg);
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. skl_bUseOTPfunc\n", __LINE__);
		goto get_sensor_err;
	}
	stat_info->otp_func = rreg;

	rc = s5k4ec_i2c_reg_read(REG_ash_bUseOTPData, &rreg);
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. ash_bUseOTPData\n", __LINE__);
		goto get_sensor_err;
	}
	stat_info->otp_data = rreg;

	rc = s5k4ec_i2c_reg_read(REG_awbb_otp_disable, &rreg);
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. awbb_otp_disable\n", __LINE__);
		goto get_sensor_err;
	}
	stat_info->otp_awb = rreg;

	rc = s5k4ec_i2c_reg_read(REG_ash_bUseGasAlphaOTP, &rreg);
	if (rc != 0) {
		pr_err("[%d]:s5k4ec read failed!. ash_bUseGasAlphaOTP\n", __LINE__);
		goto get_sensor_err;
	}
	stat_info->otp_gas = rreg;

	if (copy_to_user(cdata->cfg.sensor_get_stat.pstat,
		stat_info,
		sizeof(struct sensor_get_stat_info))) {
		rc = -EFAULT;
	}

get_sensor_err:
	kfree(stat_info);
	return rc;
};

static int s5k4ec_check_camcon(void)
{
    int ret = 0;
    int32_t rc = 0;
    unsigned short sensorId  = 0;
    unsigned short version   = 0;
    unsigned short revision  = 0;

    do {
        rc = s5k4ec_i2c_write(0xFCFC, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002C, 0x7000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }

        rc = s5k4ec_i2c_write(0x002E, 0x01A4, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }

        rc = s5k4ec_i2c_read(0x0F12, &sensorId, sizeof(short));
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_read failed!.\n", __LINE__);
            break;
        }
        else {
            CDBG("sensorId=0x%04x\n", sensorId);
        }

        rc = s5k4ec_i2c_read(0x0F12, &version, sizeof(short));
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_read failed!.\n", __LINE__);
            break;
        }
        else {
            CDBG("version=0x%04x\n", version);
        }

        rc = s5k4ec_i2c_read(0x0F12, &revision, sizeof(short));
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_read failed!.\n", __LINE__);
            break;
        }
        else {
            CDBG(KERN_INFO "revision=0x%04x\n", revision);
        }
//        if((sensorId != 0x4EC0) || (version != 0x0011) || (revision != 0xAAAA)) {
        if((sensorId != 0x4EC0) || (version != 0x0011)) {
            pr_err("s5k4ec:[%d] camcon error!!.\n", __LINE__);
            break;
        }

        ret = 1;

    } while(0);

    return ret;
}

static int s5k4ec_reg_read(unsigned short wpage,
                unsigned short waddr, unsigned short* wdata)
{
    int ret = 0;
    int32_t rc;

    do {
        rc = s5k4ec_i2c_write(0xFCFC, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002C, wpage, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002E, waddr, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_read(0x0F12, wdata, sizeof(short));
        if(0 == rc) {
            ret = 1;
        }
        else {
            pr_err("s5k4ec:[%d] reg read error!!.\n", __LINE__);
        }
    } while (0);

    return ret;
}

static int s5k4ec_reg_write(unsigned short wpage,
                unsigned short waddr, unsigned short wdata)
{
    int ret = 0;
    int32_t rc = 0;

    do {
        rc = s5k4ec_i2c_write(0xFCFC, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x0028, wpage, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002A, waddr, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x0F12, wdata, S5K4EC_I2C_RETRY_TIMES);
        if(0 == rc) {
            ret = 1;
        }
        else {
            pr_err("s5k4ec:[%d] reg write err!!.\n", __LINE__);
        }
    } while (0);

    return ret;
}

#if defined(S5K4EC_IIC_BURSTMODE_WRITE)
#define TRANS_BLOCKSIZE  128

int32_t s5k4ec_i2c_multipleWrite(const struct s5k4ec_sensor* tbl, int rlen)
{
    int32_t rc = -EIO;
    unsigned char* buf = NULL;
    unsigned short addr = 0;
    unsigned short data = 0;
    int writeSize;
    int i;
    int flg;

    buf = kmalloc((TRANS_BLOCKSIZE + 10), GFP_KERNEL);
    if(NULL == buf) {
        pr_err("s5k4ec:[%d] kzalloc failed!.\n", __LINE__);
        return -EFAULT;
    }

    mutex_lock(&s5k4ec_mutex);

    CDBG("rlen=%d\n", rlen);
    for(i = 0, writeSize = 0, flg = 0; i < rlen; i++) {
        addr = tbl[i].addr;
        data = tbl[i].data;
        switch(addr) {
        case S5K4EC_FACTORY:
        case S5K4EC_SETPAGE:
        case S5K4EC_SETADDR:
            if(1 == flg) {
                rc = s5k4ec_i2c_txdata(s5k4ec_client->addr,
                                        &buf[0], writeSize);
                if(rc < 0) {
                    pr_err("s5k4ec:[%d] multipleWrite failed!. %d\n",
                           __LINE__, rc);
                    goto write_error;
                }
                flg = 0;
                writeSize = 0;
            }
            buf[writeSize++] = (addr & 0xFF00) >> 8;
            buf[writeSize++] = (addr & 0x00FF);
            buf[writeSize++] = (data & 0xFF00) >> 8;
            buf[writeSize++] = (data & 0x00FF);
            rc = s5k4ec_i2c_txdata(s5k4ec_client->addr, &buf[0], writeSize);
            if(rc < 0) {
                pr_err("s5k4ec:[%d] multipleWrite failed!.\n", __LINE__);
                goto write_error;
            }
            writeSize = 0;
            break;
        case S5K4EC_READWRITE:
            if(0 == flg) {
                buf[writeSize++] = (addr & 0xFF00) >> 8;
                buf[writeSize++] = (addr & 0x00FF);
                flg = 1;
            }
            if (writeSize >= TRANS_BLOCKSIZE) {
                rc = s5k4ec_i2c_txdata(s5k4ec_client->addr,
                                        &buf[0], writeSize);
                if (rc < 0) {
                    pr_err("s5k4ec:[%d] multipleWrite failed!. %d\n",
                           __LINE__, rc);
                    goto write_error;
                }
                writeSize = 0;
                buf[writeSize++] = (addr & 0xFF00) >> 8;
                buf[writeSize++] = (addr & 0x00FF);
            }
            buf[writeSize++] = (data & 0xFF00) >> 8;
            buf[writeSize++] = (data & 0x00FF);
            break;
        default:
            goto write_error;
            break;
        }
    }

    if(writeSize > 0) {
        rc = s5k4ec_i2c_txdata(s5k4ec_client->addr, &buf[0], writeSize);
        if (rc < 0) {
            pr_err("s5k4ec:[%d] multipleWrite failed!. %d\n", __LINE__, rc);
            goto write_error;
        }
        else {
            usleep(5);
        }
    }

    CDBG("s5k4ec multipleWrite is successful.\n");

write_error:
    kfree(buf);

    mutex_unlock(&s5k4ec_mutex);

    return rc;
}
EXPORT_SYMBOL(s5k4ec_i2c_multipleWrite);
#endif /* defined(S5K4EC_IIC_BURSTMODE_WRITE) */

#if defined(S5K4EC_IIC_BURSTMODE_READ)
static int32_t s5k4ec_i2c_multipleRead(unsigned short raddr,
                               unsigned char *rdata, int rlen)
{
    int32_t rc = -EIO;

    if (!rdata)
        return rc;

    rdata[0] = (raddr & 0xFF00) >> 8;
    rdata[1] = (raddr & 0x00FF);

    rc = s5k4ec_i2c_rxdata(s5k4ec_client->addr, &rdata[0], rlen);
    if (rc < 0) {
        pr_err("s5k4ec:[%d] multipleRead failed!.\n", __LINE__);
        return rc;
    }
    CDBG("s5k4ec multipleRead is successful 0x%04x\n", raddr);
    return rc;
}
#endif /* defined(S5K4EC_IIC_BURSTMODE_READ) */

static int s5k4ec_get_otpdata(unsigned char page, unsigned char offset,
                                short len, unsigned char* buf)
{
    int i = 0;
    int ret = 0;
    int32_t rc;
#if !defined(S5K4EC_IIC_BURSTMODE_READ)
    int j = 0;
    unsigned short temp;
#endif /* #if defined(S5K4EC_IIC_BURSTMODE_READ) */
#if defined(S5K4EC_DEBUG)
    int k = 0;
#endif /* defined(S5K4EC_DEBUG) */
//    unsigned short sPage = (unsigned short)(page & 0x00FF);

    do {
        rc = s5k4ec_i2c_write(0xFCFC, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }

        /*-------- s/w core reset --------*/
        rc = s5k4ec_i2c_write(0x0028, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002A, 0x0012, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x0F12, 0x0001, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }

        /*-------- clock enable to control block --------*/
        rc = s5k4ec_i2c_write(0x0028, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002A, 0x007A, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x0F12, 0x0000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }

        /*-------- make initial state --------*/
        rc = s5k4ec_i2c_write(0x0028, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002A, 0xA000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x0F12, 0x0004, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }


        /*-------- read by page --------*/
        for(i = 0; i < 16; i++) {
             /*-------- set page --------*/
            rc = s5k4ec_i2c_write(0x0028, 0xD000, S5K4EC_I2C_RETRY_TIMES);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
                goto i2c_error;
            }
            rc = s5k4ec_i2c_write(0x002A, 0xA002, S5K4EC_I2C_RETRY_TIMES);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
                goto i2c_error;
            }
            rc = s5k4ec_i2c_write(0x0F12, i, S5K4EC_I2C_RETRY_TIMES);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
                goto i2c_error;
            }

             /*-------- start read mode --------*/
            rc = s5k4ec_i2c_write(0x0028, 0xD000, S5K4EC_I2C_RETRY_TIMES);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
                goto i2c_error;
            }
            rc = s5k4ec_i2c_write(0x002A, 0xA000, S5K4EC_I2C_RETRY_TIMES);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
                goto i2c_error;
            }
            rc = s5k4ec_i2c_write(0x0F12, 0x0001, S5K4EC_I2C_RETRY_TIMES);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
                goto i2c_error;
            }

            udelay(100);

            rc = s5k4ec_i2c_write(0x002C, 0xD000, S5K4EC_I2C_RETRY_TIMES);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
                goto i2c_error;
            }
            rc = s5k4ec_i2c_write(0x002E, 0xA006, S5K4EC_I2C_RETRY_TIMES);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
                goto i2c_error;
            }

#if defined(S5K4EC_IIC_BURSTMODE_READ)
            rc = s5k4ec_i2c_multipleRead(0x0F12, &buf[(i * 64)], 64);
            if(0 != rc) {
                pr_err("s5k4ec:[%d] i2c_multipleRead failed!.\n", __LINE__);
                goto i2c_error;
            }
#if defined(S5K4EC_DEBUG)
            else {
                for(k = 0; k < 64; k++) {
                    CDBG("buf[%d]=0x%02X\n",
                        ((i * 64) + k), buf[((i * 64) + k)]);
                }
            }
#endif /* defined(S5K4EC_DEBUG) */
#else
            for(j = 0; j < 32; j++) {
                temp = 0;
                rc = s5k4ec_i2c_read(0x0F12, &temp, sizeof(short));
                if(0 != rc) {
                    pr_err("s5k4ec:[%d] i2c_read failed!.\n", __LINE__);
                    goto i2c_error;
                }
                else {
                    buf[(i * 64) + (j * 2)]     = (temp & 0xFF00) >> 8;
                    buf[(i * 64) + (j * 2) + 1] = (temp & 0x00FF);
#if defined(S5K4EC_DEBUG)
                    CDBG("buf[%d]=0x%02X, buf[%d]=0x%02X\n",
                        ((i*64)+(j*2)), buf[((i*64)+(j*2))],
                        ((i*64)+(j*2)+1), buf[((i*64)+(j*2)+1)]);
#endif /* defined(S5K4EC_DEBUG) */
                }
            } /* for(j = 0; j < 32; j++) */
#endif /* defined(S5K4EC_IIC_BURSTMODE_READ) */

        } /* for(i = 0; i < 16; i++) */


        /*-------- make initial state --------*/
        rc = s5k4ec_i2c_write(0x0028, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002A, 0xA000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x0F12, 0x0004, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }

        /*-------- interface off --------*/
        rc = s5k4ec_i2c_write(0x0028, 0xD000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x002A, 0xA000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }
        rc = s5k4ec_i2c_write(0x0F12, 0x0000, S5K4EC_I2C_RETRY_TIMES);
        if(0 != rc) {
            pr_err("s5k4ec:[%d] i2c_write failed!.\n", __LINE__);
            break;
        }

        ret = 1;
    } while (0);

i2c_error:
    return ret;
}


static struct regulator_bulk_data cam_regs[] = {
	{ .supply = "gp2",  .min_uV = 2800000, .max_uV = 2800000 },	/* L11 */
	{ .supply = "gp9",  .min_uV = 2800000, .max_uV = 2800000 },	/* L12 */
	{ .supply = "lvs1" }
};

/* Individual support ICS */
static struct gpio sh_cam_gpio[] = {
	{15, GPIOF_DIR_OUT, "CAM_MCLK"},
	{21, GPIOF_DIR_OUT, "VCAM12_EN"},
	{22, GPIOF_DIR_OUT, "CAMRST_N"},
	{50, GPIOF_DIR_OUT, "CAM_STBY_N"}
};

static int s5k4ec_powerOn(struct platform_device *pdev)
{
    int rc = 0;
    int ret = 0;
    struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
    struct msm_camera_device_platform_data *camdev = sdata->pdata;
    int resetGpioNo = sdata->sensor_reset;
    int v1p2_GpioNo = sdata->sensor_pwd;
    int stbynGpioNo = 50;

    int count = ARRAY_SIZE(cam_regs);
    struct device *dev = &pdev->dev;

    CDBG("--CAMERA-- %s (Start...)\n", __func__);
    /* Individual support ICS start */
    rc = gpio_request_array(sh_cam_gpio, ARRAY_SIZE(sh_cam_gpio));
    if(rc) {
        dev_err(dev, "%s: could not gpio request: %d\n", __func__, rc);
        return ret;
    }
    /* Individual support ICS end */

    rc = regulator_bulk_get(dev, count, cam_regs);
    if(rc) {
        dev_err(dev, "%s: could not get regulators: %d\n", __func__, rc);
        return ret;
    }

    rc = regulator_bulk_set_voltage(count, cam_regs);
    if(rc) {
        dev_err(dev, "%s: could not set voltages: %d\n", __func__, rc);
        goto reg_free;
    }

    /*----- Start Sequence -----*/
    /*----- 2.8v -----*/
    rc = regulator_bulk_enable(2, cam_regs);
    if(rc) {
        dev_err(dev, "%s: could not enable regulators: %d\n", __func__, rc);
        goto reg_free;
    }

    /*----- 1.2v -----*/
    gpio_set_value(v1p2_GpioNo, 1);

    /*----- 1.8v -----*/
    rc = regulator_enable(cam_regs[2].consumer);
    if(rc) {
        dev_err(dev, "%s: could not enable regulator LVS1\n", __func__);
        goto reg_free;
    }

    /*----- start MCLK(24MHz) input -----*/
    camdev->camera_gpio_on();
    rc = msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
    if(rc) {
        dev_err(dev, "%s: could not enable camio_clk\n", __func__);
        goto cam_clk_err;
    }
    msm_camio_clk_rate_set(S5K4EC_DEFAULT_MASTER_CLK_RATE);

    /*----- STBYN to High (Release) -----*/
    gpio_set_value(stbynGpioNo, 1);

    /*----- Wait 30usec -----*/
    udelay(30);

    /*----- RSTN to High (Release) -----*/
    gpio_set_value(resetGpioNo, 1);

    /*----- Wait 120usec -----*/
    udelay(120);

    ret = 1;

    CDBG("--CAMERA-- %s (End)\n", __func__);

    return ret;

cam_clk_err:
    camdev->camera_gpio_off();
reg_free:
    gpio_set_value(v1p2_GpioNo, 0);
    regulator_bulk_disable(count, cam_regs);
    regulator_bulk_free(count, cam_regs);
    /* Individual support ICS */
    gpio_free_array(sh_cam_gpio, ARRAY_SIZE(sh_cam_gpio));
    return ret;
}

static int s5k4ec_powerOff(struct platform_device *pdev)
{
    int rc = 0;
    int ret = 0;
    struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
    struct msm_camera_device_platform_data *camdev = sdata->pdata;
    int resetGpioNo = sdata->sensor_reset;
    int v1p2_GpioNo = sdata->sensor_pwd;
    int stbynGpioNo = 50;

    int count = ARRAY_SIZE(cam_regs);
    struct device *dev = &pdev->dev;

    /*----- Stop Sequence More -----*/
    /*----- RSTN to Low -----*/
    gpio_set_value(resetGpioNo, 0);

    /*----- Wait 100usec -----*/
    udelay(100);

    /*----- stop MCLK(24MHz) input -----*/
    rc = msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
    if(rc) {
        dev_err(dev, "%s: could not disable camio_clk\n", __func__);
    }
    camdev->camera_gpio_off();

    /*----- STBYN to Low -----*/
    gpio_set_value(stbynGpioNo, 0);

    /*----- v1.8 -----*/
    rc = regulator_disable(cam_regs[2].consumer);
    if(rc) {
        dev_err(dev, "%s: could not disable regulator LVS1\n", __func__);
    }

    /*----- v1.2 -----*/
    gpio_set_value(v1p2_GpioNo, 0);

    /*----- v2.8 -----*/
    rc = regulator_bulk_disable(2, cam_regs);
    if(rc) {
        dev_err(dev, "%s: could not disable regulators: %d\n", __func__, rc);
    }

    regulator_bulk_free(count, cam_regs);

    /* Individual support ICS */
    gpio_free_array(sh_cam_gpio, ARRAY_SIZE(sh_cam_gpio));

    ret = 1;

    return ret;
}

static int s5k4ec_power_ctrl(struct platform_device *pdev, int8_t power)
{
    int ret = 0;

    if(1 == power) {
        ret = s5k4ec_powerOn(pdev);
    }
    else {
        ret = s5k4ec_powerOff(pdev);
    }

    return ret;
}

static int s5k4ec_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long rc = 0;

	if (copy_from_user(&cdata, (void *)argp,
				sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	CDBG("--CAMERA-- %s %d\n", __func__, cdata.cfgtype);

	mutex_lock(&s5k4ec_mutex);

	switch (cdata.cfgtype) {
	case CFG_SET_MODE:
		rc = s5k4ec_set_sensor_mode(cdata.mode, cdata.rs);
		break;

	case CFG_SET_EFFECT:
		CDBG("--CAMERA-- CFG_SET_EFFECT mode=%d,"
				"effect = %d !!\n", cdata.mode,
				cdata.cfg.effect);
		rc = s5k4ec_set_effect(cdata.mode, cdata.cfg.effect);
		break;

	case CFG_PWR_DOWN:
		CDBG("--CAMERA-- CFG_PWR_DOWN \n");
		rc = s5k4ec_power_down();
		break;

	case CFG_MOVE_FOCUS:
		CDBG("--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
		break;

	case CFG_SET_BRIGHTNESS:
		CDBG("--CAMERA-- CFG_SET_BRIGHTNESS  !!\n");
		rc = s5k4ec_set_brightness(cdata.cfg.brightness);
		break;

	case CFG_SET_GAMMA:
		CDBG("--CAMERA-- CFG_SET_GAMMA  !!\n");
		rc = s5k4ec_set_gamma(cdata.cfg.gamma);
		break;

	case CFG_SET_ANTIBANDING:
		CDBG("--CAMERA-- CFG_SET_ANTIBANDING !!\n");
		rc = s5k4ec_set_antibanding(cdata.cfg.antibanding);
		break;

	case CFG_SET_WB:
		CDBG("--CAMERA-- CFG_SET_WB!!\n");
		rc = s5k4ec_set_wb(cdata.cfg.wb_val);
		break;

	case CFG_SET_AF_DEFAULT_CODE:
		CDBG("--CAMERA-- CFG_SET_AF_DEFAULT_CODE!!\n");
		rc =
		s5k4ec_set_af_default_code(&(cdata.cfg.focus_default_info));
		break;

	case CFG_SET_AF_TABLE:
		CDBG("--CAMERA-- CFG_SET_AF_TABLE!!\n");
		rc = s5k4ec_set_af_table(&cdata);
		break;

	case CFG_SET_FOCUS_MODE:
		CDBG("--CAMERA-- CFG_SET_FOCUS_MODE !\n");
		rc = s5k4ec_set_focus_mode(cdata.cfg.focus_mode);
		break;

	case CFG_SET_AUTO_FOCUS:
		CDBG("--CAMERA-- CFG_SET_AUTO_FOCUS !\n");
		rc = s5k4ec_sensor_start_af(cdata.cfg.focus_mode);
		break;

	case CFG_GET_AF_SEARCH_STATUS:
		CDBG("--CAMERA-- CFG_GET_AF_SEARCH_STATUS !\n");
		rc = s5k4ec_get_af_search_status(&cdata);
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_CANCEL_AUTO_FOCUS:
		CDBG("--CAMERA-- CFG_SET_CANCEL_AUTO_FOCUS !\n");
		rc = s5k4ec_sensor_cancel_af();
		break;

	case CFG_GET_AF_LENS_POSITION:
		CDBG("--CAMERA-- CFG_GET_AF_LENS_POSITION !\n");
		rc = s5k4ec_get_af_lens_position(&cdata);
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FOCUS_AREA:
		CDBG("--CAMERA-- CFG_SET_FOCUS_AREA !\n");
		rc = s5k4ec_set_focus_area(&cdata);
		break;

	case CFG_SET_DIMENSION:
		CDBG("--CAMERA-- CFG_SET_DIMENSION !\n");
		rc = s5k4ec_set_dimension(&cdata);
		break;

	case CFG_SET_ZOOM:
		CDBG("--CAMERA-- CFG_SET_ZOOM !\n");
		rc = s5k4ec_set_zoom(&cdata);
		break;

	case CFG_SET_ISO:
		CDBG("--CAMERA-- CFG_SET_ISO !\n");
		rc = s5k4ec_set_iso(cdata.cfg.iso_val);
		break;

	case CFG_SET_SCENE:
		CDBG("--CAMERA-- CFG_SET_SCENE !\n");
		rc = s5k4ec_set_scene(cdata.cfg.scene_mode);
		break;

	case CFG_SET_LOCK_UNLOCK:
		CDBG("--CAMERA-- CFG_SET_LOCK_UNLOCK !\n");
		rc = s5k4ec_set_lock_unlock(&cdata);
		break;

	case CFG_SET_FPS:
		CDBG("--CAMERA-- CFG_SET_FPS !\n");
		rc = s5k4ec_set_fps(&cdata);
		break;

	case CFG_GET_CAMERA_INFO:
		rc = s5k4ec_get_camera_info(&cdata);
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_SNAPSHOT_INFO:
		CDBG("--CAMERA-- CFG_GET_SNAPSHOT_INFO !\n");
		rc = s5k4ec_get_snapshot_info(&cdata);
		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_SENSOR_STAT:
		CDBG("--CAMERA-- CFG_GET_SENSOR_STAT !\n");
		rc = s5k4ec_get_sensor_stat(&cdata);
		break;

	default:
		CDBG("%s: Command=%d (Not Implement)!!\n", __func__,
				cdata.cfgtype);
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&s5k4ec_mutex);
	return rc;
}

static struct i2c_driver s5k4ec_i2c_driver = {
	.id_table = s5k4ec_i2c_id,
	.probe  = s5k4ec_i2c_probe,
	.remove = __exit_p(s5k4ec_i2c_remove),
	.driver = {
		.name = "s5k4ec",
	},
};

static int s5k4ec_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;

	CDBG("--CAMERA-- %s (Start...)\n", __func__);
	rc = i2c_add_driver(&s5k4ec_i2c_driver);
	CDBG("--CAMERA-- i2c_add_driver ret:0x%x,s5k4ec_client=0x%x\n",
			rc, (unsigned int)s5k4ec_client);
	if ((rc < 0) || (s5k4ec_client == NULL)) {
		CDBG("--CAMERA-- i2c_add_driver FAILS!!\n");
		return rc;
	}

	s->s_init			= s5k4ec_sensor_open_init;
	s->s_release		= s5k4ec_sensor_release;
	s->s_config			= s5k4ec_sensor_config;
	s->s_camera_type	= BACK_CAMERA_2D;
	s->s_mount_angle	= 0;

	s->s_camcon			= s5k4ec_check_camcon;
	s->s_reg_read		= s5k4ec_reg_read;
	s->s_reg_write		= s5k4ec_reg_write;
	s->s_otp_read		= s5k4ec_get_otpdata;
	s->s_power_ctrl		= s5k4ec_power_ctrl;
	s->s_set_af_table	= NULL;
	s->s_reset_af_table	= NULL;

	CDBG("--CAMERA-- %s (End...)\n", __func__);
	return rc;
}

static int s5k4ec_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	CDBG("--CAMERA-- %s ... (Start...)\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("--CAMERA--i2c_check_functionality failed!.\n");
		return -ENOMEM;
	}

	s5k4ec_sensorw = kzalloc(sizeof(struct s5k4ec_work), GFP_KERNEL);
	if (!s5k4ec_sensorw) {
		pr_err("--CAMERA--kzalloc failed!.\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, s5k4ec_sensorw);
	s5k4ec_init_client(client);
	s5k4ec_client = client;

	CDBG("--CAMERA-- %s ... (End...)\n", __func__);
	return 0;
}

static int __s5k4ec_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, s5k4ec_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe	= __s5k4ec_probe,
	.driver	= {
		.name	= "msm_camera_s5k4ec",
		.owner	= THIS_MODULE,
	},
};

static int __init s5k4ec_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k4ec_init);

MODULE_DESCRIPTION("S5K4EC YUV MIPI sensor driver");
MODULE_LICENSE("GPL v2");
