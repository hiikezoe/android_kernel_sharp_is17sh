/* include/sharp/shhpamp.h
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

#ifndef __SHHPAMP_H__
#define __SHHPAMP_H__

extern void shhpamp_poweron(void);
extern void shhpamp_poweroff(void);
extern void shhpamp_and_shspamp_poweron(void);
extern void shhpamp_and_shspamp_poweroff(void);

/* PATH */
enum {
    SHHPAMP_PATH_HP,
    SHHPAMP_PATH_HP_AND_SP,
};

/* REGISTER ADDRESS */
#define SHHPAMP_AUDIO_INTERFACE_0			0x18
#define SHHPAMP_DAC_DIGITAL_VOLUME_LEFT		0x1E
#define SHHPAMP_DAC_DIGITAL_VOLUME_RIGHT	0x1F
#define SHHPAMP_ANALOGUE_OUT1_LEFT			0x39
#define SHHPAMP_ANALOGUE_OUT1_RIGHT			0x3A

/* AUDIO_INTERFACE_0 */
#define SHHPAMP_AIFRXR_SRC_RIGHT			0x0010
#define SHHPAMP_AIFTXR_SRC_RIGHT			0x0040
#define SHHPAMP_AUDIO_INTERFACE_0_OTHER		(SHHPAMP_AIFRXR_SRC_RIGHT | SHHPAMP_AIFTXR_SRC_RIGHT)
#define SHHPAMP_DAC_BOOST_0DB				0x0000
#define SHHPAMP_DAC_BOOST_6DB				0x0001
#define SHHPAMP_DAC_BOOST_12DB				0x0010
#define SHHPAMP_DAC_BOOST_18DB				0x0011

/* DAC_DIGITAL_VOLUME_LEFT/RIGHT */
#define SHHPAMP_DAC_VU						0x0100
#define SHHPAMP_DAC_VOL_0DB					0x00C0

/* ANALOGUE_OUT1_LEFT/RIGHT */
#define SHHPAMP_HPOUT_VU					0x0080
#define SHHPAMP_HPOUT_VOL_MINUS57DB			0x0000
#define SHHPAMP_HPOUT_VOL_MINUS56DB			0x0001
#define SHHPAMP_HPOUT_VOL_MINUS5DB			0x0034
#define SHHPAMP_HPOUT_VOL_0DB				0x0039
#define SHHPAMP_HPOUT_VOL_5DB				0x003E
#define SHHPAMP_HPOUT_VOL_6DB				0x003F

#endif /* __SHHPAMP_H__ */
