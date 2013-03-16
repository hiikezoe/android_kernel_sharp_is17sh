/* include/sharp/shusb_vbus_power.h
 *
 * Copyright (C) 2012 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SHUSB_VBUS_POWER_EN_H__
#define __SHUSB_VBUS_POWER_EN_H__

#define PM8058_5V_POWER_EN_GEIGER	0
#define PM8058_5V_POWER_EN_USBHOST	1

extern void set_pm8058_5v_power_en(int, int);

#endif /* __SHUSB_VBUS_POWER_EN_H__ */
