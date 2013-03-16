/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_RPC_SERVER_HANDSET_H
#define __ASM_ARCH_MSM_RPC_SERVER_HANDSET_H

typedef signed char shextdet_form_position_result_t;
enum
{
SHEXTDET_FORM_POSITION_OPEN = 0,
SHEXTDET_FORM_POSITION_CLOSE,
SHEXTDET_FORM_POSITION_SWIVEL
};

struct msm_handset_platform_data {
	const char *hs_name;
	uint32_t pwr_key_delay_ms; /* default 500ms */
};

void report_headset_status(bool connected);
shextdet_form_position_result_t shextdet_get_form_state (void);

#define SHEXTDET_HEADSET_WAKELOCK
#ifdef SHEXTDET_HEADSET_WAKELOCK	/* Wake_Lock -> */
#define SHEXTDET_IOCTL_MAGIC 'x'
#define SHEXTDET_WAKE_UNLOCK _IO(SHEXTDET_IOCTL_MAGIC, 0x01)   /* Wake UnLock */
#define SHEXTDET_WAKE_LOCK_START _IO(SHEXTDET_IOCTL_MAGIC, 0x02)
#endif	/* Wake_Lock <- */

#endif
