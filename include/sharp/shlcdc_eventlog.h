/*
 * Copyright (C) 2012 SHARP CORPORATION All rights reserved.
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
 * SHARP DISPLAY DRIVER EVENT LOG
 */

#ifndef SHLCDC_EVENTLOG_H
#define SHLCDC_EVENTLOG_H

/*
 * ID RANGE
 */
#define	SHLCDC_EVENTLOG_FB			0x00000000
#define	SHLCDC_EVENTLOG_SHLCDC		0x00002000
#define	SHLCDC_EVENTLOG_SHDISP		0x00004000
#define	SHLCDC_EVENTLOG_USEREVENT	0x00080000

#define	SHLCDC_EVENTLOG_ANY		0x00100000

#define	DEVICE_NAME_EVENTLOG	"shlcdc_eventlog"
#define	DEVICE_PATH_EVENTLOG	"/dev/shlcdc_eventlog"

/*
 * STRUCTS and enums
 */
struct stEventlog_st {
	unsigned long	kind;
	unsigned long	eventID;
	unsigned long	param;
};

enum enEventlog_Kind {
	EEVENTLOG_USER_EVENT = 0,
	EEVENTLOG_USER_BUSY = 1,
	EEVENTLOG_USER_ERR = 2,
};

/*
 * intefaces
 */
#define SHLCDC_EVENTLOG_MAGIC 'l'
#define SHLCDC_EVENTLOG_USER	_IOW(SHLCDC_EVENTLOG_MAGIC, 0, struct stEventlog_st *)


#ifdef __KERNEL__

/*
 * PROTOTYPES
 */

void shlcdc_eventlog_rec(unsigned long eventID, unsigned long param);
void shlcdc_busylog_rec(unsigned long eventID, unsigned long param);
void shlcdc_errlog_rec(unsigned long eventID, unsigned long param);


#ifndef	CONFIG_SHLCDC_BOARD
void shlcdc_eventlog_rec(unsigned long eventID, unsigned long param)
{
	return;
}
void shlcdc_busylog_rec(unsigned long eventID, unsigned long param)
{
	return;
}
void shlcdc_errlog_rec(unsigned long eventID, unsigned long param)
{
	return;
}
#endif	/* CONFIG_SHLCDC_BOARD */
#endif /* __KERNEL__ */

#endif /* SHLCDC_LCDC_EVENTLOG_H */
