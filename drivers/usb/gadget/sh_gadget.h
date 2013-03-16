/* drivers/usb/gadget/sh_gadget.h
 *
 * sh_gadget.h - interface to USB gadget constant and variable
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
 *
 */

#ifndef __SH_GADGET_H
#define __SH_GADGET_H

#define D_SH_SERIAL_SETUP_PORT_NUM		(USB_GADGET_SH_SERIAL_TRANSPORT_MAX)
#define D_SH_SERIAL_SETUP_PORT_OBEX		(USB_GADGET_SH_SERIAL_TRANSPORT_OBEX)
#define D_SH_SERIAL_SETUP_PORT_MDLM		(USB_GADGET_SH_SERIAL_TRANSPORT_MDLM)
#define D_SH_SERIAL_SETUP_PORT_MODEM	(USB_GADGET_SH_SERIAL_TRANSPORT_MODEM)

enum sh_transport_type {
	USB_GADGET_SH_SERIAL_TRANSPORT_OBEX = 0,
	USB_GADGET_SH_SERIAL_TRANSPORT_MDLM,
	USB_GADGET_SH_SERIAL_TRANSPORT_MODEM,
	USB_GADGET_SH_SERIAL_TRANSPORT_MAX,
};

static bool mdlm_multiple_open;

#endif /* __SH_GADGET_H */
