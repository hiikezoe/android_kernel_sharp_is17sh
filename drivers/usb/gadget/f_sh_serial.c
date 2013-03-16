/* drivers/usb/gadget/f_sh_serial.c
 *
 * f_sh_serial.c - Gadget Driver for Android generic USB serial function driver
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

/* Module */
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_LICENSE("GPL");

static int serial_set_info = 0;

int sh_serial_setup(struct usb_composite_dev *cdev)
{
	int ret = 0;
	
	if(!serial_set_info){
		ret = gserial_setup(cdev->gadget, D_SH_SERIAL_SETUP_PORT_NUM);
		serial_set_info = 1;	
	}

	return ret;
}

void sh_serial_cleanup(void)
{
	if(serial_set_info){
		gserial_cleanup();
		serial_set_info = 0;
	}
}

static enum sh_transport_type sh_serial_str_to_transport(const char *name)
{
	if (!strcasecmp("obex", name))
		return USB_GADGET_SH_SERIAL_TRANSPORT_OBEX;
	if (!strcasecmp("mdlm", name))
		return USB_GADGET_SH_SERIAL_TRANSPORT_MDLM;
	if (!strcasecmp("modem", name))
		return USB_GADGET_SH_SERIAL_TRANSPORT_MODEM;

	return USB_GADGET_SH_SERIAL_TRANSPORT_MAX;
}

int sh_serial_bind_config(struct usb_configuration *c, const char *name)
{
	int err = 0;
	enum sh_transport_type transport;

	transport = sh_serial_str_to_transport(name);

	switch (transport) {
	case USB_GADGET_SH_SERIAL_TRANSPORT_OBEX:
		err = obex_bind_config(c, D_SH_SERIAL_SETUP_PORT_OBEX);
		if (err)
			pr_err("%s: obex_bind_config failed\n", __func__);
		break;
	case USB_GADGET_SH_SERIAL_TRANSPORT_MDLM:
		err = gser_bind_config(c, D_SH_SERIAL_SETUP_PORT_MDLM);
		if (err)
			pr_err("%s: gser_bind_config failed\n", __func__);
		break;
	case USB_GADGET_SH_SERIAL_TRANSPORT_MODEM:
		err = acm_bind_config(c, D_SH_SERIAL_SETUP_PORT_MODEM);
		if (err)
			pr_err("%s: acm_bind_config failed\n", __func__);
		break;
	default:
		pr_err("%s: Un-supported port\n", __func__);
		break;
	}

	return err;
}
