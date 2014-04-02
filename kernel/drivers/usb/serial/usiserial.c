/*
 * USI WCDMA Modem driver
 *
 * Copyright (C) 2011 John Tseng
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include "usb-wwan.h"

#define USI_UNA_VENDOR_ID           0x0e8d
#define USI_UNA_PRODUCT_3COM        0x00a1
#define USI_UNA_PRODUCT_2COM        0x00a2
#define USI_UNAP_PRODUCT_5COM       0x00a7

static int debug;
static int usi_probe(struct usb_serial *serial, const struct usb_device_id *id);

static struct usb_device_id id_table [] = {
	{ USB_DEVICE(USI_UNA_VENDOR_ID, USI_UNA_PRODUCT_3COM) },
	{ USB_DEVICE(USI_UNA_VENDOR_ID, USI_UNA_PRODUCT_2COM) },
	{ USB_DEVICE(USI_UNA_VENDOR_ID, USI_UNAP_PRODUCT_5COM) },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver usi_driver = {
	.name =		"usi-modem",
	.probe =	usb_serial_probe,
	.disconnect =	usb_serial_disconnect,
	.id_table =	id_table,
	.suspend =	usb_serial_suspend,
	.resume =	usb_serial_resume,
	.no_dynamic_id = 	1,
};

static struct usb_serial_driver usi_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"usi-modem",
	},
	.description       = "USI USB modem",
	.usb_driver        = &usi_driver,
	.id_table          = id_table,
	.num_ports         = 1,
	.probe             = usi_probe,
	.open              = usb_wwan_open,
	.close             = usb_wwan_close,
	.dtr_rts	   = usb_wwan_dtr_rts,
	.write             = usb_wwan_write,
	.write_room        = usb_wwan_write_room,
	.chars_in_buffer   = usb_wwan_chars_in_buffer,
	.set_termios       = usb_wwan_set_termios,
	.tiocmget          = usb_wwan_tiocmget,
	.tiocmset          = usb_wwan_tiocmset,
	.attach            = usb_wwan_startup,
	.disconnect        = usb_wwan_disconnect,
	.release           = usb_wwan_release,
#ifdef CONFIG_PM
	.suspend           = usb_wwan_suspend,
	.resume            = usb_wwan_resume,
#endif

};

static int usi_probe(struct usb_serial *serial, const struct usb_device_id *id)
{
        struct usb_wwan_intf_private *data;

	if (serial->dev->descriptor.idProduct == USI_UNA_PRODUCT_2COM &&
		serial->interface->cur_altsetting->desc.bInterfaceNumber != 2)
		return -ENODEV;

	if (serial->dev->descriptor.idProduct == USI_UNA_PRODUCT_3COM &&
		serial->interface->cur_altsetting->desc.bInterfaceNumber != 2 &&
                    serial->interface->cur_altsetting->desc.bInterfaceNumber != 3)
		return -ENODEV;

	if (serial->dev->descriptor.idProduct == USI_UNAP_PRODUCT_5COM &&
		serial->interface->cur_altsetting->desc.bInterfaceNumber < 2)
		return -ENODEV;


	data = serial->private = kzalloc(sizeof(struct usb_wwan_intf_private), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&data->susp_lock);
	data->private = (void *)id->driver_info;
	return 0;
}

static int __init usi_init(void)
{
	int retval;

	retval = usb_serial_register(&usi_device);
	if (retval)
		return retval;
	retval = usb_register(&usi_driver);
	if (retval)
		usb_serial_deregister(&usi_device);
	return retval;
}

static void __exit usi_exit(void)
{
	usb_deregister(&usi_driver);
	usb_serial_deregister(&usi_device);
}

module_init(usi_init);
module_exit(usi_exit);
MODULE_LICENSE("GPL");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");
