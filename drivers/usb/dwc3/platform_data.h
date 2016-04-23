/*
 * A header file for the USB device controller driver in the Quatro processors
 * Copyright (c) 2014 Cambridge Silicon Radio Ltd.
 *
 * Based on platform_data.h Copyright (C) 2010-2011 Texas Instruments Incorporated
 * - http://www.ti.com by Felipe Balbi <balbi@ti.com> and
 *                        Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/usb/ch9.h>
#include <linux/usb/otg.h>

struct dwc3_platform_data {
	enum usb_device_speed maximum_speed;
	enum usb_dr_mode dr_mode;
	bool tx_fifo_resize;
};
