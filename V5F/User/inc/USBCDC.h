#ifndef USBCDC_H
#define USBCDC_H

#include "ch32h417_usbhs_device.h"
#include "usbd_compatibility_hid.h"

void usbhs_hid_enable(void);
void usbhs_hid_poll(void);

#endif