#ifndef __USBDRIVER_H
#define __USBDRIVER_H

#include "stm32f4xx.h"
#include "usb_dcd_int.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"
#include "usb_bsp.h"
#include "usbd_cdc_core.h"

typedef void (*UsbCallback)(uint8_t data);

void Usb_Init(void);
void Usb_Send(uint8_t *dataToSend, uint8_t length);
void Usb_SetRecvCallback(UsbCallback usbCallback);

#endif
