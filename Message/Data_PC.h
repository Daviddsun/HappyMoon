#ifndef __DATA_PC_H
#define __DATA_PC_H

#include "DronePara.h"
#include "stm32f4xx.h"
#include "Task.h"
#include "Usart3toBluetooth.h"
#include "UsbDriver.h"
#include "Type_conversion.h"
#include "MahonyAHRS.h"
void SendParaInfo(void);
void SendRTInfo(void);


#endif 

