#ifndef __DATA_PC_H
#define __DATA_PC_H

#include "DronePara.h"
#include "stm32f4xx.h"
#include "Task.h"
#include "Usart3toBluetooth.h"
#include "Type_conversion.h"
#include "MahonyAHRS.h"
#include "Adc_Battery.h"
void SendParaInfo(void);
void SendRTInfo(void);
void SendDataToARM(Vector3f_t AccRaw,Vector3f_t GyroRaw,uint64_t TriCount,uint64_t Milli);

#endif 

