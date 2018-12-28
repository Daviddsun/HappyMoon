#ifndef __DATA_DEAL_H
#define __DATA_DEAL_H

#include "stm32f4xx.h"
#include "Task.h"
#include "DronePara.h"
#include "Flash.h"
#include "MPU6000.h"
#include "Type_conversion.h"

void GroundStationDataDeal(Receive_GroundStation rx);
void DataStitching(Data_Rx rx);
uint8_t GetCopterStatus(void);
uint8_t GetCopterTest(void);
uint8_t SendPID(void); 
void ResetSendPID(void);
Vector3angle_t GetRemoteControlAngle(void);
Vector3f_t GetRemoteControlAngleVel(void);
RemoteControl GetRemoteControlFlyData(void);
#endif 

