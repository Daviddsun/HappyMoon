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
 
 
#endif 

