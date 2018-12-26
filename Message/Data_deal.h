#ifndef __DATA_DEAL_H
#define __DATA_DEAL_H

#include "stm32f4xx.h"
#include "Task.h"
#include "DronePara.h"
#include "Flash.h"
#include "MPU6000.h"
#include "Type_conversion.h"

void DataDeal(_Data_Rx rx);
void DataStitching(_Data_Rx rx);
 
 
#endif 

