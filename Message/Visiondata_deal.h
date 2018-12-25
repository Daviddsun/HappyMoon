#ifndef __VISIONDATA_DEAL_H
#define __VISIONDATA_DEAL_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include "Task.h"
#include "arm_math.h"

void Vision_datadeal(_Data_Rx rx);

extern float_union position_x,position_y,position_z,velocity_x,velocity_y,
								velocity_z,Quaternion0,Quaternion1,Quaternion2,Quaternion3;

#endif


