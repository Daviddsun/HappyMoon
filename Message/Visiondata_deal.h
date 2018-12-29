#ifndef __VISIONDATA_DEAL_H
#define __VISIONDATA_DEAL_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include "Task.h"

void Vision_DataDeal(Receive_VisualOdometry rx);
Vector3f_t GetVisualOdometryPos(void);
Vector3f_t GetVisualOdometryVel(void);
Vector3angle_t GetVisualOdometryAtt(void);
Vector3f_t GetVisualOdometryRefPos(void);
#endif


