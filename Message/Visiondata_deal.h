#ifndef __VISIONDATA_DEAL_H
#define __VISIONDATA_DEAL_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include "Task.h"
//线程FPS读取
typedef struct {
	float CurrentTime;
	uint64_t LastTime;
}FPS_VisualOdometry;

typedef struct {
	float CurrentTime;
	uint64_t LastTime;
}FPS_ReferenceRoute;

void Vision_DataDeal(Receive_VisualOdometry rx);
Vector3f_t GetVisualOdometryPos(void);
Vector3f_t GetVisualOdometryVel(void);
Vector3f_t GetVisualOdometryPosTrans(void);
Vector3f_t GetVisualOdometryVelTrans(void);
Vector3angle_t GetVisualOdometryAngle(void);
Vector3f_t GetVisualOdometryRefPos(void);
float GetFPSVisualOdometry(void);
#endif


