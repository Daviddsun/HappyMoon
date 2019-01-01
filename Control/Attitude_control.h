#ifndef _ATTITUDE_CONTROL_H
#define _ATTITUDE_CONTROL_H
#include "stm32f4xx.h"
#include "DronePara.h"
#include "Task.h"
#include "Vector3.h"
#include "PID_control.h"
#include "MahonyAHRS.h"
#include <math.h>
//线程FPS读取
typedef struct {
	float CurrentTime;
	uint64_t LastTime;
}FPS_AttitudeControl;

Vector3f_t Attitude_InnerControl(Vector3f_t ExpectGyro, Vector3f_t EstimateGyro);
Vector3angle_t Attitude_OuterControl(Vector3angle_t ExpectAngle);		
float GetFPSAttitudeControl(void);
#endif

