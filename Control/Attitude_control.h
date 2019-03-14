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

typedef struct{
	Vector3angle_t ExpectAnguleRate;
	Vector3f_t LastEstimateGyro;
	Vector3f_t Thrust;
}AttitudeControl;

void Attitude_InnerController(Vector3f_t EstimateGyro);
void Attitude_OuterController(void);		
float GetFPSAttitudeControl(void);
Vector3f_t GetExpectAnguleRate(void);
Vector3f_t GetExpectThrust(void);
void ResetAttitudeControlValue(void);
#endif

