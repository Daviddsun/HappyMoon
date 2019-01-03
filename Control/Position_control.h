#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_
#include "stm32f4xx.h"
#include "Task.h"
#include "DronePara.h"
#include "PID_control.h"
#include "Limits.h"

#define pxy_error_max 	0.6f
#define vxy_error_max 	1.0f
#define pz_error_max    0.3f
#define vz_error_max    0.75f

void Position_Controller(Vector3f_t ExpectPos);
float GetDesiredControlAcc(void);
Vector3f_t GetExpectVel(void);
#endif


