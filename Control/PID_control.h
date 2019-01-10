#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "stm32f4xx.h"
#include "Task.h"
#include <stdio.h>
#include <math.h>
#include "DronePara.h"

float PID_GetPID(PID_t* pid, float error, float dt);
float PID_GetPI(PID_t* pid, float error, float dt);
float PID_GetP(PID_t* pid, float error);
void PID_SetLpf(PID_t* pid, float dCutFreq);
#endif


