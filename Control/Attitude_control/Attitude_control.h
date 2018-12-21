#ifndef _ATTITUDE_CONTROL_H
#define _ATTITUDE_CONTROL_H
#include "stm32f4xx.h"
#include "DronePara.h"
#include "arm_math.h"
#include "Task.h"
#include <math.h>

void Attitude_control(void);
void MotorThrust(float f1,float f2,float f3,float f4);
void MotorThrustCallback(float Accelerator1,float Accelerator2,float Accelerator3,float Accelerator4,float arm_length,float rotor_drag_coeff);
void ThrustMixer(float arm_length);
void PWM_OUTPUT(	unsigned int Motor1,
									unsigned int Motor2,
									unsigned int Motor3,
									unsigned int Motor4);
									
void Safety_Protection(void);

									
#endif

