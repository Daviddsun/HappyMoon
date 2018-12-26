#ifndef __THRUSTMIXER_H
#define __THRUSTMIXER_H

#include "Task.h"
#include "DronePara.h"

void ThrustMixer(float arm_length,Vector3f_t RotateThrust);
void MotorThrust(float f1,float f2,float f3,float f4);
void PWM_OUTPUT(unsigned int Motor1,unsigned int Motor2,
									unsigned int Motor3,unsigned int Motor4);
void Safety_Protection(void);

#endif

