#ifndef __LEVENBERGMARQUARDT_H
#define __LEVENBERGMARQUARDT_H

#include "DronePara.h"
#include "stm32f4xx.h"
#include <math.h>
#include "MathTool.h"

void LevenbergMarquardt(Vector3f_t inputData[6], Vector3f_t* offset, Vector3f_t* scale, float initBeta[6], float length);


#endif


