#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_
#include "mathTool.h"
#include "Vector3.h"
#include "Kalman3.h"
#include "KalmanVel.h"
#include "Visiondata_deal.h"

enum
{
    VIO_VEL_X = 0,
    VIO_VEL_Y,
    VIO_VEL_Z,
    BARO_VEL,
    TOF_VEL
};

typedef struct {
    Vector3f_t accel;
    Vector3f_t accel_bias;
    
    Vector3f_t velocity;
    float      velMeasure[6];
    
    Vector3f_t position;
    Vector3f_t posMeasure;
} NAVGATION_t;

void NavigationInit(void);
void VelocityEstimate(void);
void PositionEstimate(void);
#endif
