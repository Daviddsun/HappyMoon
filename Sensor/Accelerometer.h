#ifndef __ACCELEROMETER_H
#define	__ACCELEROMETER_H

#include "mathTool.h"
#include "Sensor.h"
#include "lowPassFilter.h"
#include "Gyroscope.h"
#include "Parameter.h"
typedef struct {
    Vector3f_t data;
    Vector3f_t dataLpf;
    LPF2ndData_t lpf_2nd;
} ACCELEROMETER_t;


void AccPreTreatInit(void);
void AccCalibration(Vector3f_t accRaw);
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData);
Vector3f_t AccGetData(void);
#endif
