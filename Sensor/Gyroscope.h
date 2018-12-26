#ifndef __GYROSCOPE_H
#define	__GYROSCOPE_H
#include "mathTool.h"
#include "lowPassFilter.h"
#include "Sensor.h"
typedef struct {
    Vector3f_t data;
    Vector3f_t dataLpf;
    LPF2ndData_t lpf_2nd;
} GYROSCOPE_t;

void GyroPreTreatInit(void);
void GyroCalibration(Vector3f_t gyroRaw);
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData, Vector3f_t* gyroLpfData);
void PlaceStausCheck(Vector3f_t gyro);
uint8_t GetPlaceStatus(void);

#endif
