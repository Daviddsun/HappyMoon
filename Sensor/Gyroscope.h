#ifndef __GYROSCOPE_H
#define	__GYROSCOPE_H
#include "mathTool.h"
#include "lowPassFilter.h"
#include "Sensor.h"
#include "Parameter.h"
typedef struct {
    Vector3f_t data;
    Vector3f_t dataLpf;
    LPF2ndData_t lpf_2nd;
} GYROSCOPE_t;

//放置状态
enum
{
    STATIC,		            //静止
    MOTIONAL			    		//运动
};

void GyroPreTreatInit(void);
void GyroCalibration(Vector3f_t gyroRaw);
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData, Vector3f_t* gyroLpfData, Vector3f_t GyroLevelError);
void PlaceStausCheck(Vector3f_t gyro);
uint8_t GetPlaceStatus(void);
Vector3f_t GyroLpfGetData(void);

#endif
