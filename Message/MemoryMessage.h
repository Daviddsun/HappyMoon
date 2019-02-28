#ifndef __MEMORYMESSAGE_H
#define __MEMORYMESSAGE_H

#include "stm32f4xx.h"
#include "includes.h"
#include "Vector3.h"
enum {
    GYRO_SENSOR_READ,
    ACC_SENSOR_READ,
    TEMP_SENSOR_READ,
    GYRO_DATA_PRETREAT,
    ACC_DATA_PRETREAT,
    GYRO_FOR_CONTROL,
		VISUAL_ODOMETRY,
		GROUND_STATION,
		TIMEOFFLY_DATA,
    QUEUE_NUM
};

enum {
	GYRO_SENSOR_RAW,
	ACC_SENSOR_RAW,
	TEMP_SENSOR_RAW,
	GYRO_SENSOR_PRETREAT,
	ACC_SENSOR_PRETREAT,
	GYRO_SENSOR_LPF,
	MEM_NUM
};

void MessageQueueCreate(OS_ERR p_err);
void MemoryCreate(OS_ERR p_err);

extern OS_Q messageQueue[QUEUE_NUM];
extern OS_MEM memoryInfo[MEM_NUM];
#endif

