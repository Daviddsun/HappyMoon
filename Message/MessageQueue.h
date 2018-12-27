#ifndef __MESSAGEQUEUE_H
#define __MESSAGEQUEUE_H

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
    QUEUE_NUM
};

void MessageQueueCreate(OS_ERR p_err);

extern OS_Q messageQueue[QUEUE_NUM];

#endif
