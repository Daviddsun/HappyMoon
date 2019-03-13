#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "Flash.h"
#include "DronePara.h"
#include "Task.h"

typedef struct {
	u32 isGood;
	PIDPara pidPara;
	OffsetInfo Offset_Data;
}FlashData;

void Load_SensorConfig(void);
void Load_PIDConfig(void);
void Write_Config(void);
void Load_ALLConfig(void);
#endif


