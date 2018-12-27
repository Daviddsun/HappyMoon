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

void Load_Config(void);
void Write_Config(void);

#endif
