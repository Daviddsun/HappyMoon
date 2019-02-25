#ifndef __TOFMINIPLUS_H
#define	__TOFMINIPLUS_H

#include "DronePara.h"
#include "Board.h"
//线程FPS读取
typedef struct {
	float CurrentTime;
	uint64_t LastTime;
}FPS_TimeOfFly;


void TOF_DataDeal(Receive_TOFData rx);
float GetTofHeightData(void);
float GetTofHeightVel(void);
#endif
