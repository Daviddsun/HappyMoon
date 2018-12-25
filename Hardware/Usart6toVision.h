#ifndef __USART6TOVISION_H
#define __USART6TOVISION_H

#include "DronePara.h"
#include "Task.h"
#include <os.h>
#include <string.h>

extern _Data_Rx Vision; 

void Usart6toVision_Init(u32 Bound);
void Usart6_tx(uint8_t *data,uint16_t size);

#endif

