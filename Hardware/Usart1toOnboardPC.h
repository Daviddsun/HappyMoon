#ifndef _USARTTOONBOARDPC_H
#define _USARTTOONBOARDPC_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include <string.h>
#include <os.h>
#include "Task.h"
void Uart1_tx(uint8_t *data,uint16_t size);
void Usart1toOnboardPC_Init(u32 Bound);

#endif 
