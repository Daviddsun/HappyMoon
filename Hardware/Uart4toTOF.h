#ifndef _UART4TOTOF_H
#define _UART4TOTOF_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include "Board.h"
#include <string.h>
#include <os.h>

void Uart4toTOF_Init(u32 Bound);
void Uart4_tx(uint8_t *data,uint16_t size);

#endif




