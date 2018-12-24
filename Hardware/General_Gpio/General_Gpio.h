#ifndef __GENERAL_GPIO_H
#define __GENERAL_GPIO_H

#include "stm32f4xx.h"

void GeneralGpio_Init(void);

#define LED_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define LED_ON  GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define Trigger_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define Trigger_ON  GPIO_SetBits(GPIOB,GPIO_Pin_6)

#endif 
