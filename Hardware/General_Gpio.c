#include "General_Gpio.h"
void GeneralGpio_Init(){
	GPIO_InitTypeDef GPIO_InitStructure;																					
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	
	//Trigger signal
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;													
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;												
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;										
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;													
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	Trigger_OFF;//首先关闭触发信号
}




