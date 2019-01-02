#include "mcpwm.h"

void MotorPWM_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;		   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);	
	
  TIM_TimeBaseStructure.TIM_Period = 2000;      							
  TIM_TimeBaseStructure.TIM_Prescaler = 168-1;    						
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;			
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;						
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		
 
	TIM_OCInitStructure.TIM_Pulse = 1000;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);	 
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
		
	TIM_OCInitStructure.TIM_Pulse = 1000;	 	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);	  
  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse = 1000;	 	
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);	 
  TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse = 1000;	 	
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OC4Init(TIM8, &TIM_OCInitStructure);	
  TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
  
	TIM_ARRPreloadConfig(TIM8, ENABLE);			 
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
  TIM_Cmd(TIM8, ENABLE);   
}



