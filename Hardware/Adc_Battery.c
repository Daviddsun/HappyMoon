/**********************************************************************************************************
 * @文件     Adc_Battery.c
 * @说明     电池采样ADC
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "Adc_Battery.h"

volatile u16 Adc_Battery[10];
static float batVoltage;

void Adc_Init(){	
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;	
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_480Cycles );	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 
	ADC_DMACmd(ADC1, ENABLE);           
	ADC_Cmd(ADC1, ENABLE);

	DMA_DeInit(DMA2_Stream0); 
	DMA_InitStructure.DMA_BufferSize = 5;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Adc_Battery;   
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);  
	DMA_InitStructure.DMA_PeripheralBurst =DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0,ENABLE);
	
	ADC_SoftwareStartConv(ADC1); 
}
/**********************************************************************************************************
*函 数 名: Get_Battery
*功能说明: 获取电压
*形    参: 无
*返 回 值: ADC采样值
**********************************************************************************************************/
float Get_Battery(void){
	float RTbattery = 11.0f*3.3f*((float)Adc_Battery[0]/4096);
	return RTbattery;
}

/**********************************************************************************************************
*函 数 名: BatteryVoltageUpdate
*功能说明: 电池电压采样更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BatteryVoltageUpdate(void){
   batVoltage = batVoltage * 0.995f + Get_Battery() * 0.005f;
}

/**********************************************************************************************************
*函 数 名: GetBatteryVoltage
*功能说明: 获取电池电压
*形    参: 无
*返 回 值: 电压值
**********************************************************************************************************/
float GetBatteryVoltage(void){
    return batVoltage;
}

/**********************************************************************************************************
*函 数 名: GetBatteryStatus
*功能说明: 获取电池状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetBatteryStatus(void){
    if(batVoltage < VOLTAGE_LOW)	{
        return BATTERY_LOW;
    }
    else{
        return BATTERY_NORMAL;
    }
}
