#ifndef __ADC_BATTERY_H
#define __ADC_BATTERY_H
#include "stm32f4xx.h"

#define VOLTAGE_LOW            10.5f   //电压不能低于10.5V 即单节电池不低于3.5V

enum{
    BATTERY_NORMAL,
    BATTERY_LOW,
};

void Adc_Init(void);
float Get_Battery(void);
void BatteryVoltageUpdate(void);
uint8_t GetBatteryStatus(void);
#endif 
