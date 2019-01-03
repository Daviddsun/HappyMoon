#ifndef __ADC_BATTERY_H
#define __ADC_BATTERY_H
#include "stm32f4xx.h"

#define VOLTAGE_LOW            14.0f   //电压不能低于14.0V 即单节电池不低于3.5V

enum{
    BATTERY_NORMAL,
    BATTERY_LOW,
};

void Adc_Init(void);
float Get_Battery(void);
void BatteryVoltageUpdate(void);
float GetBatteryVoltage(void);
uint8_t GetBatteryStatus(void);
#endif 
