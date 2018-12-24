#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f4xx.h"
#include "explore_system.h"
#include "explore_systick.h"
#include "SPI1.h"
#include "IMU_AHRS.h"
#include "Task.h"
#include "mcpwm.h"
#include "Flash.h"
#include "Adc_Battery.h"
#include "General_Gpio.h"
#include "Bluetooth.h"
#include "Usart1toOdroid.h"
#include "Sensor.h"

void Board_Init(void);

#endif

