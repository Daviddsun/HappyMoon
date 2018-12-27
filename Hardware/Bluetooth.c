#include "Bluetooth.h"

u8 bluetooth_bound[8]="AT+BAUD8";
u8 bluetooth_name[13]="AT+NAMExiluna";

void Bluetooth_Init(void)
{
	Usart3toBluetooth_Init(115200);
}

