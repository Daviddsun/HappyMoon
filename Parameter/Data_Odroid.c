#include "Data_Odroid.h"

void SendIMUdata(void){
	float temp;
	u8 dataToOdroid[40];
	u8 floatToHex[4];	
	dataToOdroid[0]='$';
	dataToOdroid[1]=0x03;
	
	temp = RT_Info.wx;
	FloatToByte(temp,floatToHex);
	arrycat(dataToOdroid,2,floatToHex,4);
	
	temp = RT_Info.wy;
	FloatToByte(temp,floatToHex);
	arrycat(dataToOdroid,6,floatToHex,4);
	
	temp = RT_Info.wz;
	FloatToByte(temp,floatToHex);
	arrycat(dataToOdroid,10,floatToHex,4);
	
	temp = RT_Info.accXaxis;
	FloatToByte(temp,floatToHex);
	arrycat(dataToOdroid,14,floatToHex,4);

	temp = RT_Info.accYaxis;
	FloatToByte(temp,floatToHex);
	arrycat(dataToOdroid,18,floatToHex,4);
	
	temp = RT_Info.accZaxis;
	FloatToByte(temp,floatToHex);
	arrycat(dataToOdroid,22,floatToHex,4);
	
//	dataToOdroid[26] = RT_Info.Timestamp>>24;
//	dataToOdroid[27] = RT_Info.Timestamp>>16;
//	dataToOdroid[28] = RT_Info.Timestamp>>8;
//	dataToOdroid[29] = RT_Info.Timestamp;
//	
//	dataToOdroid[30] = RT_Info.TriggerCounter>>24;
//	dataToOdroid[31] = RT_Info.TriggerCounter>>16;
//	dataToOdroid[32] = RT_Info.TriggerCounter>>8;
//	dataToOdroid[33] = RT_Info.TriggerCounter;
	
	dataToOdroid[34] = '\r';
	dataToOdroid[35] = '\n';
	
	Uart1_tx(dataToOdroid,36);
	
}


