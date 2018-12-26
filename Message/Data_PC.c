#include "Data_PC.h"

void sendParaInfo(void)
{
//	u8 paraToPC[160];
//	u8 floatToHex[4];	
//	u8 intToHex[4];
//	int i=0;

//	paraToPC[0]=0X55;
//	paraToPC[1]=0XAA;
//	paraToPC[2]=0X02;

//	/* Pitch PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,3,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,7,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,11,floatToHex,4);

//	/* Roll PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,15,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,19,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,23,floatToHex,4);

//	/* Yaw PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,27,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,31,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,35,floatToHex,4);
//	
//	/* Height PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,39,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,43,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,47,floatToHex,4);
//			
//	/* ratePitch PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,51,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,55,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,59,floatToHex,4);

//	/* rateRoll PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,63,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,67,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,71,floatToHex,4);

//	/* rateYaw PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,75,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,79,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,83,floatToHex,4);

//	/* VelHeight PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,87,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,91,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,95,floatToHex,4);
//	
//	/* PositionX PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,99,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,103,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,107,floatToHex,4);
//	
//	/* PositionY PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,111,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,115,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,119,floatToHex,4);
//	
//	/* SpeedX PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,123,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,127,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,131,floatToHex,4);
//	
//	/* SpeedY PidPara */
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,135,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,139,floatToHex,4);
//	FloatToByte(0.0,floatToHex);
//	arrycat(paraToPC,143,floatToHex,4);

//	for(i=147;i<157;i++)
//	{
//		paraToPC[i]=0;
//	}	
//	IntToByte(0.0,intToHex);
//	arrycat(paraToPC,157,intToHex,2);
//	
//	for(i=0;i<159;i++)
//	{
//		paraToPC[159]+=paraToPC[i];
//	}
//	Uart3_tx(paraToPC,160);
}

void SendRTInfo(void)
{
  float temp;
	u8 floatToHex[4];		
	u8 dataToPC[64];	
	u8 i=0;

	dataToPC[0]=0X55;
	dataToPC[1]=0XAA;
	dataToPC[2]=0X01;
		
	temp = ahrs.angle.y * 180/PI;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,3,floatToHex,4);
	
	temp = ahrs.angle.x * 180/PI;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,7,floatToHex,4);
	
	temp = ahrs.angle.z * 180/PI;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,11,floatToHex,4);
	
	temp = state_estimate.postionZ;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,15,floatToHex,4);

	temp = RT_Info.batteryVoltage;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,19,floatToHex,4);
	
	temp = UAVThrust.collective_thrust;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,23,floatToHex,4);
	
	for(i=27;i<47;i++)
	{
		dataToPC[i]=0;
	}
	for(i=0;i<47;i++)
	{
		dataToPC[47]+=dataToPC[i];
	}
	
	Uart3_tx(dataToPC,48);
}
