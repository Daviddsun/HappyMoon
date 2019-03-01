/**********************************************************************************************************
 * @文件     Data_PC.c
 * @说明     数据通向PC
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "Data_PC.h"

void SendParaInfo(void)
{
	u8 paraToPC[207];
	u8 floatToHex[4];	

	paraToPC[0]=0X55;
	paraToPC[1]=0XAA;
	paraToPC[2]=0XF1;

	/* Pitch PidPara */
	FloatToByte(OriginalPitch.kP,floatToHex);
	arrycat(paraToPC,3,floatToHex,4);
	FloatToByte(OriginalPitch.kI,floatToHex);
	arrycat(paraToPC,7,floatToHex,4);
	FloatToByte(OriginalPitch.kD,floatToHex);
	arrycat(paraToPC,11,floatToHex,4);

	/* Roll PidPara */
	FloatToByte(OriginalRoll.kP,floatToHex);
	arrycat(paraToPC,15,floatToHex,4);
	FloatToByte(OriginalRoll.kI,floatToHex);
	arrycat(paraToPC,19,floatToHex,4);
	FloatToByte(OriginalRoll.kD,floatToHex);
	arrycat(paraToPC,23,floatToHex,4);
	
	/* WyRate PidPara */
	FloatToByte(OriginalWyRate.kP,floatToHex);
	arrycat(paraToPC,27,floatToHex,4);
	FloatToByte(OriginalWyRate.kI,floatToHex);
	arrycat(paraToPC,31,floatToHex,4);
	FloatToByte(OriginalWyRate.kD,floatToHex);
	arrycat(paraToPC,35,floatToHex,4);
	
	/* WxRate PidPara */
	FloatToByte(OriginalWxRate.kP,floatToHex);
	arrycat(paraToPC,39,floatToHex,4);
	FloatToByte(OriginalWxRate.kI,floatToHex);
	arrycat(paraToPC,43,floatToHex,4);
	FloatToByte(OriginalWxRate.kD,floatToHex);
	arrycat(paraToPC,47,floatToHex,4);

	/* Yaw PidPara */
	FloatToByte(OriginalYaw.kP,floatToHex);
	arrycat(paraToPC,51,floatToHex,4);
	FloatToByte(OriginalYaw.kI,floatToHex);
	arrycat(paraToPC,55,floatToHex,4);
	FloatToByte(OriginalYaw.kD,floatToHex);
	arrycat(paraToPC,59,floatToHex,4);
	
	/* WzRate PidPara */
	FloatToByte(OriginalWzRate.kP,floatToHex);
	arrycat(paraToPC,63,floatToHex,4);
	FloatToByte(OriginalWzRate.kI,floatToHex);
	arrycat(paraToPC,67,floatToHex,4);
	FloatToByte(OriginalWzRate.kD,floatToHex);
	arrycat(paraToPC,71,floatToHex,4);
	
	/* PositionZ PidPara */
	FloatToByte(OriginalPosZ.kP,floatToHex);
	arrycat(paraToPC,75,floatToHex,4);
	FloatToByte(OriginalPosZ.kI,floatToHex);
	arrycat(paraToPC,79,floatToHex,4);
	FloatToByte(OriginalPosZ.kD,floatToHex);
	arrycat(paraToPC,83,floatToHex,4);

	/* VelZ PidPara */
	FloatToByte(OriginalVelZ.kP,floatToHex);
	arrycat(paraToPC,87,floatToHex,4);
	FloatToByte(OriginalVelZ.kI,floatToHex);
	arrycat(paraToPC,91,floatToHex,4);
	FloatToByte(OriginalVelZ.kD,floatToHex);
	arrycat(paraToPC,95,floatToHex,4);
	
	/* PosX PidPara */
	FloatToByte(OriginalPosX.kP,floatToHex);
	arrycat(paraToPC,99,floatToHex,4);
	FloatToByte(OriginalPosX.kI,floatToHex);
	arrycat(paraToPC,103,floatToHex,4);
	FloatToByte(OriginalPosX.kD,floatToHex);
	arrycat(paraToPC,107,floatToHex,4);
	
	/* PosY PidPara */
	FloatToByte(OriginalPosY.kP,floatToHex);
	arrycat(paraToPC,111,floatToHex,4);
	FloatToByte(OriginalPosY.kI,floatToHex);
	arrycat(paraToPC,115,floatToHex,4);
	FloatToByte(OriginalPosY.kD,floatToHex);
	arrycat(paraToPC,119,floatToHex,4);
	
	/* SpeedX PidPara */
	FloatToByte(OriginalVelX.kP,floatToHex);
	arrycat(paraToPC,123,floatToHex,4);
	FloatToByte(OriginalVelX.kI,floatToHex);
	arrycat(paraToPC,127,floatToHex,4);
	FloatToByte(OriginalVelX.kD,floatToHex);
	arrycat(paraToPC,131,floatToHex,4);
	
	/* SpeedY PidPara */
	FloatToByte(OriginalVelY.kP,floatToHex);
	arrycat(paraToPC,135,floatToHex,4);
	FloatToByte(OriginalVelY.kI,floatToHex);
	arrycat(paraToPC,139,floatToHex,4);
	FloatToByte(OriginalVelY.kD,floatToHex);
	arrycat(paraToPC,143,floatToHex,4);
	
	/* FlowX PidPara */
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,147,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,151,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,155,floatToHex,4);
	
	/* FlowY PidPara */
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,159,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,163,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,167,floatToHex,4);
	
	/* FlowVelX PidPara */
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,171,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,175,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,179,floatToHex,4);
	
	/* FlowVelY PidPara */
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,183,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,187,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,191,floatToHex,4);
	
	/* AccZ PidPara */
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,195,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,199,floatToHex,4);
	FloatToByte(0,floatToHex);
	arrycat(paraToPC,203,floatToHex,4);

	Uart3_tx(paraToPC,207);
}

void SendRTInfo(void)
{
  float temp;
	u8 floatToHex[4];		
	u8 dataToPC[55];	
	//各个数据获取
	Vector3f_t GyroData = GyroLpfGetData();
	Vector3f_t AccData = AccGetData();
	dataToPC[0]=0X55;
	dataToPC[1]=0XAA;
	dataToPC[2]=0XF0;
		
	temp = AccData.x * 10.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,3,floatToHex,4);
	
	temp = AccData.y * 10.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,7,floatToHex,4);
	
	temp = AccData.z * 10.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,11,floatToHex,4); 
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,15,floatToHex,4);

	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,19,floatToHex,4);
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,23,floatToHex,4);
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,27,floatToHex,4);
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,31,floatToHex,4);
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,35,floatToHex,4);
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,39,floatToHex,4);
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,43,floatToHex,4);
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,47,floatToHex,4);
	
	temp = 0.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,51,floatToHex,4);	
	
	Uart3_tx(dataToPC,55);
}

//发送函数
void SendDataToARM(Vector3f_t AccRaw,Vector3f_t GyroRaw,uint64_t TriCount,uint64_t Milli){
	u8 SendData[40];
	float_union acc_x,acc_y,acc_z,
							gyro_x,gyro_y,gyro_z;
	
	acc_x.fvalue = AccRaw.x;
	acc_y.fvalue = AccRaw.y;
	acc_z.fvalue = AccRaw.z;
	gyro_x.fvalue = GyroRaw.x;
	gyro_y.fvalue = GyroRaw.y;
	gyro_z.fvalue = GyroRaw.z;
	
	SendData[0]='$';
	SendData[1]=0x03;
	
	SendData[2] = gyro_x.cv[0];
	SendData[3] = gyro_x.cv[1];
	SendData[4] = gyro_x.cv[2];
	SendData[5] = gyro_x.cv[3];
	
	SendData[6] = gyro_y.cv[0];
	SendData[7] = gyro_y.cv[1];
	SendData[8] = gyro_y.cv[2];
	SendData[9] = gyro_y.cv[3];
	
	SendData[10] = gyro_z.cv[0];
	SendData[11] = gyro_z.cv[1];
	SendData[12] = gyro_z.cv[2];
	SendData[13] = gyro_z.cv[3];
	
	SendData[14] = acc_x.cv[0];
	SendData[15] = acc_x.cv[1];
	SendData[16] = acc_x.cv[2];
	SendData[17] = acc_x.cv[3];
	
	SendData[18] = acc_y.cv[0];
	SendData[19] = acc_y.cv[1];
	SendData[20] = acc_y.cv[2];
	SendData[21] = acc_y.cv[3];	
	
	SendData[22] = acc_z.cv[0];
	SendData[23] = acc_z.cv[1];
	SendData[24] = acc_z.cv[2];
	SendData[25] = acc_z.cv[3];		
			
	SendData[26] = Milli>>24;
	SendData[27] = Milli>>16;
	SendData[28] = Milli>>8;
	SendData[29] = Milli;
	
	SendData[30] = TriCount>>24;
	SendData[31] = TriCount>>16;
	SendData[32] = TriCount>>8;
	SendData[33] = TriCount;
	
	SendData[34] = '\r';
	SendData[35] = '\n';
	
	Uart1_tx(SendData,36);
	
}

