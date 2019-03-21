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
	Vector3angle_t AHRSAngle = GetCopterAngle();
	Vector3angle_t VIOAngle = GetVisualOdometryAngle();
	Vector3f_t VIOVel = GetVisualOdometryVelTrans();
	Vector3f_t VIOPos = GetVisualOdometryPos();
	Vector3f_t KalmanVel = GetCopterVelocity();
	Vector3f_t KalmanPos = GetCopterPosition();
	Vector3f_t EarthAcc = EarthAccGetData();
	Vector3f_t BodyAcc = AccGetData();
	float TOFHeightData = GetTofHeightData();
	float TOFHeightVelData = GetTofHeightVel();
	float BatteryVoltage = GetBatteryVoltage();
	float Trust = GetDesiredControlAcc();
	
	dataToPC[0]=0X55;
	dataToPC[1]=0XAA;
	dataToPC[2]=0XF0;
		
	temp = AHRSAngle.pitch * 180/PI;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,3,floatToHex,4);
	
	temp = AHRSAngle.roll * 180/PI;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,7,floatToHex,4);
	
	temp = VIOAngle.yaw * 180/PI;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,11,floatToHex,4); 
	
	temp = KalmanPos.z;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,15,floatToHex,4);

	temp = BatteryVoltage;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,19,floatToHex,4);
	
	temp = KalmanVel.z * 100.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,23,floatToHex,4);
	
	//暂时没有使用的两位
	temp = TOFHeightData;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,27,floatToHex,4);
	
	temp = TOFHeightVelData;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,31,floatToHex,4);
	
	temp = VIOVel.x * 100.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,35,floatToHex,4);
	
	temp = VIOVel.y * 100.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,39,floatToHex,4);
	
	temp = KalmanVel.x * 100.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,43,floatToHex,4);
	
	temp = KalmanVel.y * 100.0f;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,47,floatToHex,4);
	
	temp = 0 ;
	FloatToByte(temp,floatToHex);
	arrycat(dataToPC,51,floatToHex,4);	
	
	Uart3_tx(dataToPC,55);
}
