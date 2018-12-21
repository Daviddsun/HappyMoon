/******************* (C) COPYRIGHT 2015-20~~ Xiluna Tech ************************
 * 作者    ：Xiluna Tech
 * 文件名  ：IMU_AHRS.c
 * 描述    ：AHRS算法库
 * 官网    ：http://xiluna.com/
 * 公众号  ：XilunaTech
*********************************************************************************/
#include "IMU_AHRS.h"
#include "MPU6000.h"
#include "explore_systick.h"
#include "MahonyAHRS.h"
#include "digital_filter.h"
#include <math.h>
#include "Task.h"
#include "DronePara.h"

// asin函数
float safe_asin(float v)
{
	if (isnan(v)) {
		return 0.0f;
	}
	if (v >= 1.0f) {
		return PI/2;
	}
	if (v <= -1.0f) {
		return -PI/2;
	}
	return asin(v);
}

void IMU_HardwareInit()
{
	MPU6000_initialize();					//mpu6000初始化
	delay_ms(100);
	MPU6000_initOffset();         //陀螺仪零偏置校准
	delay_ms(100);
	
}
void IMU_getInfo(void){
	static float q[4];
	static float q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3;
	IMU_getValues();
	
	if(OffsetData.success == false){
		PlaceStausCheck(RT_Info.wx,RT_Info.wy,RT_Info.wz);
		ImuOrientationDetect(RT_Info.accXaxis,RT_Info.accYaxis,RT_Info.accZaxis);
		Calibration_ACC(RT_Info.accXaxis,RT_Info.accYaxis,RT_Info.accZaxis);
	}
	else{
		MahonyAHRSupdateIMU(RT_Info.wx, RT_Info.wy, RT_Info.wz,
													RT_Info.Calibra_accXaxis, RT_Info.Calibra_accYaxis, RT_Info.Calibra_accZaxis);	
		//实际获取四元数
		q[0] = q0;
		q[1] = q1;
		q[2] = q2;
		q[3] = q3;
		//mahony滤波产生的四元数
		quaternion.qw = q0;
		quaternion.qx = q1;
		quaternion.qy = q2;
		quaternion.qz = q3;
		//四元数计算
		q0q1 = q[0]*q[1];
		q0q2 = q[0]*q[2];
		q0q3 = q[0]*q[3];
		q1q1 = q[1]*q[1];
		q1q2 = q[1]*q[2];
		q1q3 = q[1]*q[3];
		q2q2 = q[2]*q[2];
		q2q3 = q[2]*q[3];
		q3q3 = q[3]*q[3]; 
		//yaw roll pitch
		RT_Info.Roll = (atan2(2.0f*(q0q1 + q2q3),
											 1 - 2.0f*(q1q1 + q2q2)));
		RT_Info.Pitch = safe_asin(2.0f*(q0q2 - q1q3));
		
		RT_Info.Yaw = atan2(2.0f*q1q2 + 2.0f*q0q3, -2.0f*q2q2 - 2.0f*q3q3 + 1);
	}
}





