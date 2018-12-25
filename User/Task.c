#include "Task.h"
/* 全局变量初始化 */
DroneFlightControl FlightControl;        
DroneRTInfo RT_Info;                   
DroneTargetInfo Target_Info;           
RemoteControl RockerControl;           
OffsetInfo OffsetData;                 
Data_Combine DataCombineflag;      		 
Thrust UAVThrust;											 
Throttle Throttle_Info;								 
PIDOut OriginalPitch,OriginalRoll,OriginalYaw,OriginalPosX,OriginalPosY,OriginalPosZ,
					OriginalWxRate,OriginalWyRate,OriginalWzRate,OriginalVelX,OriginalVelY,OriginalVelZ;
PIDPara PID_ParaInfo;
State_estimate state_estimate;
Reference_state reference_state;
Quaternion quaternion;
/**
 * @Description 传感器数据读取 1khz读取
 */
void SensorRead_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	Vector3f_t accRawData;
	Vector3f_t gyroRawData;
	while(1){
		//读取加速计传感器
		MPU6000_ReadAcc(&accRawData);
		//读取陀螺仪传感器
		MPU6000_ReadGyro(&gyroRawData);
//		//加速计校准
//		AccCalibration(*accRawData);
//		//陀螺仪校准
//		GyroCalibration(*gyroRawData);
//		//加速计数值处理
//		AccDataPreTreat(*accRawData, accCalibData);
//		//陀螺仪数据处理
//		GyroDataPreTreat(*gyroRawData, gyroCalibData);
		//更新消息队列
		OSQPost(&messageQueue[ACC_SENSOR_READ],&accRawData,sizeof(accRawData),OS_OPT_POST_FIFO,&err);
//		OSQPost(&messageQueue[GYRO_DATA_PRETREAT],&gyroRawData,sizeof(gyroRawData),OS_OPT_POST_FIFO,&err);
		//睡眠一个时钟节拍，2ms
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
/**
 * @Description 所有导航任务
 */
void Navigation_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	char        *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	int a = 0;
	while(1){
		p_msg = OSQPend(&messageQueue[ACC_DATA_PRETREAT],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		a ++;
	}
}




