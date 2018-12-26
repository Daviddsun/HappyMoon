#include "Task.h"
/* 全局变量初始化 */
DroneFlightControl FlightControl;        
DroneRTInfo RT_Info;                   
DroneTargetInfo Target_Info;           
RemoteControl RockerControl;           
OffsetInfo OffsetData;                      		 
Thrust UAVThrust;											 
Throttle Throttle_Info;								 
PID_t OriginalPitch,OriginalRoll,OriginalYaw,OriginalPosX,OriginalPosY,OriginalPosZ,
					OriginalWxRate,OriginalWyRate,OriginalWzRate,OriginalVelX,OriginalVelY,OriginalVelZ;
State_estimate state_estimate;
Reference_state reference_state;
Quaternion quaternion;

/**
 * @Description 传感器数据读取 1khz读取
 */
void SensorRead_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	Vector3f_t accRawData,gyroRawData;
	while(1){
		//读取加速计传感器
		MPU6000_ReadAcc(&accRawData);
		//读取陀螺仪传感器
		MPU6000_ReadGyro(&gyroRawData);
		//更新消息队列
		OSQPost(&messageQueue[ACC_SENSOR_READ],&accRawData,sizeof(accRawData),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_SENSOR_READ],&gyroRawData,sizeof(gyroRawData),OS_OPT_POST_FIFO,&err);
		//睡眠一个时钟节拍，1ms
		OSTimeDlyHMSM(0,0,0,1,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
/**
 * @Description 传感器数据预处理
 */
void SensorPreDeal_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Vector3f_t accRawData,gyroRawData,accCalibData,gyroCalibData,gyroLpfData;
	//默认不进行传感器校准
	OffsetData.acc_success = false;
	OffsetData.gyro_success = false;
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[ACC_SENSOR_READ],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			accRawData = *((Vector3f_t *)p_msg);
		}
		p_msg = OSQPend(&messageQueue[GYRO_SENSOR_READ],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			gyroRawData = *((Vector3f_t *)p_msg);
		}
		//加速计校准
		AccCalibration(accRawData);
		//陀螺仪校准
		GyroCalibration(gyroRawData);
		//加速计数值处理
		AccDataPreTreat(accRawData, &accCalibData);
		//陀螺仪数据处理
		GyroDataPreTreat(gyroRawData, &gyroCalibData, &gyroLpfData);
		//更新消息队列
		OSQPost(&messageQueue[ACC_DATA_PRETREAT],&accCalibData,sizeof(accCalibData),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_DATA_PRETREAT],&gyroCalibData,sizeof(gyroCalibData),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_FOR_CONTROL],&gyroLpfData,sizeof(gyroLpfData),OS_OPT_POST_FIFO,&err);	
	}
}

/**
 * @Description 所有导航任务
 */
void Navigation_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Vector3f_t accCalibData,gyroCalibData;
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[ACC_DATA_PRETREAT],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			accCalibData = *((Vector3f_t *)p_msg);
		}
		p_msg = OSQPend(&messageQueue[GYRO_DATA_PRETREAT],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			gyroCalibData = *((Vector3f_t *)p_msg);
		}
		MahonyAHRSupdate(gyroCalibData.x,gyroCalibData.y,gyroCalibData.z,
														accCalibData.x,accCalibData.y,accCalibData.z,0,0,0);
	}
}

/**
 * @Description 飞行控制任务
 */
void FlightControl_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Vector3f_t estimategyro,expectgyro,expectangle;
	Vector3f_t rotatethrust;
	static uint32_t count = 0;
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[GYRO_FOR_CONTROL],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			estimategyro = *((Vector3f_t *)p_msg);
		}
		//100hz
		if(count % 10 == 0){
			//飞行位置控制
		}
		//200hz
		if(count % 5 == 0){
			//飞行角度控制
			expectgyro = Attitude_OuterControl(expectangle);
			//飞行速度控制
			
		}
		//500hz
		if(count % 2 == 0){
			//飞行角速率环控制
			rotatethrust = Attitude_InnerControl(expectgyro,estimategyro);
			//推力融合
			ThrustMixer(ARM_Length,rotatethrust);
		}
		count++;
	}
}
/**
 * @Description 飞行状态任务
 */
void FlightStatus_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	while(1){
//		//飞行器放置状态检测
//    PlaceStausCheck(GyroLpfGetData());
//		//传感器方向检测（用于校准时的判断）
//    ImuOrientationDetect();
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
/**
 * @Description Message任务
 */
void Message_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	while(1){
		SendRTInfo();
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

