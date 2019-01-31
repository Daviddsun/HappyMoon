#include "Task.h"
/* 全局变量初始化 */ 
PID_t OriginalPitch,OriginalRoll,OriginalYaw,OriginalPosX,OriginalPosY,OriginalPosZ,
					OriginalWxRate,OriginalWyRate,OriginalWzRate,OriginalVelX,OriginalVelY,OriginalVelZ;
PIDPara PID_ParaInfo;
OffsetInfo OffsetData;
/**
 * @Description 传感器数据读取 1khz读取
 */
void IMUSensorRead_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	// 申请内存
	Vector3f_t *accRawData = OSMemGet(&memoryInfo[ACC_SENSOR_RAW],&err);
	Vector3f_t *gyroRawData = OSMemGet(&memoryInfo[GYRO_SENSOR_RAW],&err);
	while(1){
#ifdef SpeedyBeeF4
		//读取加速计 陀螺仪 传感器
		MPU6000_ReadAccGyro(accRawData ,gyroRawData);
#else
		//读取加速计 陀螺仪 传感器
		MPU6500_ReadAccGyro(accRawData ,gyroRawData);
#endif
		//更新消息队列
		OSQPost(&messageQueue[ACC_SENSOR_READ],accRawData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_SENSOR_READ],gyroRawData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		//睡眠一个时钟节拍1ms
		OSTimeDlyHMSM(0,0,0,1,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 传感器数据预处理
 */
void IMUSensorPreDeal_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	CPU_SR_ALLOC();
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Vector3f_t accRawData,gyroRawData;
	//申请内存
	Vector3f_t *accCalibData = OSMemGet(&memoryInfo[ACC_SENSOR_PRETREAT],&err);
	Vector3f_t *gyroCalibData = OSMemGet(&memoryInfo[GYRO_SENSOR_PRETREAT],&err);
	Vector3f_t *gyroLpfData = OSMemGet(&memoryInfo[GYRO_SENSOR_LPF],&err);
	//默认不进行传感器校准
	OffsetData.acc_success = false;
	OffsetData.gyro_success = false;	
	OffsetData.level_success = false;
	//进入临界区
	OS_CRITICAL_ENTER();
	//IMU传感器校准参数读取 
	Load_SensorConfig();
	//陀螺仪预处理初始化
	GyroPreTreatInit();
	//加速度预处理初始化
	AccPreTreatInit();	
	//离开临界区 	
	OS_CRITICAL_EXIT();	
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
		//IMU安装误差校准
		ImuLevelCalibration();
		//加速计数据处理
		AccDataPreTreat(accRawData, accCalibData,OffsetData.level_scale);
		//陀螺仪数据处理
		GyroDataPreTreat(gyroRawData, gyroCalibData, gyroLpfData,OffsetData.level_scale);
		
		//更新下一级消息队列
		OSQPost(&messageQueue[ACC_DATA_PRETREAT],accCalibData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_DATA_PRETREAT],gyroCalibData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_FOR_CONTROL],gyroLpfData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);	
	}
}
