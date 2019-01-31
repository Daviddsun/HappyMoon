#include "Task.h"

void SendDataToARM(Vector3f_t AccRaw,Vector3f_t GyroRaw,uint64_t TriCount,uint64_t Milli);

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
 * @Description CameraIMU数据发送
 */
void CameraIMUSend_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Vector3f_t accRawData,gyroRawData;
	//申请内存
	Vector3f_t *accCalibData = OSMemGet(&memoryInfo[ACC_SENSOR_PRETREAT],&err);
	Vector3f_t *gyroCalibData = OSMemGet(&memoryInfo[GYRO_SENSOR_PRETREAT],&err);
	uint64_t Millisecond = 0;
	uint64_t TriggerCounter = 0;
	bool CameraFlag = false;
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
		//首先关闭触发脚
		Trigger_OFF;
		//20hz Camera触发
		if(Millisecond % 50 == 0){
			Trigger_ON;
			CameraFlag = true;
		}
		//200hz IMU数据
		if(Millisecond % 5 == 0){
			if(CameraFlag){
				CameraFlag = false;
				TriggerCounter ++;
			}
			SendDataToARM(accRawData,gyroRawData,TriggerCounter,Millisecond);
		}
		Millisecond ++;
	}
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

