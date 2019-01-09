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
		//睡眠一个时钟节拍，1ms
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
	//IMU传感器校准参数读取 
	Load_SensorConfig();
	//陀螺仪预处理初始化
	GyroPreTreatInit();
	//加速度预处理初始化
	AccPreTreatInit();
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
	
		//加速计数据处理
		AccDataPreTreat(accRawData, accCalibData);
		//陀螺仪数据处理
		GyroDataPreTreat(gyroRawData, gyroCalibData, gyroLpfData);
		
		//更新消息队列
		OSQPost(&messageQueue[ACC_DATA_PRETREAT],accCalibData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_DATA_PRETREAT],gyroCalibData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);
		OSQPost(&messageQueue[GYRO_FOR_CONTROL],gyroLpfData,sizeof(Vector3f_t),OS_OPT_POST_FIFO,&err);	
	}
}

/**
 * @Description 姿态解算 Mahony 滤波
 */
void AttitudeFilter_task(void *p_arg){
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
		//飞行姿态估计
		MahonyAHRSupdateIMU(gyroCalibData.x,gyroCalibData.y,gyroCalibData.z,
															accCalibData.x,accCalibData.y,accCalibData.z);
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
	Vector3f_t Estimate_Gyro,Expect_Gyro,Rotate_Thrust,Expect_Pos;
	Vector3angle_t Expect_Angle;
	Vector4PosController DesiredControlCommands;
	static uint32_t count = 0;
	/** 控制参数读取 **/
	Load_PIDConfig();
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[GYRO_FOR_CONTROL],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			Estimate_Gyro = *((Vector3f_t *)p_msg);
		}
		//起飞检测
		if(GetCopterStatus() == Drone_Off){
			//置位期望位置
			ResetExpectPosition(&Expect_Pos);
			PWM_OUTPUT(0,0,0,0);
			count = 0;
		}
		else{
			if(count < 2000 && GetCopterTest() == Drone_Mode_4Axis){
				PWM_OUTPUT(200,200,200,200);
			}
			else{
				//100hz
				if(count % 10 == 0){
					//安全保护
					SafeControl();
				}
				//200hz
				if(count % 5 == 0){
					//飞行位置控制
					Position_Controller(Expect_Pos);
					//期望角度选择
					if(GetCopterTest() == Drone_Mode_Pitch || 
											GetCopterTest() == Drone_Mode_Roll){
						//手柄控制
						Expect_Angle = GetRemoteControlAngle();
					}else{
						//位置控制
						Expect_Angle = GetDesiredControlAngle();
					}
					//飞行角度控制
					Expect_Gyro.x = (Attitude_OuterController(Expect_Angle)).roll;
					Expect_Gyro.y = (Attitude_OuterController(Expect_Angle)).pitch;
					Expect_Gyro.z = (Attitude_OuterController(Expect_Angle)).yaw;
				}
				//500hz
				if(count % 2 == 0){
					//期望角速率选择
					if(GetCopterTest() == Drone_Mode_RatePitch || 
											GetCopterTest() == Drone_Mode_RateRoll){
						Expect_Gyro = GetRemoteControlAngleVel();
					}
					//期望高度加速度
					DesiredControlCommands.ExpectAcc = GetDesiredControlAcc();
					//飞行角速率控制
					Rotate_Thrust = Attitude_InnerController(Expect_Gyro,Estimate_Gyro);
					//推力融合
					ThrustMixer(ARM_Length,DesiredControlCommands.ExpectAcc,Rotate_Thrust);
				}
			}
			count++;
		}
	}
}

/**
 * @Description 全项数据融合 500Hz
 */
void OmniFusion_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	//导航参数初始化
	NavigationInit();
	while(1){
		//飞行速度估计
		VelocityEstimate();
		//飞行位移估计
		PositionEstimate();
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 其他传感器数据更新 100Hz
 */
void OtherSensorUpdate_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	while(1){
		//电池电压电流采样更新
    BatteryVoltageUpdate();
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
	* @Description 视觉惯性里程计任务
 */
void VisualOdometry_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Receive_VisualOdometry  VisualOdometry;
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[VISUAL_ODOMETRY],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			VisualOdometry = *((Receive_VisualOdometry *)p_msg);
		}
		Vision_DataDeal(VisualOdometry);
	}
}

/**
 * @Description 地面站处理任务
 */
void GroundStation_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	void   *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;
	Receive_GroundStation GroundStation;
	while(1){
		//消息队列信息提取
		p_msg = OSQPend(&messageQueue[GROUND_STATION],0,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
		if(err == OS_ERR_NONE){
			GroundStation = *((Receive_GroundStation *)p_msg);
		}
		GroundStationDataDeal(GroundStation);
	}
}

/**
 * @Description 飞行状态任务 50hz
 */
void FlightStatus_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	while(1){
		//飞行器放置状态检测
    PlaceStausCheck(GyroLpfGetData());
		//传感器方向检测
    ImuOrientationDetect(AccGetData());
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description Message任务 25hz
 */
void Message_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	while(1){
		//发送参数信息
		if(SendPID() == Report_SET){
			SendParaInfo();
			ResetSendPID();
		}
		//发送传感器数据
		SendRTInfo();
		OSTimeDlyHMSM(0,0,0,40,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

