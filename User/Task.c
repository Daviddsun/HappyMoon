/******************* (C) COPYRIGHT 2015-20~~ Xiluna Tech ************************
 * 作者    ：Xiluna Tech
 * 文件名  ：task.c
 * 描述    ：任务文件
 * 官网    ：http://xiluna.com/
 * 公众号  ：XilunaTech
*********************************************************************************/
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
 * @Description IMU本机解算
 */
void IMU_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	OffsetData.success = true;
	while(1){
 		IMU_getInfo();
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/*
 * @Description 姿态控制
 */
bool PositionControlFlag = false;
void Attitude_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	unsigned int fly_Pretime = 0;
	bool PreparationFlag = true;
	while(1){
		if(FlightControl.OnOff==Drone_On){
			
			if(FlightControl.droneMode == Drone_Mode_4Axis){
				
				if(fly_Pretime<500){
					fly_Pretime++;
					PWM_OUTPUT(200,200,200,200);
				}
				else{
					if(PreparationFlag){
						PreparationFlag = false;
						PositionControlFlag = true;
						PositionParameterclear();
					}
					Attitude_control();
					/* 安全保护 */
					Safety_Protection();
				}
			}
			else{
				Attitude_control();
			}
		}
		else{
			fly_Pretime = 0;
			PreparationFlag = true;
			PositionControlFlag = false;
			PWM_OUTPUT(0,0,0,0);
		}
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 位置控制
 */
void Position_task(void *p_arg){
	OS_ERR err;
	p_arg = p_arg;
	reference_state.postionX = 0.0f;
	reference_state.postionY = 0.0f;
	reference_state.postionZ = 0.75f;
	while(1){
		if(FlightControl.droneMode == Drone_Mode_4Axis && PositionControlFlag == true){
			Position_control();
		}
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 处理视觉数据
 */
OS_SEM Vision_proc;
void Vision_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	static _Data_Rx Vision_data;
	
	OSSemCreate ((OS_SEM*)&Vision_proc,
								(CPU_CHAR*)"Vision_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1){
		OSSemPend (&Vision_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		memcpy(Vision_data.buf,Odroid_rx.buf,sizeof(Odroid_rx.buf));
		Vision_data.len = Odroid_rx.len;
		Vision_datadeal(Vision_data);
	}
}


/**
 * @Description 处理PC数据
 */
OS_SEM DataDeal_proc;
void DataDeal_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	static _Data_Rx PC_data;
	
	OSSemCreate ((OS_SEM*)&DataDeal_proc,
								(CPU_CHAR*)"DataDeal_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1){
		OSSemPend (&DataDeal_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		memcpy(PC_data.buf,Bluetooth_rx.buf,sizeof(Bluetooth_rx.buf));
		PC_data.len = Bluetooth_rx.len;
		/* 接收PC数据 */
		DataStitching(PC_data);		
	}
}

/**
 * @Description 实时数据回发至地面站
 */
void BluetoothtoPC_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1){
		/* 发送PID参数 */
		if(FlightControl.ReportSW==Report_SET){
			sendParaInfo();
			FlightControl.ReportSW=Report_RESET;
		}
		/* 发送实时飞控数据 */
		sendRTInfo();
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 电池电量检测
 */
void Battery_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	float Battery_Array[10];
	while(1){
		/* 电压读取 */
		RT_Info.batteryVoltage = Average_Filter(Get_battery(),10,Battery_Array);
		/* 电压低于14.8V不允许起飞 */
		if(RT_Info.batteryVoltage<14.80f && (FlightControl.OnOff != Drone_On)){
			RT_Info.lowPowerFlag = 1;
		}
		else{
			/* 空中飞行时电压低于14.0V自动降落 */
			if(RT_Info.batteryVoltage < 14.00f){
				FlightControl.landFlag = 1;
			}
			RT_Info.lowPowerFlag = 0;
		}
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}




