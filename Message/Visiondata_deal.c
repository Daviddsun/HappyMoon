#include "Visiondata_deal.h"

FPS_VisualOdometry FPSVisualOdometry;
//估计状态定义
float_union position_x,position_y,position_z,                       //估计位置
							velocity_x,velocity_y,velocity_z,										  //估计速度
								Quaternion0,Quaternion1,Quaternion2,Quaternion3,	  //估计姿态
									reference_posx,reference_posy,reference_posz;			//参考航点

/**********************************************************************************************************
*函 数 名: Vision_datadeal
*功能说明: 接收视觉里程计数据
*形    参: 无
*返 回 值: Position
**********************************************************************************************************/
void Vision_DataDeal(Receive_VisualOdometry rx){
	OS_ERR err;
	if(rx.buf[0]==0x55 && rx.buf[1]==0xAA && rx.buf[55]==0xAA){
		if(rx.buf[2] == 0x30){
			//计算函数运行时间间隔
			FPSVisualOdometry.CurrentTime = (OSTimeGet(&err) - FPSVisualOdometry.LastTime) * 1e-3;
			FPSVisualOdometry.LastTime = OSTimeGet(&err);
			//X轴位置数据
			position_x.cv[0] = rx.buf[3];
			position_x.cv[1] = rx.buf[4];
			position_x.cv[2] = rx.buf[5];
			position_x.cv[3] = rx.buf[6];
			//Y轴位置数据
			position_y.cv[0] = rx.buf[7];
			position_y.cv[1] = rx.buf[8];
			position_y.cv[2] = rx.buf[9];
			position_y.cv[3] = rx.buf[10];
			//Z轴位置数据
			position_z.cv[0] = rx.buf[11];
			position_z.cv[1] = rx.buf[12];
			position_z.cv[2] = rx.buf[13];
			position_z.cv[3] = rx.buf[14];
			//X轴速度数据
			velocity_x.cv[0] = rx.buf[15];
			velocity_x.cv[1] = rx.buf[16];
			velocity_x.cv[2] = rx.buf[17];
			velocity_x.cv[3] = rx.buf[18];
			//Y轴速度数据
			velocity_y.cv[0] = rx.buf[19];
			velocity_y.cv[1] = rx.buf[20];
			velocity_y.cv[2] = rx.buf[21];
			velocity_y.cv[3] = rx.buf[22];	
			//Z轴速度数据
			velocity_z.cv[0] = rx.buf[23];
			velocity_z.cv[1] = rx.buf[24];
			velocity_z.cv[2] = rx.buf[25];
			velocity_z.cv[3] = rx.buf[26];
			//视觉里程计的姿态数据
			Quaternion0.cv[0] = rx.buf[27];
			Quaternion0.cv[1] = rx.buf[28];
			Quaternion0.cv[2] = rx.buf[29];
			Quaternion0.cv[3] = rx.buf[30];
			
			Quaternion1.cv[0] = rx.buf[31];
			Quaternion1.cv[1] = rx.buf[32];
			Quaternion1.cv[2] = rx.buf[33];
			Quaternion1.cv[3] = rx.buf[34];
			
			Quaternion2.cv[0] = rx.buf[35];
			Quaternion2.cv[1] = rx.buf[36];
			Quaternion2.cv[2] = rx.buf[37];
			Quaternion2.cv[3] = rx.buf[38];
			
			Quaternion3.cv[0] = rx.buf[39];
			Quaternion3.cv[1] = rx.buf[40];
			Quaternion3.cv[2] = rx.buf[41];
			Quaternion3.cv[3] = rx.buf[42];
			//期望的位置数据
			reference_posx.cv[0] = rx.buf[43];
			reference_posx.cv[1] = rx.buf[44];
			reference_posx.cv[2] = rx.buf[45];
			reference_posx.cv[3] = rx.buf[46];

			reference_posy.cv[0] = rx.buf[47];
			reference_posy.cv[1] = rx.buf[48];
			reference_posy.cv[2] = rx.buf[49];
			reference_posy.cv[3] = rx.buf[50];

			reference_posz.cv[0] = rx.buf[51];
			reference_posz.cv[1] = rx.buf[52];
			reference_posz.cv[2] = rx.buf[53];
			reference_posz.cv[3] = rx.buf[54];
		}
	}
}


/**********************************************************************************************************
*函 数 名: GetVisualOdometryPos
*功能说明: 获取视觉里程计的Pos
*形    参: 无
*返 回 值: Position
**********************************************************************************************************/
Vector3f_t GetVisualOdometryPos(void){
	Vector3f_t Position;
	Position.x = position_x.fvalue;
	Position.y = position_y.fvalue;
	Position.z = position_z.fvalue;
  return Position;
}
/**********************************************************************************************************
*函 数 名: GetVisualOdometryVel
*功能说明: 获取视觉里程计的Vel
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
Vector3f_t GetVisualOdometryVel(void){
	Vector3f_t Velocity;
	Velocity.x = velocity_x.fvalue;
	Velocity.y = velocity_y.fvalue;
	Velocity.z = velocity_z.fvalue;
  return Velocity;
}
/**********************************************************************************************************
*函 数 名: GetVisualOdometryAtt
*功能说明: 获取视觉里程计的Att
*形    参: 无
*返 回 值: Attitude
**********************************************************************************************************/
Vector3angle_t GetVisualOdometryAtt(void){
	Vector3angle_t Attitude;
	Attitude.roll = atan2(2.0f*(Quaternion0.fvalue*Quaternion1.fvalue + Quaternion2.fvalue*Quaternion3.fvalue), 
											1 - 2.0f*(Quaternion1.fvalue*Quaternion1.fvalue + Quaternion2.fvalue*Quaternion2.fvalue));
	Attitude.pitch = safe_asin(2.0f*(Quaternion0.fvalue*Quaternion2.fvalue - Quaternion1.fvalue*Quaternion3.fvalue));
	Attitude.yaw = atan2(2.0f*Quaternion1.fvalue*Quaternion2.fvalue + 2.0f*Quaternion0.fvalue*Quaternion3.fvalue, 
											-2.0f*Quaternion2.fvalue*Quaternion2.fvalue - 2.0f*Quaternion3.fvalue*Quaternion3.fvalue + 1);
  return Attitude;
}
/**********************************************************************************************************
*函 数 名: GetVisualOdometryRefPos
*功能说明: 获取视觉里程计的Vel
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
Vector3f_t GetVisualOdometryRefPos(void){
	Vector3f_t RefPosition;
	RefPosition.x = reference_posx.fvalue;
	RefPosition.y = reference_posy.fvalue;
	RefPosition.z = reference_posz.fvalue;
  return RefPosition;
}
/**********************************************************************************************************
*函 数 名: GetVisualOdometryStatus
*功能说明: 获取视觉里程计的状态
*形    参: 无
*返 回 值: Status
**********************************************************************************************************/
bool GetVisualOdometryStatus(void){
	static Vector3f_t lastPosition;
	Vector3f_t Position;
	Position = GetVisualOdometryPos();
	bool status;
	if(abs(Position.x - lastPosition.x)!=0 
		|| abs(Position.y - lastPosition.y)!=0 
			|| abs(Position.z - lastPosition.z)!=0){
			status = true;
	}else{
		status = false;
	}
	lastPosition = Position;
	
  return status;
}
/**********************************************************************************************************
*函 数 名: GetFPSVisualOdometry
*功能说明: 返回视觉里程计的FPS
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
float GetFPSVisualOdometry(void){
  return FPSVisualOdometry.CurrentTime;
}

