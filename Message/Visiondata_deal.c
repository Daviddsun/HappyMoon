/**********************************************************************************************************
 * @文件     Visiondata_deal.c
 * @说明     视觉数据接收
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "Visiondata_deal.h"
FPS_VisualOdometry FPSVisualOdometry;
FPS_ReferenceRoute FPSReferenceRoute;
//估计状态定义
float_union position_x,position_y,position_z,                       //估计位置
							velocity_x,velocity_y,velocity_z,										  //估计速度
								Quaternion0,Quaternion1,Quaternion2,Quaternion3,	  //估计姿态
									reference_posx,reference_posy,reference_posz,			//参考航点
										reference_velx,reference_vely,reference_velz,   //参考速度
											reference_accx,reference_accy,reference_accz; //参考加速度
/**********************************************************************************************************
*函 数 名: Vision_datadeal
*功能说明: 接收视觉里程计数据
*形    参: 无
*返 回 值: Position
**********************************************************************************************************/
void Vision_DataDeal(Receive_VisualOdometry rx){
	OS_ERR err;
	if(rx.buf[0]==0x55 && rx.buf[1]==0xAA && rx.buf[43]==0xAA){
		if(rx.buf[2] == 0x10){
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
		}
		if(rx.buf[2] == 0x20){
			//计算函数运行时间间隔
			FPSReferenceRoute.CurrentTime = (OSTimeGet(&err) - FPSReferenceRoute.LastTime) * 1e-3;
			FPSReferenceRoute.LastTime = OSTimeGet(&err);
			//期望的位置数据
			reference_posx.cv[0] = rx.buf[3];
			reference_posx.cv[1] = rx.buf[4];
			reference_posx.cv[2] = rx.buf[5];
			reference_posx.cv[3] = rx.buf[6];

			reference_posy.cv[0] = rx.buf[7];
			reference_posy.cv[1] = rx.buf[8];
			reference_posy.cv[2] = rx.buf[9];
			reference_posy.cv[3] = rx.buf[10];

			reference_posz.cv[0] = rx.buf[11];
			reference_posz.cv[1] = rx.buf[12];
			reference_posz.cv[2] = rx.buf[13];
			reference_posz.cv[3] = rx.buf[14];
			//期望的速度数据
			reference_velx.cv[0] = rx.buf[15];
			reference_velx.cv[1] = rx.buf[16];
			reference_velx.cv[2] = rx.buf[17];
			reference_velx.cv[3] = rx.buf[18];

			reference_vely.cv[0] = rx.buf[19];
			reference_vely.cv[1] = rx.buf[20];
			reference_vely.cv[2] = rx.buf[21];
			reference_vely.cv[3] = rx.buf[22];

			reference_velz.cv[0] = rx.buf[23];
			reference_velz.cv[1] = rx.buf[24];
			reference_velz.cv[2] = rx.buf[25];
			reference_velz.cv[3] = rx.buf[26];
			//期望的加速度数据
			reference_accx.cv[0] = rx.buf[27];
			reference_accx.cv[1] = rx.buf[28];
			reference_accx.cv[2] = rx.buf[29];
			reference_accx.cv[3] = rx.buf[30];
			
			reference_accy.cv[0] = rx.buf[31];
			reference_accy.cv[1] = rx.buf[32];
			reference_accy.cv[2] = rx.buf[33];
			reference_accy.cv[3] = rx.buf[34];
			
			reference_accz.cv[0] = rx.buf[35];
			reference_accz.cv[1] = rx.buf[36];
			reference_accz.cv[2] = rx.buf[37];
			reference_accz.cv[3] = rx.buf[38];
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
*函 数 名: GetVisualOdometryAngle
*功能说明: 获取视觉里程计的Angle
*形    参: 无
*返 回 值: Attitude
**********************************************************************************************************/
Vector3angle_t GetVisualOdometryAngle(void){
	Vector3angle_t Attitude;
	Attitude.roll = atan2(2.0f*(Quaternion0.fvalue*Quaternion1.fvalue + Quaternion2.fvalue*Quaternion3.fvalue), 
											1 - 2.0f*(Quaternion1.fvalue*Quaternion1.fvalue + Quaternion2.fvalue*Quaternion2.fvalue));
	Attitude.pitch = safe_asin(2.0f*(Quaternion0.fvalue*Quaternion2.fvalue - Quaternion1.fvalue*Quaternion3.fvalue));
	Attitude.yaw = atan2(2.0f*Quaternion1.fvalue*Quaternion2.fvalue + 2.0f*Quaternion0.fvalue*Quaternion3.fvalue, 
											-2.0f*Quaternion2.fvalue*Quaternion2.fvalue - 2.0f*Quaternion3.fvalue*Quaternion3.fvalue + 1);
  return Attitude;
}
/**********************************************************************************************************
*函 数 名: GetVisualOdometryVelTrans
*功能说明: 将VIO坐标系下的速度转化到机体坐标系
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
Vector3f_t GetVisualOdometryVelTrans(void){
	Vector3f_t TransVelocity;
	TransVelToBodyFrame(GetVisualOdometryVel(),&TransVelocity,GetVisualOdometryAngle().yaw);
  return TransVelocity;
}
/**********************************************************************************************************
*函 数 名: GetWayPointRefPos
*功能说明: 获取航向规划里面的位移（对其坐标）
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
Vector3f_t GetWayPointRefPos(void){
	Vector3f_t RefPosition;
	RefPosition.x = reference_posy.fvalue;
	RefPosition.y = -reference_posx.fvalue;
	RefPosition.z = reference_posz.fvalue;
	//期望高度不低于0.5m
	if(RefPosition.z < 0.5f){
		RefPosition.z = 0.5f;
	}
  return RefPosition;
}
/**********************************************************************************************************
*函 数 名: GetWayPointRefVel
*功能说明: 获取航向规划里面的速度（对其坐标）
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
Vector3f_t GetWayPointRefVel(void){
	Vector3f_t RefVelocity;
	RefVelocity.x = reference_vely.fvalue;
	RefVelocity.y = -reference_velx.fvalue;
	RefVelocity.z = reference_velz.fvalue;
  return RefVelocity;
}

/**********************************************************************************************************
*函 数 名: GetWayPointRefVelTrans
*功能说明: 获取航向规划里面的速度转化到机体坐标系
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
Vector3f_t GetWayPointRefVelTrans(void){
	Vector3f_t RefVelocityTrans;
	TransVelToBodyFrame(GetWayPointRefVel(),&RefVelocityTrans,GetVisualOdometryAngle().yaw);
  return RefVelocityTrans;
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
/**********************************************************************************************************
*函 数 名: GetFPSWayPointNav
*功能说明: 返回的FPS
*形    参: 无
*返 回 值: Velocity
**********************************************************************************************************/
float GetFPSWayPointNav(void){
  return FPSReferenceRoute.CurrentTime;
}
