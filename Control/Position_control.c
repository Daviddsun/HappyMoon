#include "Position_control.h"
Vector4PosController PosControllerOut;
FPS_PositionControl FPSPositionControl;

/**********************************************************************************************************
*函 数 名: Position_Controller
*功能说明: 位置控制器
*形    参: 期望位置，期望速度，估计位置，估计数据
*返 回 值: 无
**********************************************************************************************************/
void Position_Controller(void){
	OS_ERR err;
	Vector3f_t ExpectPos,ExpectVel,ErrorVel;	
	static Vector3f_t EstimatePosLpf,EstimateVelLpf;
	//计算函数运行时间间隔
	FPSPositionControl.CurrentTime = (OSTimeGet(&err) - FPSPositionControl.LastTime) * 1e-3;
  FPSPositionControl.LastTime = OSTimeGet(&err);
	// 期望获取
	ExpectPos = GetVisualOdometryRefPos();
	ExpectVel = GetVisualOdometryRefVel();
	TransVelToBodyFrame(ExpectPos, &ExpectPos, GetVisualOdometryAngle().yaw);
	TransVelToBodyFrame(ExpectVel, &ExpectVel, GetVisualOdometryAngle().yaw);
/******* 降落控制 ********/	
	if(GetCopterFlyMode() == Land){
		ExpectVel.z = -0.25f;
		if(GetCopterPosition().z < 0.05f){
			SetCopterFlyMode(Nothing);
			SetCopterStatus(Drone_Off);
		}
	}
/******* 高度原始串级PID控制 ********/	
	// 获取当前飞机位置，并低通滤波，减少数据噪声对控制的干扰
	// 来自自身卡尔曼滤波
	EstimatePosLpf.x = EstimatePosLpf.x * 0.95f + GetCopterPosition().x * 0.05f;
	EstimatePosLpf.y = EstimatePosLpf.y * 0.95f + GetCopterPosition().y * 0.05f;
	EstimatePosLpf.z = EstimatePosLpf.z * 0.9f + GetCopterPosition().z * 0.1f;
	// 计算速度期望
	if(GetCopterFlyMode() == Nothing){
		//速度限幅在0.5m/s
		ExpectVel.z = OriginalPosZ.kP * (ExpectPos.z - EstimatePosLpf.z);
		ExpectVel.z = ConstrainFloat(ExpectVel.z,-0.5,0.5);
	}
	// 对速度测量值进行低通滤波，减少数据噪声对控制器的影响
	// 来自自身卡尔曼滤波
	EstimateVelLpf.x = EstimateVelLpf.x * 0.9f + GetCopterVelocity().x * 0.1f;
	EstimateVelLpf.y = EstimateVelLpf.y * 0.9f + GetCopterVelocity().y * 0.1f;
	EstimateVelLpf.z = EstimateVelLpf.z * 0.9f + GetCopterVelocity().z * 0.1f;
	//速度误差计算
	ErrorVel.z = ExpectVel.z - EstimateVelLpf.z;
	//PID计算
	PosControllerOut.ExpectAcc = PID_GetPID(&OriginalVelZ, ErrorVel.z, FPSPositionControl.CurrentTime) + Gravity_Acceleration;

/******* 位置控制 ********/		
	// x轴加速度
	float x_pos_error = ExpectPos.x - EstimatePosLpf.x;
	x_pos_error = ConstrainFloat(x_pos_error,-0.6,0.6);	
	float x_vel_error = ExpectVel.x - EstimateVelLpf.x;
	x_vel_error = ConstrainFloat(x_vel_error,-1.0,1.0);	
//	PosControllerOut.ExpectAngle.pitch = (OriginalPosX.kP * x_pos_error + OriginalPosX.kD * x_vel_error) * PI/180;
	// y轴加速度
	float y_pos_error = ExpectPos.y - EstimatePosLpf.y;
	y_pos_error = ConstrainFloat(y_pos_error,-0.6,0.6);
	float y_vel_error = ExpectVel.y - EstimateVelLpf.y;
	y_vel_error = ConstrainFloat(y_vel_error,-1.0,1.0);
//	PosControllerOut.ExpectAngle.roll = - (OriginalPosY.kP * y_pos_error + OriginalPosY.kD * y_vel_error) * PI/180;	
	
	PosControllerOut.ExpectAngle.pitch = -GetRemoteControlFlyData().XaxisPos * 0.04f * PI/180;
	PosControllerOut.ExpectAngle.roll = GetRemoteControlFlyData().YaxisPos * 0.04f * PI/180;
	
/******* 苏黎世控制框架暂不使用，因为没有解决视觉里程计与自身飞控板之间的四元数对齐问题 ********/
//	Vector3f_t acc_error,EstimatePos,EstimateVel,ExpectVel;
////	//获取kalman里面的速度和位移
////	EstimatePos = GetCopterPosition();
////	EstimateVel = GetCopterVelocity();
//	//期望速度获取 二次函数获取 1.5f * X * X
//	ExpectVel.x = 1.5f * (ExpectPos.x - EstimatePos.x) * (ExpectPos.x - EstimatePos.x);
//	ExpectVel.y = 1.5f * (ExpectPos.y - EstimatePos.y) * (ExpectPos.y - EstimatePos.y);
//	ExpectVel.z = 1.5f * (ExpectPos.z - EstimatePos.z) * (ExpectPos.z - EstimatePos.z);
//	// x轴加速度
//	float x_pos_error = ExpectPos.x - EstimatePos.x;
//	x_pos_error = ConstrainFloat(x_pos_error,-pxy_error_max,pxy_error_max);
//	float x_vel_error = ExpectVel.x - EstimateVel.x;
//	x_vel_error = ConstrainFloat(x_vel_error,-vxy_error_max,vxy_error_max);
//	acc_error.x = OriginalVelZ.kP * x_pos_error + OriginalVelZ.kD * x_vel_error;
//	// y轴加速度
//	float y_pos_error = ExpectPos.y - EstimatePos.y;
//	y_pos_error = ConstrainFloat(y_pos_error,-pxy_error_max,pxy_error_max);
//	float y_vel_error = ExpectVel.y - EstimateVel.y;
//	y_vel_error = ConstrainFloat(y_vel_error,-vxy_error_max,vxy_error_max);
//	acc_error.y = OriginalVelZ.kP * y_pos_error + OriginalVelZ.kD * y_vel_error;	
//	// z轴加速度
//	float z_pos_error = ExpectPos.z - EstimatePos.z;
//	z_pos_error = ConstrainFloat(z_pos_error,-pz_error_max,pz_error_max);
//	float z_vel_error = ExpectVel.z - EstimateVel.z;
//	z_vel_error = ConstrainFloat(z_vel_error,-vz_error_max,vz_error_max);
//	acc_error.z = OriginalPosZ.kP * z_pos_error + OriginalPosZ.kD * z_vel_error + Gravity_Acceleration; //z轴加上重力加速度
////	// 计算期望控制量
//	PosControllerOut.ExpectAcc = acc_error.z;
//	PosControllerOut.ExpectAcc = Gravity_Acceleration + (GetRemoteControlFlyData().ZaxisPos) * 2.5f;
//	PosControllerOut.ExpectAngle.pitch = -GetRemoteControlFlyData().XaxisPos * 0.04f * PI/180;
//	PosControllerOut.ExpectAngle.roll = GetRemoteControlFlyData().YaxisPos * 0.04f * PI/180;
//	PosControllerOut.ExpectAngle.yaw = 0;
}
/**********************************************************************************************************
*函 数 名: GetDesiredControlAcc
*功能说明: 获取期望加速度
*形    参: 期望位置，期望速度，估计位置，估计数据
*返 回 值: 无
**********************************************************************************************************/	
float GetDesiredControlAcc(void){
	return PosControllerOut.ExpectAcc;
}
/**********************************************************************************************************
*函 数 名: GetDesiredControlAngle
*功能说明: 获取期望角度值
*形    参: 期望位置，期望速度，估计位置，估计数据
*返 回 值: 无
**********************************************************************************************************/	
Vector3angle_t GetDesiredControlAngle(void){
	return PosControllerOut.ExpectAngle;
}
/**********************************************************************************************************
*函 数 名: ResetPositionPara
*功能说明: 置位位置参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
void ResetPositionPara(void){
	OriginalVelX.integrator = 0;
	OriginalVelY.integrator = 0;
	OriginalVelZ.integrator = 0;
	//roll pitch 最大积分角度 5°
	OriginalVelX.imax = 5;
	OriginalVelY.imax = 5;
	//height 最大积分加速度 5m/s^2
	OriginalVelZ.imax = 5;
	//位置输出归零
	PosControllerOut.ExpectAcc = 0;
	PosControllerOut.ExpectAngle.pitch = 0;
	PosControllerOut.ExpectAngle.roll = 0;
	PosControllerOut.ExpectAngle.yaw = 0;
}

