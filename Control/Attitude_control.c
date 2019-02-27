#include "Attitude_control.h"
FPS_AttitudeControl FPSAttitudeControl;
/**********************************************************************************************************
*函 数 名: Attitude_Control
*功能说明: 飞行内环控制，包括姿态内环和高度内环控制
*形    参: 角速度测量值
*返 回 值: 无
**********************************************************************************************************/
Vector3f_t Attitude_InnerController(Vector3f_t ExpectGyro, Vector3f_t EstimateGyro){
	OS_ERR err;
	Vector3f_t ErrorGyro,Thrust;
	static Vector3f_t LastEstimateGyro;
	//计算函数运行时间间隔
	FPSAttitudeControl.CurrentTime = (OSTimeGet(&err) - FPSAttitudeControl.LastTime) * 1e-3;
  FPSAttitudeControl.LastTime = OSTimeGet(&err);
	
	//计算角速度环控制误差：目标角速度 - 实际角速度（低通滤波后的陀螺仪测量值）
	ErrorGyro.x = ExpectGyro.x - EstimateGyro.x;
	ErrorGyro.y = ExpectGyro.y - EstimateGyro.y;
	ErrorGyro.z = ExpectGyro.z - EstimateGyro.z;
	//PID算法，计算出角速度环的控制量
	Thrust.x = PID_GetPID(&OriginalWxRate, ErrorGyro.x, FPSAttitudeControl.CurrentTime)
												//轴间耦合
												+ EstimateGyro.y * (Inertia_Wz * EstimateGyro.z) - (Inertia_Wy * EstimateGyro.y) * EstimateGyro.z
															//角速度前馈
															+ Inertia_Wx * (EstimateGyro.x - LastEstimateGyro.x) * 1000.0f;
	
	Thrust.y = PID_GetPID(&OriginalWyRate, ErrorGyro.y, FPSAttitudeControl.CurrentTime)
												+ (-(EstimateGyro.x * (Inertia_Wz * EstimateGyro.z) - (Inertia_Wx * EstimateGyro.x) * EstimateGyro.z))
															+ Inertia_Wy * (EstimateGyro.y - LastEstimateGyro.y) * 1000.0f;
															
	Thrust.z = PID_GetPID(&OriginalWzRate, ErrorGyro.z, FPSAttitudeControl.CurrentTime)
												+ EstimateGyro.x * (Inertia_Wy * EstimateGyro.y) - (Inertia_Wx * EstimateGyro.x) * EstimateGyro.y
															+ Inertia_Wz * (EstimateGyro.z - LastEstimateGyro.z) * 1000.0f;
															
	LastEstimateGyro.x = EstimateGyro.x;
	LastEstimateGyro.y = EstimateGyro.y;
	LastEstimateGyro.z = EstimateGyro.z;
	
	return Thrust;
}

/**********************************************************************************************************
*函 数 名: AttitudeOuterControl
*功能说明: 姿态外环控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
Vector3angle_t Attitude_OuterController(Vector3angle_t ExpectAngle){
	static Vector3angle_t AngleLpf;
  Vector3angle_t Angle,VIOAngle,ExpectAnguleRate,ErrorAngle;
	//获取当前飞机的姿态角(mahany filter)
  Angle = GetCopterAngle();
	//获取视觉里程计姿态
	VIOAngle = GetVisualOdometryAngle();
	//对姿态测量值进行低通滤波，减少数据噪声对控制器的影响
	AngleLpf.roll = AngleLpf.roll * 0.95f + Angle.roll * 0.05f;
	AngleLpf.pitch = AngleLpf.pitch * 0.95f + Angle.pitch * 0.05f;
	AngleLpf.yaw = VIOAngle.yaw;
	//计算姿态外环控制误差：目标角度 - 实际角度
	ErrorAngle.roll = ExpectAngle.roll - AngleLpf.roll;
	ErrorAngle.pitch = ExpectAngle.pitch - AngleLpf.pitch;
	ErrorAngle.yaw = ExpectAngle.yaw - AngleLpf.yaw;
	//PID算法，计算出姿态外环的控制量 限幅 2.5rad/s
	ExpectAnguleRate.roll = PID_GetP(&OriginalRoll,  ErrorAngle.roll);
	ExpectAnguleRate.roll = ConstrainFloat(ExpectAnguleRate.roll,-2.5,2.5); 
	ExpectAnguleRate.pitch = PID_GetP(&OriginalPitch, ErrorAngle.pitch);
	ExpectAnguleRate.pitch = ConstrainFloat(ExpectAnguleRate.pitch,-2.5,2.5);
	ExpectAnguleRate.yaw = PID_GetP(&OriginalYaw, ErrorAngle.yaw);
	ExpectAnguleRate.yaw = ConstrainFloat(ExpectAnguleRate.yaw,-2.5,2.5);
	return ExpectAnguleRate;
}

/**********************************************************************************************************
*函 数 名: GetFPSAttitudeControl
*功能说明: 获取姿态控制频率
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float GetFPSAttitudeControl(void){
	return FPSAttitudeControl.CurrentTime;
}


