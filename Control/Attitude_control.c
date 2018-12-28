#include "Attitude_control.h"
/**********************************************************************************************************
*函 数 名: Attitude_Control
*功能说明: 飞行内环控制，包括姿态内环和高度内环控制
*形    参: 角速度测量值
*返 回 值: 无
**********************************************************************************************************/
Vector3f_t Attitude_InnerControl(Vector3f_t ExpectGyro, Vector3f_t EstimateGyro){
	OS_ERR err;
	//计算函数运行时间间隔
  static uint64_t previousT;	
	float deltaT = (OSTimeGet(&err) - previousT) * 1e-3;
  previousT = OSTimeGet(&err);
	Vector3f_t ErrorGyro,Thrust;
	
	//计算角速度环控制误差：目标角速度 - 实际角速度（低通滤波后的陀螺仪测量值）
	ErrorGyro.x = ExpectGyro.x - EstimateGyro.x;
	ErrorGyro.y = ExpectGyro.y - EstimateGyro.y;
	ErrorGyro.z = ExpectGyro.z - EstimateGyro.z;
	//PID算法，计算出角速度环的控制量
	Thrust.x = PID_GetPID(&OriginalWxRate, ErrorGyro.x, deltaT)
												+ EstimateGyro.y * (Inertia_Wz * EstimateGyro.z) - (Inertia_Wy * EstimateGyro.y) * EstimateGyro.z;//考虑轴间耦合现象
	Thrust.y = PID_GetPID(&OriginalWyRate, ErrorGyro.y, deltaT)
												+ (-(EstimateGyro.x * (Inertia_Wz * EstimateGyro.z) - (Inertia_Wx * EstimateGyro.x) * EstimateGyro.z));//考虑轴间耦合现象
	Thrust.z = PID_GetPID(&OriginalWzRate, ErrorGyro.z, deltaT)
												+ EstimateGyro.x * (Inertia_Wy * EstimateGyro.y) - (Inertia_Wx * EstimateGyro.x) * EstimateGyro.y;//考虑轴间耦合现象
	return Thrust;
}

/**********************************************************************************************************
*函 数 名: AttitudeOuterControl
*功能说明: 姿态外环控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
Vector3angle_t Attitude_OuterControl(Vector3angle_t ExpectAngle){
  Vector3angle_t Angle,AngleLpf,ExpectAnguleRate,ErrorAngle;
	//获取当前飞机的姿态角
//  angle = GetCopterAngle();
	//对姿态测量值进行低通滤波，减少数据噪声对控制器的影响
	AngleLpf.roll = AngleLpf.roll * 0.92f + Angle.roll * 0.08f;
	AngleLpf.pitch = AngleLpf.pitch * 0.92f + Angle.pitch * 0.08f;
	AngleLpf.yaw = AngleLpf.yaw * 0.92f + Angle.yaw * 0.08f;
	//计算姿态外环控制误差：目标角度 - 实际角度
	ErrorAngle.roll = ExpectAngle.roll - AngleLpf.roll;
	ErrorAngle.pitch = ExpectAngle.pitch - AngleLpf.pitch;
	ErrorAngle.yaw = ExpectAngle.yaw - AngleLpf.yaw;
	//PID算法，计算出姿态外环的控制量
	ExpectAnguleRate.roll = PID_GetP(&OriginalRoll,  ErrorAngle.roll);
	ExpectAnguleRate.pitch = PID_GetP(&OriginalPitch, ErrorAngle.pitch);
	ExpectAnguleRate.yaw = PID_GetP(&OriginalYaw, ErrorAngle.yaw);
	return ExpectAnguleRate;
}




