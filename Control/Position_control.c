#include "Position_control.h"
Vector4PosController PosControllerOut;

/**********************************************************************************************************
*函 数 名: Position_Controller
*功能说明: 位置控制器
*形    参: 期望位置，期望速度，估计位置，估计数据
*返 回 值: 无
**********************************************************************************************************/
void Position_Controller(Vector3f_t ExpectPos){
	Vector3f_t acc_error,EstimatePos,EstimateVel,ExpectVel;
//	//获取kalman里面的速度和位移
//	EstimatePos = GetCopterPosition();
//	EstimateVel = GetCopterVelocity();
	//期望速度获取 二次函数获取 1.5f * X * X
	ExpectVel.x = 1.5f * (ExpectPos.x - EstimatePos.x) * (ExpectPos.x - EstimatePos.x);
	ExpectVel.y = 1.5f * (ExpectPos.y - EstimatePos.y) * (ExpectPos.y - EstimatePos.y);
	ExpectVel.z = 1.5f * (ExpectPos.z - EstimatePos.z) * (ExpectPos.z - EstimatePos.z);
	// x轴加速度
	float x_pos_error = ExpectPos.x - EstimatePos.x;
	x_pos_error = ConstrainFloat(x_pos_error,-pxy_error_max,pxy_error_max);
	float x_vel_error = ExpectVel.x - EstimateVel.x;
	x_vel_error = ConstrainFloat(x_vel_error,-vxy_error_max,vxy_error_max);
	acc_error.x = OriginalVelZ.kP * x_pos_error + OriginalVelZ.kD * x_vel_error;
	// y轴加速度
	float y_pos_error = ExpectPos.y - EstimatePos.y;
	y_pos_error = ConstrainFloat(y_pos_error,-pxy_error_max,pxy_error_max);
	float y_vel_error = ExpectVel.y - EstimateVel.y;
	y_vel_error = ConstrainFloat(y_vel_error,-vxy_error_max,vxy_error_max);
	acc_error.y = OriginalVelZ.kP * y_pos_error + OriginalVelZ.kD * y_vel_error;	
	// z轴加速度
	float z_pos_error = ExpectPos.z - EstimatePos.z;
	z_pos_error = ConstrainFloat(z_pos_error,-pz_error_max,pz_error_max);
	float z_vel_error = ExpectVel.z - EstimateVel.z;
	z_vel_error = ConstrainFloat(z_vel_error,-vz_error_max,vz_error_max);
	acc_error.z = OriginalPosZ.kP * z_pos_error + OriginalPosZ.kD * z_vel_error + Gravity_Acceleration; //z轴加上重力加速度
//	// 计算期望控制量
//	PosControllerOut.ExpectAcc = acc_error.z;
	PosControllerOut.ExpectAcc = Gravity_Acceleration + (GetRemoteControlFlyData().ZaxisPos) * 2.5f;
	PosControllerOut.ExpectAngle.pitch = -GetRemoteControlFlyData().XaxisPos * 0.04f * PI/180;
	PosControllerOut.ExpectAngle.roll = GetRemoteControlFlyData().YaxisPos * 0.04f * PI/180;
	PosControllerOut.ExpectAngle.yaw = 0;
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


//	Target_Info.Pitch = -RockerControl.XaxisPos * 0.04f * PI/180; 
//	
//	Target_Info.Roll = RockerControl.YaxisPos * 0.04f * PI/180;


	/************************ 降落设置 ************************/
//	if(FlightControl.DroneMode==1){
//		reference_state.postionZ = reference_state.postionZ - 0.005f;
//		if(state_estimate.postionZ < 0.05f){
//			FlightControl.OnOff = Drone_Off;
//      FlightControl.landFlag = 0;
//		}
//	}
//	if(PositionDivision){
//		/************************ X轴 ************************/
//		OriginalPosX.value = PID_ParaInfo.PosX.Kp * (reference_state.postionX - state_estimate.postionX);
//		/************************ Y轴 ************************/
//		OriginalPosY.value = PID_ParaInfo.PosY.Kp * (reference_state.postionY - state_estimate.postionY);
//		/************************ Z轴************************/
//		OriginalPosZ.value = PID_ParaInfo.PosZ.Kp * (reference_state.postionZ - state_estimate.postionZ);
//	}
//	PositionDivision = ~PositionDivision;

//	/************************ X轴控制 ************************/
//	Target_Info.Roll =  PID_Control(&PID_ParaInfo.VelX,&OriginalVelX,OriginalPosX.value,state_estimate.velocityX,0.02,5,Filter20Hz) * PI/180;	
//	/************************ Y轴控制 ************************/
//	Target_Info.Pitch = PID_Control(&PID_ParaInfo.VelY,&OriginalVelY,OriginalPosY.value,state_estimate.velocityY,0.02,5,Filter20Hz) * PI/180;		
//	/************************ Z轴控制 ************************/
//	UAVThrust.collective_thrust = PID_Control(&PID_ParaInfo.VelZ,&OriginalVelZ,OriginalPosZ.value,state_estimate.velocityZ,0.02,75,Filter20Hz);

