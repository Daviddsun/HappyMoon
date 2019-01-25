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
	Vector3f_t ExpectPos,ExpectVel,ErrorVel,PosPID,VelPID,PosPIDTrans;	
	static Vector3f_t EstimatePosLpf,EstimateVelLpf;
	static uint8_t FlightMethod;
	float x_pos_error,x_vel_error,y_pos_error,y_vel_error;
	//计算函数运行时间间隔
	FPSPositionControl.CurrentTime = (OSTimeGet(&err) - FPSPositionControl.LastTime) * 1e-3;
  FPSPositionControl.LastTime = OSTimeGet(&err);
	//获取飞行模式
	FlightMethod = GetCopterFlightMethod();
	//模式选择
	switch(FlightMethod){
		case 0:
			//姿态
			PosControllerOut.ExpectAngle.pitch = -GetRemoteControlFlyData().XaxisPos * 0.04f * PI/180;
			
			PosControllerOut.ExpectAngle.roll = GetRemoteControlFlyData().YaxisPos * 0.04f * PI/180;
			
			PosControllerOut.ExpectAngle.yaw = 0;	
			//高度
			PosControllerOut.ExpectAcc = GetRemoteControlFlyData().ZaxisPos * 5.0f + Gravity_Acceleration;
			
			break;
		case 1:
			
			break;
		case 2:
			
			break;
		case 3:
			// 期望获取
			ExpectPos = GetVisualOdometryRefPos();
			ExpectVel = GetVisualOdometryRefVelTrans();

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
			EstimatePosLpf.z = EstimatePosLpf.z * 0.95f + GetCopterPosition().z * 0.05f;
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
			x_pos_error = ExpectPos.x - EstimatePosLpf.x;
			x_pos_error = ConstrainFloat(x_pos_error,-0.6,0.6);	
			PosPID.x = PID_GetPID(&OriginalPosX, x_pos_error, FPSPositionControl.CurrentTime); 																						//位移PID
			x_vel_error = ExpectVel.x - EstimateVelLpf.x;
			x_vel_error = ConstrainFloat(x_vel_error,-1.0,1.0);	
			VelPID.x = PID_GetPID(&OriginalVelX, x_vel_error, FPSPositionControl.CurrentTime);                                            //速度PID
			
			y_pos_error = ExpectPos.y - EstimatePosLpf.y;
			y_pos_error = ConstrainFloat(y_pos_error,-0.6,0.6);
			PosPID.y = PID_GetPID(&OriginalPosY, y_pos_error, FPSPositionControl.CurrentTime);																						//位移PID
			y_vel_error = ExpectVel.y - EstimateVelLpf.y;
			y_vel_error = ConstrainFloat(y_vel_error,-1.0,1.0);
			VelPID.y = PID_GetPID(&OriginalVelY, y_vel_error, FPSPositionControl.CurrentTime);																						//速度PID
			
			//将位置的控制量转成机体坐标系
			TransVelToBodyFrame(PosPID, &PosPIDTrans, GetVisualOdometryAngle().yaw);
			
			PosControllerOut.ExpectAngle.pitch = (PosPIDTrans.x + VelPID.x) * PI/180;
			
			PosControllerOut.ExpectAngle.roll = -(PosPIDTrans.y + VelPID.y) * PI/180;
			
			PosControllerOut.ExpectAngle.yaw = 0;
			break;
		
		default:
			break;
		}
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
	//位移积分归零
	OriginalPosX.integrator = 0;
	OriginalPosY.integrator = 0;
	OriginalPosZ.integrator = 0;
	//位置积分最大
	OriginalPosX.imax = 5;
	OriginalPosY.imax = 5;
	//速度积分归零
	OriginalVelX.integrator = 0;
	OriginalVelY.integrator = 0;
	OriginalVelZ.integrator = 0;
	//速度积分最大
	OriginalVelX.imax = 5;
	OriginalVelY.imax = 5;
	//height 最大积分加速度 5.0m/s^2
	OriginalVelZ.imax = 5.0;
	//位置输出归零
	PosControllerOut.ExpectAcc = 0;
	PosControllerOut.ExpectAngle.pitch = 0;
	PosControllerOut.ExpectAngle.roll = 0;
	PosControllerOut.ExpectAngle.yaw = 0;
}

