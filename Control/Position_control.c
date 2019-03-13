#include "Position_control.h"
Vector4PosController PosControllerOut;
FPS_PositionControl FPSPositionControl;
FPS_PositionControl FPSAltitudeControl;
/**********************************************************************************************************
*函 数 名: Position_Controller
*功能说明: 位置控制器
*形    参: 期望位置，期望速度，估计位置，估计数据
*返 回 值: 无
**********************************************************************************************************/
 void Position_InnerController(void){
	OS_ERR err;
	Vector3f_t ErrorVel;//PosPID,VelPID,PosPIDTrans
	static Vector3f_t EstimateVelLpf;
	static uint8_t FlightMethod;
//	float x_pos_error,x_vel_error,y_pos_error,y_vel_error;
	//计算函数运行时间间隔
	FPSPositionControl.CurrentTime = (OSTimeGet(&err) - FPSPositionControl.LastTime) * 1e-3;
	FPSPositionControl.CurrentTime = ConstrainFloat(FPSPositionControl.CurrentTime, 0.005, 0.02);
  FPSPositionControl.LastTime = OSTimeGet(&err);
	//获取飞行模式
	FlightMethod = GetCopterFlightMethod();
	//模式选择
	switch(FlightMethod){
		//姿态飞行控制
		case 0:
			//姿态
			PureAttitude_Control();
			break;
		//定高飞行
		case 1:
			PureAttitude_Control();
			break;
		//遥控飞行
		case 2:
			//转化到速度为1.0m/s最高
			PosControllerOut.ExpectVel.x = -GetRemoteControlFlyData().Xaxis * 0.004f; 
			PosControllerOut.ExpectVel.y = -GetRemoteControlFlyData().Yaxis * 0.004f;
			//对速度测量值进行低通滤波，减少数据噪声对控制器的影响
			//来自自身卡尔曼滤波
			EstimateVelLpf.x = EstimateVelLpf.x * 0.95f + GetCopterVelocity().x * 0.05f;
			EstimateVelLpf.y = EstimateVelLpf.y * 0.95f + GetCopterVelocity().y * 0.05f;	
			//速度误差计算
			ErrorVel.x = PosControllerOut.ExpectVel.x - EstimateVelLpf.x;
			ErrorVel.y = PosControllerOut.ExpectVel.y - EstimateVelLpf.y;
			//角度转化为rad弧度
			PosControllerOut.ExpectAngle.roll = - PID_GetPID(&OriginalVelY, ErrorVel.y, FPSPositionControl.CurrentTime) * PI/180;
			PosControllerOut.ExpectAngle.roll = ConstrainFloat(PosControllerOut.ExpectAngle.roll, -0.175, 0.175);
			PosControllerOut.ExpectAngle.pitch = PID_GetPID(&OriginalVelX, ErrorVel.x, FPSPositionControl.CurrentTime) * PI/180;
			PosControllerOut.ExpectAngle.pitch = ConstrainFloat(PosControllerOut.ExpectAngle.pitch, -0.175, 0.175); //角度不大于10度
			PosControllerOut.ExpectAngle.yaw = 0;	
			break;
		//阶跃响应
		case 3:
			//获得期望速度
			PosControllerOut.ExpectVel = GetDesiredControlVel();
			//对速度测量值进行低通滤波，减少数据噪声对控制器的影响
			//来自自身卡尔曼滤波
			EstimateVelLpf.x = EstimateVelLpf.x * 0.95f + GetCopterVelocity().x * 0.05f;
			EstimateVelLpf.y = EstimateVelLpf.y * 0.95f + GetCopterVelocity().y * 0.05f;	
			//速度误差计算
			ErrorVel.x = PosControllerOut.ExpectVel.x - EstimateVelLpf.x;
			ErrorVel.y = PosControllerOut.ExpectVel.y - EstimateVelLpf.y;
			//角度转化为rad弧度
			PosControllerOut.ExpectAngle.roll = - PID_GetPID(&OriginalVelY, ErrorVel.y, FPSPositionControl.CurrentTime) * PI/180;
			PosControllerOut.ExpectAngle.roll = ConstrainFloat(PosControllerOut.ExpectAngle.roll, -0.175, 0.175);
			PosControllerOut.ExpectAngle.pitch = PID_GetPID(&OriginalVelX, ErrorVel.x, FPSPositionControl.CurrentTime) * PI/180;
			PosControllerOut.ExpectAngle.pitch = ConstrainFloat(PosControllerOut.ExpectAngle.pitch, -0.175, 0.175); //角度不大于10度
			PosControllerOut.ExpectAngle.yaw = 0;	
			break;
		//轨迹追踪
		case 4:
//			// 期望获取
//			ExpectPos = GetWayPointRefPos();
//			ExpectVel = GetWayPointRefVelTrans();

//			//获取当前飞机位置，并低通滤波，减少数据噪声对控制的干扰
//			//来自自身卡尔曼滤波
//			EstimatePosLpf.x = EstimatePosLpf.x * 0.95f + GetCopterPosition().x * 0.05f;
//			EstimatePosLpf.y = EstimatePosLpf.y * 0.95f + GetCopterPosition().y * 0.05f;
//			//对速度测量值进行低通滤波，减少数据噪声对控制器的影响
//			//来自自身卡尔曼滤波
//			EstimateVelLpf.x = EstimateVelLpf.x * 0.95f + GetCopterVelocity().x * 0.05f;
//			EstimateVelLpf.y = EstimateVelLpf.y * 0.95f + GetCopterVelocity().y * 0.05f;

//			/******* 位置控制 ********/		
//			x_pos_error = ExpectPos.x - EstimatePosLpf.x;
//			x_pos_error = ConstrainFloat(x_pos_error,-0.6,0.6);	
//			PosPID.x = PID_GetPID(&OriginalPosX, x_pos_error, FPSPositionControl.CurrentTime); 																						//位移PID
//			x_vel_error = ExpectVel.x - EstimateVelLpf.x;
//			x_vel_error = ConstrainFloat(x_vel_error,-1.0,1.0);	
//			VelPID.x = PID_GetPID(&OriginalVelX, x_vel_error, FPSPositionControl.CurrentTime);                                            //速度PID
//			
//			y_pos_error = ExpectPos.y - EstimatePosLpf.y;
//			y_pos_error = ConstrainFloat(y_pos_error,-0.6,0.6);
//			PosPID.y = PID_GetPID(&OriginalPosY, y_pos_error, FPSPositionControl.CurrentTime);																						//位移PID
//			y_vel_error = ExpectVel.y - EstimateVelLpf.y;
//			y_vel_error = ConstrainFloat(y_vel_error,-1.0,1.0);
//			VelPID.y = PID_GetPID(&OriginalVelY, y_vel_error, FPSPositionControl.CurrentTime);																						//速度PID
//			
//			//将位置的控制量转成机体坐标系
//			TransVelToBodyFrame(PosPID, &PosPIDTrans, GetVisualOdometryAngle().yaw);
//			
//			PosControllerOut.ExpectAngle.pitch = (PosPIDTrans.x + VelPID.x) * PI/180;
//			
//			PosControllerOut.ExpectAngle.roll = -(PosPIDTrans.y + VelPID.y) * PI/180;
//			
//			PosControllerOut.ExpectAngle.yaw = 0;
			break;
		
		default:
			break;
		}
}
/*******************************************************************************************************
*函 数 名: Position_OuterController
*功能说明: 位置外环控制
*形    参: 无
*返 回 值: 无
********************************************************************************************************/	
void Position_OuterController(void){
	Vector3f_t ExpectPos;
	static Vector3f_t EstimatePosLpf;
	//获取阶跃信号
	ExpectPos = GetStepSignalValue();
	//获取当前飞机位置，并低通滤波，减少数据噪声对控制的干扰
	//来自自身卡尔曼滤波
	EstimatePosLpf.x = EstimatePosLpf.x * 0.95f + GetCopterPosition().x * 0.05f;
	EstimatePosLpf.y = EstimatePosLpf.y * 0.95f + GetCopterPosition().y * 0.05f;	
	//速度限幅在0.75m/s
	PosControllerOut.ExpectVel.x = OriginalPosX.kP * (ExpectPos.x - EstimatePosLpf.x);
	PosControllerOut.ExpectVel.x = ConstrainFloat(PosControllerOut.ExpectVel.x,-0.75,0.75);
	//速度限幅在0.75m/s
	PosControllerOut.ExpectVel.y = OriginalPosY.kP * (ExpectPos.y - EstimatePosLpf.y);
	PosControllerOut.ExpectVel.y = ConstrainFloat(PosControllerOut.ExpectVel.y,-0.75,0.75);		
	TransVelToBodyFrame(PosControllerOut.ExpectVel, &PosControllerOut.ExpectVel, GetVisualOdometryAngle().yaw);
}
 
/********************************************************************************************************
*函 数 名: Altitude_Controller
*功能说明: 高度控制
*形    参: 无
*返 回 值: 无
*********************************************************************************************************/	
void Altitude_Controller(void){
	OS_ERR err;
	static uint8_t AltitudeFlightMethod;
	static float EstimateAltitudeLpf,EstimateAltitudeVelLpf;
	float ExpectAltitude,ExpectAltitudeVel,ErrorAltitude;
	static uint64_t count = 0;
	//计算函数运行时间间隔
	FPSAltitudeControl.CurrentTime = (OSTimeGet(&err) - FPSAltitudeControl.LastTime) * 1e-3;
	FPSAltitudeControl.CurrentTime = ConstrainFloat(FPSAltitudeControl.CurrentTime, 0.0005, 0.005);
  FPSAltitudeControl.LastTime = OSTimeGet(&err);
	//获取飞行模式
	AltitudeFlightMethod = GetCopterFlightMethod();
	if(AltitudeFlightMethod == PurePosture){
		//纯姿态
		PosControllerOut.ExpectAcc = GetRemoteControlFlyData().Zaxis * 5.0f + Gravity_Acceleration;
	}else{
		ExpectAltitude = GetStepSignalValue().z;
		/******* 降落控制 ********/	
		if(GetCopterFlyMode() == Land){
			ExpectAltitudeVel = -0.3f;
			if(GetCopterPosition().z < 0.1f){
				SetCopterFlyMode(Nothing);
				SetCopterStatus(Drone_Off);
			}
		}
		//获取当前飞机位置，并低通滤波，减少数据噪声对控制的干扰
		EstimateAltitudeLpf = EstimateAltitudeLpf * 0.95f + GetCopterPosition().z * 0.05f;
		if(count++ % 2 == 0){
			//计算速度期望
			if(GetCopterFlyMode() == Nothing){
				//速度限幅在0.75m/s
				ExpectAltitudeVel = OriginalPosZ.kP * (ExpectAltitude - EstimateAltitudeLpf);
				ExpectAltitudeVel = ConstrainFloat(ExpectAltitudeVel,-0.75,0.75);
			}
		}
		//对速度测量值进行低通滤波，减少数据噪声对控制器的影响
		//来自自身卡尔曼滤波
		EstimateAltitudeVelLpf = EstimateAltitudeVelLpf * 0.9f + GetCopterVelocity().z * 0.1f;
		//速度误差计算
		ErrorAltitude = ExpectAltitudeVel - EstimateAltitudeVelLpf;
		//PID计算
		PosControllerOut.ExpectAcc = PID_GetPID(&OriginalVelZ, ErrorAltitude, FPSAltitudeControl.CurrentTime) + Gravity_Acceleration;
	}
}

/*********************************************************************************************************
*函 数 名: PureAttitude_Control
*功能说明: 纯姿态控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
void PureAttitude_Control(void){
	//姿态
	PosControllerOut.ExpectAngle.pitch = -GetRemoteControlFlyData().Xaxis * 0.04f * PI/180;

	PosControllerOut.ExpectAngle.roll = GetRemoteControlFlyData().Yaxis * 0.04f * PI/180;

	PosControllerOut.ExpectAngle.yaw = 0;	
}

/**********************************************************************************************************
*函 数 名: GetDesiredVel
*功能说明: 获得期望的速度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/	
Vector3f_t GetDesiredControlVel(void){
	return PosControllerOut.ExpectVel;
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
	OriginalPosX.imax = 10;
	OriginalPosY.imax = 10;
	//速度积分归零
	OriginalVelX.integrator = 0;
	OriginalVelY.integrator = 0;
	OriginalVelZ.integrator = 0;
	//位置速度积分最大
	OriginalVelX.imax = 10;
	OriginalVelY.imax = 10;
	//高度速度积分最大
	OriginalVelZ.imax = 10;
	//位置输出归零
	PosControllerOut.ExpectAcc = 0;
	PosControllerOut.ExpectAngle.pitch = 0;
	PosControllerOut.ExpectAngle.roll = 0;
	PosControllerOut.ExpectAngle.yaw = 0;
}


