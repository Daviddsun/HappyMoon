#include "Navigation.h"
NAVGATION_t nav;
KalmanVel_t kalmanVel;
FPS_Navigation FPSNavigation;
static void KalmanVelInit(void);
/**********************************************************************************************************
*函 数 名: NavigationInit
*功能说明: 导航参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationInit(void)
{
  KalmanVelInit();
}
/**********************************************************************************************************
*函 数 名: KalmanVelInit
*功能说明: 飞行速度估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void KalmanVelInit(void)
{
	OS_ERR err;
	float qMatInit[6][6] = {{0.2, 0, 0, 0, 0, 0},
													{0, 0.2, 0, 0, 0, 0},
													{0, 0, 0.03, 0, 0, 0},      
													{0.003, 0, 0, 0, 0, 0},
													{0, 0.003, 0, 0, 0, 0},
													{0, 0, 0.003, 0, 0, 0}};

	float rMatInit[6][6] = {{0.1, 0, 0, 0, 0, 0},          //VIO速度x轴数据噪声方差
													{0, 0.1, 0, 0, 0, 0},          //VIO速度y轴数据噪声方差
													{0, 0, 5.0, 0, 0, 0},          //VIO速度z轴数据噪声方差
													{0, 0, 0, 2500, 0, 0},         //气压速度数据噪声方差
													{0, 0, 0, 0, 10.0, 0},       	 //TOF速度数据噪声方差
													{0, 0, 0, 0, 0, 500000}};      //z轴速度高通滤波系数

	float pMatInit[6][6] = {{10, 0, 0, 0, 0, 0},
													{0, 10, 0, 0, 0, 0},
													{0, 0, 10, 0, 0, 0},      
													{5, 0, 0, 2, 0, 0},
													{0, 5, 0, 0, 2, 0},
													{0, 0, 2, 0, 0, 2}};    //增大协方差P的初值，可以提高初始化时bias的收敛速度

	float hMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
													{0, 1, 0, 0, 0, 0},
													{0, 0, 1, 0, 0, 0},      
													{0, 0, 1, 0, 0, 0},
													{0, 0, 1, 0, 0, 0},
													{0, 0, 1, 0, 0, 0}};    //h[5][2]:速度z轴增加少许高通滤波效果

	float fMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
													{0, 1, 0, 0, 0, 0},
													{0, 0, 1, 0, 0, 0},      
													{0, 0, 0, 1, 0, 0},
													{0, 0, 0, 0, 1, 0},
													{0, 0, 0, 0, 0, 1}};
	
	float bMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
													{0, 1, 0, 0, 0, 0},
													{0, 0, 1, 0, 0, 0},      
													{0, 0, 0, 0, 0, 0},
													{0, 0, 0, 0, 0, 0},
													{0, 0, 0, 0, 0, 0}};
    
	//初始化卡尔曼滤波器的相关矩阵
	KalmanVelQMatSet(&kalmanVel, qMatInit);
	KalmanVelRMatSet(&kalmanVel, rMatInit);
	KalmanVelCovarianceMatSet(&kalmanVel, pMatInit);
	KalmanVelObserveMapMatSet(&kalmanVel, hMatInit);
	KalmanVelStateTransMatSet(&kalmanVel, fMatInit);
	KalmanVelBMatSet(&kalmanVel, bMatInit);
													
	//状态滑动窗口，用于解决卡尔曼状态估计量与观测量之间的相位差问题
	kalmanVel.slidWindowSize = 50;
	kalmanVel.stateSlidWindow = OSMemGet(&memoryInfo[KALMAN_VEL],&err);												
	kalmanVel.fuseDelay[VIO_VEL_X] = 50;    //VIO速度x轴数据延迟参数：0.1s
	kalmanVel.fuseDelay[VIO_VEL_Y] = 50;    //VIO速度y轴数据延迟参数：0.1s
	kalmanVel.fuseDelay[VIO_VEL_Z] = 50;    //VIO速度z轴数据延迟参数：0.1s
	kalmanVel.fuseDelay[BARO_VEL]  = 50;    //气压速度数据延迟参数：0.02s
	kalmanVel.fuseDelay[TOF_VEL]   = 50;    //TOF速度数据延迟参数：0.02s
	
	kalmanVel.state[0] = 0;
  kalmanVel.state[1] = 0;
  kalmanVel.state[2] = 0;
	kalmanVel.state[3] = 0;
  kalmanVel.state[4] = 0;
  kalmanVel.state[5] = 0;
}
/**********************************************************************************************************
*函 数 名: VelocityEstimate
*功能说明: 飞行速度估计 融合加速度、VIO、气压计及TOF等多个传感器的数据
*          
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void VelocityEstimate(void){
	OS_ERR err;
	Vector3f_t VIOVel;
	static uint32_t count;
	static bool fuseFlag;
        
	//计算时间间隔，用于积分
	FPSNavigation.VelCurrentTime = (OSTimeGet(&err)  - FPSNavigation.VelLastTime) * 1e-3;
	FPSNavigation.VelCurrentTime = ConstrainFloat(FPSNavigation.VelCurrentTime, 0.0005, 0.005);
	FPSNavigation.VelLastTime = OSTimeGet(&err);

	//获取运动加速度
	nav.accel = EarthAccGetData();

	//加速度数据更新频率500 Hz
	if(count++ % 20 == 0){
		//获取视觉里程计数据
		VIOVel = GetVisualOdometryVelTrans();
		
		nav.velMeasure[0] = VIOVel.x;           				//机体速度x轴
		nav.velMeasure[1] = VIOVel.y;          					//机体速度y轴
		nav.velMeasure[2] = VIOVel.z;          					//机体速度z轴
		nav.velMeasure[3] = 0;  												//气压速度值
		nav.velMeasure[4] = GetTofHeightVel();          //TOF速度值
		nav.velMeasure[5] = 0;   
			
		//禁用VIO中Z轴的速度
		KalmanVelUseMeasurement(&kalmanVel, VIO_VEL_Z, false);
		//禁用气压传感器：未安装
		KalmanVelUseMeasurement(&kalmanVel, BARO_VEL, false);
		//已安装TOF传感器
		KalmanVelUseMeasurement(&kalmanVel, TOF_VEL, true);
		
		fuseFlag = true;
	}
	else
	{
		fuseFlag = false;
	}
	
	/*
	更新卡尔曼滤波器
	估计飞行速度及加速度bias
	*/
	KalmanVelUpdate(&kalmanVel, &nav.velocity, &nav.accel_bias, nav.accel, nav.velMeasure, FPSNavigation.VelCurrentTime, fuseFlag);
}

/**********************************************************************************************************
*函 数 名: GetCopterAccel
*功能说明: 获取飞行加速度
*形    参: 无
*返 回 值: 加速度值
**********************************************************************************************************/
Vector3f_t GetCopterAccel(void)
{
    return nav.accel;
}
/**********************************************************************************************************
*函 数 名: GetAccelBias
*功能说明: 获取加速度bias
*形    参: 无
*返 回 值: 加速度bias值
**********************************************************************************************************/
Vector3f_t GetAccelBias(void)
{
    return nav.accel_bias;
}
/**********************************************************************************************************
*函 数 名: GetCopterVelocity
*功能说明: 获取飞行速度估计值
*形    参: 无
*返 回 值: 速度值
**********************************************************************************************************/
Vector3f_t GetCopterVelocity(void)
{
    return nav.velocity;
}
/**********************************************************************************************************
*函 数 名: GetCopterVelMeasure
*功能说明: 获取飞行速度测量值
*形    参: 无
*返 回 值: 速度值
**********************************************************************************************************/
float* GetCopterVelMeasure(void)
{
    return nav.velMeasure;
}
/**********************************************************************************************************
*函 数 名: GetCopterPosition
*功能说明: 获取位置估计值
*形    参: 无
*返 回 值: 位置值
**********************************************************************************************************/
Vector3f_t GetCopterPosition(void)
{
    return nav.position;
}
/**********************************************************************************************************
*函 数 名: GetCopterPosMeasure
*功能说明: 获取位置测量值
*形    参: 无
*返 回 值: 速度值
**********************************************************************************************************/
Vector3f_t GetCopterPosMeasure(void)
{
    return nav.posMeasure;
}
/**********************************************************************************************************
*函 数 名: GetFPSNavigationPos
*功能说明: 获取位置融合FPS
*形    参: 无
*返 回 值: 
**********************************************************************************************************/
float GetFPSNavigationPos(void)
{
    return FPSNavigation.PosCurrentTime;
}
/**********************************************************************************************************
*函 数 名: GetFPSNavigationVel
*功能说明: 获取位置融合FPS
*形    参: 无
*返 回 值: 
**********************************************************************************************************/
float GetFPSNavigationVel(void)
{
    return FPSNavigation.VelCurrentTime;
}

