#include "Navigation.h"
NAVGATION_t nav;
Kalman_t kalmanPos;
FPS_Navigation FPSNavigation;
static void KalmanPosInit(void);

/**********************************************************************************************************
*函 数 名: NavigationInit
*功能说明: 导航参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationInit(void)
{
    KalmanPosInit();
}

/**********************************************************************************************************
*函 数 名: KalmanPosInit
*功能说明: 位置估计的卡尔曼结构体初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void KalmanPosInit(void)
{
    float qMatInit[9] = {0.003, 0, 0, 0, 0.003, 0, 0, 0, 0.03};
    float rMatInit[9] = {500, 0,  0, 0, 500, 0, 0, 0, 1000};
    float pMatInit[9] = {3, 0, 0, 0, 3, 0, 0, 0, 5};
    float fMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float hMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float bMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    //初始化卡尔曼滤波器的相关矩阵
    KalmanQMatSet(&kalmanPos, qMatInit);
    KalmanRMatSet(&kalmanPos, rMatInit);
    KalmanBMatSet(&kalmanPos, bMatInit);
    KalmanCovarianceMatSet(&kalmanPos, pMatInit);
    KalmanStateTransMatSet(&kalmanPos, fMatInit);
    KalmanObserveMapMatSet(&kalmanPos, hMatInit);
    //状态滑动窗口，用于解决卡尔曼状态估计量与观测量之间的相位差问题
		
		kalmanPos.state.x = 0;
		kalmanPos.state.y = 0;
		kalmanPos.state.z = 0;
}

/**********************************************************************************************************
*函 数 名: PositionEstimate
*功能说明: 
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PositionEstimate(void)
{
	OS_ERR err;
	Vector3f_t input;
	static uint32_t count;
	static bool fuseFlag;

	//计算时间间隔，用于积分
	FPSNavigation.PosCurrentTime = (OSTimeGet(&err) - FPSNavigation.PosLastTime) * 1e-3;
	FPSNavigation.PosCurrentTime = ConstrainFloat(FPSNavigation.PosCurrentTime, 0.0005, 0.002);
	FPSNavigation.PosLastTime = OSTimeGet(&err);

	//速度数据更新频率1KHz，VIO数据只有10hz
	//这里将VIO更新频率拉到20hz
	if(count++ % 50 == 0)
	{
		//获取VIO位置
		nav.posMeasure = GetVisualOdometryPos();

		fuseFlag = true;
	}
	else
	{
		fuseFlag = false;
	}

	//速度积分
	input.x = 0 * FPSNavigation.PosCurrentTime;
	input.y = 0 * FPSNavigation.PosCurrentTime;
	input.z = 0 * FPSNavigation.PosCurrentTime;

	//位置估计
	KalmanUpdate(&kalmanPos, input, nav.posMeasure, fuseFlag);
	nav.position = kalmanPos.state;
}

///**********************************************************************************************************
//*函 数 名: NavigationReset
//*功能说明: 导航相关数据复位
//*形    参: 无
//*返 回 值: 无
//**********************************************************************************************************/
//void NavigationReset(void)
//{
////    kalmanVel.state[0] = 0;
////    kalmanVel.state[1] = 0;
////    kalmanVel.state[2] = 0;

//}
///**********************************************************************************************************
//*函 数 名: GetCopterAccel
//*功能说明: 获取飞行加速度
//*形    参: 无
//*返 回 值: 加速度值
//**********************************************************************************************************/
//Vector3f_t GetCopterAccel(void)
//{
//    return nav.accel;
//}
///**********************************************************************************************************
//*函 数 名: GetAccelBias
//*功能说明: 获取加速度bias
//*形    参: 无
//*返 回 值: 加速度bias值
//**********************************************************************************************************/
//Vector3f_t GetAccelBias(void)
//{
//    return nav.accel_bias;
//}
///**********************************************************************************************************
//*函 数 名: GetCopterVelocity
//*功能说明: 获取飞行速度估计值
//*形    参: 无
//*返 回 值: 速度值
//**********************************************************************************************************/
//Vector3f_t GetCopterVelocity(void)
//{
//    return nav.velocity;
//}
///**********************************************************************************************************
//*函 数 名: GetCopterVelMeasure
//*功能说明: 获取飞行速度测量值
//*形    参: 无
//*返 回 值: 速度值
//**********************************************************************************************************/
//float* GetCopterVelMeasure(void)
//{
//    return nav.velMeasure;
//}
///**********************************************************************************************************
//*函 数 名: GetCopterPosition
//*功能说明: 获取位置估计值
//*形    参: 无
//*返 回 值: 位置值
//**********************************************************************************************************/
//Vector3f_t GetCopterPosition(void)
//{
//    return nav.position;
//}
///**********************************************************************************************************
//*函 数 名: GetCopterPosMeasure
//*功能说明: 获取位置测量值
//*形    参: 无
//*返 回 值: 速度值
//**********************************************************************************************************/
//Vector3f_t GetCopterPosMeasure(void)
//{
//    return nav.posMeasure;
//}
///**********************************************************************************************************
//*函 数 名: GetFPSNavigationPos
//*功能说明: 获取位置融合FPS
//*形    参: 无
//*返 回 值: 
//**********************************************************************************************************/
//float GetFPSNavigationPos(void)
//{
//    return FPSNavigation.PosCurrentTime;
//}
///**********************************************************************************************************
//*函 数 名: GetFPSNavigationVel
//*功能说明: 获取位置融合FPS
//*形    参: 无
//*返 回 值: 
//**********************************************************************************************************/
//float GetFPSNavigationVel(void)
//{
//    return FPSNavigation.VelCurrentTime;
//}

