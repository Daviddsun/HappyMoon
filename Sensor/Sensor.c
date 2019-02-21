/**********************************************************************************************************
 * @文件     Sensor.c
 * @说明     传感器读取函数
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "Sensor.h"

enum ORIENTATION_STATUS orientationStatus;
/**********************************************************************************************************
*函 数 名: Sensor_Init
*功能说明: 各个传感器初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Sensor_Init(void){
	//陀螺仪加速计初始化
#ifdef SpeedyBeeF4
	MPU6000_Initialize();
#else
	MPU6500_Initialize();
#endif
	delay_ms(100);
}


/**********************************************************************************************************
*函 数 名: ImuOrientationDetect
*功能说明: 检测传感器放置方向
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ImuOrientationDetect(Vector3f_t acc){
    const float CONSTANTS_ONE_G = 1.0;
    const float accel_err_thr = 0.5;

    // [ g, 0, 0 ]
    if (fabsf(acc.x - CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_FRONT;
    }
    // [ -g, 0, 0 ]
    if (fabsf(acc.x + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_BACK;
    }
    // [ 0, g, 0 ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y - CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_LEFT;
    }
    // [ 0, -g, 0 ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_RIGHT;
    }
    // [ 0, 0, g ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z - CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_UP;
    }
    // [ 0, 0, -g ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z + CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_DOWN;
    }
}

/**********************************************************************************************************
*函 数 名: GetImuOrientation
*功能说明: 获取传感器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
enum ORIENTATION_STATUS GetImuOrientation(void)
{
    return orientationStatus;
}


