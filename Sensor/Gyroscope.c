#include "Gyroscope.h"

GYROSCOPE_t gyroValue;
uint8_t placement;
/**********************************************************************************************************
*函 数 名: GyroPreTreatInit
*功能说明: 陀螺仪预处理初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GyroPreTreatInit(void)
{
    //陀螺仪低通滤波系数计算
    LowPassFilter2ndFactorCal(0.001, GYRO_LPF_CUT, &gyroValue.lpf_2nd);
}

/**********************************************************************************************************
*函 数 名: GyroCalibration
*功能说明: 陀螺仪校准
*形    参: 陀螺仪原始数据
*返 回 值: 无
**********************************************************************************************************/
void GyroCalibration(Vector3f_t gyroRaw)
{
	const int16_t CALIBRATING_GYRO_CYCLES = 1000;
	static float gyro_sum[3] = {0, 0, 0};
	Vector3f_t gyro_cali_temp, gyro_raw_temp;
	static int16_t count = 0;

	if(!OffsetData.gyro_success)
      return;

  gyro_raw_temp = gyroRaw;

	gyro_sum[0] += gyro_raw_temp.x;
	gyro_sum[1] += gyro_raw_temp.y;
	gyro_sum[2] += gyro_raw_temp.z;
	count++;
	if(count == CALIBRATING_GYRO_CYCLES){
		gyro_cali_temp.x = gyro_sum[0] / CALIBRATING_GYRO_CYCLES;
		gyro_cali_temp.y = gyro_sum[1] / CALIBRATING_GYRO_CYCLES;
		gyro_cali_temp.z = gyro_sum[2] / CALIBRATING_GYRO_CYCLES;
		gyro_sum[0] = 0;
		gyro_sum[1] = 0;
		gyro_sum[2] = 0;
		
		OffsetData.gyro_offectx = gyro_cali_temp.x;
		OffsetData.gyro_offecty = gyro_cali_temp.y;
		OffsetData.gyro_offectz = gyro_cali_temp.z;
		OffsetData.gyro_scalex = 1.0;
		OffsetData.gyro_scaley = 1.0;
		OffsetData.gyro_scalez = 1.0;
		Write_Config();
		OffsetData.gyro_success = false;
	}
}

/**********************************************************************************************************
*函 数 名: GyroDataPreTreat
*功能说明: 陀螺仪数据预处理
*形    参: 陀螺仪原始数据 陀螺仪预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData, Vector3f_t* gyroLpfData, Vector3f_t GyroLevelError){
	
    Vector3f_t gyrodata = gyroRaw;

    //零偏误差校准
    gyrodata.x = (gyrodata.x - OffsetData.gyro_offectx) * OffsetData.gyro_scalex;
    gyrodata.y = (gyrodata.y - OffsetData.gyro_offecty) * OffsetData.gyro_scaley;
    gyrodata.z = (gyrodata.z - OffsetData.gyro_offectz) * OffsetData.gyro_scalez;
		//安装误差校准
    gyrodata = VectorRotateToBodyFrame(gyrodata, GyroLevelError);
		//低通滤波
		gyroValue.dataLpf = LowPassFilter2nd(&gyroValue.lpf_2nd, gyrodata);
    *gyroData = gyrodata;
		*gyroLpfData = gyroValue.dataLpf;
}
/***********************************************************************************
*函 数 名: PlaceStausCheck
*功能说明: 飞行器放置状态检测：静止或运动
*形    参: 角速度
*返 回 值: 无
*************************************************************************************/
void PlaceStausCheck(Vector3f_t gyro)
{
    Vector3f_t gyroDiff;
    static Vector3f_t lastGyro;
    static float threshold = 0.1f;
    static uint16_t checkNum = 0;
    static int16_t count = 0;

    gyroDiff.x = gyro.x - lastGyro.x;
    gyroDiff.y = gyro.y - lastGyro.y;
    gyroDiff.z = gyro.z - lastGyro.z;
    lastGyro = gyro;

    if(count < 30)
    {
        count++;
        //陀螺仪数值变化大于阈值
        if(abs(gyroDiff.x) > threshold || abs(gyroDiff.y) > threshold || abs(gyroDiff.z) > threshold)
        {
            checkNum++;
        }
    }
    else
    {
        //陀螺仪数据抖动次数大于一定值时认为飞机不处于静止状态
        if(checkNum > 10)
            placement = MOTIONAL;
        else
            placement = STATIC;

        checkNum = 0;
        count = 0;
    }
}


/**********************************************************************************************************
*函 数 名: GyroLpfGetData
*功能说明: 获取低通滤波后的陀螺仪数据
*形    参: 无
*返 回 值: 角速度
**********************************************************************************************************/
Vector3f_t GyroLpfGetData(void)
{
    return gyroValue.dataLpf;
}

/**********************************************************************************************************
*函 数 名: GetPlaceStatus
*功能说明: 获取飞行器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetPlaceStatus(void)
{
    return placement;
}


