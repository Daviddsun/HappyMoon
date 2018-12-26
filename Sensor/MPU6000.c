#include "MPU6000.h"

float MPU6000Gx_offset,MPU6000Gy_offset,MPU6000Gz_offset;
enum ORIENTATION_STATUS orientationStatus;
uint8_t placement;

/***********************************************************************************
*函 数 名: MPU6000_writeReg
*功能说明: spi读写MPU6000
*形    参: 
*返 回 值: 状态
************************************************************************************/
void MPU6000_writeReg(u8 reg, u8 data)
{
	MPU6000_CSL();
	SPI1_ReadWrite_Byte(reg);
	SPI1_ReadWrite_Byte(data);
	MPU6000_CSH();	
}
/***********************************************************************************
*函 数 名: MPU6000_Initialize
*功能说明: 初始化MPU6000
*形    参: 
*返 回 值: 状态
************************************************************************************/
void MPU6000_Initialize(void)
{ 
	
	MPU6000_writeReg(MPU_RA_PWR_MGMT_1, 0x80);
	delay_ms(150);

	MPU6000_writeReg(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	delay_ms(150);

	MPU6000_writeReg(MPU_RA_PWR_MGMT_1, 0x00);
	delay_ms(15);

	MPU6000_writeReg(MPU_RA_USER_CTRL, 0x10);
	delay_ms(15);

	MPU6000_writeReg(MPU_RA_PWR_MGMT_2, 0x00);
	delay_ms(15);

	//陀螺仪采样率0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
	MPU6000_writeReg(MPU_RA_SMPLRT_DIV, (1000/1000 - 1));
	delay_ms(15);

	//i2c旁路模式
	// INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
	MPU6000_writeReg(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
	delay_ms(15);

	//低通滤波频率
	MPU6000_writeReg(MPU_RA_CONFIG, MPU_LPF_42HZ);
	delay_ms(15);

	//陀螺仪自检及测量范围，典型值0x18(不自检，2000deg/s) (0x10 1000deg/s) (0x10 1000deg/s) (0x08 500deg/s)
	MPU6000_writeReg(MPU_RA_GYRO_CONFIG, 0x10);
	delay_ms(15);

	//加速度自检、测量范围(不自检，+-8G)
	MPU6000_writeReg(MPU_RA_ACCEL_CONFIG, 2 << 3);

	delay_ms(5);


}
/**********************************************************************************************************
*函 数 名: MPU6000_ReadAcc
*功能说明: MPU6000读取加速度传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6000_ReadAcc(Vector3f_t* acc)
{
    uint8_t buffer[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
    Vector3i_t accRaw;
//		MPU6000_CSL();
//		SPI1_readRegs(MPU_RA_ACCEL_XOUT_H,6,buffer);
//		MPU6000_CSH();
    accRaw.x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    accRaw.y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
    accRaw.z = ((((int16_t)buffer[4]) << 8) | buffer[5]);

    acc->x = (float)accRaw.x * MPU_A_8mg;
    acc->y = (float)accRaw.y * MPU_A_8mg;
    acc->z = (float)accRaw.z * MPU_A_8mg;

}

/**********************************************************************************************************
*函 数 名: MPU6000_ReadGyro
*功能说明: MPU6000读取陀螺仪传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6000_ReadGyro(Vector3f_t* gyro)
{
    uint8_t buffer[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
    Vector3i_t gyroRaw;
//		MPU6000_CSL();
//		SPI1_readRegs(MPU_RA_GYRO_XOUT_H,6,buffer);
//		MPU6000_CSH();
    gyroRaw.x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    gyroRaw.y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
    gyroRaw.z = ((((int16_t)buffer[4]) << 8) | buffer[5]);

    gyro->x = gyroRaw.x * MPU_G_s1000dps;
    gyro->y = gyroRaw.y * MPU_G_s1000dps;
    gyro->z = gyroRaw.z * MPU_G_s1000dps;
}

/**********************************************************************************************************
*函 数 名: MPU6000_ReadTemp
*功能说明: MPU6000读取温度传感器
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6000_ReadTemp(float* temp)
{
    uint8_t buffer[2] = {0x07,0x08};
    static int16_t temperature_temp;
//		MPU6000_CSL();
//		SPI1_readRegs(MPU_RA_TEMP_OUT_H,2,buffer);
//		MPU6000_CSH();
    temperature_temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    *temp = 36.53f + (float)temperature_temp / 340;
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

/**********************************************************************************************************
*函 数 名: ImuOrientationDetect
*功能说明: 检测传感器放置方向
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ImuOrientationDetect(float accx,float accy,float accz){
    const float CONSTANTS_ONE_G = 1.0;
    const float accel_err_thr = 0.5;

    // [ g, 0, 0 ]
    if (fabsf(accx - CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(accy) < accel_err_thr &&
            fabsf(accz) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_FRONT;
    }
    // [ -g, 0, 0 ]
    if (fabsf(accx + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(accy) < accel_err_thr &&
            fabsf(accz) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_BACK;
    }
    // [ 0, g, 0 ]
    if (fabsf(accx) < accel_err_thr &&
            fabsf(accy - CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(accz) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_LEFT;
    }
    // [ 0, -g, 0 ]
    if (fabsf(accx) < accel_err_thr &&
            fabsf(accy + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(accz) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_RIGHT;
    }
    // [ 0, 0, g ]
    if (fabsf(accx) < accel_err_thr &&
            fabsf(accy) < accel_err_thr &&
            fabsf(accz - CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_UP;
    }
    // [ 0, 0, -g ]
    if (fabsf(accx) < accel_err_thr &&
            fabsf(accy) < accel_err_thr &&
            fabsf(accz + CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_DOWN;
    }
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

/****************************************************************************************
*函 数 名: AccCalibration
*功能说明: 校准加速计数据
*形    参: 无
*返 回 值: 无
*****************************************************************************************/
void AccCalibration(Vector3f_t accRaw){
	static uint16_t samples_count = 0;
	static uint8_t orientationCaliFlag[6];
	static Vector3f_t new_offset;
	static Vector3f_t new_scale;
	static Vector3f_t samples[6];
	static uint8_t caliFlag = 0;
	static uint32_t caliCnt = 0;

	if(!OffsetData.acc_success)
			return;

	/*********************************检测IMU放置方向************************************/
	if(GetImuOrientation() == ORIENTATION_UP && !orientationCaliFlag[ORIENTATION_UP])
	{
			//判断IMU是否处于静止状态
			if(GetPlaceStatus() == STATIC)
					caliCnt++;
			else
					caliCnt = 0;

			if(caliCnt > 1000)
			{
					caliFlag = 1;
					orientationCaliFlag[ORIENTATION_UP] = 1;
					samples_count = 0;
					OffsetData.acc_calibra_cnt ++;
			}
	}	
	if(GetImuOrientation() == ORIENTATION_DOWN && !orientationCaliFlag[ORIENTATION_DOWN])
	{
			//判断IMU是否处于静止状态
			if(GetPlaceStatus() == STATIC)
					caliCnt++;
			else
					caliCnt = 0;

			if(caliCnt > 1000)
			{
					caliFlag = 1;
					orientationCaliFlag[ORIENTATION_DOWN] = 1;
					samples_count = 0;
					OffsetData.acc_calibra_cnt++;
			}
	}

	if(GetImuOrientation() == ORIENTATION_FRONT && !orientationCaliFlag[ORIENTATION_FRONT])
	{
			//判断IMU是否处于静止状态
			if(GetPlaceStatus() == STATIC)
					caliCnt++;
			else
					caliCnt = 0;

			if(caliCnt > 1000)
			{
					caliFlag = 1;
					orientationCaliFlag[ORIENTATION_FRONT] = 1;
					samples_count = 0;
					OffsetData.acc_calibra_cnt++;
			}
	}

	if(GetImuOrientation() == ORIENTATION_BACK && !orientationCaliFlag[ORIENTATION_BACK])
	{
			//判断IMU是否处于静止状态
			if(GetPlaceStatus() == STATIC)
					caliCnt++;
			else
					caliCnt = 0;

			if(caliCnt > 1000)
			{
					caliFlag = 1;
					orientationCaliFlag[ORIENTATION_BACK] = 1;
					samples_count = 0;
					OffsetData.acc_calibra_cnt++;
			}
	}

	if(GetImuOrientation() == ORIENTATION_LEFT && !orientationCaliFlag[ORIENTATION_LEFT])
	{
			//判断IMU是否处于静止状态
			if(GetPlaceStatus() == STATIC)
					caliCnt++;
			else
					caliCnt = 0;

			if(caliCnt > 1000)
			{
					caliFlag = 1;
					orientationCaliFlag[ORIENTATION_LEFT] = 1;
					samples_count = 0;
					OffsetData.acc_calibra_cnt++;
			}
	}

	if(GetImuOrientation() == ORIENTATION_RIGHT && !orientationCaliFlag[ORIENTATION_RIGHT])
	{
			//判断IMU是否处于静止状态
			if(GetPlaceStatus() == STATIC)
					caliCnt++;
			else
					caliCnt = 0;

			if(caliCnt > 1000)
			{
					caliFlag = 1;
					orientationCaliFlag[ORIENTATION_RIGHT] = 1;
					samples_count = 0;
					OffsetData.acc_calibra_cnt++;
			}
	}
	if(caliFlag)
	{
       if(samples_count < 500)
        {
            samples[OffsetData.acc_calibra_cnt - 1].x += accRaw.x;
            samples[OffsetData.acc_calibra_cnt - 1].y += accRaw.y;
            samples[OffsetData.acc_calibra_cnt - 1].z += accRaw.z;
            samples_count++;
        }
        else if(samples_count == 500)
        {
            samples[OffsetData.acc_calibra_cnt - 1].x /= 500;
            samples[OffsetData.acc_calibra_cnt - 1].y /= 500;
            samples[OffsetData.acc_calibra_cnt - 1].z /= 500;
            samples_count++;

            caliFlag = 0;
            caliCnt  = 0;
        }		
	}
	
	if(OffsetData.acc_calibra_cnt == 6 && samples_count == 501)
  {
		//计算方程解初值
		float initBeta[6];
		initBeta[0] = 0;
		initBeta[1] = 0;
		initBeta[2] = 0;
		initBeta[3] = 1;
		initBeta[4] = 1;
		initBeta[5] = 1;

		//LM法求解传感器误差方程最优解
		LevenbergMarquardt(samples, &new_offset, &new_scale, initBeta, 1);

		//判断校准参数是否正常  无论成功与否全部退出函数
		if(fabsf(new_scale.x-1.0f) > 0.1f || fabsf(new_scale.y-1.0f) > 0.1f || fabsf(new_scale.z-1.0f) > 0.1f)
		{
				OffsetData.acc_success = true;
		}
		else if(fabsf(new_offset.x) > (1 * 0.35f) || fabsf(new_offset.y) > (1 * 0.35f) || fabsf(new_offset.z) > (1 * 0.6f))
		{
				OffsetData.acc_success = true;
		}
		else
		{
			for(u8 i=0; i<6; i++)
			{
					samples[i].x = 0;
					samples[i].y = 0;
					samples[i].z = 0;
			}
			OffsetData.acc_offectx = new_offset.x;
			OffsetData.acc_offecty = new_offset.y;
			OffsetData.acc_offectz = new_offset.z;
			OffsetData.acc_scalex = new_scale.x;
			OffsetData.acc_scaley = new_scale.y;
			OffsetData.acc_scalez = new_scale.z;
			Write_config();
			OffsetData.acc_calibra_cnt = 0;
			for(uint8_t i=0; i<6; i++)
					orientationCaliFlag[i] = 0;
			OffsetData.acc_success = true;
		}

   }
}

/**********************************************************************************************************
*函 数 名: AccDataPreTreat
*功能说明: 加速度数据预处理
*形    参: 加速度原始数据 加速度预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData)
{
	Vector3f_t accdata = accRaw;

	//加速度数据校准
	accdata.x = (accdata.x - OffsetData.acc_offectx) * OffsetData.acc_scalex;
	accdata.y = (accdata.y - OffsetData.acc_offecty) * OffsetData.acc_scaley;
	accdata.z = (accdata.z - OffsetData.acc_offectz) * OffsetData.acc_scalez;

	*accData = accdata;
} 

/**********************************************************************************************************
*函 数 名: GyroCalibration
*功能说明: 陀螺仪校准
*形    参: 陀螺仪原始数据
*返 回 值: 无
**********************************************************************************************************/
void GyroCalibration(Vector3f_t gyroRaw)
{
    static float gyro_sum[3] = {0, 0, 0};
    Vector3f_t gyro_raw_temp;
    static int16_t count = 0;
		
    if(!OffsetData.gyro_success)
        return;
		
    gyro_raw_temp = gyroRaw;

    gyro_sum[0] += gyro_raw_temp.x;
    gyro_sum[1] += gyro_raw_temp.y;
    gyro_sum[2] += gyro_raw_temp.z;
    count++;
}


/**********************************************************************************************************
*函 数 名: GyroDataPreTreat
*功能说明: 陀螺仪数据预处理
*形    参: 陀螺仪原始数据 陀螺仪预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void GyroDataPreTreat(Vector3f_t gyroRaw, Vector3f_t* gyroData, Vector3f_t* gyroLpfData){
	
    Vector3f_t gyrodata = gyroRaw;

    //零偏误差校准
    gyrodata.x = (gyrodata.x - OffsetData.gyro_offectx) * OffsetData.gyro_scalex;
    gyrodata.y = (gyrodata.y - OffsetData.gyro_offecty) * OffsetData.gyro_scaley;
    gyrodata.z = (gyrodata.z - OffsetData.gyro_offectz) * OffsetData.gyro_scalez;

    *gyroData = gyrodata;
		*gyroLpfData = gyrodata;
}

