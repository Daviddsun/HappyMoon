#include "Accelerometer.h"

ACCELEROMETER_t accValue;

/**********************************************************************************************************
*函 数 名: AccPreTreatInit
*功能说明: 加速度预处理初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AccPreTreatInit(void){
    //加速度低通滤波系数计算
    LowPassFilter2ndFactorCal(0.001, ACC_LPF_CUT, &accValue.lpf_2nd);
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
				OffsetData.acc_success = false;
		}
		else if(fabsf(new_offset.x) > (1 * 0.35f) || fabsf(new_offset.y) > (1 * 0.35f) || fabsf(new_offset.z) > (1 * 0.6f))
		{
				OffsetData.acc_success = false;
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
			Write_Config();
			OffsetData.acc_calibra_cnt = 0;
			for(uint8_t i=0; i<6; i++)
					orientationCaliFlag[i] = 0;
			OffsetData.acc_success = false;
		}

   }
}


/**********************************************************************************************************
*函 数 名: AccDataPreTreat
*功能说明: 加速度数据预处理
*形    参: 加速度原始数据 加速度预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData){
	Vector3f_t accdata = accRaw;
	//加速度数据校准
	accdata.x = (accdata.x - OffsetData.acc_offectx) * OffsetData.acc_scalex;
	accdata.y = (accdata.y - OffsetData.acc_offecty) * OffsetData.acc_scaley;
	accdata.z = (accdata.z - OffsetData.acc_offectz) * OffsetData.acc_scalez;
	accValue.data = accdata;
	*accData = accdata;
} 

/**********************************************************************************************************
*函 数 名: AccGetData
*功能说明: 获取经过处理后的加速度数据
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t AccGetData(void){
    return accValue.data;
}

/**********************************************************************************************************
*函 数 名: EarthAccGetData
*功能说明: 获取经过处理后的加速度数据
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t EarthAccGetData(void){
	Vector3f_t EarthAcc;
	Vector4q_t quaternion = GetCopterQuaternion();
	EarthAcc.x = (quaternion.qw*quaternion.qw + quaternion.qx*quaternion.qx - quaternion.qy*quaternion.qy - quaternion.qz*quaternion.qz)*accValue.data.x 
					+ (2.f * (quaternion.qx*quaternion.qy - quaternion.qw*quaternion.qz))*accValue.data.y 
								+ (2.f * (quaternion.qx*quaternion.qz + quaternion.qw*quaternion.qy))*accValue.data.z;
	
	EarthAcc.y = (2.f * (quaternion.qx*quaternion.qy + quaternion.qw*quaternion.qz))*accValue.data.x
					+ (quaternion.qw*quaternion.qw - quaternion.qx*quaternion.qx + quaternion.qy*quaternion.qy - quaternion.qz*quaternion.qz)*accValue.data.y 
								+ (2.f * (quaternion.qy*quaternion.qz - quaternion.qw*quaternion.qx))*accValue.data.z;
	
	EarthAcc.z = ((2.f * (quaternion.qx*quaternion.qz - quaternion.qw*quaternion.qy))*accValue.data.x
					+ (2.f * (quaternion.qy*quaternion.qz + quaternion.qw*quaternion.qx))*accValue.data.y 
								+ (quaternion.qw*quaternion.qw - quaternion.qx*quaternion.qx - quaternion.qy*quaternion.qy + quaternion.qz*quaternion.qz)*accValue.data.z) - 1.0f;
  return EarthAcc;
}

