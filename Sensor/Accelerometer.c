/**********************************************************************************************************
 * @文件     Accelerometer.c
 * @说明     加速度数据预处理
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
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
*函 数 名: ImuLevelCalibration
*功能说明: IMU传感器的水平校准（安装误差），主要读取静止时的加速度数据并求平均值，得到校准角度值
*形    参: 无
*返 回 值: 水平安装误差
**********************************************************************************************************/
void ImuLevelCalibration(void){
	const int16_t CALIBRATING_ACC_LEVEL_CYCLES = 3000;
	static float acc_sum[3] = {0, 0, 0};
	Vector3f_t accAverage;
	Vector3f_t caliTemp;
	static int16_t count = 0;
	
	if(!OffsetData.level_success)
			return;
	if(count == 0){
		OffsetData.level_scale.x = 0;
		OffsetData.level_scale.y = 0;
		OffsetData.level_scale.z = 0;
	}else{
		acc_sum[0] += accValue.data.x;
		acc_sum[1] += accValue.data.y;
		acc_sum[2] += accValue.data.z;
	}
	count ++;
	if(count == CALIBRATING_ACC_LEVEL_CYCLES){
		accAverage.x = acc_sum[0] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		accAverage.y = acc_sum[1] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		accAverage.z = acc_sum[2] / (CALIBRATING_ACC_LEVEL_CYCLES-1);
		acc_sum[0] = 0;
		acc_sum[1] = 0;
		acc_sum[2] = 0;
		count = 0;
		//加速度向量转化为姿态角
    AccVectorToRollPitchAngle(&caliTemp, accAverage);
		if(abs(caliTemp.x) < 0.1 && abs(caliTemp.y) < 0.1){
			OffsetData.level_scale.x = -caliTemp.x;
			OffsetData.level_scale.y = -caliTemp.y;
			OffsetData.level_scale.z = 0;
			Write_Config();
			OffsetData.level_success = false;
		}
		else{
			OffsetData.level_success = false;
		}
	}
}

/**********************************************************************************************************
*函 数 名: AccDataPreTreat
*功能说明: 加速度数据预处理
*形    参: 加速度原始数据 加速度预处理数据指针
*返 回 值: 无
**********************************************************************************************************/
void AccDataPreTreat(Vector3f_t accRaw, Vector3f_t* accData, Vector3f_t AccLevelError){
	Vector3f_t accdata = accRaw;
	//加速度数据校准
	accdata.x = (accdata.x - OffsetData.acc_offectx) * OffsetData.acc_scalex;
	accdata.y = (accdata.y - OffsetData.acc_offecty) * OffsetData.acc_scaley;
	accdata.z = (accdata.z - OffsetData.acc_offectz) * OffsetData.acc_scalez;
	
	//机械误差校准
	accdata = VectorRotateToBodyFrame(accdata, AccLevelError);
	
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
*函 数 名: BodyFrameToEarthFrame
*功能说明: 转换向量到地理坐标系
*形    参: 转动角度 转动向量 转动后的向量指针
*返 回 值: 无
**********************************************************************************************************/
void BodyFrameToEarthFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorEf)
{
    Vector3f_t anglerad;

    anglerad.x = angle.x;
    anglerad.y = angle.y;
    anglerad.z = 0;
    *vectorEf  = VectorRotateToEarthFrame(vector, anglerad);
}

/**********************************************************************************************************
*函 数 名: EarthFrameToBodyFrame
*功能说明: 转换向量到机体坐标系
*形    参: 转动角度 转动向量 转动后的向量指针
*返 回 值: 无
**********************************************************************************************************/
void EarthFrameToBodyFrame(Vector3f_t angle, Vector3f_t vector, Vector3f_t* vectorBf)
{
    Vector3f_t anglerad;

    anglerad.x = angle.x;
    anglerad.y = angle.y;
    anglerad.z = 0;
    *vectorBf  = VectorRotateToBodyFrame(vector, anglerad);
}

/**********************************************************************************************************
*函 数 名: TransAccToEarthFrame
*功能说明: 转换加速度到地理坐标系，并去除重力加速度
*形    参: 当前飞机姿态 加速度 地理系加速度 地理系加速度低通滤波 加速度零偏
*返 回 值: 无
**********************************************************************************************************/
void TransAccToEarthFrame(Vector3f_t angle, Vector3f_t acc, Vector3f_t* accEf, Vector3f_t* accEfLpf, Vector3f_t* accBfOffset){
	
	Vector3f_t gravityBf;

	//计算重力加速度在机体坐标系的投影
	gravityBf.x = 0;
	gravityBf.y = 0;
	gravityBf.z = 1;
	EarthFrameToBodyFrame(angle, gravityBf, &gravityBf);

	//减去重力加速度和加速度零偏
	acc.x -= (gravityBf.x + accBfOffset->x);
	acc.y -= (gravityBf.y + accBfOffset->y);
	acc.z -= (gravityBf.z + accBfOffset->z);

	//转化加速度到地理坐标系
	BodyFrameToEarthFrame(angle, acc, accEf);

	//地理系加速度低通滤波
	accEfLpf->x = accEfLpf->x * 0.998f + accEf->x * 0.002f;
	accEfLpf->y = accEfLpf->y * 0.998f + accEf->y * 0.002f;
	accEfLpf->z = accEfLpf->z * 0.998f + accEf->z * 0.002f;

}


/**********************************************************************************************************
*函 数 名: GetCopterAccEf
*功能说明: 获取地理坐标系下的运动加速度
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t GetCopterAccEf(void){
	Vector3f_t AccEfValue,AccEfLpfValue,AngleValue;
	AngleValue.x = GetCopterAngle().roll;
	AngleValue.y = GetCopterAngle().pitch;
	AngleValue.z = 0;
	TransAccToEarthFrame(AngleValue,accValue.data,&AccEfValue,&AccEfLpfValue,0);
	return AccEfValue;
}


/**********************************************************************************************************
*函 数 名: EarthAccGetData
*功能说明: 获取经过处理后的加速度数据
*形    参: 无
*返 回 值: 加速度
**********************************************************************************************************/
Vector3f_t EarthAccGetData(void){
	Vector3f_t EarthAcc,CopterAngle;
	//获取自身角度
	CopterAngle.x = GetCopterAngle().roll;
	CopterAngle.y = GetCopterAngle().pitch;
	CopterAngle.z = 0;
	EarthAcc = VectorRotateToBodyFrame(accValue.data,CopterAngle);
	EarthAcc.z = EarthAcc.z - 1.0f;
  return EarthAcc;
}

