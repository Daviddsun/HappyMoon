#include "MPU6000.h"

#define mpu6000Buf_SIZE  10	  

int16_t  MPU6000_FIFO[7][mpu6000Buf_SIZE];
uint8_t MPU6000Wr_Index = 0;
float MPU6000Gx_offset,MPU6000Gy_offset,MPU6000Gz_offset;
enum ORIENTATION_STATUS orientationStatus;
uint8_t placement;
u8 testdata[2];

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

void MPU6000_NewVal(int16_t* buf,int16_t val) {
  	buf[MPU6000Wr_Index] = val;
}

int16_t MPU6000_GetAvg(int16_t* buf)
{
  int i;
	int32_t	sum = 0;
	for(i=0;i<mpu6000Buf_SIZE;i++)
		sum += buf[i];
	sum = sum / mpu6000Buf_SIZE;
	return (int16_t)sum;
}

void MPU6000_writeReg(u8 reg, u8 data)
{
	MPU6000_CSL();
	SPI1_ReadWrite_Byte(reg);
	SPI1_ReadWrite_Byte(data);
	MPU6000_CSH();	
}

void MPU6000_initialize(void)
{ 

	MPU6000_writeReg(MPU_RA_PWR_MGMT_1,BIT_H_RESET); 			
	delay_ms(150);
	MPU6000_writeReg(MPU_RA_SIGNAL_PATH_RESET,BIT_GYRO | BIT_ACC | BIT_TEMP);
	delay_ms(150);
	
	MPU6000_writeReg(MPU_RA_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ); 		
	delay_ms(15);
	//Disable i2c 
	MPU6000_writeReg(MPU_RA_USER_CTRL,BIT_I2C_IF_DIS); 		
	delay_ms(15);
	MPU6000_writeReg(MPU_RA_PWR_MGMT_2,0x00); 		
	delay_ms(15);
	MPU6000_writeReg(MPU_RA_SMPLRT_DIV,0x00);			
	delay_ms(15);
	//DLPF enable
	MPU6000_writeReg(MPU_RA_CONFIG,0x02);
	delay_ms(15);
	//Gyro +/- 2000 DPS Full Scale
	MPU6000_writeReg(MPU_RA_GYRO_CONFIG,INV_FSR_2000DPS << 3);  					
	delay_ms(15);
	//Accel +/- 16 G Full Scale
	MPU6000_writeReg(MPU_RA_ACCEL_CONFIG,INV_FSR_16G << 3); 			
	delay_ms(15);
}

void MPU6000_readGyro_Acc(int16_t *gyro,int16_t *acc)
{
	static u8 buf[14];
	static int16_t gx,gy,gz;
	static int16_t ax,ay,az;
	MPU6000_CSL();
	SPI1_readRegs(MPU_RA_ACCEL_XOUT_H,14,buf);
	MPU6000_CSH();
	//acc
	MPU6000_NewVal(&MPU6000_FIFO[0][0],(int16_t)(((int16_t)buf[0]) << 8 | buf[1]));
	MPU6000_NewVal(&MPU6000_FIFO[1][0],(int16_t)(((int16_t)buf[2]) << 8 | buf[3]));
	MPU6000_NewVal(&MPU6000_FIFO[2][0],(int16_t)(((int16_t)buf[4]) << 8 | buf[5]));
	//temp
//	MPU6000_NewVal(&MPU6000_FIFO[3][0],(int16_t)(((int16_t)buf[6]) << 8 | buf[7]));
	//gyro
	MPU6000_NewVal(&MPU6000_FIFO[4][0],(int16_t)(((int16_t)buf[8]) << 8 | buf[9]));
	MPU6000_NewVal(&MPU6000_FIFO[5][0],(int16_t)(((int16_t)buf[10]) << 8 | buf[11]));
	MPU6000_NewVal(&MPU6000_FIFO[6][0],(int16_t)(((int16_t)buf[12]) << 8 | buf[13]));
	
	MPU6000Wr_Index = (MPU6000Wr_Index + 1) % mpu6000Buf_SIZE;

	gx =  MPU6000_GetAvg(&MPU6000_FIFO[4][0]);
	gy =  MPU6000_GetAvg(&MPU6000_FIFO[5][0]);
	gz =  MPU6000_GetAvg(&MPU6000_FIFO[6][0]);
	
	gyro[0] = gx - MPU6000Gx_offset;
	gyro[1] = gy - MPU6000Gy_offset;
	gyro[2] = gz - MPU6000Gz_offset;
			  
	ax = 	MPU6000_GetAvg(&MPU6000_FIFO[0][0]);
	ay = 	MPU6000_GetAvg(&MPU6000_FIFO[1][0]);
	az = 	MPU6000_GetAvg(&MPU6000_FIFO[2][0]);
				
	acc[0] = ax;
	acc[1] = ay;
	acc[2] = az;	
}

void IMU_getValues() 
{ 
	float values[6];
	int16_t accgyroval[6];
	int i;
	//获取加计和陀螺仪的数据
	MPU6000_readGyro_Acc(&accgyroval[3],&accgyroval[0]);
	for(i = 0; i<6; i++) 
	{
    if(i < 3) 
		{
  			values[i] = ((float) accgyroval[i]) / 2048.0f;// 32/65536 = 1/2048 m/s^2
    }
		
    else 
		{
        values[i] = ((float) accgyroval[i]) / 16.384f;// 4000/65536 = 1/16.384 °/s
    }
  }
	//加速计原始数据
	RT_Info.accXaxis = values[0];
	RT_Info.accYaxis = values[1];
	RT_Info.accZaxis = values[2];
	//陀螺仪除去零偏置后的数据
	RT_Info.wx=values[3] * PI/180; 
	RT_Info.wy=values[4] * PI/180;
	RT_Info.wz=values[5] * PI/180;
	//加速计校准以后的数据
	RT_Info.Calibra_accXaxis = (RT_Info.accXaxis - OffsetData.acc_offectx) * OffsetData.acc_scalex;
	RT_Info.Calibra_accYaxis = (RT_Info.accYaxis - OffsetData.acc_offecty) * OffsetData.acc_scaley;	
	RT_Info.Calibra_accZaxis = (RT_Info.accZaxis - OffsetData.acc_offectz) * OffsetData.acc_scalez;		
}

void MPU6000_initOffset(void){
	unsigned int i;
	int16_t temp[3],temp2[3];
	int32_t tempgx=0,tempgy=0,tempgz=0;
	MPU6000Gx_offset=0;
	MPU6000Gy_offset=0;
	MPU6000Gz_offset=0;

	for(i=0;i<500;i++){
		delay_us(200);
		MPU6000_readGyro_Acc(temp,temp2);
		tempgx += temp[0];
		tempgy += temp[1];
		tempgz += temp[2];
	}
	MPU6000Gx_offset=tempgx/500;
	MPU6000Gy_offset=tempgy/500;
	MPU6000Gz_offset=tempgz/500;
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
void PlaceStausCheck(float gyrox,float gyroy,float gyroz)
{
    Vector3f_t gyroDiff;
    static Vector3f_t lastGyro;
    static float threshold = 0.1f;
    static uint16_t checkNum = 0;
    static int16_t count = 0;

    gyroDiff.x = gyrox - lastGyro.x;
    gyroDiff.y = gyroy - lastGyro.y;
    gyroDiff.z = gyroz - lastGyro.z;
    lastGyro.x = gyrox;
		lastGyro.y = gyroy;
		lastGyro.z = gyroz;

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
*函 数 名: Calibration_ACC
*功能说明: 校准加速计数据
*形    参: 无
*返 回 值: 无
*****************************************************************************************/
void Calibration_ACC(float accx,float accy,float accz){
	static uint16_t samples_count = 0;
	static uint8_t orientationCaliFlag[6];
	static Vector3f_t new_offset;
	static Vector3f_t new_scale;
	static Vector3f_t samples[6];
	static uint8_t caliFlag = 0;
	static uint32_t caliCnt = 0;
	
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
					OffsetData.calibra_cnt ++;
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
					OffsetData.calibra_cnt++;
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
					OffsetData.calibra_cnt++;
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
					OffsetData.calibra_cnt++;
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
					OffsetData.calibra_cnt++;
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
					OffsetData.calibra_cnt++;
			}
	}
	if(caliFlag)
	{
			if(samples_count < 500)
			{
					samples[OffsetData.calibra_cnt - 1].x += accx;
					samples[OffsetData.calibra_cnt - 1].y += accy;
					samples[OffsetData.calibra_cnt - 1].z += accz;
					samples_count++;
			}
			else if(samples_count == 500)
			{
					samples[OffsetData.calibra_cnt - 1].x /= 500;
					samples[OffsetData.calibra_cnt - 1].y /= 500;
					samples[OffsetData.calibra_cnt - 1].z /= 500;
					samples_count++;

					caliFlag = 0;
					caliCnt  = 0;
			}
			

	}
	
	if(OffsetData.calibra_cnt == 6 && samples_count == 501)
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
						OffsetData.success = true;
        }
        else if(fabsf(new_offset.x) > (1 * 0.35f) || fabsf(new_offset.y) > (1 * 0.35f) || fabsf(new_offset.z) > (1 * 0.6f))
        {
            OffsetData.success = true;
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
					
					OffsetData.calibra_cnt = 0;
					for(uint8_t i=0; i<6; i++)
							orientationCaliFlag[i] = 0;
          OffsetData.success = true;
        }

    }
	
}
