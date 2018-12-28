#include "MPU6500.h"

/***********************************************************************************
*函 数 名: MPU6500_writeReg
*功能说明: spi读写MPU6000
*形    参: 
*返 回 值: 状态
************************************************************************************/
void MPU6500_writeReg(u8 reg, u8 data)
{
	MPU6500_CSL();
	SPI3_ReadWrite_Byte(reg);
	SPI3_ReadWrite_Byte(data);
	MPU6500_CSH();	
}

/**********************************************************************************************************
*函 数 名: MPU6500_Initialize
*功能说明: MPU6500寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MPU6500_Initialize(void)
{
    MPU6500_writeReg(MPU_RA_PWR_MGMT_1, 0x80);
    delay_ms(150);

    MPU6500_writeReg(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay_ms(150);

    MPU6500_writeReg(MPU_RA_PWR_MGMT_1, 0x03);
    delay_ms(15);

    MPU6500_writeReg(MPU_RA_USER_CTRL, 0x10);
    delay_ms(15);

    MPU6500_writeReg(MPU_RA_PWR_MGMT_2, 0x00);
    delay_ms(15);

    //陀螺仪采样率0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
    MPU6500_writeReg(MPU_RA_SMPLRT_DIV, (1000/1000 - 1));
    delay_ms(15);

    //i2c旁路模式
    // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    MPU6500_writeReg(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
    delay_ms(15);

    //低通滤波频率
    MPU6500_writeReg(MPU_RA_CONFIG, MPU_LPF_42HZ);
    delay_ms(15);

    //陀螺仪自检及测量范围，典型值0x18(不自检，2000deg/s) (0x10 1000deg/s) (0x10 1000deg/s) (0x08 500deg/s)
    MPU6500_writeReg(MPU_RA_GYRO_CONFIG, 0x10);
    delay_ms(15);

    //加速度自检、测量范围(不自检，+-8G)
    MPU6500_writeReg(MPU_RA_ACCEL_CONFIG, 2 << 3);
    delay_ms(15);

    //加速度低通滤波设置
    MPU6500_writeReg(MPU_RA_ACCEL_CONFIG2, MPU_LPF_42HZ);

    delay_ms(15);
}

/**********************************************************************************************************
*函 数 名: MPU6500_ReadAccGyro
*功能说明: MPU6500读取加速度陀螺仪传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void MPU6500_ReadAccGyro(Vector3f_t *acc ,Vector3f_t *gyro){
	static uint8_t buffer[14];
	static Vector3i_t accRaw,gyroRaw;
	MPU6500_CSL();
	SPI3_readRegs(MPU_RA_ACCEL_XOUT_H,14,buffer);
	MPU6500_CSH();
	//acc
	accRaw.x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
	accRaw.y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
	accRaw.z = ((((int16_t)buffer[4]) << 8) | buffer[5]);
	//gyro
	gyroRaw.x = ((((int16_t)buffer[8]) << 8) | buffer[9]);
  gyroRaw.y = ((((int16_t)buffer[10]) << 8) | buffer[11]);
  gyroRaw.z = ((((int16_t)buffer[12]) << 8) | buffer[13]);

	acc->x = ((float)accRaw.x) * MPU_A_8mg;
	acc->y = ((float)accRaw.y) * MPU_A_8mg;
	acc->z = ((float)accRaw.z) * MPU_A_8mg;
	
	gyro->x = ((float)gyroRaw.x) * MPU_G_s1000dps * PI/180;
	gyro->y = ((float)gyroRaw.y) * MPU_G_s1000dps * PI/180;
	gyro->z = ((float)gyroRaw.z) * MPU_G_s1000dps * PI/180;
	
}
