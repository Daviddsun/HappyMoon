#include "Parameter.h"

FlashData flashData;
/***********************************************************************************************
*函 数 名: Write_Config
*功能说明: 向flash写入参数
*形    参: 无
*返 回 值: 无
************************************************************************************************/
void Write_Config(void)
{																
	u32 *ptr = &flashData.isGood;
	flashData.Offset_Data = OffsetData;
	STMFLASH_Write(0X080E0004,ptr,sizeof(flashData));
}

/***********************************************************************************************
*函 数 名: Load_Config
*功能说明: 从flash读出参数
*形    参: 无
*返 回 值: 无
************************************************************************************************/
void Load_Config(void){
	
	u32 *ptr = &flashData.isGood;
	STMFLASH_Read(0X080E0004,ptr,sizeof(flashData));
	
	if(flashData.isGood==0xA55A5AA5)
	{
		//姿态外环参数
		OriginalPitch.kP=flashData.pidPara.Pitch.Kp;
		OriginalPitch.kI=flashData.pidPara.Pitch.Ki;
		OriginalPitch.kD=flashData.pidPara.Pitch.Kd;
		
		OriginalRoll.kP=flashData.pidPara.Roll.Kp;
		OriginalRoll.kI=flashData.pidPara.Roll.Ki;
		OriginalRoll.kD=flashData.pidPara.Roll.Kd;
		
		OriginalYaw.kP=flashData.pidPara.Yaw.Kp;
		OriginalYaw.kI=flashData.pidPara.Yaw.Ki;
		OriginalYaw.kD=flashData.pidPara.Yaw.Kd;	
		//姿态内环参数
		OriginalWxRate.kP=flashData.pidPara.WxRate.Kp;
		OriginalWxRate.kI=flashData.pidPara.WxRate.Ki;
		OriginalWxRate.kD=flashData.pidPara.WxRate.Kd;
		
		OriginalWyRate.kP=flashData.pidPara.WyRate.Kp;
		OriginalWyRate.kI=flashData.pidPara.WyRate.Ki;
		OriginalWyRate.kD=flashData.pidPara.WyRate.Kd;
		
		OriginalWyRate.kP=flashData.pidPara.WzRate.Kp;
		OriginalWyRate.kI=flashData.pidPara.WzRate.Ki;
		OriginalWyRate.kD=flashData.pidPara.WzRate.Kd;	
		//三轴position参数
		OriginalPosX.kP=flashData.pidPara.PosX.Kp;
		OriginalPosX.kI=flashData.pidPara.PosX.Ki;
		OriginalPosX.kD=flashData.pidPara.PosX.Kd;
		
		OriginalPosY.kP=flashData.pidPara.PosY.Kp;
		OriginalPosY.kI=flashData.pidPara.PosY.Ki;
		OriginalPosY.kD=flashData.pidPara.PosY.Kd;	
		
		OriginalPosZ.kP=flashData.pidPara.PosZ.Kp;
		OriginalPosZ.kI=flashData.pidPara.PosZ.Ki;
		OriginalPosZ.kD=flashData.pidPara.PosZ.Kd;	
		
	  OriginalVelX.kP=flashData.pidPara.VelX.Kp;
		OriginalVelX.kI=flashData.pidPara.VelX.Ki;
		OriginalVelX.kD=flashData.pidPara.VelX.Kd;
		
		OriginalVelY.kP=flashData.pidPara.VelY.Kp;
		OriginalVelY.kI=flashData.pidPara.VelY.Ki;
		OriginalVelY.kD=flashData.pidPara.VelY.Kd;	
		
		OriginalVelZ.kP=flashData.pidPara.VelZ.Kp;
		OriginalVelZ.kI=flashData.pidPara.VelZ.Ki;
		OriginalVelZ.kD=flashData.pidPara.VelZ.Kd;	
		
		//传感器校准参数
		OffsetData.acc_offectx=flashData.Offset_Data.acc_offectx;
		OffsetData.acc_offecty=flashData.Offset_Data.acc_offecty;
		OffsetData.acc_offectz=flashData.Offset_Data.acc_offectz;

		OffsetData.acc_scalex=flashData.Offset_Data.acc_scalex;
		OffsetData.acc_scaley=flashData.Offset_Data.acc_scaley;
		OffsetData.acc_scalez=flashData.Offset_Data.acc_scalez;
		
		OffsetData.gyro_offectx=flashData.Offset_Data.gyro_offectx;
		OffsetData.gyro_offecty=flashData.Offset_Data.gyro_offecty;
		OffsetData.gyro_offectz=flashData.Offset_Data.gyro_offectz;
		
		OffsetData.gyro_scalex=flashData.Offset_Data.gyro_scalex;
		OffsetData.gyro_scaley=flashData.Offset_Data.gyro_scaley;
		OffsetData.gyro_scalez=flashData.Offset_Data.gyro_scalez;
		
	}else{
		
		flashData.isGood=0xA55A5AA5;
		//姿态外环参数		
		OriginalPitch.kP=flashData.pidPara.Pitch.Kp = 0;
		OriginalPitch.kI=flashData.pidPara.Pitch.Ki = 0;
		OriginalPitch.kD=flashData.pidPara.Pitch.Kd = 0;
		
		OriginalRoll.kP=flashData.pidPara.Roll.Kp = 0;
		OriginalRoll.kI=flashData.pidPara.Roll.Ki = 0;
		OriginalRoll.kD=flashData.pidPara.Roll.Kd = 0;
		
		OriginalYaw.kP=flashData.pidPara.Yaw.Kp = 0;
		OriginalYaw.kI=flashData.pidPara.Yaw.Ki = 0;
		OriginalYaw.kD=flashData.pidPara.Yaw.Kd = 0;
		//姿态内环参数
		OriginalWxRate.kP=flashData.pidPara.WxRate.Kp = 0;
		OriginalWxRate.kI=flashData.pidPara.WxRate.Ki = 0;
		OriginalWxRate.kD=flashData.pidPara.WxRate.Kd = 0;
		
		OriginalWyRate.kP=flashData.pidPara.WyRate.Kp = 0;
		OriginalWyRate.kI=flashData.pidPara.WyRate.Ki = 0;
		OriginalWyRate.kD=flashData.pidPara.WyRate.Kd = 0;
		
		OriginalWyRate.kP=flashData.pidPara.WzRate.Kp = 0;
		OriginalWyRate.kI=flashData.pidPara.WzRate.Ki = 0;
		OriginalWyRate.kD=flashData.pidPara.WzRate.Kd = 0;	
		//三轴position参数
		OriginalPosX.kP=flashData.pidPara.PosX.Kp = 0;
		OriginalPosX.kI=flashData.pidPara.PosX.Ki = 0;
		OriginalPosX.kD=flashData.pidPara.PosX.Kd = 0;
		
		OriginalPosY.kP=flashData.pidPara.PosY.Kp = 0;
		OriginalPosY.kI=flashData.pidPara.PosY.Ki = 0;
		OriginalPosY.kD=flashData.pidPara.PosY.Kd = 0;	
		
		OriginalPosZ.kP=flashData.pidPara.PosZ.Kp = 0;
		OriginalPosZ.kI=flashData.pidPara.PosZ.Ki = 0;
		OriginalPosZ.kD=flashData.pidPara.PosZ.Kd = 0;	
		
	  OriginalVelX.kP=flashData.pidPara.VelX.Kp = 0;
		OriginalVelX.kI=flashData.pidPara.VelX.Ki = 0;
		OriginalVelX.kD=flashData.pidPara.VelX.Kd = 0;
		
		OriginalVelY.kP=flashData.pidPara.VelY.Kp = 0;
		OriginalVelY.kI=flashData.pidPara.VelY.Ki = 0;
		OriginalVelY.kD=flashData.pidPara.VelY.Kd = 0;	
		
		OriginalVelZ.kP=flashData.pidPara.VelZ.Kp = 0;
		OriginalVelZ.kI=flashData.pidPara.VelZ.Ki = 0;
		OriginalVelZ.kD=flashData.pidPara.VelZ.Kd = 0;	
		//传感器校准参数
		OffsetData.acc_offectx=flashData.Offset_Data.acc_offectx = 0;
		OffsetData.acc_offecty=flashData.Offset_Data.acc_offecty = 0;
		OffsetData.acc_offectz=flashData.Offset_Data.acc_offectz = 0;
		
		OffsetData.acc_scalex=flashData.Offset_Data.acc_scalex = 1;
		OffsetData.acc_scaley=flashData.Offset_Data.acc_scaley = 1;
		OffsetData.acc_scalez=flashData.Offset_Data.acc_scalez = 1;
		
		OffsetData.gyro_offectx=flashData.Offset_Data.gyro_offectx = 0;
		OffsetData.gyro_offecty=flashData.Offset_Data.gyro_offecty = 0;
		OffsetData.gyro_offectz=flashData.Offset_Data.gyro_offectz = 0;
		
		OffsetData.gyro_scalex=flashData.Offset_Data.gyro_scalex = 1;
		OffsetData.gyro_scaley=flashData.Offset_Data.gyro_scaley = 1;
		OffsetData.gyro_scalez=flashData.Offset_Data.gyro_scalez = 1;
	
		Write_Config();	
	}
}


