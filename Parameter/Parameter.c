#include "Parameter.h"

FlashData flashData;

/***********************************************************************************************
*函 数 名: Write_Config
*功能说明: 向flash写入参数
*形    参: 无
*返 回 值: 无
************************************************************************************************/
void Write_Config(void){																
	u32 *ptr = &flashData.isGood;
	flashData.pidPara = PID_ParaInfo;
	flashData.Offset_Data = OffsetData;
	STMFLASH_Write(0X080E0004,ptr,sizeof(flashData));
}

/***********************************************************************************************
*函 数 名: Load_SensorConfig
*功能说明: 从flash读出参数IMU校准参数
*形    参: 无
*返 回 值: 无
************************************************************************************************/
void Load_SensorConfig(void){
	
	u32 *ptr = &flashData.isGood;
	STMFLASH_Read(0X080E0004,ptr,sizeof(flashData));
	
	if(flashData.isGood==0xA55A5AA5){
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
		
		OffsetData.level_scale.x=flashData.Offset_Data.level_scale.x;
		OffsetData.level_scale.y=flashData.Offset_Data.level_scale.y;
		OffsetData.level_scale.z=flashData.Offset_Data.level_scale.z;
	}else{
		flashData.isGood=0xA55A5AA5;
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
		
		OffsetData.level_scale.x=flashData.Offset_Data.level_scale.x = 0;
		OffsetData.level_scale.y=flashData.Offset_Data.level_scale.y = 0;
		OffsetData.level_scale.z=flashData.Offset_Data.level_scale.z = 0;
	
		Write_Config();	
	}
}

/***********************************************************************************************
*函 数 名: Load_PIDConfig
*功能说明: 从flash读出参数IMU校准参数
*形    参: 无
*返 回 值: 无
************************************************************************************************/
void Load_PIDConfig(void){
	
	u32 *ptr = &flashData.isGood;
	STMFLASH_Read(0X080E0004,ptr,sizeof(flashData));
	if(flashData.isGood==0xA55A5AA5){
		
		//姿态外环参数
		OriginalPitch.kP=PID_ParaInfo.Pitch.Kp=flashData.pidPara.Pitch.Kp;
		OriginalPitch.kI=PID_ParaInfo.Pitch.Ki=flashData.pidPara.Pitch.Ki;
		OriginalPitch.kD=PID_ParaInfo.Pitch.Kd=flashData.pidPara.Pitch.Kd;
		
		OriginalRoll.kP=PID_ParaInfo.Roll.Kp=flashData.pidPara.Roll.Kp;
		OriginalRoll.kI=PID_ParaInfo.Roll.Ki=flashData.pidPara.Roll.Ki;
		OriginalRoll.kD=PID_ParaInfo.Roll.Kd=flashData.pidPara.Roll.Kd;
		
		OriginalYaw.kP=PID_ParaInfo.Yaw.Kp=flashData.pidPara.Yaw.Kp;
		OriginalYaw.kI=PID_ParaInfo.Yaw.Ki=flashData.pidPara.Yaw.Ki;
		OriginalYaw.kD=PID_ParaInfo.Yaw.Kd=flashData.pidPara.Yaw.Kd;	
		//姿态内环参数
		OriginalWxRate.kP=PID_ParaInfo.WxRate.Kp=flashData.pidPara.WxRate.Kp;
		OriginalWxRate.kI=PID_ParaInfo.WxRate.Ki=flashData.pidPara.WxRate.Ki;
		OriginalWxRate.kD=PID_ParaInfo.WxRate.Kd=flashData.pidPara.WxRate.Kd;
		
		OriginalWyRate.kP=PID_ParaInfo.WyRate.Kp=flashData.pidPara.WyRate.Kp;
		OriginalWyRate.kI=PID_ParaInfo.WyRate.Ki=flashData.pidPara.WyRate.Ki;
		OriginalWyRate.kD=PID_ParaInfo.WyRate.Kd=flashData.pidPara.WyRate.Kd;
		
		OriginalWzRate.kP=PID_ParaInfo.WzRate.Kp=flashData.pidPara.WzRate.Kp;
		OriginalWzRate.kI=PID_ParaInfo.WzRate.Ki=flashData.pidPara.WzRate.Ki;
		OriginalWzRate.kD=PID_ParaInfo.WzRate.Kd=flashData.pidPara.WzRate.Kd;	
		//三轴position参数
		OriginalPosX.kP=PID_ParaInfo.PosX.Kp=flashData.pidPara.PosX.Kp;
		OriginalPosX.kI=PID_ParaInfo.PosX.Ki=flashData.pidPara.PosX.Ki;
		OriginalPosX.kD=PID_ParaInfo.PosX.Kd=flashData.pidPara.PosX.Kd;
		
		OriginalPosY.kP=PID_ParaInfo.PosY.Kp=flashData.pidPara.PosY.Kp;
		OriginalPosY.kI=PID_ParaInfo.PosY.Ki=flashData.pidPara.PosY.Ki;
		OriginalPosY.kD=PID_ParaInfo.PosY.Kd=flashData.pidPara.PosY.Kd;	
		
		OriginalPosZ.kP=PID_ParaInfo.PosZ.Kp=flashData.pidPara.PosZ.Kp;
		OriginalPosZ.kI=PID_ParaInfo.PosZ.Ki=flashData.pidPara.PosZ.Ki;
		OriginalPosZ.kD=PID_ParaInfo.PosZ.Kd=flashData.pidPara.PosZ.Kd;	
		
	  OriginalVelX.kP=PID_ParaInfo.VelX.Kp=flashData.pidPara.VelX.Kp;
		OriginalVelX.kI=PID_ParaInfo.VelX.Ki=flashData.pidPara.VelX.Ki;
		OriginalVelX.kD=PID_ParaInfo.VelX.Kd=flashData.pidPara.VelX.Kd;
		
		OriginalVelY.kP=PID_ParaInfo.VelY.Kp=flashData.pidPara.VelY.Kp;
		OriginalVelY.kI=PID_ParaInfo.VelY.Ki=flashData.pidPara.VelY.Ki;
		OriginalVelY.kD=PID_ParaInfo.VelY.Kd=flashData.pidPara.VelY.Kd;	
		
		OriginalVelZ.kP=PID_ParaInfo.VelZ.Kp=flashData.pidPara.VelZ.Kp;
		OriginalVelZ.kI=PID_ParaInfo.VelZ.Ki=flashData.pidPara.VelZ.Ki;
		OriginalVelZ.kD=PID_ParaInfo.VelZ.Kd=flashData.pidPara.VelZ.Kd;	
	}
	else{
		flashData.isGood=0xA55A5AA5;
		//姿态外环参数
		OriginalPitch.kP=PID_ParaInfo.Pitch.Kp=flashData.pidPara.Pitch.Kp=0;
		OriginalPitch.kI=PID_ParaInfo.Pitch.Ki=flashData.pidPara.Pitch.Ki=0;
		OriginalPitch.kD=PID_ParaInfo.Pitch.Kd=flashData.pidPara.Pitch.Kd=0;
		
		OriginalRoll.kP=PID_ParaInfo.Roll.Kp=flashData.pidPara.Roll.Kp=0;
		OriginalRoll.kI=PID_ParaInfo.Roll.Ki=flashData.pidPara.Roll.Ki=0;
		OriginalRoll.kD=PID_ParaInfo.Roll.Kd=flashData.pidPara.Roll.Kd=0;
		
		OriginalYaw.kP=PID_ParaInfo.Yaw.Kp=flashData.pidPara.Yaw.Kp=0;
		OriginalYaw.kI=PID_ParaInfo.Yaw.Ki=flashData.pidPara.Yaw.Ki=0;
		OriginalYaw.kD=PID_ParaInfo.Yaw.Kd=flashData.pidPara.Yaw.Kd=0;	
		//姿态内环参数
		OriginalWxRate.kP=PID_ParaInfo.WxRate.Kp=flashData.pidPara.WxRate.Kp=0;
		OriginalWxRate.kI=PID_ParaInfo.WxRate.Ki=flashData.pidPara.WxRate.Ki=0;
		OriginalWxRate.kD=PID_ParaInfo.WxRate.Kd=flashData.pidPara.WxRate.Kd=0;
		
		OriginalWyRate.kP=PID_ParaInfo.WyRate.Kp=flashData.pidPara.WyRate.Kp=0;
		OriginalWyRate.kI=PID_ParaInfo.WyRate.Ki=flashData.pidPara.WyRate.Ki=0;
		OriginalWyRate.kD=PID_ParaInfo.WyRate.Kd=flashData.pidPara.WyRate.Kd=0;
		
		OriginalWzRate.kP=PID_ParaInfo.WzRate.Kp=flashData.pidPara.WzRate.Kp=0;
		OriginalWzRate.kI=PID_ParaInfo.WzRate.Ki=flashData.pidPara.WzRate.Ki=0;
		OriginalWzRate.kD=PID_ParaInfo.WzRate.Kd=flashData.pidPara.WzRate.Kd=0;	
		//三轴position参数
		OriginalPosX.kP=PID_ParaInfo.PosX.Kp=flashData.pidPara.PosX.Kp=0;
		OriginalPosX.kI=PID_ParaInfo.PosX.Ki=flashData.pidPara.PosX.Ki=0;
		OriginalPosX.kD=PID_ParaInfo.PosX.Kd=flashData.pidPara.PosX.Kd=0;
		
		OriginalPosY.kP=PID_ParaInfo.PosY.Kp=flashData.pidPara.PosY.Kp=0;
		OriginalPosY.kI=PID_ParaInfo.PosY.Ki=flashData.pidPara.PosY.Ki=0;
		OriginalPosY.kD=PID_ParaInfo.PosY.Kd=flashData.pidPara.PosY.Kd=0;	
		
		OriginalPosZ.kP=PID_ParaInfo.PosZ.Kp=flashData.pidPara.PosZ.Kp=0;
		OriginalPosZ.kI=PID_ParaInfo.PosZ.Ki=flashData.pidPara.PosZ.Ki=0;
		OriginalPosZ.kD=PID_ParaInfo.PosZ.Kd=flashData.pidPara.PosZ.Kd=0;	
		
	  OriginalVelX.kP=PID_ParaInfo.VelX.Kp=flashData.pidPara.VelX.Kp=0;
		OriginalVelX.kI=PID_ParaInfo.VelX.Ki=flashData.pidPara.VelX.Ki=0;
		OriginalVelX.kD=PID_ParaInfo.VelX.Kd=flashData.pidPara.VelX.Kd=0;
		
		OriginalVelY.kP=PID_ParaInfo.VelY.Kp=flashData.pidPara.VelY.Kp=0;
		OriginalVelY.kI=PID_ParaInfo.VelY.Ki=flashData.pidPara.VelY.Ki=0;
		OriginalVelY.kD=PID_ParaInfo.VelY.Kd=flashData.pidPara.VelY.Kd=0;	
		
		OriginalVelZ.kP=PID_ParaInfo.VelZ.Kp=flashData.pidPara.VelZ.Kp=0;
		OriginalVelZ.kI=PID_ParaInfo.VelZ.Ki=flashData.pidPara.VelZ.Ki=0;
		OriginalVelZ.kD=PID_ParaInfo.VelZ.Kd=flashData.pidPara.VelZ.Kd=0;	
		
		Write_Config();	
	}
}





/* END */
