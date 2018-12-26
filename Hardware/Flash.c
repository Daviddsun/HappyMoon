#include "Flash.h"

FlashData flashData;

u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  

uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}


void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	
	FLASH_Unlock();									
  FLASH_DataCacheCmd(DISABLE);
 		
	addrx=WriteAddr;				
	endaddr=WriteAddr+NumToWrite*4;	
	if(addrx<0X1FFF0000)			
	{
		while(addrx<endaddr)		
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);
				if(status!=FLASH_COMPLETE)break;	
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)
			{ 
				break;	
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);
	FLASH_Lock();
} 


void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);
		ReadAddr+=4;
	}
}


void Write_config(void)
{																
	u32 *ptr = &flashData.isGood;
	flashData.Offset_Data = OffsetData;
	STMFLASH_Write(0X080E0004,ptr,sizeof(flashData));
}

void Load_Config(void){
	
	u32 *ptr = &flashData.isGood;
	STMFLASH_Read(0X080E0004,ptr,sizeof(flashData));
	
	if(flashData.isGood==0xA55A5AA5)
	{		
//		PID_ParaInfo.Pitch.Kp=flashData.pidPara.Pitch.Kp;
//		PID_ParaInfo.Pitch.Ki=flashData.pidPara.Pitch.Ki;
//		PID_ParaInfo.Pitch.Kd=flashData.pidPara.Pitch.Kd;
//		
//		PID_ParaInfo.Roll.Kp=flashData.pidPara.Roll.Kp;
//		PID_ParaInfo.Roll.Ki=flashData.pidPara.Roll.Ki;
//		PID_ParaInfo.Roll.Kd=flashData.pidPara.Roll.Kd;
//		
//		PID_ParaInfo.Yaw.Kp=flashData.pidPara.Yaw.Kp;
//		PID_ParaInfo.Yaw.Ki=flashData.pidPara.Yaw.Ki;
//		PID_ParaInfo.Yaw.Kd=flashData.pidPara.Yaw.Kd;	
//			
//		PID_ParaInfo.WxRate.Kp=flashData.pidPara.WxRate.Kp;
//		PID_ParaInfo.WxRate.Ki=flashData.pidPara.WxRate.Ki;
//		PID_ParaInfo.WxRate.Kd=flashData.pidPara.WxRate.Kd;
//		
//		PID_ParaInfo.WyRate.Kp=flashData.pidPara.WyRate.Kp;
//		PID_ParaInfo.WyRate.Ki=flashData.pidPara.WyRate.Ki;
//		PID_ParaInfo.WyRate.Kd=flashData.pidPara.WyRate.Kd;
//		
//		PID_ParaInfo.WzRate.Kp=flashData.pidPara.WzRate.Kp;
//		PID_ParaInfo.WzRate.Ki=flashData.pidPara.WzRate.Ki;
//		PID_ParaInfo.WzRate.Kd=flashData.pidPara.WzRate.Kd;	
//	
//		PID_ParaInfo.PosX.Kp=flashData.pidPara.PosX.Kp;
//		PID_ParaInfo.PosX.Ki=flashData.pidPara.PosX.Ki;
//		PID_ParaInfo.PosX.Kd=flashData.pidPara.PosX.Kd;
//		
//		PID_ParaInfo.PosY.Kp=flashData.pidPara.PosY.Kp;
//		PID_ParaInfo.PosY.Ki=flashData.pidPara.PosY.Ki;
//		PID_ParaInfo.PosY.Kd=flashData.pidPara.PosY.Kd;	
//		
//		PID_ParaInfo.PosZ.Kp=flashData.pidPara.PosZ.Kp;
//		PID_ParaInfo.PosZ.Ki=flashData.pidPara.PosZ.Ki;
//		PID_ParaInfo.PosZ.Kd=flashData.pidPara.PosZ.Kd;	
//		
//	  PID_ParaInfo.VelX.Kp=flashData.pidPara.VelX.Kp;
//		PID_ParaInfo.VelX.Ki=flashData.pidPara.VelX.Ki;
//		PID_ParaInfo.VelX.Kd=flashData.pidPara.VelX.Kd;
//		
//		PID_ParaInfo.VelY.Kp=flashData.pidPara.VelY.Kp;
//		PID_ParaInfo.VelY.Ki=flashData.pidPara.VelY.Ki;
//		PID_ParaInfo.VelY.Kd=flashData.pidPara.VelY.Kd;	
//		
//		PID_ParaInfo.VelZ.Kp=flashData.pidPara.VelZ.Kp;
//		PID_ParaInfo.VelZ.Ki=flashData.pidPara.VelZ.Ki;
//		PID_ParaInfo.VelZ.Kd=flashData.pidPara.VelZ.Kd;	
//		
//		OffsetData.acc_offectx=flashData.Offset_Data.acc_offectx;
//		OffsetData.acc_offecty=flashData.Offset_Data.acc_offecty;
//		OffsetData.acc_offectz=flashData.Offset_Data.acc_offectz;

//		OffsetData.acc_scalex=flashData.Offset_Data.acc_scalex;
//		OffsetData.acc_scaley=flashData.Offset_Data.acc_scaley;
//		OffsetData.acc_scalez=flashData.Offset_Data.acc_scalez;
//		
//		OffsetData.gyro_offectx=flashData.Offset_Data.gyro_offectx;
//		OffsetData.gyro_offecty=flashData.Offset_Data.gyro_offecty;
//		OffsetData.gyro_offectz=flashData.Offset_Data.gyro_offectz;
//		
//		OffsetData.gyro_scalex=flashData.Offset_Data.gyro_scalex;
//		OffsetData.gyro_scaley=flashData.Offset_Data.gyro_scaley;
//		OffsetData.gyro_scalez=flashData.Offset_Data.gyro_scalez;
		
	}else{
		
		flashData.isGood=0xA55A5AA5;
//		PID_ParaInfo.Pitch.Kp=flashData.pidPara.Pitch.Kp = 0;
//		PID_ParaInfo.Pitch.Ki=flashData.pidPara.Pitch.Ki = 0;
//		PID_ParaInfo.Pitch.Kd=flashData.pidPara.Pitch.Kd = 0;
//		
//		PID_ParaInfo.Roll.Kp=flashData.pidPara.Roll.Kp = 0;
//		PID_ParaInfo.Roll.Ki=flashData.pidPara.Roll.Ki = 0;
//		PID_ParaInfo.Roll.Kd=flashData.pidPara.Roll.Kd = 0;
//		
//		PID_ParaInfo.Yaw.Kp=flashData.pidPara.Yaw.Kp = 0;
//		PID_ParaInfo.Yaw.Ki=flashData.pidPara.Yaw.Ki = 0;
//		PID_ParaInfo.Yaw.Kd=flashData.pidPara.Yaw.Kd = 0;	
//		
//		PID_ParaInfo.WxRate.Kp=flashData.pidPara.WxRate.Kp = 0;
//		PID_ParaInfo.WxRate.Ki=flashData.pidPara.WxRate.Ki = 0;
//		PID_ParaInfo.WxRate.Kd=flashData.pidPara.WxRate.Kd = 0;
//		
//		PID_ParaInfo.WyRate.Kp=flashData.pidPara.WyRate.Kp = 0;
//		PID_ParaInfo.WyRate.Ki=flashData.pidPara.WyRate.Ki = 0;
//		PID_ParaInfo.WyRate.Kd=flashData.pidPara.WyRate.Kd = 0;
//		
//		PID_ParaInfo.WzRate.Kp=flashData.pidPara.WzRate.Kp = 0;
//		PID_ParaInfo.WzRate.Ki=flashData.pidPara.WzRate.Ki = 0;
//		PID_ParaInfo.WzRate.Kd=flashData.pidPara.WzRate.Kd = 0;	
//		
//		PID_ParaInfo.PosX.Kp=flashData.pidPara.PosX.Kp = 0;
//		PID_ParaInfo.PosX.Ki=flashData.pidPara.PosX.Ki = 0;
//		PID_ParaInfo.PosX.Kd=flashData.pidPara.PosX.Kd = 0;
//		
//		PID_ParaInfo.PosY.Kp=flashData.pidPara.PosY.Kp = 0;
//		PID_ParaInfo.PosY.Ki=flashData.pidPara.PosY.Ki = 0;
//		PID_ParaInfo.PosY.Kd=flashData.pidPara.PosY.Kd = 0;	
//		
//		PID_ParaInfo.PosZ.Kp=flashData.pidPara.PosZ.Kp = 0;
//		PID_ParaInfo.PosZ.Ki=flashData.pidPara.PosZ.Ki = 0;
//		PID_ParaInfo.PosZ.Kd=flashData.pidPara.PosZ.Kd = 0;	
//		
//		PID_ParaInfo.VelX.Kp=flashData.pidPara.VelX.Kp = 0;
//		PID_ParaInfo.VelX.Ki=flashData.pidPara.VelX.Ki = 0;
//		PID_ParaInfo.VelX.Kd=flashData.pidPara.VelX.Kd = 0;
//		
//		PID_ParaInfo.VelY.Kp=flashData.pidPara.VelY.Kp = 0;
//		PID_ParaInfo.VelY.Ki=flashData.pidPara.VelY.Ki = 0;
//		PID_ParaInfo.VelY.Kd=flashData.pidPara.VelY.Kd = 0;	
//		
//		PID_ParaInfo.VelZ.Kp=flashData.pidPara.VelZ.Kp = 0;
//		PID_ParaInfo.VelZ.Ki=flashData.pidPara.VelZ.Ki = 0;
//		PID_ParaInfo.VelZ.Kd=flashData.pidPara.VelZ.Kd = 0;	
//		
//		OffsetData.acc_offectx=flashData.Offset_Data.acc_offectx = 0;
//		OffsetData.acc_offecty=flashData.Offset_Data.acc_offecty = 0;
//		OffsetData.acc_offectz=flashData.Offset_Data.acc_offectz = 0;
//		
//		OffsetData.acc_scalex=flashData.Offset_Data.acc_scalex = 1;
//		OffsetData.acc_scaley=flashData.Offset_Data.acc_scaley = 1;
//		OffsetData.acc_scalez=flashData.Offset_Data.acc_scalez = 1;
//		
//		OffsetData.gyro_offectx=flashData.Offset_Data.gyro_offectx = 0;
//		OffsetData.gyro_offecty=flashData.Offset_Data.gyro_offecty = 0;
//		OffsetData.gyro_offectz=flashData.Offset_Data.gyro_offectz = 0;
//		
//		OffsetData.gyro_scalex=flashData.Offset_Data.gyro_scalex = 1;
//		OffsetData.gyro_scaley=flashData.Offset_Data.gyro_scaley = 1;
//		OffsetData.gyro_scalez=flashData.Offset_Data.gyro_scalez = 1;
	
		Write_config();	
	}
}


