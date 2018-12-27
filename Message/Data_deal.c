#include "Data_deal.h"
int sumCheck(Data_Rx rx)
{
	int i=0;
	unsigned char sum;
	for(i=0;i<rx.len-1;i++)
	{
		sum^=rx.buf[i];
	}
	if(sum==rx.buf[rx.len-1])
		return 1;
	else 
		return 0;
}

static u8 firstdata[20];
static u8 secdata[20];
Data_Rx Stitchingdata;

void DataStitching(Data_Rx rx)
{
	static u8 i,firstlen;
	static u8 BluetoothStitch[20];
	static u8 stitchingflag = 0;

	if(rx.len!=20 && stitchingflag == 1 && rx.buf[0]!=0x55 && rx.buf[1]!=0xAA)
	{
		memcpy(secdata,rx.buf,sizeof(rx.buf));
		for(i = firstlen;i<(rx.len + firstlen);i++)
		{
			BluetoothStitch[i] = secdata[i-firstlen];
		}
		Stitchingdata.len = sizeof(BluetoothStitch);
		stitchingflag = 0;
		memcpy(Stitchingdata.buf,BluetoothStitch,sizeof(BluetoothStitch));
		DataDeal(Stitchingdata);
	}

	else if(rx.len<=20 && rx.buf[0]==0x55)
	{
		memcpy(firstdata,rx.buf,sizeof(rx.buf));
		for(i = 0;i<rx.len;i++)
		{
			BluetoothStitch[i] = firstdata[i];
		}
		firstlen = 	rx.len;	
		stitchingflag = 1;
	}
	
	else if(rx.len==20 && rx.buf[0]==0x55 && rx.buf[1]==0xAA)
	{
		DataDeal(rx);
	}
}
void DataDeal(Data_Rx rx)
{
//	float pidParaTemp[3];
	u8 HexToFloat[4];
	u8 i,j;
	if( rx.len==20 && rx.buf[0]==0x55 && rx.buf[1]==0xAA )
	{
		if(rx.buf[2]==0xff && RT_Info.lowPowerFlag==0) 
		{
			if(rx.buf[3]==0){
				FlightControl.OnOff = Drone_Off;
			}
			else if(rx.buf[3]==1){					
				FlightControl.OnOff = Drone_On;
			}
			else if(rx.buf[3]==2){
				FlightControl.landFlag=1;
			}						
		}
		else if(rx.buf[2]==1 && FlightControl.droneMode!=Drone_Mode_4Axis ) 
		{
			/*  Target_Pitch */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[3+i];
			}
			Target_Info.Pitch = Hex_To_Decimal(HexToFloat,4) * PI/180; 
			/*  Target_Roll */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[7+i];
			}
//			Target_Info.Roll = Hex_To_Decimal(HexToFloat,4) * PI/180;								
			/*  Target_Yaw */					
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[11+i];
			}
//			Target_Info.Yaw = Hex_To_Decimal(HexToFloat,4) * PI/180; 
		}	
		else if(rx.buf[2]==2)
		{
			switch(rx.buf[3])
			{
				case 0:
						FlightControl.droneMode=Drone_Mode_None;
					break;
				case 1:
						FlightControl.droneMode=Drone_Mode_Pitch;
					break;
				case 2:
						FlightControl.droneMode=Drone_Mode_Roll;		
					break;
				case 3:
						FlightControl.droneMode=Drone_Mode_4Axis; 
					break;
				case 4:
						FlightControl.droneMode=Drone_Mode_RatePitch; 
					break;
				case 5:
						FlightControl.droneMode=Drone_Mode_RateRoll;
				default:
					break;
			}
		}
		/* 发送PID参数 */
		else if(rx.buf[2] == 3) 
		{
			FlightControl.ReportSW=Report_SET;					
		}		
		/* 加速计校准 */
		else if(rx.buf[2] == 4)
		{
			OffsetData.acc_success = true;
		}
		/* Pitch PID */	
		else if(rx.buf[2] == 5)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.Pitch.Kp=pidParaTemp[0];
//			PID_ParaInfo.Pitch.Ki=pidParaTemp[1];
//			PID_ParaInfo.Pitch.Kd=pidParaTemp[2];
//			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}		
		/* Roll PID */					
		else if(rx.buf[2] == 6)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}     
//			PID_ParaInfo.Roll.Kp=pidParaTemp[0];
//			PID_ParaInfo.Roll.Ki=pidParaTemp[1];
//			PID_ParaInfo.Roll.Kd=pidParaTemp[2];
//			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/* Yaw PID*/					
		else if(rx.buf[2] == 7)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.Yaw.Kp=pidParaTemp[0];
//			PID_ParaInfo.Yaw.Ki=pidParaTemp[1];
//			PID_ParaInfo.Yaw.Kd=pidParaTemp[2];
//			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/* PosZ PID*/					
		else if(rx.buf[2] == 8)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.PosZ.Kp=pidParaTemp[0];
//			PID_ParaInfo.PosZ.Ki=pidParaTemp[1];
//			PID_ParaInfo.PosZ.Kd=pidParaTemp[2];
//			Write_config();		
			FlightControl.ReportSW=Report_SET;
		}	 
	 /* ratePitch PID*/					
		else if(rx.buf[2] == 11)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.WyRate.Kp=pidParaTemp[0];
//			PID_ParaInfo.WyRate.Ki=pidParaTemp[1];
//			PID_ParaInfo.WyRate.Kd=pidParaTemp[2];
//			Write_config();		
			FlightControl.ReportSW=Report_SET;
		}
	 /* rateRoll PID*/					
		else if(rx.buf[2] == 12)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.WxRate.Kp=pidParaTemp[0];
//			PID_ParaInfo.WxRate.Ki=pidParaTemp[1];
//			PID_ParaInfo.WxRate.Kd=pidParaTemp[2];
//			Write_config();		
			FlightControl.ReportSW=Report_SET;
		}
		/* rateYaw PID*/					
		else if(rx.buf[2] == 13)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.WzRate.Kp=pidParaTemp[0];
//			PID_ParaInfo.WzRate.Ki=pidParaTemp[1];
//			PID_ParaInfo.WzRate.Kd=pidParaTemp[2];
//			Write_config();		
			FlightControl.ReportSW=Report_SET;
		}
		/* VelZ PID*/					
		else if(rx.buf[2] == 14)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}
//			PID_ParaInfo.VelZ.Kp=pidParaTemp[0];
//			PID_ParaInfo.VelZ.Ki=pidParaTemp[1];
//			PID_ParaInfo.VelZ.Kd=pidParaTemp[2];
//			Write_config();
			FlightControl.ReportSW=Report_SET;
		}	
		/* Rate */
		else if(rx.buf[2]==15) 
		{
			/* Target_RatePitch */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[3+i];
			}
//			Target_Info.RatePitch = Hex_To_Decimal(HexToFloat,4); 
			/* Target_RateRoll */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[7+i];
			}
//			Target_Info.RateRoll = Hex_To_Decimal(HexToFloat,4);										
		}
		/* 阶跃信号 */
		else if(rx.buf[2]==16){
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[3+i];
			}
//			reference_state.postionX = Hex_To_Decimal(HexToFloat,4);
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[7+i];
			}
//			reference_state.postionY = Hex_To_Decimal(HexToFloat,4); 
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[11+i];
			}
//			reference_state.postionZ = Hex_To_Decimal(HexToFloat,4);			
		}
		/* 遥控器数据 */
		else if(rx.buf[2]==17)
		{
			/* XaxisPos */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[3+i];
			}
//			RockerControl.XaxisPos = Hex_To_Decimal(HexToFloat,4); 
			/* YaxisPos */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[7+i];
			}
//			RockerControl.YaxisPos = Hex_To_Decimal(HexToFloat,4); 					
			/* Navigation */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[11+i];
			}
//			RockerControl.Navigation = Hex_To_Decimal(HexToFloat,4);
			/* ZaxisPos */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[15+i];
			}
//			RockerControl.ZaxisPos = Hex_To_Decimal(HexToFloat,4);
		}
		/* PositionX PID */					
		else if(rx.buf[2] == 18)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.PosX.Kp=pidParaTemp[0];
//			PID_ParaInfo.PosX.Ki=pidParaTemp[1];
//			PID_ParaInfo.PosX.Kd=pidParaTemp[2];
//			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/* PositionY PID*/					
		else if(rx.buf[2] == 19)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.PosY.Kp=pidParaTemp[0];
//			PID_ParaInfo.PosY.Ki=pidParaTemp[1];
//			PID_ParaInfo.PosY.Kd=pidParaTemp[2];
//			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/* SpeedX PID*/					
		else if(rx.buf[2] == 20)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.VelX.Kp=pidParaTemp[0];
//			PID_ParaInfo.VelX.Ki=pidParaTemp[1];
//			PID_ParaInfo.VelX.Kd=pidParaTemp[2];
//			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/* SpeedY PID*/					
		else if(rx.buf[2] == 21)
		{
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
//				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
//			PID_ParaInfo.VelY.Kp=pidParaTemp[0];
//			PID_ParaInfo.VelY.Ki=pidParaTemp[1];
//			PID_ParaInfo.VelY.Kd=pidParaTemp[2];
//			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}		
	}
}

