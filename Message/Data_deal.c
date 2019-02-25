/**********************************************************************************************************
 * @文件     Data_deal.c
 * @说明     接收PC数据
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
**********************************************************************************************************/
#include "Data_deal.h"
DroneFlightControl FlightControl;                          
DroneTargetInfo Target_Info;           
RemoteControl RockerControl;  
DroneFlightMethod Flight_Method;
Vector3f_t StepSignal;
/**********************************************************************************************************
*函 数 名: GroundStationDataDeal
*功能说明: 获取地面站信息
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GroundStationDataDeal(Receive_GroundStation rx){
	float pidParaTemp[3];
	u8 HexToFloat[4];
	u8 i,j;
	if(rx.buf[0]==0x55 && rx.buf[1]==0xAA){
		//飞行状态
		if(rx.buf[2] == 0xff){ 
			switch(rx.buf[3]){
				case 0:
					FlightControl.OnOff = Drone_Off;
					break;
				case 1:
					FlightControl.OnOff = Drone_On;
					break;
				case 2:
					FlightControl.DroneStatus = Land;
					break;
				default:
					break;
			}					
		}
		else{
			switch(rx.buf[2]){
				case 1:
					/* Target_Pitch */
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[3+i];
					}
					Target_Info.TargetAngle.pitch = Hex_To_Decimal(HexToFloat,4) * PI/180; 
					/* Target_Roll */
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[7+i];
					}
					Target_Info.TargetAngle.roll = Hex_To_Decimal(HexToFloat,4) * PI/180;								
					/* Target_Yaw */					
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[11+i];
					}
					Target_Info.TargetAngle.yaw = Hex_To_Decimal(HexToFloat,4) * PI/180; 
					break;
				/* 调试模式选择 */
				case 2:
					switch(rx.buf[3]){
						case 0:
								FlightControl.DroneMode=Drone_Mode_None;
							break;
						case 1:
								FlightControl.DroneMode=Drone_Mode_RatePitch;
							break;
						case 2:
								FlightControl.DroneMode=Drone_Mode_RateRoll;		
							break;
						case 3:
								FlightControl.DroneMode=Drone_Mode_Pitch; 
							break;
						case 4:
								FlightControl.DroneMode=Drone_Mode_Roll; 
							break;
						case 5:
								FlightControl.DroneMode=Drone_Mode_4Axis;
							break;
						default:
							break;
					}
					break;
				/* 发送PID参数 */
				case 3:
					FlightControl.ReportSW = Report_SET;
					break;
				/* 打舵角速度 */	
			  case 4:
					/* Target_RatePitch */
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[3+i];
					}
					Target_Info.TargetW.y = Hex_To_Decimal(HexToFloat,4);
					Target_Info.TargetW.y = Target_Info.TargetW.y * 0.0002f;//2rad/s最高
					/* Target_RateRoll */
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[7+i];
					}
					Target_Info.TargetW.x = Hex_To_Decimal(HexToFloat,4);	
					Target_Info.TargetW.x = Target_Info.TargetW.x * 0.0002f;//2rad/s最高
					break;
				/* 遥控器数据 */
				case 5:
					/* XaxisPos */
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[3+i];
					}
					RockerControl.Xaxis = Hex_To_Decimal(HexToFloat,4); 
					/* YaxisPos */
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[7+i];
					}
					RockerControl.Yaxis = Hex_To_Decimal(HexToFloat,4); 					
					break;
				case 6:
					/* Navigation */
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[11+i];
					}
					RockerControl.Navigation = Hex_To_Decimal(HexToFloat,4);
					/* ZaxisPos */
					for(i=0;i<4;i++){
						HexToFloat[i]=rx.buf[15+i];
					}
					RockerControl.Zaxis = Hex_To_Decimal(HexToFloat,4);
					break;
				/* 写入Pitch PID参数 */	
				case 7:
					for(i=0;i<3;i++)
					{
						for(j=0;j<4;j++)
						{
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalPitch.kP=PID_ParaInfo.Pitch.Kp=pidParaTemp[0];
					OriginalPitch.kI=PID_ParaInfo.Pitch.Ki=pidParaTemp[1];
					OriginalPitch.kD=PID_ParaInfo.Pitch.Kd=pidParaTemp[2];
					Write_Config();	
					FlightControl.ReportSW=Report_SET;
					break;
				/* 写入Roll PID参数 */	
				case 8:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}     
					OriginalRoll.kP=PID_ParaInfo.Roll.Kp=pidParaTemp[0];
					OriginalRoll.kI=PID_ParaInfo.Roll.Ki=pidParaTemp[1];
					OriginalRoll.kD=PID_ParaInfo.Roll.Kd=pidParaTemp[2];
					Write_Config();	
					FlightControl.ReportSW=Report_SET;
					break;
				/* 写入Yaw PID参数 */	
				case 9:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalYaw.kP=PID_ParaInfo.Yaw.Kp=pidParaTemp[0];
					OriginalYaw.kI=PID_ParaInfo.Yaw.Ki=pidParaTemp[1];
					OriginalYaw.kD=PID_ParaInfo.Yaw.Kd=pidParaTemp[2];
					Write_Config();	
					FlightControl.ReportSW=Report_SET;		
					break;
				/* 写入PosZ PID参数 */	
				case 10:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalPosZ.kP=PID_ParaInfo.PosZ.Kp=pidParaTemp[0];
					OriginalPosZ.kI=PID_ParaInfo.PosZ.Ki=pidParaTemp[1];
					OriginalPosZ.kD=PID_ParaInfo.PosZ.Kd=pidParaTemp[2];
					Write_Config();		
					FlightControl.ReportSW=Report_SET;	
					break;
				/* 写入Wy PID参数 */
				case 11:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalWyRate.kP=PID_ParaInfo.WyRate.Kp=pidParaTemp[0];
					OriginalWyRate.kI=PID_ParaInfo.WyRate.Ki=pidParaTemp[1];
					OriginalWyRate.kD=PID_ParaInfo.WyRate.Kd=pidParaTemp[2];
					Write_Config();		
					FlightControl.ReportSW=Report_SET;
					break;
				/* 写入Wx PID参数 */
				case 12:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalWxRate.kP=PID_ParaInfo.WxRate.Kp=pidParaTemp[0];
					OriginalWxRate.kI=PID_ParaInfo.WxRate.Ki=pidParaTemp[1];
					OriginalWxRate.kD=PID_ParaInfo.WxRate.Kd=pidParaTemp[2];
					Write_Config();		
					FlightControl.ReportSW=Report_SET;	
					break;
				/* 写入Wz PID参数 */
				case 13:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalWzRate.kP=PID_ParaInfo.WzRate.Kp=pidParaTemp[0];
					OriginalWzRate.kI=PID_ParaInfo.WzRate.Ki=pidParaTemp[1];
					OriginalWzRate.kD=PID_ParaInfo.WzRate.Kd=pidParaTemp[2];
					Write_Config();		
					FlightControl.ReportSW=Report_SET;
					break;
				/* 写入VelZ PID参数 */
				case 14:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}
					OriginalVelZ.kP=PID_ParaInfo.VelZ.Kp=pidParaTemp[0];
					OriginalVelZ.kI=PID_ParaInfo.VelZ.Ki=pidParaTemp[1];
					OriginalVelZ.kD=PID_ParaInfo.VelZ.Kd=pidParaTemp[2];
					Write_Config();
					FlightControl.ReportSW=Report_SET;
					break;	
				/* 写入AccZ PID参数 */
				case 15:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}
					FlightControl.ReportSW=Report_SET;
					break;
				/* 写入PositionX PID参数 */
				case 18:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalPosX.kP=PID_ParaInfo.PosX.Kp=pidParaTemp[0];
					OriginalPosX.kI=PID_ParaInfo.PosX.Ki=pidParaTemp[1];
					OriginalPosX.kD=PID_ParaInfo.PosX.Kd=pidParaTemp[2];
					Write_Config();	
					FlightControl.ReportSW=Report_SET;
					break;
				/* 写入PositionY PID参数 */
				case 19:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalPosY.kP=PID_ParaInfo.PosY.Kp=pidParaTemp[0];
					OriginalPosY.kI=PID_ParaInfo.PosY.Ki=pidParaTemp[1];
					OriginalPosY.kD=PID_ParaInfo.PosY.Kd=pidParaTemp[2];
					Write_Config();	
					FlightControl.ReportSW=Report_SET;
					break;
				/* 写入SpeedX PID参数 */	
				case 20:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalVelX.kP=PID_ParaInfo.VelX.Kp=pidParaTemp[0];
					OriginalVelX.kI=PID_ParaInfo.VelX.Ki=pidParaTemp[1];
					OriginalVelX.kD=PID_ParaInfo.VelX.Kd=pidParaTemp[2];
					Write_Config();	
					FlightControl.ReportSW=Report_SET;
					break;
				/* 写入SpeedY PID参数 */
				case 21:
					for(i=0;i<3;i++){
						for(j=0;j<4;j++){
							HexToFloat[j]=rx.buf[3+j+i*4];
						}
						pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
					}  
					OriginalVelY.kP=PID_ParaInfo.VelY.Kp=pidParaTemp[0];
					OriginalVelY.kI=PID_ParaInfo.VelY.Ki=pidParaTemp[1];
					OriginalVelY.kD=PID_ParaInfo.VelY.Kd=pidParaTemp[2];
					Write_Config();	
					FlightControl.ReportSW=Report_SET;
					break;
				/* 飞行模式确定 */
				case 22:
					switch(rx.buf[3]){
						case 0:
							Flight_Method = PurePosture;
							break;
						case 1:
							Flight_Method = FixedHeight;
							break;
						case 2:
							Flight_Method = FixedPoint;
							break;
						case 3:
							Flight_Method = StepResponse;
							break;
						case 4:
							Flight_Method = TrajectoryTracking;
							break;
						default:
							break;
					}	
					break;
				/* 陀螺仪校准 */
				case 48:
					OffsetData.gyro_success = true;
					break;
				/* 加速计校准 */
				case 49:
					OffsetData.acc_success = true;
					break;
				/* IMU水平校准 */
				case 50:
					OffsetData.level_success = true;	
					break;
//				/* 阶跃信号测试 */	
//				case 16:
//					for(i=0;i<4;i++)
//					{
//						HexToFloat[i]=rx.buf[3+i];
//					}
//					StepSignal.x = Hex_To_Decimal(HexToFloat,4);
//					for(i=0;i<4;i++)
//					{
//						HexToFloat[i]=rx.buf[7+i];
//					}
//					StepSignal.y = Hex_To_Decimal(HexToFloat,4); 
//					for(i=0;i<4;i++)
//					{
//						HexToFloat[i]=rx.buf[11+i];
//					}
//					StepSignal.z = Hex_To_Decimal(HexToFloat,4);
//					break;

				
				default:
					break;
			}
		}
	}
}


/**********************************************************************************************************
*函 数 名: GetCopterStatus
*功能说明: 获取飞行器状态 起飞 关机
*形    参: 无
*返 回 值: uint8_t 类型
**********************************************************************************************************/
uint8_t GetCopterStatus(void){
  return FlightControl.OnOff;
}

/**********************************************************************************************************
*函 数 名: SetCopterStatus
*功能说明: 设置飞行器状态 起飞 关机
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SetCopterStatus(DroneFlightOnOff_TypeDef status){
  FlightControl.OnOff = status;
}

/**********************************************************************************************************
*函 数 名: GetCopterFlyMode
*功能说明: 获取飞行器飞行模式
*形    参: 无
*返 回 值: uint8_t 类型
**********************************************************************************************************/
uint8_t GetCopterFlyMode(void){
  return FlightControl.DroneStatus;
}

/**********************************************************************************************************
*函 数 名: SetCopterFlyMode
*功能说明: 设置飞行模式
*形    参: 无
*返 回 值: uint8_t 类型
**********************************************************************************************************/
void SetCopterFlyMode(DroneFlightStatus_TypeDef FlyMode){
  FlightControl.DroneStatus = FlyMode;
}

/**********************************************************************************************************
*函 数 名: GetCopterTest
*功能说明: 获取飞行器调试实验
*形    参: 无
*返 回 值: uint8_t 类型
**********************************************************************************************************/
uint8_t GetCopterTest(void){
  return FlightControl.DroneMode;
}

/**********************************************************************************************************
*函 数 名: SendPID
*功能说明: 发送PID参数
*形    参: 无
*返 回 值: uint8_t 类型
**********************************************************************************************************/
uint8_t SendPID(void){
  return FlightControl.ReportSW;
}

/**********************************************************************************************************
*函 数 名: ResetSendPID
*功能说明: 置位写PID参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ResetSendPID(void){
  FlightControl.ReportSW = Report_RESET;
}

/**********************************************************************************************************
*函 数 名: GetRemoteControlAngle
*功能说明: 获取遥控器姿态信息
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
Vector3angle_t GetRemoteControlAngle(void){
	return Target_Info.TargetAngle;
}

/**********************************************************************************************************
*函 数 名: GetRemoteControlAngleVel
*功能说明: 获取遥控器角速度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
Vector3f_t GetRemoteControlAngleVel(void){
	return Target_Info.TargetW;
}

/**********************************************************************************************************
*函 数 名: GetRemoteControlFlyData
*功能说明: 获取遥控器飞行数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
RemoteControl GetRemoteControlFlyData(void){
	return RockerControl;
}

/**********************************************************************************************************
*函 数 名: GetStepSignalValue
*功能说明: 获取阶跃信号
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
Vector3f_t GetStepSignalValue(void){
	//期望高度不低于0.5m
	StepSignal.x = 0;
	StepSignal.y = 0;
	StepSignal.z = 1.0f;
	return StepSignal;
}
/**********************************************************************************************************
*函 数 名: GetCopterFlightMethod
*功能说明: 获取飞行器调试实验
*形    参: 无
*返 回 值: uint8_t 类型
**********************************************************************************************************/
uint8_t GetCopterFlightMethod(void){
  return Flight_Method;
}

/**********************************************************************************************************
*函 数 名: SetCopterFlightMethod
*功能说明: 设置飞行模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SetCopterFlightMethod(void){
  Flight_Method = FixedHeight;
}
