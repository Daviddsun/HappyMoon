#include "Position_control.h"
bool PositionDivision = true;
void Position_control(void){
	/************************ 降落设置 ************************/
	if(FlightControl.landFlag==1){
		reference_state.postionZ = reference_state.postionZ - 0.005f;
		if(state_estimate.postionZ < 0.05f){
			FlightControl.OnOff = Drone_Off;
      FlightControl.landFlag = 0;
		}
	}
//	if(PositionDivision){
//		/************************ X轴 ************************/
//		OriginalPosX.value = PID_ParaInfo.PosX.Kp * (reference_state.postionX - state_estimate.postionX);
//		/************************ Y轴 ************************/
//		OriginalPosY.value = PID_ParaInfo.PosY.Kp * (reference_state.postionY - state_estimate.postionY);
//		/************************ Z轴************************/
//		OriginalPosZ.value = PID_ParaInfo.PosZ.Kp * (reference_state.postionZ - state_estimate.postionZ);
//	}
//	PositionDivision = ~PositionDivision;

//	/************************ X轴控制 ************************/
//	Target_Info.Roll =  PID_Control(&PID_ParaInfo.VelX,&OriginalVelX,OriginalPosX.value,state_estimate.velocityX,0.02,5,Filter20Hz) * PI/180;	
//	/************************ Y轴控制 ************************/
//	Target_Info.Pitch = PID_Control(&PID_ParaInfo.VelY,&OriginalVelY,OriginalPosY.value,state_estimate.velocityY,0.02,5,Filter20Hz) * PI/180;		
//	/************************ Z轴控制 ************************/
//	UAVThrust.collective_thrust = PID_Control(&PID_ParaInfo.VelZ,&OriginalVelZ,OriginalPosZ.value,state_estimate.velocityZ,0.02,75,Filter20Hz);
}
	
//	Target_Info.Pitch = -RockerControl.XaxisPos * 0.04f * PI/180; 
//	
//	Target_Info.Roll = RockerControl.YaxisPos * 0.04f * PI/180;


