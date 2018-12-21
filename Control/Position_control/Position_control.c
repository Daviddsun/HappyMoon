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
	if(PositionDivision){
		/************************ X轴 ************************/
		OriginalPosX.value = PID_ParaInfo.PosX.Kp * (reference_state.postionX - state_estimate.postionX);
		OriginalPosX.value = Limits_data(OriginalPosX.value,Vxy_error_max,-Vxy_error_max);//跟踪速度不超过Vxy_error_max
		/************************ Y轴 ************************/
		OriginalPosY.value = PID_ParaInfo.PosY.Kp * (reference_state.postionY - state_estimate.postionY);
		OriginalPosY.value = Limits_data(OriginalPosY.value,Vxy_error_max,-Vxy_error_max);//跟踪速度不超过Vxy_error_max
		/************************ Z轴************************/
		OriginalPosZ.value = PID_ParaInfo.PosZ.Kp * (reference_state.postionZ - state_estimate.postionZ);
		OriginalPosZ.value = Limits_data(OriginalPosZ.value,Vz_error_max,-Vz_error_max);//跟踪速度不超过Vz_error_max
	}
	PositionDivision = ~PositionDivision;

	/************************ X轴控制 ************************/
	Target_Info.Roll =  PID_Control(&PID_ParaInfo.VelX,&OriginalVelX,OriginalPosX.value,state_estimate.velocityX,0.02,5,Filter20Hz) * PI/180;	
	/************************ Y轴控制 ************************/
	Target_Info.Pitch = PID_Control(&PID_ParaInfo.VelY,&OriginalVelY,OriginalPosY.value,state_estimate.velocityY,0.02,5,Filter20Hz) * PI/180;		
	/************************ Z轴控制 ************************/
	UAVThrust.collective_thrust = PID_Control(&PID_ParaInfo.VelZ,&OriginalVelZ,OriginalPosZ.value,state_estimate.velocityZ,0.02,75,Filter20Hz);
}
	
//	Target_Info.Pitch = -RockerControl.XaxisPos * 0.04f * PI/180; 
//	
//	Target_Info.Roll = RockerControl.YaxisPos * 0.04f * PI/180;


void PositionParameterclear(void){
	
	/* 所有参数归零 */
	OriginalPosX.value = 0;
	OriginalPosY.value = 0;
	OriginalPosZ.value = 0;
	
	OriginalVelX.pOut = 0;
	OriginalVelY.pOut = 0;
	OriginalVelZ.pOut = 0;
	
	OriginalVelX.iOut = 0;
	OriginalVelY.iOut = 0;
	OriginalVelZ.iOut = 0;
	
	OriginalVelX.dOut = 0;		
	OriginalVelY.dOut = 0;
	OriginalVelZ.dOut = 0;

	OriginalVelX.differential = 0;		
	OriginalVelY.differential = 0;
	OriginalVelZ.differential = 0;
	
	OriginalVelX.differentialFliter = 0;
	OriginalVelY.differentialFliter = 0;
	OriginalVelZ.differentialFliter = 0;
	
	OriginalVelX.lasterror = 0;
	OriginalVelY.lasterror = 0;
	OriginalVelZ.lasterror = 0;
	
	OriginalVelX.value = 0;
	OriginalVelY.value = 0;
	OriginalVelZ.value = 0;
	
	reference_state.postionZ = 0.75f;
	UAVThrust.collective_thrust = 0;
	
}

