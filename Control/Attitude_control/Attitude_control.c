#include "Attitude_control.h"
bool AttitueDivision = true;
void Attitude_control(void){	
	
	if(AttitueDivision){
		//角度误差
		/* 来自飞控本身 */
		float rollErro = (Target_Info.Roll - RT_Info.Roll);
		OriginalRoll.value = PID_ParaInfo.Roll.Kp * rollErro;
		/* 来自飞控本身 */
		float pitchErro = (Target_Info.Pitch - RT_Info.Pitch);
		OriginalPitch.value = PID_ParaInfo.Pitch.Kp * pitchErro;
		/* 来自视觉里程计 */
		float yawErro = (Target_Info.Yaw - state_estimate.AttitudeYaw);
		OriginalYaw.value = PID_ParaInfo.Yaw.Kp * yawErro;
	}
	AttitueDivision = ~AttitueDivision;
	

	if(FlightControl.droneMode!=Drone_Mode_RateRoll)
	{
			UAVThrust.wxThrust = PID_Control(&PID_ParaInfo.WxRate,&OriginalWxRate,OriginalRoll.value,RT_Info.wx,0.005,TorqueLimit,Filter20Hz)
																		+ RT_Info.wy * (Inertia_Wz * RT_Info.wz) - (Inertia_Wy * RT_Info.wy) * RT_Info.wz;//考虑轴间耦合现象																																																													
	}
	else
	{
			UAVThrust.wxThrust = PID_Control(&PID_ParaInfo.WxRate,&OriginalWxRate,Target_Info.RateRoll/5000,RT_Info.wx,0.005,TorqueLimit,Filter20Hz)
																		+ RT_Info.wy * (Inertia_Wz * RT_Info.wz) - (Inertia_Wy * RT_Info.wy) * RT_Info.wz;
	}
	

	if(FlightControl.droneMode!=Drone_Mode_RatePitch)
	{
			UAVThrust.wyThrust = PID_Control(&PID_ParaInfo.WyRate,&OriginalWyRate,OriginalPitch.value,RT_Info.wy,0.005,TorqueLimit,Filter20Hz)
																		+ (-(RT_Info.wx * (Inertia_Wz * RT_Info.wz) - (Inertia_Wx * RT_Info.wx) * RT_Info.wz));
	}
	else
	{
			UAVThrust.wyThrust = PID_Control(&PID_ParaInfo.WyRate,&OriginalWyRate,Target_Info.RatePitch/5000,RT_Info.wy,0.005,TorqueLimit,Filter20Hz)
																		+ (-(RT_Info.wx * (Inertia_Wz * RT_Info.wz) - (Inertia_Wx * RT_Info.wx) * RT_Info.wz));
	}
	
	UAVThrust.wzThrust = PID_Control(&PID_ParaInfo.WzRate,&OriginalWzRate,OriginalYaw.value,RT_Info.wz,0.005,TorqueLimit,Filter20Hz)
																		+ RT_Info.wx * (Inertia_Wy * RT_Info.wy) - (Inertia_Wx * RT_Info.wx) * RT_Info.wy;
	
	
	ThrustMixer(ARM_Length);
}

void Safety_Protection(void){
	if(RT_Info.Pitch * 180/PI > 30.0f || RT_Info.Pitch * 180/PI < -30.0f || RT_Info.Roll * 180/PI > 30.0f || RT_Info.Roll * 180/PI < -30.0f){
		PWM_OUTPUT(0,0,0,0);
		FlightControl.OnOff = Drone_Off;
	}
}



void ThrustMixer(float arm_length){
	
	if(FlightControl.droneMode==Drone_Mode_Pitch || FlightControl.droneMode==Drone_Mode_RatePitch){
		UAVThrust.f1 = +1.414f / (arm_length * 4.0f) * UAVThrust.wyThrust + Gravity_acceleration * Drone_Mass / 4.0f;	
		UAVThrust.f2 = -1.414f / (arm_length * 4.0f) * UAVThrust.wyThrust + Gravity_acceleration * Drone_Mass / 4.0f;
		UAVThrust.f3 = +1.414f / (arm_length * 4.0f) * UAVThrust.wyThrust + Gravity_acceleration * Drone_Mass / 4.0f;
		UAVThrust.f4 = -1.414f / (arm_length * 4.0f) * UAVThrust.wyThrust + Gravity_acceleration * Drone_Mass / 4.0f;
	}
	else if(FlightControl.droneMode==Drone_Mode_Roll || FlightControl.droneMode==Drone_Mode_RateRoll){
		UAVThrust.f1 = -1.414f / (arm_length * 4.0f) * UAVThrust.wxThrust + Gravity_acceleration * Drone_Mass / 4.0f;
		UAVThrust.f2 = -1.414f / (arm_length * 4.0f) * UAVThrust.wxThrust + Gravity_acceleration * Drone_Mass / 4.0f;
		UAVThrust.f3 = +1.414f / (arm_length * 4.0f) * UAVThrust.wxThrust + Gravity_acceleration * Drone_Mass / 4.0f;
		UAVThrust.f4 = +1.414f / (arm_length * 4.0f) * UAVThrust.wxThrust + Gravity_acceleration * Drone_Mass / 4.0f;
	}
	else if(FlightControl.droneMode==Drone_Mode_4Axis){
		UAVThrust.f1 = -1.414f / (arm_length * 4.0f) * UAVThrust.wxThrust  																		//roll
										+1.414f / (arm_length * 4.0f) * UAVThrust.wyThrust                                  	//pitch
											+ 14.2f * UAVThrust.wzThrust                                                        //yaw	
												+ Gravity_acceleration * Drone_Mass / 4.0f;			  											  				//mass		 																									
		
		UAVThrust.f2 = -1.414f / (arm_length * 4.0f) * UAVThrust.wxThrust
										-1.414f / (arm_length * 4.0f) * UAVThrust.wyThrust
											- 14.2f * UAVThrust.wzThrust
												+ Gravity_acceleration * Drone_Mass / 4.0f;									

		UAVThrust.f3 = +1.414f / (arm_length * 4.0f) * UAVThrust.wxThrust
										+1.414f / (arm_length * 4.0f) * UAVThrust.wyThrust
											- 14.2f * UAVThrust.wzThrust
												+ Gravity_acceleration * Drone_Mass / 4.0f;

		UAVThrust.f4 = +1.414f / (arm_length * 4.0f) * UAVThrust.wxThrust 
										-1.414f / (arm_length * 4.0f) * UAVThrust.wyThrust
											+ 14.2f * UAVThrust.wzThrust
											  + Gravity_acceleration * Drone_Mass / 4.0f;
		
	}
	
	MotorThrust(UAVThrust.f1,UAVThrust.f2,UAVThrust.f3,UAVThrust.f4);
}

/*
%拟合一次函数

%     Linear model Poly1:
%     f(x) = p1*x + p2
%     Coefficients (with 95% confidence bounds):
%       p1 =    0.06925  (0.06455, 0.07396)
%       p2 =    0.1663  (0.1344, 0.1983)
	x=[0.3511; 0.6190; 0.9509; 1.2869; 1.6834; 2.2164; 2.7569; 3.3862; 4.1674; 5.0020; 5.8659; 6.7348; 7.5752; 8.4073; 9.2637; 10.0283; 10.8768; 11.7583; 12.3326]; %推力数据 单位N
	y=[0.1000; 0.1500; 0.2000; 0.2500; 0.3000; 0.3500; 0.4000; 0.4500; 0.5000; 0.5500; 0.6000; 0.6500; 0.7000; 0.7500; 0.8000; 0.85000; 0.90000; 0.95000; 1.00000]; %油门数据 占空比
	p=fittype('poly1') 
	f=fit(x,y,p) 
	plot(f,x,y);
*/

/*
%拟合二次函数
	16.4V
	x=[0.3511; 0.6190; 0.9509; 1.2869; 1.6834; 2.2164; 2.7569; 3.3862; 4.1674; 5.0020; 5.8659; 6.7348; 7.5752; 8.4073; 9.2637; 10.0283; 10.8768; 11.7583; 12.3326]; 
	y=[0.1000; 0.1500; 0.2000; 0.2500; 0.3000; 0.3500; 0.4000; 0.4500; 0.5000; 0.5500; 0.6000; 0.6500; 0.7000; 0.7500; 0.8000; 0.85000; 0.90000; 0.95000; 1.00000]; 
	p=polyfit(x,y,2); 
	ye=y-polyval(p,x);
	ye2s=sum(ye.^2);
	xx=linspace(min(x),max(x));
	yy=polyval(p,xx); 
	plot(x,y,'o',xx,yy); 
	
	A = -0.0022714
	B = 0.096917
	C = 0.11769
	
*/


void MotorThrust(float f1,float f2,float f3,float f4){
	
	float M1,M2,M3,M4;
//	M1 = 0.06925f * f1 + 0.1663f;
//	M2 = 0.06925f * f2 + 0.1663f;
//	M3 = 0.06925f * f3 + 0.1663f;
//	M4 = 0.06925f * f4 + 0.1663f;
	M1 = -0.0022714f * f1 * f1 + 0.096917f * f1 + 0.11769f;
	M2 = -0.0022714f * f2 * f2 + 0.096917f * f2 + 0.11769f;
	M3 = -0.0022714f * f3 * f3 + 0.096917f * f3 + 0.11769f;
	M4 = -0.0022714f * f4 * f4 + 0.096917f * f4 + 0.11769f;
	
	Throttle_Info.M1 = (int)(M1 * 1000.0f)+ ((int)UAVThrust.collective_thrust);
	Throttle_Info.M2 = (int)(M2 * 1000.0f)+ ((int)UAVThrust.collective_thrust);
	Throttle_Info.M3 = (int)(M3 * 1000.0f)+ ((int)UAVThrust.collective_thrust);
	Throttle_Info.M4 = (int)(M4 * 1000.0f)+ ((int)UAVThrust.collective_thrust);
	
	if(Throttle_Info.M1 > 750)  Throttle_Info.M1=750;
	if(Throttle_Info.M2 > 750)  Throttle_Info.M2=750;
	if(Throttle_Info.M3 > 750)  Throttle_Info.M3=750;
	if(Throttle_Info.M4 > 750)  Throttle_Info.M4=750;

	if(Throttle_Info.M1 < 100)  Throttle_Info.M1=100;
	if(Throttle_Info.M2 < 100)  Throttle_Info.M2=100;
	if(Throttle_Info.M3 < 100)  Throttle_Info.M3=100;
	if(Throttle_Info.M4 < 100)  Throttle_Info.M4=100;
	
	PWM_OUTPUT(Throttle_Info.M1,Throttle_Info.M2,Throttle_Info.M3,Throttle_Info.M4);
	
}


void PWM_OUTPUT( unsigned int Motor1,
								 unsigned int Motor2,
								 unsigned int Motor3,
								 unsigned int Motor4)
{
	Motor1+=1000;
	Motor2+=1000;
	Motor3+=1000;
	Motor4+=1000;

	if(RT_Info.lowPowerFlag==1)
	{
		TIM8->CCR1=1000;
		TIM8->CCR2=1000;
		TIM8->CCR3=1000;
		TIM8->CCR4=1000;
	}
	else
	{
		TIM8->CCR1=Motor1;
		TIM8->CCR2=Motor2;
		TIM8->CCR3=Motor3;
		TIM8->CCR4=Motor4;
	}
}

