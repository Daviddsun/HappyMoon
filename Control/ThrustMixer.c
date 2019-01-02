#include "ThrustMixer.h"
Throttle ThrottleInfo;
ThrustUav UavThrust;
/***********************************************************************************************
*函 数 名: ThrustMixer
*功能说明: 推力融合
*形    参: 四旋翼臂长arm_length . 转轴扭矩RotateThrust
*返 回 值: 无
************************************************************************************************/
void ThrustMixer(float arm_length,Vector3f_t RotateThrust){
	
	if(GetCopterTest()==Drone_Mode_Pitch || GetCopterTest()==Drone_Mode_RatePitch){
		UavThrust.f1 = +1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_acceleration * Drone_Mass / 4.0f;	
		UavThrust.f2 = -1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_acceleration * Drone_Mass / 4.0f;
		UavThrust.f3 = +1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_acceleration * Drone_Mass / 4.0f;
		UavThrust.f4 = -1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_acceleration * Drone_Mass / 4.0f;
	}else if(GetCopterTest()==Drone_Mode_Roll || GetCopterTest()==Drone_Mode_RateRoll){
		UavThrust.f1 = -1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_acceleration * Drone_Mass / 4.0f;
		UavThrust.f2 = -1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_acceleration * Drone_Mass / 4.0f;
		UavThrust.f3 = +1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_acceleration * Drone_Mass / 4.0f;
		UavThrust.f4 = +1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_acceleration * Drone_Mass / 4.0f;
	}else if(GetCopterTest()==Drone_Mode_4Axis){
		UavThrust.f1 = -1.414f / (arm_length * 4.0f) * RotateThrust.x  																		//roll
										+1.414f / (arm_length * 4.0f) * RotateThrust.y                                  	//pitch
											+ 14.2f * RotateThrust.z                                                        //yaw	
												+ Gravity_acceleration * Drone_Mass / 4.0f;			  											  		//mass		 																									
		
		UavThrust.f2 = -1.414f / (arm_length * 4.0f) * RotateThrust.x
										-1.414f / (arm_length * 4.0f) * RotateThrust.y
											- 14.2f * RotateThrust.z
												+ Gravity_acceleration * Drone_Mass / 4.0f;									

		UavThrust.f3 = +1.414f / (arm_length * 4.0f) * RotateThrust.x
										+1.414f / (arm_length * 4.0f) * RotateThrust.y
											- 14.2f * RotateThrust.z
												+ Gravity_acceleration * Drone_Mass / 4.0f;

		UavThrust.f4 = +1.414f / (arm_length * 4.0f) * RotateThrust.x 
										-1.414f / (arm_length * 4.0f) * RotateThrust.y
											+ 14.2f * RotateThrust.z
											  + Gravity_acceleration * Drone_Mass / 4.0f;
		
	}
	MotorThrust(UavThrust.f1,UavThrust.f2,UavThrust.f3,UavThrust.f4);
}
/***********************************************************************************************
*函 数 名: MotorThrust
*功能说明: 电机推力生成
*形    参: 无
*返 回 值: 无
************************************************************************************************/
/*
															T-Motor 电机 
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

/*
															HoppyWing 电机
	%拟合二次函数

	
*/



void MotorThrust(float f1,float f2,float f3,float f4){
	
	float M1,M2,M3,M4;

	M1 = -0.0022714f * f1 * f1 + 0.096917f * f1 + 0.11769f;
	M2 = -0.0022714f * f2 * f2 + 0.096917f * f2 + 0.11769f;
	M3 = -0.0022714f * f3 * f3 + 0.096917f * f3 + 0.11769f;
	M4 = -0.0022714f * f4 * f4 + 0.096917f * f4 + 0.11769f;
	
	ThrottleInfo.M1 = (int)(M1 * 1000.0f);//+ ((int)UavThrust.collective_thrust)
	ThrottleInfo.M2 = (int)(M2 * 1000.0f);
	ThrottleInfo.M3 = (int)(M3 * 1000.0f);
	ThrottleInfo.M4 = (int)(M4 * 1000.0f);
	
	if(ThrottleInfo.M1 > 750)  ThrottleInfo.M1=750;
	if(ThrottleInfo.M2 > 750)  ThrottleInfo.M2=750;
	if(ThrottleInfo.M3 > 750)  ThrottleInfo.M3=750;
	if(ThrottleInfo.M4 > 750)  ThrottleInfo.M4=750;

	if(ThrottleInfo.M1 < 100)  ThrottleInfo.M1=100;
	if(ThrottleInfo.M2 < 100)  ThrottleInfo.M2=100;
	if(ThrottleInfo.M3 < 100)  ThrottleInfo.M3=100;
	if(ThrottleInfo.M4 < 100)  ThrottleInfo.M4=100;
	
	PWM_OUTPUT(ThrottleInfo.M1,ThrottleInfo.M2,ThrottleInfo.M3,ThrottleInfo.M4);
	
}
/***********************************************************************************************
*函 数 名: PWM_OUTPUT
*功能说明: 电调信号输出
*形    参: 电机参数
*返 回 值: 无
************************************************************************************************/
void PWM_OUTPUT(unsigned int Motor1,unsigned int Motor2,
								 unsigned int Motor3,unsigned int Motor4){
	Motor1+=1000;
	Motor2+=1000;
	Motor3+=1000;
	Motor4+=1000;
	//实际输出到电机
	TIM8->CCR1=Motor1;
	TIM8->CCR2=Motor2;
	TIM8->CCR3=Motor3;
	TIM8->CCR4=Motor4;
}
