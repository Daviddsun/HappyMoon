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
	
	static float HeightThrustLpf;
	HeightThrustLpf = HeightThrustLpf * 0.995f + GetDesiredControlAcc() * 0.005f;
	
	if(GetCopterTest()==Drone_Mode_Pitch || GetCopterTest()==Drone_Mode_RatePitch){
		UavThrust.f1 = +1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_Acceleration * Drone_Mass / 4.0f;	
		UavThrust.f2 = -1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f3 = +1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f4 = -1.414f / (arm_length * 4.0f) * RotateThrust.y + Gravity_Acceleration * Drone_Mass / 4.0f;
	}else if(GetCopterTest()==Drone_Mode_Roll || GetCopterTest()==Drone_Mode_RateRoll){
		UavThrust.f1 = -1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f2 = -1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f3 = +1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_Acceleration * Drone_Mass / 4.0f;
		UavThrust.f4 = +1.414f / (arm_length * 4.0f) * RotateThrust.x + Gravity_Acceleration * Drone_Mass / 4.0f;
	}else if(GetCopterTest()==Drone_Mode_4Axis){
		UavThrust.f1 = -1.414f / (arm_length * 4.0f) * RotateThrust.x  																		//roll
										+1.414f / (arm_length * 4.0f) * RotateThrust.y                                  	//pitch
											+ 14.2f * RotateThrust.z                                                        //yaw	
												+ HeightThrustLpf * Drone_Mass / 4.0f;			  											  				//mass		 																									
		
		UavThrust.f2 = -1.414f / (arm_length * 4.0f) * RotateThrust.x
										-1.414f / (arm_length * 4.0f) * RotateThrust.y
											- 14.2f * RotateThrust.z
												+ HeightThrustLpf * Drone_Mass / 4.0f;									

		UavThrust.f3 = +1.414f / (arm_length * 4.0f) * RotateThrust.x
										+1.414f / (arm_length * 4.0f) * RotateThrust.y
											- 14.2f * RotateThrust.z
												+ HeightThrustLpf * Drone_Mass / 4.0f;

		UavThrust.f4 = +1.414f / (arm_length * 4.0f) * RotateThrust.x 
										-1.414f / (arm_length * 4.0f) * RotateThrust.y
											+ 14.2f * RotateThrust.z
											  + HeightThrustLpf * Drone_Mass / 4.0f;
		
	}
	MotorThrust(UavThrust.f1,UavThrust.f2,UavThrust.f3,UavThrust.f4);
}
/**********************************************************************************************************
*函 数 名: PreTakeOff
*功能说明: 预起飞函数
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void PreTakeOff(uint16_t Time){
	float ThurstValue = sqrt(Time/35) * Drone_Mass / 4.0f;
	MotorThrust(ThurstValue,ThurstValue,ThurstValue,ThurstValue);
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
	16.0V
	x=[0.519665; 4.64994; 8.246005; 11.47185; 15.2958]; %推力数据 单位N
	y=[0.05; 0.28; 0.52; 0.76; 1.00]; %油门数据 占空比
	p=polyfit(x,y,2);  % 拟合出的二次函数的系数
	ye=y-polyval(p,x);  % 计算误差
	ye2s=sum(ye.^2); % 误差的平方和
	disp(sprintf('误差的平方和=%d',ye2s));
	xx=linspace(min(x),max(x));  % 绘图用到的点的横坐标
	yy=polyval(p,xx);   % 拟合曲线的纵坐标
	plot(x,y,'o',xx,yy);  % 绘图，原始数据+拟合曲线
	
	A = 0.00044084
	B = 0.05836
	C = 0.012826
*/



void MotorThrust(float f1,float f2,float f3,float f4){
	
	float M1,M2,M3,M4;
#ifdef T_Motor
	M1 = -0.0022714f * f1 * f1 + 0.096917f * f1 + 0.11769f;
	M2 = -0.0022714f * f2 * f2 + 0.096917f * f2 + 0.11769f;
	M3 = -0.0022714f * f3 * f3 + 0.096917f * f3 + 0.11769f;
	M4 = -0.0022714f * f4 * f4 + 0.096917f * f4 + 0.11769f;
#else
	M1 = 0.00044084f * f1 * f1 + 0.05836f * f1 + 0.012826f;
	M2 = 0.00044084f * f2 * f2 + 0.05836f * f2 + 0.012826f;
	M3 = 0.00044084f * f3 * f3 + 0.05836f * f3 + 0.012826f;
	M4 = 0.00044084f * f4 * f4 + 0.05836f * f4 + 0.012826f;
#endif
	
	ThrottleInfo.M1 = (int)(M1 * 1000.0f);
	ThrottleInfo.M2 = (int)(M2 * 1000.0f);
	ThrottleInfo.M3 = (int)(M3 * 1000.0f);
	ThrottleInfo.M4 = (int)(M4 * 1000.0f);
	
	if(ThrottleInfo.M1 > 850)  ThrottleInfo.M1=850;
	if(ThrottleInfo.M2 > 850)  ThrottleInfo.M2=850;
	if(ThrottleInfo.M3 > 850)  ThrottleInfo.M3=850;
	if(ThrottleInfo.M4 > 850)  ThrottleInfo.M4=850;

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
