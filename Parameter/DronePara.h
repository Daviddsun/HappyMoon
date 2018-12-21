#ifndef __DRONWPARA_H
#define __DRONWPARA_H

#include "stm32f4xx.h"
#include <stdbool.h>
#define TX_LEN  160
#define RX_LEN  64
#define ARM_Length 0.190f
#define Drone_Mass 1.700f
#define Gravity_acceleration  9.805f
#define TorqueLimit   0.25f
#define Inertia_Wx    0.001f
#define Inertia_Wy    0.001f
#define Inertia_Wz    0.002f

#define Vxy_error_max  0.75f 
#define Vz_error_max  0.75f

#define  kAlmostZeroValueThreshold  0.001f
#define  kAlmostZeroThrustThreshold  0.01f

// f_cut = 1/(2*PI*cutoff_freq)
// f_cut = 2 Hz -> _filter = 79.5774e-3
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3

#define Filter20Hz  7.9577e-3f
#define Filter2Hz  	79.5774e-3f

typedef enum
{ 
	Drone_Mode_None= 0,
  Drone_Mode_Pitch= 1,		 	
  Drone_Mode_Roll= 2, 
	Drone_Mode_4Axis= 3,	
	Drone_Mode_RatePitch= 4, 
  Drone_Mode_RateRoll= 5, 	
}DroneFlightMode_TypeDef;

typedef enum
{  
	Drone_Off  = 0x00,
  Drone_On   = 0x01,
}DroneFlightOnOff_TypeDef;

typedef enum
{  
	Report_SET      = 0x01,
  Report_RESET    = 0x00, 		 	
}DroneReportSW_TypeDef;

typedef struct
{
	DroneFlightOnOff_TypeDef OnOff;
	DroneFlightMode_TypeDef droneMode;
	DroneReportSW_TypeDef ReportSW;
	int landFlag;
}DroneFlightControl;

typedef enum
{
	Data_Headmode = 0,
	Data_Headfree = 1,
	Data_Point = 2,
	Data_Flow = 3,
	Data_VIO = 4,
}Data_Combine;


typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
	float RateRoll;
	float RatePitch;
	float RateYaw;
	float Height;
	float VelHeight;
	float AccHeight;
}DroneTargetInfo;

typedef struct
{
	float error;
	float lasterror;
	float differential;
	float differentialFliter;
	float pOut;
  float iOut;
	float dOut;
	float value;
	
}PIDOut;

typedef struct
{
	float Kp;
	float Ki;
	float	Kd;
}PID;

typedef struct
{
	
	PID Pitch;
	PID Roll;
	PID Yaw;
	
	PID WxRate;
	PID WyRate;
	PID WzRate;
	
  PID PosX;
	PID PosY;
	PID PosZ;
	
	PID VelX;
	PID VelY;
	PID VelZ;
	
}PIDPara;



typedef struct{
	float acc_offectx;
	float acc_offecty;
	float acc_offectz;
	float acc_scalex;
	float acc_scaley;
	float acc_scalez;
	unsigned int calibra_cnt;
	bool success;
}OffsetInfo;

//飞控放置方向
enum ORIENTATION_STATUS
{
    ORIENTATION_UP,
    ORIENTATION_DOWN,
    ORIENTATION_LEFT,
    ORIENTATION_RIGHT,
    ORIENTATION_FRONT,
    ORIENTATION_BACK,
};
//放置状态
enum
{
    STATIC,		            //静止
    MOTIONAL			    		//运动
};

typedef struct{
    float XaxisPos;
    float YaxisPos;
    float ZaxisPos;
    float Navigation;
}RemoteControl;

typedef struct
{
	float Pitch; 					
	float Roll;						
  float Yaw;						
  float	wx;							
	float wy;							
	float wz;							
	float accXaxis;       
	float accYaxis;				
	float accZaxis;	
	float Calibra_accXaxis;
	float Calibra_accYaxis;
	float Calibra_accZaxis;
	float WorldAccXaxis;  
	float WorldAccYaxis;  
	float WorldAccZaxis;  
	float batteryVoltage; 
	float frequency; 
	int lowPowerFlag;		   
}DroneRTInfo;

typedef struct{
	float AttitudePitch; 					
	float AttitudeRoll;						
  float AttitudeYaw;
	float postionX;
	float postionY;
	float postionZ;
	float velocityX;
	float velocityY;
	float velocityZ;
}State_estimate;

typedef struct{
	float AttitudePitch; 					
	float AttitudeRoll;						
  float AttitudeYaw;
	float postionX;
	float postionY;
	float postionZ;
	float velocityX;
	float velocityY;
	float velocityZ;
	float AccelerationX;
	float AccelerationY;
	float AccelerationZ;
	float Heading;
}Reference_state;

typedef struct{
	float worldX;
	float worldY;
	float worldZ;
}Desired_acceleration;

typedef struct{
	float qw;
	float qx;
	float qy;
	float qz;
}Quaternion;

typedef struct{
	float wxThrust;
	float wyThrust;
	float wzThrust;
	float HeightThrust;
	float BasicThrust;
	
	float body_torquesX;
	float body_torquesY;
	float body_torquesZ;	
	float collective_thrust;
	
	float f1;
	float f2;
	float f3;
	float f4;
}Thrust;

typedef struct
{
	int M1;
	int M2;
	int M3;
	int M4;
}Throttle;


typedef struct {
	u32 isGood;
	PIDPara pidPara;
	OffsetInfo Offset_Data;
}FlashData;

typedef struct
{
	unsigned int len;
	unsigned char buf[64];
}_Data_Rx;

typedef union{
	float fvalue;
	unsigned char cv[4];
}float_union;

typedef struct{
	float x;
	float y;
	float z;
}Vector3f_t;


#endif

