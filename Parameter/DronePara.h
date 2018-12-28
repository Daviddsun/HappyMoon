#ifndef __DRONWPARA_H
#define __DRONWPARA_H

#include "stm32f4xx.h"
#include "Vector3.h"
#include <stdbool.h>
#define TX_LEN  160
#define RX_LEN  128
#define ARM_Length 0.190f
#define Drone_Mass 1.700f
#define Gravity_acceleration  9.805f
#define TorqueLimit   0.25f
#define Inertia_Wx    0.001f
#define Inertia_Wy    0.001f
#define Inertia_Wz    0.002f

typedef enum{ 
	Drone_Mode_None= 0,
  Drone_Mode_Pitch= 1,		 	
  Drone_Mode_Roll= 2, 
	Drone_Mode_4Axis= 3,	
	Drone_Mode_RatePitch= 4, 
  Drone_Mode_RateRoll= 5, 	
}DroneFlightMode_TypeDef;

typedef enum{  
	Drone_Off  = 0x00,
  Drone_On   = 0x01,
	Drone_Land = 0x02,
}DroneFlightOnOff_TypeDef;

typedef enum{  
	Report_SET      = 0x01,
  Report_RESET    = 0x00, 		 	
}DroneReportSW_TypeDef;

typedef struct{
	DroneFlightOnOff_TypeDef OnOffLand;
	DroneFlightMode_TypeDef DroneMode;
	DroneReportSW_TypeDef ReportSW;
}DroneFlightControl;

typedef struct
{
	Vector3angle_t TargetAngle;
	Vector3f_t TargetW;
	Vector3pos_t TargetPos;
}DroneTargetInfo;

typedef struct
{
    float kP;
    float kI;
    float kD;

    float imax;
    float integrator;
    float lastError;
    float lastDerivative;
    float dFilter;
} PID_t;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
} PID;

typedef struct{
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
	float gyro_offectx;
	float gyro_offecty;
	float gyro_offectz;
	float gyro_scalex;
	float gyro_scaley;
	float gyro_scalez;
	unsigned int acc_calibra_cnt;
	unsigned int gyro_calibra_cnt;
	bool acc_success;
	bool gyro_success;
}OffsetInfo;


typedef struct{
    float XaxisPos;
    float YaxisPos;
    float ZaxisPos;
    float Navigation;
}RemoteControl;

typedef struct{
	float worldX;
	float worldY;
	float worldZ;
}Desired_acceleration;

typedef struct{
	unsigned int len;
	unsigned char buf[64];
}Data_Rx;

typedef struct{
	unsigned char buf[32];
}Receive_GroundStation;

typedef struct{
	unsigned char buf[64];
}Receive_VisualOdometry;

typedef union{
	float fvalue;
	unsigned char cv[4];
}float_union;



#endif

