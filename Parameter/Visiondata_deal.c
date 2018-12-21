#include "Visiondata_deal.h"

float_union position_x,position_y,position_z,velocity_x,velocity_y,velocity_z,
								Quaternion0,Quaternion1,Quaternion2,Quaternion3,reference_posx,reference_posy,reference_posz;

void Vision_datadeal(_Data_Rx rx){
	static unsigned int tPre=0;
	unsigned int t;
	t=micros();
	RT_Info.frequency	= (tPre>0)?((t-tPre)/1000000.0f):1;
	tPre=t;
	
	if(rx.len==56 && rx.buf[0]==0x55 && rx.buf[1]==0xAA && rx.buf[55]==0xAA){
		
		if(rx.buf[2] == 0x30){
			//X轴位置数据
			position_x.cv[0] = rx.buf[3];
			position_x.cv[1] = rx.buf[4];
			position_x.cv[2] = rx.buf[5];
			position_x.cv[3] = rx.buf[6];
			//Y轴位置数据
			position_y.cv[0] = rx.buf[7];
			position_y.cv[1] = rx.buf[8];
			position_y.cv[2] = rx.buf[9];
			position_y.cv[3] = rx.buf[10];
			//Z轴位置数据
			position_z.cv[0] = rx.buf[11];
			position_z.cv[1] = rx.buf[12];
			position_z.cv[2] = rx.buf[13];
			position_z.cv[3] = rx.buf[14];
			//X轴速度数据
			velocity_x.cv[0] = rx.buf[15];
			velocity_x.cv[1] = rx.buf[16];
			velocity_x.cv[2] = rx.buf[17];
			velocity_x.cv[3] = rx.buf[18];
			//Y轴速度数据
			velocity_y.cv[0] = rx.buf[19];
			velocity_y.cv[1] = rx.buf[20];
			velocity_y.cv[2] = rx.buf[21];
			velocity_y.cv[3] = rx.buf[22];	
			//Z轴速度数据
			velocity_z.cv[0] = rx.buf[23];
			velocity_z.cv[1] = rx.buf[24];
			velocity_z.cv[2] = rx.buf[25];
			velocity_z.cv[3] = rx.buf[26];
			//视觉里程计的姿态数据
			Quaternion0.cv[0] = rx.buf[27];
			Quaternion0.cv[1] = rx.buf[28];
			Quaternion0.cv[2] = rx.buf[29];
			Quaternion0.cv[3] = rx.buf[30];
			
			Quaternion1.cv[0] = rx.buf[31];
			Quaternion1.cv[1] = rx.buf[32];
			Quaternion1.cv[2] = rx.buf[33];
			Quaternion1.cv[3] = rx.buf[34];
			
			Quaternion2.cv[0] = rx.buf[35];
			Quaternion2.cv[1] = rx.buf[36];
			Quaternion2.cv[2] = rx.buf[37];
			Quaternion2.cv[3] = rx.buf[38];
			
			Quaternion3.cv[0] = rx.buf[39];
			Quaternion3.cv[1] = rx.buf[40];
			Quaternion3.cv[2] = rx.buf[41];
			Quaternion3.cv[3] = rx.buf[42];
			//期望的位置数据
			reference_posx.cv[0] = rx.buf[43];
			reference_posx.cv[1] = rx.buf[44];
			reference_posx.cv[2] = rx.buf[45];
			reference_posx.cv[3] = rx.buf[46];

			reference_posy.cv[0] = rx.buf[47];
			reference_posy.cv[1] = rx.buf[48];
			reference_posy.cv[2] = rx.buf[49];
			reference_posy.cv[3] = rx.buf[50];

			reference_posz.cv[0] = rx.buf[51];
			reference_posz.cv[1] = rx.buf[52];
			reference_posz.cv[2] = rx.buf[53];
			reference_posz.cv[3] = rx.buf[54];
			//实际反馈数据
			state_estimate.postionX = position_x.fvalue;
			state_estimate.postionY = position_y.fvalue;
			state_estimate.postionZ = position_z.fvalue;
			state_estimate.velocityX = velocity_x.fvalue;
			state_estimate.velocityY = velocity_y.fvalue;
			state_estimate.velocityZ = velocity_z.fvalue;

			//视觉里程计 与 实际四旋翼航向角对应关系
			state_estimate.AttitudeYaw = atan2(2.0f * Quaternion1.fvalue * Quaternion2.fvalue + 2.0f * Quaternion0.fvalue * Quaternion3.fvalue,
																						-2.0f * Quaternion2.fvalue * Quaternion2.fvalue - 2.0f * Quaternion3.fvalue * Quaternion3.fvalue + 1);
		}
			

	}
}


