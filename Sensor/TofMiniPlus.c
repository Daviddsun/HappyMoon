#include "TofMiniPlus.h"
FPS_TimeOfFly FPSTimeOfFly; 
static float TOFHeightLpf,TOFHeightLpfLast;
static float TOFHeightVel;
/**********************************************************************************************************
*函 数 名: TOF_DataDeal
*功能说明: TOF传感器数据处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TOF_DataDeal(Receive_TOFData rx){
	if(rx.buf[0]==0x59 && rx.buf[1]==0x59){
		OS_ERR err;
		//计算函数运行时间间隔
		FPSTimeOfFly.CurrentTime = (OSTimeGet(&err) - FPSTimeOfFly.LastTime) * 1e-3;
		FPSTimeOfFly.CurrentTime = ConstrainFloat(FPSTimeOfFly.CurrentTime, 0.005, 0.05);
		FPSTimeOfFly.LastTime = OSTimeGet(&err);
		uint16_t Distance = (((uint16_t)rx.buf[3]) << 8) + rx.buf[2];
		uint16_t Strength = (((uint16_t)rx.buf[5]) << 8) + rx.buf[4];
		//根据数据手册强度在这个范围内有效
		if(Strength>200 && Strength<65535){
			float Height = ((float)Distance) * 0.01f; // 米制单位
			TOFHeightLpf = TOFHeightLpf * 0.92f + Height * 0.08f;
			//速度计算
			TOFHeightVel = (TOFHeightLpf - TOFHeightLpfLast) * 100.0f;
			TOFHeightLpfLast = TOFHeightLpf;
		}
	}
}
/**********************************************************************************************************
*函 数 名: GetTofHeightData
*功能说明: 获得Tof的高度数据
*形    参: 无
*返 回 值: TOFHeightLpf
**********************************************************************************************************/
float GetTofHeightData(void){
	return TOFHeightLpf;
}
/**********************************************************************************************************
*函 数 名: GetTofHeightVel
*功能说明: 获得Tof的高度速度
*形    参: 无
*返 回 值: TOFHeightVel
**********************************************************************************************************/
float GetTofHeightVel(void){
	return TOFHeightVel;
}

/**********************************************************************************************************
*函 数 名: GetFPSTimeOFfly
*功能说明: 返回视觉里程计的FPS
*形    参: 无
*返 回 值: CurrentTime
**********************************************************************************************************/
float GetFPSTimeOFfly(void){
  return FPSTimeOfFly.CurrentTime;
}

