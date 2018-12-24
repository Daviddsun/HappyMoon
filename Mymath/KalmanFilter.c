#include "KalmanFilter.h"

const float KalmanFilter_Unit[3][6] = {

	{   0.08   ,   0.01   ,   0.01   ,   1   ,   1   ,   0.01   },
	{   0.08   ,   0.01   ,   0.01   ,   1   ,   1   ,   0.01   },
	{   0.08   ,   0.01   ,   0.01   ,   1   ,   1   ,   0.01   },
};

/*
 *
 *  KalmanFilter 
 *
 */
void KalmanFilter_Init(KalmanFilter *XAXIS,KalmanFilter *YAXIS,KalmanFilter *ZAXIS){
    XAXIS->Q_Velocity = KalmanFilter_Unit[0][0];
    XAXIS->Q_Bias = KalmanFilter_Unit[0][1];
    XAXIS->R_Vel = KalmanFilter_Unit[0][2];
    XAXIS->AxisC_0 = KalmanFilter_Unit[0][3];
    XAXIS->AxisPP[0][0] = KalmanFilter_Unit[0][4];
    XAXIS->AxisPP[1][1] = KalmanFilter_Unit[0][4];
    XAXIS->Merge_t = KalmanFilter_Unit[0][5];

    YAXIS->Q_Velocity = KalmanFilter_Unit[1][0];
    YAXIS->Q_Bias = KalmanFilter_Unit[1][1];
    YAXIS->R_Vel = KalmanFilter_Unit[1][2];
    YAXIS->AxisC_0 = KalmanFilter_Unit[1][3];
    YAXIS->AxisPP[0][0] = KalmanFilter_Unit[1][4];
    YAXIS->AxisPP[1][1] = KalmanFilter_Unit[1][4];
    YAXIS->Merge_t = KalmanFilter_Unit[1][5];

    ZAXIS->Q_Velocity = KalmanFilter_Unit[2][0];
    ZAXIS->Q_Bias = KalmanFilter_Unit[2][1];
    ZAXIS->R_Vel = KalmanFilter_Unit[2][2];
    ZAXIS->AxisC_0 = KalmanFilter_Unit[2][3];
    ZAXIS->AxisPP[0][0] = KalmanFilter_Unit[2][4];
    ZAXIS->AxisPP[1][1] = KalmanFilter_Unit[2][4];
    ZAXIS->Merge_t = KalmanFilter_Unit[2][5];
}
	
void Kalman_Filter(KalmanFilter *KalmanFilter_Input,float Vel,float Acc){
	
	KalmanFilter_Input->Axis_Vel += (Acc - KalmanFilter_Input->Axis_Bias)*KalmanFilter_Input->Merge_t;
	
	KalmanFilter_Input->AxisPdot[0] = KalmanFilter_Input->Q_Velocity - KalmanFilter_Input->AxisPP[0][1] - KalmanFilter_Input->AxisPP[1][0];
	KalmanFilter_Input->AxisPdot[1] = -KalmanFilter_Input->AxisPP[1][1];
	KalmanFilter_Input->AxisPdot[2] = -KalmanFilter_Input->AxisPP[1][1];
	KalmanFilter_Input->AxisPdot[3] = KalmanFilter_Input->Q_Bias;
	
	KalmanFilter_Input->AxisPP[0][0] += KalmanFilter_Input->AxisPdot[0]*KalmanFilter_Input->Merge_t;
	KalmanFilter_Input->AxisPP[0][1] += KalmanFilter_Input->AxisPdot[1]*KalmanFilter_Input->Merge_t;
	KalmanFilter_Input->AxisPP[1][0] += KalmanFilter_Input->AxisPdot[2]*KalmanFilter_Input->Merge_t;
	KalmanFilter_Input->AxisPP[1][1] += KalmanFilter_Input->AxisPdot[3]*KalmanFilter_Input->Merge_t;

	KalmanFilter_Input->AxisPCt_0 = KalmanFilter_Input->AxisC_0 * KalmanFilter_Input->AxisPP[0][0];
  KalmanFilter_Input->AxisPCt_1 = KalmanFilter_Input->AxisC_0 * KalmanFilter_Input->AxisPP[1][0];

	KalmanFilter_Input->AxisE = KalmanFilter_Input->R_Vel + KalmanFilter_Input->AxisC_0 * KalmanFilter_Input->AxisPCt_0;

	KalmanFilter_Input->AxisK_0 = KalmanFilter_Input->AxisPCt_0/KalmanFilter_Input->AxisE;
	KalmanFilter_Input->AxisK_1 = KalmanFilter_Input->AxisPCt_1/KalmanFilter_Input->AxisE;
    
	KalmanFilter_Input->Axis_Err = Vel - KalmanFilter_Input->Axis_Vel;
	KalmanFilter_Input->Axis_Vel += KalmanFilter_Input->AxisK_0 * KalmanFilter_Input->Axis_Err;
	KalmanFilter_Input->Q_Bias += KalmanFilter_Input->AxisK_1 * KalmanFilter_Input->Axis_Err;
	
	KalmanFilter_Input->Axist_0 = KalmanFilter_Input->AxisPCt_0;
  KalmanFilter_Input->Axist_1 = KalmanFilter_Input->AxisC_0 * KalmanFilter_Input->AxisPP[0][1];
	
	KalmanFilter_Input->AxisPP[0][0] -= KalmanFilter_Input->AxisK_0*KalmanFilter_Input->Axist_0;
	KalmanFilter_Input->AxisPP[0][1] -= KalmanFilter_Input->AxisK_0*KalmanFilter_Input->Axist_1;
	KalmanFilter_Input->AxisPP[1][0] -= KalmanFilter_Input->AxisK_1*KalmanFilter_Input->Axist_0;
	KalmanFilter_Input->AxisPP[1][1] -= KalmanFilter_Input->AxisK_1*KalmanFilter_Input->Axist_1;

}
	
	


