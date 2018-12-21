#ifndef __KALMANFILTER_H
#define __KALMANFILTER_H

#include "DronePara.h"

void KalmanFilter_Init(KalmanFilter *XAXIS,KalmanFilter *YAXIS,KalmanFilter *ZAXIS);
void Kalman_Filter(KalmanFilter *KalmanFilter_Input,float Vel,float Acc);

#endif 
