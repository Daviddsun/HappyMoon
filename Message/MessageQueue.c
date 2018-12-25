#include "MessageQueue.h"


//声明消息队列句柄
OS_Q messageQueue[QUEUE_NUM];

/**********************************************************************************************************
*函 数 名: MessageQueueCreate
*功能说明: 消息队列创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageQueueCreate(OS_ERR p_err)
{	
	OSQCreate(&messageQueue[ACC_SENSOR_READ],"ACC_SENSOR_READ",24,&p_err);
	OSQCreate(&messageQueue[GYRO_SENSOR_READ],"GYRO_SENSOR_READ",24,&p_err);
	OSQCreate(&messageQueue[TEMP_SENSOR_READ],"TEMP_SENSOR_READ",24,&p_err);
	
	OSQCreate(&messageQueue[ACC_DATA_PRETREAT],"ACC_DATA_PRETREAT",24,&p_err);
	OSQCreate(&messageQueue[GYRO_DATA_PRETREAT],"GYRO_DATA_PRETREAT",24,&p_err);
	OSQCreate(&messageQueue[GYRO_FOR_CONTROL],"GYRO_FOR_CONTROL",24,&p_err);
}
