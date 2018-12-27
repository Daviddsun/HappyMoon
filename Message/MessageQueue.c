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
	//传感器原始数据
	OSQCreate((OS_Q *)&messageQueue[ACC_SENSOR_READ],
												(CPU_CHAR *)"ACC_SENSOR_READ",(OS_MSG_QTY)24,(OS_ERR *)&p_err);
	OSQCreate((OS_Q *)&messageQueue[GYRO_SENSOR_READ],
												(CPU_CHAR *)"GYRO_SENSOR_READ",(OS_MSG_QTY)24,(OS_ERR *)&p_err);
	OSQCreate((OS_Q *)&messageQueue[TEMP_SENSOR_READ],
												(CPU_CHAR *)"TEMP_SENSOR_READ",(OS_MSG_QTY)24,(OS_ERR *)&p_err);
	//处理后的传感器数据
	OSQCreate((OS_Q *)&messageQueue[ACC_DATA_PRETREAT],
												(CPU_CHAR *)"ACC_DATA_PRETREAT",(OS_MSG_QTY)24,(OS_ERR *)&p_err);
	OSQCreate((OS_Q *)&messageQueue[GYRO_DATA_PRETREAT],
												(CPU_CHAR *)"GYRO_DATA_PRETREAT",(OS_MSG_QTY)24,(OS_ERR *)&p_err);
	OSQCreate((OS_Q *)&messageQueue[GYRO_FOR_CONTROL],
												(CPU_CHAR *)"GYRO_FOR_CONTROL",(OS_MSG_QTY)24,(OS_ERR *)&p_err);
	//视觉例程数据
	OSQCreate((OS_Q *)&messageQueue[VISUAL_ODOMETRY],
												(CPU_CHAR *)"VISUAL_ODOMETRY",(OS_MSG_QTY)100,(OS_ERR *)&p_err);
	//地面站数据
	OSQCreate((OS_Q *)&messageQueue[GROUND_STATION],
												(CPU_CHAR *)"GROUND_STATION",(OS_MSG_QTY)32,(OS_ERR *)&p_err);
}

