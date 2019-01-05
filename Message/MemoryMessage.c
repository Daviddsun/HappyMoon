#include "MemoryMessage.h"
//创建出内存
OS_MEM memoryInfo[MEM_NUM];
Vector3f_t accRawData,gyroRawData,accCalibData,
						gyroCalibData,gyroCalibDataLpf,Vel_stateSlidWindow,Pos_stateSlidWindow;


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
	//视觉里程计数据
	OSQCreate((OS_Q *)&messageQueue[VISUAL_ODOMETRY],
												(CPU_CHAR *)"VISUAL_ODOMETRY",(OS_MSG_QTY)100,(OS_ERR *)&p_err);
	//地面站数据
	OSQCreate((OS_Q *)&messageQueue[GROUND_STATION],
												(CPU_CHAR *)"GROUND_STATION",(OS_MSG_QTY)36,(OS_ERR *)&p_err);
}

/**********************************************************************************************************
*函 数 名: MemoryCreate
*功能说明: 消息队列创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MemoryCreate(OS_ERR p_err)
{	 
	// 传感器原始参数内存分配
	OSMemCreate((OS_MEM     *)&memoryInfo[ACC_SENSOR_RAW],(CPU_CHAR *)"GYRO_SENSOR_RAW",(void *)&accRawData,//内存分区起始地址
								(OS_MEM_QTY   )2,								//内存分区里的内存块数量
										(OS_MEM_SIZE  )24,					//每个内存块的大小(字节)
												(OS_ERR      *)&p_err);
  OSMemCreate((OS_MEM     *)&memoryInfo[GYRO_SENSOR_RAW],(CPU_CHAR *)"GYRO_SENSOR_RAW",(void *)&gyroRawData,//内存分区起始地址
								(OS_MEM_QTY   )2,								//内存分区里的内存块数量
										(OS_MEM_SIZE  )24,					//每个内存块的大小(字节)
												(OS_ERR      *)&p_err);
	// 传感器处理后的数据内存分配
	OSMemCreate((OS_MEM     *)&memoryInfo[ACC_SENSOR_PRETREAT],(CPU_CHAR *)"ACC_SENSOR_PRETREAT",(void *)&accCalibData,//内存分区起始地址
								(OS_MEM_QTY   )2,								//内存分区里的内存块数量
										(OS_MEM_SIZE  )24,					//每个内存块的大小(字节)
												(OS_ERR      *)&p_err);
  OSMemCreate((OS_MEM     *)&memoryInfo[GYRO_SENSOR_PRETREAT],(CPU_CHAR *)"GYRO_SENSOR_PRETREAT",(void *)&gyroCalibData,//内存分区起始地址
								(OS_MEM_QTY   )2,								//内存分区里的内存块数量
										(OS_MEM_SIZE  )24,					//每个内存块的大小(字节)
												(OS_ERR      *)&p_err);
  OSMemCreate((OS_MEM     *)&memoryInfo[GYRO_SENSOR_LPF],(CPU_CHAR *)"GYRO_SENSOR_LPF",(void *)&gyroCalibDataLpf,//内存分区起始地址
								(OS_MEM_QTY   )2,								//内存分区里的内存块数量
										(OS_MEM_SIZE  )24,					//每个内存块的大小(字节)
												(OS_ERR      *)&p_err);	
//	// 滑动窗口内存分配
//  OSMemCreate((OS_MEM     *)&memoryInfo[VEL_STATE_SlidWindow],(CPU_CHAR *)"VEL_STATE_SlidWindow",(void *)&Vel_stateSlidWindow,//内存分区起始地址
//								(OS_MEM_QTY   )2,								//内存分区里的内存块数量
//										(OS_MEM_SIZE  )1500,					//每个内存块的大小(字节)
//												(OS_ERR      *)&p_err);	
//  OSMemCreate((OS_MEM     *)&memoryInfo[POS_STATE_SlidWindow],(CPU_CHAR *)"POS_STATE_SlidWindow",(void *)&Pos_stateSlidWindow,//内存分区起始地址
//								(OS_MEM_QTY   )2,								//内存分区里的内存块数量
//										(OS_MEM_SIZE  )1500,					//每个内存块的大小(字节)
//												(OS_ERR      *)&p_err);	
}



