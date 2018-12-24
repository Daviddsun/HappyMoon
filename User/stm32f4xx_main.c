/******************* (C) COPYRIGHT 2015-20~~ HappyMoon **************************
 * @文件     main.c
 * @说明     程序入口
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
*********************************************************************************/
#include "Board.h"

#define START_TASK_PRIO 3						                     // 任务优先级
#define START_STK_SIZE 512						                   // 任务堆栈大小
OS_TCB StartTaskTCB;							                       // 任务控制块
CPU_STK START_TASK_STK[START_STK_SIZE];					         // 任务堆栈
void start_task(void *p_arg);						                 // 任务函数

//IMU任务
#define AHRS_TASK_PRIO 4						
#define AHRS_STK_SIZE 512						
OS_TCB AHRSTaskTCB;				
CPU_STK AHRS_TASK_STK[AHRS_STK_SIZE];					
void AHRS_task(void *p_arg);

//姿态环任务
#define Attitude_TASK_PRIO 5						
#define Attitude_STK_SIZE 512						
OS_TCB AttitudeTaskTCB;							
CPU_STK Attitude_TASK_STK[Attitude_STK_SIZE];					
void Attitude_task(void *p_arg);

//视觉惯性里程计数据接收
#define Vision_TASK_PRIO 6						
#define Vision_STK_SIZE	512						
OS_TCB VisionTaskTCB;							
CPU_STK Vision_TASK_STK[Vision_STK_SIZE];					
void Vision_task(void *p_arg);

//位置环任务
#define Position_TASK_PRIO 7						
#define Position_STK_SIZE 512						
OS_TCB PositionTaskTCB;							
CPU_STK Position_TASK_STK[Position_STK_SIZE];					
void Position_task(void *p_arg);

//DataDeal任务
#define DataDeal_TASK_PRIO 8						
#define DataDeal_STK_SIZE 512						
OS_TCB DataDealTaskTCB;							
CPU_STK DataDeal_TASK_STK[DataDeal_STK_SIZE];					
void DataDeal_task(void *p_arg);

//BluetoothtoPC任务
#define BluetoothtoPC_TASK_PRIO 9						
#define BluetoothtoPC_STK_SIZE 512						
OS_TCB BluetoothtoPCTaskTCB;							
CPU_STK BluetoothtoPC_TASK_STK[BluetoothtoPC_STK_SIZE];					
void BluetoothtoPC_task(void *p_arg);

//电池电压任务
#define Battery_TASK_PRIO 10						
#define Battery_STK_SIZE 512						
OS_TCB BatteryTaskTCB;							
CPU_STK Battery_TASK_STK[Battery_STK_SIZE];					
void Battery_task(void *p_arg);

/**
 * @Description 主函数启动操作系统
 */	
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	/** 飞控板各个硬件初始化 **/
	Board_Init();
	/** 飞控参数初始化 **/
	Load_Config();
	/** 各个传感器初始化 **/
	Sensor_Init();
	OSInit(&err);												                     // 初始化UCOSIII
	OS_CRITICAL_ENTER();										                 // 进入临界区
	OSTaskCreate(												                     // 创建开始任务
		(OS_TCB*)&StartTaskTCB,									               // 任务控制块
		(CPU_CHAR*)"start task", 								               // 任务名字
		(OS_TASK_PTR)start_task, 								               // 任务函数
		(void*)0,												                       // 传递给任务函数的参数
		(OS_PRIO)START_TASK_PRIO,								               // 任务优先级
		(CPU_STK*)&START_TASK_STK[0],							             // 任务堆栈基地址
		(CPU_STK_SIZE)START_STK_SIZE/10,						           // 任务堆栈深度限位
		(CPU_STK_SIZE)START_STK_SIZE,							             // 任务堆栈大小
		(OS_MSG_QTY)0,											                   // 任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
		(OS_TICK)0,												                     // 当使能时间片轮转时的时间片长度，为0时为默认长度，
		(void*)0,												                       // 用户补充的存储区
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,		   // 任务选项
		(OS_ERR*)&err											                     // 存放该函数错误时的返回值
		);
	OS_CRITICAL_EXIT();											                 // 退出临界区
	OSStart(&err);												                   // 开启UCOSIII
	while(1);
}

/**
 * @Description 开始任务函数
 */
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err);										  	// 统计任务
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN												// 如果使能了测量中断关闭时间
	CPU_IntDisMeasMaxCurReset();	
#endif
	//默认打开
#if OS_CFG_SCHED_ROUND_ROBIN_EN												// 当使用时间片轮转的时候
	//使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
#endif

	OS_CRITICAL_ENTER();																// 进入临界区
	OSTaskCreate(																				// 创建姿态IMU任务
		(OS_TCB*)&AHRSTaskTCB,
		(CPU_CHAR*)"AHRS task",
		(OS_TASK_PTR )AHRS_task,
		(void*)0,
		(OS_PRIO)AHRS_TASK_PRIO,
		(CPU_STK*)&AHRS_TASK_STK[0],
		(CPU_STK_SIZE)AHRS_STK_SIZE/10,
		(CPU_STK_SIZE)AHRS_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																				// 创建姿态任务
		(OS_TCB*)&AttitudeTaskTCB,
		(CPU_CHAR*)"Attitude task",
		(OS_TASK_PTR )Attitude_task,
		(void*)0,
		(OS_PRIO)Attitude_TASK_PRIO,
		(CPU_STK*)&Attitude_TASK_STK[0],
		(CPU_STK_SIZE)Attitude_STK_SIZE/10,
		(CPU_STK_SIZE)Attitude_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																				// 创建位置Position任务
		(OS_TCB*)&PositionTaskTCB,
		(CPU_CHAR*)"Position task",
		(OS_TASK_PTR )Position_task,
		(void*)0,
		(OS_PRIO)Position_TASK_PRIO,
		(CPU_STK*)&Position_TASK_STK[0],
		(CPU_STK_SIZE)Position_STK_SIZE/10,
		(CPU_STK_SIZE)Position_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																				// 创建视觉Vision任务
		(OS_TCB*)&VisionTaskTCB,
		(CPU_CHAR*)"Vision task",
		(OS_TASK_PTR )Vision_task,
		(void*)0,
		(OS_PRIO)Vision_TASK_PRIO,
		(CPU_STK*)&Vision_TASK_STK[0],
		(CPU_STK_SIZE)Vision_STK_SIZE/10, 
		(CPU_STK_SIZE)Vision_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(													    					// 创建数据处理DataDeal任务
		(OS_TCB*)&DataDealTaskTCB,
		(CPU_CHAR*)"DataDeal task",
		(OS_TASK_PTR )DataDeal_task,
		(void*)0,
		(OS_PRIO)DataDeal_TASK_PRIO,
		(CPU_STK*)&DataDeal_TASK_STK[0],
		(CPU_STK_SIZE)DataDeal_STK_SIZE/10,
		(CPU_STK_SIZE)DataDeal_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																				// 创建数据发送Usart1toPC任务
		(OS_TCB*)&BluetoothtoPCTaskTCB,
		(CPU_CHAR*)"BluetoothtoPC task",
		(OS_TASK_PTR )BluetoothtoPC_task,
		(void*)0,
		(OS_PRIO)BluetoothtoPC_TASK_PRIO,
		(CPU_STK*)&BluetoothtoPC_TASK_STK[0],
		(CPU_STK_SIZE)BluetoothtoPC_STK_SIZE/10,
		(CPU_STK_SIZE)BluetoothtoPC_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																				// 创建电池电压Battery任务
		(OS_TCB*)&BatteryTaskTCB,
		(CPU_CHAR*)"Battery task",
		(OS_TASK_PTR )Battery_task,
		(void*)0,
		(OS_PRIO)Battery_TASK_PRIO,
		(CPU_STK*)&Battery_TASK_STK[0],
		(CPU_STK_SIZE)Battery_STK_SIZE/10,
		(CPU_STK_SIZE)Battery_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);				// 挂起开始任务
	OS_CRITICAL_EXIT();																	// 离开临界区 
}



