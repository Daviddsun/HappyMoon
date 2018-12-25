/******************* (C) COPYRIGHT 2015-20~~ HappyMoon **************************
 * @文件     main.c
 * @说明     程序入口
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
*********************************************************************************/
#include "Board.h"

#define START_TASK_PRIO 3						                     // 任务优先级
#define START_STK_SIZE 256						                   // 任务堆栈大小
OS_TCB StartTaskTCB;							                       // 任务控制块
CPU_STK START_TASK_STK[START_STK_SIZE];					         // 任务堆栈
void start_task(void *p_arg);						                 // 任务函数

//SensorRead任务
#define SensorRead_TASK_PRIO 4						
#define SensorRead_STK_SIZE 512						
OS_TCB SensorReadTaskTCB;				
CPU_STK SensorRead_TASK_STK[SensorRead_STK_SIZE];					
void SensorRead_task(void *p_arg);

//Navigation任务
#define Navigation_TASK_PRIO 5						
#define Navigation_STK_SIZE 512						
OS_TCB NavigationTaskTCB;				
CPU_STK Navigation_TASK_STK[Navigation_STK_SIZE];					
void Navigation_task(void *p_arg);

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
	/** 启动操作系统 **/
	OSInit(&err);	
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
	//使能时间片轮转调度功能,时间片长度为1个系统时钟节拍
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
#endif

	OS_CRITICAL_ENTER();																// 进入临界区
	/** 创建一个消息队列 **/
	MessageQueueCreate(err);
	
	OSTaskCreate(																				// 传感器数据读取任务
		(OS_TCB*)&SensorReadTaskTCB,
		(CPU_CHAR*)"SensorRead task",
		(OS_TASK_PTR )SensorRead_task,
		(void*)0,
		(OS_PRIO)SensorRead_TASK_PRIO,
		(CPU_STK*)&SensorRead_TASK_STK[0],
		(CPU_STK_SIZE)SensorRead_STK_SIZE/10,
		(CPU_STK_SIZE)SensorRead_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																				// 导航任务读取任务
		(OS_TCB*)&NavigationTaskTCB,
		(CPU_CHAR*)"Navigation task",
		(OS_TASK_PTR )Navigation_task,
		(void*)0,
		(OS_PRIO)Navigation_TASK_PRIO,
		(CPU_STK*)&Navigation_TASK_STK[0],
		(CPU_STK_SIZE)Navigation_STK_SIZE/10,
		(CPU_STK_SIZE)Navigation_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);				// 挂起开始任务
	OS_CRITICAL_EXIT();																	// 离开临界区 
}



