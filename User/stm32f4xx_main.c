/******************* (C) COPYRIGHT 2015-20~~ HappyMoon **************************
 * @文件     main.c
 * @说明     程序入口
 * @作者     YuyingJin
 * @网站     https://yuyingjin0111.github.io/
 * @日期     2018 ~
*********************************************************************************/
#include "Board.h"

//Start任务
#define START_TASK_PRIO 3						                     // 任务优先级
#define START_STK_SIZE 512						                   // 任务堆栈大小
OS_TCB StartTaskTCB;							                       // 任务控制块
CPU_STK START_TASK_STK[START_STK_SIZE];					         // 任务堆栈
void start_task(void *p_arg);						                 // 任务函数

//IMU数据读取任务
#define IMUSensorRead_TASK_PRIO 4						
#define IMUSensorRead_STK_SIZE 512						
OS_TCB IMUSensorReadTaskTCB;				
CPU_STK IMUSensorRead_TASK_STK[IMUSensorRead_STK_SIZE];					
void IMUSensorRead_task(void *p_arg);

//IMU预处理任务
#define IMUSensorPreDeal_TASK_PRIO 5					
#define IMUSensorPreDeal_STK_SIZE 512						
OS_TCB IMUSensorPreDealTaskTCB;				
CPU_STK IMUSensorPreDeal_TASK_STK[IMUSensorPreDeal_STK_SIZE];					
void IMUSensorPreDeal_task(void *p_arg);

//导航任务
#define Navigation_TASK_PRIO 6						
#define Navigation_STK_SIZE 1024						
OS_TCB NavigationTaskTCB;				
CPU_STK Navigation_TASK_STK[Navigation_STK_SIZE];					
void Navigation_task(void *p_arg);

//飞行控制任务
#define FlightControl_TASK_PRIO 7						
#define FlightControl_STK_SIZE 1024						
OS_TCB FlightControlTaskTCB;				
CPU_STK FlightControl_TASK_STK[FlightControl_STK_SIZE];					
void FlightControl_task(void *p_arg);

//视觉里程计数据处理
#define VisualOdometry_TASK_PRIO 8						
#define VisualOdometry_STK_SIZE 1024						
OS_TCB VisualOdometryTaskTCB;				
CPU_STK VisualOdometry_TASK_STK[VisualOdometry_STK_SIZE];					
void VisualOdometry_task(void *p_arg);

//地面站数据处理
#define GroundStation_TASK_PRIO 9						
#define GroundStation_STK_SIZE 1024				
OS_TCB GroundStationTaskTCB;				
CPU_STK GroundStation_TASK_STK[GroundStation_STK_SIZE];					
void GroundStation_task(void *p_arg);

//其他传感器数据更新任务
#define OtherSensorUpdate_TASK_PRIO 10						
#define OtherSensorUpdate_STK_SIZE 512						
OS_TCB OtherSensorUpdateTaskTCB;				
CPU_STK OtherSensorUpdate_TASK_STK[OtherSensorUpdate_STK_SIZE];					
void OtherSensorUpdate_task(void *p_arg);

//飞行状态任务
#define FlightStatus_TASK_PRIO 11						
#define FlightStatus_STK_SIZE 512						
OS_TCB FlightStatusTaskTCB;				
CPU_STK FlightStatus_TASK_STK[FlightStatus_STK_SIZE];					
void FlightStatus_task(void *p_arg);

//Message任务
#define Message_TASK_PRIO 12						
#define Message_STK_SIZE 1024						
OS_TCB MessageTaskTCB;				
CPU_STK Message_TASK_STK[Message_STK_SIZE];					
void Message_task(void *p_arg);

/**
 * @Description 主函数启动操作系统 
 */	
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	/** 飞控板各个硬件初始化 **/
	Board_Init();
	/** IMU传感器初始化 **/
	Sensor_Init();
//	/** 电机MotorPWM初始化 **/
//	MotorPWM_Init();
	/** 启动操作系统 **/
	OSInit(&err);	
	/** 创建一个消息队列 **/
	MemoryCreate(err);
	MessageQueueCreate(err);
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
void start_task(void *p_arg){
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
	/** 创建用户任务 **/
	OSTaskCreate(																				// 传感器数据读取任务
		(OS_TCB*)&IMUSensorReadTaskTCB,
		(CPU_CHAR*)"IMUSensorRead task",
		(OS_TASK_PTR )IMUSensorRead_task,
		(void*)0,
		(OS_PRIO)IMUSensorRead_TASK_PRIO,
		(CPU_STK*)&IMUSensorRead_TASK_STK[0],
		(CPU_STK_SIZE)IMUSensorRead_STK_SIZE/10,
		(CPU_STK_SIZE)IMUSensorRead_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OSTaskCreate(																				// 传感器数据预处理
		(OS_TCB*)&IMUSensorPreDealTaskTCB,
		(CPU_CHAR*)"IMUSensorPreDeal task",
		(OS_TASK_PTR )IMUSensorPreDeal_task,
		(void*)0,
		(OS_PRIO)IMUSensorPreDeal_TASK_PRIO,
		(CPU_STK*)&IMUSensorPreDeal_TASK_STK[0],
		(CPU_STK_SIZE)IMUSensorPreDeal_STK_SIZE/10,
		(CPU_STK_SIZE)IMUSensorPreDeal_STK_SIZE,
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
	OSTaskCreate(																				// 飞行控制任务
		(OS_TCB*)&FlightControlTaskTCB,
		(CPU_CHAR*)"FlightControl task",
		(OS_TASK_PTR )FlightControl_task,
		(void*)0,
		(OS_PRIO)FlightControl_TASK_PRIO,
		(CPU_STK*)&FlightControl_TASK_STK[0],
		(CPU_STK_SIZE)FlightControl_STK_SIZE/10,
		(CPU_STK_SIZE)FlightControl_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);		
	OSTaskCreate(																				// 视觉里程计任务
		(OS_TCB*)&VisualOdometryTaskTCB,
		(CPU_CHAR*)"VisualOdometry task",
		(OS_TASK_PTR )VisualOdometry_task,
		(void*)0,
		(OS_PRIO)VisualOdometry_TASK_PRIO,
		(CPU_STK*)&VisualOdometry_TASK_STK[0],
		(CPU_STK_SIZE)VisualOdometry_STK_SIZE/10,
		(CPU_STK_SIZE)VisualOdometry_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);	
	OSTaskCreate(																				// 地面站数据处理任务
		(OS_TCB*)&GroundStationTaskTCB,
		(CPU_CHAR*)"GroundStation task",
		(OS_TASK_PTR )GroundStation_task,
		(void*)0,
		(OS_PRIO)GroundStation_TASK_PRIO,
		(CPU_STK*)&GroundStation_TASK_STK[0],
		(CPU_STK_SIZE)GroundStation_STK_SIZE/10,
		(CPU_STK_SIZE)GroundStation_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);	
	OSTaskCreate(																				// 地面站数据处理任务
		(OS_TCB*)&OtherSensorUpdateTaskTCB,
		(CPU_CHAR*)"OtherSensorUpdate task",
		(OS_TASK_PTR )OtherSensorUpdate_task,
		(void*)0,
		(OS_PRIO)OtherSensorUpdate_TASK_PRIO,
		(CPU_STK*)&OtherSensorUpdate_TASK_STK[0],
		(CPU_STK_SIZE)OtherSensorUpdate_STK_SIZE/10,
		(CPU_STK_SIZE)OtherSensorUpdate_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);	
	OSTaskCreate(																				// 飞行状态任务
		(OS_TCB*)&FlightStatusTaskTCB,
		(CPU_CHAR*)"FlightStatus task",
		(OS_TASK_PTR )FlightStatus_task,
		(void*)0,
		(OS_PRIO)FlightStatus_TASK_PRIO,
		(CPU_STK*)&FlightStatus_TASK_STK[0],
		(CPU_STK_SIZE)FlightStatus_STK_SIZE/10,
		(CPU_STK_SIZE)FlightStatus_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);		
	OSTaskCreate(																				// Message信息任务
		(OS_TCB*)&MessageTaskTCB,
		(CPU_CHAR*)"Message task",
		(OS_TASK_PTR )Message_task,
		(void*)0,
		(OS_PRIO)Message_TASK_PRIO,
		(CPU_STK*)&Message_TASK_STK[0],
		(CPU_STK_SIZE)Message_STK_SIZE/10,
		(CPU_STK_SIZE)Message_STK_SIZE,
		(OS_MSG_QTY)0,
		(OS_TICK)0,
		(void*)0,
		(OS_OPT)OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
		(OS_ERR*)&err
		);
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);				// 挂起开始任务
	OS_CRITICAL_EXIT();																	// 离开临界区 
}



