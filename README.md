# HappyMoon开源项目
# HappyMoon Open source project

## 高性能高集成度飞控代码
### 描述
#### 软件方面：

1.使用了Ucos III操作系统作为基础。有了RTOS的支持，可以使得代码更具可读性扩展性。

2.IMU板载解算使用Mahony滤波算法 （六轴姿态解算）也可以使用九轴姿态解算

3.陀螺仪校准使用平均值校准

4.加速计校准使用LM算法（高斯牛顿法）

5.位置速度融合利用kalman滤波（滑动窗口解决延时问题）

6.角速度控制利用PD控制（同时补偿轴间耦合）

7.位置控制包括PID与苏黎世控制框架（可选）

#### 硬件方面：

1.主控STM32F405 飞控可选Speedybee F4 AIO

2.IMU MPU6000 MPU6500

#### 位置信息来源: 

1.视觉惯性里程计 （室内精准定位）

2.GPS （室外精准定位） 暂时还未融合

#### 航线规划

1. ethz-asl/waypoint_navigator 苏黎世算法：https://github.com/ethz-asl/waypoint_navigator

2. HKUST-Aerial-Robotics/grad_traj_optimization 香港科技大学：https://github.com/HKUST-Aerial-Robotics/grad_traj_optimization













