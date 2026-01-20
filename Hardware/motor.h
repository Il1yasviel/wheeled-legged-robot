#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "data_read.h"

extern float upright_Kp;
extern float upright_Kd;
extern float cascade_speed_Kp;
extern float cascade_speed_Ki;
extern float turn_Kp;
extern float turn_Kd;
extern float mechanical_zero;

void control_motor(void);

// --- 【新增】外部调用的接口 ---
// 1. 设置目标高度 (在串口接收到 K 指令时调用)
void Motor_Set_Target_Height(float yL, float yR);

// 2. PID 刷新任务 (在主循环 mainTask 中周期调用)
void Motor_PID_Update_Task(void);
#endif 

