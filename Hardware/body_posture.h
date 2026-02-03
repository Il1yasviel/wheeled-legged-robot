#ifndef __BODY_POSTURE_H
#define __BODY_POSTURE_H		

#include "stm32f10x.h" 
#include "data_read.h"
#include <math.h>

// 【新增】X轴范围 (0=后, 44=前, 22=中)
#define LEG_X_MIN       0.0f
#define LEG_X_MAX       44.0f
#define LEG_X_CENTER    22.0f


// --- 机械限位配置 (放在头文件方便修改) ---
#define LEG_MAX_HEIGHT  137.0f  // 腿最长伸至 170mm
#define LEG_MIN_HEIGHT  83.0f   // 腿最短缩至 90mm

// --- 1. 参数配置区 ---
#define H_MIN  83.0f    // 最低高度
#define H_MAX  137.0f   // 最高高度

// PID 映射范围 (全部乘以 1.5)
#define KP_AT_MIN  300.0f    // 原 150.0 -> 225.0
#define KP_AT_MAX  460.0f    // 原 230.0 -> 345.0 (或者保守点 320)

#define KD_AT_MIN  -3.0f   // 原 -0.25 -> -0.375
#define KD_AT_MAX  -4.2f   // 原 -0.35 -> -0.525



extern float cog_Kp, cog_Kd;       // 【新增】重心PID参数
extern float internal_target_avg_height; // 目标平均高度
extern float internal_current_sim_height;// 当前模拟高度
//用于接收坐标的全局变量 ---
extern float cmd_xL, cmd_yL, cmd_xR, cmd_yR;
extern float roll_Kp;
extern float roll_Kd;
extern float target_roll_angle; // 期望翻滚角（通常是0，也可以设定为倾斜）
// 【新增】导出机械中值变量
extern float roll_mechanical_zero;





void Motor_Set_Target_Height(float yL, float yR);

// body_posture.h
void Set_Target_Roll_Angle(float angle);

//输入传感器数据，通过指针“带回”计算好的最终高度
void Body_Balance_Compute(float current_roll, float gyro_roll_rate, 
                          float target_speed, float current_speed, 
                          float *out_x, float *out_yL, float *out_yR);


#endif


