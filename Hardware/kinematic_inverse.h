#ifndef __KINEMATIC_INVERSE_H
#define __KINEMATIC_INVERSE_H

#include <math.h>
#include <stdint.h>

/* ============================================================ */
/* 1. 物理参数配置 (单位: mm)                                   */
/* ============================================================ */
// 根据之前的沟通，你的臂长是 6cm = 60.0mm
#define LINK_L1    60.0f   // 左主动臂长度
#define LINK_L2    100.0f   // 左从动臂长度
#define LINK_L3    100.0f   // 右从动臂长度
#define LINK_L4    60.0f   // 右主动臂长度

#define LINK_L5    44.0f   

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* ============================================================ */
/* 2. 数据结构定义                                              */
/* ============================================================ */
typedef struct {
    // --- 输入：目标坐标 (单位: mm) ---
    float Target_X_Left;
    float Target_Y_Left;
    float Target_X_Right;
    float Target_Y_Right;

    // --- 中间变量：计算出的纯几何弧度 (调试用) ---
    float Rad_Alpha_Left;
    float Rad_Beta_Left;
    float Rad_Alpha_Right;
    float Rad_Beta_Right;
	
	
	// --- 【新增】纯几何角度 (调试用，不带舵机安装误差) ---
    uint16_t Math_Angle_Alpha_Left;  // 左后臂几何角度
    uint16_t Math_Angle_Beta_Left;   // 左前臂几何角度
    uint16_t Math_Angle_Alpha_Right; // 右后臂几何角度
    uint16_t Math_Angle_Beta_Right;  // 右前臂几何角度

    // --- 输出：最终舵机角度 (单位: 度, 0~180 或 0~270) ---
    // 这些值已经包含了安装误差修正 (如 +90, -270)
    uint16_t Angle_Servo_Left_Front;  // 左前舵机
    uint16_t Angle_Servo_Left_Rear;   // 左后舵机
    uint16_t Angle_Servo_Right_Front; // 右前舵机
    uint16_t Angle_Servo_Right_Rear;  // 右后舵机
    
} IK_Data_t;

/* ============================================================ */
/* 3. 全局变量与函数声明                                        */
/* ============================================================ */
extern IK_Data_t Robot_IK;

// 初始化数据
void IK_Init(void);

// 核心解算函数：输入毫米坐标，直接更新结构体内的角度
uint8_t IK_Compute(float xL, float yL, float xR, float yR);

#endif /* __KINEMATIC_INVERSE_H */

