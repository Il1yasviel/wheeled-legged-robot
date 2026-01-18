#include "kinematic_inverse.h"
#include <math.h> 
#include <stdio.h> 

// =================================================================
// 1. 必须要把定义的限位数值放在这里，编译器才能看见
//    根据你的机械结构，你可以修改后面的数字
// =================================================================

// Alpha 角 (大腿/主臂) 的安全范围 (度)
#define ALPHA_MIN_LIMIT  120   // 最小值
#define ALPHA_MAX_LIMIT  160  // 最大值

// Beta 角 (小腿/副臂) 的安全范围 (度)
#define BETA_MIN_LIMIT   20   // 最小值
#define BETA_MAX_LIMIT   60  // 最大值

// =================================================================

// 全局结构体实例
IK_Data_t Robot_IK;

// 内部辅助函数：弧度转角度整数
static uint16_t RadianToAngle(float rad) {
    // (rad / 2π) * 360 = rad * (180 / π)
    float angle = rad * (180.0f / PI);
    return (uint16_t)angle; 
}

void IK_Init(void) {
    // 给一些安全的初始值
    Robot_IK.Target_X_Left = 0.0f;
    Robot_IK.Target_Y_Left = 90.0f; 
    Robot_IK.Target_X_Right = 0.0f;
    Robot_IK.Target_Y_Right = 90.0f;
}

// 带安全检测的逆运动学解算函数
// 返回值: 1=成功, 0=失败(超出范围)
// 带安全检测的逆运动学解算函数
// 返回值: 1=成功(安全), 0=失败(超限或无解)
uint8_t IK_Compute(float xL, float yL, float xR, float yR) {
    
    // --- 定义临时变量 ---
    float temp_rad_alpha_L = 0, temp_rad_beta_L = 0;
    float temp_rad_alpha_R = 0, temp_rad_beta_R = 0;
    uint8_t valid_L = 1;
    uint8_t valid_R = 1;

    // =================================================================
    //                            左腿数学解算 (保持不变)
    // =================================================================
    float aLeft = 2 * xL * LINK_L1;
    float bLeft = 2 * yL * LINK_L1;
    float cLeft = xL * xL + yL * yL + LINK_L1 * LINK_L1 - LINK_L2 * LINK_L2;
    float dLeft = 2 * LINK_L4 * (xL - LINK_L5);
    float eLeft = 2 * LINK_L4 * yL;
    float fLeft = (xL - LINK_L5) * (xL - LINK_L5) + LINK_L4 * LINK_L4 + yL * yL - LINK_L3 * LINK_L3;
    float delta_alpha_L = aLeft * aLeft + bLeft * bLeft - cLeft * cLeft;
    float delta_beta_L  = dLeft * dLeft + eLeft * eLeft - fLeft * fLeft;

    if (delta_alpha_L < 0 || delta_beta_L < 0) valid_L = 0; 
    else {
        float sqrt_val_a = sqrtf(delta_alpha_L);
        float alpha1 = 2 * atan2f(bLeft + sqrt_val_a, aLeft + cLeft);
        float alpha2 = 2 * atan2f(bLeft - sqrt_val_a, aLeft + cLeft);
        if(alpha1 < 0) alpha1 += 2 * PI; if(alpha2 < 0) alpha2 += 2 * PI;
        temp_rad_alpha_L = (alpha1 >= PI / 4.0f) ? alpha1 : alpha2;

        float sqrt_val_b = sqrtf(delta_beta_L);
        float beta1 = 2 * atan2f(eLeft + sqrt_val_b, dLeft + fLeft);
        float beta2 = 2 * atan2f(eLeft - sqrt_val_b, dLeft + fLeft);
        if(beta1 < 0) beta1 += 2 * PI; if(beta2 < 0) beta2 += 2 * PI;
        temp_rad_beta_L = (beta1 >= 0 && beta1 <= PI / 4.0f) ? beta1 : beta2;
    }

    // =================================================================
    //                            右腿数学解算 (保持不变)
    // =================================================================
    float aRight = 2 * xR * LINK_L1;
    float bRight = 2 * yR * LINK_L1;
    float cRight = xR * xR + yR * yR + LINK_L1 * LINK_L1 - LINK_L2 * LINK_L2;
    float dRight = 2 * LINK_L4 * (xR - LINK_L5);
    float eRight = 2 * LINK_L4 * yR;
    float fRight = (xR - LINK_L5) * (xR - LINK_L5) + LINK_L4 * LINK_L4 + yR * yR - LINK_L3 * LINK_L3;
    float delta_alpha_R = aRight * aRight + bRight * bRight - cRight * cRight;
    float delta_beta_R  = dRight * dRight + eRight * eRight - fRight * fRight;

    if (delta_alpha_R < 0 || delta_beta_R < 0) valid_R = 0; 
    else {
        float sqrt_val_a = sqrtf(delta_alpha_R);
        float alpha1 = 2 * atan2f(bRight + sqrt_val_a, aRight + cRight);
        float alpha2 = 2 * atan2f(bRight - sqrt_val_a, aRight + cRight);
        if(alpha1 < 0) alpha1 += 2 * PI; if(alpha2 < 0) alpha2 += 2 * PI;
        temp_rad_alpha_R = (alpha1 >= PI / 4.0f) ? alpha1 : alpha2;

        float sqrt_val_b = sqrtf(delta_beta_R);
        float beta1 = 2 * atan2f(eRight + sqrt_val_b, dRight + fRight);
        float beta2 = 2 * atan2f(eRight - sqrt_val_b, dRight + fRight);
        if(beta1 < 0) beta1 += 2 * PI; if(beta2 < 0) beta2 += 2 * PI;
        temp_rad_beta_R = (beta1 >= 0 && beta1 <= PI / 4.0f) ? beta1 : beta2;
    }

    // 数学错误（构不成三角形）无法计算角度，直接返回
    if (valid_L == 0 || valid_R == 0) {
        printf(">> IK Error: Math Impossible (Delta < 0)\r\n");
        return 0; 
    }

    // =================================================================
    // 【关键步骤 1】先强制保存所有数据！(无论是否越界)
    // =================================================================
    
    // 转换成整数角度
    int temp_alphaL_Deg = RadianToAngle(temp_rad_alpha_L);
    int temp_betaL_Deg  = RadianToAngle(temp_rad_beta_L);
    int temp_alphaR_Deg = RadianToAngle(temp_rad_alpha_R);
    int temp_betaR_Deg  = RadianToAngle(temp_rad_beta_R);

    // 保存数学角度
    Robot_IK.Math_Angle_Alpha_Left  = temp_alphaL_Deg;
    Robot_IK.Math_Angle_Beta_Left   = temp_betaL_Deg;
    Robot_IK.Math_Angle_Alpha_Right = temp_alphaR_Deg;
    Robot_IK.Math_Angle_Beta_Right  = temp_betaR_Deg;

    // 保存舵机角度 (根据公式计算，哪怕算出负数也存进去)
    Robot_IK.Angle_Servo_Left_Front  = 120 - temp_betaL_Deg;//1号舵机
    Robot_IK.Angle_Servo_Left_Rear   = 300 - temp_alphaL_Deg;//2号
    Robot_IK.Angle_Servo_Right_Front = 120 + temp_betaR_Deg;//3号
    Robot_IK.Angle_Servo_Right_Rear  = temp_alphaR_Deg -60;//4号
    
    // 更新目标坐标 (方便调试看输入了什么)
    Robot_IK.Target_X_Left = xL; Robot_IK.Target_Y_Left = yL;
    Robot_IK.Target_X_Right = xR; Robot_IK.Target_Y_Right = yR;

    // =================================================================
    // 【关键步骤 2】保存完之后，再检查是否越界，决定返回值
    // =================================================================

    // 检查 Alpha
    if (temp_alphaL_Deg < ALPHA_MIN_LIMIT || temp_alphaL_Deg > ALPHA_MAX_LIMIT ||
        temp_alphaR_Deg < ALPHA_MIN_LIMIT || temp_alphaR_Deg > ALPHA_MAX_LIMIT) {
        return 0; // 虽然保存了数据，但返回失败
    }

    // 检查 Beta
    if (temp_betaL_Deg < BETA_MIN_LIMIT || temp_betaL_Deg > BETA_MAX_LIMIT ||
        temp_betaR_Deg < BETA_MIN_LIMIT || temp_betaR_Deg > BETA_MAX_LIMIT) {
        return 0; // 虽然保存了数据，但返回失败
    }

    return 1; // 成功且安全
}



