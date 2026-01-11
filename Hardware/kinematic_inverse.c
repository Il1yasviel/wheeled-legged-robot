#include "kinematic_inverse.h"

// 定义全局结构体实例
IK_Data_t Robot_IK;

// 内部辅助函数：弧度转角度整数
static uint16_t RadianToAngle(float rad) {
    // (rad / 2π) * 360 = rad * (180 / π)
    float angle = rad * (180.0f / PI);
    return (uint16_t)angle; 
}

void IK_Init(void) {
    // 给一些安全的初始值，避免刚上电数据为0导致舵机乱转
    Robot_IK.Target_X_Left = 0.0f;
    Robot_IK.Target_Y_Left = 90.0f; // 假设默认高度 100mm
    Robot_IK.Target_X_Right = 0.0f;
    Robot_IK.Target_Y_Right = 90.0f;
}

void IK_Compute(float xL, float yL, float xR, float yR) {
    // 1. 更新输入数据到结构体方便查看
    Robot_IK.Target_X_Left = xL;
    Robot_IK.Target_Y_Left = yL;
    Robot_IK.Target_X_Right = xR;
    Robot_IK.Target_Y_Right = yR;

    float alpha1, alpha2, beta1, beta2;
    
    // =================================================================
    //                            左腿解算
    // =================================================================
    // 求解 Alpha (左后舵机控制的主臂)
    float aLeft = 2 * xL * LINK_L1;
    float bLeft = 2 * yL * LINK_L1;
    float cLeft = xL * xL + yL * yL + LINK_L1 * LINK_L1 - LINK_L2 * LINK_L2;

    // 求解 Beta (左前舵机控制的副臂链)
    // 这里的 d,e,f 对应公式中的 A,B,C
    float dLeft = 2 * LINK_L4 * (xL - LINK_L5);
    float eLeft = 2 * LINK_L4 * yL;
    float fLeft = (xL - LINK_L5) * (xL - LINK_L5) + LINK_L4 * LINK_L4 + yL * yL - LINK_L3 * LINK_L3;

    float delta_alpha_L = aLeft * aLeft + bLeft * bLeft - cLeft * cLeft;
    float delta_beta_L  = dLeft * dLeft + eLeft * eLeft - fLeft * fLeft;

    // --- 解 Alpha ---
    if (delta_alpha_L >= 0) {
        float sqrt_val = sqrtf(delta_alpha_L);
        alpha1 = 2 * atan2f(bLeft + sqrt_val, aLeft + cLeft);
        alpha2 = 2 * atan2f(bLeft - sqrt_val, aLeft + cLeft);
        
        // 归一化到 0~2PI
        if(alpha1 < 0) alpha1 += 2 * PI;
        if(alpha2 < 0) alpha2 += 2 * PI;

        // 选解逻辑: 根据你的机械结构，通常选 > 45度(PI/4)的那个解
        if (alpha1 >= PI / 4.0f) Robot_IK.Rad_Alpha_Left = alpha1;
        else Robot_IK.Rad_Alpha_Left = alpha2;
    }

    // --- 解 Beta ---
    if (delta_beta_L >= 0) {
        float sqrt_val = sqrtf(delta_beta_L);
        beta1 = 2 * atan2f(eLeft + sqrt_val, dLeft + fLeft);
        beta2 = 2 * atan2f(eLeft - sqrt_val, dLeft + fLeft);

        if(beta1 < 0) beta1 += 2 * PI;
        if(beta2 < 0) beta2 += 2 * PI;

        // 选解逻辑: 通常选 < 45度(PI/4)的那个解
        if (beta1 >= 0 && beta1 <= PI / 4.0f) Robot_IK.Rad_Beta_Left = beta1;
        else Robot_IK.Rad_Beta_Left = beta2;
    }

    // =================================================================
    //                            右腿解算
    // =================================================================
    // 右腿的计算逻辑完全一致，只是输入参数不同 (原点平移)
    float aRight = 2 * xR * LINK_L1;
    float bRight = 2 * yR * LINK_L1;
    float cRight = xR * xR + yR * yR + LINK_L1 * LINK_L1 - LINK_L2 * LINK_L2;

    float dRight = 2 * LINK_L4 * (xR - LINK_L5);
    float eRight = 2 * LINK_L4 * yR;
    float fRight = (xR - LINK_L5) * (xR - LINK_L5) + LINK_L4 * LINK_L4 + yR * yR - LINK_L3 * LINK_L3;

    float delta_alpha_R = aRight * aRight + bRight * bRight - cRight * cRight;
    float delta_beta_R  = dRight * dRight + eRight * eRight - fRight * fRight;

    if (delta_alpha_R >= 0) {
        float sqrt_val = sqrtf(delta_alpha_R);
        alpha1 = 2 * atan2f(bRight + sqrt_val, aRight + cRight);
        alpha2 = 2 * atan2f(bRight - sqrt_val, aRight + cRight);

        if(alpha1 < 0) alpha1 += 2 * PI;
        if(alpha2 < 0) alpha2 += 2 * PI;

        if (alpha1 >= PI / 4.0f) Robot_IK.Rad_Alpha_Right = alpha1;
        else Robot_IK.Rad_Alpha_Right = alpha2;
    }

    if (delta_beta_R >= 0) {
        float sqrt_val = sqrtf(delta_beta_R);
        beta1 = 2 * atan2f(eRight + sqrt_val, dRight + fRight);
        beta2 = 2 * atan2f(eRight - sqrt_val, dRight + fRight);

        if(beta1 < 0) beta1 += 2 * PI;
        if(beta2 < 0) beta2 += 2 * PI;

        if (beta1 >= 0 && beta1 <= PI / 4.0f) Robot_IK.Rad_Beta_Right = beta1;
        else Robot_IK.Rad_Beta_Right = beta2;
    }

    // =================================================================
    //              结果输出：弧度 -> 角度(整数) -> 机械修正
    // =================================================================
    int alphaLeftAngle  = RadianToAngle(Robot_IK.Rad_Alpha_Left);
    int betaLeftAngle   = RadianToAngle(Robot_IK.Rad_Beta_Left);
    int alphaRightAngle = RadianToAngle(Robot_IK.Rad_Alpha_Right);
    int betaRightAngle  = RadianToAngle(Robot_IK.Rad_Beta_Right);
	
	
	// 2. 【新增】将纯数学角度存入结构体，方便你查看/打印
    Robot_IK.Math_Angle_Alpha_Left  = alphaLeftAngle;
    Robot_IK.Math_Angle_Beta_Left   = betaLeftAngle;
    Robot_IK.Math_Angle_Alpha_Right = alphaRightAngle;
    Robot_IK.Math_Angle_Beta_Right  = betaRightAngle;

    // 【关键】偏移量修正
    // 这里保留了你原代码中的修正逻辑。
    // 如果发现实际方向反了，请修改这里的 + 或 -，以及基准值 (90/270)
    Robot_IK.Angle_Servo_Left_Front  = 120 - betaLeftAngle;
    Robot_IK.Angle_Servo_Left_Rear   = 300 - alphaLeftAngle;
    Robot_IK.Angle_Servo_Right_Front = 270 - betaRightAngle;
    Robot_IK.Angle_Servo_Right_Rear  = 270 - alphaRightAngle;
}

