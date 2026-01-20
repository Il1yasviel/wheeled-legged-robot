#include "robot_ctrl.h"
#include "kinematic_inverse.h" // 包含你的逆运动学解算
#include "servo_motor.h"       // 包含你的舵机驱动
#include "motor.h"             // 包含 upright_Kp, upright_Kd 全局变量
#include <stdio.h>

// =============================================================
// 内部变量 (static 保护，不让外部直接乱改)
// =============================================================
// 目标值：串口发来的指令
static float Target_X_L, Target_Y_L, Target_X_R, Target_Y_R; 
// 平滑值：实际发给舵机的值 (缓慢跟随目标)
static float Smooth_X_L, Smooth_Y_L, Smooth_X_R, Smooth_Y_R; 

// 全局配置实例
Robot_Config_t SysConfig;

// =============================================================
// 1. 初始化函数
// =============================================================
void Robot_Ctrl_Init(void)
{
    // 1. 初始化配置参数 (默认值)
    SysConfig.Auto_PID_Enable = 0; // 默认关闭，建议调试好参数后再通过串口开启
    
    // --- 高度范围配置 ---
    SysConfig.Height_Min = 83.0f;
    SysConfig.Height_Max = 137.0f;
    
    // --- PID 参数映射配置 ---
    // 这组参数需要你先手动测出来：蹲下最稳的参数
    SysConfig.Kp_At_Min  = 110.0f; 
    SysConfig.Kd_At_Min  = -0.25f;   
    
    // 这组参数需要你先手动测出来：站立最稳的参数
    SysConfig.Kp_At_Max  = 170.0f;; 
    SysConfig.Kd_At_Max  = -0.35f;   
    
    SysConfig.Smooth_Alpha = 0.03f; // 默认平滑系数

    // 2. 初始化坐标 (默认为蹲下状态, X轴归中)
    // 假设 X=22 是物理中心
    Target_X_L = 22.0f; Target_Y_L = SysConfig.Height_Min;
    Target_X_R = 22.0f; Target_Y_R = SysConfig.Height_Min;

    // 初始状态下，让平滑值直接等于目标值，避免上电瞬间舵机猛地归位
    Smooth_X_L = Target_X_L; Smooth_Y_L = Target_Y_L;
    Smooth_X_R = Target_X_R; Smooth_Y_R = Target_Y_R;
    
    // 初始化逆运动学模块
    IK_Init();
}

// =============================================================
// 2. 设置目标 (外部接口)
// =============================================================
void Robot_Set_Target(float xL, float yL, float xR, float yR)
{
    // --- 左腿限幅保护 ---
    // 防止串口输入错误数值导致舵机撞坏结构
    if(yL < SysConfig.Height_Min) yL = SysConfig.Height_Min;
    if(yL > SysConfig.Height_Max) yL = SysConfig.Height_Max;

    // --- 右腿限幅保护 (完整补全) ---
    if(yR < SysConfig.Height_Min) yR = SysConfig.Height_Min;
    if(yR > SysConfig.Height_Max) yR = SysConfig.Height_Max;

    // 更新静态目标变量
    Target_X_L = xL; Target_Y_L = yL;
    Target_X_R = xR; Target_Y_R = yR;
    
    // 调试打印 (可选，频繁打印会影响性能，建议注释掉)
    // printf(">> [Ctrl] Target Updated: L=%.1f, R=%.1f\r\n", yL, yR);
}

// =============================================================
// 3. 内部私有函数：计算自动 PID
// =============================================================
static void Auto_Calc_PID(float current_yL, float current_yR)
{
    // 1. 如果开关没开，直接退出，保持 PID 不变 (允许手动调试)
    if(SysConfig.Auto_PID_Enable == 0) return; 

    // 2. 计算左右腿的平均高度
    float avg_h = (current_yL + current_yR) / 2.0f;
    
    // 3. 再次限制范围，防止计算溢出 (双重保险)
    if(avg_h < SysConfig.Height_Min) avg_h = SysConfig.Height_Min;
    if(avg_h > SysConfig.Height_Max) avg_h = SysConfig.Height_Max;

    // 4. 计算当前高度在行程中的百分比 (0.0 ~ 1.0)
    float range = SysConfig.Height_Max - SysConfig.Height_Min;
    
    // 防止分母为0 (如果最大最小高度设成一样)
    if(range < 0.1f) range = 1.0f; 
    
    float ratio = (avg_h - SysConfig.Height_Min) / range;
    
    // 5. 线性插值更新全局 PID 变量
    // 公式: Kp = 最小值 + 比率 * (最大值 - 最小值)
    // 这些变量 (upright_Kp/Kd) 必须在 motor.h 中声明为 extern
    upright_Kp = SysConfig.Kp_At_Min + ratio * (SysConfig.Kp_At_Max - SysConfig.Kp_At_Min);
    upright_Kd = SysConfig.Kd_At_Min + ratio * (SysConfig.Kd_At_Max - SysConfig.Kd_At_Min);
}

// =============================================================
// 4. 核心循环更新 (被主任务调用)
// =============================================================
// 修改 robot_ctrl.c 中的这个函数
void Robot_Update_Loop(void)
{
    // --- Step 1: 平滑滤波 ---
    float alpha = SysConfig.Smooth_Alpha;
    // 防止未初始化导致 alpha 为 0
    if(alpha < 0.001f) alpha = 0.03f; 
    
    Smooth_X_L += alpha * (Target_X_L - Smooth_X_L);
    Smooth_Y_L += alpha * (Target_Y_L - Smooth_Y_L);
    Smooth_X_R += alpha * (Target_X_R - Smooth_X_R);
    Smooth_Y_R += alpha * (Target_Y_R - Smooth_Y_R);

    // --- Step 2: 自动调整 PID ---
    Auto_Calc_PID(Smooth_Y_L, Smooth_Y_R);

    // --- Step 3: 逆运动学解算 (关键修改！) ---
    // 获取返回值，判断是否安全
    uint8_t is_safe = IK_Compute(Smooth_X_L, Smooth_Y_L, Smooth_X_R, Smooth_Y_R);

    // --- Step 4: 驱动舵机 (增加安全锁) ---
    if (is_safe == 1)
    {
        // 只有算对了，才发指令
        Servo_Move(1, Robot_IK.Angle_Servo_Left_Front, 0);
        Servo_Move(2, Robot_IK.Angle_Servo_Left_Rear,  0);
        Servo_Move(3, Robot_IK.Angle_Servo_Right_Front, 0);
        Servo_Move(4, Robot_IK.Angle_Servo_Right_Rear,  0);
    }
    else
    {
        // 如果解算失败（比如坐标是0,0），绝对不动舵机！
        // 可以在这里加个 LED 闪烁报错
    }
}

