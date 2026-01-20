#ifndef __ROBOT_CTRL_H
#define __ROBOT_CTRL_H

#include <stdint.h>

// =============================================================
// 1. 定义配置结构体 (全局参数)
// =============================================================
typedef struct {
    // --- 功能开关 ---
    uint8_t Auto_PID_Enable;   // 1:开启高度自动调整PID; 0:关闭(使用固定值)

    // --- 高度物理边界 (根据你的机械结构设定, 单位: mm) ---
    float Height_Min;          // 最低蹲伏高度 (例如 83.0)
    float Height_Max;          // 最高站立高度 (例如 137.0)

    // --- 蹲下时的参数 (低重心，稳，参数小) ---
    float Kp_At_Min;           
    float Kd_At_Min;

    // --- 站立时的参数 (高重心，晃，参数大) ---
    float Kp_At_Max;
    float Kd_At_Max;

    // --- 平滑系数 (0.01~1.0) ---
    // 值越小，动作越慢越柔和；值越大，反应越快
    float Smooth_Alpha;        

} Robot_Config_t;

// 声明全局配置实例，方便 main 或 串口任务 修改它
extern Robot_Config_t SysConfig;

// =============================================================
// 2. 函数接口声明
// =============================================================

// 初始化 (赋默认值，上电调用一次)
void Robot_Ctrl_Init(void);

// 设置新的目标坐标 (给串口任务调用)
// 只需要输入目标，内部会自动处理平滑过渡
void Robot_Set_Target(float xL, float yL, float xR, float yR);

// 核心更新函数 (必须放在 mainTask 的 5ms 循环里调用)
// 包含: 平滑处理 -> 自动PID计算 -> 逆运动学 -> 驱动舵机
void Robot_Update_Loop(void);

#endif /* __ROBOT_CTRL_H */

