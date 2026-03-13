#include "body_posture.h"         
#include <math.h>

// 定义滤波系数，建议范围 0.1 ~ 0.4
// 0.1 表示 10% 原始信号 + 90% 上次信号（滤波强，延迟高）
// 0.8 表示 80% 原始信号 + 20% 上次信号（滤波弱，延迟低）
#define ROLL_FILTER_ALPHA 0.95f 


// 死区阈值，默认 3.0 度
float roll_dead_zone = 3.0f;
float cog_dead_zone  = 3.0f;     // 【新增】重心环死区 (速度差小于此值，X轴不动)


//用于接收坐标的全局变量 ---
float cmd_xL, cmd_yL, cmd_xR, cmd_yR;


float internal_target_avg_height = 83.0f; // 目标平均高度
float internal_current_sim_height = 83.0f;// 当前模拟高度

// PID 参数
//在外部传递一个目标期望翻滚角参数，然后身体应该自动趋向于设定的期望高度       （输入是当前翻滚角，输出是高度偏移量。）
//定义局部期望翻滚角变量，定义角度环的Kp、Kd全局变量。已经有测得的翻滚角roll，翻滚角原始角速度数据gyrox
    
float roll_Kp = 1.5f;   // 需要调试：每偏1度，腿伸缩多少mm    差值为20度的时候，输出最大的高度
float roll_Kd = 0.0f;  // 需要调试：阻尼，抑制震荡
float target_roll_angle = 0.0f; // 默认为0度（水平平衡）


// 5. 重心环参数 (X轴 - 前后平衡)   前馈机制
//float cog_Kp = 2.0f;   // 速度差越大，腿伸缩越多
//float cog_Kd = 0.0f;   // 抑制前后晃动



// ==========================================================
// 5. 重心环参数 (X轴) 【死区过滤底噪 + 纯正闭环版】
// ==========================================================
// 闭环反馈系数 
float cog_Kp = 1.4f;  

// 【核心死区】：根据你的日志，1个脉冲跳变大概是 100~200 PPS。
// 只要速度在这个范围内跳，视为编码器底噪，腿绝对不乱动！
float cog_stop_deadzone = 40.0f; 



//翻滚机械零点
float roll_mechanical_zero = -2.85f;



static float COG_PID_Core(float target_speed, float current_speed) 
{
    // ====================================================
    // 1. 核心死区拦截：专治原地抽搐！
    // ====================================================
    // 如果目标是 0（想站住），且当前速度全是底噪毛刺，强行当做 0 处理！
    if (target_speed == 0.0f && fabs(current_speed) <= cog_stop_deadzone) 
    {
        current_speed = 0.0f; 
    }

    // ====================================================
    // 2. 最纯粹的闭环计算
    // ====================================================
    float error = target_speed - current_speed;
    float output = error * cog_Kp;

    // 【闭环物理推演 - 完全符合你的要求】：
    // 1. 发送 80：误差 80，output = 4mm -> 腿向后蹬，前倾加速！
    // 2. 速度超过 80 (例如跑到 200)：误差 = 80 - 200 = -120，output = -6mm -> 腿越过中心向前伸，主动降速！
    // 3. 速度降回 60：误差 = 80 - 60 = 20，output = 1mm -> 腿又微微往后收，继续保持。

    // 3. 极限保护：防止 IK 逆解报错
    if (output > 22.0f) output = 22.0f;
    if (output < -22.0f) output = -22.0f;

    // 4. 加一层极轻微的保护，防止机械结构磨损，不影响响应速度
    static float last_output = 0.0f;
    output = (output * 0.5f) + (last_output * 0.5f);
    last_output = output;

    return output;
}

/**
 * @brief  设置目标高度 (由 main.c 解析完指令后调用)
 * @param  yL 左腿高度
 * @param  yR 右腿高度
 */
void Motor_Set_Target_Height(float yL, float yR)
{
    // 直接计算平均高度并存起来
    internal_target_avg_height = (yL + yR) / 2.0f;
    
    // 安全限幅
    if (internal_target_avg_height < H_MIN) internal_target_avg_height = H_MIN;
    if (internal_target_avg_height > H_MAX) internal_target_avg_height = H_MAX;
}


//写一个翻滚角的PID角度环，控制身体左右倾斜

static float Roll_PID_Core(float current_roll, float gyro_roll_rate)
{
    float error = target_roll_angle - current_roll - roll_mechanical_zero;
    float dead_zone_error = 0.0f;

    // --- 平滑死区逻辑 ---
    if (error > roll_dead_zone) {
        // 例如：误差 5.1，死区 5.0 -> 有效误差 0.1 (平滑过渡)
        dead_zone_error = error - roll_dead_zone; 
    }
    else if (error < -roll_dead_zone) {
        // 例如：误差 -5.1，死区 5.0 -> 有效误差 -0.1
        dead_zone_error = error + roll_dead_zone;
    }
    else {
        // 在死区内，直接返回0
        return 0.0f;
    }

    // 使用减去死区后的 error 进行计算
    float output = (roll_Kp * dead_zone_error) + (roll_Kd * (-gyro_roll_rate));
    return output;
}


// 【新增】提供一个给 main.c 调用的接口来修改目标角度
void Set_Target_Roll_Angle(float angle)
{
    target_roll_angle = angle;
}


//程序可以实时读取轮子的转速，
//然后也会知道上位机设定的目标速度，
//那么就可以使用目标速度减去当前轮速，
//获得一个差值，然后使用PID算法来输出一个X的偏移量。
//X为0时腿伸向最后边，X为44时，腿伸向最前边。
//默认情况下X为22，也就是中间。所以可以把解析的x坐标换成 x+x_Offset，
//这样在车子启动的时候和紧急刹车的时候，差值最大，然后腿部就会改变姿势从而改变重心。
// 2. 重心 COG PID (控制前后 X 位置)
// 2. 重心 COG PID (X轴) - 【新增死区逻辑】
// static float COG_PID_Core(float target_speed, float current_speed) {
//     float error = target_speed - current_speed;
    
//     // 【新增】重心死区逻辑
//     // 如果速度误差很小（比如只是路面颠簸引起的编码器波动），不要调整重心
//     // 只有当真正的急加速或急刹车（误差很大）时，才动腿
//     if (fabs(error) <= cog_dead_zone) return 0.0f;

//     static float last_error = 0;
//     float output = (cog_Kp * error) + (cog_Kd * (error - last_error));
//     last_error = error;
//     return output;
// }




/**
 * @brief  姿态平衡核心任务 - 独立控制版
 * @param  current_roll   当前翻滚角
 * @param  gyro_roll_rate 当前角速度
 * @param  out_xL/R       [输出] 最终左/右腿X坐标
 * @param  out_yL/R       [输出] 最终左/右腿Y坐标
 */
void Body_Balance_Compute(float current_roll, float gyro_roll_rate, 
                          float *out_xL, float *out_yL, 
                          float *out_xR, float *out_yR)
{
    // --- 1. Y轴平衡修正 (高度控制) ---
    static float last_filtered_roll = 0.0f; 
    float filtered_roll = ROLL_FILTER_ALPHA * current_roll + (1.0f - ROLL_FILTER_ALPHA) * last_filtered_roll;
    last_filtered_roll = filtered_roll; 

    // 计算平衡补偿量
    float height_offset = Roll_PID_Core(filtered_roll, gyro_roll_rate);

    // 最终高度 = 上位机基准高度 + 平衡补偿
    float temp_yL = cmd_yL - height_offset; 
    float temp_yR = cmd_yR + height_offset; 

    // Y轴限幅
    if (temp_yL > LEG_MAX_HEIGHT) temp_yL = LEG_MAX_HEIGHT; if (temp_yL < LEG_MIN_HEIGHT) temp_yL = LEG_MIN_HEIGHT;
    if (temp_yR > LEG_MAX_HEIGHT) temp_yR = LEG_MAX_HEIGHT; if (temp_yR < LEG_MIN_HEIGHT) temp_yR = LEG_MIN_HEIGHT;

    // --- 2. X轴坐标处理 (独立控制) ---
    // 不再计算 PID，直接使用上位机传入的 cmd_xL 和 cmd_xR
    float temp_xL = cmd_xL;
    float temp_xR = cmd_xR;

    // X轴限幅
    if (temp_xL > LEG_X_MAX) temp_xL = LEG_X_MAX; if (temp_xL < LEG_X_MIN) temp_xL = LEG_X_MIN;
    if (temp_xR > LEG_X_MAX) temp_xR = LEG_X_MAX; if (temp_xR < LEG_X_MIN) temp_xR = LEG_X_MIN;

    // --- 3. 输出赋值 ---
    *out_yL = temp_yL;
    *out_yR = temp_yR;
    *out_xL = temp_xL; 
    *out_xR = temp_xR;
}



