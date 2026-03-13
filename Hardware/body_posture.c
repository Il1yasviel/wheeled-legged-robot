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
// 5. 重心环参数 (X轴 - 前后平衡) 【无延迟暴力响应版】
// ==========================================================
float cog_Kp_accel = 0.5f;  // 起步后蹬比例
float cog_Kp_brake = 4.0f;  // 【调大！】刹车与手推时的前伸比例

// 【极其重要：小车的物理极速！】
// 请根据你的车轮实际空转能达到的最大速度来填。
// 如果不填这个，腿永远不可能收回中间！
float MAX_PHYSICAL_SPEED = 20.0f; 



float roll_mechanical_zero = -2.85f;




static float COG_PID_Core(float target_speed, float current_speed) {
    // =====================================================
    // 1. 【强制量力而行：解决腿永远收不回来的死结】
    // 强行把上位机的 140 限制到电机能达到的 40。
    // 这样 current_speed 才能追上 target_speed，误差才能归零，腿才能回正！
    // =====================================================
    if (target_speed > MAX_PHYSICAL_SPEED) target_speed = MAX_PHYSICAL_SPEED;
    if (target_speed < -MAX_PHYSICAL_SPEED) target_speed = -MAX_PHYSICAL_SPEED;

    // 2. 纯净死区 (删掉所有低通滤波，恢复零延迟)
    // 你说噪声在 0.5~0.8，那我们就把死区卡在 1.2。
    if (target_speed == 0 && fabs(current_speed) <= 1.2f) {
        return 0.0f; // 彻底停稳，腿立马回正，绝不拖泥带水！
    }

    // 3. 纯粹的误差方向计算
    float error = target_speed - current_speed;
    float output = 0.0f;

    // 4. 完美分流
    if (target_speed == 0) 
    {
        // 手推 或 刹车：
        // 假设你手推速度为 5，5 * 4.0 = 20mm！腿会瞬间顶满！
        output = error * cog_Kp_brake; 
    } 
    else 
    {
        // 行驶起步：
        output = error * cog_Kp_accel; 
    }

    // =====================================================
    // 5. 救命的物理限幅 (保护机械结构不死锁)
    // 必须保留！否则算出来偏移量太大，IK解算无解，舵机会死机不动！
    // =====================================================
    if (output > 20.0f) output = 20.0f;
    if (output < -20.0f) output = -20.0f;

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
 * @brief  姿态平衡核心任务
 * @param  current_roll   当前翻滚角
 * @param  gyro_roll_rate 当前角速度
 * @param  out_yL         [输出] 计算并限幅后的左腿高度
 * @param  out_yR         [输出] 计算并限幅后的右腿高度
 */
//还有两条腿在X轴方向的偏移量
//传进来参数：当前速度和目标速度  用于重心环的计算，也就是腿在x轴的PID
void Body_Balance_Compute(float current_roll, float gyro_roll_rate, 
                          float target_speed, float current_speed,
                          float *out_x, float *out_yL, float *out_yR)
{
    // 1. 一阶滤波处理
    static float last_filtered_roll = 0.0f; // 静态变量，保留上次计算结果
    float filtered_roll = ROLL_FILTER_ALPHA * current_roll + (1.0f - ROLL_FILTER_ALPHA) * last_filtered_roll;
    last_filtered_roll = filtered_roll; // 更新历史记录

    // 2. 使用滤波后的值进行 Y 轴平衡计算
    // 注意：这里传入的是 filtered_roll 而不是原始的 current_roll
    float height_offset = Roll_PID_Core(filtered_roll, gyro_roll_rate);

    float temp_yL = cmd_yL - height_offset; 
    float temp_yR = cmd_yR + height_offset; 

    // 限幅 Y
    if (temp_yL > LEG_MAX_HEIGHT) temp_yL = LEG_MAX_HEIGHT; if (temp_yL < LEG_MIN_HEIGHT) temp_yL = LEG_MIN_HEIGHT;
    if (temp_yR > LEG_MAX_HEIGHT) temp_yR = LEG_MAX_HEIGHT; if (temp_yR < LEG_MIN_HEIGHT) temp_yR = LEG_MIN_HEIGHT;
    
    // B. 计算重心调整 (X轴)
    float x_offset = COG_PID_Core(target_speed, current_speed);
    
    // 逻辑：急刹车(Error负) -> x_offset负 -> 22 - (负) = 变大 -> 腿前伸
    float temp_x = LEG_X_CENTER - x_offset; 
    
    // 限幅 X
    if (temp_x > LEG_X_MAX) temp_x = LEG_X_MAX;
    if (temp_x < LEG_X_MIN) temp_x = LEG_X_MIN;

    // C. 输出
    *out_yL = temp_yL;
    *out_yR = temp_yR;
    *out_x  = temp_x; 
}


