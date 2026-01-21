#include "body_posture.h"         

// 【新增】死区阈值，默认 3.0 度
float roll_dead_zone = 3.0f;

//用于接收坐标的全局变量 ---
float cmd_xL, cmd_yL, cmd_xR, cmd_yR;


float internal_target_avg_height = 83.0f; // 目标平均高度
float internal_current_sim_height = 83.0f;// 当前模拟高度

// PID 参数
//在外部传递一个目标期望翻滚角参数，然后身体应该自动趋向于设定的期望高度       （输入是当前翻滚角，输出是高度偏移量。）
//定义局部期望翻滚角变量，定义角度环的Kp、Kd全局变量。已经有测得的翻滚角roll，翻滚角原始角速度数据gyrox
    
float roll_Kp = 2.0f;   // 需要调试：每偏1度，腿伸缩多少mm    差值为20度的时候，输出最大的高度
float roll_Kd = 0.002f;  // 需要调试：阻尼，抑制震荡
float target_roll_angle = 0.0f; // 默认为0度（水平平衡）

float roll_mechanical_zero = -2.85f;

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

// 内部使用的 PID 计算函数 (可以是 static，不给外部看)
static float Roll_PID_Core(float current_roll, float gyro_roll_rate)
{
	
    float error = target_roll_angle - current_roll - roll_mechanical_zero;


    // 2. 【核心】死区逻辑
    // 如果误差的绝对值小于死区值 (例如 -3 < error < 3)
    if (fabs(error) <= roll_dead_zone)
    {
		return 0.0f;
    }

    float output = (roll_Kp * error) + (roll_Kd * (-gyro_roll_rate));
    return output;
}

/**
 * @brief  姿态平衡核心任务
 * @param  current_roll   当前翻滚角
 * @param  gyro_roll_rate 当前角速度
 * @param  out_yL         [输出] 计算并限幅后的左腿高度
 * @param  out_yR         [输出] 计算并限幅后的右腿高度
 */
void Body_Balance_Compute(float current_roll, float gyro_roll_rate, float *out_yL, float *out_yR)
{
    // 1. 计算 PID 偏移量 (offset)
    float height_offset = Roll_PID_Core(current_roll, gyro_roll_rate);

    // 2. 叠加基础高度 (cmd_yL/R 是全局变量，直接读取)
    float temp_yL = cmd_yL - height_offset;
    float temp_yR = cmd_yR + height_offset;

    // 3. 饱和限幅 (关键逻辑)
    if (temp_yL > LEG_MAX_HEIGHT) temp_yL = LEG_MAX_HEIGHT;
    if (temp_yL < LEG_MIN_HEIGHT) temp_yL = LEG_MIN_HEIGHT;

    if (temp_yR > LEG_MAX_HEIGHT) temp_yR = LEG_MAX_HEIGHT;
    if (temp_yR < LEG_MIN_HEIGHT) temp_yR = LEG_MIN_HEIGHT;
    
    // 4. 将最终结果赋值给指针指向的变量，传回给调用者
    *out_yL = temp_yL;
    *out_yR = temp_yR;
}


// 【新增】提供一个给 main.c 调用的接口来修改目标角度
void Set_Target_Roll_Angle(float angle)
{
    target_roll_angle = angle;
}