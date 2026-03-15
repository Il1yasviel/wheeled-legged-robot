#include "motor.h"
#include <math.h>

//机械零点
float mechanical_zero=-5.3f;//0.3
//直立环
float upright_Kp=1400.0f;  //原150，乘上1.5倍
float upright_Kd=-3.5f;       //原0.25，也乘上1.5倍
//速度环
// 修改后的速度环参数（已转换）
float cascade_speed_Kp = -0.02f;
float cascade_speed_Ki =-0.01f;

// motor.c 顶部参数微调
float turn_Kp = -1.5f;    // 缩小量级，因为 Gyro_Z 原始值通常在几百到上千
float turn_Ki = -0.05f;   // 积分也要小，慢慢纠正匀速
float turn_Kd = 0.3f; 
float turn_limit = 1500.0f; // 原地旋转不需要太大功率，限幅减小防止干扰直立


//暴露PWM1和PWM2给外部，实时查看PWM的值，检查车身突然无力摔倒的原因
int16_t debug_pwm1 = 0;
int16_t debug_pwm2 = 0;



int16_t limit_pwm(int16_t pwm)//内部使用
{
    if(pwm>3400) pwm=3400;
    if(pwm<-3400) pwm=-3400;
    return pwm;
}


void Motor1_SetSpeed(int16_t pwm)//pwm 0-3200 静止-满速
{
    pwm=limit_pwm(pwm);    //忘记给PWM限定范围了，导致PWM几秒钟后就数值超限，导致小车突然无力摔倒。
    if(pwm>0)
    {
        TIM_SetCompare1(TIM3,3600-pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_4);//释放
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);//正转
    }
    if(pwm<0)
    {
        TIM_SetCompare1(TIM3,3600+pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_4);//释放
        GPIO_SetBits(GPIOB,GPIO_Pin_0);//反转       
    }
    if(pwm==0)
    {
        TIM_SetCompare1(TIM3,3600);
        GPIO_ResetBits(GPIOA,GPIO_Pin_4);//刹车
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);
    }
    
}

void Motor2_SetSpeed(int16_t pwm)// -2000<=Speed<=2000
{
	pwm = limit_pwm(pwm); // 必须要把返回值赋给 pwm ！！！   忘记加限幅导致PWM值超过电机的PWM范围，于是电机会突然摔倒
    if(pwm>=0)
    {
        TIM_SetCompare2(TIM3,3600-pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_5);//释放
        GPIO_SetBits(GPIOB,GPIO_Pin_1);//正转
    }
    if(pwm<0)
    {
        TIM_SetCompare2(TIM3,3600+pwm);
        GPIO_SetBits(GPIOA,GPIO_Pin_5);//释放
        GPIO_ResetBits(GPIOB,GPIO_Pin_1);//反转       
    }
    if(pwm==0)
    {
        TIM_SetCompare2(TIM3,3600);
        GPIO_ResetBits(GPIOA,GPIO_Pin_5);//释放
        GPIO_ResetBits(GPIOB,GPIO_Pin_1);
    }
}


//直立环 增加目标角度
int16_t upright_ring(float Angle,float Gyro, float Target_Angle)
{  
   float err;
	 int16_t pwm_upright;
	 err=mechanical_zero+ Target_Angle-Angle;    //期望值-实际值，这里期望小车平衡，因此期望值就是机械中值       
	 pwm_upright=upright_Kp*err+Gyro*upright_Kd;//计算平衡控制的电机PWM
	 return pwm_upright;
}



// 速度环（外环）- 极简纯P控制 + 10分频版 (50ms周期)
// 速度环（外环）
float speed_ring(int16_t encoder_left, int16_t encoder_right)
{  
    static uint8_t speed_count = 0;
    static float encoder_sum_left = 0.0f;
    static float encoder_sum_right = 0.0f;
    static float last_target_angle = 0.0f;
    static float speed_filter = 0.0f;
    static float speed_integral = 0.0f;
    static int16_t last_movement = 0;             

    float current_speed;
    float speed_error;

    // 1. 5ms 脉冲采集
    encoder_sum_left += encoder_left;
    encoder_sum_right += encoder_right;
    speed_count++;
    
    // 2. 每 10ms 计算一次速度环
    if (speed_count >= 2)
    {
        // ==========================================
        // 【核心排错 1】：上位机指令死区过滤！
        // 绝大多数清零失败，都是因为松手时 Movement 带有残余值 (如 1 或 -2)
        // ==========================================
        int16_t actual_move = Movement;
        if (actual_move > -5 && actual_move < 5) 
        {
            actual_move = 0; // 强行拉回绝对的 0，确保能触发后续的刹车逻辑！
        }

        // 计算当前速度 (保留你的 * 100 转换)
        current_speed = ((encoder_sum_left + encoder_sum_right) / 2.0f) * 100.0f; 
        
        // 计数器清零
        encoder_sum_left = 0.0f;
        encoder_sum_right = 0.0f;
        speed_count = 0;

        // 低通滤波 
        speed_filter = speed_filter * 0.9f + current_speed * 0.1f;

        // 3. 误差计算 (目标 - 实际)
        speed_error = (float)actual_move - speed_filter;

        // ==========================================
        // 4. 彻底生效的积分与清零逻辑
        // ==========================================
        if (actual_move == 0) 
        {
            // 刹车时：不要用 0，用 0.5 快速衰减！
            // 暴力赋值为0会让车身“咯噔”一下瞬间脱力，乘以0.5能在几帧内(几十毫秒)迅速归零，既快又稳。
            speed_integral *= 0.5f; 
        }
        else if ((actual_move > 0 && last_movement < 0) || (actual_move < 0 && last_movement > 0))
        {
            // 突然反转方向时，直接踹掉旧积分
            speed_integral = 0.0f; 
            speed_integral += speed_error;
        }
        else
        {
            // 正常按住前进/后退时，提供源源不断的动力
            speed_integral += speed_error; 
        }

        last_movement = actual_move; // 记录滤波后的真实指令

        // 原地旋转时屏蔽速度环乱跑
        if (actual_move == 0 && turnment != 0)
        {
             speed_integral = 0.0f;
             speed_filter = 0.0f;
        }

        // ==========================================
        // 【核心排错 2】：收紧积分限幅
        // 你的 Ki 是 -0.04，如果积分允许到 10000，产生的角度就是 400 度！
        // 我们把它死死卡在 2000 以内 (2000 * 0.04 = 80度，完全足够了)
        // ==========================================
        if (speed_integral > 2000.0f)  speed_integral = 2000.0f;
        if (speed_integral < -2000.0f) speed_integral = -2000.0f;

        // 6. PI 计算
        last_target_angle = (cascade_speed_Kp * speed_error) + (cascade_speed_Ki * speed_integral);

        // 7. 最终角度限幅
        if (last_target_angle > 8.0f) last_target_angle = 8.0f;
        else if (last_target_angle < -8.0f) last_target_angle = -8.0f;
    }

    // 8. 跌倒保护
    if (pitch > 40.0f || pitch < -40.0f) speed_integral = 0.0f;

    return last_target_angle;
}




int16_t turn_ring(float Target_Turn_Speed, float Gyro_Z)
{
    static float turn_integral = 0.0f;
    static float smooth_gyro = 0.0f;   // 陀螺仪平滑值
    static float smooth_target = 0.0f; // 目标速度平滑值
    int16_t turn_pwm;

    // ========================================================
    // 【核心修复 1】：对陀螺仪原始数据进行“重度低通滤波”！
    // 就像给汽车装上减震弹簧，彻底杀掉高频震荡，电机绝对不再吱吱叫！
    // ========================================================
    smooth_gyro = smooth_gyro * 0.7f + Gyro_Z * 0.3f;

    // 用平滑后的数据做死区判断
    if (smooth_gyro > -25.0f && smooth_gyro < 25.0f)
    {
        smooth_gyro = 0.0f;
    }

    // 情况 A：防漂移模式
    if (Target_Turn_Speed == 0)
    {
        turn_integral = 0.0f; 
        smooth_target = 0.0f; // 目标清零
        turn_pwm = (int16_t)(turn_Kd * smooth_gyro); 
    }
    // 情况 B：闭环旋转模式
    else
    {
        float actual_target = Target_Turn_Speed * 10.0f; 
        
        // ========================================================
        // 【核心修复 2】：对上位机指令进行“缓启动滤波”！
        // 防止目标瞬间从 0 突变到 300 导致电机暴力撕扯
        // ========================================================
        smooth_target = smooth_target * 0.9f + actual_target * 0.1f;

        // 用纯净、平滑的数据计算误差
        float turn_error = smooth_target - smooth_gyro;
        turn_integral += turn_error;

        // 积分限幅
        if (turn_integral > 30000.0f) turn_integral = 30000.0f;
        if (turn_integral < -30000.0f) turn_integral = -30000.0f;

        // PI 控制输出
        turn_pwm = (int16_t)((turn_Kp * turn_error) + (turn_Ki * turn_integral));
    }

    return turn_pwm;
}


void control_motor(void)
{
    int16_t pwm1,pwm2;
    float target_angle=0;
    int16_t upright_out;
    int16_t turn_out;


    // 1. 【速度环】获取目标角度
    // 输入左右编码器当前速度，输出小车应该倾斜的角度
    target_angle = speed_ring(speedLeft, speedRight);


    // ==========================================
    // 【核心修复】：屏蔽旋转时的假位移
    // 当指令为原地旋转时，强行让目标角为 0，防止小车“乱冲”
    // ==========================================
    if (Movement == 0 && turnment != 0) 
    {
        target_angle = 0; 
    }


    // 2. 【直立环】计算维持平衡的 PWM
    // 将速度环计算出的 target_angle 传入
    upright_out = upright_ring(pitch, gyroy, target_angle);


    // 3. 【转向环】计算转向补偿 (根据你最初代码的逻辑)
    // 这里简单处理：如果没有转向需求(turnment=0)，则用 gyroz 抑制自旋
    //if(turnment == 0) turn_out = turn_Kd * gyroz; 
    //else turn_out = (turn_Kp * turnment) - (turn_Kd * gyroz);


    // 3. 【转向环】计算转向补偿
    // 如果 turnment 为 0，期望角速度为0，误差就是 (0 - gyroz)，依然能起到抑制自旋的作用
    turn_out = turn_ring(turnment, gyroz);
	
	// --- 【转向限速核心代码】 ---
    if(turn_out > turn_limit)  turn_out = turn_limit;
    if(turn_out < -turn_limit) turn_out = -turn_limit;

    // 4. 【并联组合】
    // 直立环和速度环已经通过串级合体了，这里只需加上转向环
    pwm1 = upright_out + turn_out;
    pwm2 = upright_out - turn_out;
	
	


    // 5. 【死区补偿】 
    if(pwm1 > 0) pwm1 += 120; 
    else if(pwm1 < 0) pwm1 -= 120;

    if(pwm2 > 0) pwm2 += 120; 
    else if(pwm2 < 0) pwm2 -= 120;


	pwm1=limit_pwm(pwm1); //双重限制，解决了PWM超限导致突然摔倒的错误
	pwm2=limit_pwm(pwm2); 

	//******************************
	// 将最终计算出的PWM值存入全局变量，暴露给外部       总结：不是因为PWM超限的问题，而是因为电机本身有堵转保护，
    debug_pwm1 = pwm1;								 //给PWM，但是轮子不转，过几秒后电机会立刻脱力					
    debug_pwm2 = pwm2;
	//********************************

    Motor1_SetSpeed(pwm1);
    Motor2_SetSpeed(pwm2);

}

/**
 * @brief  目的是根据高度来实时更新电机的PID系数
 * @note   请放在 mainTask 的 while(1) 中调用，建议 5ms 一次
 */
void Motor_PID_Update_Task(void)
{
    // 1. 高度平滑过渡 (模拟舵机运动轨迹)
    // 假设 5ms 调用一次，步长 0.3mm，约等于 60mm/s 的速度
    float step = 0.3f; 

    if (internal_current_sim_height < internal_target_avg_height)
    {
        internal_current_sim_height += step;
        if(internal_current_sim_height > internal_target_avg_height) 
            internal_current_sim_height = internal_target_avg_height;
    }
    else if (internal_current_sim_height > internal_target_avg_height)
    {
        internal_current_sim_height -= step;
        if(internal_current_sim_height < internal_target_avg_height) 
            internal_current_sim_height = internal_target_avg_height;
    }

    // 2. 线性插值计算 PID
    float ratio;
    
    // 计算当前高度在 [83, 137] 中的比例 (0.0 ~ 1.0)
    ratio = (internal_current_sim_height - H_MIN) / (H_MAX - H_MIN);
    
    // 防止计算越界
    if(ratio < 0.0f) ratio = 0.0f;
    if(ratio > 1.0f) ratio = 1.0f;

    // 3. 更新全局 PID 参数
    upright_Kp = KP_AT_MIN + ratio * (KP_AT_MAX - KP_AT_MIN);
    upright_Kd = KD_AT_MIN + ratio * (KD_AT_MAX - KD_AT_MIN);
}



