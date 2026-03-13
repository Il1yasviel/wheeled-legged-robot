#include "motor.h"
#include <math.h>

//机械零点
float mechanical_zero=-5.3f;//0.3
//直立环
float upright_Kp=400.0f;  //原150，乘上1.5倍
float upright_Kd=-1.8f;       //原0.25，也乘上1.5倍
//速度环
// 修改后的速度环参数（已转换）
float cascade_speed_Kp =0.3f;//0.267f; 
float cascade_speed_Ki =0.001f;//0.00133f;
//转向环
float turn_Kp=-10.5f;   //极性负 期望小车转向，正反馈
float turn_Kd=0.4f;    //极性正抑制小车转向，负反馈

float turn_limit = 1000.0f; // 建议值在 300 到 800 之间，根据电机动力调整


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


//发现一个问题，就是停止的时候总是会滑翔，发现是腿部助跑后，归位的太慢了，然后就在停止的时候让轮子加速跑，从而加快腿部归到原位，从而减少滑翔的影响
//例如向前加速的时候，腿部向后蹬，重心在前面，然后突然停止的时候，腿部会向前回归到原位，速度要是慢的话，就会一直滑翔。这时候轮子和腿一直向前冲，
//让重心快速落到机器人的中心，达到快速停止的效果。
// 速度环：直接输出目标角度
float speed_ring(int16_t encoder_left, int16_t encoder_right)
{  
    static float Encoder_Integral, Encoder;
    static float current_movement = 0.0f; 
    
    float current_speed = (encoder_left + encoder_right);
    float speed_err;

    float step; 
    
    // 判断指令是否在“对抗”当前的车身运动（主动反方向打摇杆救车）
    // 如果你在往前滑 (current_speed > 2)，但摇杆往后打 (Movement < 0)
    // 或者你在往后溜 (current_speed < -2)，但摇杆往前打 (Movement > 0)
    if ((Movement > 0 && current_speed < -2) || (Movement < 0 && current_speed > 2))
    {
        // 直接给出极大的步长，没有延迟，瞬间出腿抵消！
        step = 80.0f; 
    }
    else
    {

        step = 80.0f; 
    }

    // 执行斜坡运算
    if (current_movement < Movement) current_movement += step;
    else if (current_movement > Movement) current_movement -= step;
    
    // 消除逼近误差
    if (fabs((int)(Movement - current_movement)) <= step) 
    {
        current_movement = Movement;
    }

    // 1. 常规行驶时的速度偏差
    speed_err = current_speed - current_movement; 
    
    // 只有当遥控器归零，且平滑速度也完全降到 0 时，才启动暴力手刹
    if (Movement == 0 && current_movement == 0)
    {
        if (current_speed > 10 || current_speed < -10)
        {
            speed_err = current_speed * 45.0f; 
        }
        else if (current_speed > 5 || current_speed < -5)
        {
            speed_err = current_speed * 10.0f; 
        }
        else
        {
            if (current_speed >= -5 && current_speed <= 5)
            {
                speed_err = 0; 
                Encoder_Integral *= 0.9f; 
            }
        }
    }


    // 一阶低通滤波器
    Encoder *= 0.7f;                
    Encoder += speed_err * 0.3f;    

    // 积分累加
    Encoder_Integral += Encoder;   
    
    // 积分限幅 (紧紧勒住，防止憋大招暴走)
    if(Encoder_Integral > 500) Encoder_Integral = 500;
    if(Encoder_Integral < -500) Encoder_Integral = -500; 

    // 计算出直立环的期望目标角度
    float target_angle = (cascade_speed_Kp * Encoder) + (cascade_speed_Ki * Encoder_Integral);
    
    // 限幅保护
    if(target_angle > 5.0f)  target_angle = 5.0f;  
    if(target_angle < -5.0f) target_angle = -5.0f; 

    if((pitch >= 80) || (pitch <= -80)) Encoder_Integral = 0;    

    return target_angle; 
}


// 转向环：开环前馈(提供动力) + 陀螺仪阻尼(防止疯转)
int16_t turn_ring(float Target_Turn_Speed, float Gyro_Z)
{
    int16_t turn_pwm;
    // 这样当 Target_Turn_Speed 为 0 时，加上陀螺仪的反馈，就能产生抵抗扭转的阻尼力
    turn_pwm = (turn_Kp * Target_Turn_Speed) + (turn_Kd * Gyro_Z); 
    
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
    if(pwm1 >= 0) pwm1 += 150; else pwm1 -= 150;    //电池从3S换成了2S，死区从140修改成250
    if(pwm2 >= 0) pwm2 += 150; else pwm2 -= 150;


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



