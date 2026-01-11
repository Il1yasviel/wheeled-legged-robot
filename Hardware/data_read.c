#include "data_read.h"
#include "TIM2.h"


float pitch,roll,yaw;  //欧拉角
short gyrox,gyroy,gyroz;//陀螺仪原始数据  gyroyf俯仰角速度用于PID的Kd项
int16_t speedLeft=0;
int16_t speedRight=0;
int turnment=0; // 目标转向值，一般为0
// 目标移动速度：0为原地平衡，正数前进，负数后退
int16_t Movement = 0;

void MPU6050_Data_read(void)
{
    // 1. 尝试读取 DMP 数据
    // 不要用 while 死等，而是用 if 判断
    if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
    {
        // 返回 0 说明读取成功，可以使用 pitch, roll, yaw
        // 可以在这里打印，或者把数据发给队列
    }
    else
    {
        // 读取失败（可能是FIFO没满，或者IIC错误）
        // 可以做一个简单的错误计数，或者直接忽略，等待下一次循环
    }

    // 2. 如果你确实还需要原始陀螺仪数据
    if (MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz) == 0)
    {
        // 读取成功
    }
}

int16_t EncoderLeft_Get(void)//int16_t 计数器从0自减为65535时可自动补码为-1
{
    int16_t Temp;
    Temp=TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2,0);
    return Temp;
}

int16_t EncoderRight_Get(void)//int16_t 计数器从0自减为65535时可自动补码为-1
{
    int16_t Temp;
    Temp=TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4,0);
    return Temp;
}

void speed_read(void)
{
    speedLeft=EncoderLeft_Get();
    speedRight=EncoderRight_Get();
}
