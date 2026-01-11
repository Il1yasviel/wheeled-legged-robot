#include "GPIO.h"

void My_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);


    //PA4 PA5 电机刹车引脚初始化 0刹车 1释放
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);



    //PB0 PB1 电机方向引脚初始化 0反转 1正转(实际方向根据电机安装位置进行调整)
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB,&GPIO_InitStructure);




    //用于I2C通信  B8是SCL  B9是SDA   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	 // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 

    GPIO_SetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9);						




}

