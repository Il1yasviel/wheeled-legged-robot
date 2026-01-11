#include "TIM3.h"

void TIM3_Init(void)
{   //定时器3的通道1和通道2  PA6 PA7   输出PWM到电机
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_Period=3600-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);

    //TIM3 ch1 ch2出输出比较通道配置
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出比较使能
	TIM_OCInitStructure.TIM_Pulse = 3600;		//CCR占空比
	TIM_OC1Init(TIM3, & TIM_OCInitStructure); 	                 //初始化  使用通道1
	TIM_OC2Init(TIM3, & TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
       
    //TIM3使能

    TIM_Cmd(TIM3,ENABLE);

}

