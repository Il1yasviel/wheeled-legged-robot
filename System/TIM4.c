#include "TIM4.h"

//PB6 PB7  用于编码器读数
void TIM4_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;
    TIM_TimeBaseInitStructure.TIM_Period=65536-1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);

    //定时器2输入捕获通道设置
	TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);//定时器输入捕获结构体初始化 编码器模式结构体中部分参数没作用，调用初始化函数给默认值
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//通道1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//滤波器
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//此处上升沿表示编码器信号不反向 高低不反转
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//通道2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;//滤波器
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;//此处上升沿表示编码器信号不反向 高低不反转
	TIM_ICInit(TIM4, &TIM_ICInitStructure);


    //编码器模式配置 
    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Rising);//后两个参数同为编码器信号反向设置此次配置会覆盖上面，所以上面不设置也行
    
    TIM_Cmd(TIM4,ENABLE);//定时器使能

}
