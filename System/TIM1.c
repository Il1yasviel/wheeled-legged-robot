#include "TIM1.h"

void TIM1_Init(void)
{
    // 1. 开启 TIM1 时钟 (TIM1 在 APB2 总线上)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    // 2. 配置时基
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; 
    
    // 计数模式: 向上计数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    // 自动重装载值: 设为最大 0xFFFF (STM32F103 TIM1 是 16位)
    // 这样它会从 0 跑到 65535 然后回滚，形成自由运行计数器
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
    
    // 时钟分频: 不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    
    // 重复计数器 (仅高级定时器TIM1/TIM8有，设为0即可)
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    
    // 3. 初始化并使能
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM1, ENABLE);
}


