#include "IWDG.h"

/**
 * @brief 初始化独立看门狗
 * 超时时间计算: Tout = (预分频 * 重装载值) / 40kHz
 * 设定为 1秒: 1s = (64 * 625) / 40000
 */
void IWDG_Init(void)
{
    // 1. 取消寄存器写保护，允许操作 IWDG_PR 和 IWDG_RLR
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    
    // 2. 设置预分频系数为 64
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    
    // 3. 设置重装载值 (0~4095)
    // 40kHz / 64 = 625Hz，即每秒数 625 下
    // 所以填 625 就是 1 秒超时
    IWDG_SetReload(625); 
    
    // 4. 重载计数器（喂一次狗，加载初始值）
    IWDG_ReloadCounter();
    
    // 5. 使能看门狗
    IWDG_Enable();
}

/**
 * @brief 喂狗函数
 * 必须在 1秒 内调用一次，否则系统复位
 */
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}


