#include "delay.h"

void delay_us(uint32_t nus)
{
    uint16_t start_tick, current_tick, elapsed_tick;
    uint32_t total_elapsed = 0;

    if (nus == 0) return;

    // 获取当前计数值
    start_tick = TIM1->CNT;

    while (total_elapsed < nus)
    {
        current_tick = TIM1->CNT;

        // 计算经过的时间 (处理 16位 溢出)
        // 核心逻辑: 即使 current < start (发生了溢出)，
        // (uint16_t)(current - start) 依然能得到正确的差值
        elapsed_tick = current_tick - start_tick;

        // 累加时间
        if (elapsed_tick > 0)
        {
            total_elapsed += elapsed_tick;
            start_tick = current_tick; // 更新基准点
        }
    }
}

void delay_ms(uint32_t nms)
{
    // 确保调度器已经启动
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        // 计算需要的 Tick 数
        // configTICK_RATE_HZ 在 FreeRTOSConfig.h 中定义，通常是 1000
        TickType_t ticks = nms / portTICK_PERIOD_MS;
        
        // 至少延时 1 个 Tick
        if (ticks == 0) ticks = 1;
        
        vTaskDelay(ticks);
    }
    else
    {
        // 如果调度器还没启动 (比如在初始化阶段调用)，
        // 则退化为使用 delay_us 进行忙等待
        delay_us(nms * 1000);
    }
}

