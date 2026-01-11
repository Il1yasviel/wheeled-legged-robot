#ifndef __DELAY_H
#define __DELAY_H 			

#include "stm32f10x.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "TIM1.h"

void delay_ms(uint32_t nms);
void delay_us(uint32_t nus);

#endif

