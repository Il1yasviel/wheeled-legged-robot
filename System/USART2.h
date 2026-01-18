#ifndef __USART2_H
#define __USART2_H

#include "stm32f10x.h"

extern char USART2_RxBuffer[256]; // 接收缓冲区
extern volatile uint8_t USART2_RxFlag; // 增加 volatile 关键字  接收完成标志位

void USART2_Init(void);
void USART2_SendString(char *str); 	

#endif 

