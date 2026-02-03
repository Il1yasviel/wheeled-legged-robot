#ifndef __USART2_H
#define __USART2_H

#include "stm32f10x.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>


// 【新增】声明模式开关
//extern volatile uint8_t USART2_Lock_Mode; 

extern char USART2_RxBuffer[256]; // 接收缓冲区
extern volatile uint8_t USART2_RxFlag; // 增加 volatile 关键字  接收完成标志位

extern uint8_t USART2_RxIndex;
//extern volatile uint8_t Data_Locked;

void USART2_Init(void);
void USART2_SendString(char *str);
void USART2_Printf(char *fmt, ...);

#endif 

