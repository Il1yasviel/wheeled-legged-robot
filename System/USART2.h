#ifndef __USART2_H
#define __USART2_H

#include "stm32f10x.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>



extern char USART2_RxBuffer[256]; // 接收缓冲区
extern volatile uint8_t USART2_RxFlag; // 增加 volatile 关键字  接收完成标志位
// 【新增】把下标变量公开出来，方便在清除缓存时归零
extern volatile uint16_t USART2_RxIndex; 


void USART2_Init(void);
void USART2_SendString(char *str);
void USART2_Printf(char *fmt, ...);

#endif 

