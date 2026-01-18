#ifndef __USART3_H
#define __USART3_H

#include "stm32f10x.h"
#include <stdio.h>

// --- 全局变量声明 ---
extern volatile uint8_t USART3_RxFlag; // 告诉外界：这个变量在 c 文件里定义了
extern char Serial_RxPacket[64];

void USART3_Init(void);


#endif 

