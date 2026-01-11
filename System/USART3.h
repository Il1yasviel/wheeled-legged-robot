#ifndef __USART3_H
#define __USART3_H

#include "stm32f10x.h"
#include <stdio.h>

// --- 全局变量声明 ---
extern char Serial_RxPacket[64];
extern volatile uint8_t Serial_RxFlag; 

void USART3_Init(void);


#endif 

