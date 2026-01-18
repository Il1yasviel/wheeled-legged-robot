#include "USART3.h"
#include <stdio.h> // 确保包含这个头文件以使用 printf

char Serial_RxPacket[64]; // 接收缓冲区
volatile uint8_t USART3_RxFlag = 0; // 增加 volatile 关键字  接收完成标志位

void USART3_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位字长
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3, ENABLE);
}

// 重定向 fputc 函数
// 当你调用 printf 时，标准库最终会通过这个 fputc 函数一个字符一个字符地发送数据
int fputc(int ch, FILE *f)
{
    // 等待发送寄存器为空 (TXE = 1)
    // 如果不等待，数据发送太快会产生覆盖，导致串口乱码
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    
    // 将字符交给 USAR3 外设发送
    USART_SendData(USART3, (uint8_t)ch);
    
    return ch;
}

// 替换 USART3.c 中的 USART3_IRQHandler 函数
void USART3_IRQHandler(void)
{
    // 依然使用你原来的静态变量名
    static uint8_t pRxPacket = 0; 
    
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t RxData = USART_ReceiveData(USART3);
        
        // 1. 判断是否为结束符
        if (RxData == '\r' || RxData == '\n')
        {

            if (pRxPacket > 0) 
            {
                Serial_RxPacket[pRxPacket] = '\0'; // 封口
                USART3_RxFlag = 1;                 // 置位
                pRxPacket = 0;                     // 清零下标
				

            }
        }
        else 
        {
            // 2. 正常接收
            if (pRxPacket < 63)
            {
                Serial_RxPacket[pRxPacket++] = RxData;
            }
        }
        
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}


