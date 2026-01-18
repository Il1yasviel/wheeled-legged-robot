#include "usart1.h"


//单线半双工模式
void USART1_Init(void)
{
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 串口1在APB2总线
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // 引脚A在APB2总线

    // 2. GPIO引脚配置
    GPIO_InitTypeDef GPIO_InitStructure;

	// 2. 配置 PA9 (TX)
    // 必须是 复用开漏 (AF_OD)
    // 只有设置为开漏，才能实现单线双向通信而不发生电平冲突
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 - USART1_RX (浮空输入或上拉输入)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. USART1 参数配置
    USART_InitTypeDef USART_InitStructure;
    
    USART_InitStructure.USART_BaudRate = 115200;                  // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;   // 8位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;        // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;           // 无校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    
    USART_Init(USART1, &USART_InitStructure);
	
    
    // 4. 使能串口
    USART_Cmd(USART1, ENABLE);
}


