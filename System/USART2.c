 #include "USART2.h"

char USART2_RxBuffer[256]; // 接收缓冲区
volatile uint8_t USART2_RxFlag = 0; // 增加 volatile 关键字  接收完成标志位
 
 void USART2_Init(void)
 {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. 开启时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能 GPIOA 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 使能 USART2 时钟

    /* 2. GPIO 配置 */
    // PA2 (USART2_TX) 配置为 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA3 (USART2_RX) 配置为 浮空输入或上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 3. USART2 参数配置 */
    USART_InitStructure.USART_BaudRate = 115200;            // 建议 115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 开启接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* 5. 使能串口 */
    USART_Cmd(USART2, ENABLE);

 }


 
 /**
 * @brief 通过串口2发送字符串
 * @param str: 要发送的字符串
 */
void USART2_SendString(char *str)
{
    while (*str)
    {
        // 发送一个字节
        USART_SendData(USART2, (uint8_t)*str++);
        
        // 等待发送寄存器为空 (TXE)
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
}
 
 

void USART2_IRQHandler(void)
{
	static uint16_t index = 0;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t RxData = USART_ReceiveData(USART2);

        // 1. 判断是否为结束符
        if (RxData == '\r' || RxData == '\n')
        {
            if (index > 0) 
            {
                USART2_RxBuffer[index] = '\0'; // 封口
                USART2_RxFlag = 1;                 // 置位
                index = 0;                     // 清零下标
				

            }
        }

        else
        {
            // 2. 正常接收
           if (index < 254) 
            {
                USART2_RxBuffer[index++] = RxData;
            }
        }

            USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

