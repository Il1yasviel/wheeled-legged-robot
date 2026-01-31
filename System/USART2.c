 #include "USART2.h"

char USART2_RxBuffer[256]; // 接收缓冲区
volatile uint8_t USART2_RxFlag = 0; // 增加 volatile 关键字  接收完成标志位
// 【新增】定义全局下标变量
volatile uint16_t USART2_RxIndex = 0; 

 
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


/**
 * @brief 像 printf 一样通过串口2发送格式化数据
 * @param fmt: 格式化字符串，例如 "Value: %d\r\n"
 */
void USART2_Printf(char *fmt, ...)
{
    char buffer[128]; // 临时缓冲区，确保不要溢出
    va_list arg_ptr;
    
    va_start(arg_ptr, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, arg_ptr);
    va_end(arg_ptr);
    
    USART2_SendString(buffer);
}
 
 

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t RxData = USART_ReceiveData(USART2);

        // 防止缓冲区溢出 (最大255)
        if (USART2_RxIndex < 255)
        {
            // 收到什么就存什么
            USART2_RxBuffer[USART2_RxIndex++] = RxData;
        }

        // 1. 判断是否为结束符 (ESP8266 发送的是 \r\n)
        // 这里的逻辑改为：收到换行符，置位标志位，但【不清零下标】
        if (RxData == '\n') // 收到换行
        {
            // 给字符串封口，方便 printf 打印
            // 注意：不要覆盖当前位置，要在下一个位置写 \0，但 index 不自增
            USART2_RxBuffer[USART2_RxIndex] = '\0'; 
            
            // 置位标志位，告诉主程序有数据来了
            USART2_RxFlag = 1; 
            
            // 【重点】这里绝对不要写 index = 0; ！！！
            // 让它继续往后存，直到主程序调用 ClearBuffer
        }
        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

