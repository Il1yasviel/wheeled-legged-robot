#include "esp32_01s.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief 清空接收缓冲区
 */
void ESP8266_ClearBuffer(void) {
    memset(USART2_RxBuffer, 0, 256);
    USART2_RxFlag = 0;
    
    // 【新增】必须在这里手动把下标归零
    USART2_RxIndex = 0; 
}

/**
 * @brief 发送 AT 指令并等待特定响应
 */
uint8_t ESP8266_SendCmd(char *cmd, char *res, uint32_t timeout_ms) {
    ESP8266_ClearBuffer();
    USART2_SendString(cmd);
    USART2_SendString("\r\n"); // ESP8266 必须以 \r\n 结尾

    while (timeout_ms--) {
        if (USART2_RxFlag) {
            // 在缓冲区中寻找是否有期望的字符串 (如 "OK")
            if (strstr(USART2_RxBuffer, res) != NULL) return 0; 
            USART2_RxFlag = 0; // 如果不是我们要的，继续等
        }
        delay_ms(1); // 这里的 Delay_ms 需要你自己工程里的延时函数
    }
    return 1; // 超时失败
}

/**
 * @brief 连接指定 WiFi
 */
uint8_t ESP8266_ConnectWiFi(void) {
    // 1. 设置 Station 模式
    if (ESP8266_SendCmd("AT+CWMODE=1", "OK", 2000)) return 1;
    
    // 2. 连接 WiFi
    // 这里的 WIFI_SSID 和 WIFI_PASS 定义在 esp8266.h 中
    char cmd[128];
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", "CMCC-E4EN", "KCQ4WUNE");
    
    if (ESP8266_SendCmd(cmd, "WIFI GOT IP", 10000)) return 2;
    
    return 0; // 成功
}



/**
 * @brief 开启 TCP Server 并打印 IP (简化版)
 * 现在中断支持累积数据了，我们可以直接死等 1 秒，然后一次性打印所有内容
 */
void ESP8266_StartTCPServer(void)
{
    printf(">> [ESP] Config TCP Server...\r\n");
    
    // 刚连上 WiFi，稍微缓一下
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    // ========================================================
    // 1. 获取并打印 IP 地址
    // ========================================================
    
    printf(">> [ESP] Requesting IP Info...\r\n");
    ESP8266_ClearBuffer();             // 下标归零
    USART2_SendString("AT+CIFSR\r\n"); // 发送查询
    
    // 直接死等 1 秒！
    // 因为现在中断会把 IP、MAC、OK 全部攒在 Buffer 里，不会覆盖了
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    if(USART2_RxFlag)
    {
        // 打印 Buffer 里的所有内容
        // 你会看到类似：
        // +CIFSR:STAIP,"192.168.x.x"
        // +CIFSR:STAMAC,"xx:xx:xx"
        // OK
        printf("==========================================\r\n");
        printf(">> ESP8266 RESPONSE: \r\n%s\r\n", USART2_RxBuffer);
        printf("==========================================\r\n");
    }
    else
    {
        printf(">> [ESP] Warning: No response!\r\n");
    }

    // ========================================================
    // 2. 开启 Server
    // ========================================================
    // 后面的逻辑保持不变
    if(ESP8266_SendCmd("AT+CIPMUX=1", "OK", 1000) != 0)
    {
        printf(">> [ESP] Error: CIPMUX Failed\r\n");
    }
    
    if(ESP8266_SendCmd("AT+CIPSERVER=1,8080", "OK", 1000) == 0)
    {
        printf(">> [ESP] TCP Server Started! Port: 8080\r\n");
    }
    else
    {
        printf(">> [ESP] Error: Server Start Failed\r\n");
    }
}

