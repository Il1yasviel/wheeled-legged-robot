#include "esp32_01s.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief 清空接收缓冲区
 */
void ESP8266_ClearBuffer(void) {
    memset(USART2_RxBuffer, 0, 256);
    USART2_RxFlag = 0;
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
 * @brief 开启 TCP Server 模式 (端口 8080)
 * 注意：必须在 WiFi 连接成功后调用
 */
void ESP8266_StartTCPServer(void)
{
    printf(">> [ESP] Starting TCP Server...\r\n");

    // 1. 查询本地 IP (这一步是为了让你在串口助手看到 IP，好填进 Python 里)
    // ESP 返回格式: +CIFSR:STAIP,"192.168.x.x"
    ESP8266_SendCmd("AT+CIFSR", "OK", 1000); 

    // 2. 开启多连接模式 (Server 模式必须开启 MUX=1)
    if(ESP8266_SendCmd("AT+CIPMUX=1", "OK", 1000) != 0)
    {
        printf(">> [ESP] Error: CIPMUX Failed\r\n");
        return;
    }

    // 3. 开启服务器，端口 8080
    if(ESP8266_SendCmd("AT+CIPSERVER=1,8080", "OK", 1000) == 0)
    {
        printf(">> [ESP] TCP Server Started! Port: 8080\r\n");
    }
    else
    {
        printf(">> [ESP] Error: Server Start Failed\r\n");
    }
}

