#ifndef __ESP32_01S_H
#define __ESP32_01S_H		

#include "stm32f10x.h"
#include "USART2.h"
#include "delay.h"

// WiFi 信息
#define WIFI_SSID     "CMCC-E4EN"
#define WIFI_PASS     "KCQ4WUNE"

// 函数声明
void ESP8266_Init(void);
void ESP8266_ClearBuffer(void);
uint8_t ESP8266_SendCmd(char *cmd, char *res, uint32_t timeout_ms);
uint8_t ESP8266_ConnectWiFi(void);
void ESP8266_StartTCPServer(void);

#endif

