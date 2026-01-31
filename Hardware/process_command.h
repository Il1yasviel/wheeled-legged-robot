#ifndef __PROCESS_COMMAND_H 
#define __PROCESS_COMMAND_H   

#include <stdint.h>

// 声明全局变量，以便 main.c 可以读取并在串口打印
extern int Last_Servo_ID;
extern int Last_Servo_Angle;

/**
 * @brief 统一指令处理函数
 * @param cmd_str 接收到的指令字符串 (例如 "M100", "A1,90")
 */
void Process_Command(char *cmd_str);

#endif

