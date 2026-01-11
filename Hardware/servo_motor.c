#include "servo_motor.h"


/* * ---------------------------------------------------------
 * 新增函数 1: 底层发送函数
 * 作用: 遍历数组，将每一个字节通过      USAR1      发送出去
 * ---------------------------------------------------------
 */
void Servo_Transmit(uint8_t *packet, uint8_t length)
{
    for (int i = 0; i < length; i++)
    {
        // 1. 等待发送寄存器为空 (TXE = 1)
        // 重要：如果不等待，上一个字节还没发完，下一个字节写入会覆盖数据，导致丢包
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        
        // 2. 发送当前字节
        USART_SendData(USART1, packet[i]);
    }
}


//    command_packet[2] = 0x00; // 是广播ID
void Servo_Move(uint8_t id, uint8_t angle, uint16_t time_ms)
{
	
	//限定舵机角度范围，也就是机械臂运动范围
	
    uint8_t command_packet[10];

    command_packet[0] = 0xFA; // 帧头1
    command_packet[1] = 0xAF; // 帧头2
    command_packet[2] = id;   // 舵机ID
    command_packet[3] = 0x01; // 命令: 转动
    command_packet[4] = angle; // 参数1: 目标角度
    command_packet[5] = (uint8_t)(time_ms / 20); // 参数2: 运动时间
    command_packet[6] = 0x00; // 锁定时间高位
    command_packet[7] = 0x00; // 锁定时间低位

    uint8_t checksum = 0;
    for (int i = 2; i <= 7; i++) {
        checksum += command_packet[i];
    }
    command_packet[8] = checksum; // 校验和
    command_packet[9] = 0xED;     // 帧尾

    // 调用上面写好的发送函数
    Servo_Transmit(command_packet, sizeof(command_packet));
	delay_ms(50);
}


//舵机初始化到30度
void servo_motor_position_Init(void)
{
	Servo_Move(3, 120, 1000);
	 delay_ms(50);
	Servo_Move(4, 120, 1000);
	 delay_ms(50);
}
