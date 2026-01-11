#ifndef __MPUIIC_H
#define __MPUIIC_H

#include "stm32f10x.h"

// --- 方向设置 (直接操作 CRH 寄存器，保持不变) ---
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(uint32_t)8<<4;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(uint32_t)3<<4;}

// --- IO操作函数 (使用位带映射地址，支持直接赋值操作) ---
// 逻辑：将 GPIOB_ODR 和 IDR 的位直接映射到一个 32 位的地址，实现 L-value 赋值

// SCL -> PB8 (对应 GPIOB_ODR 的第 8 位)
#define MPU_IIC_SCL    *(volatile uint32_t *)(0x422181A0) 

// SDA -> PB9 (对应 GPIOB_ODR 的第 9 位)
#define MPU_IIC_SDA    *(volatile uint32_t *)(0x422181A4) 

// READ_SDA -> PB9 (对应 GPIOB_IDR 的第 9 位)
#define MPU_READ_SDA   *(volatile uint32_t *)(0x42218124)

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















