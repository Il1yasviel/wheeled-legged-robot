#ifndef __DATA_READ_H
#define __DATA_READ_H

#include "inv_mpu.h"
#include <stdio.h>
#include "mpu6050.h"


extern float pitch,roll,yaw;  //欧拉角
extern short gyrox,gyroy,gyroz;//陀螺仪原始数据
extern int16_t speedLeft;
extern int16_t speedRight;
extern int turnment; // 目标转向值，一般为0
// 目标移动速度：0为原地平衡，正数前进，负数后退
extern int16_t Movement;


void MPU6050_Data_read(void);
void speed_read(void);

int16_t EncoderLeft_Get(void);
int16_t EncoderRight_Get(void);

#endif 

