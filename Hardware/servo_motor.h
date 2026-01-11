#ifndef __SERVO_MOTOR_H
#define __SERVO_MOTOR_H

#include "USART1.h"
#include "kinematic_inverse.h"
#include "delay.h"

void Servo_Move(uint8_t id, uint8_t angle, uint16_t time_ms);

void servo_motor_position_Init(void);


#endif 

