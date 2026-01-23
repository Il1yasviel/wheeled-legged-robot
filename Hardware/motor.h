#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "data_read.h"
#include "body_posture.h"


extern float upright_Kp;
extern float upright_Kd;
extern float cascade_speed_Kp;
extern float cascade_speed_Ki;
extern float turn_Kp;
extern float turn_Kd;
extern float mechanical_zero;


extern int16_t debug_pwm1;
extern int16_t debug_pwm2;


void control_motor(void);

void Motor_PID_Update_Task(void);
#endif 

