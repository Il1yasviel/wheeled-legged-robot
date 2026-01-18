#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "data_read.h"

extern float upright_Kp;
extern float upright_Kd;
extern float cascade_speed_Kp;
extern float cascade_speed_Ki;
extern float turn_Kp;
extern float turn_Kd;
extern float mechanical_zero;

void control_motor(void);


#endif 

