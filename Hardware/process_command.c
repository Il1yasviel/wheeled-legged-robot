#include "process_command.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 引入项目所需的头文件，确保能调用底层驱动函数
#include "motor.h"         // 包含 Motor_Set_Target_Height 等
#include "servo_motor.h"   // 包含 Servo_Move
#include "body_posture.h"  // 包含 Set_Target_Roll_Angle, cmd_xL 等变量

// ============================================================
// 外部变量引用 (告诉编译器这些变量在其他地方定义过)
// ============================================================
// 运动控制变量 (通常在 main.c 或 motor.c
#include "data_read.h"
extern  int16_t Movement;
extern  int16_t turnment;

// PID 参数 (通常在 motor.c 或 pid.c)
extern float upright_Kp, upright_Kd;
extern float cascade_speed_Kp, cascade_speed_Ki;
extern float turn_Kp, turn_Kd;
extern float mechanical_zero;

// 逆运动学坐标变量 (通常在 body_posture.h/.c)
extern float cmd_xL, cmd_yL, cmd_xR, cmd_yR;

// ============================================================
// 本文件全局变量定义
// ============================================================
int Last_Servo_ID = 0;     
int Last_Servo_Angle = 0;

// ============================================================
// 函数实现
// ============================================================
void Process_Command(char *cmd_str)
{
    // 临时变量
    char *pSeparator = NULL;
    int id = 0;
    int angle = 0;
    
    // 获取指令首字母
    char cmd = cmd_str[0];
    
    // 获取数值部分 (从第二个字符开始，用于 M, T 等简单指令)
    int val = atoi(&cmd_str[1]);

    switch (cmd)
    {
        // ===========================
        // 运动控制
        // ===========================
        case 'M': case 'm': // 速度 (例: M100)
            Movement = val;
            printf(">> [CMD] Set Movement: %d\r\n", Movement);
            break;

        case 'T': case 't': // 转向 (例: T50)
            turnment = val;
            printf(">> [CMD] Set Turnment: %d\r\n", turnment);
            break;

        case 'S': case 's': // 急停 (例: S)
            Movement = 0;
            turnment = 0;
            printf(">> [CMD] EMERGENCY STOP!\r\n");
            break;

        // ===========================
        // PID 参数在线调试
        // 格式: P<ID>,<Value> (例: P1,120.5)
        // ===========================
        case 'P': case 'p':
        {
            int pid_id = 0;
            float pid_val = 0.0f;
            
            // 解析 ID 和 浮点数值
            if (sscanf(&cmd_str[1], "%d,%f", &pid_id, &pid_val) == 2)
            {
                switch (pid_id)
                {
                    case 1: 
                        upright_Kp = pid_val; 
                        printf(">> [PID] Upright KP set to: %.3f\r\n", upright_Kp);
                        break;
                    case 2: 
                        upright_Kd = pid_val; 
                        printf(">> [PID] Upright KD set to: %.3f\r\n", upright_Kd);
                        break;
                    case 3: 
                        cascade_speed_Kp = pid_val; 
                        printf(">> [PID] Speed KP set to: %.3f\r\n", cascade_speed_Kp);
                        break;
                    case 4: 
                        cascade_speed_Ki = pid_val; 
                        printf(">> [PID] Speed KI set to: %.3f\r\n", cascade_speed_Ki);
                        break;
                    case 5: 
                        turn_Kp = pid_val; 
                        printf(">> [PID] Turn KP set to: %.3f\r\n", turn_Kp);
                        break;
                    case 6: 
                        turn_Kd = pid_val; 
                        printf(">> [PID] Turn KD set to: %.3f\r\n", turn_Kd);
                        break;
                    case 7: 
                        mechanical_zero = pid_val; 
                        printf(">> [Setting] Mechanical Zero set to: %.3f\r\n", mechanical_zero); 
                        break;
                    default:
                        printf(">> Error: Unknown PID ID (Use 1-7)\r\n");
                        break;
                }
            }
            else
            {
                printf(">> Error: Format must be P<ID>,<Val>\r\n");
            }
            break;
        }

        // ===========================
        // 逆运动学坐标控制
        // 格式: K<xL>,<yL>,<xR>,<yR>
        // ===========================
        case 'K': case 'k':
        {
            if (sscanf(&cmd_str[1], "%f,%f,%f,%f", &cmd_xL, &cmd_yL, &cmd_xR, &cmd_yR) == 4)
            {
                Motor_Set_Target_Height(cmd_yL, cmd_yR);
                printf(">> [CMD] IK Set L(%.1f,%.1f) R(%.1f,%.1f)\r\n", cmd_xL, cmd_yL, cmd_xR, cmd_yR);
            }
            else
            {
                printf(">> Error: Format must be KxL,yL,xR,yR\r\n");
            }
            break;
        }
        
        // ===========================
        // 目标翻滚角设置
        // 格式: R<Angle> (例: R5.0)
        // ===========================
        case 'R': case 'r': 
        {
            float target_angle = atof(&cmd_str[1]); 
            Set_Target_Roll_Angle(target_angle);
            printf(">> [CMD] Set Target Roll: %.2f Degree\r\n", target_angle);
            break;
        }

        // ===========================
        // 单个舵机控制
        // 格式: A<ID>,<Angle> (例: A1,90)
        // ===========================
        case 'A': case 'a':
        {
            pSeparator = strchr(cmd_str, ',');
            if (pSeparator != NULL)
            {
                id = atoi(&cmd_str[1]);
                angle = atoi(pSeparator + 1);

                // 角度限幅
                if(angle < 0) angle = 0;
                if(angle > 240) angle = 240;

                // 执行动作
                Servo_Move(id, angle, 1000);
                
                // 更新全局变量
                Last_Servo_ID = id;
                Last_Servo_Angle = angle;

                printf(">> [CMD] Servo ID:%d -> Angle:%d\r\n", id, angle);
            }
            else
            {
                printf(">> Error: Format must be A<ID>,<Angle>\r\n");
            }
            break;
        }
        
        // ===========================
        // 纯高度控制
        // 格式: H<yL>,<yR> (例: H160.5,160.5)
        // ===========================
        case 'H': case 'h':
        {
             if (sscanf(&cmd_str[1], "%f,%f", &cmd_yL, &cmd_yR) == 2)
             {
                 Motor_Set_Target_Height(cmd_yL, cmd_yR);
                 printf(">> [CMD] Height Set -> L:%.1f, R:%.1f\r\n", cmd_yL, cmd_yR);
             }
             break;
        }

        default:
            // printf(">> Unknown Command: %s\r\n", cmd_str);
            break;
    }
}

