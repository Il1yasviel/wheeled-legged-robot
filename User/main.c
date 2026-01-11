#include "stm32f10x.h"      // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "GPIO.h"
//#include "USART1.h"
#include "USART3.h"
#include "inv_mpu.h"
#include "data_read.h"
#include "OLED.h"
#include "TIM1.h"
#include "TIM2.h"
#include "TIM3.h"
#include "TIM4.h"
#include "motor.h"
#include <stdlib.h>
#include "delay.h"
#include "IWDG.h" 
#include "servo_motor.h"
#include <string.h>       // 必须包含，用于查找逗号 strchr
#include "kinematic_inverse.h" 
#include <stdio.h>             // sscanf 需要这个

// 任务句柄
TaskHandle_t infTaskHandler;
TaskHandle_t mainTaskHandler;

/**
 * @brief 串口指令解析与数据打印任务
 * @param arg FreeRTOS任务参数
 */
void messageTask(void *arg)
{
    // 任务启动提示
    printf("USART3 Control Task Started! \r\n");
    printf("Commands: M(Speed), T(Turn), S(Stop) \r\n");
	
	// --- 【新增】初始化逆运动学默认值 ---
    IK_Init(); 
	
	// --- 【新增】用于接收坐标的临时变量 ---
    float cmd_xL, cmd_yL, cmd_xR, cmd_yR;
    
    // --- 【修改点1】定义变量来“记住”最后一次舵机的状态 ---
    // 必须定义在 while 循环外面，否则每次循环都会被清零
    int Last_Servo_ID = 0;     // 默认ID 0
    int Last_Servo_Angle = 0;  // 默认角度 0

    // 解析用的临时变量
    char *pSeparator = NULL; 
    int id = 0;
    int angle = 0;
	
    while(1) // 必须是死循环
    {
        // --- 1. 指令解析逻辑 ---
        if (Serial_RxFlag == 1)
        {
            // 解析指令：第一个字母决定功能
            char cmd = Serial_RxPacket[0];
			
            
            // atoi 从字符串第二个字符开始解析数字
            // 如果只有字母没有数字，atoi 返回 0
            int val = atoi(&Serial_RxPacket[1]); 
            
            switch (cmd)
            {
                case 'M': case 'm': // 调整目标速度 (例: M100)
                    Movement = val;
                    printf(">> Set Movement to: %d\r\n", Movement);
                    break;
                    
                case 'T': case 't': // 调整目标转向 (例: T50)
                    turnment = val;
                    printf(">> Set Turnment to: %d\r\n", turnment);
                    break;
                    
                case 'S': case 's': // 急停指令 (例: S)
                    Movement = 0;
                    turnment = 0;
                    printf(">> EMERGENCY STOP!\r\n");
                    break;
				
				
				
				                // ---------------------------------------------------------
                // 【新增】逆运动学坐标指令
                // 协议格式: K<xL>,<yL>,<xR>,<yR>
                // 例如: K0,90,0,90  (两腿都在 (0,90) 位置)
                // ---------------------------------------------------------
                case 'K': case 'k':
                {
                    // 使用 sscanf 解析 4 个浮点数
                    // %f 对应 float, 逗号是分隔符
                    // &Serial_RxPacket[1] 跳过开头的 'K'
                    int parsed_count = sscanf(&Serial_RxPacket[1], "%f,%f,%f,%f", 
                                              &cmd_xL, &cmd_yL, &cmd_xR, &cmd_yR);

                    if(parsed_count == 4) // 确保成功解析了4个数字
                    {
                        // 1. 调用核心解算函数
                        IK_Compute(cmd_xL, cmd_yL, cmd_xR, cmd_yR);
						
					   // 5. 执行动作 (假设动作时间给 1000ms)
					   Servo_Move(3, Robot_IK.Angle_Servo_Left_Rear, 1000); 
                        Servo_Move(4, Robot_IK.Angle_Servo_Left_Front, 1000); 
					   
				
						
                        // 2. 打印确认收到的坐标
                        printf(">> IK Input: L(%.1f, %.1f) R(%.1f, %.1f)\r\n", 
                               cmd_xL, cmd_yL, cmd_xR, cmd_yR);
						
						// 3. 【修改这里】打印对比数据
						// 格式说明：
						// Math: LA(左Alpha) LB(左Beta) ...
						// Servo: LF(左前) LR(左后) ...
						printf(">> Math: LA:%d LB:%d RA:%d RB:%d || Servo: LF:%d LR:%d RF:%d RR:%d\r\n",
							   // --- 第一组：纯数学角度 ---
							   Robot_IK.Math_Angle_Alpha_Left,
							   Robot_IK.Math_Angle_Beta_Left,
							   Robot_IK.Math_Angle_Alpha_Right,
							   Robot_IK.Math_Angle_Beta_Right,
							   
							   // --- 第二组：计算后的舵机角度 ---
							   Robot_IK.Angle_Servo_Left_Front,  // LF
							   Robot_IK.Angle_Servo_Left_Rear,   // LR
							   Robot_IK.Angle_Servo_Right_Front, // RF (记得确保右腿公式也改好了)
							   Robot_IK.Angle_Servo_Right_Rear   // RR
								);
                    }
                    else
                    {
                        printf(">> Error: Format must be KxL,yL,xR,yR\r\n");
                    }
                    break;
                }

				
				
				                // ====================================================
                // 【新增】舵机控制指令
                // 协议格式: A<ID>,<Angle>  (例如: A4,90)
                // ====================================================
                case 'A': case 'a':
                {
                    // 1. 查找逗号的位置
                    pSeparator = strchr(Serial_RxPacket, ',');
                    
                    if (pSeparator != NULL)
                    {
                        // 2. 解析 ID 
                        // &Serial_RxPacket[1] 是 'A' 后面的字符
                        // atoi 遇到非数字(这里是逗号)会自动停止
                        id = atoi(&Serial_RxPacket[1]);
                        
                        // 3. 解析 角度 
                        // pSeparator 是逗号的地址，+1 就是逗号后面那个数字的地址
                        angle = atoi(pSeparator + 1);

                        // 4. 角度限幅 (保护舵机不被烧毁)
                        if(angle < 0) angle = 0;
                        if(angle > 240) angle = 240; 

                        // 5. 执行动作 (假设动作时间给 1000ms)
                        Servo_Move(id, angle, 1000); 
						
						// --- 【修改点2】更新记忆变量 ---
                        Last_Servo_ID = id;
                        Last_Servo_Angle = angle;

                        
                        printf(">> Servo ID:%d -> Angle:%d\r\n", id, angle);
                    }
                    else
                    {
                        printf(">> Error: Format must be A<ID>,<Angle> (e.g. A1,90)\r\n");
                    }
                    break;
                }

				
                
                default:
                    printf(">> Unknown Command: %s\r\n", Serial_RxPacket);
                    break;
            }
            
            // 处理完后务必清空标志位，准备接收下一帧
            Serial_RxFlag = 0; 
			
        }

        // --- 【修改点3】在打印中增加舵机信息 (SID: 舵机ID, Ang: 角度) ---
        // P:姿态 | M:速度 | T:转向 | SID:舵机ID | Ang:舵机角度
//        printf("P:%.2f | M:%d | T:%d | SID:%d | Ang:%d\r\n", 
//                pitch, Movement, turnment, Last_Servo_ID, Last_Servo_Angle);

        // --- 3. 延时 ---
        // 40ms 刷新一次打印，既能看清数据，又不会占满串口带宽
        vTaskDelay(40); 
    }
}

void mainTask(void *arg)
{
	TickType_t xLastWakeTime;
    const TickType_t xFrequency = 5; // 定义周期为 5ms (对应 FreeRTOS configTICK_RATE_HZ 为 1000)	
	
	// 定义一个静态变量作为分频计数器
    static uint8_t task_tick = 0; 
	
	    // 2. 初始化时间变量
    xLastWakeTime = xTaskGetTickCount();
	//初始化舵机的位置
	servo_motor_position_Init();
	
	    // 2. 一切准备就绪后，开启看门狗！
    IWDG_Init(); 
    printf("IWDG Enabled.\r\n");
	
	while(1)
    {
		// --- A. 进入临界区或绝对时间控制开始 ---
		// vTaskDelayUntil 会确保从上一次唤醒时刻开始，严格过了 5ms 后才唤醒
		// 这样可以保证控制频率严格稳定在 200Hz
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		
		// 2. 计数器自增
        task_tick++;
		
		
		// 3. 【分频逻辑】每 10ms 读取一次 MPU6050  必须大于10ms，否则太快读取不到MPU的数据
        if(task_tick >= 2) 
        {
            MPU6050_Data_read(); // 读取姿态 (此时 DMP 刚好有数据)
            task_tick = 0;       // 清零计数器
        }

		speed_read();
		control_motor();
		
		// --- 新增：喂狗 ---
        // 只要程序能运行到这里，说明控制逻辑正常，没有死机
        IWDG_Feed(); 

    }
}

int main(void)
{
	My_GPIO_Init();
	USART1_Init();
	USART3_Init();
	TIM1_Init();
	TIM2_Init();
	TIM3_Init();
	TIM4_Init();
	
//	delay_ms(200); // 上电后稍作等待
//	mpu_dmp_init();
//	
//	// --- MPU6050 初始化带超时重启机制 ---
//    
//    int retry_count = 0;
//    const int MAX_RETRIES = 25; // 设置最大重试次数：25次 * 200ms = 5秒
//	
//	// 此时调度器未启动，函数内部会自动调用 delay_us 进行忙等待
//    while(mpu_dmp_init() != 0) 
//    {        
//		retry_count++;
//        // 判断是否超时
//        if(retry_count >= MAX_RETRIES)
//        {
//            printf(">> MPU Error Timeout! System Resetting Now... <<\r\n");
//            
//            // 重要：在复位前给一点点延时，确保串口把上面那句话发完，否则你看不到提示就复位了
//            delay_ms(100); 
//            
//            // --- 系统软复位函数 ---
//            // 这是 CMSIS 标准库自带的，不需要额外 include，在 stm32f10x.h 里最终会包含
//            NVIC_SystemReset(); 
//        }
//        
//        delay_ms(200); // 每次失败等待 200ms
//    }
//	



    
	
    // 4. 创建任务
    // 参数：任务函数, 任务名, 堆栈深度, 参数, 优先级, 任务句柄
    xTaskCreate(messageTask, "print_Inf", 1024, NULL, 2, &infTaskHandler);
    xTaskCreate(mainTask, "main_Task", 1024, NULL, 4, &mainTaskHandler);
    // 5. 启动调度器
    vTaskStartScheduler();

    // 正常情况下，程序不会运行到这里
    while (1)
    {
        
    }
}


