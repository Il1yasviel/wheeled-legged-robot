#include "stm32f10x.h"      // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "GPIO.h"
#include "USART1.h"
#include "USART2.h"
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
#include "esp32_01s.h"

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
	
	
	//AT指令
	uint8_t status = ESP8266_ConnectWiFi();
	
	    if (status == 0)
    {
        printf("WiFi Connect Success!\r\n");
    }
    else
    {
        // 如果失败，status 的值能告诉你死在哪一步
        // 1: AT指令无响应; 2: 连接AP失败
        printf("WiFi Connect Failed! Error Code: %d\r\n", status);
    }
	
    while(1) 
    {		
		if (USART2_RxFlag == 1)
		{
			// 如果收到 ESP32 发来的任何透传数据，直接打印到串口3看
			printf("Receive: %s\r\n", USART2_RxBuffer);

			// 处理完记得清空标志位
			USART2_RxFlag = 0; 

		}
			
		
		
        // --- 1. 指令解析逻辑 ---
        if (USART3_RxFlag == 1)
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
				
				
				
				// ====================================================
				// 【新增】PID 参数在线调试指令
				// 协议格式: P<ID>,<Value>  (例如: P1,120.5 修改直立环KP)
				// ====================================================
                case 'P': case 'p':
                {
                    int pid_id = 0;
                    float pid_val = 0.0f;
                    
                    // 使用 sscanf 解析：整数ID + 逗号 + 浮点数值
                    int parsed_cnt = sscanf(&Serial_RxPacket[1], "%d,%f", &pid_id, &pid_val);

                    if (parsed_cnt == 2) // 确保成功解析了 ID 和 数值 两个数据
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
							
							// --- 【新增】机械零点调节 ---
                            case 7: 
                                mechanical_zero = pid_val; 
                                printf(">> [Setting] Mechanical Zero set to: %.3f\r\n", mechanical_zero); 
                                break;
							
                            default:
                                printf(">> Error: Unknown PID ID (Use 1-6)\r\n");
                                break;
                        }
                    }
                    else
                    {
                        printf(">> Error: Format must be P<ID>,<Val>\r\n");
                        printf(">> IDs: 1:U_Kp, 2:U_Kd, 3:V_Kp, 4:V_Ki, 5:T_Kp, 6:T_Kd\r\n");
                    }
                    break;
                }
				
				
				// ---------------------------------------------------------
                // 【修改后】逆运动学坐标指令 - 支持双腿/四舵机 + 数学角度显示
                // 协议格式: K<xL>,<yL>,<xR>,<yR>
                // ID映射: 左前=1, 左后=2, 右前=3, 右后=4
                // ---------------------------------------------------------
				case 'K': case 'k':
                {
                    // 使用 sscanf 解析 4 个浮点数
                    int parsed_count = sscanf(&Serial_RxPacket[1], "%f,%f,%f,%f", 
                                              &cmd_xL, &cmd_yL, &cmd_xR, &cmd_yR);

                    if(parsed_count == 4) // 确保成功解析了4个数字
                    {
                        // 1. 调用核心解算函数
                        // 注意：根据刚才修改的 IK_Compute，即使返回 0，Robot_IK 里也已经存入了计算出的“错误”数值
                        uint8_t is_safe = IK_Compute(cmd_xL, cmd_yL, cmd_xR, cmd_yR);
                        
                        // 打印出刚刚算出的数学角度和舵机角度
                        printf(">> [DEBUG] Input: L(%.0f, %.0f) R(%.0f, %.0f)\r\n", cmd_xL, cmd_yL, cmd_xR, cmd_yR);
                        
                        printf(">> LEFT : Math[A:%d, B:%d] -> Servo[ID2:%d, ID1:%d]\r\n", 
                               Robot_IK.Math_Angle_Alpha_Left, 
                               Robot_IK.Math_Angle_Beta_Left,
                               Robot_IK.Angle_Servo_Left_Rear,  // 对应 Alpha
                               Robot_IK.Angle_Servo_Left_Front  // 对应 Beta
                              );
                              
                        printf(">> RIGHT: Math[A:%d, B:%d] -> Servo[ID4:%d, ID3:%d]\r\n", 
                               Robot_IK.Math_Angle_Alpha_Right, 
                               Robot_IK.Math_Angle_Beta_Right,
                               Robot_IK.Angle_Servo_Right_Rear, 
                               Robot_IK.Angle_Servo_Right_Front
                              );

                        // 3. 【安全开关】只有返回 1 (安全) 才允许动舵机
                        if (is_safe == 1)
                        {
                            Servo_Move(1, Robot_IK.Angle_Servo_Left_Front, 1000); 
                            Servo_Move(2, Robot_IK.Angle_Servo_Left_Rear, 1000);  
                            Servo_Move(3, Robot_IK.Angle_Servo_Right_Front, 1000);
                            Servo_Move(4, Robot_IK.Angle_Servo_Right_Rear, 1000); 
                            printf(">> [STATUS] Safe -> Executing Move.\r\n");
                        }
                        else
                        {
                            // 4. 如果不安全，明确提示
                            // 此时舵机不动，但你已经在上面看到了算出来的越界数值
                            printf(">> [STATUS] UNSAFE LIMITS! Motors LOCKED.\r\n");
                        }
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
            USART3_RxFlag = 0; 
			
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	My_GPIO_Init();
	USART1_Init();
	USART2_Init();
	USART3_Init();
	TIM1_Init();
	TIM2_Init();
	TIM3_Init();
	TIM4_Init();
	
	delay_ms(200); // 上电后稍作等待
	mpu_dmp_init();
	
	// --- MPU6050 初始化带超时重启机制 ---
    
    int retry_count = 0;
    const int MAX_RETRIES = 25; // 设置最大重试次数：25次 * 200ms = 5秒
	
	// 此时调度器未启动，函数内部会自动调用 delay_us 进行忙等待
    while(mpu_dmp_init() != 0) 
    {        
		retry_count++;
        // 判断是否超时
        if(retry_count >= MAX_RETRIES)
        {
            printf(">> MPU Error Timeout! System Resetting Now... <<\r\n");
            
            // 重要：在复位前给一点点延时，确保串口把上面那句话发完，否则你看不到提示就复位了
            delay_ms(100); 
            
            // --- 系统软复位函数 ---
            // 这是 CMSIS 标准库自带的，不需要额外 include，在 stm32f10x.h 里最终会包含
            NVIC_SystemReset(); 
        }
        
        delay_ms(200); // 每次失败等待 200ms
    }
	



    
	
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


