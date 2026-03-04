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
#include <string.h>
#include "body_posture.h"
#include "process_command.h"


// 定义超时时间（毫秒），500ms 到 1000ms
#define CMD_TIMEOUT_MS  150 

// 任务句柄
TaskHandle_t infTaskHandler;
TaskHandle_t mainTaskHandler;
TaskHandle_t balanceTaskHandler;

//用于接收坐标的全局变量，放到body_posture.h里
//float cmd_xL, cmd_yL, cmd_xR, cmd_yR;

/**
 * @brief 串口指令解析与数据打印任务
 * @param arg FreeRTOS任务参数
 */
void messageTask(void *arg)
{
    // 任务启动提示
    printf("USART3 Control Task Started! \r\n");
    printf("Commands: M(Speed), T(Turn), S(Stop) \r\n");
	
	// 初始化逆运动学默认值 ---
    IK_Init(); 	
    
    // 定义变量来“记住”最后一次舵机的状态 ---
    // 必须定义在 while 循环外面，否则每次循环都会被清零
    int Last_Servo_ID = 0;     // 默认ID 0
    int Last_Servo_Angle = 0;  // 默认角度 0
	
	
	// 初始化
//    Data_Locked = 0; 
//    USART2_RxIndex = 0;
	

	//AT指令
	uint8_t status = ESP8266_ConnectWiFi();
	
	if (status == 0)
    {
        printf("WiFi Connect Success!\r\n");
		
		// 【新增】连接成功后，启动 TCP 服务
        vTaskDelay(pdMS_TO_TICKS(500)); // 稍作延时
        //ESP8266_StartTCPServer();
        ESP8266_StartUDP();
		
		printf(">> Switch to UDP Lock Mode.\r\n");
        
        // 清空一下之前的缓存，准备干净的开始
        memset(USART2_RxBuffer, 0, sizeof(USART2_RxBuffer));
        USART2_RxIndex = 0;
//        Data_Locked = 0;
//        
//        // 开启锁模式
//        USART2_Lock_Mode = 1; 
    }
    else
    {
        // 如果失败，status 的值能告诉你死在哪一步
        // 1: AT指令无响应; 2: 连接AP失败
        printf("WiFi Connect Failed! Error Code: %d\r\n", status);
    }
	
	
	// 【新增】定义变量记录最后一次收到指令的时间
    TickType_t Last_Cmd_Time = xTaskGetTickCount();
	
    while(1) 
    {		
		// 1. 处理 WiFi (USART2)
        if (USART2_RxFlag == 1)
        {
            char *pIPD = strstr(USART2_RxBuffer, "+IPD");
            if (pIPD != NULL)
            {
                char *pColon = strchr(pIPD, ':');
                if (pColon != NULL)
                {
                    // 【调用】统一处理函数
                    Process_Command(pColon + 1);
					
				   // 【新增】收到有效指令，更新时间戳
				   Last_Cmd_Time = xTaskGetTickCount(); 
                }
            }
            USART2_RxIndex = 0; 
            USART2_RxFlag = 0; 
            memset(USART2_RxBuffer, 0, sizeof(USART2_RxBuffer));
			
						
        }


//		if (Data_Locked == 1)
//		{
//            char *pIPD = strstr(USART2_RxBuffer, "+IPD");
//            if (pIPD != NULL)
//            {
//                char *pColon = strchr(pIPD, ':');
//                if (pColon != NULL)
//                {
//                    // 解析命令
//                    Process_Command(pColon + 1);
//                    
//                    // 喂狗（重置超时时间）
//                    Last_Cmd_Time = xTaskGetTickCount();
//                }
//            }
//			// -----------------------------------------------------
//            // 处理完毕，准备开锁
//            // -----------------------------------------------------
//            // 1. 清零索引
//            USART2_RxIndex = 0; 
//            // 2. 清空Buffer (其实只要第一个字符置0即可)
//            USART2_RxBuffer[0] = '\0'; 
//		    memset(USART2_RxBuffer, 0, sizeof(USART2_RxBuffer));
//            // 3.开锁！允许中断继续接收新数据
//            Data_Locked = 0; 
//			
//		}

        // 2. 处理 串口3 (USART3)
        if (USART3_RxFlag == 1)
        {
            // 【调用】统一处理函数
            Process_Command(Serial_RxPacket);
            USART3_RxFlag = 0; 
			
		   // 【新增】收到有效指令，更新时间戳
            Last_Cmd_Time = xTaskGetTickCount(); 
        }
		
		// ===========================================================
        // 【新增】超时安全保护逻辑 (软件看门狗)
        // ===========================================================
        // 计算：(当前系统时间 - 上次收到指令时间) 是否大于 设定阈值
        if ((xTaskGetTickCount() - Last_Cmd_Time) > pdMS_TO_TICKS(CMD_TIMEOUT_MS))
        {
            // 只有当车还在动的时候才执行停止，避免重复赋值
            if (Movement != 0 || turnment != 0)
            {
                Movement = 0;
                turnment = 0;
                printf(">> [Safety] Signal Lost! Auto Stop.\r\n");
            }
        }
        // ===========================================================
		

        // 打印状态 (Last_Servo_ID 和 Last_Servo_Angle 会自动从 process_command.c 获取)
        printf("P:%.2f | R:%.2f | M:%d | SID:%d | Ang:%d\r\n", 
                pitch, (float)roll, (int)Movement, Last_Servo_ID, Last_Servo_Angle);

        vTaskDelay(40); 
//		if (USART2_RxFlag == 1)
//		{
//			// 如果收到 ESP32 发来的任何透传数据，直接打印到串口3看
//			printf("Receive: %s\r\n", USART2_RxBuffer);
//			
//			
//			// 2. 检查是否是 TCP 数据 (特征：包含 "+IPD")
//            char *pIPD = strstr(USART2_RxBuffer, "+IPD");
//			
//			
//			if (pIPD != NULL)
//            {
//				
//				// 2. 寻找冒号 ':'
//				char *pColon = strchr(pIPD, ':');
//				if (pColon != NULL)
//				{
//					// 3. 定义一个临时变量 (分配在栈上，用完即焚)
//					char temp_cmd_buffer[64]; 
//					// 为了安全，先清空一下这个临时数组
//					memset(temp_cmd_buffer, 0, sizeof(temp_cmd_buffer));
//					// 4. 把冒号后面的指令存到临时变量里
//					// pColon + 1 就是指令的起始位置
//					// 这里的 strcpy 会把后面的换行符也一起拷进去，不过打印出来没影响
//					strcpy(temp_cmd_buffer, pColon + 1);
//					// 5. 打印这个临时变量，看看是否提取正确
//					printf(">> [DEBUG] Received TCP Cmd: %s\r\n", temp_cmd_buffer);
//				}
//			}
//				
//			// 处理完记得清空标志位
//			USART2_RxFlag = 0; 

//		}
//			
//		
//		
//        // --- 1. 指令解析逻辑 ---
//        if (USART3_RxFlag == 1)
//        {
//            // 解析指令：第一个字母决定功能
//            char cmd = Serial_RxPacket[0];
//			
//            
//            // atoi 从字符串第二个字符开始解析数字
//            // 如果只有字母没有数字，atoi 返回 0
//            int val = atoi(&Serial_RxPacket[1]); 
//            
//            switch (cmd)
//            {
//                case 'M': case 'm': // 调整目标速度 (例: M100)
//                    Movement = val;
//                    printf(">> Set Movement to: %d\r\n", Movement);
//                    break;
//                    
//                case 'T': case 't': // 调整目标转向 (例: T50)
//                    turnment = val;
//                    printf(">> Set Turnment to: %d\r\n", turnment);
//                    break;
//                    
//                case 'S': case 's': // 急停指令 (例: S)
//                    Movement = 0;
//                    turnment = 0;
//                    printf(">> EMERGENCY STOP!\r\n");
//                    break;
//				
//				
//				
//				// ====================================================
//				// 【新增】PID 参数在线调试指令
//				// 协议格式: P<ID>,<Value>  (例如: P1,120.5 修改直立环KP)
//				// ====================================================
//                case 'P': case 'p':
//                {
//                    int pid_id = 0;
//                    float pid_val = 0.0f;
//                    
//                    // 使用 sscanf 解析：整数ID + 逗号 + 浮点数值
//                    int parsed_cnt = sscanf(&Serial_RxPacket[1], "%d,%f", &pid_id, &pid_val);

//                    if (parsed_cnt == 2) // 确保成功解析了 ID 和 数值 两个数据
//                    {
//                        switch (pid_id)
//                        {
//                            case 1: 
//                                upright_Kp = pid_val; 
//                                printf(">> [PID] Upright KP set to: %.3f\r\n", upright_Kp);
//                                break;
//                            case 2: 
//                                upright_Kd = pid_val; 
//                                printf(">> [PID] Upright KD set to: %.3f\r\n", upright_Kd);
//                                break;
//                            case 3: 
//                                cascade_speed_Kp = pid_val; 
//                                printf(">> [PID] Speed KP set to: %.3f\r\n", cascade_speed_Kp);
//                                break;
//                            case 4: 
//                                cascade_speed_Ki = pid_val; 
//                                printf(">> [PID] Speed KI set to: %.3f\r\n", cascade_speed_Ki);
//                                break;
//                            case 5: 
//                                turn_Kp = pid_val; 
//                                printf(">> [PID] Turn KP set to: %.3f\r\n", turn_Kp);
//                                break;
//                            case 6: 
//                                turn_Kd = pid_val; 
//                                printf(">> [PID] Turn KD set to: %.3f\r\n", turn_Kd);
//                                break;
//							
//							// --- 【新增】机械零点调节 ---
//                            case 7: 
//                                mechanical_zero = pid_val; 
//                                printf(">> [Setting] Mechanical Zero set to: %.3f\r\n", mechanical_zero); 
//                                break;
//							
//                            default:
//                                printf(">> Error: Unknown PID ID (Use 1-6)\r\n");
//                                break;
//                        }
//                    }
//                    else
//                    {
//                        printf(">> Error: Format must be P<ID>,<Val>\r\n");
//                        printf(">> IDs: 1:U_Kp, 2:U_Kd, 3:V_Kp, 4:V_Ki, 5:T_Kp, 6:T_Kd\r\n");
//                    }
//                    break;
//                }
//				
//				
//				// ---------------------------------------------------------
//                // 【修改后】逆运动学坐标指令 - 支持双腿/四舵机 + 数学角度显示
//                // 协议格式: K<xL>,<yL>,<xR>,<yR>
//                // ID映射: 左前=1, 左后=2, 右前=3, 右后=4
//                // ---------------------------------------------------------
//				case 'K': case 'k':
//                {
//                    // 使用 sscanf 解析 4 个浮点数
//                    int parsed_count = sscanf(&Serial_RxPacket[1], "%f,%f,%f,%f", 
//                                              &cmd_xL, &cmd_yL, &cmd_xR, &cmd_yR);

//                    if(parsed_count == 4) // 确保成功解析了4个数字
//                    {
////                        // 1. 调用核心解算函数
////                        // 注意：根据刚才修改的 IK_Compute，即使返回 0，Robot_IK 里也已经存入了计算出的“错误”数值
////                        uint8_t is_safe = IK_Compute(cmd_xL, cmd_yL, cmd_xR, cmd_yR);
////                        
////                        // 打印出刚刚算出的数学角度和舵机角度
////                        printf(">> [DEBUG] Input: L(%.0f, %.0f) R(%.0f, %.0f)\r\n", cmd_xL, cmd_yL, cmd_xR, cmd_yR);
////                        
////                        printf(">> LEFT : Math[A:%d, B:%d] -> Servo[ID2:%d, ID1:%d]\r\n", 
////                               Robot_IK.Math_Angle_Alpha_Left, 
////                               Robot_IK.Math_Angle_Beta_Left,
////                               Robot_IK.Angle_Servo_Left_Rear,  // 对应 Alpha
////                               Robot_IK.Angle_Servo_Left_Front  // 对应 Beta
////                              );
////                              
////                        printf(">> RIGHT: Math[A:%d, B:%d] -> Servo[ID4:%d, ID3:%d]\r\n", 
////                               Robot_IK.Math_Angle_Alpha_Right, 
////                               Robot_IK.Math_Angle_Beta_Right,
////                               Robot_IK.Angle_Servo_Right_Rear, 
////                               Robot_IK.Angle_Servo_Right_Front
////                              );

////                        // 3. 【安全开关】只有返回 1 (安全) 才允许动舵机
////                        if (is_safe == 1)
////                        {
////                            //先根据解算的结果应用到每个舵机上
////                            Servo_Move(1, Robot_IK.Angle_Servo_Left_Front, 1000); 
////                            Servo_Move(2, Robot_IK.Angle_Servo_Left_Rear, 1000);  
////                            Servo_Move(3, Robot_IK.Angle_Servo_Right_Front, 1000);
////                            Servo_Move(4, Robot_IK.Angle_Servo_Right_Rear, 1000); 
////							
////                            //然后再根据高度值来获取平均高度，以便之后实时更新PID的系数值
////						    Motor_Set_Target_Height(cmd_yL, cmd_yR);	
////							
////                            printf(">> [STATUS] Safe -> Executing Move.\r\n");
////                        }
////                        else
////                        {
////                            // 4. 如果不安全，明确提示
////                            // 此时舵机不动，但你已经在上面看到了算出来的越界数值
////                            printf(">> [STATUS] UNSAFE LIMITS! Motors LOCKED.\r\n");
////                        }

//						Motor_Set_Target_Height(cmd_yL, cmd_yR);

//                    }
//                    else
//                    {
//                        printf(">> Error: Format must be KxL,yL,xR,yR\r\n");
//                    }
//                    break;
//                }



//                case 'R': case 'r': // 指令格式：R5.0 (向右歪5度) 或 R-5.0 (向左歪5度)
//                {
//                    float target_angle = 0.0f;
//                    
//                    // 解析浮点数
//                    // 这里的 &Serial_RxPacket[1] 跳过第一个字母 'R'
//                    // 假设你的 atof 或 sscanf 支持浮点解析
//                    target_angle = atof(&Serial_RxPacket[1]); 
//                    
//                    // 调用刚才写的函数更新目标值
//                    Set_Target_Roll_Angle(target_angle);
//                    
//                    printf(">> Set Target Roll: %.2f Degree\r\n", target_angle);
//                    break;
//                }
//				
//				
//				                // ====================================================
//                // 【新增】舵机控制指令
//                // 协议格式: A<ID>,<Angle>  (例如: A4,90)
//                // ====================================================
//                case 'A': case 'a':
//                {
//                    // 1. 查找逗号的位置
//                    pSeparator = strchr(Serial_RxPacket, ',');
//                    
//                    if (pSeparator != NULL)
//                    {
//                        // 2. 解析 ID 
//                        // &Serial_RxPacket[1] 是 'A' 后面的字符
//                        // atoi 遇到非数字(这里是逗号)会自动停止
//                        id = atoi(&Serial_RxPacket[1]);
//                        
//                        // 3. 解析 角度 
//                        // pSeparator 是逗号的地址，+1 就是逗号后面那个数字的地址
//                        angle = atoi(pSeparator + 1);

//                        // 4. 角度限幅 (保护舵机不被烧毁)
//                        if(angle < 0) angle = 0;
//                        if(angle > 240) angle = 240; 

//                        // 5. 执行动作 (假设动作时间给 1000ms)
//                        Servo_Move(id, angle, 1000); 
//						
//						// --- 【修改点2】更新记忆变量 ---
//                        Last_Servo_ID = id;
//                        Last_Servo_Angle = angle;

//                        
//                        printf(">> Servo ID:%d -> Angle:%d\r\n", id, angle);
//                    }
//                    else
//                    {
//                        printf(">> Error: Format must be A<ID>,<Angle> (e.g. A1,90)\r\n");
//                    }
//                    break;
//                }
//				
//				
//				
//				
//				case 'H': case 'h':
//				{
//					// 使用 sscanf 解析 2 个浮点数 (只解析 yL 和 yR)
//					// 预期格式例如: H160.5,160.5
//					int parsed_count = sscanf(&Serial_RxPacket[1], "%f,%f", &cmd_yL, &cmd_yR);

//					if(parsed_count == 2) // 确保成功解析了2个数字
//					{
//						// 这里只执行高度设置函数
//						Motor_Set_Target_Height(cmd_yL, cmd_yR);

//						// 建议加一句调试打印，方便确认数据是否正确接收
//						printf(">> [CMD H] Set Height -> L:%.1f, R:%.1f\r\n", cmd_yL, cmd_yR);
//					}
//					break; // switch 语句通常需要 break，防止穿透
//				}

//				
//                
//                default:
//                    printf(">> Unknown Command: %s\r\n", Serial_RxPacket);
//                    break;
//            }
//            
//            // 处理完后务必清空标志位，准备接收下一帧
//            USART3_RxFlag = 0; 
//			
//        }


//        //翻滚角向右侧翻滚是负值，向左侧翻滚是负值
//        // P:俯仰角 | R:翻滚角 | M:速度 | T:转向 | SID:舵机ID | Ang:舵机角度
//        printf("P:%.2f | R:%.2f | PWM1:%d | PWM2:%d | M:%d | T:%d | SID:%d | Ang:%d\r\n", 
//                pitch, (float)roll , debug_pwm1, debug_pwm2 , (int)Movement, turnment, Last_Servo_ID, Last_Servo_Angle);

//        // --- 3. 延时 ---
//        // 40ms 刷新一次打印，既能看清数据，又不会占满串口带宽
//        vTaskDelay(40); 
    }
}

void mainTask(void *arg)
{
	TickType_t xLastWakeTime;
    const TickType_t xFrequency = 5; // 定义周期为 5ms (对应 FreeRTOS configTICK_RATE_HZ 为 1000)	
	
	// 定义静态变量作为分频计数器
	static uint8_t mpu_tick = 0; 
    static uint8_t print_tick = 0;
	
	    // 2. 初始化时间变量
    xLastWakeTime = xTaskGetTickCount();

	    // 2. 一切准备就绪后，开启看门狗！
    IWDG_Init(); 
    printf("IWDG Enabled.\r\n");
	
	while(1)
    {
		// --- A. 进入临界区或绝对时间控制开始 ---
		// vTaskDelayUntil 会确保从上一次唤醒时刻开始，严格过了 5ms 后才唤醒
		// 这样可以保证控制频率严格稳定在 200Hz
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		
		// 计数器自增
        mpu_tick++;
        print_tick++; // 专门用来计时的，不会被清零干扰
		
		
		// 3. 【分频逻辑】每 10ms 读取一次 MPU6050  必须大于10ms，否则太快读取不到MPU的数据
        if(mpu_tick >= 2) 
        {
            MPU6050_Data_read(); // 读取姿态 (此时 DMP 刚好有数据)
		    mpu_tick = 0;  // 清零计数器
        }


		Motor_PID_Update_Task(); 
		speed_read();
		control_motor();
		
		// --- 新增：喂狗 ---
        // 只要程序能运行到这里，说明控制逻辑正常，没有死机
        IWDG_Feed(); 

    }
}


// =============================================================
// 【新增】独立的平衡任务 (Balance Task)
// 专门负责算腿，不干扰主任务，也不会被主任务卡死
// =============================================================
void balanceTask(void *arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 40; 
    
    // 接收计算结果的变量，是驱动舵机的最终坐标*****
    float final_yL, final_yR;
    float final_x; 
	
	
	// 平均速度
    float current_avg_speed = 0.0f;
    
    // 初始化时间
    xLastWakeTime = xTaskGetTickCount();
    
    // 初始化逆解
    IK_Init(); 
    
    // 高度初始化 (处在中间位置，便于活动)  
    cmd_xL = 22.0f; cmd_yL = 110.0f;
    cmd_xR = 22.0f; cmd_yR = 110.0f;

    while(1)
    {
        // 1. 严格控制周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 1. 获取当前平均速度 (读取 data_read.h 里的全局变量)
        current_avg_speed = (float)(speedLeft + speedRight) / 2.0f;

        // 2. 核心计算
        // 传入：姿态、目标速度(Movement)、当前速度(current_avg_speed)
        // 传出：final_x (前后重心), final_yL/R (左右高度)
        Body_Balance_Compute(roll, gyrox, (float)Movement, current_avg_speed, 
                             &final_x, &final_yL, &final_yR);

		
		
		//最终输出=上位机指令（基准）+PID计算结果（修正）  并不会覆盖基准指令，自动控制仅仅只在基准指令的基础上，添加修正量。
		//逆运动学解算
		//***********************************   核心部分，根据坐标来驱动舵机，这是自动化执行   
        uint8_t is_safe = IK_Compute(final_x, final_yL, final_x, final_yR);
        // 驱动舵机 (如果解算成功)
        if (is_safe)
        {
            Servo_Move(1, Robot_IK.Angle_Servo_Left_Front, 50); //这里从1000ms换成了25ms，加快舵机的响应速度
            Servo_Move(2, Robot_IK.Angle_Servo_Left_Rear, 50);  
            Servo_Move(3, Robot_IK.Angle_Servo_Right_Front, 50);
            Servo_Move(4, Robot_IK.Angle_Servo_Right_Rear, 50); 
        }
		//**********************************
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
	// 3. 【新增】平衡任务 (优先级 3 - 负责腿部舵机)
    xTaskCreate(balanceTask, "bal_Task", 1024, NULL, 3, &balanceTaskHandler);
	
    // 5. 启动调度器
    vTaskStartScheduler();

    // 正常情况下，程序不会运行到这里
    while (1)
    {
        
    }
}


