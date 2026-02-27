/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "sensor.h"
#include <string.h>
#include "queue.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float prev_error;

    int target_speed;  // encoder value
    int measured_speed;
    int pwm_output;
} PID_Controller;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for readICMTask */
osThreadId_t readICMTaskHandle;
const osThreadAttr_t readICMTask_attributes = {
  .name = "readICMTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

// Multitasking threads TODO: untested
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "OledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t turnTaskHandle;
const osThreadAttr_t turnTask_attributes = {
  .name = "turnTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t moveTaskHandle;
const osThreadAttr_t moveTask_attributes = {
  .name = "moveTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t parseCommandTaskHandle;
const osThreadAttr_t parseCommandTask_attributes = {
  .name = "ParseCommandTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t executeCommandTaskHandle;
const osThreadAttr_t executeCommandTask_attributes = {
  .name = "ExecuteCommandTask",
  .stack_size = 256 * 8,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t frontWheelCalibHandle;
const osThreadAttr_t frontWheelCalib_attributes = {
  .name = "frontWheelCalib",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

// icm20948 (gyro)
#define ICM20948_I2C_ADDR   (0x68 << 1)
volatile float gyro_z_dps = 0.0f;

// Servo constants
const int SERVO_STRAIGHT = 172;
const int SERVO_LEFT_MAX = 95;
const int SERVO_LEFT = 120;
const int SERVO_RIGHT = 240;
const int SERVO_RIGHT_MAX = 249; // PROBABLY CAN GO HIGHER, BUT NOT TESTED

// variables for communication with pi via UART
#define MAX_BUF_SIZE 20
volatile uint8_t command_buf[MAX_BUF_SIZE];
volatile char uart_input;
volatile int command_len = 0;

typedef struct {
	uint8_t command[20];
	uint32_t command_len;
} Command;



// public variables for multi threading
volatile uint8_t display_buf[20];
volatile uint8_t display_buf2[20];
volatile uint8_t display_buf3[20];
volatile int ready_execute = 1; // set to 0 when executing a command, so future commands are queued.
volatile int ready_parse = 0;

// used as shared variables for move and turn args
volatile int move_flag = 0;
volatile int delay_flag = 1000;
volatile int turn_flag = -1;
volatile int turned_angle = 0; //track how many degrees the car has turned
volatile int target_angle = 0;

const int SPEED_MAX = 7190;
const int SPEED_NORM = 3000;
const int SPEED_SLOW = 1500;

volatile int speedA = SPEED_NORM; // the pwm wave, 7199 is max i think
volatile int speedB = SPEED_NORM;
uint32_t cntA = 0;
uint32_t cntB = 0;

volatile int encSpeedA = 0; //pid definition
volatile int encSpeedB = 0;

// used for encoders
//const float ENCODER_COUNTS_PER_REVOLUTION = 1540.0f;
const float ENCODER_COUNTS_PER_REVOLUTION = 770.0f;
const float WHEEL_CIRCUMFERENCE_CM = 6.5f*3.14f;
volatile int rotations = 0;

volatile float Adist = 0; //temp vars
volatile float Bdist = 0;

/* USER CODE END PV */
// DC Motor PID
const PID_Controller defaultPid = {1.0f, 0.8f, 0.15f, 0, 0, 200, 0, 0};
volatile PID_Controller pidA = defaultPid;
volatile PID_Controller pidB = defaultPid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void encoder(void *argument);
void readICM(void *argument);

void MoveFowardPID(void);

/* USER CODE BEGIN PFP */
void parse_command(void);
void parse_command_thread(void *args);
/* Definitions for frontWheelCalib */
void frontWheelCalibTask(void *argument);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

QueueHandle_t commandQueue;


void display(uint8_t* str, int buf_no){
	// display_thread just looks at display_buf.
	// buf_no is to put it in line1 or line2
	if (buf_no == 1){
		strncpy(display_buf, (const char*) str, MAX_BUF_SIZE); //get rid of volatile compilation warning
		// display_buf[strlen(str)] = '\0';
	}
	else if (buf_no == 2){
		strncpy(display_buf2, (const char*) str, MAX_BUF_SIZE); //get rid of volatile compilation warning
	}
	else if (buf_no == 3){
		strncpy(display_buf3, (const char*) str, MAX_BUF_SIZE); //get rid of volatile compilation warning
	}
	return;
}

void send(uint8_t* str){
    HAL_UART_Transmit(&huart3, str, strlen((char*)str), 0xFFFF);
}

/*
void send(uint8_t* str){
	HAL_UART_Transmit(&huart3, (uint8_t *) str, sizeof(str), 0xFFFF);
}
*/

void display_thread(void* args){
	uint8_t buf[20];
	uint8_t buf2[20];
	uint8_t buf3[20];

	for (;;){
		strncpy(buf, (const char*)display_buf, MAX_BUF_SIZE); //get rid of volatile compilation warning
		//sprintf(buf, "testing %s\0", display_buf);
		OLED_Clear();
		OLED_ShowString(10, 10, buf);

		strncpy(buf2, (const char*)display_buf2, MAX_BUF_SIZE); //get rid of volatile compilation warning
		OLED_ShowString(10, 20, buf2);

		strncpy(buf3, (const char*)display_buf3, MAX_BUF_SIZE); //get rid of volatile compilation warning
		OLED_ShowString(10, 30, buf3);

		OLED_Refresh_Gram();
		osDelay(100);
	}
}

void stop(){
    speedA = SPEED_NORM;
    speedB = SPEED_NORM;
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,7199);
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1, 7199);
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, 7199);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 7199);
	/*
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - forward
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin,GPIO_PIN_RESET);
	*/
}
void move(int direction){
    // 在 OCPOLARITY_LOW 模式下：
    // 数值 7199 = 输出 0V (停止)
    // 数值 0    = 输出 3.3V (全速)
    int stop_val = 7199;
    int run_valA = 7199 - speedA;
    int run_valB = 7199 - speedB;

    if (direction == 1){ // 强制定义为前进
        // 如果前进是后退，就把这里的 CHANNEL_3 和 4 对调
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, run_valA);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, stop_val);

        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, run_valB);
        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, stop_val);
    }
    else if (direction == 2){ // 强制定义为后退
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, stop_val);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, run_valA);

        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, stop_val);
        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, run_valB);
    }
}
/*
void move(int direction){
	int pwmVal = 7199;
	//int pwmVal = motor_speed;

	if (direction == 1){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0); // set IN1 to maximum PWM (7199) for '1'
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, speedA); // PWM to Motor A (IN2)

		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, 0);
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, speedB); // PWM to Motor B (IN2)
	}
	else if (direction == 2){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, speedA); // PWM to Motor A (IN1)


		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, speedB);
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0); // PWM to Motor B (IN2)
	}

}
*/
/* USER CODE BEGIN Header_frontWheelCalibrationTask */
/**
* @brief Function implementing the frontWheelCalib thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_frontWheelCalibrationTask */
void frontWheelCalibrationTask(void *argument)
{
    // toggle 用来切换左右，静态局部变量
    static uint8_t toggle = 0;

    for(;;)
    {
        // 只用现有变量 turn_flag 控制舵机
        if(toggle){
            turn_flag = SERVO_RIGHT; // 前轮向右
            toggle = 0;
        }else{
            turn_flag = SERVO_LEFT;  // 前轮向左
            toggle = 1;
        }

        osDelay(10); // 根据舵机响应调整间隔
    }
}




/* USER CODE BEGIN 0 */
QueueHandle_t commandQueue;



/* USER CODE END 0 */


/* USER CODE END Includes */

/* USER CODE BEGIN PFP */
/* 函数原型声明：必须放在 main 函数外面！ */
void testMoveThread(void *argument);
void testTurnThread(void *argument);
void execute_command_thread(void *argument);
void display_thread(void *argument);
void parse_command_thread(void *argument);
void encoder(void *argument);
void readICM(void *argument);
void StartDefaultTask(void *argument);

void turn(int value);
void stop(void);
void move(int direction);
/* USER CODE END PFP */


int main(void)
{
  /* 1. MCU 基础硬件初始化 */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM12_Init();      // 舵机
  MX_USART3_UART_Init();
  MX_TIM2_Init();       // 编码器A
  MX_TIM3_Init();       // 编码器B
  MX_TIM4_Init();       // 电机A
  MX_TIM9_Init();       // 电机B
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  // 2. 外设软件初始化
  OLED_Init();
  IR_Sensors_Init();

  // 3. 启动所有 PWM 通道
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  // 4. 启动编码器计数
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // 5. 开启串口中断接收
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart_input, 1);
  /* USER CODE BEGIN PV */

  /* USER CODE END PV */
  /* USER CODE END 2 */

  /* 6. RTOS 初始化 */
  osKernelInitialize();

  // 7. 创建指令队列
//  commandQueue = xQueueCreate(5, sizeof(Command));
//
//  // --- 模拟开机自启动指令：后退 2000ms ---
//  Command testCmd;
//  memset(&testCmd, 0, sizeof(Command));
//  strcpy((char*)testCmd.command, "FW 20"); // 这里你可以改成 FW
//  testCmd.command_len = strlen((char*)testCmd.command);
//  xQueueSend(commandQueue, &testCmd, 0);
  // ------------------------------------

  /* 8. 创建所有任务线程 */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  EncoderTaskHandle = osThreadNew(encoder, NULL, &EncoderTask_attributes);
  readICMTaskHandle = osThreadNew(readICM, NULL, &readICMTask_attributes);
  oledTaskHandle    = osThreadNew(display_thread, NULL, &oledTask_attributes);

  moveTaskHandle    = osThreadNew(testMoveThread, NULL, &moveTask_attributes);
  turnTaskHandle    = osThreadNew(testTurnThread, NULL, &turnTask_attributes);

  parseCommandTaskHandle   = osThreadNew(parse_command_thread, NULL, &parseCommandTask_attributes);
  executeCommandTaskHandle = osThreadNew(execute_command_thread, NULL, &executeCommandTask_attributes);

  /* 9. 启动内核（程序开始运行任务） */
  osKernelStart();

  /* 正常不应到达此处 */
  while (1) {}
}

/* ---------------------------------------------------------------------------
 * 任务回调补充（确保逻辑闭环）
 * --------------------------------------------------------------------------- */

void testMoveThread(void* arg){
    for (;;){
        if (move_flag != 0) { // 只有在移动时才进行实时修正

            // 实时读取编码器数值 (假设你的变量叫 countA 和 countB)
            // 如果 A 跑得比 B 慢了
            if (cntA < cntB) {
                speedA -= 10; // 实时给 A 加力
                speedB += 10; // 实时给 B 减速
            }
            // 如果 A 跑得比 B 快了
            else if (cntA > cntB) {
                speedA += 10;
                speedB -= 10;
            }

            // 边界限速，防止数值溢出 (0-7199 范围)
            if(speedA > 6000) speedA = 6000;
            if(speedB > 6000) speedB = 6000;
            if(speedA < 1000) speedA = 1000;
            if(speedB < 1000) speedB = 1000;

            move(move_flag); // 应用最新的速度
        } else {
            stop();
        }

        osDelay(50); // 每 50ms 动态检测并调整一次
    }
}
// 注意：在 execute_command_thread 中，FW 指令会将 move_flag 设为 1

void testTurnThread(void* arg){
	char command[20];
	for (;;){
		if (turn_flag != -1){

			sprintf(command, "IN TURN %d !", turn_flag); //TESTING
			display(command, 1);

			turn(turn_flag);
			turn_flag = -1;
		}
	}
}

void turn(int value){
	// turns the front wheels
	if (value < SERVO_LEFT_MAX || value > SERVO_RIGHT_MAX){
		display("value too extreme", 1);
		return;
	}

	htim12.Instance->CCR1 = value; // straight
}


void icm20948_init(void){
    uint8_t data;

    // Wake up
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, ICM20948_I2C_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // Enable accel & gyro
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, ICM20948_I2C_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // Disable ICM internal I2C master (required for BYPASS)
	data = 0x00; // USER_CTRL (0x03)
	HAL_I2C_Mem_Write(&hi2c2, ICM20948_I2C_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	// Enable BYPASS so MCU can talk to AK09916 at 0x0C
	data = 0x02; // INT_PIN_CFG (0x0F): BYPASS_EN=1
	HAL_I2C_Mem_Write(&hi2c2, 0x68<<1, 0x0F, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	// Put AK09916 into continuous mode (e.g., 100 Hz)
	data = 0x08; // CNTL2 (0x31): 100 Hz
	HAL_I2C_Mem_Write(&hi2c2, 0x0C<<1, 0x31, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);

    // 只接受可见 ASCII
    if ((uart_input >= 'A' && uart_input <= 'Z') ||
            (uart_input >= '0' && uart_input <= '9') ||
            (uart_input == ' '))
        {
            if (command_len < MAX_BUF_SIZE - 1) {
                command_buf[command_len++] = uart_input;
            }
        }
    else if (uart_input == ';')   // 指令结束
    {
        command_buf[command_len] = '\0';
        ready_parse = 1;
        command_len = 0;
    }
    else if (uart_input == ':')   // 强制清空
    {
        command_len = 0;
        memset(command_buf, 0, MAX_BUF_SIZE);
    }
    // 其他字符（退格、冒号前的垃圾）全部丢弃
    send("RX OK\r\n");
    HAL_UART_Receive_IT(&huart3, &uart_input, 1);
}

void parse_command_thread(void* args){
	for(;;){
		if (ready_parse == 1){
			parse_command();
			osDelay(100);
		}
	}
}
void parse_command()
{
    if (ready_parse)
    {
        ready_parse = 0;

        Command cmd;
        memset(&cmd, 0, sizeof(cmd));

        strncpy((char*)cmd.command, (char*)command_buf, 19);
        cmd.command_len = strlen((char*)cmd.command);

        xQueueSend(commandQueue, &cmd, portMAX_DELAY);

        memset(command_buf, 0, MAX_BUF_SIZE);
    }
}


void execute_command_thread(void* args){
	char tmp[20];
	char command[4];
	int value;
	const char *commands[] = {"FW\0", "BW\0", "LFT\0", "RGT\0", "FL\0", "FR\0", "BL\0", "BR\0"};
	Command cmd;

	for(;;){
		if(ready_execute == 1){
			if(xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdPASS){
				ready_execute = 0;

				strcpy(tmp, "");
				value = 0;

				strncpy(tmp, (const char*)cmd.command, cmd.command_len);
				tmp[cmd.command_len] = '\0';
				sscanf(tmp, "%s %d", command, &value);

				display(tmp, 3);

				// ================== 前进 (距离控制) ==================
				if (strcmp(command, commands[0]) == 0 ){
					Adist = 0; Bdist = 0; // 重置起始距离
					move_flag = 1;
					float target = (float)value;

					while (Adist < target) {
						// 减速缓冲区：还剩10cm时减速
//						if (target - Adist < 10.0f) speedA = speedB = SPEED_SLOW;
//						else speedA = speedB = SPEED_NORM;
						osDelay(10);
					}
					move_flag = 0;
					stop();
					send("OK!\r\n");
				}
				// ================== 后退 (距离控制) ==================
				else if (strcmp(command, commands[1]) == 0){
					Adist = 0; Bdist = 0;
					move_flag = 2;
					float target = (float)value;

					while (fabs(Adist) < target) {
						if (target - fabs(Adist) < 10.0f) speedA = speedB = SPEED_SLOW;
						else speedA = speedB = SPEED_NORM;
						osDelay(20);
					}
					move_flag = 0;
					stop();
					send("OK!\r\n");
				}
				// ================== 转向逻辑 (保持原样) ==================
				else if (strcmp(command, commands[2]) == 0){ // LFT
					turn_flag = value; osDelay(1000); send("OK!\r\n");
				}
				else if (strcmp(command, commands[3]) == 0){ // RGT
					turn_flag = value; osDelay(1000); send("OK!\r\n");
				}
				// ================== 弧线前进/后退 (类似修改) ==================
				else if (strcmp(command, commands[4]) == 0 || strcmp(command, commands[5]) == 0){ // FL or FR
					turn_flag = (strcmp(command, commands[4]) == 0) ? SERVO_LEFT : SERVO_RIGHT;
					osDelay(500); // 等待舵机转到位
					Adist = 0; move_flag = 1;
					while (Adist < (float)value) osDelay(20);
					move_flag = 0; turn_flag = SERVO_STRAIGHT;
					send("OK!\r\n");
				}
				// ... BL/BR 逻辑同理，使用 fabs(Adist) ...

				osDelay(100);
				ready_execute = 1;
			}
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void MoveFowardPID(void)
{
    // PID 参数（先用保守值）, can change
    const float Kp = 1.0f;
    const float Ki = 0.0f;
    const float Kd = 0.1f;

    static float integral = 0;
    static float prev_error = 0;

    // speed differences
    float error = (float)(encSpeedA - encSpeedB);

    integral += error;
    float derivative = error - prev_error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;

    // fix speedA / speedB
    speedA -= (int)output;
    speedB += (int)output;

    // set limitations
    if (speedA > SPEED_MAX) speedA = SPEED_MAX;
    if (speedA < 0)         speedA = 0;
    if (speedB > SPEED_MAX) speedB = SPEED_MAX;
    if (speedB < 0)         speedB = 0;
}


//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  Init();
//
//  /* USER CODE BEGIN Init */
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_TIM12_Init();
//  MX_USART3_UART_Init();
//  MX_TIM2_Init();
//  MX_TIM3_Init();
//  MX_TIM4_Init();
//  MX_TIM9_Init();
//  MX_I2C2_Init();
//  /* USER CODE BEGIN 2 */
//  OLED_Init();
//  IR_Sensors_Init();
//
//  // SERVO
//  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
//
//  // DC MOTORS
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
//  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
//
//  // encoders?
//  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
//  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
//
//
//  // wait for an input
//  // reads 1 character at a time
//  HAL_UART_Receive_IT(&huart3, (uint8_t *) &uart_input, 1);
//  //HAL_UART_Transmit_IT(&huart3, (uint8_t *) &command_buf, command_len, 0xFFFF); // send instruction
//  /* USER CODE BEGIN 2 */
//
//  /* USER CODE END 2 */
//
//  /* Init scheduler */
//  osKernelInitialize();
//
//  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
//  /* USER CODE END RTOS_MUTEX */
//
//  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  /* add semaphores, ... */
//  /* USER CODE END RTOS_SEMAPHORES */
//
//  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
//  /* USER CODE END RTOS_TIMERS */
//
//  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
//  commandQueue = xQueueCreate(2, sizeof(Command));
//  /* USER CODE END RTOS_QUEUES */
//
//  /* Create the thread(s) */
//  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
//
//  /* creation of EncoderTask */
//  EncoderTaskHandle = osThreadNew(encoder, NULL, &EncoderTask_attributes);
//
//  /* creation of readICMTask */
//  readICMTaskHandle = osThreadNew(readICM, NULL, &readICMTask_attributes);
//
//  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
//  oledTaskHandle = osThreadNew(display_thread, NULL, &oledTask_attributes);
//  moveTaskHandle = osThreadNew(testMoveThread, NULL, &moveTask_attributes);
//  turnTaskHandle = osThreadNew(testTurnThread, NULL, &turnTask_attributes);
//  parseCommandTaskHandle = osThreadNew(parse_command_thread, NULL, &parseCommandTask_attributes);
//  executeCommandTaskHandle = osThreadNew(execute_command_thread, NULL, &executeCommandTask_attributes);
//  /* creation of frontWheelCalib */
//  //frontWheelCalibHandle = osThreadNew(frontWheelCalibrationTask, NULL, &frontWheelCalib_attributes);
//  /* USER CODE END RTOS_THREADS */
//
//  /* USER CODE BEGIN RTOS_EVENTS */
//  /* add events, ... */
//  /* USER CODE END RTOS_EVENTS */
//
//  /* Start scheduler */
//  osKernelStart();
//
//  /* We should never get here as control is now taken by the scheduler */
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//} /* USER CODE BEGIN SysInit */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65536;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 7199;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 160;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin_Pin|OLED_RES_Pin_Pin|OLED_SDA_Pin_Pin|OLED_SCL_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin_Pin OLED_RES_Pin_Pin OLED_SDA_Pin_Pin OLED_SCL_Pin_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin_Pin|OLED_RES_Pin_Pin|OLED_SDA_Pin_Pin|OLED_SCL_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_IR_Pin Right_IR_Pin */
  GPIO_InitStruct.Pin = Left_IR_Pin|Right_IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */


  for(;;)
  {
	  uint8_t ch = 'A';
	  for(;;)
	  {
		//HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, 0xFFFF); // send instruction
		if (ch<'Z'){
			ch++;
		}
		else{
			ch='A';
		}
		//sprintf(display_buf, "%c\0", ch);
		//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	    osDelay(1000);
	  }

  }



	osDelay(1);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
    // 将局部变量删掉，直接使用你在全局定义的 uint32_t cntA, cntB
    // 这样 testMoveThread 也能看到这些实时数据

    // 初始化清零
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    Adist = 0;
    Bdist = 0;

    for (;;)
    {
        // 1. 实时读取并【立即清零】
        // 使用 int16_t 强制转换，可以直接处理计数器减量（正负号）
        int16_t diffA = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t diffB = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);

        // 2. 更新全局计数（用于显示）
        // 注意：如果发现 Bdist 总是负数，请把下面的 -diffB 改成 +diffB
        Adist += (float)diffA / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;
        Bdist += (float)(-diffB) / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;

        // 3. 直线修正逻辑
        if (move_flag != 0)
        {
            // 计算误差：A 走得比 B 多多少
            float error = Adist - Bdist;
            float Kp = 15.0f; // 如果修正太慢就调大，如果左右晃荡就调小

            int correction = (int)(Kp * error);

            // 重要：根据你的 move() 函数逻辑调整正负号
            // 假设：speedA 越大，run_valA(7199-speedA)越小，占空比越大，速度越快
            // 如果 A 跑多了 (error > 0)，我们应该【减小】speedA
            speedA = SPEED_NORM - correction;
            speedB = SPEED_NORM + correction;

            // 限幅
            if (speedA > SPEED_MAX) speedA = SPEED_MAX;
            if (speedA < 500)       speedA = 500;
            if (speedB > SPEED_MAX) speedB = SPEED_MAX;
            if (speedB < 500)       speedB = 500;
        }
        else
        {
            // 不动的时候重置误差，防止下次启动时突然弹射
            Adist = 0;
            Bdist = 0;
            speedA = SPEED_NORM;
            speedB = SPEED_NORM;
        }

        // 4. OLED 调试显示
        char buf[32];
        char buf_pwm[32];
        sprintf(buf, "Dst A:%.1f B:%.1f", Adist, Bdist);
        display((uint8_t*)buf, 2);

        sprintf(buf_pwm, "PWM A:%d B:%d", speedA, speedB);
        display((uint8_t*)buf_pwm, 3);

        osDelay(50); // 建议改为 50ms，采样太快数据跳动大，PID 不稳
    }
}
/*
void encoder(void *argument)
{

	uint32_t CA, CB, prevCA, prevCB;
	int32_t Adiff, Bdiff;
	int32_t Adiffraw, Bdiffraw;

	char buf[20] = "TESTING!";

	prevCA = __HAL_TIM_GET_COUNTER(&htim2);
	prevCB = __HAL_TIM_GET_COUNTER(&htim3);


  for(;;)
  {
	  //motor A
	  CA = __HAL_TIM_GET_COUNTER(&htim2);
	  Adiffraw = (int16_t) CA - prevCA;
	  prevCA = CA;

	  //motor B
	  CB = __HAL_TIM_GET_COUNTER(&htim3);
	  Bdiffraw = (int16_t) CB - prevCB;
	  prevCB = CB;

	  //Adist += (float)Adiff / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;
	  //Adist += (float)Adiff / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;
	  //Bdist += (float)Bdiff / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;

	  // TODO: THIS PROBABLY OVERFLOWS AFTER A CERTAIN DISTANCE (CA/CB LOOPS BACK TO 0). ACCOUNT FOR THIS
	  Adist = CA / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;
	  if (CB == 0){
		  CB = 65536;
	  }
	  Bdist = (65536 - CB) / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;

	  //itoa(Adist * 1000, buf, 10);
	  sprintf(buf, "%d %d", (int)Adist, (int)Bdist);
	  //OLED_ShowString(10,20, buf);
	  display(buf, 2);

	  vTaskDelay(pdMS_TO_TICKS(1));
	  //osDelay(1); // idk why but it breaks if theres no delay.
  }
}
*/


/* USER CODE BEGIN Header_readICM */
/**
* @brief Function implementing the readICMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readICM */
void readICM(void *argument)
{
  /* USER CODE BEGIN readICM */

	// https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf

	uint8_t z_reg_addr = 0x37;    // start from GYRO_ZOUT_H
	uint8_t y_reg_addr = 0x35;
	uint8_t x_reg_addr = 0x33;
	uint8_t rawData[2];         // to store MSB and LSB
	int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
	float gyro_z_dps;
	float gyro_bias = 0.0f;

	float theta = 0.0f;

	char buf[20];
	char buf2[20];
//	int a;
	uint32_t currentTick;

	icm20948_init();
	osDelay(1000); //delay to make sure ICM 20948 power up

	uint32_t lastTick;

	/*
	display("Calibrating...", 1);

	const int samples = 1000;
	float sum = 0;

	for(int i = 0; i < samples; i++)
	{
	HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &z_reg_addr, 1, 100);
	HAL_I2C_Master_Receive(&hi2c2, 0x68 << 1, rawData, 2, 100);

	gyro_z_raw = (int16_t)((rawData[0] << 8) | rawData[1]);
	gyro_z_dps = gyro_z_raw / 131.0f;   // convert to deg/sec

	sum += gyro_z_dps;
	osDelay(2);
	}

	gyro_bias = sum / samples;

	sprintf(buf, "bias %d", gyro_bias);
	display(buf, 1);
	osDelay(1000);
	*/

	lastTick = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {

	  // -------------- Gyroscope Readings ---------------------------------------
	// Read Z axis
	// send a byte to trigger read operation?
	if(HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &z_reg_addr, 1, 1000)!= HAL_OK){
		OLED_ShowString(10, 40, "ERROR 0");
	}
	// Read 2 hex bytes (MSB + LSB)
	if (HAL_I2C_Master_Receive(&hi2c2, 0x68 << 1, rawData, 2, 1000)!=HAL_OK){
		OLED_ShowString(10, 40, "ERROR 1");
	}
	// Combine into signed 16-bit value
	gyro_z_raw = (int16_t)((rawData[0] << 8) | rawData[1]);
	gyro_z_dps = gyro_z_raw / 131.0f; // convert to degrees per sec

	if(fabs(gyro_z_dps) < 0.5f){ // noise?
		gyro_z_dps = 0.0f;
	}

	currentTick = HAL_GetTick();
	theta += gyro_z_dps * ((currentTick - lastTick) / 1000.0f);
	lastTick = currentTick;

	//display(theta, 1);
	turned_angle = theta;

	//sprintf(buf, "%d", turned_angle);
	//display(buf, 1);

	//sprintf(buf2, "%d", gyro_z_raw);
	//display(buf2, 3);


	/*
	// Read Y axis
	if(HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &y_reg_addr, 1, 1000)!= HAL_OK){
		OLED_ShowString(10, 40, "ERROR 0");
	}
	if (HAL_I2C_Master_Receive(&hi2c2, 0x68 << 1, rawData, 2, 1000)!=HAL_OK){
		OLED_ShowString(10, 40, "ERROR 1");
	}
	gyro_y_raw = (int16_t)((rawData[0] << 8) | rawData[1]);

	// Read X axis
	if(HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &x_reg_addr, 1, 1000)!= HAL_OK){
		OLED_ShowString(10, 40, "ERROR 0");
	}
	if (HAL_I2C_Master_Receive(&hi2c2, 0x68 << 1, rawData, 2, 1000)!=HAL_OK){
		OLED_ShowString(10, 40, "ERROR 1");
	}
	gyro_x_raw = (int16_t)((rawData[0] << 8) | rawData[1]);

	/*
	if(fabs(gyro_z_raw) < 0.5f){
		gyro_z_raw = 0.0f;
	}
	*/

	/*
	// Integration for angle
	if(isTurning){
		currentTime = HAL_GetTick();
		sprintf(buf4, "%d", gyro_z_raw);
		deltaTime = currentTime - lastAngleUpdateTime;
		if(deltaTime > 0){
			currentAngle += gyro_z_dps * (deltaTime / 1000.0f);
			lastAngleUpdateTime = currentTime;
		}
	}
	*/
	//format for OLED display
//	a = gyro_z_dps;
//	sprintf(buf, "%d\n", a);
//	OLED_ShowString(10, 20, buf);
//	HAL_UART_Transmit(&huart3,(uint8_t *)buf,strlen(buf),0xFFFF);
	// ------------------ End of Gyroscope Readings -------------------------------

//	sprintf(buf, "%3d", a);
//	OLED_ShowString(10, 20, buf);
	osDelay(10);
	  }
  /* USER CODE END readICM */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
