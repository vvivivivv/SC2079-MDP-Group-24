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
  .priority = (osPriority_t) osPriorityLow,
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

// used for encoders
const float ENCODER_COUNTS_PER_REVOLUTION = 1540.0f;
const float WHEEL_CIRCUMFERENCE_CM = 6.5f*3.14f;

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

/* USER CODE BEGIN PFP */

void parse_command_thread(void *args);
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
	HAL_UART_Transmit(&huart3, (uint8_t *) str, sizeof(str), 0xFFFF);
}

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
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 0);
	/*
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - forward
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin,GPIO_PIN_RESET);
	*/
}


void move(int direction){
	//int pwmVal = 7199;
	int pwmVal = 3000;
	if (direction == 1){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0); // set IN1 to maximum PWM (7199) for '1'
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, pwmVal); // PWM to Motor A (IN2)

		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, 0);
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwmVal); // PWM to Motor B (IN2)
	}
	else if (direction == 2){
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, pwmVal); // PWM to Motor A (IN1)


		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, pwmVal);
		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0); // PWM to Motor B (IN2)
	}
	/*
	// 1 for forward, 2 for backwards
	// plug motorA to the left motor (blue)
	if (direction == 2){
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET); //opposite of motorb cuz its looking the other direction

		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin,GPIO_PIN_RESET);

	}
	else if (direction == 1){
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET); // set direction of rotation for wheel A
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET); //opposite of motorb cuz its looking the other direction

		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin,GPIO_PIN_SET);
	}
	*/
}

void testMoveThread(void* arg){
	char command[20];
	for (;;){
		//sprintf(command, "IN MOVE %d ! %d", move_flag, delay_flag); //TESTING
		//display(command);
		//osDelay(1000);
		if (move_flag == 1){
			move(1);
			osDelay(delay_flag); //TODO change to distance instead of time
			move_flag = 0;
		}
		if (move_flag == 2){
			move(2);
			osDelay(delay_flag);
			move_flag = 0;
		}
		if (move_flag == 0){
			stop();
		}
		osDelay(1);
	}
}

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	/* This function is (automatically?) called upon interrupt when getting input via the UART i think */
	//TODO: completely untested

	/* special characters:
		: = clear buffer
		; = end of command, set ready flag
		command logic is in parse_command()
	*/

	// prevent unused argument(s) compilation warning

	UNUSED(huart);

	if (command_len == MAX_BUF_SIZE - 1){ // for null terminator
		// TODO: transmit to the pi that command exceeds buffer.
		// Refresh the command buffer
		command_buf[0] = '\0';
		command_len = 0;
		display("out of spaaace", 1);
	}

	if (uart_input == ':') {
		command_len = 0;
		command_buf[0] = '\0';
	} else if (uart_input == ';'){
		command_buf[command_len] = '\0';
		command_len = 0;
		ready_parse = 1;
	} else {
		command_buf[command_len] = uart_input; // append only if not special character
		command_len++;
	}
	send("buf:");
	send(command_buf);
	send("     |");
	// wait for another receive
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

void parse_command(){
	if (ready_parse == 1){
		ready_parse = 0;
		char* token = strtok(command_buf, ";");

		while (token != NULL) {
			//xQueueSend(commandQueue, token, pdMS_TO_TICKS(100));
			xQueueSend(commandQueue, token, portMAX_DELAY);
			token = strtok(NULL, ";");
		}

		memset(command_buf, 0, MAX_BUF_SIZE); //idk if i need this
	}
}

void execute_command_thread(void* args){
	/*
	 * Command structure (TODO probably will be changed. idk i just put something here to have something to test)
	 * <Command>_<Value>
	 * Commands:
	 * 	- FWD : Forward. Value is Time duration in ms
	 * 	- BCK : Backward. Value is Time duration in ms
	 * 	- LFT : Turn front wheels left. Value is angle degrees from straight
	 * 	- RGT : Turn front wheels right. Value is angle degrees from straight
	 *
	 * 	Example:
	 * 	"FWD 2000;" -> Move forward 2 seconds
	 * 	"LFT 10; BCK 1000;" -> Turn front wheels 10 degrees left (from straight), then move backward 1second
	 */

	char tmp[20];
	char command[4]; // hard force command to only be 3 characters long
	int value;
	const char *commands[] = {"FWD\0", "BCK\0", "LFT\0", "RGT\0"
			, "FWL\0", "FWR\0", "BKL\0", "BKR\0"};

	Command cmd;

	for(;;){
		if(ready_execute == 1){
			if(xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdPASS){
				ready_execute = 0;
				//display(command_buf, 1);

				strcpy(tmp, "");
				value = 0;

				strncpy(tmp, (const char*)cmd.command, cmd.command_len); //get rid of volatile compilation warning
				tmp[cmd.command_len] = '\0';
				sscanf(tmp, "%s %d", command, &value);


				if (strcmp(command, commands[0]) == 0 ){ //FWD
					//sprintf(command, "MOVE FWD %d ms, %d", value, move_flag); //TESTING
					//display(command,1);
					move_flag = 1;
					delay_flag = value;
					osDelay(value);
					send("OK!");
				}
				if (strcmp(command, commands[1]) == 0){ //BCK
					move_flag = 2;
					delay_flag = value;
					osDelay(value);
					send("OK!");
				}
				if (strcmp(command, commands[2]) == 0){ //LFT
					turn_flag = value; //TODO: change to left
					osDelay(1000);
					send("OK!");
				}
				if (strcmp(command, commands[3]) == 0){ //RGT
					turn_flag = value; //TODO: change to right
					osDelay(1000);
					send("OK!");
				}
				if (strcmp(command, commands[4]) == 0){ //FL
					// hard coded for now, since i cant figure out the encoder pid shyttt
					turn_flag = SERVO_LEFT;
					osDelay(500); // wait for the wheel to turn
					move_flag = 1;
					delay_flag = 1300;
					osDelay(1300);
					turn_flag = SERVO_STRAIGHT;
					send("OK!");
				}
				if (strcmp(command, commands[5]) == 0){ //FR
					turn_flag = SERVO_RIGHT;
					osDelay(500); // wait for the wheel to turn
					move_flag = 1;
					delay_flag = 1500;
					osDelay(1500);
					turn_flag = SERVO_STRAIGHT;
					send("OK!");
				}
				if (strcmp(command, commands[6]) == 0){ //BL
					// hard coded for now, since i cant figure out the encoder pid shyttt
					turn_flag = SERVO_LEFT;
					osDelay(500); // wait for the wheel to turn
					move_flag = 2;
					delay_flag = 1200;
					osDelay(1200);
					turn_flag = SERVO_STRAIGHT;
					send("OK!");
				}
				if (strcmp(command, commands[7]) == 0){ //BR
					turn_flag = SERVO_RIGHT;
					osDelay(500); // wait for the wheel to turn
					move_flag = 2;
					delay_flag = 1500;
					osDelay(1500);
					turn_flag = SERVO_STRAIGHT;
					send("OK!");
				}
				// reset the shared vars
				//memset(command_buf, 0, MAX_BUF_SIZE);

				// wait 100ms after each command so the bot has time to stop...
				// idk if this is necessary, maybe can try to remove this in the future?
				// might have to change the motorthread, there is a race condition without this delay...
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
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM12_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  IR_Sensors_Init();

  // SERVO
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

  // DC MOTORS
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  // encoders?
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);


  // wait for an input
  // reads 1 character at a time
  HAL_UART_Receive_IT(&huart3, (uint8_t *) &uart_input, 1);
  //HAL_UART_Transmit_IT(&huart3, (uint8_t *) &command_buf, command_len, 0xFFFF); // send instruction

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  commandQueue = xQueueCreate(2, sizeof(Command));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder, NULL, &EncoderTask_attributes);

  /* creation of readICMTask */
  readICMTaskHandle = osThreadNew(readICM, NULL, &readICMTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  oledTaskHandle = osThreadNew(display_thread, NULL, &oledTask_attributes);
  moveTaskHandle = osThreadNew(testMoveThread, NULL, &moveTask_attributes);
  turnTaskHandle = osThreadNew(testTurnThread, NULL, &turnTask_attributes);
  parseCommandTaskHandle = osThreadNew(parse_command_thread, NULL, &parseCommandTask_attributes);
  executeCommandTaskHandle = osThreadNew(execute_command_thread, NULL, &executeCommandTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

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
  /* USER CODE BEGIN encoder */
	uint16_t CA, CB, prevCA, prevCB;
	int16_t Adiff, Bdiff;
	int16_t Adiffraw;

	float Adist = 0; //temp vars
	float Bdist = 0;

	char buf[20] = "TESTING!";

	prevCA = __HAL_TIM_GET_COUNTER(&htim2);
	prevCB = __HAL_TIM_GET_COUNTER(&htim3);

  /* Infinite loop */
  for(;;)
  {
	  //motor A
	  CA = __HAL_TIM_GET_COUNTER(&htim2);
	  Adiffraw = (int16_t) CA - prevCA;

	  // checking under/over flow
	  if (Adiff > 32767){
		  Adiff = Adiffraw - 65536;
	  }
	  if (Adiff < -32767){
		  Adiff = Adiffraw + 65536;
	  }
	  prevCA = CA;

	  //motor B
	  CB = __HAL_TIM_GET_COUNTER(&htim3);
	  Bdiff = CB - prevCB;

	  // checking under/over flow
	  if (Bdiff > 32767){
		  Bdiff = Bdiff - 65536;
	  }
	  if (Bdiff < -32767){
		  Bdiff = Bdiff + 65536;
	  }
	  prevCB = CB;


	  Adist += (float)Adiff / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;
	  Bdist += (float)Bdiff / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;

	  //itoa(Adist * 1000, buf, 10);
	  sprintf(buf, "%d %d", CA, Adiffraw);
	  //OLED_ShowString(10,20, buf);
	  display(buf, 2);
	  vTaskDelay(pdMS_TO_TICKS(1));
	  osDelay(1); // idk why but it breaks if theres no delay.
  }
  /* USER CODE END encoder */
}

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

	//sprintf(buf, "%d", theta);
	//display(buf, 1);
	if (theta < 35.0f) {
		display("LESS", 1);
	}else {
		display("MORE", 1);
	}

	sprintf(buf2, "%d", gyro_z_raw);
	display(buf2, 3);


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
