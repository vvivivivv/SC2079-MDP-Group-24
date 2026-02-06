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
 TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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

osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "CommandTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


// variables for communication with pi via UART
#define MAX_BUF_SIZE 20
volatile uint8_t command_buf[MAX_BUF_SIZE];
volatile char uart_input;
volatile int command_len = 0;

// public variables for multi threading
volatile uint8_t display_buf[20];
volatile int ready_execute = 0;

// used as shared variables for move and turn args
volatile int move_flag = 0;
volatile int delay_flag = 1000;
volatile int turn_flag = -1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

void parse_command_thread(void *args);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void display(uint8_t* str){
	// display_thread just looks at display_buf.
	strncpy(display_buf, (const char*) str, MAX_BUF_SIZE); //get rid of volatile compilation warning
	// display_buf[strlen(str)] = '\0';
	return;
}


void display_thread(void* args){
	uint8_t buf[20];

	//int i =0;
	for (;;){
		/*
		if (i == 0){
			display("aaa");
			i = 1;
		}
		else {
			display("bbb");
			i = 0;
		}
		*/

		strncpy(buf, (const char*)display_buf, MAX_BUF_SIZE); //get rid of volatile compilation warning
		//sprintf(buf, "testing %s\0", display_buf);
		OLED_Clear();
		OLED_ShowString(10, 10, buf);

		OLED_Refresh_Gram();
		osDelay(500);
	}
}

void stop(){
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - forward
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin,GPIO_PIN_RESET);
}

void move(int direction){
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
			display(command);

			osDelay(2000);
			turn(turn_flag);
			turn_flag = -1;
		}
	}
}

void turn(int value){
	// TODO: calibrate the values
	//htim12.Instance->CCR3=85;

	if (value < 85 || value > 240){
		display("value too extreme");
		return;
	}

	htim12.Instance->CCR1 = value; // straight

	/*
	for (;;){
		htim12.Instance->CCR1=85; //left

		osDelay(1000);
		htim12.Instance->CCR1 = 170; // straight
		osDelay(1000);
	}
	int start = 160;

	for(;;){
		if (start > 200){
			return;
		}
		start += 5;
		htim12.Instance->CCR1 = start; //right
		display(start);
		HAL_Delay(2000);
		//htim12.Instance->CCR1 = 72; // center
		//HAL_Delay(2000);
		//htim12.Instance->CCR1 = 60; // left
		//HAL_Delay(2000);
		//htim12.Instance->CCR1 = 72; // center
	}
	*/
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
		display("out of spaaace");
	}

	if (uart_input == ':') {
		command_len = 0;
		command_buf[0] = '\0';
	} else if (uart_input == ';'){
		command_buf[command_len] = '\0';  // Null terminate
		ready_execute = 1; //TODO: multitasking
	} else {
		command_buf[command_len] = uart_input; // append only if not special character
		command_len++;
	}

	// wait for another receive
	HAL_UART_Receive_IT(&huart3, &uart_input, 1);
}

void parse_command_thread(void* args){
	//TODO UNTESTEDDDD

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

	const char *commands[] = {"FWD\0", "BCK\0", "LFT\0", "RGT\0"};

	char tmp[20];
	char command[20];
	int value;
	for(;;){
		if (ready_execute == 1){
			strncpy(tmp, (const char*)command_buf, command_len); //get rid of volatile compilation warning

			sscanf(tmp, "%s %d", command, &value);

			display(command);

			// TODO check values are valid

			if (strcmp(command, commands[0]) == 0 ){ //FWD
				//sprintf(command, "MOVE FWD %d ms, %d", value, move_flag); //TESTING
				//display(command);
				move_flag = 1;
				delay_flag = value;
			}
			if (strcmp(command, commands[1]) == 0){ //BCK
				move_flag = 2;
				delay_flag = value;
			}
			if (strcmp(command, commands[2]) == 0){ //LFT
				turn_flag = value; //TODO: change to left
			}
			// reset the shared vars
			ready_execute = 0;
			command_len = 0;
			memset(command_buf, 0, MAX_BUF_SIZE);
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
  /* USER CODE BEGIN 2 */
  OLED_Init();
  IR_Sensors_Init();
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  oledTaskHandle = osThreadNew(display_thread, NULL, &oledTask_attributes);
  moveTaskHandle = osThreadNew(testMoveThread, NULL, &moveTask_attributes);
  turnTaskHandle = osThreadNew(testTurnThread, NULL, &turnTask_attributes);
  commandTaskHandle = osThreadNew(parse_command_thread, NULL, &commandTask_attributes);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BIN1_Pin|BIN2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin_Pin|OLED_RES_Pin_Pin|OLED_SDA_Pin_Pin|OLED_SCL_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AIN2_Pin|AIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
		HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, 0xFFFF); // send instruction
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
