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
#include "pid.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

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

//ultrasonic
osThreadId_t ultrasonicTaskHandle;
const osThreadAttr_t ultrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 256 * 8,
  .priority = (osPriority_t) osPriorityLow,
};

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
  .stack_size = 256 * 12,
  .priority = (osPriority_t) osPriorityLow,
};

// icm20948 (gyro)
#define ICM20948_I2C_ADDR   (0x68 << 1)
volatile float gyro_z_dps = 0.0f;

// Servo constants
const int SERVO_STRAIGHT = 172;
const int SERVO_LEFT_MAX = 95; //100
const int SERVO_LEFT = 125; // 130 changed to 115 test again tmr // 135 == 265 in terms of diameter (~50cm inner circle), BUT 125 == 265 for circumference??? ????
const int SERVO_RIGHT = 265;
const int SERVO_RIGHT_MAX = 265; // PROBABLY CAN GO HIGHER, BUT NOT TESTED

// variables for communication with pi via UART
#define MAX_BUF_SIZE 20
volatile uint8_t command_buf[MAX_BUF_SIZE];
volatile char uart_input;
volatile int command_len = 0;
/* USER CODE BEGIN PV */
// ... 原有代�? ...

// 1. PID 目标值和状�?
const int TARGET_TICK = 180;     // 50ms内期望脉冲数（根�?�需�?调整）
const float Kp_speed = 15.0f;    // 速度环 Kp
const float Kp_straight = 12.0f; // 左�?��?�步 Kp

// 2. 外部调用的 PWM 最终输出值
volatile int pwm_outA = 0;
volatile int pwm_outB = 0;

//UART Buffer
uint8_t rxBuffer[120];

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
volatile int approaching_flag = 0; // for when the dist to go is less than 15, set this flag. the encoder then modifies target speed into half.

// used as shared variables for move and turn args
volatile int move_flag = 0;
volatile int delay_flag = 1000;
volatile int turn_flag = -1;
volatile int turned_angle = 0; //track how many degrees the car has turned
volatile int target_angle = 0;
volatile int straighten_flag = 0; // bad implementation imo, but it tells the move_thread whether to use the gyro to adjust itself to go straight.
volatile int turn_in_place_flag = 0; // set when doing turn in place so that the encoder pid thread can alter the speed

const int SPEED_MAX = 7190;
const int SPEED_HIGH = 6000;
const int SPEED_NORM_HIGH = 4500;
const int SPEED_NORM = 3000;
const int SPEED_NORM_LOW = 2000;
const int SPEED_SLOW = 1500;
const int SPEED_VERY_SLOW = 700;
const int SPEED_MIN = 700;
const int SLOW_DOWN_RATIO = 4; // when approaching, it divides the current target speed by this ratio.

volatile int speedA = SPEED_NORM; // the pwm wave, 7199 is max i think
volatile int speedB = SPEED_NORM;
volatile int speedA_target = 0; // the target speed to be achieved by the pid (represented in a pvm format :))
volatile int speedB_target = 0;

// used for encoders
//const float ENCODER_COUNTS_PER_REVOLUTION = 1540.0f;
//const float ENCODER_COUNTS_PER_REVOLUTION = 680.0f;
const float ENCODER_COUNTS_PER_REVOLUTION = 770.0f;
const float WHEEL_CIRCUMFERENCE_CM = 6.5f*3.14f * 10; // im changing this to mm but im lazy to change all the constants to MM so im just going to *10 here.
volatile int rotations = 0;

volatile double Adist = 0; //temp vars
volatile double Bdist = 0;
volatile double average_dist = 0;

// trying pid controller
static PidDef pidMatch;
const static float Kp_match = 5e4;
const static float Ki_match = 7e2;
const static float Kd_match = 3e3;

static int16_t pwmValAccel = 0, pwmValTarget = 0,
	lPwmVal = 0, rPwmVal = 0;


// trying to mitigate overshooting encoder by calibrating an offset...
float forward_error = 0;
float backward_error = 0;

// float errors[6] = {5, -18, 0, +25, 10, -10}; // fw, bw, fl, fr, bl, br. i forgor how to dict in C
float errors[6] = {6, 7, 13, 14, 15, 13};
int angle_errors[4] = {2,2,1,2,1}; // fl fr bl br cw

//Ultrasonic Variables
int ultrasonicReady = 0;
int ultraTrigger = 1; // 0 if Trigger not sent yet | 1 if Trigger is sent

int echoStart = 0;
int echoStop = 0;
int echoDiff = 0;
float ultraDist = 0;

int task2_done = 0;
#define CMD_LEN 40
char task2_buf[CMD_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM1_Init(void);
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
	HAL_UART_Transmit(&huart3, (uint8_t *) str, strlen(str), 0xFFFF);
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
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, 0); // set all too high for brake
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 0);
	TIM4->CNT = 0;
	TIM9->CNT = 0;
	/*
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // set direction of rotation for wheel A - forward
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin,GPIO_PIN_RESET);
	*/
}

float check_ultrasonic(){
	// turn on flag
	ultraTrigger = 0;
	// get and return result
	while (ultraTrigger != 2){
		osDelay(1);
	}

	return ultraDist;

}
void task2(){
	int u_dist;
	task2_done = 0;

	char buf[20];
	int next_dir = 0; // 0 for left, 1 for right?

	int angle_cp;
	// 1. check ultrasonic dist

	// 2. move until 30cm from the wall (let this be point A)

	// temp valule
	//u_dist = 500;
	//sprintf(task2_buf, "FW %d", u_dist - 300);
	//xQueueSend(commandQueue, task2_buf, portMAX_DELAY);

	// 3. wait until rpi says left / right

	// 4. hardcoded left / right
	if (next_dir == 0){
		//xQueueSend(commandQueue, "FL 40", 0); // to be tested
		//xQueueSend(commandQueue, "FR 80", 0);
		//xQueueSend(commandQueue, "FL 40", 0);
		fl(40,0);
		osDelay(150);
		fr(40,0);
		fr(40,0);
		osDelay(150);
		fl(40,0);
		//osDelay(5000);
	}

	// 4.5 use ultra to check how close it is
	// for now, move backwards a bit to account for the next turn

	bw(200);

	// 5. repeat 1-3.

	// 6. hardcode turn left / right
	if (next_dir == 0){
		//xQueueSend(commandQueue, "FL 90", 0); // to be tested
		fl(90, 0);
	}

	// 7. follow ir sensor until wall disappears
	if (next_dir == 0){
		if (irSense(1) == 0){ // assuming have not overshot the wall. 0 is detected
			while (irSense(1) == 0){ // keep going until overshot
				fw(50); // chang ethis
			}
		}
		buzz();
		osDelay(1000);
		buzz();
	}
	// 8. uturn
	angle_cp = turned_angle;
	if (next_dir == 0){
		set_speed(SPEED_NORM_HIGH, SPEED_MIN);
		//set_speed(SPEED_NORM_LOW, SPEED_NORM_LOW / 2.5);
		turn_flag = SERVO_RIGHT;
		osDelay(100); // wait for the servo to turn

		/* this line treats value as a dist
		move_by_dist(1, value - errors[3]); //-25 // 20 to 25

		*/
		target_angle = turned_angle - 90;
		move_flag = 1;

		while (turned_angle > target_angle + angle_errors[1]){ // turning right makes gyro go negative
			osDelay(1);
		}
		move_flag = 0;
		turn_flag = SERVO_STRAIGHT;

		bw(300, 0);

		set_speed(SPEED_NORM_HIGH, SPEED_MIN);
		//set_speed(SPEED_NORM_LOW, SPEED_NORM_LOW / 2.5);
		turn_flag = SERVO_RIGHT;
		osDelay(100); // wait for the servo to turn

		/* this line treats value as a dist
		move_by_dist(1, value - errors[3]); //-25 // 20 to 25

		*/
		target_angle = turned_angle - 90;
		move_flag = 1;

		while (turned_angle > target_angle + angle_errors[1]){ // turning right makes gyro go negative
			osDelay(1);
		}
		move_flag = 0;
		turn_flag = SERVO_STRAIGHT;
	}
	osDelay(1000);

	// 9. follow ir sensor until wall disappears
	if (next_dir == 0){
		if (irSense(1) == 0){ // assuming have not overshot the wall. 0 is detected
			while (irSense(1) == 0){ // keep going until overshot
				fw(50); // chang ethis
			}
		}
		buzz();
		osDelay(1000);
		buzz();
	}
	// 10. turn back and move straight until approx point A (measure distance moved since A?)
	if (next_dir == 0){
		//fr(90);
		// anticipate the uturn to overshoot.
		target_angle = abs(angle_cp) + 90 + 180 - abs(turned_angle);
		sprintf(buf, "%d %d %d", angle_cp, turned_angle, target_angle);
		send(buf);
		fr(target_angle);
	}
	return;
	// 10. 45 degree turn into the carpark

	if (next_dir == 0){
		xQueueSend(commandQueue, "FR 45", 0); // to be tested
		xQueueSend(commandQueue, "FW 100", 0);
	}
}

void irSense(int dir){
	// for now just turn on the buzzer
	// dir =0 for left, 1 for right
	if (dir == 0)
		return IR_LeftDetected(); //0 means something is there.
	else
		return IR_RightDetected();
	/*
	if (dir == 0){
		return IR_LeftDetected() == 0){
				buzz();
				osDelay(1000);
				buzz();
				return;
			}
			osDelay(1);
		}
	}
	if (dir == 1){
		while (1){
			if (IR_RightDetected() == 0){
				buzz();
				osDelay(1000);
				buzz();
				return;
			}
			osDelay(1);
		}
	}
	*/
}
void move(int direction){
    if (direction == 1){ // �?进
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm_outA); // 使用 PID �?�的值

        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 0);
        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_outB); // 使用 PID �?�的值
    }
    else if (direction == 2){ // �?�退
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwm_outA);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);

        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, pwm_outB);
        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0);
    }
}
//void move(int direction){
//	//int pwmVal = motor_speed;
//	if (direction == 1){
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0); // set IN1 to maximum PWM (7199) for '1'
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, speedA); // PWM to Motor A (IN2)
//
//		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, 0);
//		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1, speedB); // PWM to Motor B (IN2)
//	}
//	else if (direction == 2){
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, speedA);
//		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 0); // PWM to Motor A (IN1)
//
//		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, speedB);
//		__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0); // PWM to Motor B (IN2)
//	}
//}

//void testMoveThread(void* arg){
//    for (;;){
//        if (move_flag != 0){
//            // 现在的 speedA 和 speedB 已�?在 encoder 里被 PID 实时算好了
//            move(move_flag);
//
//            // 如果你�?�常想�?留陀螺仪微调，�?�在这里�?留一�?段
//            if (straighten_flag == 1) {
//                // 这里的 turn() �?�影�?�?轮方�?�，�?影�?�?�轮动力，所以�?冲�?
//                // ... 你原本的陀螺仪 turn 代�? ...
//            }
//        } else {
//            stop();
//        }
//        osDelay(10); // 物�?�刷新�?�以快一点
//    }
//}

void buzz(){
	// toggles the buzzer.
	HAL_GPIO_TogglePin(Buzz_Pin_GPIO_Port, Buzz_Pin_Pin);
}


int pwm_to_encoder(int pwm){
	// returns the expected encoder value for a given pwm value.
	// i wrote down the encoder values for pwm values starting from 0 to 4000 and stepping by 200
	// assume the linear line between 2 points is close enough to the actual pwm value corresponding to that encoder
	// example: given pwm value of 2444, it will be map[2400] + (map[2600] - map[2400] * 44/200)

	// using an array since its in the helper function. just multiply the index by 200.
	//const int map = {};

	// OK WELL. during testing it seems like linearly every 200 increase in pwm = 6.7 (lol) increase in encoder.
	// the starting value of 600 = 16
	// or at least, its veeeery close to it.
	return (pwm - 600)/200 * 6.7 + 16;
}

int encoder_to_pwm(int encoder){
	// read pwm_to_encoder and its just the opposite of the formula there.
	return (encoder - 16) / 6.7 * 200 + 600;
}

/*
void testMoveThread(void* arg){
    float Kp_gyro = 1.5f; // 这个系数决定了修正的�?��?度
    static int start_angle = 0;
    static int is_moving = 0;

    for (;;){
        if (move_flag != 0){
            // 记录刚开始移动时的角度作为基准
            if (is_moving == 0) {
                start_angle = turned_angle;
                is_moving = 1;
            }

            if (straighten_flag == 1){
                // 计算角度�??差
                int angle_error = turned_angle - start_angle;

                // PID 核心逻辑：转�?�角度 = 中值 + (�??差 * 系数)
                // 如果是�?进(move_flag=1)，逻辑如下：
                int servo_output = SERVO_STRAIGHT;
                if (move_flag == 1) {
                    servo_output = SERVO_STRAIGHT + (int)(angle_error * Kp_gyro);
                } else { // �?�退(move_flag=2) 修正方�?�相�??
                    servo_output = SERVO_STRAIGHT - (int)(angle_error * Kp_gyro);
                }

                // �?幅：防止舵机打死
                if(servo_output > SERVO_RIGHT_MAX) servo_output = SERVO_RIGHT_MAX;
                if(servo_output < SERVO_LEFT_MAX) servo_output = SERVO_LEFT_MAX;

                turn(servo_output);
            }

            // 这里的 move 内部会自动使用 encoder 线程算好的 speedA/B
            move(move_flag);
        } else {
            if (is_moving == 1) {
                stop();
                turn(SERVO_STRAIGHT); // �?�下时舵机回正
                is_moving = 0;
            }
        }
        osDelay(20); // 给系统和传感器留点喘�?�时间（50Hz 刷新率）

    }
}
*/

void testMoveThread(void* arg){
	char command[20];
	int start_angle;

	/*
	char buf[20];
	sprintf(buf, "%d < %d", (int)start_angle ,  (int)turned_angle);
	display(buf, 2);
	*/

	for (;;){
		//sprintf(command, "IN MOVE %d ! %d", move_flag, delay_flag); //TESTING
		//display(command);
		//osDelay(1000);
		if (move_flag == 1){
			move(1);
			start_angle = turned_angle;

			// TODO: this should go somewhere else in the future, just here to test
			// read gyroscope and try to keep it going straight
			while (move_flag == 1 && straighten_flag == 1){
				move(1); // speedA is volatile, so the move thread (self) has to either keep track of when it changes, or just always call it lol
				if (start_angle < turned_angle){
					// drifting left
					turn(SERVO_STRAIGHT + 10); // temp value
				} else if (start_angle > turned_angle){
					//drifting right
					turn(SERVO_STRAIGHT - 10); //temp value
				} else {
					turn(SERVO_STRAIGHT);
				}

				//sprintf(buf, "%d < %d", (int)start_angle, (int)turned_angle);
				//display(buf, 2);
			}
		}
		if (move_flag == 2){
			move(2);
			start_angle = turned_angle;

			// read gyroscope and try to keep it going straight
			while (move_flag == 2 && straighten_flag == 1){
				move(2); // speedB is volatile, so the move thread (self) has to either keep track of when it changes, or just always call it lol
				if (start_angle < turned_angle){
					// drifting left
					turn(SERVO_STRAIGHT - 10); // temp value
				} else if (start_angle > turned_angle){
					//drifting right
					turn(SERVO_STRAIGHT + 10); //temp value
				} else {
					turn(SERVO_STRAIGHT);
				}
			}
			//osDelay(delay_flag);
			//move_flag = 0;
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
			//display(command, 1);

			turn(turn_flag);
			turn_flag = -1;
		}
	}
}

void turn(int value){
	// turns the front wheels
	if (value < SERVO_LEFT_MAX || value > SERVO_RIGHT_MAX){
		//display("value too extreme", 1);
		return;
	}

	htim12.Instance->CCR1 = value; // straight
}

void turn_in_place(int angle, int dir){

	char buf[20];
	//sprintf(buf, "%d > %d",  (int)turned_angle, (int)target_angle);
	//display(buf, 2);

	// dir = 0 => Clockwise
	// dir = 1 => CounterClockwise

	int cp1 = 0; // so the speed doesnt keep getting reset to a min speed (due to the acceleration code in encoder)
	int cp2 = 0;
	int cnt = 0;
	turn_in_place_flag = 1;
	if (dir == 0) {
		int target_angle = turned_angle - angle;
		turn(SERVO_LEFT_MAX);
		osDelay(500); // wait for servo to turn
		for (;;){
			cnt += 1;

			if (turned_angle > target_angle){
				//sprintf(buf, "%d > %d",  (int)turned_angle, (int)target_angle);
				//display(buf, 2);
				if (turned_angle - target_angle < 30 && cp2 == 0) {
					// slow down when approaching the target angle
					set_speed(SPEED_SLOW/1.5, SPEED_NORM/1.5);
					speedA = SPEED_SLOW/1.5; // min speed is too slow
					speedB = SPEED_NORM/1.5;
					cp2 = 1;

				} else if (cp1 == 0){
					set_speed(SPEED_SLOW, SPEED_NORM);
					speedA = SPEED_SLOW;
					speedB = SPEED_NORM;
					cp1 = 1;
				}
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0); // set IN1 to maximum PWM (7199) for '1'
				__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, pwm_outA); // PWM to Motor A (IN2)
				TIM4->EGR | TIM_EGR_UG;
				TIM4->SR &= ~TIM_SR_UIF;

				__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, pwm_outB);
				__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0); // PWM to Motor B (IN2)
				TIM9->EGR | TIM_EGR_UG;
				TIM9->SR &= ~TIM_SR_UIF;
				if (cnt > 1000){
					sprintf(buf , "%d, %d", pwm_outA, pwm_outB);
					send(buf);
					cnt = 0;
				}
			}
			else {
				stop();
				turn(SERVO_STRAIGHT);
				turn_in_place_flag = 0;
				cp1 = 0;
				cp2 = 0;
				osDelay(100);
				return;
			}
		}
	}
	else if (dir == 1){
		int target_angle = turned_angle + angle;
		turn(SERVO_RIGHT_MAX);
		osDelay(500); // wait for servo to turn
		for (;;){
			if (turned_angle < target_angle){
				sprintf(buf, "%d > %d",  (int)turned_angle, (int)target_angle);
				display(buf, 2);
				if (target_angle - turned_angle < 30) {
					// slow down when approaching the target angle
					__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, SPEED_NORM); // set IN1 to maximum PWM (7199) for '1'
					__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 0); // PWM to Motor A (IN2)

					__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, 0);
					__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, SPEED_SLOW); // PWM to Motor B (IN2)
				} else {
					__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4, SPEED_NORM); // set IN1 to maximum PWM (7199) for '1'
					__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 0); // PWM to Motor A (IN2)

					__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, 0);
					__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, SPEED_HIGH); // PWM to Motor B (IN2)
				}
			}
			else {
				stop();
				turn(SERVO_STRAIGHT);
				turn_in_place_flag = 0;
				return;
			}
		}
	}
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

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
//	HAL_UART_Transmit(&huart3, rxBuffer, 10, 10);
//	if(huart == &huart3){
//		rxBuffer[Size] = "\0";
//		HAL_UART_Transmit(&huart3, rxBuffer, 10, 10);
//		snprintf(command_buf, Size, strtok(rxBuffer, ";"));
//		if(strlen(command_buf) > 0) ready_parse = 1;
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, 100);
//	}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART3) {
        // 1. 处�?�清除符 ':'
        if (uart_input == ':') {
            command_len = 0;
            memset((uint8_t*)command_buf, 0, MAX_BUF_SIZE);
            send((uint8_t*)"Buffer Cleared\r\n");
        }
        // 2. 处�?�指令结�?�符 ';'
        else if (uart_input == ';'){
            command_buf[command_len] = '\0';

            // --- 核心修�?：手动将数�?��?装并�?入队列 ---
            Command cmd;
            memset(cmd.command, 0, 20);
            strncpy((char*)cmd.command, (const char*)command_buf, command_len);
            cmd.command_len = command_len;

            // 使用 FromISR 版本�?��?到队列
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(commandQueue, &cmd, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            // ---------------------------------------

            command_len = 0; // �?置缓冲区索引
            ready_execute = 1;
        }
        // 3. 正常字符处�?�（修�? 0 输入问题）
        else {
            if (uart_input >= 32 && uart_input <= 126) {
                if (command_len < MAX_BUF_SIZE - 1) {
                    command_buf[command_len++] = uart_input;
                    command_buf[command_len] = '\0';
                }
            }
        }

        // 4. 调试回显
        //send((uint8_t*)"buf:");
        //send((uint8_t*)command_buf);
        // (uint8_t*)"\r\n");

        // 5. �?新开�?�接收
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart_input, 1);
    }
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	/* This function is (automatically?) called upon interrupt when getting input via the UART i think */
//	//TODO: completely untested
//
//	/* special characters:
//		: = clear buffer
//		; = end of command, set ready flag
//		command logic is in parse_command()
//	*/
//
//	// prevent unused argument(s) compilation warning
//
//	UNUSED(huart);
//
//	if (command_len == MAX_BUF_SIZE - 1){ // for null terminator
//		// TODO: transmit to the pi that command exceeds buffer.
//		// Refresh the command buffer
//		command_buf[0] = '\0';
//		command_len = 0;
//		display("out of spaaace", 1);
//	}
//
//	if (uart_input == ':') {
//		command_len = 0;
//		command_buf[0] = '\0';
//	} else if (uart_input == ';'){
//		command_buf[command_len] = '\0';
//		command_len = 0;
//		ready_parse = 1;
//	} else {
//		command_buf[command_len] = uart_input; // append only if not special character
//		command_len++;
//	}
//	send("buf:");
//	send(command_buf);
//	send("     |");
//	// wait for another receive
//	HAL_UART_Receive_IT(&huart3, &uart_input, 1);
//}
//

//Thread for Ultrasonic
void ultrasonic_thread(void *args){
 while(1){
  if(ultraTrigger == 0){
   char ultrabuf[30];
   echoStart = 0;
   echoStop = 0;
   sprintf(ultrabuf, "Sonic Triggered");
   display((uint8_t*)ultrabuf, 1);
   HAL_GPIO_WritePin(TriggerPin_GPIO_Port, TriggerPin_Pin, GPIO_PIN_SET);

   // Allowance for Trigger to wait 10us
   __HAL_TIM_SET_COUNTER(&htim1, 0);
   while(__HAL_TIM_GET_COUNTER(&htim1) < 10);

   HAL_GPIO_WritePin(TriggerPin_GPIO_Port, TriggerPin_Pin, GPIO_PIN_RESET);
   __HAL_TIM_SET_COUNTER(&htim1, 0);
   while(__HAL_TIM_GET_COUNTER(&htim1) < 60);
   ultraTrigger = 1;
  }
  else if(ultrasonicReady){
   char ultrabuf[30];
   sprintf(ultrabuf, "Distance: %.2f", ultraDist);
   display((uint8_t*)ultrabuf, 1);
   ultrasonicReady = 0;
   ultraTrigger = 2;
  }

  osDelay(10);
 }
}

//GPIO Interrupt Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	char buf[20];
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

  if(GPIO_Pin == EchoPin_Pin){
   if(ultraTrigger){
    if(HAL_GPIO_ReadPin(EchoPin_GPIO_Port, EchoPin_Pin) == GPIO_PIN_SET){
     echoStart = __HAL_TIM_GET_COUNTER(&htim1);
     sprintf(buf, "EchoStart: !%d!\n", echoStart);
     send(buf);
    }
    else if(HAL_GPIO_ReadPin(EchoPin_GPIO_Port, EchoPin_Pin) == GPIO_PIN_RESET){
     echoStop = __HAL_TIM_GET_COUNTER(&htim1);
     sprintf(buf, "EchoStop: !%d!\n", echoStop);
     send(buf);
     if(echoStop >= echoStart) echoDiff = echoStop - echoStart;
     else echoDiff = (65535-echoStart) + echoStop;
     ultraDist = (float) echoDiff / 58.0f;
     ultrasonicReady = 1;
     sprintf(buf, "EchoDiff: !%d!\n", echoDiff);
     send(buf);
    }
   }
  }
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

void move_by_dist(int dir, int dist_target){
	// interacts with motor speed and encoder to slow down
	// triggers flags for move(), then stop() after dist is travelled.

	int starting_dist;
	int dist_moved;
	int tmp_a, tmp_b;
	int dist_to_go;
	char buf[20];

	int cp1 = 0; // checkpoints. set to true when the conditional is met so it doesnt run multiple times
	int cp2 = 0;
	int cp3 = 0;

	starting_dist = average_dist;
	move_flag = dir;

	 //TODO: change from average dist to a PID accounting for both dists

	dist_moved = average_dist - starting_dist;
	while (abs(dist_moved) < abs(dist_target)) {
		dist_moved = average_dist - starting_dist;
		sprintf(buf, "%d - %d = %d", abs(dist_target), abs(dist_moved), abs(dist_target) - abs(dist_moved));
		//fdisplay(buf, 2);

		dist_to_go = (abs(dist_target) - abs(dist_moved));
		if (cp1 == 0 && dist_to_go <= 150){
			// slow down if 10 cm away
			//TODO implement PIDDDDD AHHH
			//tmp = ((SPEED_SLOW - SPEED_VERY_SLOW) / 10 * dist_to_go)  + SPEED_VERY_SLOW;
			//speedA = tmp;
			//speedB = tmp;
			set_speed(speedA_target / 2, speedB_target /2 );
			cp1 = 1;
			//approaching_flag = 1;
		} else if (cp2 == 0 && dist_to_go <= 100){
			tmp_a = speedA_target / 2 < SPEED_MIN ? SPEED_MIN : speedA_target / 2;
			tmp_b = speedB_target / 2 < SPEED_MIN ? SPEED_MIN : speedB_target / 2;

			// IF TURNING, THEN THE OUTER WHEEL MUST PRESERVE THE RATIO. I HARDCODED THIS VALUE SO I PRAY THE RATIO WONT CHANGE LMAO
			if (speedA_target < speedB_target){
				if (tmp_a == SPEED_MIN){
					tmp_b = SPEED_MIN * 2;
				}
			} else if (speedB_target < speedA_target){
				if (tmp_b == SPEED_MIN){
					tmp_a = SPEED_MIN * 2;
				}
			}
			set_speed(tmp_a, tmp_b);
			cp2 = 1;
			//approaching_flag = 0;
		} else if (cp3 == 0 && dist_to_go <= 50){
			tmp_a = speedA_target / 2 < SPEED_MIN ? SPEED_MIN : speedA_target / 2;
			tmp_b = speedB_target / 2 < SPEED_MIN ? SPEED_MIN : speedB_target / 2;

			// IF TURNING, THEN THE OUTER WHEEL MUST PRESERVE THE RATIO. I HARDCODED THIS VALUE SO I PRAY THE RATIO WONT CHANGE LMAO
			if (speedA_target < speedB_target){
				if (tmp_a == SPEED_MIN){
					tmp_b = SPEED_MIN * 2;
				}
			} else if (speedB_target < speedA_target){
				if (tmp_b == SPEED_MIN){
					tmp_a = SPEED_MIN * 2;
				}
			}
			set_speed(tmp_a, tmp_b);
			cp3 = 1;
			//approaching_flag = 0;
		}
	}

	move_flag = 0;
}

void calibrate(){
	// I hate hate hate how it looks. its just copy pasted from the //fw //bw //fr etc.

	// keep moving forward by 10cm. track the actual distance moved. get the cndoer
	// get error for fw
	float sums[6] = {0,0,0,0,0,0};
	float prev_dist = average_dist;
	float error = 0;
	int value = 80;
	char buf[20];
	send("calibrating...");
	// getting fw error
	for (int i =0; i<5; i++){
		//xQueueSend(commandQueue, "FW 10", portMAX_DELAY);

		// fw
		set_speed(SPEED_SLOW, SPEED_SLOW);
		straighten_flag = 1;
		move_by_dist(1, value);
		straighten_flag = 0;

		osDelay(1000);
		sums[0] += average_dist - prev_dist - value;
		prev_dist = average_dist;

		// bw
		set_speed(SPEED_SLOW, SPEED_SLOW);
		straighten_flag = 1;
		move_by_dist(2, value); //+18 // for some reason, the robot is always short by 2cm.. so lol
		straighten_flag = 0;

		osDelay(1000);
		sums[1] += prev_dist - average_dist - value;
		prev_dist = average_dist;

		// fl
		set_speed(SPEED_SLOW/2 , SPEED_SLOW); // min to norm is 9:1 ratio.
		turn_flag = SERVO_LEFT;
		osDelay(500); // wait for the servo to turn
		move_by_dist(1, value);
		turn_flag = SERVO_STRAIGHT;

		osDelay(1000);
		sums[2] +=  average_dist - prev_dist - value;
		prev_dist = average_dist;

		//xQueueSend(commandQueue, "FW 10", portMAX_DELAY);

		set_speed(SPEED_SLOW , SPEED_SLOW/2); // min to norm is 9:1 ratio.
		turn_flag = SERVO_RIGHT;
		osDelay(500); // wait for the servo to turn
		move_by_dist(1, value);
		turn_flag = SERVO_STRAIGHT;

		osDelay(1000);
		sums[3] +=  average_dist - prev_dist - value;
		prev_dist = average_dist;

		//xQueueSend(commandQueue, "FW 10", portMAX_DELAY);

		set_speed(SPEED_SLOW, SPEED_SLOW /2 );
		turn_flag = SERVO_RIGHT;
		osDelay(500); // wait for the servo to turn
		move_by_dist(2, value); // 7 to 10 //TODO: this prob has to increase
		turn_flag = SERVO_STRAIGHT;

		osDelay(1000);
		sums[4] +=  prev_dist - average_dist  - value;
		prev_dist = average_dist;

	// BR ERROR
		//xQueueSend(commandQueue, "FW 10", portMAX_DELAY);

		set_speed(SPEED_SLOW/2, SPEED_SLOW );
		turn_flag = SERVO_LEFT;
		osDelay(500); // wait for the servo to turn
		move_by_dist(2, value); // 7 to 10 //TODO: this prob has to increase
		turn_flag = SERVO_STRAIGHT;

		osDelay(1000);
		sums[5] +=  prev_dist - average_dist - value;
		prev_dist = average_dist;

	}
	for (int i =0; i<6; i++){
		errors[i] = sums[i] / 5;
		sprintf(buf, "ER: %.1f\0", errors[i]);
		send(buf);
	}
}

void set_speed(int sA, int sB){
	speedA_target = sA;
	speedB_target = sB;
	speedA = sA;
	speedB = sB;

	/*
	// ACCOUNT FOR TURNING RATIO im too tired to make this look nice
	if (sA < sB){
		speedA = SPEED_MIN;
		speedB = SPEED_MIN * 2;
	} else if (sB < sA){
		speedA = SPEED_MIN * 2;
		speedB = SPEED_MIN;
	} else {
		speedA = SPEED_MIN;
		speedB = SPEED_MIN;
	}
	*/
}

void fw(int value){
	//set_speed(SPEED_SLOW, SPEED_SLOW);
	set_speed(SPEED_HIGH, SPEED_HIGH);
	straighten_flag = 1;
	move_by_dist(1, value); // -5 // OVERSHOOT BY ~5MM
	straighten_flag = 0;
}

void bw(int value){
	set_speed(SPEED_HIGH, SPEED_HIGH);
	straighten_flag = 1;
	move_by_dist(2, value - errors[1]); //+18 // for some reason, the robot is always short by 2cm.. so lol
	straighten_flag = 0;
}

/*
void fl_task2(value){
	// trying to get a tighter turn by spinning the wheel different dirs
	starting_dist = average_dist;

	while starting_dist < average
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0); // set IN1 to maximum PWM (7199) for '1'
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, pwm_outA); // PWM to Motor A (IN2)
	TIM4->EGR | TIM_EGR_UG;
	TIM4->SR &= ~TIM_SR_UIF;

	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2, pwm_outB);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0); // PWM to Motor B (IN2)
	TIM9->EGR | TIM_EGR_UG;
	TIM9->SR &= ~TIM_SR_UIF;
}
*/

void fl(value, wait){
	set_speed(SPEED_SLOW /2  , SPEED_SLOW);
	//set_speed(SPEED_NORM_LOW / 2.5, SPEED_NORM_LOW);
	turn_flag = SERVO_LEFT;
	if (wait == 1)	osDelay(500); // wait for the servo to turn
	//move_by_dist(1, value - errors[2]); // this line treats value as a dist

	target_angle = turned_angle + value;
	move_flag = 1;

	while (turned_angle < target_angle - angle_errors[0]) {
		/*
		// Slow down when approaching target
		if (turned_angle > target_angle - 20 ){
			motor_speed = SPEED_SLOW;
		}
		else {
		*/
		osDelay(1);
	}
	move_flag = 0; //stop
	if (wait == 1)	osDelay(100); //wait for inertia
	turn_flag = SERVO_STRAIGHT;
}

void fr(value, wait){
	set_speed(SPEED_SLOW, SPEED_SLOW / 2);
	//set_speed(SPEED_NORM_LOW, SPEED_NORM_LOW / 2.5);
	turn_flag = SERVO_RIGHT;
	if (wait == 1) osDelay(500); // wait for the servo to turn

	/* this line treats value as a dist
	move_by_dist(1, value - errors[3]); //-25 // 20 to 25

	*/
	target_angle = turned_angle - value;
	move_flag = 1;

	while (turned_angle > target_angle + angle_errors[1]){ // turning right makes gyro go negative
		osDelay(1);
	}
	move_flag = 0;
	if (wait == 1) osDelay(100); //wait for inertia
	turn_flag = SERVO_STRAIGHT;

}

void bl(value, wait){
	set_speed(SPEED_SLOW , SPEED_SLOW/2);
	//set_speed(SPEED_NORM_LOW / 2.5, SPEED_NORM_LOW);
	turn_flag = SERVO_RIGHT;
	if (wait == 1)	osDelay(500); // wait for the servo to turn
	//move_by_dist(1, value - errors[2]); // this line treats value as a dist

	target_angle = turned_angle + value;
	move_flag = 2;

	while (turned_angle < target_angle - angle_errors[0]) {
		/*
		// Slow down when approaching target
		if (turned_angle > target_angle - 20 ){
			motor_speed = SPEED_SLOW;
		}
		else {
		*/
		osDelay(1);
	}
	move_flag = 0; //stop
	if (wait == 1)	osDelay(100); //wait for inertia
	turn_flag = SERVO_STRAIGHT;
}

void br(value, wait){
	set_speed(SPEED_SLOW/2, SPEED_SLOW);
	//set_speed(SPEED_NORM_LOW, SPEED_NORM_LOW / 2.5);
	turn_flag = SERVO_LEFT;
	if (wait == 1) osDelay(500); // wait for the servo to turn

	/* this line treats value as a dist
	move_by_dist(1, value - errors[3]); //-25 // 20 to 25

	*/
	target_angle = turned_angle - value;
	move_flag = 2;

	while (turned_angle > target_angle + angle_errors[1]){ // turning right makes gyro go negative
		osDelay(1);
	}
	move_flag = 0;
	if (wait == 1) osDelay(100); //wait for inertia
	turn_flag = SERVO_STRAIGHT;

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
	char buf[20];
	int value;
	//const char *commands[] = {"FWD\0", "BCK\0", "LFT\0", "RGT\0"
	//		, "FWL\0", "FWR\0", "BKL\0", "BKR\0"};

	const char *commands[] = {"FW\0", "BW\0", "LFT\0", "CW\0", "CCW\0"
			, "FL\0", "FR\0", "BL\0", "BR\0"
			, "A4\0"}; // THERES SOME BUFFER OVERFLOW HAPPENING OR STH, THE FIRST 4 ELEMENTS ARE OVERWRITTEN. I CANT FIND IT SO LOL.

	Command cmd;

	// make sure the robot is straight before anything
	turn(SERVO_STRAIGHT);

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

				// REDUCE BY A CALIBRATED ERROR. THIS WILL LIKELY NEED TO CHANGE IN THE FUTURE TODO:
				// TEMP HARDCODE IT TO STOP 2cm early to prevent overshooting.
				//forward_error = 15;
				forward_error = 0;

				if (value > forward_error * 2) { // only minus if the distance is significant enough to maybe cause overshootin
					value -= forward_error;
				}

				display(tmp, 3);
				//send(command);
				if (strcmp(command,"FW") == 0 ){ //FWD

					/*
					for(int i = 600; i< 4000; i += 200){
						set_speed(SPEED_NORM, i);
						move_by_dist(1, 300);
						osDelay(3000);
					}

					continue;
					*/
					fw(value);
				}
				if (strcmp(command, "BW") == 0){ //BCK
					bw(value);
				}
				if (strcmp(command,  "LFT") == 0){ //LFT
					turn_flag = value; //TODO: change to left
					osDelay(1000);
				}
				if (strcmp(command, commands[3]) == 0){ //CW //testing turn in place
					turn_in_place(value, 0); // hardcode a -1 for inertia... it feels bad but it kinda works so eh
				}
				if (strcmp(command, commands[4]) == 0){ //CCW //testing turn in place
					turn_in_place(value, 1);
				}
				if (strcmp(command, commands[5]) == 0){ //FL
					//set_speed(SPEED_SLOW, SPEED_NORM);
					fl(value, 1);
				}
				if (strcmp(command, commands[6]) == 0){ //FR
					fr(value, 1);
					/* Already forcing slow speed
					while (turned_angle > target_angle) {
						// Slow down when approaching target
						if (turned_angle < target_angle + 20 ){
							motor_speed = SPEED_SLOW;
						}
						else {
							osDelay(10);
						}
					}

					move_flag = 0; //stop
					motor_speed = SPEED_NORM; //TODO: use a constant here prob

					turn_flag = SERVO_STRAIGHT;
					*/

				}
				if (strcmp(command, commands[7]) == 0){ //BL
					set_speed(SPEED_SLOW, SPEED_SLOW / 2);
					turn_flag = SERVO_RIGHT;
					osDelay(500); // wait for the servo to turn
					//move_by_dist(2, value - errors[4]); //-10 // 7 to 10 //TODO: this prob has to increase

					target_angle = turned_angle + value;
					move_flag = 2;

					while (turned_angle < target_angle - angle_errors[2]){ // turning right makes gyro go negative
						osDelay(1);
					}
					move_flag = 0;
					osDelay(100); //wait for inertia
					turn_flag = SERVO_STRAIGHT;

					/*
					target_angle = turned_angle - 90;
					move_flag = 2;

					while (turned_angle > target_angle) {
						// Slow down when approaching target
						if (turned_angle < target_angle + 20 ){
							motor_speed = SPEED_SLOW;
						}
						else {
							osDelay(10);
						}
					}
					move_flag = 0;
					motor_speed = SPEED_NORM;

					turn_flag = SERVO_STRAIGHT;
					*/
				}
				if (strcmp(command, commands[8]) == 0){ //BR
					set_speed(SPEED_SLOW / 2, SPEED_SLOW);
					turn_flag = SERVO_LEFT;
					osDelay(500); // wait for the servo to turn
					//move_by_dist(2, value - errors[5]); //+10 :( // +18

					target_angle = turned_angle - value;
					move_flag = 2;

					while (turned_angle > target_angle + angle_errors[3]){ // turning right makes gyro go negative
						osDelay(1);
					}
					move_flag = 0;
					osDelay(100); //wait for inertia
					turn_flag = SERVO_STRAIGHT;

					/*
					target_angle = turned_angle + 90;
					move_flag = 2;

					while (turned_angle < target_angle) {
						// Slow down when approaching target
						if (turned_angle > target_angle - 20 ){
							motor_speed = SPEED_SLOW;
						}
						else {
							osDelay(10);
						}
					}
					move_flag = 0; //stop
					motor_speed = SPEED_NORM;

					turn_flag = SERVO_STRAIGHT;
					*/
				}
				if (strcmp(command, commands[9]) == 0){ //A4
					// we get 2mins to calibrate the bot, so ill just hijack this thread to do my calibration
					//calibrate();
					//task2();
					send("i am here");
					float u_dist = check_ultrasonic();
					sprintf(buf, "DIST: %d!!!", (int)u_dist);

					send(buf);
					/*
					// THIS IS FL BUT VALUE IS DEGREE TURN NOT DIST
					turn(SERVO_LEFT);
					osDelay(300);
					target_angle = turned_angle + value;
					move_flag = 1;

					while (turned_angle < target_angle) {
						// Slow down when approaching target
						if (turned_angle > target_angle - 30 ){
							speedA = SPEED_SLOW;
							speedB = SPEED_SLOW;
						}
						else {
							osDelay(10);
						}
					}
					move_flag = 0; //stop
					speedA = SPEED_NORM;
					speedB = SPEED_NORM;
					turn(SERVO_STRAIGHT);

					*/
				}
				// reset the shared vars
				//memset(command_buf, 0, MAX_BUF_SIZE);

				// wait 100ms after each command so the bot has time to stop...
				// idk if this is necessary, maybe can try to remove this in the future?
				// might have to change the motorthread, there is a race condition without this delay...

				//buzz(); // turn on buzzer to show sending ok
				//char buf5[20];
				//sprintf(buf5, "OK%d\0", turned_angle);
				//send(buf5);
				send("OK!");
				//osDelay(1000); // change this to 100 later. setting higher for debugging buzzer
				//buzz(); // turn off
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM9_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  IR_Sensors_Init();

  pid_init(&pidMatch, Kp_match, Ki_match, Kd_match);

  // ULTRAONIC
  HAL_TIM_Base_Start(&htim1);

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
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxBuffer, 1);
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

  commandQueue = xQueueCreate(20, CMD_LEN);
  /* --- 强行测试代�?开始 --- */
  // 确�?队列已�?创建好了 (commandQueue = xQueueCreate...)

  /* --- 强行测试代�?结�?� --- */

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
  ultrasonicTaskHandle = osThreadNew(ultrasonic_thread, NULL, &ultrasonicTask_attributes);

  /* Start scheduler */
  osKernelStart();
  /* USER CODE END RTOS_QUEUES */


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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TriggerPin_GPIO_Port, TriggerPin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzz_Pin_GPIO_Port, Buzz_Pin_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : EchoPin_Pin */
  GPIO_InitStruct.Pin = EchoPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EchoPin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TriggerPin_Pin */
  GPIO_InitStruct.Pin = TriggerPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TriggerPin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzz_Pin_Pin */
  GPIO_InitStruct.Pin = Buzz_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzz_Pin_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
    /* --- 1. 定义 PID �?�数 --- */
    const int TARGET_TICK = 160;     // 50ms内期望的脉冲数（根�?�你的电机调整）
    const float Kp_speed = 12.0f;    // 速度环：补�?�摩擦力
    const float Kp_sync = 10.0f;     // �?�步环：修正左�?��?移差

    int16_t CA, CB, prevCA = 0, prevCB = 0;
    prevCA = __HAL_TIM_GET_COUNTER(&htim2);
    prevCB = __HAL_TIM_GET_COUNTER(&htim3);

    for(;;)
    {
        // 2. 计算 50ms 增�?
        CA = __HAL_TIM_GET_COUNTER(&htim2);
        CB = __HAL_TIM_GET_COUNTER(&htim3);

        int16_t Adiff = (int16_t)(CA - prevCA);
        int16_t Bdiff = (int16_t)(CB - prevCB);

        prevCA = CA;
        prevCB = CB;

        // 3. 更新累计�?移
        Adist += (float)Adiff / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;
        // 注�?：如果你的 B 轮编�?器是�??�?�安装，这里用�?�?�
        Bdist -= (float)Bdiff / ENCODER_COUNTS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE_CM;
        average_dist = (Adist + Bdist) / 2.0f;

        // 4. 核心 PID 控制逻辑
        if (move_flag == 1 || move_flag == 2 || turn_in_place_flag == 1)
        {
            // A. 速度补�?�：如果实际速度 < 目标，误差为正，PWM 自动上调
            float current_speed = (float)abs(Adiff);
            //float speed_error = speedA_target - current_speed;
            int speed_error_a = 0;
            int speed_error_b = 0;
            int tmp = 0;
            // 计算基础 PWM：原本设定的 speedA + PID 补�?�
            //int base_pwm_a = speedA_target + (int)(speed_error * Kp_speed);

            current_speed = (float)abs(Bdiff);
            //speed_error = speedB_target - current_speed ;
            //int base_pwm_b = speedB_target + (int)(speed_error * Kp_speed);
            // B. 直线补正：如果 A 跑多了，error 为正，那么 speedA �?�?，speedB 增加

            /*
            float dist_error = Adist - Bdist;
            int sync_correction = (int)(dist_error * Kp_sync);
            sync_correction = 0; // testing for now. since turn in place requires the wheels to spin at different speeds, this will cause problems.
            // 5. 最终输出赋值给全局�?��? speedA/B
            speedA = base_pwm_a - sync_correction;
            speedB = base_pwm_b + sync_correction;
            */

            // CHECK THE EXPECTED ENCODER VALUE FOR THE TARGET SPEED.
            // IF THE ACTUAL ENCODER VALUE IS NOT MATCHING, GRADUALLY INCREASE THE SPEED UNTIL ITS CLOSE ENOUGH.

            if (speedA_target > SPEED_MIN){
            	// only do it if its at least of a certain speed
				if (abs(Adiff) < abs(pwm_to_encoder(speedA_target))){
					speed_error_a = encoder_to_pwm(pwm_to_encoder(speedA_target) - abs(Adiff));
					/*
					if (approaching_flag){
						speed_error_a = encoder_to_pwm(pwm_to_encoder(speedA_target / 2) - abs(Adiff));
						//tmp = speedA_target/SLOW_DOWN_RATIO < SPEED_MIN ? SPEED_MIN : speedA_target/SLOW_DOWN_RATIO;
						//speed_error_a = encoder_to_pwm(pwm_to_encoder(tmp - abs(Adiff)));
					}
					*/
				}
            }

            if (speedB_target > SPEED_MIN){
				// only do it if its at least of a certain speed
				if (abs(Bdiff) < abs(pwm_to_encoder(speedB_target))){
					speed_error_b = encoder_to_pwm(pwm_to_encoder(speedB_target) - abs(Bdiff));
					//tmp = speedB_target/SLOW_DOWN_RATIO < SPEED_MIN ? SPEED_MIN : speedB_target/SLOW_DOWN_RATIO;
					/*
					if (approaching_flag){
						speed_error_b = encoder_to_pwm(pwm_to_encoder(speedB_target/2) - abs(Bdiff));
						//speed_error_b = encoder_to_pwm(pwm_to_encoder(tmp - abs(Bdiff)));
					}
					*/
				}
			}

            //char buf5[20];
            //sprintf(buf5, "%d, %d", (int) encoder_to_pwm(Adiff), (int) encoder_to_pwm(Bdiff));
            //send(buf5);

            //speedA = speedA_target + speed_error_a;
            //speedB = speedB_target + speed_error_b;

            // TRY TO ACCELERATE INTO THE TARGET SPEED INSTEAD OF INSTANTLY
            //speedA = (speedA + speedA_target + speed_error_a) / 2;
            //speedB = (speedB + speedB_target + speed_error_b) / 2;
            const int INCREMENT_STEP = 100; // only increments by 100 at each iteration


            if (speedA < speedA_target + speed_error_a){
            	if ((speedA_target + speed_error_a - speedA) < INCREMENT_STEP){
            		speedA = speedA_target + speed_error_a;
            	} else {
            		if(speedA_target < speedB_target){
            			speedA += INCREMENT_STEP / 2; // account for turning ratio. hardcoded so ill kms if this ratio ever changes
            		} else {
            			speedA += INCREMENT_STEP;
            		}
            	}
            }

            if (speedB < speedB_target + speed_error_b){
				if ((speedB_target + speed_error_b - speedB) < INCREMENT_STEP){
					speedB = speedB_target + speed_error_b;
				} else {
					if(speedB_target < speedA_target){
						speedB += INCREMENT_STEP / 2; // account for turning ratio. hardcoded so ill kms if this ratio ever changes
					} else {
						speedB += INCREMENT_STEP;
					}
				}
			}


            if (approaching_flag){
            	speedA = speedA_target / 2 + speed_error_a;
            	speedB = speedB_target / 2 + speed_error_b;
            	//speedA = (speedA_target/SLOW_DOWN_RATIO < SPEED_MIN ? SPEED_MIN : speedA_target/SLOW_DOWN_RATIO) + speed_error_a;
            	//speedB = (speedB_target/SLOW_DOWN_RATIO < SPEED_MIN ? SPEED_MIN : speedB_target/SLOW_DOWN_RATIO) + speed_error_b;
            }

            char buf[20];
            //sprintf(buf, "S %d, %d, %d", (int)speed_error_a, encoder_to_pwm(abs(Adiff)), (int) speedA_target);
            sprintf(buf, "S %d, %d, %d", CA, Adiff, Bdiff);

            display(buf, 2);

            // 6. 安全�?幅
            if (speedA > 7000) speedA = 7000; //if (speedA < 500) speedA = 500;
            if (speedB > 7000) speedB = 7000; //if (speedB < 500) speedB = 500;

            pwm_outA = speedA;
            pwm_outB = speedB;
        }
        else
        {
        	pwm_outA = 0;
            pwm_outB = 0;
            stop(); // 确�? move_flag 为 0 时彻底�?�下

        }
        // 在 encoder 任务的�?幅代�?�?��?�加上：

        // C. 实时�??馈：在 OLED 第二行显示进度

        // OLED 调试信�?�
        char buf[30];
        sprintf(buf, "V:%d D:%.1f", Adiff, average_dist);
        display((uint8_t*)buf, 3);

        osDelay(50); // 采样周期调至 50ms
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
	  char errBuf[30];

	  if (HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &z_reg_addr, 1, 1000) != HAL_OK)
	  {
	      uint32_t error = HAL_I2C_GetError(&hi2c2);

	      sprintf(errBuf, "I2C ERR: 0x%lX", error);
	      OLED_ShowString(10, 40, errBuf);

	      buzz();
	      osDelay(1000);
	      buzz();
	      osDelay(1000); // since oled is turned off for now, buzz when the gyro fails
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
	turned_angle = (int) theta;

	//sprintf(buf,"%d", turned_angle);
	//display(buf, 1);

	//sprintf(buf2, "%d", gyro_z_raw);
	//display(buf2, 1);


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
