/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
//#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef union{
	uint8_t buff[4];
	float valueOdo;
} packet_buffer_t;

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral;
	float prev_error;
	float derivative;
	float output;
} PID_Controller_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 25
#define MAX_PWM 1000
#define MAX_RPM 78.47// Adjust according to PWM resolution and motor driver requirements
#define WHEEL_DIAMETER 0.0455 // Diameter of the wheels in cm
#define WHEEL_BASE 0.1605 // Distance between the wheels in cm
#define TICKS_PER_REV 5880 // Number of encoder ticks per wheel revolution
#define PI 3.14159265358979323846 // PI
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

uint8_t RxBuffer[BUFFER_SIZE];
uint8_t TxBuffer[BUFFER_SIZE];
uint8_t TxBuffer2[BUFFER_SIZE];

int32_t Right_encoder_value;
int32_t Left_encoder_value;

packet_buffer_t X_value;
packet_buffer_t Y_value;
packet_buffer_t Theta_value;
packet_buffer_t velocity_packet;
packet_buffer_t X_packet, Y_packet, Theta_packet, send_velocity_packet;

PID_Controller_t pid_left = {0.5, 0.1, 1, 0, 0, 0, 0};  // Initialize these values appropriately
PID_Controller_t pid_right = {0.5, 0.1, 1, 0, 0, 0, 0};

float Desired_Velocity_Left; // Received from ESP32
float Desired_Velocity_Right;
float Desired_RPM_Left;
float Desired_RPM_Right;

float rpm_left;
float rpm_right;


int k;// Received from ESP32

//Odometry Variables
float x = 0.0; // x position in cm
float y = 0.0; // y position in cm
float W = 0.0; // W Angular Velocity
float theta = 0.0; // orientation angle in radians
float RPMLeft = 0.0;
float RPMRight = 0.0;
float v = 0;

float Desired_X,Desired_Y, Desired_Theta;

// Timer variables
float prev_time = 0;
float delta_t = 0;
float current_time = 0;

float distancePerCount = (WHEEL_DIAMETER * PI) / TICKS_PER_REV;

float control_left = 0;
float control_right = 0;

float rpm_left_error ; //desired RPM
float rpm_right_error;

// Variables to store previous encoder counts
int32_t Left_count = 0;
int32_t Right_count = 0;
int32_t Prevleft_count = 0;
int32_t Prevright_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void motor_orientation(float, float);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
void PID_Update(PID_Controller_t* pid, float error, float dt);
float constrain(float value, float min, float max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	TIM1->CNT = 500; //esquerda
	TIM2->CNT = 500;
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_SPI_TransmitReceive_DMA(&hspi1, TxBuffer, RxBuffer, BUFFER_SIZE);

	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
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
	htim4.Init.Prescaler = 60-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000-1;
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
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
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
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 60-1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 1000-1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, STBY_Pin|BIN2_Pin|BIN1_Pin|AIN2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : STBY_Pin BIN2_Pin BIN1_Pin AIN2_Pin */
	GPIO_InitStruct.Pin = STBY_Pin|BIN2_Pin|BIN1_Pin|AIN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : AIN1_Pin */
	GPIO_InitStruct.Pin = AIN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(AIN1_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	// Sending data to ESP32
	TxBuffer[5] = X_value.buff[0];
	TxBuffer[6] = X_value.buff[1];
	TxBuffer[7] = X_value.buff[2];
	TxBuffer[8] = X_value.buff[3];

	TxBuffer[9] = Y_value.buff[0];
	TxBuffer[10] = Y_value.buff[1];
	TxBuffer[11] = Y_value.buff[2];
	TxBuffer[12] = Y_value.buff[3];

	TxBuffer[13] = Theta_value.buff[0];
	TxBuffer[14] = Theta_value.buff[1];
	TxBuffer[15] = Theta_value.buff[2];
	TxBuffer[16] = Theta_value.buff[3];

	TxBuffer[17] = send_velocity_packet.buff[0];
	TxBuffer[18] = send_velocity_packet.buff[1];
	TxBuffer[19] = send_velocity_packet.buff[2];
	TxBuffer[20] = send_velocity_packet.buff[3];



	// V_max = 18.275 cm/s

	//---Receiving Data From ESP32---/
	velocity_packet.buff[0] = RxBuffer[5];
	velocity_packet.buff[1] = RxBuffer[6];
	velocity_packet.buff[2] = RxBuffer[7];
	velocity_packet.buff[3] = RxBuffer[8];

	Desired_Velocity_Left = velocity_packet.valueOdo;


	//---Receiving Data From ESP32---/
	velocity_packet.buff[0] = RxBuffer[9];
	velocity_packet.buff[1] = RxBuffer[10];
	velocity_packet.buff[2] = RxBuffer[11];
	velocity_packet.buff[3] = RxBuffer[12];

	Desired_Velocity_Right = velocity_packet.valueOdo;


	X_packet.buff[0] = RxBuffer[13];
	X_packet.buff[1] = RxBuffer[14];
	X_packet.buff[2] = RxBuffer[15];
	X_packet.buff[3] = RxBuffer[16];

	Desired_X = X_packet.valueOdo;


	Y_packet.buff[0] = RxBuffer[17];
	Y_packet.buff[1] = RxBuffer[18];
	Y_packet.buff[2] = RxBuffer[19];
	Y_packet.buff[3] = RxBuffer[20];

	Desired_Y = Y_packet.valueOdo;

	Theta_packet.buff[0] = RxBuffer[21];
	Theta_packet.buff[1] = RxBuffer[22];
	Theta_packet.buff[2] = RxBuffer[23];
	Theta_packet.buff[3] = RxBuffer[24];

	Desired_Theta = Theta_packet.valueOdo;

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim5)
{
	//If one Wheel is inverted equal it with a minus
	Left_encoder_value = (int32_t)(TIM1->CNT-500);
	Right_encoder_value = (int32_t)(TIM2->CNT-500);
	TIM1->CNT = 500; //esquerda
	TIM2->CNT = 500;

	Left_count = -Left_encoder_value;
	Right_count = Right_encoder_value;

	Prevleft_count = Left_encoder_value;
	Prevright_count = Right_encoder_value;

	delta_t = 0.001; // time interval
	current_time = HAL_GetTick(); // Elapsed Time

	float dleft = distancePerCount * Left_count;
	float dright = distancePerCount * Right_count;
	float distance = (dleft + dright) / 2.0;
	float d_theta = (dright - dleft) / WHEEL_BASE;

	float vleft = dleft / delta_t;
	float vright = dright / delta_t;

	v = (vleft + vright) / 2.0;

	send_velocity_packet.valueOdo = v;

	// Convert angular velocities to RPM
	rpm_left = (vleft * 60) / (WHEEL_DIAMETER * PI);
	//float pwm_left = 1000/78.47 * rpm_left;
	rpm_right = (vright * 60) / (WHEEL_DIAMETER * PI);
	//float pwm_right = 1000/78.47 * rpm_right;

	// Update robot position
	y += distance * sin(theta+d_theta/2);
	x += distance * cos(theta+d_theta/2);

	theta += d_theta;

	if(theta >= (2*PI)) {theta = theta - 2*PI;}
	if(theta <= (-2*PI)) {theta = theta + 2*PI;}

	// Stores Values to Send them over SPI
	Theta_value.valueOdo = theta;
	X_value.valueOdo = (float) x;
	Y_value.valueOdo = (float) y;

	Desired_RPM_Left = (Desired_Velocity_Left * 60)/ (WHEEL_DIAMETER * PI);
	Desired_RPM_Right = (Desired_Velocity_Right * 60)/ (WHEEL_DIAMETER * PI);;

	rpm_left_error = Desired_RPM_Left-rpm_left; //desired RPM
	rpm_right_error = Desired_RPM_Right-rpm_right;

	// PID Update
	PID_Update(&pid_left, rpm_left_error, delta_t);
	PID_Update(&pid_right, rpm_right_error, delta_t);

	// Calculate PWM from PID output
	int32_t pwmL = (int32_t)(pid_left.output) * (MAX_PWM/MAX_RPM);
	int32_t pwmR = (int32_t)(pid_right.output) * (MAX_PWM/MAX_RPM);

	pwmL = (int32_t)constrain(pwmL, -MAX_PWM, MAX_PWM);
	pwmR = (int32_t)constrain(pwmR, -MAX_PWM, MAX_PWM);

	if(pwmL<0){
		pwmL=-pwmL;
	}
	if(pwmR<0){
		pwmR=-pwmR;
	}

	motor_orientation(rpm_left_error,rpm_right_error);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwmL);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwmR);
}


void motor_orientation(float L, float R){
	// Declaração de variáveis para estados dos pinos
	GPIO_PinState AIN1_State, AIN2_State, BIN1_State, BIN2_State;

	if (L > 0.1 && R > 0.1) {
		// Ambos os motores para frente
		AIN1_State = GPIO_PIN_SET;
		AIN2_State = GPIO_PIN_RESET;
		BIN1_State = GPIO_PIN_RESET;
		BIN2_State = GPIO_PIN_SET;
	}
	else if (L > 0.1 && R < -0.1) {
		// Esquerda para frente, direita para trás
		AIN1_State = GPIO_PIN_SET;
		AIN2_State = GPIO_PIN_RESET;
		BIN1_State = GPIO_PIN_SET;
		BIN2_State = GPIO_PIN_RESET;
	}
	else if (L < -0.1 && R > 0.1) {
		// Esquerda para trás, direita para frente
		AIN1_State = GPIO_PIN_RESET;
		AIN2_State = GPIO_PIN_SET;
		BIN1_State = GPIO_PIN_RESET;
		BIN2_State = GPIO_PIN_SET;
	}
	else if (L < -0.1 && R < -0.1) {
		// Ambos os motores para trás
		AIN1_State = GPIO_PIN_RESET;
		AIN2_State = GPIO_PIN_SET;
		BIN1_State = GPIO_PIN_SET;
		BIN2_State = GPIO_PIN_RESET;
	} else {
		// Caso nenhuma das condições seja atendida, desligue os motores
		AIN1_State = GPIO_PIN_RESET;
		AIN2_State = GPIO_PIN_RESET;
		BIN1_State = GPIO_PIN_RESET;
		BIN2_State = GPIO_PIN_RESET;
	}

	// Atualização dos estados dos pinos
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, AIN1_State);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, AIN2_State);
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, BIN1_State);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, BIN2_State);
}

void PID_Update(PID_Controller_t* pid, float error, float dt) {
	pid->integral += error * dt;
	pid->derivative = (error - pid->prev_error) / dt;
	pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * pid->derivative;
	pid->prev_error = error;
}

float constrain(float value, float min, float max) {
	if (value < min) {
		return min;
	} else if (value > max) {
		return max;
	} else {
		return value;
	}
}
/* USER CODE END 4 */

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
