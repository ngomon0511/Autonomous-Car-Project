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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdbool.h>
#include <math.h>

int cntMilSecond; // control runing time

// To read ultrasonic sensor
#define TRIG_PIN GPIO_Output_trig_Pin
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_Input_echo_Pin
#define ECHO_PORT GPIOA
uint32_t uMillis;
uint32_t val1 = 0;
uint32_t val2 = 0;
uint16_t disObs = 0;
uint16_t safeDis = 10;

// Transmit - Receive data with Jeson Nano
#define SIZE_TX_BUFFER 12
uint8_t uartTxBuffer[SIZE_TX_BUFFER];
uint8_t convertedSpeed1[2];
uint8_t convertedSpeed2[2];
uint8_t convertedDisObs[2];
#define SIZE_RX_BUFFER 8
uint8_t uartRxBuffer[SIZE_RX_BUFFER];

// To control motor velocity
int16_t sig1, sig2; // signal control two motors
int setSpeed1, setSpeed2;
char runMotor1, runMotor2, directMotor1[3], directMotor2[3];

// To design PI controller
float Kp, Ki, Kb;
const float Ts = 0.01;
const int16_t maxPWM = 100;
const int16_t minPWM = 0;

// To read encoder
int cntPulse1, cntPulse2;
float rpm1, rpm2, encodeSpeed1, encodeSpeed2; // round per minute and radian per second

bool compareString(const char* str1, const char* str2) {
    while (*str1 != '\0' || *str2 != '\0') {
        if (*str1 != *str2) {
            return false;
        }
        str1++;
        str2++;
    }
    return true;
}

void numberToChar(float num, uint8_t result[2]) // num to two chars
{
    int firstChar = (int)num / 10;
    int secondChar = (int)num % 10;
    result[0] = firstChar + '0';
    result[1] = secondChar + '0';
}

int16_t PI_Controller(const char* motor, const char* direction, float targetSpeed, float encoderSpeed)
{
	if (compareString(motor, "motor1"))
	{
	    if (compareString(direction, "cw"))
	    {
	        Kp = 5.364516129;
	        Ki = 32.25806452;
	        Kb = 6.013229104;
	    }
	    else if (compareString(direction, "ccw"))
	    {
	        Kp = 4.75;
	        Ki = 31.25;
	        Kb = 6.578947368;
	    }
	}
	else if (compareString(motor, "motor2"))
	{
	    if (compareString(direction, "ccw"))
	    {
	        Kp = 4.240625;
	        Ki = 31.25;
	        Kb = 7.369196758;
	    }
	    else if (compareString(direction, "cw"))
	    {
	        Kp = 4.153333333;
	        Ki = 33.33333333;
	        Kb = 8.025682183;
	    }
	}

	int16_t u_out;
	float error, u_p, u_i, u_design;
	static float errorSignal, u_i_pre;

	error = targetSpeed - encoderSpeed;

	// For P
	u_p = Kp*error;

	// For I
	u_i = u_i_pre + Ki*Ts*error + Kb*Ts*errorSignal;
	u_i_pre = u_i;

	// PI Controller
	u_design = u_p + u_i;

	// Check error of signal
	if (u_design > maxPWM) {
		u_out = maxPWM;
		errorSignal = u_out - u_design;
	}
	else if (u_design < minPWM)
	{
		u_out = minPWM;
		errorSignal = u_out - u_design;
	}
	else
	{
		u_out = (int16_t)u_design;
		errorSignal = 0;
	}

	return u_out;
}

void readInfraredSensor()
{
    uartTxBuffer[0] = (HAL_GPIO_ReadPin(GPIOB, GPIO_Input_IS1_Pin) == GPIO_PIN_SET) ? '1' : '0';
    uartTxBuffer[1] = (HAL_GPIO_ReadPin(GPIOB, GPIO_Input_IS2_Pin) == GPIO_PIN_SET) ? '1' : '0';
    uartTxBuffer[2] = (HAL_GPIO_ReadPin(GPIOB, GPIO_Input_IS3_Pin) == GPIO_PIN_SET) ? '1' : '0';
    uartTxBuffer[3] = (HAL_GPIO_ReadPin(GPIOA, GPIO_Input_IS4_Pin) == GPIO_PIN_SET) ? '1' : '0';
    uartTxBuffer[4] = (HAL_GPIO_ReadPin(GPIOA, GPIO_Input_IS5_Pin) == GPIO_PIN_SET) ? '1' : '0';
}

void readUltrasonicSensor()
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim7, 0);
	while (__HAL_TIM_GET_COUNTER (&htim7) < 10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

	uMillis = HAL_GetTick(); // calib sensor
	while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && uMillis + 10 >  HAL_GetTick());
	val1 = __HAL_TIM_GET_COUNTER (&htim7);

	uMillis = HAL_GetTick(); // calib sensor
	while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && uMillis + 50 > HAL_GetTick());
	val2 = __HAL_TIM_GET_COUNTER (&htim7);

	disObs = (val2-val1)* 0.034/2; //cm
}

void transmitData()
{
	char sendData[SIZE_TX_BUFFER];
	memcpy(sendData, uartTxBuffer, SIZE_TX_BUFFER);
	sendData[SIZE_TX_BUFFER - 1] = '.';
	HAL_UART_Transmit(&huart2, (uint8_t*)sendData, SIZE_TX_BUFFER, HAL_MAX_DELAY);
}

void processDataReceived()
{
	runMotor1 = uartRxBuffer[0];
	strcpy(directMotor1, (uartRxBuffer[1] == '0')  ? "cw" : "ccw");
	setSpeed1 = (uartRxBuffer[2] - '0')*10 + (uartRxBuffer[3] - '0');
	runMotor2 = uartRxBuffer[4];
	strcpy(directMotor2, (uartRxBuffer[5] == '0') ? "cw" : "ccw");
	setSpeed2 = (uartRxBuffer[6] - '0')*10 + (uartRxBuffer[7] - '0');
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		HAL_UART_Receive_DMA(&huart2, uartRxBuffer, SIZE_RX_BUFFER);
		processDataReceived();

		if (runMotor1 == '0')
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN1_Pin, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN2_Pin, 0);
		}

		if (runMotor2 == '0')
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN3_Pin, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN4_Pin, 0);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_6)
    {
    	cntPulse1++;
    }

    if (GPIO_Pin == GPIO_PIN_4)
    {
    	cntPulse2++;
    }
}

int16_t controlVelocityM1(const char* direction, float targetSpeed, float encoderSpeed)
{
	int16_t controlPulse = PI_Controller("motor1", direction, targetSpeed, encoderSpeed);

	if (compareString(direction, "cw"))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN1_Pin, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN2_Pin, 0);
	}

	else if (compareString(direction, "ccw"))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN2_Pin, 1);
	}

	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, controlPulse);

	return controlPulse;
}

int16_t controlVelocityM2(const char* direction, float targetSpeed, float encoderSpeed)
{
	int16_t controlPulse = PI_Controller("motor2", direction, targetSpeed, encoderSpeed);

	if (compareString(direction, "cw"))
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN3_Pin, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN4_Pin, 0);
	}

	else if (compareString(direction, "ccw"))
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN3_Pin, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN4_Pin, 1);
	}

	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, controlPulse);

	return controlPulse;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == htim6.Instance) // 1 millisecond
	{
		cntMilSecond++;

		if (cntMilSecond == 50) // 50 milliseconds
		{
			readInfraredSensor();

			numberToChar(disObs > 99 ? 99 : disObs, convertedDisObs);
			uartTxBuffer[5] = convertedDisObs[0];
			uartTxBuffer[6] = convertedDisObs[1];

			// Record motor speed to transmit
			numberToChar(encodeSpeed1, convertedSpeed1);
			uartTxBuffer[7] = convertedSpeed1[0];
			uartTxBuffer[8] = convertedSpeed1[1];

			numberToChar(encodeSpeed2, convertedSpeed2);
			uartTxBuffer[9] = convertedSpeed2[0];
			uartTxBuffer[10] = convertedSpeed2[1];

			transmitData();

			cntMilSecond = 0;
		}

		if (cntMilSecond % 10 == 0) // 10 milliseconds
		{
			rpm1 = ((float)cntPulse1/330)*100*60;
			encodeSpeed1 = rpm1 / 60 * (2*M_PI);
			cntPulse1 = 0;

			if (runMotor1 == '1')
			{
				sig1 = controlVelocityM1(directMotor1, setSpeed1, encodeSpeed1);
			}
			else if (runMotor1 == '0')
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN1_Pin, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN2_Pin, 0);
			}

			rpm2 = ((float)cntPulse2/330)*100*60;
			encodeSpeed2 = rpm2 / 60 * (2*M_PI);
			cntPulse2 = 0;

			if (runMotor2 == '1')
			{
				sig2 = controlVelocityM2(directMotor2, setSpeed2, encodeSpeed2);
			}
			else if (runMotor2 == '0')
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN3_Pin, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN4_Pin, 0);
			}
		}
	}
}

void runM1(bool run, const char* direction, int16_t speedPercent) /* No PI controller */
{
    if (run)
    {
    	if (compareString(direction, "cw"))
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN1_Pin, 1);
            HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN2_Pin, 0);
        }

    	else if (compareString(direction, "ccw"))
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN1_Pin, 0);
            HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN2_Pin, 1);
        }

        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, speedPercent);
    }

    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN1_Pin, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN2_Pin, 0);
    }
}

void runM2(bool run, const char* direction, int16_t speedPercent) /* No PI controller */
{
    if (run)
    {
    	if (compareString(direction, "cw"))
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN3_Pin, 1);
            HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN4_Pin, 0);
        }

    	else if (compareString(direction, "ccw"))
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN3_Pin, 0);
            HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN4_Pin, 1);
        }

        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, speedPercent);
    }

    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN3_Pin, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN4_Pin, 0);
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
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_UART_Receive_DMA(&huart2, uartRxBuffer, SIZE_RX_BUFFER);
  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	readUltrasonicSensor();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 3;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_Output_IN4_Pin|GPIO_Output_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_Output_IN1_Pin|GPIO_Output_IN2_Pin|GPIO_Output_trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_Output_IN4_Pin GPIO_Output_IN3_Pin */
  GPIO_InitStruct.Pin = GPIO_Output_IN4_Pin|GPIO_Output_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Output_IN1_Pin GPIO_Output_IN2_Pin GPIO_Output_trig_Pin */
  GPIO_InitStruct.Pin = GPIO_Output_IN1_Pin|GPIO_Output_IN2_Pin|GPIO_Output_trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Input_echo_Pin GPIO_Input_IS5_Pin GPIO_Input_IS4_Pin */
  GPIO_InitStruct.Pin = GPIO_Input_echo_Pin|GPIO_Input_IS5_Pin|GPIO_Input_IS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Input_IS1_Pin GPIO_Input_IS2_Pin GPIO_Input_IS3_Pin */
  GPIO_InitStruct.Pin = GPIO_Input_IS1_Pin|GPIO_Input_IS2_Pin|GPIO_Input_IS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
