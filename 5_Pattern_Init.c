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
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define dataPin			GPIOA, GPIO_PIN_9
#define clockPin(x)		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, x? GPIO_PIN_SET:GPIO_PIN_RESET)
#define latchPin(x)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, x? GPIO_PIN_SET:GPIO_PIN_RESET)

#define buzzer(x)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, x? GPIO_PIN_SET:GPIO_PIN_RESET)

#define enA				HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)
#define enB				HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)
#define enS				HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t t0,t1;
uint8_t encA, encB;
int8_t i=0, j=0;
uint8_t mode=0, cnt=0;
int16_t pattern[10][5] = {	{ 0xEF06, 0xF709, 0xFB09, 0xFD09, 0xFE06 },		//-- num 0
							{ 0xEF04, 0xF706, 0xFB04, 0xFD04, 0xFE0E },		//-- num 1
							{ 0xEF06, 0xF709, 0xFB04, 0xFD02, 0xFE0F },		//-- num 2
							{ 0xEF06, 0xF709, 0xFB04, 0xFD09, 0xFE06 },		//-- num 3
							{ 0xEF04, 0xF706, 0xFB05, 0xFD0F, 0xFE04 },		//-- num 4
							{ 0xEF0F, 0xF701, 0xFB07, 0xFD08, 0xFE07 },		//-- num 5
							{ 0xEF0E, 0xF701, 0xFB07, 0xFD09, 0xFE06 },		//-- num 6
							{ 0xEF0F, 0xF708, 0xFB04, 0xFD02, 0xFE01 },		//-- num 7
							{ 0xEF06, 0xF709, 0xFB06, 0xFD09, 0xFE06 },		//-- num 8
							{ 0xEF06, 0xF709, 0xFB0E, 0xFD08, 0xFE07 }};	//-- num 9

int16_t init[9][5] = 	{	{ 0xEF01, 0xF701, 0xFB01, 0xFD01, 0xFE0F },		//-- seq 0
							{ 0xEF02, 0xF702, 0xFB02, 0xFD0F, 0xFE02 },		//-- seq 1
							{ 0xEF04, 0xF704, 0xFB0F, 0xFD04, 0xFE04 },		//-- seq 2
							{ 0xEF08, 0xF70F, 0xFB08, 0xFD08, 0xFE08 },		//-- seq 3
							{ 0xEF0F, 0xF708, 0xFB08, 0xFD08, 0xFE08 },		//-- seq 4
							{ 0xEF08, 0xF70F, 0xFB08, 0xFD08, 0xFE08 },		//-- seq 5
							{ 0xEF04, 0xF704, 0xFB0F, 0xFD04, 0xFE04 },		//-- seq 6
							{ 0xEF02, 0xF702, 0xFB02, 0xFD0F, 0xFE02 },		//-- seq 7
							{ 0xEF00, 0xF700, 0xFB00, 0xFD00, 0xFE00 }};	//-- seq 8: reset

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void pushDataToShiftRegister(uint16_t val)
{	for(int16_t z=15;z>-1;z--)
	{	HAL_GPIO_WritePin(dataPin, (val&(1<<z)) ? GPIO_PIN_SET:GPIO_PIN_RESET);
		clockPin(1);
		clockPin(0);
	}
}
void beep(uint8_t kali, uint16_t masa)
{	for(uint8_t l=0;l<kali;l++)
	{	buzzer(1);	HAL_Delay(masa);
		buzzer(0);	HAL_Delay(masa);
	}
}
int8_t getEncoder (void)
{	static int8_t x0,x1;
	int8_t y=0;
	x0=(enA<<1)|(enB<<0);

	if(x1 != x0)
	{	switch(x1)
		{	case 0:
				if(x0==2)	y=-1;
				if(x0==1)	y=1;
				break;
			case 1:
				if(x0==0)	y=-1;
				if(x0==3)	y=1;
				break;
			case 2:
				if(x0==3)	y=-1;
				if(x0==0)	y=1;
				break;
			case 3:
				if(x0==1)	y=-1;
				if(x0==2)	y=1;
				break;
		}
		x1=x0;
	}
	return y;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	if(htim->Instance==TIM6)
	{	for(uint8_t j=0;j<5;j++)
		{	latchPin(0);
			if(mode==0) pushDataToShiftRegister(init[i][j]);
			if(mode==1) pushDataToShiftRegister(pattern[i][j]);
			latchPin(1);
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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  t0 = t1 = HAL_GetTick();
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	  if(mode==0)
	  {	  if(t0 < HAL_GetTick())
		  {	  t0 += 100;

		  	  i++;
		  	  if(i>7)
		  	  {	  i=0;
		  		  cnt++;
		  	  }
		  	  if(cnt==3)
		  	  {	  i=8;
		  	  	  HAL_Delay(50);
		  		  mode=1;
		  		  i=0;
		  		  t0=t1=HAL_GetTick();
		  	  }
		  }
	  }
  	  if(mode==1)
  	  {	  if(t1 < HAL_GetTick())
  		  {
  			  t1 += 500;
  			  if(++i>9) i=0;
  		  }
  	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim6.Init.Prescaler = 2400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10-1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|SDATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|RCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin SDATA_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SDATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin RCLK_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|RCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SRCLK_Pin */
  GPIO_InitStruct.Pin = SRCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SRCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_A_Pin */
  GPIO_InitStruct.Pin = EN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_B_Pin EN_PB_Pin */
  GPIO_InitStruct.Pin = EN_B_Pin|EN_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
