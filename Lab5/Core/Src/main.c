/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"

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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t button = 0;
uint8_t CheckButton = 1;
uint8_t Hz = 0;
uint8_t State = 0;
uint8_t RxBuffer[10];
uint8_t TxBuffer[40];
uint8_t Button[1];
uint8_t Menu[] =
		"Press the number of choise \r\n"
		"      0 LED Control        \r\n"
		"      1 Button Status      \r\n";
uint8_t Menu1[] =
		"        LED Control        \r\n"
		"     a : Speed Up +1 Hz    \r\n"
		"     s : Speed Down -1 Hz  \r\n"
		"     d : On / Off LED      \r\n"
		"     x : Prevoius menus    \r\n";
uint8_t Menu2[] =
		"        Button Status      \r\n"
		"     show button status B1 \r\n"
		"     x : Prevoius menus    \r\n";
uint8_t ledon[]   = "LedOn\r\n";
uint8_t ledoff[]  = "LedOff\r\n";
uint8_t Press[]   = "Button Pressed\r\n";
uint8_t unPress[] = "Button Unpressed\r\n";
uint16_t CheckButtonB1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//void UARTPollingMethod();
//void DummyTrak();
//void UARTInterruptConfig();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  uint8_t text[] = "HELLO FIBO";
//  HAL_UART_Transmit(&huart2, text , 11 , 10);
//  UARTInterrupConfig();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
  	  UARTInterrupConfig();
  	  static uint32_t timestamp = 0;
  	  if (HAL_GetTick() > timestamp)
  	  {
  		  timestamp = HAL_GetTick()+100;
//	  		  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
  	  }
  	  switch(State)
  	  {
  	  case 0:
  		  HAL_UART_Transmit_IT(&huart2, Menu , strlen((char*)Menu) );
  		  State = 1;
  		  break;
  	  case 1:
  		  if(RxBuffer[0] == 48)
  		  {
  			  State = 2;

  		  }
  		  else if(RxBuffer[0] == 49)
  		  {
  			  State = 4;

  		  }
  		  break;
  	  case 2:
  		  HAL_UART_Transmit_IT(&huart2, Menu1 , strlen((char*)Menu1));
  		  State = 3;
  		  break;
  	  case 3:
  		  if(RxBuffer[0] == 97)
  		  {

  			  Hz = Hz + 1;
  			  State = 3;
  		  }
  		  else if(RxBuffer[0] == 115)
  		  {

  			  Hz = Hz - 1;
  			  State = 3;
  		  }
  		  else if(RxBuffer[0] == 100)
  		  {

  			  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1 && CheckButton == 0 )
  			  {
  				  CheckButton = 1;
  				  HAL_UART_Transmit(&huart2, ledon , strlen((char*)ledon),50 );
  				  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
  			  }
  			  else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0 && CheckButton == 0)
  			  {
  				  CheckButton = 1;
  				  HAL_UART_Transmit(&huart2, ledoff , strlen((char*)ledoff),50 );
  				  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
  			  }
  			  State = 3;
  		  }
  		  else if(RxBuffer[0] == 120)
  		  {

  			  State = 0;
  		  }
  		  else
  		  {
  			  State = 3;
  		  }
  		  break;
  	  case 4:
  		  HAL_UART_Transmit_IT(&huart2, Menu2 , strlen((char*)Menu2) );
  		  State = 5;
  		  break;
  	  case 5:
  		  button = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  		  if(button == 1 && CheckButton == 0)
  		  {
  			  CheckButton = 1;
  			  HAL_UART_Transmit(&huart2, Press , strlen((char*)Press),50 );
  			  State = 4;
  		  }
  		  else if(button == 0 && CheckButton == 0)
  		  {
  			  CheckButton = 1;
  			  HAL_UART_Transmit(&huart2, unPress , strlen((char*)unPress),50 );
  			  State = 4;
  		  }
  		  else if(RxBuffer[0] == 120)
  		  {

  			  State = 0;
  		  }
  		  break;


  	  }

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void UARTPollingMethod()
//{
//	//read uart 10 char with in 10s
//	HAL_StatusTypeDef HAL_status = HAL_UART_Receive(&huart2,RxBuffer,10,1000);
//
//	//if complete read 10 char
//	if(HAL_status == HAL_OK)
//	{
//		RxBuffer[10] = '\0';
//
//		sprintf((char*)TxBuffer,"Received : %s\r\n",RxBuffer);
//		HAL_UART_Transmit(&huart2,TxBuffer,strlen((char*)TxBuffer),10);
//	}
//	else if(HAL_status == HAL_TIMEOUT)
//	{
//		uint32_t lastCharPos = huart2.RxXferSize - huart2.RxXferCount;
//		RxBuffer[lastCharPos] = '\0';
//
//		sprintf((char*)TxBuffer,"Received Timeout : %s\r\n",RxBuffer);
//		HAL_UART_Transmit(&huart2,TxBuffer,strlen((char*)TxBuffer),10);
//	}
//}

//void DummyTask()
//{
//	static uint32_t timestamp = 0;
//	if(HAL_GetTick()>=timestamp)
//	{
//		timestamp = HAL_GetTick()+100;
//		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
//	}
//}

void UARTInterrupConfig()
{
	HAL_UART_Receive_IT(&huart2,RxBuffer,1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
{
	if(huart == &huart2)
	{
		RxBuffer[1] = '\0';
		CheckButton = 0;
		sprintf((char*)TxBuffer,"Received : %s\r\n",RxBuffer);
		HAL_UART_Transmit(&huart2,TxBuffer,strlen((char*)TxBuffer),50);

		HAL_UART_Receive_IT(&huart2,RxBuffer,1);
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
