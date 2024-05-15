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
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[20];
uint8_t TxBuffer[60];
uint8_t output[6];
uint8_t wordle[6] = "PPDEK";
uint8_t point = 0;
uint8_t attempt = 0;
uint8_t readFlag = 0;
uint8_t s = 2;

// SPI //

uint8_t SPIRx[10];
uint8_t SPITx[10];
uint8_t Mode;
uint8_t Switch = 1;
uint8_t LMode1 = 1;
uint8_t n = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void UARTPollingMethod();
void DummyTask();
void SPITxRx_readIO();
void IODIRB_Setup();
void ReadSwitch();

//void UARTInterruptConfig();
//void UARTDMAConfig();
void Wordle();
void LightSetup();
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
  MX_LPUART1_UART_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // UART CODE //
  //UARTDMAConfig();
  //HAL_UART_Transmit(ส่งด้วย , สิ่งที่ส่ง ,
  sprintf((char*)TxBuffer,"Welcome to WORDLE, You have 5 attempts to guess the word\r\n\n");
  HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer), 5);
  sprintf((char*)TxBuffer,"PRESS - to clear your word\r\n");
  HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer), 5);
  sprintf((char*)TxBuffer,"SEND 11111 to re-attempt \n\n");
  HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer), 5);
  sprintf((char*)TxBuffer,"Turn CAPLOCK on and begin typing \r\n");
  HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer), 5);
  // UART END //
  // SPI CODE //
  LightSetup();
  IODIRB_Setup();
  HAL_TIM_Base_Start_IT(&htim2);
  // SPI END //



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  UARTPollingMethod();
	  //UARTInterruptConfig();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(readFlag && attempt < 5 && point != 5 && RxBuffer[0] != 1 && RxBuffer[4] != 1){
		  readFlag = 0;
		  Wordle();
	  }

	  HAL_Delay(1);
	  SPITxRx_readIO();
	  ReadSwitch();


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*/ SPI Function begin /*/
void IODIRB_Setup()//at BEGIN 2
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
	SPITx[0] = 0b01000000;//write
	SPITx[1] = 0x01;//IODIRB
	SPITx[2] = 0b00000000;
	HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 3);
}

void SPITxRx_readIO()
{
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2))
	{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0); // CS Select
	SPITx[0] = 0b01000001;
	SPITx[1] = 0x12;
	SPITx[2] = 0;
	SPITx[3] = 0;
	HAL_SPI_TransmitReceive_IT(&hspi3, SPITx, SPIRx, 4);
	}
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1); //CS dnSelect
}

/*/ SPI Function end /*/

void Wordle()
	{
		//readFlag = 0;
	for(uint8_t i = 0;i <5;i++)
	{
		output[5] = '\0';

		if(RxBuffer[i] == '-') //retry press "-"
		{
			sprintf((char*)TxBuffer,"Retrying");
			HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer) , 5); // uart1, text , size , timeout
			return;
		}
		else if(RxBuffer[i] == wordle[i]){
			output[i] = RxBuffer[i];
			point++;

		}
		else {
			for(uint8_t j=0; j<5;j++)
			{
				if(RxBuffer[i] == wordle[j])
				{
					output[i] = '?';
					break;
				}
			}
		}
		if(output[i] != '?' && output[i] != RxBuffer[i]){
			output[i] = '_';
		}
	}
	if(point == 5){
		sprintf((char*)TxBuffer,"\n Congratulations\n %s is correct\r\n", output);
		HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer) , 5);
	}
	else{
		point = 0;
		attempt++;
		if(attempt >= 5){
				sprintf((char*)TxBuffer,"\n Game Over\r\n\n Correct Answer is %s\n\r\n", (char*)wordle);
				HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer) , 5);
		}
		else if(RxBuffer[1] != '1'){
			sprintf((char*)TxBuffer,"Wrong Answer \n Your word : %s\r\n", (char*)output);
			HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer) , 5);
			sprintf((char*)TxBuffer,"Remaining Chances: %d/5 \r\n", (int)(5-attempt));
			HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer) , 5);
		}

	}

}

void UARTPollingMethod()
	{
	//read UART 10 char with in 10s
	HAL_StatusTypeDef HAL_status = HAL_UART_Receive(&hlpuart1, RxBuffer, 5, 15000);

	//if complete read 10 char
	if(HAL_status == HAL_OK)
	{
		readFlag = 1;
		RxBuffer[5] = '\0';
		if(RxBuffer[0] == '1' || RxBuffer[4] == '1'){
			point = 0;
			attempt = 0;
			sprintf((char*)TxBuffer,"Reset Completed");
		}
		else if(attempt >= 5 || point == 5){
			sprintf((char*)TxBuffer,"Game Halted, SEND 11111 to re-attempt\r\n");
		}
		else{
			sprintf((char*)TxBuffer,"Your answer : %s\r\n",(char*)RxBuffer);
		}
		HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer), 5);
	}
	else if(HAL_status == HAL_TIMEOUT)
	{
		//(for string only) Add string stop symbol \0 to end string
		uint32_t lastCharPos = hlpuart1.RxXferSize - hlpuart1.RxXferCount;
		RxBuffer[lastCharPos] = '\0';

		//return received char
		sprintf((char*)TxBuffer,"turn CAPLOCK on, send S____ to begin\r\n");
		HAL_UART_Transmit(&hlpuart1, TxBuffer, strlen((char*)TxBuffer), 5);

	}
}
//Blink LED 5 Hz
void DummyTask()
{
	static uint32_t timestamp=0;
	if(HAL_GetTick()>= timestamp)
	{
		timestamp = HAL_GetTick()+100;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
