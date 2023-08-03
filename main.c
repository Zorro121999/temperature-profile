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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "EMA_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
HAL_StatusTypeDef status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIR_FILTER_LENGTH 11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t raw;
float temp;
float tempFilter;
uint16_t tempInt;
float tempDiv=0;
uint32_t sampleDiv=0;
char msg[7];
char timer[7];
char rx_data[5];
volatile float setpoint=0.0;
volatile float oldSetpoint;
char received[3];
uint8_t hysteresis1;
uint8_t hysteresis2;
volatile uint32_t ticks;
uint32_t ticksLong;
uint8_t counter=0;
volatile uint8_t buttonCount;
GPIO_PinState transistor;
uint8_t hystOn;

float alpha;
EMA filt;
float filterOut;

//char txSetpoint[7]={0x7B,0x4D,0x00,0x0D,0x0A};
//char txSetpoint[10]={0x7B, 0x4D, 0x30, 0x30, 0x30, 0x42, 0x42, 0x38, 0x0D, 0x0A};
//char txSetpoint[7]={0x7B,0x4D,0x00,0x0B,0xB8,0x0D,0x0A};
char txSetpoint[1]={0xAA};
char txProcess[10]={0x7B,0x4D,0x30,0x37,0x2A,0x2A,0x2A,0x2A,0x0D,0x0A};
uint16_t set=3000;
char stringSet[2];
char rxSetpoint[10];

float firBuf[11];
float inpFIR;
float tempFIR;
uint8_t sumIndex;
uint8_t bufIndex=11;
float FIR_FILTER_RESPONSE[11]={-0.000000000000000002,
		-0.007855854095023677,
		0.040171175263434403,
		-0.103314801557551267,
		0.170762156469434240,
		0.800474647839412690,
		0.170762156469434268,
		-0.103314801557551308,
		0.040171175263434410,
		-0.007855854095023681,
		-0.000000000000000002};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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
  //EMA_Init(&filt,alpha);
  alpha=0.5;
  filt.out=0.0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //SysTick configuration

    SystemCoreClockUpdate();
    //generate interrupt for every 100ms
    SysTick_Config(SystemCoreClock/10);
    //SysTick ->LOAD = 72000-1;
    SysTick ->CTRL = 0;
    SysTick ->VAL = 0;
    SysTick ->CTRL = (SysTick_CTRL_TICKINT_Msk |
  		            SysTick_CTRL_ENABLE_Msk |
  					SysTick_CTRL_CLKSOURCE_Msk);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_ADC3_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&hlpuart1,(uint8_t*)rx_data,5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  HAL_UART_Transmit(&huart4, (uint8_t)*txSetpoint,1,HAL_MAX_DELAY);
	  //wait for a time of HAL_MAX_DELAY for a response from the cooler
	  //status=HAL_UART_Receive(&huart4, (uint8_t)*rxSetpoint,10,5000);
	  while(status==HAL_TIMEOUT)
	  {
	      HAL_UART_Transmit(&huart4, (uint8_t)*txProcess,10,HAL_MAX_DELAY);
	  	  HAL_UART_Receive(&huart4, (uint8_t)*rxSetpoint,10,5000);
	  }
	  while(status==HAL_OK)
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t)*txSetpoint,1,HAL_MAX_DELAY);
		  //currentTicks=ticks;
		  /*
		  while(ticks-currentTicks<100)
		  {

		  }
		  */
		  HAL_Delay(10);
		  //delay(100);
	  }

	  //verify that message was received correctly by printing the response on the console
	  HAL_UART_Transmit(&huart4, (uint8_t)*rxSetpoint,10,HAL_MAX_DELAY);

	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3,HAL_MAX_DELAY);
	raw=HAL_ADC_GetValue(&hadc3);
	temp=(raw-1030)*0.022;
	tempInt=(uint32_t)temp;
	sampleDiv++;
	if(sampleDiv==130000)
	{

		sampleDiv=0;
		tempFilter=EMA_Update(&filt,temp,alpha);
		//tempFIR=FIRFilter(temp);
		sprintf(msg,"%.3f\r\n", tempFilter);
		//sprintf(msg, "%hu\r\n",tempInt);
	    HAL_UART_Transmit(&hlpuart1,(uint8_t*)msg, strlen(msg),HAL_MAX_DELAY);
	    sprintf(timer,"%ld\r\n",ticksLong);
	    HAL_UART_Transmit(&hlpuart1,(uint8_t*)timer, strlen(timer),HAL_MAX_DELAY);
	}
	//sprintf(msg, "%hu\r\n",tempDiv);
	//gcvt(tempDiv,5,msg);

	if(tempInt>setpoint)
	{
	  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	  hystOn=1;
	}
	else if(tempInt<setpoint-1)
	{
	  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	  hystOn=0;
	}
	else if(tempInt<=setpoint && tempInt>=setpoint-1 && hystOn==1)
	{
	  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	}
	else if(tempInt<=setpoint && tempInt>=setpoint-1 && hystOn==0)
	{
	  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	}


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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* LPUART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(LPUART1_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : transistor_Pin */
  GPIO_InitStruct.Pin = transistor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(transistor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  UNUSED(huart);

  //HAL_UART_Receive_IT(&hlpuart1, (uint8_t*) rx_data, strlen(rx_data), 5000);
  //HAL_UART_Transmit(&hlpuart1,(uint8_t*) received, 3, HAL_MAX_DELAY);
  while(setpoint==0.0f)
  {
  HAL_UART_Receive_IT(&hlpuart1,(uint8_t*)rx_data,5);
  setpoint=atof(rx_data);
  }
  if(tempDiv<setpoint)
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  }
  else
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }


}
*/
 void SysTick_Handler(void)
 {
	 HAL_IncTick();
	 ticks++;
	 if(ticks==0)
	  	{
	  		counter++;
	  	}
 	ticksLong=counter*65535+ticks;

 }

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	 char firstChar;
	 HAL_StatusTypeDef status;
	 if(GPIO_Pin=B1_Pin)
	 {
		 if(buttonCount==0)
		 {
		   //setpoint=0.0;
		   while(setpoint==oldSetpoint)
		   {
			   HAL_UART_Receive(&hlpuart1,(uint8_t*)rx_data,5,HAL_MAX_DELAY);
			   setpoint=atof(rx_data);
		   }
		   oldSetpoint=setpoint;
		   buttonCount++;
		 }

		 else
		 {
             //send setpoint too cooler
             HAL_UART_Transmit(&huart4, (uint8_t)*txProcess,10,HAL_MAX_DELAY);
             //wait for a time of HAL_MAX_DELAY for a response from the cooler
		     status=HAL_UART_Receive(&huart4, (uint8_t)*rxSetpoint,10,5000);
		     while(status!=HAL_OK)
		     {
		    	 HAL_UART_Transmit(&huart4, (uint8_t)*txProcess,10,HAL_MAX_DELAY);
		    	 HAL_UART_Receive(&huart4, (uint8_t)*rxSetpoint,10,5000);

		     }
		     //verify that message was received correctly by printing the response on the console
		     HAL_UART_Transmit(&huart4, (uint8_t)*rxSetpoint,10,HAL_MAX_DELAY);
             //wait for 2s for in order to be able to read the message on the console
			 delay(2000);
			 buttonCount=0;
		 }
	 }
 }

 void delay(uint16_t millis)
 {
	 uint16_t currentMillis = ticks;
	 while(ticks-currentMillis<millis)
	 {

	 }
 }
/*
 float FIRFilter( inpFIR)
 {
	 float out=0;
	 firBuf[bufIndex]=inpFIR;
	 bufIndex++;

	 if(bufIndex==FIR_FILTER_LENGTH)
	 {
		 bufIndex=0;
	 }

	 sumIndex=bufIndex;
	 for(uint8_t n=0; n<FIR_FILTER_LENGTH;n++)
	 {
		 if(sumIndex>0)
		 {
			 sumIndex--;
		 }
		 else
		 {
			 sumIndex=FIR_FILTER_LENGTH-1;
		 }
		 return out+=FIR_FILTER_RESPONSE[n]*firBuf[sumIndex];
	 }
 }
*/
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
