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
#include <math.h>
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
long int tempProcessFloat;
char tempProcess[4];
char tempProcessString[7];
float floatProcess;



char rx_data[5];
volatile float setpoint=0.0;
volatile float setpointNew=0.0;
char invalidTemp[] = "Invalid Temperature";

char received[3];
uint8_t hysteresis1;
uint8_t hysteresis2;
volatile uint32_t ticks;
uint32_t ticksLong;
volatile uint8_t counter;
volatile uint8_t buttonCount;
GPIO_PinState transistor;
uint8_t hystOn;
uint8_t hystEnable;
uint32_t onTime;
uint32_t startTime;
volatile float startpoint;

float alpha;
EMA filt;
float filterOut;

char txSetpoint[10];
char rxSetpoint[10];
char txMinSetpoint[]="{M30F060\r\n";
char txMaxSetpoint[]="{M312710\r\n";
char txStopCooler[]="{M140000\r\n";
char txStartCooler[]="{M140001\r\n";
char rxStartCooler[10];
char txProcess[]="{M07****\r\n";
char rxProcess[10];


uint16_t set=3000;
char stringSet[2];
char rxSetpoint[10];
uint16_t testSetpoint;
uint8_t length;
uint32_t timeCooler;


float firBuf[31];
float inpFIR;
float outFIR;
float tempFIR;
uint8_t sumIndex;
uint8_t bufIndex;
//filter coefficients
float FIR_FILTER_RESPONSE[31]={-0.001085865252318356,
		-0.000820198808479928,
		-0.000399968844021554,
		0.000607254181376241,
		0.002691675426970146,
		0.006311591779100447,
		0.011798257363143711,
		0.019271958935107820,
		0.028584966261934557,
		0.039303202469766950,
		0.050732418817472154,
		0.061987359581297821,
		0.072095113518572937,
		0.080117842316778404,
		0.085276424945082460,
		0.087055934616432185,
		0.085276424945082460,
		0.080117842316778390,
		0.072095113518572965,
		0.061987359581297835,
		0.050732418817472175,
		0.039303202469766964,
		0.028584966261934575,
		0.019271958935107830,
		0.011798257363143716,
		0.006311591779100447,
		0.002691675426970146,
		0.000607254181376241,
		-0.000399968844021553,
		-0.000820198808479928,
		-0.001085865252318356};
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
 void delay(uint16_t millis);
float FIRFilter(float inpFIR);
float calcFloat(char hexTemp[4]);
char* calcHex(int intTemp);
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
    //generate interrupt for every 1ms
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

  //hardcode minimum and maximum Setpoint
  HAL_UART_Transmit(&huart4, (uint8_t*)txMaxSetpoint,strlen(txMaxSetpoint),HAL_MAX_DELAY);
  HAL_Delay(2000);
  HAL_UART_Transmit(&huart4, (uint8_t*)txMinSetpoint,strlen(txMinSetpoint),HAL_MAX_DELAY);
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3,HAL_MAX_DELAY);
	raw=HAL_ADC_GetValue(&hadc3);
	//transform 12 bit ADC value to temperature
	temp=(raw-1030)*0.0183;
	tempInt=(uint32_t)temp;
	sampleDiv++;
	tempFIR=FIRFilter(temp);
	HAL_Delay(2000);

	//pick one sample per second to be transmitted to the terminal
	if(sampleDiv==130000)
	{

		sampleDiv=0;
		//lowpass filter the results (it can be chosen between EMA and FIR filter)
		//tempFilter=EMA_Update(&filt,temp,alpha);
		tempFIR=FIRFilter(temp);
		sprintf(msg,"%.3f\r\n", tempFIR);
        //send filtered temp data to terminal
	    HAL_UART_Transmit(&hlpuart1,(uint8_t*)msg, strlen(msg),HAL_MAX_DELAY);
	    //send time in milliseconds to the terminal
	    sprintf(timer,"%ld\r\n",ticksLong);
	    HAL_UART_Transmit(&hlpuart1,(uint8_t*)timer, strlen(timer),HAL_MAX_DELAY);

	    //request process temperature of cooler (not tested)

	    HAL_UART_Transmit(&huart4, (uint8_t*)txProcess,strlen(txProcess),HAL_MAX_DELAY);
	    HAL_UART_Receive(&huart4, (uint8_t*)rxProcess,10,5000);
	    tempProcess[0]=rxProcess[7];
	    tempProcess[1]=rxProcess[6];
	    tempProcess[2]=rxProcess[5];
	    tempProcess[3]=rxProcess[4];

	    //convert hex value in string format into an integer
	    tempProcessFloat=calcFloat(tempProcess);

	    sprintf(tempProcessString,"%.2f\r\n",tempProcessFloat);
	    //send process temperature to PC in string format
	    HAL_UART_Transmit(&hlpuart1, (uint8_t*)tempProcessString,7,HAL_MAX_DELAY);

	}


	//Transistor On/Off control with hysteresis around final setpoint
	if(hystEnable==1)
	{
	if(temp>setpoint)
	{
	  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	  hystOn=1;
	}
	else if(temp<setpoint-1)
	{
	  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	  hystOn=0;
	}
	else if(temp<=setpoint && temp>=setpoint-1 && hystOn==1)
	{
	  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	}
	else if(temp<=setpoint && temp>=setpoint-1 && hystOn==0)
	{
	  HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	}
	}

    //switch the oven on and then off again at the time simulated in simulink
	onTime=ticks-startTime;
	if(setpoint-startpoint>15.00 && setpoint-startpoint<17.00)
	{

	   if(onTime<665000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>665000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);

	   }
	   else if(onTime>3030000)
	   {
		   //since a point close to the setpoint is reached the hysteresis control can be enabled
		   hystEnable=1;
	   }
	}
	else if(setpoint-startpoint>17.00 && setpoint-startpoint<19.00)
	{
	   if(onTime<712000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>712000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>19.00 && setpoint-startpoint<21.00)
	{
	   if(onTime<758000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>758000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>21.00 && setpoint-startpoint<23.00)
	{
	   if(onTime<804000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>804000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>23.00 && setpoint-startpoint<25.00)
	{
	   if(onTime<850000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>850000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>25.00 && setpoint-startpoint<27.00)
	{
	   if(onTime<896000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>896000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>27.00 && setpoint-startpoint<29.00)
	{
	   if(onTime<942000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>942000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>29.00 && setpoint-startpoint<31.00)
	{
	   if(onTime<988000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>988000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>31.00 && setpoint-startpoint<33.00)
	{
	   if(onTime<1034000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>1034000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>33.00 && setpoint-startpoint<35.00)
	{
	   if(onTime<1094000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>1094000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

	}
	else if(setpoint-startpoint>35.00 && setpoint-startpoint<37.00)
	{
	   if(onTime<1148000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_SET);
	   }
	   else if(onTime>1148000 && onTime<3030000)
	   {
		   HAL_GPIO_WritePin(transistor_GPIO_Port, transistor_Pin, GPIO_PIN_RESET);
	   }
	   else if(onTime>3030000)
	   {
		   hystEnable=1;
	   }

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
  //huart3.Init.OverSampling = UART_OVERSAMPLING_16;
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




*/
//
//SysTick interrupt handler
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

 //Button interrupt for user to enter the setpoints for oven and cooler
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	 char firstChar;
	 char setpointPromptTvac[]="Please enter Setpoint for TVAC";
	 HAL_StatusTypeDef status;
	 if(GPIO_Pin=B1_Pin)
	 {
		 if(buttonCount==0)
		 {

			 HAL_UART_Transmit(&huart4, (uint8_t*)setpointPromptTvac,strlen(setpointPromptTvac),HAL_MAX_DELAY);
			//UART checks if setpoint is in valid range
			while(setpointNew<20.00 || setpointNew>60.00)
			{
			  //UART waits for user to enter 5 chars
		      HAL_UART_Receive(&hlpuart1,(uint8_t*)rx_data,5,HAL_MAX_DELAY);
			  setpoint=atof(rx_data);
			  setpointNew=setpoint;
			}
			//reset setpointNew
			setpointNew=0.00;
            startTime=ticks;
            startpoint=temp;
            buttonCount++;
		 }

		 else
		 {
             //this part is not tested
			 char setpointPromptCooler[]="Please enter Setpoint for cooler";
			 char rxTemp[6];
			 float rxTempFloat;
			 int rxTempInt;
			 char rxTempHex[4];
			 char dumpArray[8];
			 float sign;
			 uint8_t signRound[4];
			 int result;
			 HAL_UART_Transmit(&hlpuart1, (uint8_t*)setpointPromptCooler,strlen(setpointPromptCooler),HAL_MAX_DELAY);
			 //UART waits for user to input setpoint in format +-xx.xx
			 HAL_UART_Receive(&hlpuart1, (uint8_t*)rxTemp,6,HAL_MAX_DELAY);


			 rxTempFloat=atof(rxTemp);
			 //Temperature value in float format is converted into hex value
			 rxTempFloat=rxTempFloat*100;
			 rxTempInt=(int)rxTempFloat;

			 if(rxTempInt<0)
			 {
			    result=65536+rxTempInt;
			 }
			 else
			 {
				 result=rxTempInt;
			 }
			 for(int i=3;i==0;i--)
			 {
				sign=result/pow(16,i);
				signRound[i]=floor(sign);
				result=result-signRound[i]*pow(16,i);
			 }

			 //hex values have to converted to string format
		     sprintf(rxTempHex[0],"%u",signRound[3]);
		     sprintf(rxTempHex[1],"%u",signRound[2]);
		     sprintf(rxTempHex[2],"%u",signRound[1]);
		     sprintf(rxTempHex[3],"%u",signRound[0]);
             //command is built be concatenation
			 memcpy(dumpArray,strcat("{M00",rxTempHex),8);
			 memcpy(txSetpoint,strcat(dumpArray,"\r\n"),10);
             //send setpoint to cooler
             HAL_UART_Transmit(&huart4, (uint8_t*)txSetpoint,sizeof(txSetpoint),HAL_MAX_DELAY);
             //wait for a time of HAL_MAX_DELAY for a response from the cooler
		     HAL_UART_Receive(&huart4, (uint8_t*)rxSetpoint,sizeof(rxSetpoint),5000);

		     //verify that message was received correctly by printing the response on the console
		     HAL_UART_Transmit(&huart4, (uint8_t*)rxSetpoint,sizeof(rxSetpoint),HAL_MAX_DELAY);
             //wait for 2s for in order to be able to read the message on the console
			 HAL_Delay(2000);
			 HAL_UART_Transmit(&huart4, (uint8_t*)txStartCooler,strlen(txStartCooler),HAL_MAX_DELAY);
			 HAL_UART_Receive(&huart4, (uint8_t*)rxStartCooler,strlen(rxStartCooler),HAL_MAX_DELAY);
			 buttonCount=0;

		 }
	 }
 }

 //calculate the temperature in readable form from hex value in command
 float calcFloat(char hexTemp[4])
 {
	 int intProcess;
	 int ergeb;
	 for(int i=0;i<=3;i++)
	 {
		 switch(hexTemp[i])
		 {
		 case '0':
			 intProcess=0;
			 break;
		 case '1':
			 intProcess=1;
			 break;
		 case '2':
			 intProcess=2;
			 break;
		 case '3':
			 intProcess=3;
		 case '4':
			 intProcess=4;
			 break;
		 case '5':
			 intProcess=5;
			 break;
		 case '6':
			 intProcess=6;
			 break;
		 case '7':
			 intProcess=7;
			 break;
		 case '8':
			 intProcess=8;
			 break;
		 case '9':
			 intProcess=9;
			 break;
		 case 'A':
			 intProcess=10;
			 break;
		 case 'B':
			 intProcess=11;
			 break;
		 case 'C':
			 intProcess=12;
			 break;
		 case 'D':
			 intProcess=13;
			 break;
		 case 'E':
			 intProcess=14;
			 break;
		 case 'F':
			 intProcess=15;
			 break;
		 }
     //in case temperature was positive
     if(hexTemp[0]=='0' || hexTemp[1]=='1' || hexTemp[2]=='2')
     {
    	 ergeb=ergeb+intProcess*pow(16,i);
     }
     else
     //in case temperature was negative
     {
    	 ergeb=ergeb+intProcess*pow(16,i)-65536;
	 }
	 return floatProcess=ergeb/100;
 }
 }



 //lowpass filter with a cutoff frequency of 0.04Hz and Hamming window
 float FIRFilter(float inpFIR)
 {
	 //fill up the ring buffer
	 firBuf[bufIndex]=inpFIR;
	 bufIndex++;

	 if(bufIndex==FIR_FILTER_LENGTH)
	 {
		 bufIndex=0;
	 }

	 outFIR=0;
	 //pointer to last input value
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
		 //calculate output vector
		 outFIR+=FIR_FILTER_RESPONSE[n]*firBuf[sumIndex];
	 }
	 return outFIR;
 }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

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
