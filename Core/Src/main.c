/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
ADC_HandleTypeDef hadc2;

IWDG_HandleTypeDef hiwdg1;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;

WWDG_HandleTypeDef hwwdg1;

/* USER CODE BEGIN PV */
uint32_t readValue;
int tCelsius;
int tFahrenheit;
#define TS30    ((uint16_t*)((uint32_t)0x08FFF814))
#define TS110   ((uint16_t*)((uint32_t)0x08FFF818))

/* macro for UART timeout */
#define UART_TIMEOUT			100

/* macros for flash */
#define DATA_TRANSMIT_LENGTH	2048 //byte

#define BYTE_TO_BIT				8
#define DATA_WRITE_AT_A_TIME	16 //byte

#define WRITING_BUFFER_DATATYPE_UINT16	16

#define DATA_TRANSMIT_LOOP	(DATA_TRANSMIT_LENGTH*WRITING_BUFFER_DATATYPE_UINT16)/(DATA_WRITE_AT_A_TIME*BYTE_TO_BIT)

#define DATA_READ_AT_A_TIME		4 //byte
#define DATA_READ_LOOP			(DATA_TRANSMIT_LENGTH*WRITING_BUFFER_DATATYPE_UINT16)/(DATA_READ_AT_A_TIME*BYTE_TO_BIT) //32-read bits at a time & 4-bytes

/* macros for watchdog */
#define WATCHDOG_RESET_TIME		25 //in seconds
#define PRESCALAR				64 //0->4, 2->8, 4->16, 8->32, 16->64, 32->128, 64->256
#define RELOAD_VAL_FROM_SECONDS	(((WATCHDOG_RESET_TIME*1000*32000)/(4*PRESCALAR*1000))-1)

uint32_t random32bit_generatedNumber;
char alarmMsg[] = "ALARM ALARM ALARM\n";


uint16_t buffer_tfs[DATA_TRANSMIT_LENGTH] = {0};
uint32_t send_address = 0x080FE000U;
uint32_t rcv_address = 0x080FE000U;


RTC_TimeTypeDef Time;
RTC_TimeTypeDef setTime;
RTC_DateTypeDef Date;
RTC_DateTypeDef setDate;
RTC_AlarmTypeDef Alarm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RNG_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG1_Init(void);
static void MX_WWDG1_Init(void);
/* USER CODE BEGIN PFP */
void set_time_custom();
void set_date_custom();
void set_alarm_custom();
void get_time_date();
int get_temperature();
void transmit_temperature(int temp);
void gen_random_number();

void FLASH_write();
void FLASH_clear();

void FLASH_Read(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);
void Print_readed_data(uint32_t holdMultipleRead[]);
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
	// Use local
	uint32_t holdMultipleRead[DATA_READ_LOOP] = {0};

	for(int i=0; i<DATA_TRANSMIT_LENGTH; i++){
		buffer_tfs[i] = i;
	}
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
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_RNG_Init();
  MX_RTC_Init();
//  MX_IWDG1_Init();
  HAL_UART_Transmit(&huart3, "uart initialized again\n", 23, UART_TIMEOUT);
  MX_WWDG1_Init();
  /* USER CODE BEGIN 2 */

  /* watchdog */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//  HAL_UART_Transmit(&huart3, "uart initialized again\n", 23, 100);
//  for(int i=0; i<=2; i++){
//	  HAL_Delay(5000);
//	  HAL_UART_Transmit(&huart3, "5 seconds\n", 10, 500);
//	  HAL_IWDG_Refresh(&hiwdg1);
//  }

//  HAL_ADC_Start(&hadc2);


  /* set time, date & alarm */
//  set_time_custom();
//  set_date_custom();
//  set_alarm_custom();

  /* clear the flash */
//  FLASH_clear();

  /* write the flash */
//  FLASH_write();

  /* Read from the flash memory */
//  FLASH_Read(rcv_address, &holdMultipleRead[0], DATA_READ_LOOP);

  /* print the readed data */
//  Print_readed_data(holdMultipleRead);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_IWDG_Refresh(&hiwdg1);
	/* get the temperature */
//	int t = get_temperature();
//
//	/* transmit temperature through UART */
//    transmit_temperature(t);
//
//	/* get the random number */
//	gen_random_number();

	/* get the time and date */
//	get_time_date();
//
//	HAL_UART_Transmit(&huart3, "\n", 1, UART_TIMEOUT);

//	HAL_Delay(1000);

	  HAL_Delay(15);
	  HAL_WWDG_Refresh(&hwwdg1);

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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */

  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg1.Init.Window = 4095;
  hiwdg1.Init.Reload = 3749;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */

  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_SEPTEMBER;
  sDate.Date = 0x20;
  sDate.Year = 0x22;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x11;
  sAlarm.AlarmTime.Minutes = 0x12;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	HAL_PWR_EnableBkUpAccess();
  /* USER CODE END RTC_Init 2 */

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
  * @brief WWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG1_Init(void)
{

  /* USER CODE BEGIN WWDG1_Init 0 */

  /* USER CODE END WWDG1_Init 0 */

  /* USER CODE BEGIN WWDG1_Init 1 */

  /* USER CODE END WWDG1_Init 1 */
  hwwdg1.Instance = WWDG1;
  hwwdg1.Init.Prescaler = WWDG_PRESCALER_4;
  hwwdg1.Init.Window = 83;
  hwwdg1.Init.Counter = 119;
  hwwdg1.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG1_Init 2 */

  /* USER CODE END WWDG1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin|USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)alarmMsg, strlen(alarmMsg), UART_TIMEOUT);
}

void get_time_date(){
	char time[30];
	char date[30];
	HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
	sprintf(date, "Date: %02d.%02d.%02d\n", Date.Date, Date.Month, Date.Year);
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	sprintf(time, "Time: %02d:%02d:%02d\n", Time.Hours, Time.Minutes, Time.Seconds);
	HAL_UART_Transmit(&huart3, (uint8_t*)date, strlen(date), 5*UART_TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)time, strlen(time), 5*UART_TIMEOUT);
}

int get_temperature(){
	HAL_ADC_PollForConversion(&hadc2, 1000);
	readValue = HAL_ADC_GetValue(&hadc2);
	int temperature;
	temperature = (int)readValue - (uint32_t)*TS30;
	temperature *= (int)((uint32_t)110 - (uint32_t)30);
	temperature /= (int)(int32_t)((uint32_t)*TS110 - (uint32_t)*TS30);
	temperature += 30;
	return temperature;
}

void transmit_temperature(int temp){
	uint8_t temperature_c[100];
	uint8_t read[100];
	sprintf(temperature_c, "Temperature in Celcius: %d\n", temp);
	sprintf(read, "Temperature read: %d\n", readValue);
	HAL_UART_Transmit(&huart3, temperature_c, strlen(temperature_c), UART_TIMEOUT);
	HAL_UART_Transmit(&huart3, read, strlen(read), UART_TIMEOUT);
}

void gen_random_number(){
	HAL_RNG_GenerateRandomNumber(&hrng, &random32bit_generatedNumber);
	uint8_t rng_data[100] = {0};
	sprintf(rng_data, "Generated random number: %d", random32bit_generatedNumber);
	HAL_UART_Transmit(&huart3, rng_data, strlen(rng_data), UART_TIMEOUT);
	HAL_UART_Transmit(&huart3, "\n", 1, UART_TIMEOUT);
}

void set_time_custom(){
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0xFF){
		setTime.Hours = 0x11;
		setTime.Minutes = 0x11;
		setTime.Seconds = 0x00;
		HAL_RTC_SetTime(&hrtc, &setTime, RTC_FORMAT_BCD);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0xFF);
	} else{
		uint8_t temp[100] = {0};
		sprintf(temp, "time already set buddy! %d\n", HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0));
		HAL_UART_Transmit(&huart3, temp, strlen(temp), 5*UART_TIMEOUT);
	}
}

void set_date_custom(){
	setDate.Date = 0x23;
	setDate.Month = 0x09;
	setDate.Year = 0x22;
}

void set_alarm_custom(){
	Alarm.Alarm = RTC_ALARM_A;
	Alarm.AlarmTime.Hours = 0x11;
	Alarm.AlarmTime.Minutes = 0x12;
	Alarm.AlarmTime.Seconds = 0x00;
	Alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	HAL_RTC_SetAlarm_IT(&hrtc, &Alarm, RTC_FORMAT_BIN);
	uint8_t temp[] = "alarm set!\n";
	HAL_UART_Transmit(&huart3, (uint8_t*)temp, strlen(temp), 5*UART_TIMEOUT);
}

void FLASH_Read(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords){
	uint32_t temp;
	while (1)
		{
			*RxBuf = *(__IO uint32_t *)StartPageAddress;
			StartPageAddress+=4;
			RxBuf++;
			if (!(--numberofwords)) break;
		}
}

void Print_readed_data(uint32_t holdMultipleRead[]){
	uint8_t temp_data[10] = {0};
	uint16_t a,b;
	for(int i=0; i<DATA_READ_LOOP; i++){
	  a = holdMultipleRead[i]>>16;
	  b = holdMultipleRead[i] & 0x0000ffff;
	  sprintf(temp_data, "%d\n%d\n", b,a);
	  HAL_UART_Transmit(&huart3, temp_data, strlen(temp_data), UART_TIMEOUT);

  }
}

void FLASH_write(){
	// unlock the flash to write
	  if(HAL_FLASH_Unlock() != HAL_OK){
		  HAL_UART_Transmit(&huart3, "fail" ,4, 5*UART_TIMEOUT);
	  }
	  for(int i=0; i<DATA_TRANSMIT_LOOP; i++){
		  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)send_address, &buffer_tfs[i*8]) != HAL_OK)
		  {
			  HAL_UART_Transmit(&huart3, "fail" ,4, 5*UART_TIMEOUT);
		  }
	  send_address+=16;
	}
	// lock the flash once donw writing
	if(HAL_FLASH_Unlock() != HAL_OK){
	  HAL_UART_Transmit(&huart3, "fail" ,4, 5*UART_TIMEOUT);
	}
}

void FLASH_clear(){
	if(HAL_FLASH_Unlock() != HAL_OK){
		  HAL_UART_Transmit(&huart3, "fail" ,4, 5*UART_TIMEOUT);
	  }
	  uint32_t PAGEError;
	  FLASH_EraseInitTypeDef EraseInitStruct;
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.NbSectors = 1;
	  EraseInitStruct.Banks = FLASH_BANK_1;
	  EraseInitStruct.Sector = FLASH_SECTOR_127;
	  if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	  {
		  HAL_UART_Transmit(&huart3, "fail" ,4, 5*UART_TIMEOUT);
	  }
	  if(HAL_FLASH_Lock() != HAL_OK){
	    HAL_UART_Transmit(&huart3, "fail" ,4, 5*UART_TIMEOUT);
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
