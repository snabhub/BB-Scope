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
#include "lcd.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

DMA_HandleTypeDef hdma_memtomem_dma2_channel1;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint16_t size = 240;
uint16_t color1 = BLUE, color2 = YELLOW;
uint16_t Y_L[]={1,2,3,5};
uint16_t begin = 0;
uint16_t first_graph[240];
uint16_t wave_graph[240];
uint16_t ADCDATA[240];
uint16_t value[240];
uint8_t y=0;
uint16_t VMin;
uint16_t VMax;
uint16_t VAve;
uint16_t Vpk_pk=0;
int16_t graphave=0;
uint8_t freeze=0;
uint8_t trigger=0;

int counter = 0;
int aState = 0;
int aLastState = 0;
int temp = 0;

float Frequency = 8000;
float Amplitude = 1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void dataAnalize(void);
void drawgridh(uint16_t j);
void drawgridv(uint16_t j);
void drawgrid(void);
void runstopgraph(uint8_t freestate);
void DrawGraph(int16_t graph[], uint16_t color);
void datamine(uint8_t state, uint32_t loop1, uint8_t t);
void showvalue(void);
void voltageAnalize(void);
void showtimebase(uint8_t state);
void showvoltagebase(uint8_t state);
void voltagechange(uint8_t state,int16_t offset);
void showbase(void);
void drawui(uint16_t text, uint16_t bg, int select);
void triggermode(uint8_t trigger);
void showtriggeronoff();

void Scope();

void Wave(int type, float freq, float amp);

void Generator();
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
  MX_FSMC_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


  HAL_TIM_Base_Start(&htim2);
  HAL_DAC_Start_DMA(&hdac,  DAC1_CHANNEL_1, wave_graph, 240, DAC_ALIGN_12B_R);
  TIM2->PSC = 1;
  LCD_Clear(0, 0, 240, 80, BG);
  drawgrid();

    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	  Scope();
  //	  Generator();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_channel1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_channel1 on DMA2_Channel1 */
  hdma_memtomem_dma2_channel1.Instance = DMA2_Channel1;
  hdma_memtomem_dma2_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_channel1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
void drawui(uint16_t text, uint16_t bg, int select){
	if (select == 0){
		LCD_DrawString(143,0, "Voltage", bg, text);
		LCD_DrawString(127,0, "Time", text, bg);
		LCD_DrawString(111,0, "Trigger", text, bg);
	}
	else if (select == 1){
		LCD_DrawString(143,0, "Voltage", text, bg);
		LCD_DrawString(127,0, "Time", bg, text);
		LCD_DrawString(111,0, "Trigger", text, bg);
	}
	else if (select == 2){
		LCD_DrawString(143,0, "Voltage", text, bg);
		LCD_DrawString(127,0, "Time", text, bg);
		LCD_DrawString(111,0, "Trigger", bg, text);
	}
}

void drawgridh(uint16_t j){
	for(uint16_t i=0;i<240;i+=2){
		LCD_DrawDot(i, j, GRID);
	}
}

void drawgridv(uint16_t j){
	for(uint16_t i=80;i<320;i+=2){
		LCD_DrawDot(j, i, GRID);
	}
}

void drawgrid(){
	for(uint16_t i=80;i<=320;i+=24){
		drawgridh(i);
	}
	  drawgridh(199);
	  drawgridh(201);

	for(uint16_t i=0;i<=240;i+=24){
		drawgridv(i);
	}
	  drawgridv(119);
	  drawgridv(121);

}

void DrawGraph(int16_t graph[], uint16_t color){
	for (uint16_t x = 1; x < 240; x++){
		if ((graph[x]>=0) && (graph[x]<240)){
		 LCD_DrawLine(graph[x], x+80, graph[x-1], x+79, color);
		}
	}
}

void triggermode(uint8_t state){
	if(state==1){
		uint16_t tri=0;
		tri=HAL_ADC_GetValue(&hadc1);
		while(tri!=2048){
			tri=HAL_ADC_GetValue(&hadc1);
		}
	}
}

void runstopgraph(uint8_t freestate){
	if(freestate==0){
		DrawGraph(first_graph, color2);
		HAL_Delay(300);
		DrawGraph(first_graph, BLACK);
		drawgrid();
	}
	else{
		DrawGraph(first_graph, color2);
		while(1){
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==1){
				while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1);
				DrawGraph(first_graph, BLACK);
				drawgrid();
				freeze+=1;
				freeze%=2;
				break;
			}
		}
	}
}

void datamine(uint8_t state, uint32_t loop1, uint8_t t){
	  while(1){
		  if(state==0){
			  if(loop1<240){
				  ADCDATA[t]=HAL_ADC_GetValue(&hadc1);
				  value[t]=ADCDATA[t]*3600/4096;
				  first_graph[t]=ADCDATA[t]*240/4096;
				  loop1+=1;
				  t+=1;
				  continue;
			  }
			  break;
		  }
		  else if(state==1){
			  if(loop1<2400){
				  if((loop1%10)==0){
					  ADCDATA[t]=HAL_ADC_GetValue(&hadc1);
					  value[t]=ADCDATA[t]*3600/4096;
					  first_graph[t]=ADCDATA[t]*240/4096;
					  t+=1;
				  }
				  loop1+=1;
				  continue;
			  }
			  break;
		  }
		  else if(state==2){
			  if(loop1<24000){
				  if((loop1%100)==0){
					  ADCDATA[t]=HAL_ADC_GetValue(&hadc1);
					  value[t]=ADCDATA[t]*3600/4096;
					  first_graph[t]=ADCDATA[t]*240/4096;
					  t+=1;
				  }
				  loop1+=1;
				  continue;
			  }
			  break;
		  }
	  }
}

void voltagechange(uint8_t state,int16_t offset){
	if(freeze==0){
		for(uint8_t i=0;i<240;i++){
			first_graph[i]=(first_graph[i]-graphave)*Y_L[state]+graphave+offset;
		}
	}
}

void voltageAnalize() {
  uint16_t d;
  uint32_t sum = 0;
  uint16_t a;
  uint32_t graphsum = 0;

  VMin = 3600;
  VMax = 0;
  for (uint16_t i = 0; i < 240; i++) {
    d = value[i];
    sum = sum + d;
    if (d < VMin) {
      VMin = d;
    }
    if (d > VMax) {
      VMax = d;
    }

    a=first_graph[i];
    graphsum=graphsum+a;
  }
  VAve = sum / 240;
  Vpk_pk= VMax-VMin;
  graphave = graphsum / 240;

}

uint16_t freqAnalize(){
	uint16_t k=0;
	uint16_t VMin5DOWN=VMin*(1-0.05);
	uint16_t VMin5UP=VMin*(1+0.05);
	for(uint8_t j=0;j<240;j++){
		if(value[j]==VAve){
			for(uint8_t i=0;i<240;i++){
				k+=1;
				if(value[i]>=VMin5DOWN && value[i]<=VMin5UP){
					break;
				}
			}
		}
	}
	return k;
}

void showvalue(){
	char stra[80];
	char strb[80];
	char strc[80];
	char strd[80];
	sprintf(stra, "%4u",VAve);
	sprintf(strb, "%4u",VMax);
	sprintf(strc, "%4u",VMin);
	sprintf(strd, "%4u",Vpk_pk);
	LCD_DrawString(220 ,32, stra, RED, BG);
	LCD_DrawString(200 ,32, strb, RED, BG);
	LCD_DrawString(180 ,32, strc, RED, BG);
	LCD_DrawString(160 ,32, strd, RED, BG);
}

void showtimebase(uint8_t state){
	if(state==0){
		LCD_DrawString(120,40," 50", RED, BG);
		LCD_DrawString(120,64,"us", FG, BG);
	}
	else if(state==1){
		LCD_DrawString(120,40,"250", RED, BG);
		LCD_DrawString(120,64,"us", FG, BG);
	}
	else if(state==2){
		LCD_DrawString(120,40,"  2", RED, BG);
		LCD_DrawString(120,64,"ms", FG, BG);
	}
}

void showtriggeronoff(){
	if(trigger==0){
		LCD_DrawString(100,50,"oFF", RED, BG);
	}
	else{
		LCD_DrawString(100,50," oN", RED, BG);
	}
}

void showvoltagebase(uint8_t state){
	if(state==0){
		LCD_DrawString(140,40,"360", RED, BG);
	}
	else if(state==1){
		LCD_DrawString(140,40,"180", RED, BG);
	}
	else if(state==2){
		LCD_DrawString(140,40,"120", RED, BG);
	}
	else if(state==3){
		LCD_DrawString(140,40," 72", RED, BG);
	}
}

void showbase(){
	  LCD_OpenWindow (0,0,240,80);
	  LCD_FillColor(240*80,WHITE);
	  LCD_DrawString(220,0,"vav=    mv", FG, BG);//v@G=
	  LCD_DrawString(200,0,"vMA=    mv", FG, BG);
	  LCD_DrawString(180,0,"vMI=    mv", FG, BG);
	  LCD_DrawString(160,0,"vPP=    mv", FG, BG);
	  LCD_DrawString(140,0,"v/DV=   mv", FG, BG);
	  LCD_DrawString(120,0,"!/DV=", FG, BG);
	  LCD_DrawString(100,0,"!rig: ", FG, BG);
	  LCD_DrawString(80,0,"Frequency:", FG, BG);
//	  LCD_DrawString(40,0,"Qmplitude:", FG, BG);
	  LCD_DrawString(40,0,"Ohase:", FG, BG);
	  drawgrid();
}

void drawencoder(){
	  aState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	  if (aState != aLastState){
		  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) != aState){
			  counter++;
		  }else{
			  if (counter > 0){
				  counter--;
			  }
		  }
	  }
	  aLastState = aState;
	  char s[3];
	  sprintf(s, "%3u",counter);
	  LCD_DrawString(0,100,s, FG, BG);
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)==0){
		  LCD_DrawString(200,200,s, FG, BG);
	  }
	  if ((counter%3)!=(temp%3)){
		  LCD_Clear(143,0, 16*3, 80, WHITE);
		  drawui(FG, BG, counter%3);
	  }
	  temp = counter;
	  if (counter%3==0){
		  LCD_DrawString(160, 2, "Selecting", FG, BG);
	  }
	  else if(counter%3==1){
		  LCD_DrawString(160, 22, "Selecting", FG, BG);
	  }
	  else{
		  LCD_DrawString(160, 42, "Selecting", FG, BG);
	  }
}

void Scope(void){
	uint8_t stateloop=0;
	uint8_t voltagestate=0;
	uint16_t voltageoffset=0;
	showbase();
	uint32_t loop1=0;
	uint8_t t=0;

	triggermode(trigger);
	if(freeze==0){
	  datamine(stateloop,loop1,t);
	}

	voltageAnalize();
	showvalue();
	showtimebase(stateloop);
	showvoltagebase(voltagestate);
	showtriggeronoff();
	voltagechange(voltagestate, voltageoffset);
	runstopgraph(freeze);

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1){
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1);
		trigger+=1;
		trigger%=2;
	}

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==1){
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1);
		freeze+=1;
		freeze%=2;
	}

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)==1){
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == 1);
		stateloop+=1;
		stateloop%=3;
	}

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)==1){
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 1);
		voltagestate+=1;
		voltagestate%=4;
	}

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)==1){
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1);
		voltageoffset+=24;
	}

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==1){
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 1);
		voltageoffset-=24;
	}
}

void Wave(int type, float freq, float amp){
//	max frequency is 18.7 kHz;
//	max amplitude is 3.3V

	amp = (amp > 3.3)? 3.3: amp;
	float arr = 150000;
	freq = (freq > 2.29)? (arr/freq): 65535;
	TIM2->ARR = (freq > 1)? (int)(freq-1): 1;
	switch(type){
		case 0:{
			for (int x = 0; x < 240; x++){
			  wave_graph[x] = (int)(amp*(sin(x*M_PI/120)+1)*(2047)/3.3);
			}
			break;
		}
		case 1:{
			for (int x = 0; x < 240; x++){
			  wave_graph[x] = (x < 120)? (int)(x*amp*4095/(240*3.3)): (int)((240-x)*amp*4095/(240*3.3));
			}
			break;
		}
		case 2:{
			for (int x = 0; x < 240; x++){
			  wave_graph[x] = (int)(x*amp*4095/(240*3.3));
			}
			break;
		}
		default:{
			for (int x = 0; x < 240; x++){
			  wave_graph[x] = (x > 120)? (int)(4095*amp/3.3): 0;
			}
			break;
		}
	}
}

void Generator(void){
	char stra[80];
	char strb[80];
	sprintf(stra, "%4uHz",(int)Frequency);
	sprintf(strb, "%4uv",(int)Amplitude);
	LCD_DrawString(210, 5, "Qmpl:", FG, BG);
	LCD_DrawString(190, 5, strb, FG, BG);
	LCD_DrawString(170, 5, "base:", FG, BG);
	LCD_DrawString(150, 5, stra, FG, BG);
	LCD_DrawString(210, 120, "Wave type:", FG, BG);
	for(int i = 80; i < 320; i++){
	  int point = wave_graph[i-80]*120/4096;
	  LCD_DrawDot(58+(point), i, BG);
	  LCD_DrawDot(60+(point), i, BG);
	  LCD_DrawDot(62+(point), i, BG);
	}
	Wave(1, Frequency, Amplitude);
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
