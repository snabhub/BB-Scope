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

TIM_HandleTypeDef htim3;

DMA_HandleTypeDef hdma_memtomem_dma2_channel1;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint16_t size = 320;
uint16_t color1 = BLUE, color2 = YELLOW;
uint32_t X_L[]={1,10,100,1000,10000,100000,1000000};
uint16_t Y_L[]={1,5,10,50,100,500,1000,5000};
uint16_t begin = 0;
uint16_t first_graph[320];
uint16_t ADCDATA[320];
uint16_t value[320];
uint8_t y=0;
uint16_t index1=0;


uint16_t VMin;
uint16_t VMax;
uint16_t VAve;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void dataAnalize();
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
  /* USER CODE BEGIN 2 */
  LCD_INIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  void DrawGrid(){
	  for (uint8_t i = 1; i < 16; i++){
		  for (uint8_t j = 0; j < 240; j+=2){
			  if(i == 8){
				  LCD_DrawDot(j, i*20-1, GRID);
				  LCD_DrawDot(j, i*20+1, GRID);
			  }
			  LCD_DrawDot(j, i*20, GRID);
		  }
	  }
	  for (uint8_t i = 1; i < 12; i++){
		  for (uint16_t j = 0; j < 320; j+=2){
			 if(i == 6){
			  LCD_DrawDot(i*20-1, j, GRID);
			  LCD_DrawDot(i*20+1, j, GRID);
			}
			LCD_DrawDot(i*20, j, GRID);
		  }
	  }
  }
  void DrawGraph(uint16_t graph[], uint32_t color){
	  for (uint16_t x = 1; x < 320; x++){
		  if (graph[x]>0 && graph[x]<240){
			  LCD_DrawLine(graph[x], x, graph[x-1], x-1, color);
		  }
	  }
  }
//  void ZoomInVertical(float graph[], uint32_t Yscale){
//	  for (uint16_t x = 0; x < 320; x++){
//		  graph[x] = graph[x]/Yscale;
//	  }
//  }
//  void ZoomOutVertical(float graph[], uint32_t Yscale){
//	  for (uint16_t x = 0; x < 320; x++){
//		  graph[x] = graph[x]*Yscale;
//	  }
//  }
//  float ZoomInHorizon(float zoom, float scale){
//	  float result = zoom*scale;
//	  return result;
//  }
//  float ZoomOutHorizon(float zoom, float scale){
//	  float result = zoom/scale;
//	  return result;
//  }
  void MoveUp(uint16_t graph[]){
	  for (uint16_t x = 0; x < 320; x++){
		  graph[x] = graph[x] + 1;
	  }
  }
  void MoveDown(uint16_t graph[]){
	  for (uint16_t x = 0; x < 320; x++){
		  graph[x] = graph[x] - 1;
	  }
  }
//	double MoveLeft(float begin, float adjust){
//	  double result = begin - adjust;
//	  return result;
//	}
//	double MoveRight(float begin, float adjust){
//	  double result = begin + adjust;
//	  return result;
//	}
  DrawGrid();

//  for (int x = 0; x < gsize1; x++){
//	  y1 = 60+60*sin(x*2*M_PI/160);
//	  y2 = 180+60*sin(x*2*M_PI/160);
//	  first_graph[x] = y1;
//	  second_graph[x] = y2;
//  }
//  size1 = sizeof(first_graph)/sizeof(first_graph[0]);
//  size2 = sizeof(second_graph)/sizeof(second_graph[0]);
//  DrawGraph(first_graph, size1, zoom_x1, color1);
//  DrawGraph(second_graph, size2, zoom_x2, color2);
//  HAL_ADC_Start_DMA(&hadc1, &value[index1],320);


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_ADC_Start_DMA(&hadc1,value,320);
//	  if(HAL_ADC_GetValue(&hadc1)*3300/4096>1500){
		for(uint16_t i=0;i<320;i++){
//			HAL_ADC_Start_DMA(&hadc1,&value[i],1);
			ADCDATA[i]=HAL_ADC_GetValue(&hadc1);
			value[i]=ADCDATA[i]*3300/4096;
			first_graph[i]=ADCDATA[i]*Y_L[y]*240/4096;
		}
//	  }

		dataAnalize();
		char stra[80];
		char strb[80];
		char strc[80];
		sprintf(stra, "%4u",VAve);
		sprintf(strb, "%4u",VMax);
		sprintf(strc, "%4u",VMin);
		LCD_DrawString(60,20 , stra );
		LCD_DrawString(60,40 , strb );
		LCD_DrawString(60,60 , strc );
		LCD_DrawString(20,20,"VAve=");
		LCD_DrawString(20,40,"VMax=");
		LCD_DrawString(20,60,"VMin=");

		DrawGraph(first_graph, color2);
		HAL_Delay(300);
		DrawGraph(first_graph, BLACK);
		DrawGrid();

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1){
			HAL_Delay(300);
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1);
			y=(y+1)%8;
		}

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1){
			HAL_Delay(300);
			while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1);
			y=(y+7)%8;
		}



//		  for (int i = 0; i < 320; i++){
//			  if (check==1&&i>0){
//				  for (int j = 0; j < 240; j++){
//					  if (j%20 != 0 && i%20 != 0 && j!= 119 && j!=121 && i!=159 && i!=161){
//						  LCD_DrawDot(j, i, BG);
//					  }
//					  else if (j%20 == 0 && j%2 != 0 && i%20 == 0 && i%2!=0) {
//						  LCD_DrawDot(j, i, GRID);
//					  }
//					  if (j==0){
//						  LCD_DrawDot(0, i, BG);
//					  }
//				  }
//			  }
//			  if (i>0){
//				  first_graph[i-1] = r;
//			  }
//			  r = HAL_ADC_GetValue(&hadc2)*240/4095+120;
//			  if (i>1){
//				  LCD_DrawLine(first_graph[i-2], i-1, first_graph[i-1], i, FG);
//			  }
//			  HAL_Delay(t/320);
//		  }
//		  check = 1;


//	  key1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
//	  	if (key1 == 1){
//	  		HAL_Delay(200);
//	  		key1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
//	  		LCD_Clear(0, 0, 240, 320, BLACK);
//	  		DrawGrid();
//	  		size1 = sizeof(first_graph)/sizeof(first_graph[0]);
//	  		size2 = sizeof(second_graph)/sizeof(second_graph[0]);
//	  		if (key1 == 1){
//	  			begin = MoveRight(begin, 10);
//	  		}
//	  		else{
//	  			begin = MoveRight(begin, 10);
//	  		}
//	  		DrawGraph(first_graph, size1, zoom_x1, color1);
//	  		DrawGraph(second_graph, size2, zoom_x2, color2);
//	  	}
//	  	key2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//	  	if (key2 == 1){
//	  		HAL_Delay(200);
//	  		key2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//	  		LCD_Clear(0, 0, 240, 320, BLACK);
//	  		DrawGrid();
//	  		if (key2 == 1){
//	  			MoveUp(first_graph, size1, 10);
//	  			MoveUp(second_graph, size2, 10);
//	  		}
//	  		else{
//	  			MoveDown(first_graph, size1, 10);
//	  			MoveDown(second_graph, size2, 10);
//	  		}
//	  		size1 = sizeof(first_graph)/sizeof(first_graph[0]);
//	  		size2 = sizeof(second_graph)/sizeof(second_graph[0]);
//	  		DrawGraph(first_graph, size1, zoom_x1, color1);
//	  		DrawGraph(second_graph, size2, zoom_x2, color2);
//	  	}
//	  	key3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
//	  	if (key3 == 1){
//	  		HAL_Delay(200);
//	  		key3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
//	  		LCD_Clear(0, 0, 240, 320, BLACK);
//	  		DrawGrid();
//	  		if (key3 == 1){
//	  			ZoomOutVertical1(first_graph, size1, zoom_y1);
//	  			ZoomOutVertical2(second_graph, size2, zoom_y2);
//	  		}
//	  		else{
//	  			ZoomInVertical1(first_graph, size1, zoom_y1);
//	  			ZoomInVertical2(second_graph, size2, zoom_y2);
//	  		}
//	  		size1 = sizeof(first_graph)/sizeof(first_graph[0]);
//	  		size2 = sizeof(second_graph)/sizeof(second_graph[0]);
//	  		DrawGraph(first_graph, size1, zoom_x1, color1);
//	  		DrawGraph(second_graph, size2, zoom_x2, color2);
//	  	}
//	  	key4 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
//	  	if (key4 == 1){
//	  		HAL_Delay(200);
//	  		key4 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
//	  		LCD_Clear(0, 0, 240, 320, BLACK);
//	  		DrawGrid();
//	  		if (key4 == 1){
//	  		  zoom_x1 = ZoomOutHorizon(zoom_x1, 2);
//	  		  zoom_x2 = ZoomOutHorizon(zoom_x2, 2);
//	  		}
//	  		else{
//	  		  zoom_x1 = ZoomInHorizon(zoom_x1, 2);
//	  		  zoom_x2 = ZoomInHorizon(zoom_x2, 2);
//	  		}
//	  		key4 = 1;
//	  		size1 = sizeof(first_graph)/sizeof(first_graph[0]);
//	  		size2 = sizeof(second_graph)/sizeof(second_graph[0]);
//	  		DrawGraph(first_graph, size1, zoom_x1, color1);
//	  		DrawGraph(second_graph, size2, zoom_x2, color2);
//	  	}
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
void dataAnalize() {
  uint16_t d;
  uint32_t sum = 0;

  VMin = 3300;
  VMax = 0;
  for (uint16_t i = 0; i < 320; i++) {
    d = value[i];
    sum = sum + d;
    if (d < VMin) {
      VMin = d;
    }
    if (d > VMax) {
      VMax = d;
    }
  }

  VAve = sum / 320;
}

void showvalue(){

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
