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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define numChannelsADC 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* Optimized memory allocation for ADC data buffer */
__attribute__((section(".RAM_D1"))) uint16_t adcData[numChannelsADC];

/* Benchmarking timer stop time value [us] */
volatile float elapsedTime = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void ADC_DMA_Conv_Init(void);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize the ADC conversions via DMA */
  ADC_DMA_Conv_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HSIKER;
  PeriphClkInit.HSIKerClockDivider = RCC_HSIKER_DIV1;

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Configure and initiate ADC conversion and DMA MtP transfers, in circular mode */
void ADC_DMA_Conv_Init(void)
  {
     /* Perform ADC enable and conversion start if no conversion is on going */
  	 if (LL_ADC_REG_IsConversionOngoing(ADC1) == 0UL)
  	   {
  	      /* Process locked */
  		  __HAL_LOCK(&hadc1);

  	      /* Specific case for first call occurrence of this function (DMA transfer */
  	      /* not activated and ADC disabled), DMA transfer must be activated        */
  	      /* with ADC disabled.                                                     */
  	      if ((ADC1->CFGR1 & ADC_CFGR1_DMAEN) == 0UL)
  	        {
  	           if (LL_ADC_IsEnabled(ADC1) != 0UL)
  	             {
  	                /* Disable ADC */
  	                LL_ADC_Disable(ADC1);
  	              }

  	             /* Enable ADC DMA mode */
  	             ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
  	         }

  	      /* Enable the ADC peripheral */
  	      LL_ADC_Enable(ADC1);

  	      /* Set ADC error code */
  	      /* Reset all ADC error code fields */
  	      ADC_CLEAR_ERRORCODE(&hadc1);

	      /* Clear regular group conversion flag and overrun flag */
  	      /* (To ensure of no unknown state from potential previous ADC           */
          /* operations)                                                          */
  	      __HAL_ADC_CLEAR_FLAG(&hadc1, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

  	      /* Process unlocked */
  	      /* Unlock before starting ADC conversions: in case of potential         */
  	      /* interruption, to let the process to ADC IRQ Handler.                 */
  	      __HAL_UNLOCK(&hadc1);

          /* Enable the interrupt in case of data overrun */
  	      ADC1->IER |= ADC_IT_OVR;

  	      /* DMA Process locked */
  	      __HAL_LOCK(&hdma_adc1);

  	      /* Disable the DMA peripheral, to configure it */
  	      DMA1_Channel1->CCR &= ~DMA_CCR_EN;

  	      /* Clear all DMA flags */
  	      DMA1->IFCR |= (DMA_FLAG_TC1 | DMA_FLAG_HT1 | DMA_FLAG_GI1);

  	      /* Configure DMA Channel data length (for all ADC channels, enter the number of channels) */
  	      DMA1_Channel1->CNDTR = numChannelsADC;

          /* MEMORY TO PERIPHERAL DMA TRANSFER PARAMETERS */

  	      /* Configure DMA Channel source address (ADC data register) */
  	      DMA1_Channel1->CPAR = (uint32_t)&hadc1.Instance->DR;

  	      /* Configure DMA Channel destination address (memory buffer, automatically filled element by element) */
  	      DMA1_Channel1->CMAR = (uint32_t)adcData;

  	      /* Enable DMA Circular mode (with this set and EOC for sequence, the conversions continue automatically until explicitly stopped) */
  	      /* If stopped, the ADC_DMA_Conv_Init function needs to be called again in order to start it again */
  	      DMA1_Channel1->CCR |= DMA_CCR_CIRC;

  	      /* Disable DMA half-transfer complete callback */
  	      DMA1_Channel1->CCR &= ~DMA_CCR_HTIE;

  	      /* Enable DMA transfer complete interrupt (occurs after ADC conversion complete and the full data transfer) */
  	      DMA1_Channel1->CCR |=  DMA_CCR_TCIE;

  	      /* Enable DMA channel 1 (for ADC data peripheral to memory transfer) */
  	      DMA1_Channel1->CCR |=  DMA_CCR_EN;

  	      /* Unlock the DMA channel for ADC */
  	      __HAL_UNLOCK(&hdma_adc1);

  	      /* Benchmarking, initialize the timer for the 1st time, start ticking until the callback is reached */
  	      TIM16->CNT = 0;
  	      TIM16->CR1 |= TIM_CR1_CEN;

  	      /* Start the initial ADC conversion */
  	      ADC1->CR |= (1<<2);

        }
   }

/* Artificially invoked by DMA transfer complete IRQ handler, for TC flag, because the ADC interrupt is disabled */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	/* Benchmarking */
    /* Stop TIM6 counter */
	TIM16->CR1 &= ~TIM_CR1_CEN;

	/* Obtain the elapsed time */
    elapsedTime = (float)(TIM16->CNT)/48.0; /* Elapsed time for TIM16 in microseconds */

    /* Benchmarking */
    /* Start, count until the next group conversion (and DMA transfer) is complete */
    TIM16->CNT = 0;
    TIM16->CR1 |= TIM_CR1_CEN;

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
