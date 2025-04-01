/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "BasicTools.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// USB CDC
//    Extern needed
extern USBD_HandleTypeDef hUsbDeviceFS;
//    Buffers needed
uint8_t usb_cdc_tx_data[USB_CDC_BUFFER_SIZE];           // Circular buffer for TX data
uint8_t usb_cdc_rx_buffer[USB_CDC_BUFFER_SIZE];         // Buffer for RX data
uint8_t usb_cdc_tx_buffer[USB_CDC_TX_PACKET_SIZE];      // Buffer for USB packet transmission


// UART1 buffers
uint8_t uart1_rx_buf[UART1_RX_DMA_SIZE];
uint8_t uart1_tx_buf[UART1_TX_DMA_SIZE];
uint8_t uart1_data_buffer[UART1_TX_Buffer_SIZE];

// UART2 buffers
uint8_t uart2_rx_buf[UART2_RX_DMA_SIZE];
uint8_t uart2_tx_buf[UART2_TX_DMA_SIZE];
uint8_t uart2_data_buffer[UART2_TX_Buffer_SIZE];
uint32_t Led_Blink_Delay = 1000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void InitializeUARTs(void)
{
  // UART1 Setup
  UART_Register(&huart1,
               uart1_rx_buf, UART1_RX_DMA_SIZE,
               uart1_tx_buf, uart1_data_buffer,
               UART1_TX_DMA_SIZE, UART1_TX_Buffer_SIZE);
  HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, UART1_RX_DMA_SIZE);	// Once started, will keep listening into buffer

  // UART2 Setup
  UART_Register(&huart2,
               uart2_rx_buf, UART2_RX_DMA_SIZE,
               uart2_tx_buf, uart2_data_buffer,
               UART2_TX_DMA_SIZE, UART2_TX_Buffer_SIZE);
  HAL_UART_Receive_DMA(&huart2, uart2_rx_buf, UART2_RX_DMA_SIZE);	// Once started, will keep listening into buffer

  // USB CDC UART Setup
  USB_CDC_Init(&hUsbDeviceFS, usb_cdc_tx_data, USB_CDC_BUFFER_SIZE,
              usb_cdc_rx_buffer, USB_CDC_BUFFER_SIZE, usb_cdc_tx_buffer);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);									// 1ms interrupt
  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);						// 50us interrupt
  InitializeUARTs();
  SerialPrintf(&huart1,"%d :Hello World\r\n", millis());
  SerialPrintf(&huart2,"%d :Hello World\r\n", micros());
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Initialize variables for USB communication */
  uint16_t usb_incoming_byte_count = 0;

  /* Initialize variables for UART1 communication with receive timeout */
  uint16_t uart1_incoming_byte_count = 0;
  uint32_t uart1_rx_time_elapsed = 0;
  uint16_t uart1_last_count = 0;

  /* Calculated UART idle timeout (time for 2 bytes at 115200 baud) */
  const uint32_t UART_IDLE_TIMEOUT_US = 1000000 * 20 / huart1.Init.BaudRate; /* ~173 microseconds for 115200*/

  while (1)
  {
    /* ---- USB CDC Serial Handling ---- */
	usb_incoming_byte_count = USBSerialAvailable();
	if(usb_incoming_byte_count)
	{
	  uint8_t incoming_data[MAX_BUFFER_SIZE] = {0};

	  /* Read USB data and echo timestamp with data */
	  USBSerialPrintf("%dus -> %d byte received", micros(), usb_incoming_byte_count);
	  USBSerialReadBuf(incoming_data, usb_incoming_byte_count);

	  /* Forward USB data to UART1 with timestamp */
	  SerialPrintf(&huart1, "%dus -> ", micros());
	  SerialWriteBuf(&huart1, incoming_data, usb_incoming_byte_count);

	  /* Forward USB data to UART2 with timestamp */
	  SerialPrintf(&huart2, "%dms -> ", millis());
	  SerialWriteBuf(&huart2, incoming_data, usb_incoming_byte_count);
	}

	/* ---- UART1 Serial Handling with Timeout ---- */
	uart1_incoming_byte_count = SerialAvailable(&huart1);
	if(uart1_incoming_byte_count)
	{
	  if(uart1_rx_time_elapsed)
	  {
	    /* Check if we've waited long enough for complete message */
	    if(micros() > (uart1_rx_time_elapsed + UART_IDLE_TIMEOUT_US))
	    {
	      uint8_t incoming_data[MAX_BUFFER_SIZE] = {0};

	      /* Read UART1 data and echo timestamp with data */
	      SerialPrintf(&huart1, "%dus -> %d byte received", micros(), uart1_incoming_byte_count);
	      SerialReadBuf(&huart1, incoming_data, uart1_incoming_byte_count);

	      /* Forward UART1 data to USB with timestamp */
	      USBSerialPrintf("%dus -> ", micros());
	      USBSerialWriteBuf(incoming_data, uart1_incoming_byte_count);

	      /* Forward UART1 data to UART2 with timestamp */
	      SerialPrintf(&huart2, "%dms -> ", millis());
	      SerialWriteBuf(&huart2, incoming_data, uart1_incoming_byte_count);

	      /* Reset timeout after processing */
	      uart1_rx_time_elapsed = 0;
	    }
	    else
	    {
	      /* If data is still arriving, reset the timeout */
	      if(uart1_last_count != uart1_incoming_byte_count)
	      {
	        uart1_rx_time_elapsed = micros();
	        uart1_last_count = uart1_incoming_byte_count;
	      }
	    }
	  }
	  else
	  {
	    /* First byte received, start the timeout */
	    uart1_rx_time_elapsed = micros();
	    uart1_last_count = uart1_incoming_byte_count;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1200-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 20-1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1 ) // Trigger every (Check IOC Timer1 setting)
  {					   // 3*1200*20/72000000 = 1ms
	Led_Blink_Delay -= 1;
	if(Led_Blink_Delay == 0)
	{
		Led_Blink_Delay = 1000;
		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		//UARTprintf(&huart1, "Waiting for USB Serial input at micros() = %d; millis() = %d;\r\n" ,micros(),HAL_GetTick());
	}
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1 )
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)	// Activated every 50us.
		{   											// 3*1200/72000000 = 50us
			USB_CDC_TimerCallback();
		}
	}
}

// In your DMA handler callbacks in stm32f1xx_it.c:
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Handle_Uart_TX_DMA_Complete(huart);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    Handle_Uart_TX_DMA_HalfComplete(huart);
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
