/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
//#include "timers.h"
#include "queue.h"
#include "semphr.h"
//#include "event_groups.h"

#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
//void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SemaphoreHandle_t SimpleMutex;



TaskHandle_t HPT_Handler;
/*TaskHandle_t MPT_Handler;
TaskHandle_t LPT_Handler;*/

void HPT_Task (void *argument);
/*void MPT_Task (void *argument);
void LPT_Task (void *argument);*/

uint8_t rx_data=0;

/*void Send_Uart (uint8_t *str)
{
	xSemaphoreTake(SimpleMutex, portMAX_DELAY);
	HAL_Delay(5000);
	HAL_UART_Transmit(&huart2, str, strlen (str), HAL_MAX_DELAY);
	xSemaphoreGive(SimpleMutex);
}
*/
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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_4 ,GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  SimpleMutex=xSemaphoreCreateMutex();

  if(SimpleMutex!=NULL)
  {
  	  HAL_UART_Transmit(&huart2,(uint8_t *)"Mutex created\r\n\n",15,1000);
  }


  xTaskCreate(HPT_Task,"HPT",128,NULL,3,&HPT_Handler);
  /*xTaskCreate(MPT_Task,"MPT",128,NULL,2,&MPT_Handler);
  xTaskCreate(LPT_Task,"LPT",128,NULL,1,&LPT_Handler);
*/
  vTaskStartScheduler();
  /* USER CODE END 2 */



  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HPT_Task (void *argument)
{
	//uint8_t *strtosend = "IN HPT===========================\r\n";
	while (1)
	{


		if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			for (int i=0;i<500;i++)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
				HAL_Delay(1);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
				HAL_Delay(1);
			}
			HAL_Delay(2000);

			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);

			for (int i=0;i<500;i++)
						{
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
							HAL_Delay(1);
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
							HAL_Delay(1);
						}


		}
		//xSemaphoreGive(SimpleMutex);(let 'r' to give semaphore)

		vTaskDelay(750);
	}
}

/*void MPT_Task(void *argument)
{
	while(1)
	{
		char *str = "IN MPT******************************\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
		vTaskDelay(2000);
	}
}*/

/*void LPT_Task (void *argument)
{
	//uint8_t *strtosend = "IN LPT===================================\r\n";
	while (1)
	{
		char *str = "Entered LPT and About to take Mutex\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

		//Send_Uart(strtosend);

		char *str2 = "Leaving LPT\r\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);

		vTaskDelay(1000);
	}
}*/


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	HAL_UART_Receive_IT(huart, &rx_data, 1);
//	if (rx_data == 'r')
//	{
//		// release the semaphore here
//		 /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
//		 it will get set to pdTRUE inside the interrupt safe API function if a
//		 context switch is required. */
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//		xSemaphoreGiveFromISR(SimpleMutex, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION
//		//xSemaphoreGiveFromISR(CountingSem, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION
//		//xSemaphoreGiveFromISR(CountingSem, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION
//
//		/* Pass the xHigherPriorityTaskWoken value into portEND_SWITCHING_ISR(). If
//		 xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
//		 then calling portEND_SWITCHING_ISR() will request a context switch. If
//		 xHigherPriorityTaskWoken is still pdFALSE then calling
//		 portEND_SWITCHING_ISR() will have no effect */
//
//		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//	}
//}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
