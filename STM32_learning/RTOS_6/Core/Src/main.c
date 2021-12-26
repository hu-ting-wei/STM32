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
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

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


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**QueueHandler**/
xQueueHandle St_Queue_Handler;

/**TaskHandler**/
xTaskHandle Sender1_Task_Handler;
xTaskHandle Sender2_Task_Handler;
xTaskHandle Receiver_Task_Handler;

void Sender1_Task(void *argument);
void Sender2_Task(void *argument);
void Receiver_Task(void *argument);
/**************** STRUCTURE DEFINITION *****************/

typedef struct {
	char *str;
	int counter;
	uint16_t large_value;
} my_struct;


int indx1=0;
int indx2=0;

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
  /***** create QUEUE *****/
   St_Queue_Handler = xQueueCreate(2, sizeof (my_struct));

   if (St_Queue_Handler == 0) // if there is some error while creating queue
   {
  	  char *str = "Unable to create STRUCTURE Queue\r\n\n";
  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
    }
   else
    {
  	  char *str = "STRUCTURE Queue Created successfully\r\n\n";
  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
    }



   xTaskCreate(Sender1_Task,"Sender1",128,NULL,2,&Sender1_Task_Handler);
   xTaskCreate(Sender2_Task,"Sender2",128,NULL,2,&Sender2_Task_Handler);
   xTaskCreate(Receiver_Task,"Receiver",128,NULL,1,&Receiver_Task_Handler);


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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void Sender1_Task (void *argument)
{
	my_struct *ptrtostruct;

	uint32_t TickDelay = pdMS_TO_TICKS(2000);
	while (1)
	{
		char *str = "Entered SENDER1_Task\r\n about to SEND to the queue\r\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

		/****** ALOOCATE MEMORY TO THE PTR ********/
		ptrtostruct = pvPortMalloc(sizeof (my_struct));

		/********** LOAD THE DATA ***********/
		ptrtostruct->counter = 1+indx1;
		ptrtostruct->large_value = 1000 + indx1*100;
		ptrtostruct->str = "HELLO FROM SENDER 1 ";

		/***** send to the queue ****/
		if (xQueueSend(St_Queue_Handler, &ptrtostruct, portMAX_DELAY) == pdPASS)
		{
			char *str2 = " Successfully sent the to the queue\r\nLeaving SENDER1_Task\r\n\n\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
		}

		indx1 = indx1+1;

		vTaskDelay(TickDelay);
	}
}



void Sender2_Task (void *argument)
{
	my_struct *ptrtostruct;

	uint32_t TickDelay = pdMS_TO_TICKS(2000);
	while (1)
	{
		char *str = "Entered SENDER2_Task\r\n about to SEND to the queue\r\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

		/****** ALOOCATE MEMORY TO THE PTR ********/
		ptrtostruct = pvPortMalloc(sizeof (my_struct));

		/********** LOAD THE DATA ***********/
		ptrtostruct->counter = 1+indx1;
		ptrtostruct->large_value = 1000 + indx1*100;
		ptrtostruct->str = "HELLO FROM SENDER 2 ";

		/***** send to the queue ****/
		if (xQueueSend(St_Queue_Handler, &ptrtostruct, portMAX_DELAY) == pdPASS)
		{
			char *str2 = " Successfully sent the to the queue\r\nLeaving SENDER2_Task\r\n\n\n";
			HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
		}

		indx1 = indx1+1;

		vTaskDelay(TickDelay);
	}
}

void Receiver_Task (void *argument)
{
	my_struct *Rptrtostruct;
	uint32_t TickDelay = pdMS_TO_TICKS(3000);
	char *ptr;

	while (1)
	{
		char *str = "Entered RECEIVER Task\r\n about to RECEIVE FROM the queue\r\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

		/**** RECEIVE FROM QUEUE *****/
		if (xQueueReceive(St_Queue_Handler, &Rptrtostruct, portMAX_DELAY) == pdPASS)
		{
			ptr = pvPortMalloc(100 * sizeof (char)); // allocate memory for the string

			sprintf (ptr, "Received from QUEUE:\n COUNTER = %d\r\n LARGE VALUE = %u\n STRING = %s\r\n\n\n",Rptrtostruct->counter,Rptrtostruct->large_value, Rptrtostruct->str);
			HAL_UART_Transmit(&huart2, (uint8_t *)ptr, strlen(ptr), HAL_MAX_DELAY);

			vPortFree(ptr);  // free the string memory
		}

		vPortFree(Rptrtostruct);  // free the structure memory

		vTaskDelay(TickDelay);
	}
}
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
