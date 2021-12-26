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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId CANTaskHandle;
osThreadId HPTHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartCANTask(void const * argument);
void StartHPT(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//==================CAN==========================================================================================================
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
uint8_t TxData[8] = {0x01, 0x1A, 0x03, 0x03, 0x02, 0x00, 0x00, 0x00};
uint8_t RxData[8];

uint16_t CAN_ID = 0x001;//這塊板子的ID
int datacheck = 0;
int CAN_Neopixel = 0;
int CAN_Motor = 0;
int MOTOR_GO = 0;
//int count=0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		  {
			  //HAL_UART_Transmit(&huart2, (uint8_t *)"Receiving Data Failed\r\n\r\n", 24, 1000);
		      Error_Handler();
		  }
	 int error_message = 0;
		  if (RxHeader.StdId == CAN_ID && RxData[1] == 0x1A) //接收到資料(一對一)
		  {
			  if(RxData[2] == 0x03 && RxData[3] >= 0x00 && RxData[3] <= 0x04 && RxData[4] >= 0x00 && RxData[4] <= 0x07)//檢查第一組資料是否正確
			  {
				  CAN_Neopixel = 1;
			  }
			  else if(RxData[2] == 0x02 && RxData[3] == 0x00 && RxData[4] >= 0x00 && RxData[4] <= 0x01)//檢查第一組資料(抽屜門)
			  {
				  CAN_Motor = 1;
			  }
			  else if(RxData[2] == 0x00 && RxData[3] == 0x00 && RxData[4] == 0x00)
			  {
			  }
			  else
			  {
				  error_message++;
			  }
			  if(RxData[5] == 0x03 && RxData[6] >= 0x00 && RxData[6] <= 0x04 && RxData[7] >= 0x00 && RxData[7] <= 0x07)//檢查第一組資料是否正確
			  {
				  CAN_Neopixel = 2;
			  }
			  else if(RxData[5] == 0x02 && RxData[6] == 0x00 && RxData[7] >= 0x00 && RxData[7] <= 0x01)//檢查第一組資料(抽屜門)
			  {
				  CAN_Motor = 2;
			  }
			  else if(RxData[5] == 0x00 && RxData[6] == 0x00 && RxData[7] == 0x00)
			  {
			  }
			  else
			  {
				  error_message++;
			  }
			  TxHeader.StdId = 0x010;
			  for(int i=0; i<8; i++)
			  {
				  TxData[i]=RxData[i];
			  }
			  if(error_message>0){
				  TxData[1]=0x1C;
			  }
			  else
			  {
				  TxData[1]=0x1B;
			  }
//			  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//			  {
//				 // HAL_UART_Transmit(&huart2, (uint8_t *)"TxBackError\r\n\r\n", 15, 1000);
//			     Error_Handler ();
//			  }
		  }

}




void weidelay(int index)
{
	for (int i=0;i<index;i++)
	{

	}
}

//====================NeoPixel===================
#define MAX_LED 39
#define USE_BRIGHTNESS 1
int datasentflag=0;
uint8_t LED_Data[MAX_LED][5];
uint8_t LED_Mod[MAX_LED][5];  // for brightness
uint16_t pwmData[(32*MAX_LED)+50];
int ledcolor=1;
int ledcolor_buffer=1;
//int ledcolor_table[8][3]={{0,0,0},{255,255,255},{251,192,45},{249,168,37},{255,152,0},{255,0,255},{255,241,118},{0,255,255}};
int ledcolor_table[8][3]={{0,0,0},{255,255,255},{255,128,128},{0,255,0},{0,0,255},{255,0,255},{255,152,0},{0,255,255}};
//{255,152,0}yellow

int ledmode=1;

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
	LED_Data[LEDnum][4] = 0;
}

void Set_Brightness (int brightness)  // 0-40
{
#if USE_BRIGHTNESS

	if (brightness > 40) brightness = 40;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<5; j++)
		{

			LED_Mod[i][j] = brightness!=40?LED_Data[i][j]*brightness/40:LED_Data[i][j];
		}
	}
#endif
}

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
		#if USE_BRIGHTNESS
				color = ((LED_Mod[i][1]<<24) | (LED_Mod[i][2]<<16) | (LED_Mod[i][3]<<8));
		#else
				color = ((LED_Data[i][1]<<24) | (LED_Data[i][2]<<16) | (LED_Data[i][3]<<8));
		#endif

		for (int j=31; j>=0; j--)
		{
			if (color&(1<<j))pwmData[indx] = 57;//    64/100
			else pwmData[indx] = 25;//   32/100
			indx++;
		}
	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
	datasentflag=1;
}

//====================NeoPixel===================
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  TxHeader.DLC = 8;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x001;
  TxHeader.TransmitGlobalTime = DISABLE;
  //TxData[0] = 0xf3;


  	/*HAL_Delay(200);
  	HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
  	HAL_Delay(200);
  	HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
  	HAL_Delay(200);
  	HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
  	HAL_Delay(200);
  	HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);*/

  	for(int i=0;i<MAX_LED;i++)
  	{
  	Set_LED(i,255,255,0);//rgb
  	}
  	Set_Brightness(20);//0~45
  	WS2812_Send();
  	HAL_Delay(200);
  	Set_Brightness(0);//0~45
  	WS2812_Send();
  	HAL_Delay(200);
  	Set_Brightness(20);//0~45
  	WS2812_Send();
  	HAL_Delay(200);
  	Set_Brightness(0);//0~45
  	WS2812_Send();
  	HAL_Delay(200);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of CANTask */
  osThreadDef(CANTask, StartCANTask, osPriorityNormal, 0, 128);
  CANTaskHandle = osThreadCreate(osThread(CANTask), NULL);

  /* definition and creation of HPT */
  osThreadDef(HPT, StartHPT, osPriorityAboveNormal, 0, 128);
  HPTHandle = osThreadCreate(osThread(HPT), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  /* USER CODE END CAN_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 90-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_LED_Pin */
  GPIO_InitStruct.Pin = BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	 /* Infinite loop */
	  for(;;)
	  {
		  if (CAN_Motor==1 || CAN_Motor==2)
		  {
			  if (RxData[CAN_Motor*3+1] == 0x01)		//open door
			  {
				  HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
				  		  for (int i=0;i<35000;i++)
				  		  {
				  		  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
				  		  	weidelay(500);
				  		  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
				  		  	weidelay(500);
				  		  }
//				  MOTOR_GO=1;
//				  osDelay(3000);
//				  MOTOR_GO=0;
 			   CAN_Motor=70;




			  }
			  else if (RxData[CAN_Motor*3+1] == 0x00)		//close door
			  {
				  HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
				  	    while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5))
				  	    {
				  	    	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
				  			weidelay(500);
				  	    	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
				  			weidelay(500);
				  	    	if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5))
				  	    	{
				  	    		break;
				  	    	}
				  	    }
				  	  CAN_Motor=69;
			  }

		  }


	    osDelay(1);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void const * argument)
{
  /* USER CODE BEGIN StartCANTask */
	/* Infinite loop */
	  for(;;)
	  {

		  if(ledcolor_buffer!=ledcolor)
		  {
			  ledcolor_buffer=ledcolor;
			  for(int i=0;i<MAX_LED;i++)
			  {
				  Set_LED(i, ledcolor_table[ledcolor_buffer][0], ledcolor_table[ledcolor_buffer][1], ledcolor_table[ledcolor_buffer][2]);
			  }
		  }


		  if(CAN_Neopixel)
		  {
			  ledmode=RxData[3*CAN_Neopixel];//3 or 6
			  ledcolor=RxData[3*CAN_Neopixel+1];//4 or 7
			  CAN_Neopixel = 0;
		  }
	    osDelay(10);
	  }
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartHPT */
/**
* @brief Function implementing the HPT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHPT */
void StartHPT(void const * argument)
{
  /* USER CODE BEGIN StartHPT */
	int breath_bright = 0;
	int breath_u1d0 = 1;
	int blink_delay = 0;
		  for(int i=0;i<MAX_LED;i++)
		  {
			  Set_LED(i, ledcolor_table[ledcolor_buffer][0], ledcolor_table[ledcolor_buffer][1], ledcolor_table[ledcolor_buffer][2]);
		  }
		/* Infinite loop */
		for(;;)
		{
			if(ledmode <= 0x02)
			{
				  Set_Brightness(ledmode*20);//正常要20但亮度不明顯
				  blink_delay=0;
			}
			else if(ledmode == 0x03)
			{
				blink_delay=0;
				if(breath_u1d0)
				{
					breath_bright++;
					breath_u1d0=breath_bright<20;
				}
				else
				{
					breath_bright--;
					breath_u1d0=!(breath_bright>0);
				}
				Set_Brightness(breath_bright);
			}
			else if(ledmode == 0x04)
			{
				if(blink_delay==0)
				{
					Set_Brightness(20);
				}
				else if(blink_delay==5)
				{
					Set_Brightness(0);
				}
				blink_delay=blink_delay<10?blink_delay+1:0;
			}
				WS2812_Send();
				osDelay (50);
		}
  /* USER CODE END StartHPT */
}

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
