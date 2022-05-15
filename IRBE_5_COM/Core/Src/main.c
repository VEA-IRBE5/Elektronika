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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "gsm.h"
#include "gps.h"
#include "lora.h"
#include "SX1278.h"

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
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
#define bit_set(val, bit_no) (((val) >> (bit_no)) & 1) // MACRO TO CHECH IF BIT IS SET
uint8_t uartRec = 0;
uint8_t rxBuf;
uint8_t cmd_tx_buffer[cmd_rx_buffer_size];
uint8_t cmd_rx_buffer[cmd_rx_buffer_size];
uint8_t LTRS = 0b11111;
uint8_t NMBR = 0b11011;
uint32_t num = 0;
           //LETTERS    A		  B        C        D        E        F         G	     H        I        J        K        L        M        N        O        P        Q        R        S        T        U        V        W        X        Y        Z        space    NULL     CR       LF
uint8_t RTTY_LTRS[30] = {0b11000, 0b10011, 0b01110, 0b10010, 0b10000, 0b100110, 0b01011, 0b00101, 0b01100, 0b11010, 0b11110, 0b01001, 0b00111, 0b00110, 0b00011, 0b01101, 0b11101, 0b01010, 0b10100, 0b00001, 0b11100, 0b01111, 0b11001, 0b10111, 0b10101, 0b10001, 0b00100, 0b00000, 0b00010, 0b01000};
			//NUMBERS   -         ?        :        $        3        !         &        #        8        '        (        )        .        ,        9        0        1        4        BELL     5        7        ;        2        /        6        "
			//			1		  2		   3		4		 5		  6			7		 8		  9		   0		-		 $		 ,		   &		#		 '		  (			)		"		/		:		   ;		?		 ,		  .


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// lora things

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

uint8_t vcomIvGPS = 1;
uint8_t vcomInGPS = 1;

uint8_t master = 1;
uint8_t ret = 0;

uint8_t loraBuf[10] = {0};
uint8_t buffer[15] = {0};
uint8_t message[] = "&&\"/1,2.3--mini--salamini--desinas--1.2,3/\"";
char info_message[8] = "##ready&";
char ok_cut_rope_message[8] = "#ok_rope";
char nok_ack_message[8] = "#nok_act";
char TEST_BUF[100] = {0};
uint8_t bufferLen = 0;
uint8_t mode = 1;
uint32_t recvd = 0; // data received
uint32_t recve = 0; // errors received

uint8_t loraModuleIrq = 0;

// lora things

//Timer things

uint8_t timePas = 0;
uint32_t u_sec_delay = 0;
uint8_t gsmBuf = 0;
uint8_t gsmRec = 0;
uint8_t do_send_tm = 1;
uint8_t doSendGSM = 0;
uint8_t receive_data = 0;

uint8_t sec_gps = 0;
uint8_t sec_lora_rec = 0;

//Timer things

//UART

uint8_t UART6_RxIsData = 0;
uint8_t UART6_TxBuf[2] = {0};
uint8_t UART6_DataBuf[50] = {0};
uint8_t UART6_RxBuf[50] = {0};
uint8_t UART6_RxBytes = 2;

//UART

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  SX1278_hw_t SX1278_hw;
  SX1278_t SX1278;

 //initialize LoRa module
 SX1278_hw.dio0.port = RF_DIO0_GPIO_Port;
 SX1278_hw.dio0.pin = RF_DIO0_Pin;
 SX1278_hw.nss.port = RF_NSS_GPIO_Port;
 SX1278_hw.nss.pin = RF_NSS_Pin;
 SX1278_hw.reset.port = RF_RST_GPIO_Port;
 SX1278_hw.reset.pin = RF_RST_Pin;
 SX1278_hw.spi = &hspi1;
 SX1278.hw = &SX1278_hw;

 //HAL_UART_Receive_DMA(&huart1, &rxBuf, 1); DOESN"T work for some reason
 HAL_UART_Receive_IT(&huart1, &rxBuf, 1); // Works like a charm, but not as good as DMA
 while(HAL_GPIO_ReadPin(RX_GPIO_Port, RX_Pin) == 0);
 HAL_UART_Receive_IT(&huart6, UART6_RxBuf, 2);

 //HAL_UART_Receive_IT(&huart2, UART6_RxBuf, 2);

 HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET); // very important.

 SX1278_hw_Reset(&SX1278_hw);
//      SX1278_begin(&SX1278, SX1278_433MHZ, MIN_POWER, SX1278_LORA_SF_8, // air time ~ 495ms
//      SX1278_LORA_BW_20_8KHZ, 10);

  	//uint8_t GSM_STATE = 0;

  	GSM_Off();

  	if(GSM_InitUart(&huart2)){ // if failed, then gg
  		return 0;
  	}

	MODE_Set(&SX1278, mode);
	if(mode == 0){
		ret = SX1278_LoRaEntryRx(&SX1278, MIN_PACKETLENGTH, 2000);
	}else{
		ret = SX1278_FSK_EntryRx(&SX1278, 8);
	}

	 //HAL_UART_Receive_DMA(&huart6, &cmd_rx_buffer, cmd_rx_buffer_size);
//	uint8_t check_sum;
//	uint8_t check_sum_arr[4] = {0, 0, 0, 0};

	uint8_t tel_dataBuf[80];
	uint8_t gsm_dataBuf[50];
	memset(tel_dataBuf, 0, sizeof(tel_dataBuf));

	//HAL_UART_Transmit_IT(&huart6, UART6_TxBuf, 2);

	while(GPS_IsData() == GPS_NOK);

	memset(UART6_RxBuf, 48, sizeof(UART6_RxBuf));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if(do_send_tm){ // its time to send gps coordinates
		 for(uint8_t tries = 0; tries < 5; tries++){
			 UART6_TxBuf[0] = 0x03;
			 UART6_TxBuf[1] = 0x99;
			 HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_SET);
			 HAL_UART_Transmit_IT(&huart6, UART6_TxBuf, 2);
			 make_string(tel_dataBuf, sizeof((char *)tel_dataBuf));
			 RTTY_Send(&SX1278, tel_dataBuf, strlen((char *)tel_dataBuf));
			 HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);
		 }
		 SX1278_FSK_TxPacket(&SX1278, info_message, 8, 100);
		 do_send_tm = 0;
		 receive_data = 1;
		 HAL_TIM_Base_Start_IT(&htim2);
	}
	 if(receive_data){
		if(sec_gps == 0){
			SX1278_FSK_EntryRx(&SX1278, 8);
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
		}else if(sec_gps >= 5){
			do_send_tm = 1;		// should send TM data
			receive_data = 0;
			sec_gps = 0;
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
			HAL_TIM_Base_Stop_IT(&htim2);
		}
		if(loraModuleIrq){
			SX1278_FSK_RxPacket(&SX1278, loraBuf, 8, 1000);
			if(strcmp((char *)loraBuf, "cutropeN") == 0){
				HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

				// SENDING COMMAND TO MCU TO CUT THE ROPE
				UART6_TxBuf[0] = 0x4f;
				UART6_TxBuf[1] = 0xcc;
				HAL_UART_Transmit_IT(&huart6, UART6_TxBuf, 2);
				// SENDING COMMAND TO MCU TO CUT THE ROPE

				SX1278_FSK_TxPacket(&SX1278, ok_cut_rope_message, 8, 100);
			}else{
				SX1278_FSK_TxPacket(&SX1278, nok_ack_message, 8, 100);
			}
			loraModuleIrq = 0;
		}
		if(gsmRec = 1 && GSM_STATE = HAL_GPIO_ReadPin(GSM_GPIO1INT_GPIO_Port, GSM_GPIO1INT_Pin)){
			GSM_On();

			make_string_gsm(gsm_dataBuf, sizeof(gsm_dataBuf));
			if(GSM_Check_Signal()){
				GSM_Message_Send(make_string_gsm, strlen((char *)make_string_gsm), 28654641);
			}

			GSM_Off();

			gsmRec = 0;
			HAL_TIM_Base_Start_IT(&htim4);
		}
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 32000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2625;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  htim3.Init.Prescaler = 84;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 32000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 13125;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 32000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 7875;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GSM_CTS_GPIO_Port, GSM_CTS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GSM_PWR_GPIO_Port, GSM_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_NSS_GPIO_Port, RF_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_0_Pin|LED_1_Pin|LED_2_Pin|LED_3_Pin
                          |RF_DIO1_Pin|GSM_RST_Pin|GSM_RTS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GSM_CTS_Pin GSM_PWR_Pin */
  GPIO_InitStruct.Pin = GSM_CTS_Pin|GSM_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_GPIO1INT_Pin */
  GPIO_InitStruct.Pin = GSM_GPIO1INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GSM_GPIO1INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_NSS_Pin */
  GPIO_InitStruct.Pin = RF_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_0_Pin LED_1_Pin LED_2_Pin LED_3_Pin
                           RF_DIO1_Pin GSM_RTS_Pin */
  GPIO_InitStruct.Pin = LED_0_Pin|LED_1_Pin|LED_2_Pin|LED_3_Pin
                          |RF_DIO1_Pin|GSM_RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_DIO0_Pin */
  GPIO_InitStruct.Pin = RF_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_RST_Pin */
  GPIO_InitStruct.Pin = RF_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_RST_Pin */
  GPIO_InitStruct.Pin = GSM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GSM_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void CMD_Send(SX1278_t * module, uint8_t *buf, uint8_t len, uint8_t cmd){
	uint8_t temp[13] = {0};
	temp[0] = cmd;
	memcpy(temp + 1, buf, len);

	if(mode == 0){
		LORA_Send(module, temp, len);
		SX1278_LoRaEntryRx(module, MIN_PACKETLENGTH, 2000);
		HAL_Delay(10);
	}else{
		SX1278_FSK_TxPacket(module, temp, len, 100);
		SX1278_FSK_EntryRx(module, MIN_PACKETLENGTH);
		HAL_Delay(10);
	}
}

uint8_t CMD_Parse(uint8_t *buf, uint8_t len){
	uint8_t cmd = buf[0];
	if(cmd == COM_ECHO){					// echo mode resend whatever received
		memcpy(buffer, buf, ret);
		buf[3] = buf[3] + 1;
		return CMD_SEND;
	}else if(cmd == COM_SNAKE){		// mode change command change to whatever mode requested
		for(uint8_t i = 0; i < 14; i++){
			buf[1 + i] = i;
		}
		MODE_Set(&SX1278, mode);
		return CMD_OK;
	}else{
		return CMD_ERROR;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){
		HAL_UART_Receive_IT(&huart1, &rxBuf, 1);
		GPS_Receive(rxBuf);
		//HAL_UART_Receive_DMA(&huart1, &rxBuf, 1);
	}

	if(huart == &huart6)
	{
		if(UART6_RxIsData == 1){
			for (uint8_t i = 0; i < UART6_RxBytes; i++)
				UART6_DataBuf[i] = UART6_RxBuf[i];
			UART6_RxIsData = 0;
			UART6_RxBytes = 2;
		}else{
			uint8_t Command = UART6_RxBuf[0];
			uint8_t Parameter = UART6_RxBuf[1];

			if(Command != 0x02){
				UART6_RxBytes = 2;
				HAL_UART_Receive_IT(&huart6, UART6_RxBuf, 2);
			}
			switch(Command){
				// Receive data from MCU
				case 0x02:
					UART6_RxIsData = 1;
					UART6_RxBytes = Parameter;
					HAL_UART_Receive_IT(&huart6, UART6_RxBuf, Parameter);
				break;
				// Send data to MCU
				case 0x03:
					UART6_TxBuf[0] = 0x02;
					UART6_TxBuf[1] = Parameter;
					memcpy(&(UART6_TxBuf[2]), UART6_DataBuf, strlen(UART6_DataBuf) + 1);
					HAL_UART_Transmit_IT(&huart6, UART6_TxBuf, strlen(UART6_TxBuf) + 1);
				break;
				default:
					//nothing happens
					//HAL_UART_Receive_IT(&huart6, UART6_RxBuf, UART6_RxBytes);
				break;
			}
		}
		memset(UART6_RxBuf, 48, sizeof(UART6_RxBuf));
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	//HAL_UART_Receive_DMA(&huart1, &rxBuf, 1); doesn't work for some reason...
	if(huart == &huart1){
		HAL_UART_Receive_IT(&huart1, &rxBuf, 1);
	}
	if(huart == &huart6){
		HAL_UART_Receive_IT(&huart6, UART6_RxBuf, UART6_RxBytes);
	}
}

void MODE_Set(SX1278_t * module, uint8_t mode){
	  switch(mode){
		  case 0:	// switch to LoRa
			  SX1278_begin(module, SX1278_433MHZ, MIN_POWER, SX1278_LORA_SF_8, // air time ~ 495ms
					  SX1278_LORA_BW_20_8KHZ, MIN_PACKETLENGTH);
		  break;

		  case 1:	// switch to FSK
			  SX1278_FSK_Config(module);
		  break;

		  default: // lets ignore that one
		  break;
	  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		sec_gps++;
	}

	if(htim->Instance == TIM3){
		u_sec_delay = 1;
	}

	if(htim->Instance == TIM4){
		HAL_TIM_Base_Stop_IT(&htim4);
		HAL_TIM_Base_Start_IT(&htim5);
	}
	if(htim->Instance == TIM5){
		HAL_TIM_Base_Stop_IT(&htim5);
		gsmRec = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t pin){
	loraModuleIrq = 1;
}

void RTTY_Send(SX1278_t * module, uint8_t *buf, uint8_t len){
	uint16_t baudTimeout = 20;
	uint8_t i;
	uint8_t curChar = 0;

	SX1278_RTTY_Config(module);

	for(i = 0; i < len; i++){
		curChar = buf[i];
		RTTY_SendSingle(module, curChar, baudTimeout);
	}
}

void RTTY_SendSingle(SX1278_t * module, uint8_t buf, uint8_t timeout){
	SX1278_RTTY_WriteLow(module); //start bit
	HAL_Delay(timeout);
	for(uint8_t j = 0; j < 8; j++){
		if(bit_set(buf, j)){
			SX1278_RTTY_WriteHigh(module);
		}else{
			SX1278_RTTY_WriteLow(module);
		}
		HAL_Delay(timeout);
	}
	SX1278_RTTY_WriteHigh(module); // stop bit
	HAL_Delay(30);
	SX1278_RTTY_Stop(module);
}

uint8_t get_check_sum(char *string){
	uint8_t XOR = 0;
	for(uint8_t i = 2; i < strlen(string) ; i++) {
		XOR = XOR ^ string[i];
	}
	return XOR;
}

void make_string(uint8_t *s, uint8_t size){

	uint8_t time[9];
	uint8_t lat[10];
	uint8_t lon[10];
	uint8_t hei[9];
	uint8_t spe[7];
	//CLEAR TEMP BUFFERS (SOMETIMES IT HAS INFORMATION IN IT, BECAUSE IT USES MEMORY LOCATION THAT WERE TEMP USED FOR OTHER STUFF) ONLY WHEN THERE IS +1 elemt in array
	memset(time, 0, sizeof(time));
	memset(lat, 0, sizeof(lat));
	memset(lon, 0, sizeof(lon));
	memset(hei, 0, sizeof(hei));
	memset(spe, 0, sizeof(spe));

	//Get all params from satalites data
	GPS_GetTime(time);
	GPS_GetLat(lat);
	GPS_GetLon(lon);
	GPS_GetHei(hei);
	GPS_GetSpe(spe);

	snprintf(s, size, "\r\n$$IRBE5,%li,%s,%s,%s,%s,%s%s", ++num, time, lat, lon, hei, spe, &(UART6_DataBuf[1]));
	uint8_t l = strlen(s);
	if(snprintf(s + l, size - l, "*%02x\r\n", get_check_sum(s))  > size - 4 - 1){
		//buffer overflow
		return;
	}
}

void make_string_gsm(uint8_t *s, uint8_t size){

	uint8_t lat[10];
	uint8_t lon[10];
	uint8_t hei[9];
	//CLEAR TEMP BUFFERS (SOMETIMES IT HAS INFORMATION IN IT, BECAUSE IT USES MEMORY LOCATION THAT WERE TEMP USED FOR OTHER STUFF) ONLY WHEN THERE IS +1 elemt in array
	memset(lat, 0, sizeof(lat));
	memset(lon, 0, sizeof(lon));
	memset(hei, 0, sizeof(hei));

	//Get all params from satalites data
	GPS_GetLat(lat);
	GPS_GetLon(lon);
	GPS_GetHei(hei);

	snprintf(s, size, "Latitude:%s\nLongitude:%s\nHeight ASL:%s",lat, lon, hei);
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

