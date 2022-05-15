/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "SX1278.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define COM_GPS_LAT										10	// Send gps latitude
#define	COM_GPS_LON										11  // Send gps longitude
#define COM_GPS_HEI										12  // Send gps height
#define COM_GPS_SPE										13  // Send gps speed

#define COM_SNAKE										20  // OH BOI

#define COM_ECHO										50  // Test command, send back received data

#define COM_ACK 										100 // ack response
#define COM_NACK										101 // nack response
#define COM_BADCOM										254 // ERROR


#define MIN_POWER										SX1278_POWER_11DBM
#define MIN_PACKETLENGTH								8


#define CMD_ERROR										0
#define CMD_OK											1
#define CMD_SEND										2

#define cmd_rx_buffer_size 								100

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void CMD_Send(SX1278_t * module, uint8_t *buf, uint8_t len, uint8_t cmd);
uint8_t CMD_Parse(uint8_t *buf, uint8_t len);
void DataReceived(uint8_t *Buf, uint8_t Len);
uint8_t MODE_Check(uint8_t modeCheck);
void MODE_Set(SX1278_t * module, uint8_t mode);
void RTTY_Send(SX1278_t * module, uint8_t *buf, uint8_t len);
void RTTY_SendSingle(SX1278_t * module, uint8_t buf, uint8_t timeout);
uint8_t RTTY_Encoder(uint8_t *buf);
uint32_t gps_CRC32_checksum(char *string, uint8_t length);
void make_string(char *s, uint8_t size);
void make_string_gsm(char *s, uint8_t size);
uint32_t CalcCRC(uint8_t * pData, uint32_t DataLength);
//static uint32_t revbit(uint32_t uData);
//static uint8_t revbit_8(uint8_t uData);
unsigned char reverse_bits(unsigned char b);
uint8_t get_check_sum(char *string);
uint16_t Get_Temperature(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GSM_CTS_Pin GPIO_PIN_13
#define GSM_CTS_GPIO_Port GPIOC
#define GSM_GPIO1INT_Pin GPIO_PIN_14
#define GSM_GPIO1INT_GPIO_Port GPIOC
#define GSM_PWR_Pin GPIO_PIN_15
#define GSM_PWR_GPIO_Port GPIOC
#define GSM_TX_Pin GPIO_PIN_2
#define GSM_TX_GPIO_Port GPIOA
#define GSM_RX_Pin GPIO_PIN_3
#define GSM_RX_GPIO_Port GPIOA
#define RF_NSS_Pin GPIO_PIN_4
#define RF_NSS_GPIO_Port GPIOA
#define RF_SCK_Pin GPIO_PIN_5
#define RF_SCK_GPIO_Port GPIOA
#define RF_MISO_Pin GPIO_PIN_6
#define RF_MISO_GPIO_Port GPIOA
#define RF_MOSI_Pin GPIO_PIN_7
#define RF_MOSI_GPIO_Port GPIOA
#define LED_0_Pin GPIO_PIN_0
#define LED_0_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_10
#define LED_3_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_11
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_12
#define RX_GPIO_Port GPIOA
#define SW_IO_Pin GPIO_PIN_13
#define SW_IO_GPIO_Port GPIOA
#define SW_CLK_Pin GPIO_PIN_14
#define SW_CLK_GPIO_Port GPIOA
#define RF_DIO1_Pin GPIO_PIN_4
#define RF_DIO1_GPIO_Port GPIOB
#define RF_DIO0_Pin GPIO_PIN_5
#define RF_DIO0_GPIO_Port GPIOB
#define RF_DIO0_EXTI_IRQn EXTI9_5_IRQn
#define RF_RST_Pin GPIO_PIN_6
#define RF_RST_GPIO_Port GPIOB
#define GSM_RST_Pin GPIO_PIN_8
#define GSM_RST_GPIO_Port GPIOB
#define GSM_RTS_Pin GPIO_PIN_9
#define GSM_RTS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
