#ifndef INC_GSM_H_
#define INC_GSM_H_

#include "main.h"
#include <string.h>
#include <stdio.h>

void GSM_On();
void GSM_Off();

uint8_t GSM_InitUart(UART_HandleTypeDef *huart);
uint8_t GSM_Check_Signal();
uint8_t GSM_Message_Send(uint8_t *dataBuf, uint8_t size, uint32_t number);
uint8_t GSM_IsOk(uint8_t *dataBuf, uint8_t size);
uint8_t GSM_Find(uint8_t *dataBuf, uint8_t size, uint8_t *toFind, uint8_t tSize);
void GSM_Send(uint8_t *dataBuf, uint8_t size);
void GSM_Receive(uint8_t *dataBuf, uint8_t size);

#endif /* INC_GSM_H_ */
