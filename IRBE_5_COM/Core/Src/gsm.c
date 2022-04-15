/*
 *	GSM library made for A6 thinkerer (might work for other modules)
 * 	Made for sending sms only
 */

#include "gsm.h"

UART_HandleTypeDef *uart;

#define GSM_MSG_END											13				// carriage return
#define GSM_MODE_TEXT										"AT+CMGF=1"
#define GSM_SET_NUMBER										"AT+CMGS=\""
#define GSM_MSG_SEND										26				//CTRL + Z
//#define GSM_MSG_SPACE										32				//SPACE


/* pass UART handle that will communicate with GSM module*/
uint8_t GSM_InitUart(UART_HandleTypeDef *huart){
	uart = huart;
	if(huart != uart){
		return HAL_ERROR;
	}else{
		return HAL_OK;
	}
}

uint8_t GSM_Check_Signal(){
	uint8_t buf[32] = {0};
	buf[0] = 'A';
	buf[1] = 'T';

	GSM_Send(buf, 2);
	GSM_Receive(buf, 9);

	if(GSM_IsOk(buf, 9)){
		return 1;
	}

	return 0;
}

uint8_t GSM_IsOk(uint8_t *dataBuf, uint8_t size){

	uint8_t i = 0;

	for(; i < size; i++){
		if(*dataBuf == 'O'){
			dataBuf++;
			if(*dataBuf == 'K'){
				return 1;
			}
		}
		dataBuf++;
	}
	return 0;
}

uint8_t GSM_Find(uint8_t *dataBuf, uint8_t size, uint8_t *toFind, uint8_t tSize){
	uint8_t i = 0;
	uint8_t f = 0;
	uint8_t temp[16];

	memcpy(temp, toFind, tSize);

	for(; i < size; i++){

		if(*dataBuf == temp[f]){
			f++;
		}else{
			f = 0;
		}

		if(f == tSize){
			return 1;
		}
		dataBuf++;
	}

	return 0;
}

void GSM_On(){
	HAL_GPIO_WritePin(GSM_PWR_GPIO_Port, GSM_PWR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GSM_RST_GPIO_Port, GSM_RST_Pin ,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GSM_RTS_GPIO_Port, GSM_RTS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GSM_CTS_GPIO_Port, GSM_CTS_Pin, GPIO_PIN_RESET);
}

void GSM_Off(){
	HAL_GPIO_WritePin(GSM_PWR_GPIO_Port, GSM_PWR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GSM_RST_GPIO_Port, GSM_RST_Pin ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GSM_RTS_GPIO_Port, GSM_RTS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GSM_CTS_GPIO_Port, GSM_CTS_Pin, GPIO_PIN_SET);
}

uint8_t GSM_Message_Send(uint8_t *dataBuf, uint8_t size, uint32_t number){
	uint8_t temp[256];

	if(GSM_Check_Signal() == 0){
		return 0;
	}

	memcpy(temp, GSM_MODE_TEXT, 9);
	GSM_Send(temp, 9);
	GSM_Receive(temp, 20);
	if(GSM_IsOk(temp, 20) == 0){
		return 0;
	}

	memcpy(temp, GSM_SET_NUMBER, 9);
	sprintf((char *)(temp + 9), "%lu", number);
	memcpy(temp + 17, "\"", 1);
	GSM_Send(temp, 17);
	GSM_Receive(temp, 40);
	if(GSM_Find(temp, 40, (uint8_t *)">", 1) == 0){
		return 0;
	}

	temp[0] = GSM_MSG_SEND;
	GSM_Send(dataBuf, size);
	GSM_Send(temp, 1);


	HAL_UART_Receive(uart, temp, 255, 100);
	HAL_UART_Receive(uart, temp, 17, 5000);
	if(GSM_IsOk(temp, size + 40) == 0){
		return 0;
	}

	return 1;
}

void GSM_Send(uint8_t *dataBuf, uint8_t size){
	uint8_t temp = GSM_MSG_END;
	HAL_UART_Transmit(uart, dataBuf, size, 50);
	HAL_UART_Transmit(uart, &temp, 1, 50);
}

void GSM_Receive(uint8_t *dataBuf, uint8_t size){
	HAL_UART_Receive(uart, dataBuf, size, 100);
}
