/*
 * lora.c
 *
 *  Created on: Sep 25, 2020
 *      Author: Petera Dzive
 */

#include "lora.h"


/*
 * @brief Sends data trough LoRa module
 *
 * @Param data array with data to send
 * @Param len length of data to send
 */
void LORA_Send(SX1278_t * module, uint8_t *data, uint8_t len){
	uint8_t temp[255] = {0};
	temp[0] = len;
	memcpy(temp + 1, data, len + 1);
	SX1278_LoRaEntryTx(module, 8, 20000);
	HAL_Delay(1);
	//SX1278_LoRaTxPacket(module, (uint8_t *) temp, len + 1, 2000);
	SX1278_LoRaTxPacket(module, temp, 8, 20000);
}

/*
 * @brief Tries to receive data using LoRa
 *
 * @Param data pointer to data storage
 * @Param timeout specifies the delay time length, in milliseconds
 *
 * @Retval length of data received, 0 if timed out
 */
uint8_t LORA_Receive(SX1278_t * module, uint8_t *data, uint16_t timeout){
    uint8_t gotResponse = 0;
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = timeout;
	uint8_t tempBuf[255] = {0};

	/* Add a freq to guarantee minimum wait */
	if (wait < HAL_MAX_DELAY){
		wait += (uint32_t)(uwTickFreq);
	}

	/*while(!gotResponse){
		gotResponse = SX1278_LoRaRxPacket(module);
		if (gotResponse > 0) {
			SX1278_read(module, (uint8_t *) tempBuf, gotResponse);
			memcpy(data, tempBuf, gotResponse);
			return gotResponse;

		}
		if((HAL_GetTick() - tickstart) >= wait){
			break;								// timeout happened
		}
	}*/
	while(!gotResponse){
		gotResponse = SX1278_LoRaRxPacket(module);
		if (gotResponse > 0) {
			SX1278_read(module, tempBuf, gotResponse);
			memcpy(data, tempBuf, tempBuf[0]);
			return gotResponse;
		}
		if((HAL_GetTick() - tickstart) >= wait){
			break;								// timeout happened
		}
	}
	return gotResponse;
}
