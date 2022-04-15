/*
 * lora.h
 *
 *  Created on: Sep 25, 2020
 *      Author: Kristers
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "SX1278.h"

void LORA_Send(SX1278_t * module, uint8_t *data, uint8_t len);
uint8_t LORA_Receive(SX1278_t * module, uint8_t *data, uint16_t timeout);

#endif /* INC_LORA_H_ */
