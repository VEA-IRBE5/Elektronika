#include "gps.h"

uint8_t gpsData[24] = {0}; //0 coord latitude //1 coord longitude "05723.06487N", "02132.70887E"
uint8_t gpsHeight[8] = {0}; //2 height
uint8_t gpsSpeed[6] = {0}; //3 speed
uint8_t gpsTime[6] = {0}; // UTC time from GPGGA
uint8_t gpsTime_UTC[9] = {0}; // UTC time from GPZDA
uint8_t gps_date[3] = {0}; // date from GPZDA
uint8_t gps_month[3] = {0}; // month from GPZDA
uint8_t gps_year[5] = {0}; // year from GPZDA
uint8_t local_zone_desc[3] = {0}; // DON"T KNOW
uint8_t local_zone_min_desc[3] = {0}; // DON"T KNOW

uint8_t gpsTemp[80];
uint8_t gpsTempLen = 255;

uint8_t isData = 0;
uint8_t isNewData = 0;

UART_HandleTypeDef *GPS_uart;

/* pass UART handle that will communicate with GPS module*/
uint8_t GPS_init_Uart(UART_HandleTypeDef *huart){
	GPS_uart = huart;
	if(huart != GPS_uart){
		return HAL_ERROR;
	}else{
		return HAL_OK;
	}
}


void GPS_init_baudrate(uint8_t port, uint8_t inProto, uint8_t outProto, uint32_t baud_rate, uint8_t autobaud_ing){

	//"PUBX,41,1,7,3,38400,0*xxCRLF"
	char temp[60];
	char command[65];
	uint16_t size = sprintf (temp, "PUBX,41,%04X,%04X,%lu,%d", inProto, outProto, baud_rate,autobaud_ing);
	uint8_t checksum = GPS_message_checksum(temp, size);
	size = sprintf(command, "%s*%02x\r\n", temp, checksum);
	HAL_UART_Transmit_IT(GPS_uart, (uint8_t *)command, size);

}

void Set_Navigation_Engine_To_Airborne(void){
	uint8_t ubx_cfg_nav5[] = {
	0xB5,0x62,0x06,0x24,0x05,0x00,Airborne_2,
	0x03,0x00,0x06,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00};
	size_t buffer_size = sizeof(ubx_cfg_nav5)/sizeof(ubx_cfg_nav5[0]);
	size_t buffer_pos = strlen((char *)ubx_cfg_nav5);
	uint16_t temp = Fletcher16(ubx_cfg_nav5, strlen((char *)ubx_cfg_nav5));
	uint8_t temp_buff[2];
	temp_buff[0] = temp;
	temp_buff[1] = (temp >> 8);
	for(uint8_t i = 0; i < 2; i++)
		buffer_pos += write_to_buffer(&ubx_cfg_nav5[buffer_pos], buffer_size - buffer_pos, temp_buff);
	HAL_UART_Transmit_IT(GPS_uart, ubx_cfg_nav5, sizeof(ubx_cfg_nav5));
}

uint16_t Fletcher16(uint8_t *data, uint8_t size){
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;
   for(uint8_t index = 0; index < size; ++index){
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}

size_t write_to_buffer(char *buffer, size_t size, uint8_t data){
	return snprintf(buffer, size++, "%x,", data);
}

/* Pass uint8_t of received data */
void GPS_Receive(uint8_t data){
	if(data == '$'){
		gpsTempLen = 0;
	}else if((data == 13 || data == 10) && gpsTempLen != 255){ // looks for new_line or vertical tab
		GPS_Parse(gpsTemp, gpsTempLen);
		gpsTempLen = 255;
	}else if(gpsTempLen != 255){
		gpsTemp[gpsTempLen] = data;
		gpsTempLen++;
	}
}

/* parses received frame */
uint8_t GPS_Parse(uint8_t *buf, uint8_t len){
	if(strncmp("GPGGA", (char *)buf, 5) == 0){ // get coordinates and height
		if(GPS_CheckSum(buf, len) == GPS_OK){
			uint8_t step = 0;
			uint8_t i = 0;
			while(step < 8){
				if(buf[i] == ','){
					i++;
					step++;
					if(step == 1){
						uint8_t tempData[6] = {0};
						uint8_t leng = 0;
						while(buf[i] != '.'){
							tempData[leng] = buf[i];
							leng++;
							i++;
						}
						if(leng == 0){
							return GPS_NOK;
						}
						memset(gpsTime, '0', 6);
						memcpy(gpsTime + (6-leng), tempData, leng);
					}else if(step == 2 || step == 3){
						uint8_t tempData[12] = {0};
						uint8_t leng = 0;
						while(buf[i] != ','){
							tempData[leng] = buf[i];
							leng++;
							i++;
						}
						if(leng == 0){
							return GPS_NOK;
						}
						i++;
						if(buf[i] == 'N' || buf[i] == 'S' || buf[i] == 'E' || buf[i] == 'W'){
							memset((gpsData + (step - 2) * 12), '0', 12);
							gpsData[(step - 2) * 12 + 11] = buf[i];
						}else{
							return GPS_NOK;
						}
						memcpy(gpsData + (step - 2) * 12 + (11 - leng), tempData, leng);
					}else if(step == 7){
						i++;
						uint8_t tempData[12] = {0};
						uint8_t leng = 0;
						while(buf[i] != ','){
							tempData[leng] = buf[i];
							leng++;
							i++;
						}
						if(leng == 0){
							return GPS_NOK;
						}
						memset(gpsHeight, '0', 8);
						memcpy(gpsHeight + (8-leng), tempData, leng);
					}
				}
				i++;
			}
			isNewData = 1;
			isData = 1;
			return GPS_OK;
		}else{
			return GPS_NOK;
		}


	}else if(strncmp("GPVTG", (char *)buf, 5) == 0){ // get speed in km/h
		if(GPS_CheckSum(buf, len) == GPS_OK){
			uint8_t step = 0;
			uint8_t i = 0;
			while(step < 8){
				if(buf[i] == ','){
					step++;
					if(step == 7){
						i++;
						uint8_t tempData[12] = {0};
						uint8_t leng = 0;
						while(buf[i] != ','){
							tempData[leng] = buf[i];
							leng++;
							i++;
						}
						if(leng == 0){
							return GPS_NOK;
						}
						memset(gpsSpeed, '0', 6);
						memcpy(gpsSpeed + (6-leng), tempData, leng);
					}
				}
				i++;
			}
			return GPS_OK;
		}else{
			return GPS_NOK;
		}
	}else if(strncmp("GPZDA", (char *)buf, 5) == 0){ // get precise time
		if(GPS_CheckSum(buf, len) == GPS_OK){
				uint8_t step = 0;
				uint8_t i = 0;
				while(step < 6){
					if(buf[i] == ','){
						step++;
						if(step == 1){
							uint8_t tempData[8] = {0};
							uint8_t leng = 0;
							while(buf[i] != ','){
								tempData[leng] = buf[i];
								leng++;
								i++;
							}
							if(leng == 0){
								return GPS_NOK;
							}
							memset(gpsTime_UTC, '0', 9);
							memcpy(gpsTime_UTC + (8-leng), tempData, leng);
						}else if(step == 2){
							uint8_t tempData[2] = {0};
							uint8_t leng = 0;
							while(buf[i] != ','){
								tempData[leng] = buf[i];
								leng++;
								i++;
							}
							if(leng == 0){
								return GPS_NOK;
							}
							memset(gps_date, '0', 3);
							memcpy(gps_date + (2-leng), tempData, leng);
						}else if(step == 3){
							uint8_t tempData[2] = {0};
							uint8_t leng = 0;
							while(buf[i] != ','){
								tempData[leng] = buf[i];
								leng++;
								i++;
							}
							if(leng == 0){
								return GPS_NOK;
							}
							memset(gps_month, '0', 3);
							memcpy(gps_month + (2-leng), tempData, leng);
						}else if(step == 4){
							uint8_t tempData[4] = {0};
							uint8_t leng = 0;
							while(buf[i] != ','){
								tempData[leng] = buf[i];
								leng++;
								i++;
							}
							if(leng == 0){
								return GPS_NOK;
							}
							memset(gps_year, '0', 5);
							memcpy(gps_year + (4-leng), tempData, leng);
						}else if(step == 5){
							uint8_t tempData[2] = {0};
							uint8_t leng = 0;
							while(buf[i] != ','){
								tempData[leng] = buf[i];
								leng++;
								i++;
							}
							if(leng == 0){
								return GPS_NOK;
							}
							memset(local_zone_desc, '0', 3);
							memcpy(local_zone_desc + (2-leng), tempData, leng);
						}
						else if(step == 5){
							uint8_t tempData[2] = {0};
							uint8_t leng = 0;
							while(buf[i] != ','){
								tempData[leng] = buf[i];
								leng++;
								i++;
							}
							if(leng == 0){
								return GPS_NOK;
							}
							memset(local_zone_min_desc, '0', 3);
							memcpy(local_zone_min_desc + (2-leng), tempData, leng);
						}
					}
					i++;
				}
			return GPS_OK;
		}else{
			return GPS_NOK;
		}
	}else{
		return GPS_NOK;
	}
	return GPS_NOK;
}

/* checks checksum returns GPS_ok if data is valid*/
uint8_t GPS_CheckSum(uint8_t *buf, uint8_t len){
	uint8_t sum;
	uint8_t check = 0;

	if(GPS_HexToByte((buf + len - 2), &sum) != GPS_OK){
		return GPS_NOK;
	}

	for(uint8_t i = 0; i < (len - 3); i++){
		check ^= buf[i];
	}

	if(check == sum){
		return GPS_OK;
	}

	return GPS_NOK;

}

uint8_t GPS_message_checksum(char *message, uint16_t size){
	uint8_t checksum = 0;
	for(uint8_t i = 0; i < size; i++){
		checksum ^= message[i];
	}
	 return checksum;
}

/* converts hex string e.g. AB (0xAB) to byte value writes to value, returns GPS_OK if valid hex */
uint8_t GPS_HexToByte(uint8_t *hex, uint8_t *value){
	uint8_t temp[2];

	for(uint8_t i = 0; i < 2; i++){
		temp[i] = hex[i];
		if(temp[i] > 47 && temp[i] < 58){
			temp[i] -= 48;
		}else if(temp[i] > 64 && temp[i] < 71){
			temp[i] -= 55;
		}else{
			return GPS_NOK;
		}
	}

	*value = (temp[0] << 4) + temp[1];

	return GPS_OK;
}

/* Returns latitude of length 12...9*/
void GPS_GetLat(uint8_t *buf){
	isNewData = 0;
	for(uint8_t i = 1; i < 10; i++){
		buf[i - 1] = gpsData[i];
	}
}

/* Returns longitude of length 12...9*/
void GPS_GetLon(uint8_t *buf){
	isNewData = 0;
	for(uint8_t i = 1; i < 10; i++){
		buf[i - 1] = gpsData[i + 12];
	}
}

/* Returns speed of length 6*/
void GPS_GetSpe(uint8_t *buf){
	isNewData = 0;
	for(uint8_t i = 0; i < 6; i++){
		buf[i] = gpsSpeed[i];
	}
}

/* Returns height of length 8*/
void GPS_GetHei(uint8_t *buf){
	isNewData = 0;
	uint8_t i = 0;
	for(; i < 8; i++){
		buf[i] = gpsHeight[i];
	}
}

/* Returns time of length 9 in format "hh:mm:ss.msms" */
void GPS_GetTime(uint8_t *buf){
	isNewData = 0;
	buf[2] = ':';
	buf[5] = ':';
	for(uint8_t i = 0; i < 2; i++){
		buf[i] = gpsTime[i];
		buf[i + 3] = gpsTime[i + 2];
		buf[i + 6] = gpsTime[i + 4];
	}
}
void GPS_GetTime_Full(uint8_t *buf){
	isNewData = 0;
	buf[2] = ':';
	buf[5] = ':';
	for(uint8_t i = 0; i < 2; i++){
		buf[i] = gpsTime_UTC[i];
		buf[i + 3] = gpsTime_UTC[i + 2];
		buf[i + 6] = gpsTime_UTC[i + 4];
	}
	buf[8] = gpsTime_UTC[6];
	buf[9] = gpsTime_UTC[7];
	buf[10] = gpsTime_UTC[8];
}

/* Returns year of length 4 in format "yyyy" */
void GPS_GetYear(uint8_t *buf){
	isNewData = 0;
	for(uint8_t i = 0; i < 5; i++){
		buf[i] = gps_year[i];
	}
}

/* Returns month of length 2 in format "xx" */
void GPS_GetMonth(uint8_t *buf){
	isNewData = 0;
	for(uint8_t i = 0; i < 2; i++){
		buf[i] = gps_month[i];
	}
}

/* Returns date of length 2 in format "xx" */
void GPS_GetDate(uint8_t *buf){
	isNewData = 0;
	for(uint8_t i = 0; i < 2; i++){
		buf[i] = gps_date[i];
	}
}

/* Returns GPS_OK if got any data */
uint8_t GPS_IsData(){
	if(isData){
		isData = 0;
		return GPS_OK;
	}else{
		return GPS_NOK;
	}
}

/* Returns GPS_OK if got new data, clears on any get function */
uint8_t GPS_IsNewData(){
	if(isNewData){
		return GPS_OK;
	}else{
		return GPS_NOK;
	}
}
