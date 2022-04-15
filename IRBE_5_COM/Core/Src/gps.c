#include "gps.h"
#include <stdio.h>
#include <stdlib.h>


uint8_t rxBuf;
uint8_t rxTemp;

//2 height
//3 speed



uint8_t gpsData[24] = {0}; //0 coord latitude //1 coord longitude "05723.06487N", "02132.70887E"
uint8_t gpsHeight[8] = {0};
uint8_t gpsSpeed[6] = {0};
uint8_t gpsTime[6] = {0};

uint8_t gpsTemp[80];
uint8_t gpsTempLen = 255;

uint8_t isData = 0;
uint8_t isNewData = 0;

// "000009.7"
// "00.358"



/* Pass uint8_t of received data */
void GPS_Receive(uint8_t data){
	if(data == '$'){
		gpsTempLen = 0;
	}else if((data == 13 || data == 10) && gpsTempLen != 255){
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
			}
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

/* Returns latitude of length 12*/
void GPS_GetLat(uint8_t *buf){
	isNewData = 0;
	for(uint8_t i = 1; i < 10; i++){
		buf[i] = gpsData[i];
	}
}

/* Returns longitude of length 12*/
void GPS_GetLon(uint8_t *buf){
	isNewData = 0;
	for(uint8_t i = 1; i < 10; i++){
		buf[i] = gpsData[i + 12];
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

/* Returns time of length 8 in format "hh:mm:ss" */
void GPS_GetTime(uint8_t *buf){
	isNewData = 0;
//	uint8_t temp_hour[2];
//	memcpy(temp_hour, gpsTime, 2);
//	if(strcmp((char *)temp_hour, "23") <= 0){
//		if(temp_hour[1] < '8'){
//			temp_hour[1] += 2;
//		}else if(temp_hour[1] == '8'){
//			temp_hour[0]++;
//			temp_hour[1] = (uint8_t) '0';
//		}else if(temp_hour[1] == '9'){
//			temp_hour[0]++;
//			temp_hour[1] = (uint8_t) '1';
//		}
//	}else if(strcmp((char *)temp_hour, "23") == 1){
//		temp_hour[0] = (uint8_t) '0';
//		temp_hour[1] = (uint8_t) '1';
//	}else if(strcmp((char *)temp_hour, "23") > 1){
//		uint8_t a = strcmp((char *)temp_hour, "23") > 0;
//		temp_hour[0] = (uint8_t) '0';
//		temp_hour[1] = (uint8_t) '1';
//	}
	buf[2] = ':';
	buf[5] = ':';
	for(uint8_t i = 0; i < 2; i++){
		//buf[i] = temp_hour[i];
		buf[i] = gpsTime[i];
		buf[i + 3] = gpsTime[i + 2];
		buf[i + 6] = gpsTime[i + 4];
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
