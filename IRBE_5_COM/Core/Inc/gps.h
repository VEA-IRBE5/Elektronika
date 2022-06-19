#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"

/*
 *
 *
 *
 */
#define Portable 0 //DEFAULT
#define Stationary 2
#define Pedestrian 3
#define Automotive 4
#define At_sea_mode 5
#define Airborne_1 6
#define Airborne_2 7
#define Airborne_4 8

#define FIX_MODE_2D 0
#define FIX_MODE_3D 1
#define FIX_MODE_AUTO 2

#define BAUD_RATE_4800 4800
#define BAUD_RATE_9600 9600
#define BAUD_RATE_19200 19200
#define BAUD_RATE_38400 38400
#define BAUD_RATE_57600 57600
#define BAUD_RATE_115200 115200

#define GPS_OK								1
#define GPS_NOK								0

void GPS_Receive(uint8_t data);
void GPS_GetCoord();

uint8_t GPS_IsData();
uint8_t GPS_IsNewData();

void GPS_GetTime(uint8_t *buf);
void GPS_GetLat(uint8_t *buf);
void GPS_GetLon(uint8_t *buf);
void GPS_GetSpe(uint8_t *buf);
void GPS_GetHei(uint8_t *buf);
void GPS_GetYear(uint8_t *buf);
void GPS_GetMonth(uint8_t *buf);
void GPS_GetDate(uint8_t *buf);
void GPS_GetTime(uint8_t *buf);
uint8_t GSM_InitUart(UART_HandleTypeDef *huart);
void GPS_init_baudrate(uint8_t port, uint8_t inProto, uint8_t outProto, uint32_t baud_rate, uint8_t autobaud_ing);
void Get_Navigation_Engine_Settings(); // 20 byte response
void Set_Navigation_Engine_To_Airborne(void);


uint8_t GPS_Parse(uint8_t *buf, uint8_t len);
uint8_t GPS_CheckSum(uint8_t *buf, uint8_t len);
uint8_t GPS_message_checksum(char *message, uint16_t size);
uint8_t GPS_HexToByte(uint8_t *hex, uint8_t *value);
uint16_t Fletcher16(uint8_t *data, uint8_t size);
size_t write_to_buffer(char *buffer, size_t size, uint8_t data);

typedef struct{
	uint16_t dig_T1; // adrese: 0x88 / 0x89
	int16_t dig_T2; // adrese: 0x8A / 0x8B
	int16_t dig_T3; // adrese: 0x8C / 0x8D
	uint16_t dig_P1; // adrese: 0x8E / 0x8F
	int16_t dig_P2; // adrese: 0x90 / 0x91
	int16_t dig_P3; // adrese: 0x92 / 0x93
	int16_t dig_P4; // adrese: 0x94 / 0x95
	int16_t dig_P5; // adrese: 0x96 / 0x97
	int16_t dig_P6; // adrese: 0x98 / 0x99
	int16_t dig_P7; // adrese: 0x9A / 0x9B
	int16_t dig_P8; // adrese: 0x9C / 0x9D
	int16_t dig_P9; // adrese: 0x9E / 0x9F
}BMP280_Compensation_Data;


extern BMP280_Compensation_Data Compensation_Data;
//
//typedef struct GPS_INFORMATION{
//	uint8_t port_id; //byte offset 0
//	uint8_t reserver_0; //byte offset 1
//	uint16_t tx_ready; //byte offest 2
//	uint32_t mode; //byte offset 4
//	uint32_t baudrate; //byte offset 8
//	uint16_t in_proto_mask; //byte offset 12
//	uint16_t out_proto_mask; //byte offset 14
//	uint16_t reserver4; //always 0 byte offset 16
//	uint16_t reserver5; //always 0 byte offset 18
//} UART_CONFIG;

typedef struct GPS_INFORMATION{

} GPS_DATA;

#endif /* INC_GPS_H_ */
