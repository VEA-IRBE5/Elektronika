#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdint.h>
#include "string.h"

//#define CR									13  // carriage return
#define LF									10  // line feed


/*
 *
 *
 *
 */
#define Portable_mode 0
#define Stationary_mode 1
#define Pedestrian_mode 2
#define Automotive_mode 3
#define At_sea_mode 4
#define Airborne_mode_1 5
#define Airborne_mode_2 6
#define Airborne_mode_3 7

/*
 *
 *
 *
 */
#define Fix_mode 0
#define Fixed_alt 1
#define Min_elev 2
#define dr_limit

/*
 *
 *
 *
 */

#define Baud_rate_4800 0
#define Baud_rate_9600 1
#define Baud_rate_19200 2
#define Baud_rate_38400 3
#define Baud_rate_57600 4
#define Baud_rate_115200 5




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
void Get_UART_configuration(); // 20 byte response
void Set_UART_configuration();

uint8_t GPS_Parse(uint8_t *buf, uint8_t len);
uint8_t GPS_CheckSum(uint8_t *buf, uint8_t len);
uint8_t GPS_HexToByte(uint8_t *hex, uint8_t *value);


typedef struct GPS_INFORMATION{
	uint8_t port_id; //byte offset 0
	uint8_t reserver_0; //byte offset 1
	uint16_t tx_ready; //byte offest 2
	uint32_t mode; //byte offset 4
	uint32_t baudrate; //byte offset 8
	uint16_t in_proto_mask; //byte offset 12
	uint16_t out_proto_mask; //byte offset 14
	uint16_t reserver4; //always 0 byte offset 16
	uint16_t reserver5; //always 0 byte offset 18
} UART_CONFIG;

typedef struct GPS_INFORMATION{

} GPS_DATA;

#endif /* INC_GPS_H_ */
