#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdint.h>
#include "string.h"

//#define CR									13  // carriage return
#define LF									10  // line feed

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

uint8_t GPS_Parse(uint8_t *buf, uint8_t len);
uint8_t GPS_CheckSum(uint8_t *buf, uint8_t len);
uint8_t GPS_HexToByte(uint8_t *hex, uint8_t *value);

typedef struct GPS_INFORMATION{

} GPS_DATA;

#endif /* INC_GPS_H_ */
