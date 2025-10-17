#ifndef _TEMPERATURE_H
#define _TEMPERATURE_H


float  Get_Rntc(void);
float Get_Temp(void);

uint8_t LookupTable(float *p , uint8_t tableNum , float data);
float GetRoughTemperature(uint8_t serialNum);
float GetAccuraryTemperature(float readRKohm);

#endif

