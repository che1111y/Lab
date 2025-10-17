#ifndef __BSP_ESP8266_MQTT_H
#define __BSP_ESP8266_MQTT_H

// #include "common.h"
#include <stdio.h>  
#include <string.h>  
#include <stdbool.h>
#include "delay.h"
// #include "bsp_led.h" 
#include "bsp_esp8266.h"
#include "stm32f4xx_hal_uart.h"

// extern int led_value;//led����ֵ
// extern uint8_t mqtt_flag;//matt���ӱ�־


bool ESP8266_MQTT_USERCFG ( void );
bool ESP8266_MQTT_CONN ( void );
bool ESP8266_MQTT_SUB ( void );
bool ESP8266_MQTT_PUB ( uint8_t temp_set,uint8_t humi_set,uint8_t led_value );
bool ESP8266_MQTT_RECV( void );
void ESP8266_StaTcpClient_Unvarnish_ConfigTest(void);


#endif /* __BSP_ESP8266_MQTT_H */


