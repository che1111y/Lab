#ifndef  __BSP_ESP8266_TEST_H
#define	 __BSP_ESP8266_TEST_H


#include "stm32f4xx.h"


/********************************** 用户需要设置的参数**********************************/
#define      macUser_ESP8266_ApSsid                       "HU"         //要连接的热点的名称
#define      macUser_ESP8266_ApPwd                        "hudalaile"           //要连接的热点的密钥


#define MQTT_BROKERADDRESS "h13l5NMgjgQ.iot-as-mqtt.cn-shanghai.aliyuncs.com"   //服务器域名

#define MQTT_CLIENT_ID  "ESP8266_PV|securemode=2\\,signmethod=hmacsha1\\,timestamp=1749714733503|"  //客户编号
#define MQTT_USER_NAME  "ESP8266_PV&h13l5NMgjgQ"                                //用户名
#define MQTT_PASSWD     "0B0B06A747477FEA372443C77CD84B9341F13F0F"              //用户密码
#define MQTT_SUBSCRIBE_TOPIC  "/sys/h13l5NMgjgQ/ESP8266_PV/thing/service/property/set"    //订阅主题
#define MQTT_PUBLISH_TOPIC    "/sys/h13l5NMgjgQ/ESP8266_PV/thing/event/property/post"     //消息发布

/********************************** 外部全局变量 ***************************************/
extern volatile uint8_t ucTcpClosedFlag;


/********************************** 测试函数声明 ***************************************/
void ESP8266_StaTcpClient_Unvarnish_ConfigTest(void);
void ESP8266_SendDHT11DataTest(void);

#endif /* __BSP_ESP8266_TEST_H */

