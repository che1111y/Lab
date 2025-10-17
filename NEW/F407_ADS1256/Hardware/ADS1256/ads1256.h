#ifndef _ads1256_H
#define _ads1256_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h" 
/* �Ĵ�����ַ�� �����Ǹ�λ��ȱʡֵ */
#define REG_STATUS	(0)	// x1H		״̬�Ĵ���
#define	REG_MUX    	(1) // 01H		�����·���������ƼĴ���
#define	REG_ADCON   (2) // 20H		A/D���ƼĴ���
#define	REG_DRATE   (3) // F0H		A/D��������
#define	REG_IO      (4) // E0H		GPIO���ƼĴ���
#define	REG_OFC0    (5) // xxH		ƫ��У׼�ֽ�0�������Ч�ֽ�
#define	REG_OFC1    (6) // xxH		ƫ��У׼�ֽ�1
#define	REG_OFC2    (7) // xxH		ƫ��У׼�ֽ�2�������Ч�ֽ�
#define	REG_FSC0    (8) // xxH		������У׼�ֽ�0�������Ч�ֽ�
#define	REG_FSC1    (9) // xxH		������У׼�ֽ�1
#define	REG_FSC2    (10)// xxH		������У׼�ֽ�2�������Ч�ֽ�

/* ����壺 TTable 24. Command Definitions --- ADS1256�����ֲ��34ҳ */
#define	CMD_WAKEUP		(0x00)	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)		���SYNC���˳�����״̬
#define	CMD_RDATA   	(0x01) 	// Read Data 0000  0001 (01h)		��ȡ����
#define	CMD_RDATAC  	(0x03) 	// Read Data Continuously 0000   0011 (03h)		������ȡ����
#define	CMD_SDATAC  	(0x0F) 	// Stop Read Data Continuously 0000   1111 (0Fh)		ֹͣ������ȡ����
#define	CMD_RREG    	(0x10) 	// Read from REG rrr 0001 rrrr (1xh)		��REG��ȡ
#define	CMD_WREG    	(0x50) 	// Write to REG rrr 0101 rrrr (5xh)			д��REG
#define	CMD_SELFCAL 	(0xF0) 	// Offset and Gain Self-Calibration 1111    0000 (F0h)		ƫ�ƺ�������У׼
#define	CMD_SELFOCAL	(0xF1) 	// Offset Self-Calibration 1111    0001 (F1h)		ƫ����У׼
#define	CMD_SELFGCAL	(0xF2) 	// Gain Self-Calibration 1111    0010 (F2h)			������У׼
#define	CMD_SYSOCAL 	(0xF3) 	// System Offset Calibration 1111   0011 (F3h)		ϵͳƫ��У׼
#define	CMD_SYSGCAL 	(0xF4) 	// System Gain Calibration 1111    0100 (F4h)			ϵͳ����У׼
#define	CMD_SYNC    	(0xFC) 	// Synchronize the A/D Conversion 1111   1100 (FCh)		ͬ��A/Dת��
#define	CMD_STANDBY 	(0xFD) 	// Begin Standby Mode 1111   1101 (FDh)		��ʼ����ģʽ
#define	CMD_RESET   	(0xFE) 	// Reset to Power-Up Values 1111   1110 (FEh)		����Ϊ����ֵ

#define PGA_1            0x00
#define PGA_2            0x01
#define PGA_4            0x02
#define PGA_8            0x03
#define PGA_16           0x04
#define PGA_32           0x05
#define PGA_64           0x06

#define POSITIVE_AIN0       (0X00)
#define POSITIVE_AIN1       (0X10)
#define POSITIVE_AIN2       (0X20)
#define POSITIVE_AIN3       (0X30)
#define POSITIVE_AIN4       (0X40)
#define POSITIVE_AIN5       (0X50)
#define POSITIVE_AIN6       (0X60)
#define POSITIVE_AIN7       (0X70)
#define POSITIVE_AINCOM     (0X80)        

#define NEGTIVE_AIN0         0X00
#define NEGTIVE_AIN1         0X01
#define NEGTIVE_AIN2         0X02
#define NEGTIVE_AIN3         0X03
#define NEGTIVE_AIN4         0X04
#define NEGTIVE_AIN5         0X05
#define NEGTIVE_AIN6         0X06
#define NEGTIVE_AIN7         0X07
#define NEGTIVE_AINCOM       0X08

#define DATARATE_30K         0xF0
#define DATARATE_15K         0xE0
#define DATARATE_7_5K        0xD0
#define DATARATE_3_7_5K      0xC0
#define DATARATE_2K          0xB0
#define DATARATE_1K          0xA0
#define DATARATE_500         0x92
#define DATARATE_100         0x82
#define DATARATE_60        	 0x72
#define DATARATE_50        	 0x63
#define DATARATE_30        	 0x53
#define DATARATE_25        	 0x43
#define DATARATE_15        	 0x33
#define DATARATE_10        	 0x23
#define DATARATE_5        	 0x13
#define DATARATE_2_5     			0x02


#define	RST				(GPIOB,GPIO_Pin_7)
#define RST_L			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)
#define RST_H			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)

#define	CS				(GPIOB,GPIO_Pin_5)
#define	CS_L			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define	CS_H			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)


#define	DIN				(GPIOB,GPIO_Pin_4)
#define	DIN_L			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define	DIN_H			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)

#define	SCLK			(GPIOB,GPIO_Pin_3)
#define	SCLK_L		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define	SCLK_H		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)


#define	DOUT			(GPIOB,GPIO_Pin_8)
#define	DOUT_L		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define	DOUT_H		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define DO_IS_H		(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8) == GPIO_PIN_SET)



#define	DRDY			(GPIOB,GPIO_Pin_9)
#define	DRDY_L		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define	DRDY_H		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define DRDY_IS_L	(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == GPIO_PIN_RESET)


///*STATUS REGISTER*/
//#define MSB_FRIST                (0x00<<3)
//#define LSB_FRIST                (0x01<<3)
//#define ACAL_OFF                 (0x00<<2)
//#define ACAL_ON                  (0x01<<2)
//#define BUFEN_OFF                (0x00<<1)
//#define BUFEN_ON                 (0x01<<1)

///*ADCON REGISTER*/
//#define CLKOUT_OFF               (0x00<<5)
//#define CLKOUT_CLKIN             (0x01<<5)
//#define DETECT_OFF               (0x00<<3)
//#define DETECT_ON_2UA            (0x02<<3)

uint32_t ADS1256_GetAdc(uint8_t channel);
float ADS1256_AdcFilter(uint8_t channel);
void ADS1256_CfgADC(uint8_t gain,uint8_t drate);
void ADS1256_GPIO_Init(void);
float ADS1256_GetVolt(uint8_t channel, uint8_t mode);
float Get_Rntc(uint8_t channel, float Rm);
float Get_Temp(uint8_t channel);
uint8_t LookupTable(float *p , uint8_t tableNum , float data);
float GetRoughTemperature(uint8_t serialNum);
float GetAccuraryTemperature(float readRKohm);

#endif

