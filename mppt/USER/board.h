#ifndef _BOARD_H
#define _BOARD_H

#include <stdio.h>
#include "math.h"
//ϵͳͷ�ļ�
#include "sys.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h" 
#include "stm32f10x_tim.h"
#include "delay.h"
#include "usart.h"
//�Լ���д��ͷ�ļ�
#include "led.h"
#include "oled.h"
#include "spi.h"
#include "ads1256.h"
#include "pwm.h"
#include "MPPT_P_O.h"
#include "Temperature.h"
#include "mppt_output.h"


void PeripheralInit(void);


#endif
