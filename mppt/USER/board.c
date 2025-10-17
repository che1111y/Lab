#include "board.h"

/*********************************************************************************************************
*	函 数 名: PeripheralInit
*	功能说明: 系统外设初始化
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************/
void PeripheralInit(void)
{	
	delay_init();	    								//延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);  								//串口初始化
	SPI2_Init();       									//SPI2初始化
	ADS1256_GPIO_Init();								//ADC GPIO初始化
	//USART_Configuration(); 
	ADS1256_CfgADC(PGA_1, DATARATE_100);				//配置ADC参数： 增益1:1, 数据输出速率 100Hz
	TIM2_Init(899,7); 									//PWM频率 f = 72000000/(7199+1)*(0+1) = 10KHZ
                      									//PWM周期 T = 1/f = 1/10000 = 0.0001S = 0.1ms
	/*OLED_Init();
	OLED_ColorTurn(0);									//0正常显示，1 反色显示
	OLED_DisplayTurn(0);								//0正常显示 1 屏幕翻转显示*/
	delay_ms (500);
}

