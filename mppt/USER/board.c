#include "board.h"

/*********************************************************************************************************
*	�� �� ��: PeripheralInit
*	����˵��: ϵͳ�����ʼ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************/
void PeripheralInit(void)
{	
	delay_init();	    								//��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);  								//���ڳ�ʼ��
	SPI2_Init();       									//SPI2��ʼ��
	ADS1256_GPIO_Init();								//ADC GPIO��ʼ��
	//USART_Configuration(); 
	ADS1256_CfgADC(PGA_1, DATARATE_100);				//����ADC������ ����1:1, ����������� 100Hz
	TIM2_Init(899,7); 									//PWMƵ�� f = 72000000/(7199+1)*(0+1) = 10KHZ
                      									//PWM���� T = 1/f = 1/10000 = 0.0001S = 0.1ms
	/*OLED_Init();
	OLED_ColorTurn(0);									//0������ʾ��1 ��ɫ��ʾ
	OLED_DisplayTurn(0);								//0������ʾ 1 ��Ļ��ת��ʾ*/
	delay_ms (500);
}

