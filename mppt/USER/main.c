#include "board.h"

// adc,pwm,oled,uart,spi,i2c,

// unsigned char i;

// char showLcd[30];

 int main(void)
 {	
	float temp,Rtem,volt;
	// uint8_t i,j;
	// //uint8_t mode;
	 
	// uint32_t adc1[8],adc2[8];									//采样结果
	// uint32_t adc_1[8];								//
	// u8 Rm=30;
	// static float Vcc=3.3;
	// float volt1[8],volt2[8],Rtem;

	PeripheralInit();								//各种初始化
	 
	while(1)
	{
		volt = Measure_Voltage(0);
		printf("Voltage: %4.3fV\r\n",1,volt);

		Rtem = Get_Rntc();
		printf("R: %4.3fO\r\n",1,Rtem);

		temp = GetAccuraryTemperature(Rtem);
		printf("Temperature: %4.3f℃\r\n",1,temp);
		//adc获取电压四通道
		//循环配置通道并读取
        // for(i = 0; i < 4; i++)
        // {
		// 	j = 2*i;
		// 	adc1[i] = (int32_t)ADS1256_GetAdc( (j << 4) | 0x08 );		// 读取采样结果
				
		// 	adc_1[i] = (adc1[i] ^ 0x800000);         //将补码转换成原码
		// 	volt1[i] = (((0.596047*adc_1[i])-5000000)/1000000);    //得到实际电压

		// 	adc2[i] = ADS1256_GetAdcValue(i);
		// 	volt2[i] = Measure_Voltage(i);
        // }
		
		// 	// Rtem = (volt[1]*(float)Rm)/(Vcc-volt[1]);
		// 	printf("111CH%d: %4.3fV\r\n",1,volt1[0]);
		// 	printf("222CH%d: %4.3fV\r\n",1,volt2[0]);
		// 	printf("111CH%d: %4.3fV\r\n",2,volt1[1]);
		// 	printf("222CH%d: %4.3fV\r\n",2,volt2[1]);
		// 	printf("111CH%d: %4.3fV\r\n",3,volt1[2]);
		// 	printf("222CH%d: %4.3fV\r\n",3,volt2[2]);
		// 	printf("111CH%d: %4.3fV\r\n",4,volt1[3]);
		// 	printf("222CH%d: %4.3fV\r\n",4,volt2[4]);
			

			// printf("111CH%d: %d\r\n",1,adc1[0]);
			// printf("111CH%d: %d\r\n",2,adc1[1]);
			// printf("111CH%d: %d\r\n",3,adc1[2]);
			// printf("111CH%d: %d\r\n",4,adc1[3]);

			// printf("222CH%d: %d\r\n",1,adc1[0]);
			// printf("222CH%d: %d\r\n",2,adc1[1]);
			// printf("222CH%d: %d\r\n",3,adc1[2]);
			// printf("222CH%d: %d\r\n",4,adc1[3]);

			// printf("R: %4.3fkOhm\r\n",Rtem);
			// sprinf(showLcd, "R1=%4.3f",volt[i]);
			// OLED_ShowString(0 ,1 ,(u8*)showLcd ,8,1);
			// OLED_Refresh();
			delay_ms(500);
	}
 }

