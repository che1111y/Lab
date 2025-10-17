//-----------------------------------------------------------------
//连线说明
//							ADS1256模块    	STM32开发板   
//     					 GND   -------  GND       
//     					 DRDY  ------>  PC9       
//     					 CS    <------  PC11    
//     					 DIN   <------  PC10   PB15  
//     					 DOUT  ------>  PC12   PB14
//    					 SCLK  <------  PC8    PB13
//     					 GND   -------  GND           
//     					 RST   <------  PC13
//							CH340模块    		STM32开发板
//     					 GND   -------  GND       
//     					 TXD   ------>  PA10       
//     					 RXD   <------  PA9

// 注意：(1)通过改变第91行mode的值可以改变扫描模式。
//			 			mode = 0表示单端8路，mode = 1表示差分4路
//			 (2)通过改变缓冲区的启动和禁用，可以改变ADS1256_ADC模块的电压输入范围。
//						缓冲区修改位置：在ads1256.c文件第174行（ADS1256_CfgADC函数中）
//						禁用buf[0] = (0 << 3) | (1 << 2) | (0 << 1);
//						启动buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
//						当缓冲区启用时，输入阻抗为80MΩ,ADS1256_ADC模块的电压输入范围为0～3V.
//						当缓冲区禁用时，输入阻抗为(150/PGA增益)KΩ，ADS1256_ADC模块的电压输入范围0～5V.
//			 (3)通过改变第89行的ADC参数，可以改变ADC的增益和采样速率。
//						PGA_1  		放大1倍			每个通道允许输入电压范围0-5V				
//						PGA_2  		放大2倍			每个通道允许输入电压范围0-2.5V
//						PGA_4  		放大4倍			每个通道允许输入电压范围0-1.25V
//						PGA_8  		放大8倍			每个通道允许输入电压范围0-0.625V
//						PGA_16 		放大16倍		  每个通道允许输入电压范围0-312.5mV
//						PGA_32 		放大32倍		  每个通道允许输入电压范围0-156.25mV
//						PGA_64 		放大64倍		  每个通道允许输入电压范围0-78.125mV
//
//						DATARATE_30K   			采样速率30KHz
//						DATARATE_15K   			采样速率15KHz
//						DATARATE_7_5K  			采样速率7.5KHz
//						DATARATE_3_7_5K			采样速率3.75KHz
//						DATARATE_2K    			采样速率2KHz
//						DATARATE_1K    			采样速率1KHz
//						DATARATE_500   			采样速率500Hz
//						DATARATE_100   			采样速率100Hz
//						DATARATE_60    			采样速率60Hz
//						DATARATE_50    			采样速率50Hz
//						DATARATE_30    			采样速率30Hz
//						DATARATE_25    			采样速率25Hz
//						DATARATE_15    			采样速率15Hz
//						DATARATE_10    			采样速率10Hz
//						DATARATE_5     			采样速率5Hz
//						DATARATE_2_5   			采样速率2.5
//			 (4)上位机串口软件的波特率设置为115200bps
//-----------------------------------------------------------------

//-----------------------------------------------------------------
#include "board.h"

/*********************************************************************************************************
*	函 数 名: ADS1256_GPIO_Init
*	功能说明: ADC GPIO初始化
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************/
void ADS1256_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	// 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);  			
	// 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13; 							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13; 							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 								
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RST_L;
	delay_ms(1);
	RST_H;
	delay_ms(100);
	CS_H;
	SCLK_L;
	DIN_H;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_Send8Bit
*	功能说明: 向SPI总线发送8个bit数据。 不带CS控制。
*	形    参: _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_Send8Bit(uint8_t data)
{
	uint8_t i;

	/* 连续发送多个字节时，需要延迟一下 */
	delay_us(1);
	delay_us(1);
	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for(i = 0; i < 8; i++)
	{
		if (data & 0x80)
		{
			DIN_H;
		}
		else
		{
			DIN_L;
		}
		SCLK_H;
		delay_us(1);
		data <<= 1;
		SCLK_L;			/* <----  ADS1256 是在SCK下降沿采样DIN数据, 数据必须维持 50nS */
		delay_us(1);
	}
}

/*********************************************************************************************************
*	函 数 名: ADS1256_Recive8Bit
*	功能说明: 从SPI总线接收8个bit数据。 不带CS控制。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************/
uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t i;
	uint8_t read = 0;

	delay_us(1);
	delay_us(1);
	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for (i = 0; i < 8; i++)
	{
		SCLK_H;
		delay_us(1);
		read = read<<1;
		SCLK_L;
		if (DO_IS_H)
		{
			read++;
		}
		delay_us(1);
	}
	return read;
}

/*********************************************************************************************************
*	函 数 名: ADS1256_WriteReg
*	功能说明: 写指定的寄存器
*	形    参:  _RegID : 寄存器ID
*			  _RegValue : 寄存器值
*	返 回 值: 无
*********************************************************************************************************/
void ADS1256_WriteReg(uint8_t RegID, uint8_t RegValue)
{
	CS_L;	/* SPI片选 = 0 */
	ADS1256_Send8Bit(CMD_WREG | RegID);	/* 写寄存器的命令, 并发送寄存器地址 */
	ADS1256_Send8Bit(0x00);		/* 寄存器个数 - 1, 此处写1个寄存器 */
	ADS1256_Send8Bit(RegValue);	/* 发送寄存器值 */
	CS_H;	/* SPI片选 = 1 */
}

/*********************************************************************************************************
*	函 数 名: ADS1256_ReadReg
*	功能说明: 读指定的寄存器
*	形    参:  _RegID : 寄存器ID
*			  _RegValue : 寄存器值。
*	返 回 值: 读到的寄存器值。
*********************************************************************************************************/
uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_L;	/* SPI片选 = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* 写寄存器的命令, 并发送寄存器地址 */
	ADS1256_Send8Bit(0x00);	/* 寄存器个数 - 1, 此处读1个寄存器 */

	delay_us(1);	/* 必须延迟才能读取芯片返回数据 */

	read = ADS1256_Recive8Bit();	/* 读寄存器值 */
	CS_H;	/* SPI片选 = 1 */

	return read;
}

/*********************************************************************************************************
*	函 数 名: ADS1256_WriteCmd
*	功能说明: 发送单字节命令
*	形    参:  _cmd : 命令
*	返 回 值: 无
*********************************************************************************************************/
void ADS1256_WriteCmd(uint8_t cmd)
{
	CS_L;	/* SPI片选 = 0 */
	ADS1256_Send8Bit(cmd);
	CS_H;	/* SPI片选 = 1 */
}

/*********************************************************************************************************
*	函 数 名: ADS1256_CfgADC
*	功能说明: 配置ADC参数，增益和数据输出速率
*	形    参: gain : 增益
*			  _		drate : 数据输出速率
*	返 回 值: 无
*********************************************************************************************************/
void ADS1256_CfgADC(uint8_t gain,uint8_t drate)   
{
	ADS1256_WriteCmd(CMD_RESET);                //写复位指令
	ADS1256_WriteReg(REG_STATUS,0XF4);          //写状态，数据传输默认高位在前，启动矫正，禁止使用缓冲
	ADS1256_WriteCmd(CMD_SELFCAL);              //自校准
	delay_us(20);
	{
		uint8_t buf[4];
		/* 状态寄存器定义
			Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

			Bit 3 ORDER: Data Output Bit Order
				0 = Most Significant Bit First (default)
				1 = Least Significant Bit First
			Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
			byte first. The ORDER bit only controls the bit order of the output data within the byte.

			Bit 2 ACAL : Auto-Calibration
				0 = Auto-Calibration Disabled (default)
				1 = Auto-Calibration Enabled
			When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
			the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
			values.

			Bit 1 BUFEN: Analog Input Buffer Enable
				0 = Buffer Disabled (default)
				1 = Buffer Enabled

			Bit 0 DRDY :  Data Ready (Read Only)
				This bit duplicates the state of the DRDY pin.

			ACAL=1使能自校准功能。当 PGA，BUFEEN, DRATE改变时会启动自校准
		*/
		buf[0] = (0 << 3) | (1 << 2) | (0 << 1);
		
		/* 高四位0表示AINP接 AIN0,  低四位8表示 AINN 固定接 AINCOM */
		buf[1] = 0x08;                      //通道设置选择
		
		/*	ADCON: A/D Control Register (Address 02h)
			Bit 7 Reserved, always 0 (Read Only)
			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
				00 = Clock Out OFF
				01 = Clock Out Frequency = fCLKIN (default)
				10 = Clock Out Frequency = fCLKIN/2
				11 = Clock Out Frequency = fCLKIN/4
				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

			Bits 4-2 SDCS1, SCDS0: Sensor Detect Current Sources
				00 = Sensor Detect OFF (default)
				01 = Sensor Detect Current = 0.5 μ A
				10 = Sensor Detect Current = 2 μ A
				11 = Sensor Detect Current = 10μ A
				The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
				ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

			Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
				000 = 1 (default)
				001 = 2
				010 = 4
				011 = 8
				100 = 16
				101 = 32
				110 = 64
				111 = 64
		*/
		buf[2] = (0 << 5) | (0 << 3) | (gain << 0);
		
		buf[3] = drate;	// DRATE_10SPS;	/* 选择数据输出速率 */
		CS_L;
		ADS1256_Send8Bit(CMD_WREG|0);          //写寄存器
		ADS1256_Send8Bit(0x03);                //连续写入4个数据
		ADS1256_Send8Bit(buf[0]);              //设置状态寄存器
		ADS1256_Send8Bit(buf[1]);              //设置输入通道参数
		ADS1256_Send8Bit(buf[2]);              //设置ADCON控制寄存器，增益
		ADS1256_Send8Bit(buf[3]);	             //设置数据速率
		CS_H;
	}
	delay_us(50);
}

/*********************************************************************************************************
*	函 数 名: ADS1256_ReadData
*	功能说明: 读取ADS1256端口的数据（即电压值）
*	形    参: 无
*	返 回 值: 读取的数据
*********************************************************************************************************/
uint32_t ADS1256_ReadData(void)
{
	
	uint32_t read = 0;

	CS_L;	/* SPI片选 = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* 读数据的命令 */

	delay_us(1);	/* 必须延迟才能读取芯片返回数据 */
	
	
	/* 读采样结果，3个字节，高字节在前 */
	read = (uint32_t)ADS1256_Recive8Bit() << 16;
	read += ((uint32_t)ADS1256_Recive8Bit() << 8);
	read += ADS1256_Recive8Bit();

	CS_H;	/* SPI片选 = 1 */

	return read;
}

/*********************************************************************************************************
*	函 数 名: ADS1256_GetAdc
*	功能说明: 获取ADS1256指定通道的数值
*	形    参: chaanal: 所选择的通道
*                    高四位表示同向通道选择，低四位表示反向通道选择，单通道输入时，channal的低四位为 1xxx
*	返 回 值: 读取的数据
*********************************************************************************************************/
uint32_t ADS1256_GetAdc(uint8_t channel)
{
	uint32_t read;
	read = 0;
	
// 	while(DRDY);           //当DRDY变为低电平时，数据开始传输
	ADS1256_WriteReg(REG_MUX,channel);       // 写入读取的通道
	
	ADS1256_WriteCmd(CMD_SYNC);              //同步A/D转换命令
	
// 	Delay_1us(1);
	ADS1256_WriteCmd(CMD_WAKEUP);            //完成SYNC并退出待机模式
	
	while(!DRDY_IS_L);                             // 等待数据转换完成	
 	read = 	ADS1256_ReadData();
	return read;	
}

/*********************************************************************************************************
*	函 数 名: ADS1256_GetAdcValue
*	功能说明: 获取ADS1256指定通道的值
*	形    参: chaanal: 所选择的通道
*	返 回 值: 读取的数据
*********************************************************************************************************/
uint32_t ADS1256_GetAdcValue(uint8_t channel)
{
	uint32_t adc[8];	//采样结果

	adc[channel] = (int32_t)ADS1256_GetAdc( ((2*channel) << 4) | 0x08 );		// 读取采样结果
	// adc[channel] = (int32_t)ADS1256_GetAdc( (channel << 4) | 0x08 );		// 读取采样结果

	return adc[channel];

}

/*********************************************************************************************************
*	函 数 名: ADS1256_GetADCAverage
*	功能说明: 获取多次采样的平均值
*	形    参: 无
*	返 回 值: 平均的AD值
*********************************************************************************************************/
// float  GetADCAverage(u8 ch)
// {
// 	/*times是样本采样次数
// 	* adc_average 是均值*/
	
// 	uint8_t t;
// 	static const uint8_t times = 5;
// 	int32_t temp_val;
// 	float adc_average;
// 	uint32_t adc[8];

// 	for(t = 0;t < times;t++)
// 	{
// 		adc[t] = ADS1256_GetAdcValue(ch);		// 读取采样结果
// 		temp_val += adc[t];
// 	}

// 	adc_average = temp_val/times;

// 	return adc_average;	
// }

/*********************************************************************************************************
*	函 数 名: Measure_Voltage
*	功能说明: 获取ADS1256指定通道的电压值
*	形    参: chaanal: 所选择的通道
*	返 回 值: 读取的数据
*********************************************************************************************************/

float Measure_Voltage(uint8_t channel)
{
	uint32_t adc[8];									//采样结果
	uint32_t adc_1[8];								//

	float voltage[8];

	adc[channel] = ADS1256_GetAdcValue(channel);		// 读取采样结果
				
	adc_1[channel] = (adc[channel] ^ 0x800000);         //将补码转换成原码
	voltage[channel] = (((0.596047*adc_1[channel])-5000000)/1000000);    //得到实际电压

	return voltage[channel];
}

/*********************************************************************************************************
*	函 数 名: Measure_Current
*	功能说明: 获取ADS1256指定通道的电流值
*	形    参: chaanal: 所选择的通道
*	返 回 值: 读取的数据
*********************************************************************************************************/
float Measure_Current(uint8_t channel, float R)
{
	uint32_t adc[8];									//采样结果
	uint32_t adc_1[8];								//

	float current[8],voltage[8];
	

	adc[channel] = ADS1256_GetAdcValue(channel);		// 读取采样结果
				
	adc_1[channel] = (adc[channel] ^ 0x800000);         //将补码转换成原码
	voltage[channel] = (((0.596047*adc_1[channel])-5000000)/1000000);    //得到实际电压

	current[channel] = voltage[channel]/R;

	return current[channel];

}


//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------



