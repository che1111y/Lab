#include "ads1256.h"
#include "delay.h"
#include "gpio.h"
#include "math.h"

// int32_t AdcNow[8];		// 8路ADC采集结果（实时）有符号数
// uint8_t Channel;			// 当前通道 
// uint8_t ScanMode;			// 扫描模式，0表示单端8路， 1表示差分4路

/*********************************************************************************************************
*	函 数 名: ADS1256_GPIO_Init
*	功能说明: ADC GPIO初始化
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************/
void ADS1256_GPIO_Init(void)
{
	MX_GPIO_Init();

	RST_L;		// 拉低复位引脚
	delay_ms(1);
	RST_H;		// 拉高复位引脚
	delay_ms(100);
	CS_H;			// 拉高片选引脚
	SCLK_L;		// 拉低时钟引脚
	DIN_H;		// 拉高输入引脚
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
	delay_20ns(50);

	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for (i = 0; i < 8; i++)
	{
		SCLK_H;
		delay_20ns(25);
		read = read<<1;
		SCLK_L;
		if (DO_IS_H)
		{
			read++;
		}
		delay_20ns(25);
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
	delay_us(1); 	/* 必须延迟才能读取芯片返回数据 */
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

//-----------------------------------------------------------------
// void ADS1256_CfgADC(unsigned char gain,unsigned char drate)
//-----------------------------------------------------------------
// 函数功能: 配置ADC参数，增益及转换速率
// 入口参数: 增益，速率
// 返回参数: 无
// 全局变量: 无
// 调用模块: void ADS1256_WriteCmd(unsigned char cmd) 
//           void ADS1256_WriteReg(unsigned char Reg_ID,unsigned char Reg_Date)
//           void ADS1256_Send8Bit(unsigned char date)
// 注意事项: 
//-----------------------------------------------------------------
void ADS1256_CfgADC(uint8_t gain,uint8_t drate)   // 初始化设置，设置增益以及转换速率
{
	ADS1256_WriteCmd(CMD_RESET);                // 写复位指令
	ADS1256_WriteReg(REG_STATUS,0XF4);          // 写状态，数据传输默认高位在前，启动矫正，禁止使用缓冲
	ADS1256_WriteCmd(CMD_SELFCAL);              // 自校准
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
		buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		
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
		ADS1256_Send8Bit(CMD_WREG|0);          // 写寄存器
		ADS1256_Send8Bit(0x03);                // 连续写入4个数据
		ADS1256_Send8Bit(buf[0]);              // 设置状态寄存器
		ADS1256_Send8Bit(buf[1]);              // 设置输入通道参数
		ADS1256_Send8Bit(buf[2]);              // 设置ADCON控制寄存器，增益
		ADS1256_Send8Bit(buf[3]);	             // 设置数据速率
		CS_H;
	}
	delay_us(50);
}

//-----------------------------------------------------------------
// unsigned long ADS1256_GetAdc(unsigned char channel)
//-----------------------------------------------------------------
// 函数功能: 获取ADC的采样结果
// 入口参数: 无
// 返回参数: ADC采样结果
// 全局变量: 无
// 调用模块: void ADS1256_WriteCmd(unsigned char cmd) 
//           void ADS1256_WriteReg(unsigned char Reg_ID,unsigned char Reg_Date)
//           void ADS1256_Send8Bit(unsigned char date)
// 注意事项: 
//-----------------------------------------------------------------
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
	CS_L;                                    //片选拉低
	ADS1256_Send8Bit(CMD_RDATA);             //读取数据命令
	
	delay_us(10);                
	 
	//连续接收3个数据，高字节在前
	read = ((uint32_t)ADS1256_Recive8Bit() << 16);          
	read +=( (uint32_t)ADS1256_Recive8Bit() << 8);
	read += ADS1256_Recive8Bit() ;
	CS_H;
//  		read=0x678965;
	return read;	
}

// int32_t adc[8];									// 采样结果
// int32_t adc_1[8];								
// float volt[8]={0,0,0,0,0,0,0,0};	// 实际电压值
// uint8_t i;
// uint8_t mode;										// 扫描模式, 单端或者差分
// uint8_t ch_num;									// 通道数
//-----------------------------------------------------------------
// unsigned long ADS1256_GetAdc(unsigned char channel)
//-----------------------------------------------------------------
// 函数功能: 获取电压值
// 入口参数: uint8_t channel(), uint8_t mode(扫描模式, 单端或者差分,0表示单端8路，1表示差分4路)
// 返回参数: ADC采样结果
// 全局变量: 无
// 调用模块: 
// 注意事项: 
//-----------------------------------------------------------------
float ADS1256_GetVolt(uint8_t channel, uint8_t mode)
{
	int32_t adc,adc_1;									// 采样结果
	int32_t ch_num;		
	float volt;

	// delay_ms (100);
	if(mode==0)
	{
		ch_num = 8;		// 当前通道数 = 8
	
	}	
	else
	{
		ch_num = 4;		// 当前通道数 = 4
	
	}

	if(mode == 0) // 模式0 8路
		{
			// for (i = 0; i < ch_num; i++)
			// {
				adc = ADS1256_GetAdc(channel<<4 | 0x08);	 	// 从全局缓冲区读取采样结果。 				
				adc_1 = (adc ^ 0x800000);				// 4194303 = 2.5V , 这是理论值，实际可以根据2.5V基准的实际值进行公式矫正 
				volt = (((0.596047*adc_1)-5000000)/1000000);  // 测量出来是以v作单位
				delay_ms (100);
			// }
		}
		else// 模式1 4路
		{
			// for (i = 0; i < ch_num; i++)
			// {		
				adc = ADS1256_GetAdc((2*channel)<<4 | 2*channel+1);	 	// 从全局缓冲区读取采样结果。			
				adc_1 = (adc ^ 0x800000);				// 4194303 = 2.5V , 这是理论值，实际可以根据2.5V基准的实际值进行公式矫正 		
				volt = (((0.596047*adc_1)-5000000)/1000000);  // 测量出来是以v作单位
				delay_ms (100);
			// }
		}
		// for(i=0;i<ch_num;i++)
		// {
		// 	printf("CH%1d:   %4.5fv\r\n", i, volt[i]); //发送数据
			
		
		// }	

	return volt;	
}

const float Rp=10000.0; //10K
const float T2 = (273.15+25.0);;//T2
const float Bx = 3950.0;//B
const float Ka = 273.15;
//-----------------------------------------------------------------
// uint32_t Get_Temp(uint8_t channel, int R)
//-----------------------------------------------------------------
// 函数功能: 获取热敏电阻阻值
// 入口参数: 
// 返回参数: 热敏电阻温度
// 全局变量: 无
// 调用模块: 
// 注意事项: 
//-----------------------------------------------------------------
float Get_Rntc(uint8_t channel, float Rm)
{
	float Rntc = 0;
	float Vcc = 3.3;

	Rntc = Rm/(Vcc/ADS1256_GetVolt(channel, 0) - 1);

	return Rntc;	
}

/*================================================================================
*	函 数 名: Get_Temp
*	功能说明: 由温度粗值得到温度精值
*	形    参: 无
*	返 回 值: temp	：温度值
================================================================================*/
float Get_Temp(uint8_t channel)
{
	float Rt;
	float temp;
	
	Rt = Get_Rntc(channel, 10);
	
	// temp=1/(1/T2+log(Rt)/Bx)-Ka; //计算温度

	//  temp=(float)Rt*3.3;
    //        temp = log(temp);
    //        temp/=Bx;
    //        temp+=(1/T2);
    //        temp = 1/(temp);
    //        temp-=Ka;

	temp = log(Rt);//ln(Rt/Rp)
	temp/=Bx;//ln(Rt/Rp)/B
	temp+=(1/T2);
	temp = 1/(temp);
	temp-=Ka;
	return temp;
} 

/*************************************************
NTC的R值数据表    
表的数值随序号的增加而减小   
*************************************************/
#define  NTCTABNum	141


static float NTCTAB[NTCTABNum]={
87.4288,82.7877,78.4386,74.3586,70.5269,66.9248,63.5354,60.3432,57.3341,54.4954, 
51.8154,49.2833,46.8895,44.6248,42.4810,40.4505,38.5264,36.7020,34.9714,33.3292, 
31.7700,30.2534,28.8159,27.4531,26.1608,24.9350,23.7720,22.6684,21.6209,20.6263, 
19.6819,18.7848,17.9326,17.1227,16.3530,15.6212,14.9254,14.2637,13.6342,13.0353, 
12.4654,11.9229,11.4064,10.9146,10.4462,10.0000,9.5749,9.1697,8.7836,8.4155,8.0644,
7.7297,7.4103,7.1057,6.8149,6.5375,6.2726,6.0197,5.7782,5.5475,5.3271,5.1165,4.9153, 
4.7229,4.5390,4.3632,4.1950,4.0341,3.8802,3.7330,3.5920,3.4571,3.3279,3.2042,3.0857, 
2.9722,2.8634,2.7592,2.6593,2.5636,2.4718,2.3837,2.2993,2.2183,2.1405,2.0659,1.9943, 
1.9256,1.8596,1.7962,1.7353,1.6768,1.6206,1.5666,1.5147,1.4648,1.4168,1.3707,1.3263, 
1.2836,1.2425,1.2030,1.1649,1.1283,1.0930,1.0590,1.0267,0.9955,0.9654,0.9363,0.9083, 
0.8812,0.8550,0.8297,0.8052,0.7816,0.7587,0.7366,0.7152,0.6945,0.6744,0.6558,0.6376, 
0.6199,0.6026,0.5858,0.5694,0.5535,0.5380,0.5229,0.5083,0.4941,0.4803,0.4669,0.4539,
0.4412,0.4290,0.4171,0.4055,0.3944,0.3835 };


/*================================================================================
*Function	Name 	:LookupTable
*Description  		:查表函数
*parameter			:1.*p 		：表头，即表的首地址
*					 2.tableNum ：表格的元素的个数
*					 3.data 	：该变量在这里传入的是当前温度下NTC的阻值
*Return				:当前NTC阻值对应在表中的位置
================================================================================*/
#if  1 
uint8_t LookupTable(float *p , uint8_t tableNum , float data)
{
		uint16_t 	begin  = 0;   
		uint16_t 	end    = 0; 
		uint16_t 	middle = 0;  
		uint8_t 	i      = 0; 
		end = tableNum-1; 
		
		if(data >= p[begin])        	return begin;
		else if(data <= p[end])     	return end; 
		
		while(begin < end)  
		{
				middle = (begin+end)/2; 
				
				if(data == p[middle]) 							break; 
				if(data <  p[middle] && data > p[middle+1]) 	break;   
				if(data >  p[middle])  	end   = middle ;                      
				else                  	begin = middle ;      
				if(i++ > tableNum) 								break; 
		}
		if(begin > end)   				return 0;   
		
		return middle;
}
#else 
uint8_t LookupTable(float *p,uint8_t tableNum,float data)
{
		uint8_t	i,index	= 0;
	
		for(i=0;i<(tableNum-1);i++)
		{
				if((data<p[i]) && (data>p[i+1]))
				index = i;	
		}
		return index;
}
#endif

/*================================================================================
*Function Name 		:GetRoughTemperature
*Description  		:由序号转化得出温度粗值
*parameter			:serialNum	：表的序号值
*Return				:roughTemp	：温度粗值
================================================================================*/
float GetRoughTemperature(uint8_t serialNum)
{
	float  roughTemp = 0;
	
	roughTemp = serialNum-20;
	// if(serialNum <= 32)			roughTemp = serialNum;
	// else if(serialNum >= 232)	roughTemp = serialNum - 190;
	// else						roughTemp = 0.05 * (serialNum - 32) + 32;
		
	return roughTemp;
}
/*该函数是观察RT表的规律得出的*/

/*================================================================================
*Function	Name 	:GetAccuraryTemperature
*Description  		:由温度粗值得到温度精值
*parameter			:readRKohm		：读取到的电阻值
*Return				:accuraryTemp	：温度精值
================================================================================*/
float GetAccuraryTemperature(float readRKohm) 	   //这里的返回值数据是要拿出去显示出来的
{
		float  	t0   = 0;
		float  	temp = 0;		
		float  	accuraryTemp = 0;
		uint8_t serialNum    = 0;  //查表得到的 AD值 或 R值 所在的位置

		if((readRKohm <= NTCTAB[0]) && (readRKohm > NTCTAB[NTCTABNum-1]))
		{
				serialNum = LookupTable(NTCTAB,NTCTABNum,readRKohm);
				t0 = GetRoughTemperature(serialNum);
				// /*== 温度范围在32℃ -- 42℃ ==*/
				// if((readRKohm <= NTCTAB[32]) && (readRKohm > NTCTAB[232]))
				// temp = 0.05*(readRKohm-NTCTAB[serialNum])/(NTCTAB[serialNum+1]-NTCTAB[serialNum])+t0;
				// /*== 温度范围在0℃ -- 32℃  以及  42℃ -- 60℃ ==*/	
				// else	
				temp = 1*(readRKohm-NTCTAB[serialNum])/(NTCTAB[serialNum+1]-NTCTAB[serialNum])+t0;
		}
		
		accuraryTemp = temp;
		
		return accuraryTemp;
}
/****************************************************************
三个点,在坐标上的顺序依次为(X1,Y1),(X,Y),(X2,Y2)
已知(X1,Y1),(X2,Y2)，求(X,Y)
两点式：(X-X1)/(Y-Y1) = (X2-X1)/(Y2-Y1)       
则：X = [(X2-X1)/(Y2-Y1 )]* (Y-Y1) + X1
由于已知(X1,Y1),(X2,Y2)为相邻两温度点  X2-X1 = 0.05
故：X = [0.05/(Y2-Y1 )]* (Y-Y1) + X1
或者X = 0.05 * (Y-Y1) / (Y2-Y1 ) + X1
其中X对应温度值 Y对应R值    这样可以把精度从RT表上的0.05提高到0.01
下图中的(Xi,Yi)就是这里描述的(X,Y);
****************************************************************/

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
