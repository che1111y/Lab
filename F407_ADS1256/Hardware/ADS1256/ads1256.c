#include "ads1256.h"
#include "delay.h"
#include "gpio.h"
#include "math.h"

// int32_t AdcNow[8];		// 8·ADC�ɼ������ʵʱ���з�����
// uint8_t Channel;			// ��ǰͨ�� 
// uint8_t ScanMode;			// ɨ��ģʽ��0��ʾ����8·�� 1��ʾ���4·

/*********************************************************************************************************
*	�� �� ��: ADS1256_GPIO_Init
*	����˵��: ADC GPIO��ʼ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************/
void ADS1256_GPIO_Init(void)
{
	MX_GPIO_Init();

	RST_L;		// ���͸�λ����
	delay_ms(1);
	RST_H;		// ���߸�λ����
	delay_ms(100);
	CS_H;			// ����Ƭѡ����
	SCLK_L;		// ����ʱ������
	DIN_H;		// ������������
}
/*
*********************************************************************************************************
*	�� �� ��: ADS1256_Send8Bit
*	����˵��: ��SPI���߷���8��bit���ݡ� ����CS���ơ�
*	��    ��: _data : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ADS1256_Send8Bit(uint8_t data)
{
	uint8_t i;
	/* �������Ͷ���ֽ�ʱ����Ҫ�ӳ�һ�� */
	delay_us(1);	
	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
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
		SCLK_L;			/* <----  ADS1256 ����SCK�½��ز���DIN����, ���ݱ���ά�� 50nS */
	  delay_us(1);	
	}
}

/*********************************************************************************************************
*	�� �� ��: ADS1256_Recive8Bit
*	����˵��: ��SPI���߽���8��bit���ݡ� ����CS���ơ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************/
uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t i;
	uint8_t read = 0;
	delay_20ns(50);

	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
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
*	�� �� ��: ADS1256_WriteReg
*	����˵��: дָ���ļĴ���
*	��    ��:  _RegID : �Ĵ���ID
*			  _RegValue : �Ĵ���ֵ
*	�� �� ֵ: ��
*********************************************************************************************************/
void ADS1256_WriteReg(uint8_t RegID, uint8_t RegValue)
{
	CS_L;	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(CMD_WREG | RegID);	/* д�Ĵ���������, �����ͼĴ�����ַ */
	ADS1256_Send8Bit(0x00);		/* �Ĵ������� - 1, �˴�д1���Ĵ��� */
	ADS1256_Send8Bit(RegValue);	/* ���ͼĴ���ֵ */
	CS_H;	/* SPIƬѡ = 1 */
}

/*********************************************************************************************************
*	�� �� ��: ADS1256_ReadReg
*	����˵��: ��ָ���ļĴ���
*	��    ��:  _RegID : �Ĵ���ID
*			  _RegValue : �Ĵ���ֵ��
*	�� �� ֵ: �����ļĴ���ֵ��
*********************************************************************************************************/
uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_L;	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* д�Ĵ���������, �����ͼĴ�����ַ */
	ADS1256_Send8Bit(0x00);	/* �Ĵ������� - 1, �˴���1���Ĵ��� */
	delay_us(1); 	/* �����ӳٲ��ܶ�ȡоƬ�������� */
	read = ADS1256_Recive8Bit();	/* ���Ĵ���ֵ */
	CS_H;	/* SPIƬѡ = 1 */

	return read;
}

/*********************************************************************************************************
*	�� �� ��: ADS1256_WriteCmd
*	����˵��: ���͵��ֽ�����
*	��    ��:  _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************/
void ADS1256_WriteCmd(uint8_t cmd)
{
	CS_L;	/* SPIƬѡ = 0 */
	ADS1256_Send8Bit(cmd);
	CS_H;	/* SPIƬѡ = 1 */
}

//-----------------------------------------------------------------
// void ADS1256_CfgADC(unsigned char gain,unsigned char drate)
//-----------------------------------------------------------------
// ��������: ����ADC���������漰ת������
// ��ڲ���: ���棬����
// ���ز���: ��
// ȫ�ֱ���: ��
// ����ģ��: void ADS1256_WriteCmd(unsigned char cmd) 
//           void ADS1256_WriteReg(unsigned char Reg_ID,unsigned char Reg_Date)
//           void ADS1256_Send8Bit(unsigned char date)
// ע������: 
//-----------------------------------------------------------------
void ADS1256_CfgADC(uint8_t gain,uint8_t drate)   // ��ʼ�����ã����������Լ�ת������
{
	ADS1256_WriteCmd(CMD_RESET);                // д��λָ��
	ADS1256_WriteReg(REG_STATUS,0XF4);          // д״̬�����ݴ���Ĭ�ϸ�λ��ǰ��������������ֹʹ�û���
	ADS1256_WriteCmd(CMD_SELFCAL);              // ��У׼
	delay_us(20);
	{
		uint8_t buf[4];
		/* ״̬�Ĵ�������
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

			ACAL=1ʹ����У׼���ܡ��� PGA��BUFEEN, DRATE�ı�ʱ��������У׼
		*/
		buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		
		/* ����λ0��ʾAINP�� AIN0,  ����λ8��ʾ AINN �̶��� AINCOM */
		buf[1] = 0x08;                      //ͨ������ѡ��
		
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
				01 = Sensor Detect Current = 0.5 �� A
				10 = Sensor Detect Current = 2 �� A
				11 = Sensor Detect Current = 10�� A
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
		
		buf[3] = drate;	// DRATE_10SPS;	/* ѡ������������� */
		CS_L;
		ADS1256_Send8Bit(CMD_WREG|0);          // д�Ĵ���
		ADS1256_Send8Bit(0x03);                // ����д��4������
		ADS1256_Send8Bit(buf[0]);              // ����״̬�Ĵ���
		ADS1256_Send8Bit(buf[1]);              // ��������ͨ������
		ADS1256_Send8Bit(buf[2]);              // ����ADCON���ƼĴ���������
		ADS1256_Send8Bit(buf[3]);	             // ������������
		CS_H;
	}
	delay_us(50);
}

//-----------------------------------------------------------------
// unsigned long ADS1256_GetAdc(unsigned char channel)
//-----------------------------------------------------------------
// ��������: ��ȡADC�Ĳ������
// ��ڲ���: ��
// ���ز���: ADC�������
// ȫ�ֱ���: ��
// ����ģ��: void ADS1256_WriteCmd(unsigned char cmd) 
//           void ADS1256_WriteReg(unsigned char Reg_ID,unsigned char Reg_Date)
//           void ADS1256_Send8Bit(unsigned char date)
// ע������: 
//-----------------------------------------------------------------
uint32_t ADS1256_GetAdc(uint8_t channel)
{
	uint32_t read;
	read = 0;
	
// 	while(DRDY);           //��DRDY��Ϊ�͵�ƽʱ�����ݿ�ʼ����
	ADS1256_WriteReg(REG_MUX,channel);       // д���ȡ��ͨ��
	ADS1256_WriteCmd(CMD_SYNC);              //ͬ��A/Dת������
// 	Delay_1us(1);
	ADS1256_WriteCmd(CMD_WAKEUP);            //���SYNC���˳�����ģʽ
	while(!DRDY_IS_L);                             // �ȴ�����ת�����
	CS_L;                                    //Ƭѡ����
	ADS1256_Send8Bit(CMD_RDATA);             //��ȡ��������
	
	delay_us(10);                
	 
	//��������3�����ݣ����ֽ���ǰ
	read = ((uint32_t)ADS1256_Recive8Bit() << 16);          
	read +=( (uint32_t)ADS1256_Recive8Bit() << 8);
	read += ADS1256_Recive8Bit() ;
	CS_H;
//  		read=0x678965;
	return read;	
}

// int32_t adc[8];									// �������
// int32_t adc_1[8];								
// float volt[8]={0,0,0,0,0,0,0,0};	// ʵ�ʵ�ѹֵ
// uint8_t i;
// uint8_t mode;										// ɨ��ģʽ, ���˻��߲��
// uint8_t ch_num;									// ͨ����
//-----------------------------------------------------------------
// unsigned long ADS1256_GetAdc(unsigned char channel)
//-----------------------------------------------------------------
// ��������: ��ȡ��ѹֵ
// ��ڲ���: uint8_t channel(), uint8_t mode(ɨ��ģʽ, ���˻��߲��,0��ʾ����8·��1��ʾ���4·)
// ���ز���: ADC�������
// ȫ�ֱ���: ��
// ����ģ��: 
// ע������: 
//-----------------------------------------------------------------
float ADS1256_GetVolt(uint8_t channel, uint8_t mode)
{
	int32_t adc,adc_1;									// �������
	int32_t ch_num;		
	float volt;

	// delay_ms (100);
	if(mode==0)
	{
		ch_num = 8;		// ��ǰͨ���� = 8
	
	}	
	else
	{
		ch_num = 4;		// ��ǰͨ���� = 4
	
	}

	if(mode == 0) // ģʽ0 8·
		{
			// for (i = 0; i < ch_num; i++)
			// {
				adc = ADS1256_GetAdc(channel<<4 | 0x08);	 	// ��ȫ�ֻ�������ȡ��������� 				
				adc_1 = (adc ^ 0x800000);				// 4194303 = 2.5V , ��������ֵ��ʵ�ʿ��Ը���2.5V��׼��ʵ��ֵ���й�ʽ���� 
				volt = (((0.596047*adc_1)-5000000)/1000000);  // ������������v����λ
				delay_ms (100);
			// }
		}
		else// ģʽ1 4·
		{
			// for (i = 0; i < ch_num; i++)
			// {		
				adc = ADS1256_GetAdc((2*channel)<<4 | 2*channel+1);	 	// ��ȫ�ֻ�������ȡ���������			
				adc_1 = (adc ^ 0x800000);				// 4194303 = 2.5V , ��������ֵ��ʵ�ʿ��Ը���2.5V��׼��ʵ��ֵ���й�ʽ���� 		
				volt = (((0.596047*adc_1)-5000000)/1000000);  // ������������v����λ
				delay_ms (100);
			// }
		}
		// for(i=0;i<ch_num;i++)
		// {
		// 	printf("CH%1d:   %4.5fv\r\n", i, volt[i]); //��������
			
		
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
// ��������: ��ȡ����������ֵ
// ��ڲ���: 
// ���ز���: ���������¶�
// ȫ�ֱ���: ��
// ����ģ��: 
// ע������: 
//-----------------------------------------------------------------
float Get_Rntc(uint8_t channel, float Rm)
{
	float Rntc = 0;
	float Vcc = 3.3;

	Rntc = Rm/(Vcc/ADS1256_GetVolt(channel, 0) - 1);

	return Rntc;	
}

/*================================================================================
*	�� �� ��: Get_Temp
*	����˵��: ���¶ȴ�ֵ�õ��¶Ⱦ�ֵ
*	��    ��: ��
*	�� �� ֵ: temp	���¶�ֵ
================================================================================*/
float Get_Temp(uint8_t channel)
{
	float Rt;
	float temp;
	
	Rt = Get_Rntc(channel, 10);
	
	// temp=1/(1/T2+log(Rt)/Bx)-Ka; //�����¶�

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
NTC��Rֵ���ݱ�    
�����ֵ����ŵ����Ӷ���С   
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
*Description  		:�����
*parameter			:1.*p 		����ͷ��������׵�ַ
*					 2.tableNum ������Ԫ�صĸ���
*					 3.data 	���ñ��������ﴫ����ǵ�ǰ�¶���NTC����ֵ
*Return				:��ǰNTC��ֵ��Ӧ�ڱ��е�λ��
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
*Description  		:�����ת���ó��¶ȴ�ֵ
*parameter			:serialNum	��������ֵ
*Return				:roughTemp	���¶ȴ�ֵ
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
/*�ú����ǹ۲�RT��Ĺ��ɵó���*/

/*================================================================================
*Function	Name 	:GetAccuraryTemperature
*Description  		:���¶ȴ�ֵ�õ��¶Ⱦ�ֵ
*parameter			:readRKohm		����ȡ���ĵ���ֵ
*Return				:accuraryTemp	���¶Ⱦ�ֵ
================================================================================*/
float GetAccuraryTemperature(float readRKohm) 	   //����ķ���ֵ������Ҫ�ó�ȥ��ʾ������
{
		float  	t0   = 0;
		float  	temp = 0;		
		float  	accuraryTemp = 0;
		uint8_t serialNum    = 0;  //���õ��� ADֵ �� Rֵ ���ڵ�λ��

		if((readRKohm <= NTCTAB[0]) && (readRKohm > NTCTAB[NTCTABNum-1]))
		{
				serialNum = LookupTable(NTCTAB,NTCTABNum,readRKohm);
				t0 = GetRoughTemperature(serialNum);
				// /*== �¶ȷ�Χ��32�� -- 42�� ==*/
				// if((readRKohm <= NTCTAB[32]) && (readRKohm > NTCTAB[232]))
				// temp = 0.05*(readRKohm-NTCTAB[serialNum])/(NTCTAB[serialNum+1]-NTCTAB[serialNum])+t0;
				// /*== �¶ȷ�Χ��0�� -- 32��  �Լ�  42�� -- 60�� ==*/	
				// else	
				temp = 1*(readRKohm-NTCTAB[serialNum])/(NTCTAB[serialNum+1]-NTCTAB[serialNum])+t0;
		}
		
		accuraryTemp = temp;
		
		return accuraryTemp;
}
/****************************************************************
������,�������ϵ�˳������Ϊ(X1,Y1),(X,Y),(X2,Y2)
��֪(X1,Y1),(X2,Y2)����(X,Y)
����ʽ��(X-X1)/(Y-Y1) = (X2-X1)/(Y2-Y1)       
��X = [(X2-X1)/(Y2-Y1 )]* (Y-Y1) + X1
������֪(X1,Y1),(X2,Y2)Ϊ�������¶ȵ�  X2-X1 = 0.05
�ʣ�X = [0.05/(Y2-Y1 )]* (Y-Y1) + X1
����X = 0.05 * (Y-Y1) / (Y2-Y1 ) + X1
����X��Ӧ�¶�ֵ Y��ӦRֵ    �������԰Ѿ��ȴ�RT���ϵ�0.05��ߵ�0.01
��ͼ�е�(Xi,Yi)��������������(X,Y);
****************************************************************/

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
