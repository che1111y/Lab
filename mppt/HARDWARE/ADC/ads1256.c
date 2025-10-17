//-----------------------------------------------------------------
//����˵��
//							ADS1256ģ��    	STM32������   
//     					 GND   -------  GND       
//     					 DRDY  ------>  PC9       
//     					 CS    <------  PC11    
//     					 DIN   <------  PC10   PB15  
//     					 DOUT  ------>  PC12   PB14
//    					 SCLK  <------  PC8    PB13
//     					 GND   -------  GND           
//     					 RST   <------  PC13
//							CH340ģ��    		STM32������
//     					 GND   -------  GND       
//     					 TXD   ------>  PA10       
//     					 RXD   <------  PA9

// ע�⣺(1)ͨ���ı��91��mode��ֵ���Ըı�ɨ��ģʽ��
//			 			mode = 0��ʾ����8·��mode = 1��ʾ���4·
//			 (2)ͨ���ı仺�����������ͽ��ã����Ըı�ADS1256_ADCģ��ĵ�ѹ���뷶Χ��
//						�������޸�λ�ã���ads1256.c�ļ���174�У�ADS1256_CfgADC�����У�
//						����buf[0] = (0 << 3) | (1 << 2) | (0 << 1);
//						����buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
//						������������ʱ�������迹Ϊ80M��,ADS1256_ADCģ��ĵ�ѹ���뷶ΧΪ0��3V.
//						������������ʱ�������迹Ϊ(150/PGA����)K����ADS1256_ADCģ��ĵ�ѹ���뷶Χ0��5V.
//			 (3)ͨ���ı��89�е�ADC���������Ըı�ADC������Ͳ������ʡ�
//						PGA_1  		�Ŵ�1��			ÿ��ͨ�����������ѹ��Χ0-5V				
//						PGA_2  		�Ŵ�2��			ÿ��ͨ�����������ѹ��Χ0-2.5V
//						PGA_4  		�Ŵ�4��			ÿ��ͨ�����������ѹ��Χ0-1.25V
//						PGA_8  		�Ŵ�8��			ÿ��ͨ�����������ѹ��Χ0-0.625V
//						PGA_16 		�Ŵ�16��		  ÿ��ͨ�����������ѹ��Χ0-312.5mV
//						PGA_32 		�Ŵ�32��		  ÿ��ͨ�����������ѹ��Χ0-156.25mV
//						PGA_64 		�Ŵ�64��		  ÿ��ͨ�����������ѹ��Χ0-78.125mV
//
//						DATARATE_30K   			��������30KHz
//						DATARATE_15K   			��������15KHz
//						DATARATE_7_5K  			��������7.5KHz
//						DATARATE_3_7_5K			��������3.75KHz
//						DATARATE_2K    			��������2KHz
//						DATARATE_1K    			��������1KHz
//						DATARATE_500   			��������500Hz
//						DATARATE_100   			��������100Hz
//						DATARATE_60    			��������60Hz
//						DATARATE_50    			��������50Hz
//						DATARATE_30    			��������30Hz
//						DATARATE_25    			��������25Hz
//						DATARATE_15    			��������15Hz
//						DATARATE_10    			��������10Hz
//						DATARATE_5     			��������5Hz
//						DATARATE_2_5   			��������2.5
//			 (4)��λ����������Ĳ���������Ϊ115200bps
//-----------------------------------------------------------------

//-----------------------------------------------------------------
#include "board.h"

/*********************************************************************************************************
*	�� �� ��: ADS1256_GPIO_Init
*	����˵��: ADC GPIO��ʼ��
*	��    ��: ��
*	�� �� ֵ: ��
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

	delay_us(1);
	delay_us(1);
	/*��ADS1256 Ҫ�� SCL�ߵ�ƽ�͵͵�ƽ����ʱ����С 200ns  */
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

	delay_us(1);	/* �����ӳٲ��ܶ�ȡоƬ�������� */

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

/*********************************************************************************************************
*	�� �� ��: ADS1256_CfgADC
*	����˵��: ����ADC����������������������
*	��    ��: gain : ����
*			  _		drate : �����������
*	�� �� ֵ: ��
*********************************************************************************************************/
void ADS1256_CfgADC(uint8_t gain,uint8_t drate)   
{
	ADS1256_WriteCmd(CMD_RESET);                //д��λָ��
	ADS1256_WriteReg(REG_STATUS,0XF4);          //д״̬�����ݴ���Ĭ�ϸ�λ��ǰ��������������ֹʹ�û���
	ADS1256_WriteCmd(CMD_SELFCAL);              //��У׼
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
		buf[0] = (0 << 3) | (1 << 2) | (0 << 1);
		
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
		ADS1256_Send8Bit(CMD_WREG|0);          //д�Ĵ���
		ADS1256_Send8Bit(0x03);                //����д��4������
		ADS1256_Send8Bit(buf[0]);              //����״̬�Ĵ���
		ADS1256_Send8Bit(buf[1]);              //��������ͨ������
		ADS1256_Send8Bit(buf[2]);              //����ADCON���ƼĴ���������
		ADS1256_Send8Bit(buf[3]);	             //������������
		CS_H;
	}
	delay_us(50);
}

/*********************************************************************************************************
*	�� �� ��: ADS1256_ReadData
*	����˵��: ��ȡADS1256�˿ڵ����ݣ�����ѹֵ��
*	��    ��: ��
*	�� �� ֵ: ��ȡ������
*********************************************************************************************************/
uint32_t ADS1256_ReadData(void)
{
	
	uint32_t read = 0;

	CS_L;	/* SPIƬѡ = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* �����ݵ����� */

	delay_us(1);	/* �����ӳٲ��ܶ�ȡоƬ�������� */
	
	
	/* �����������3���ֽڣ����ֽ���ǰ */
	read = (uint32_t)ADS1256_Recive8Bit() << 16;
	read += ((uint32_t)ADS1256_Recive8Bit() << 8);
	read += ADS1256_Recive8Bit();

	CS_H;	/* SPIƬѡ = 1 */

	return read;
}

/*********************************************************************************************************
*	�� �� ��: ADS1256_GetAdc
*	����˵��: ��ȡADS1256ָ��ͨ������ֵ
*	��    ��: chaanal: ��ѡ���ͨ��
*                    ����λ��ʾͬ��ͨ��ѡ�񣬵���λ��ʾ����ͨ��ѡ�񣬵�ͨ������ʱ��channal�ĵ���λΪ 1xxx
*	�� �� ֵ: ��ȡ������
*********************************************************************************************************/
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
 	read = 	ADS1256_ReadData();
	return read;	
}

/*********************************************************************************************************
*	�� �� ��: ADS1256_GetAdcValue
*	����˵��: ��ȡADS1256ָ��ͨ����ֵ
*	��    ��: chaanal: ��ѡ���ͨ��
*	�� �� ֵ: ��ȡ������
*********************************************************************************************************/
uint32_t ADS1256_GetAdcValue(uint8_t channel)
{
	uint32_t adc[8];	//�������

	adc[channel] = (int32_t)ADS1256_GetAdc( ((2*channel) << 4) | 0x08 );		// ��ȡ�������
	// adc[channel] = (int32_t)ADS1256_GetAdc( (channel << 4) | 0x08 );		// ��ȡ�������

	return adc[channel];

}

/*********************************************************************************************************
*	�� �� ��: ADS1256_GetADCAverage
*	����˵��: ��ȡ��β�����ƽ��ֵ
*	��    ��: ��
*	�� �� ֵ: ƽ����ADֵ
*********************************************************************************************************/
// float  GetADCAverage(u8 ch)
// {
// 	/*times��������������
// 	* adc_average �Ǿ�ֵ*/
	
// 	uint8_t t;
// 	static const uint8_t times = 5;
// 	int32_t temp_val;
// 	float adc_average;
// 	uint32_t adc[8];

// 	for(t = 0;t < times;t++)
// 	{
// 		adc[t] = ADS1256_GetAdcValue(ch);		// ��ȡ�������
// 		temp_val += adc[t];
// 	}

// 	adc_average = temp_val/times;

// 	return adc_average;	
// }

/*********************************************************************************************************
*	�� �� ��: Measure_Voltage
*	����˵��: ��ȡADS1256ָ��ͨ���ĵ�ѹֵ
*	��    ��: chaanal: ��ѡ���ͨ��
*	�� �� ֵ: ��ȡ������
*********************************************************************************************************/

float Measure_Voltage(uint8_t channel)
{
	uint32_t adc[8];									//�������
	uint32_t adc_1[8];								//

	float voltage[8];

	adc[channel] = ADS1256_GetAdcValue(channel);		// ��ȡ�������
				
	adc_1[channel] = (adc[channel] ^ 0x800000);         //������ת����ԭ��
	voltage[channel] = (((0.596047*adc_1[channel])-5000000)/1000000);    //�õ�ʵ�ʵ�ѹ

	return voltage[channel];
}

/*********************************************************************************************************
*	�� �� ��: Measure_Current
*	����˵��: ��ȡADS1256ָ��ͨ���ĵ���ֵ
*	��    ��: chaanal: ��ѡ���ͨ��
*	�� �� ֵ: ��ȡ������
*********************************************************************************************************/
float Measure_Current(uint8_t channel, float R)
{
	uint32_t adc[8];									//�������
	uint32_t adc_1[8];								//

	float current[8],voltage[8];
	

	adc[channel] = ADS1256_GetAdcValue(channel);		// ��ȡ�������
				
	adc_1[channel] = (adc[channel] ^ 0x800000);         //������ת����ԭ��
	voltage[channel] = (((0.596047*adc_1[channel])-5000000)/1000000);    //�õ�ʵ�ʵ�ѹ

	current[channel] = voltage[channel]/R;

	return current[channel];

}


//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------



