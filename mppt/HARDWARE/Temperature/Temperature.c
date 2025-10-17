#include "board.h"

const float Rp=10000.0; //10K
const float T2 = (273.15+25.0);;//T2
const float Bx = 3950.0;//B
const float Ka = 273.15;

/*********************************************************************************************************
*	�� �� ��: Get_Rntc
*	����˵��: ��ȡ���������Rֵ
*	��    ��: ��
*	�� �� ֵ: ���������Rֵ
*********************************************************************************************************/
float  Get_Rntc(void)
{
    float Vcc = 3.3;
    u16 Rm=10;
    float volt;
    float Rtem; 

    //��ȡNTC�ĵ�ѹֵ
    volt = Measure_Voltage(0);

    //�������ֵ
    Rtem = (volt*(float)Rm)/(Vcc-volt);

    return Rtem;
}

/*================================================================================
*	�� �� ��: Get_Temp
*	����˵��: ���¶ȴ�ֵ�õ��¶Ⱦ�ֵ
*	��    ��: ��
*	�� �� ֵ: temp	���¶�ֵ
================================================================================*/
float Get_Temp(void)
{
	float Rt;
	float temp;
	
	Rt = Get_Rntc();
	
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

