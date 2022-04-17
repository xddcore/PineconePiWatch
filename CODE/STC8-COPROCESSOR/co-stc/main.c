#include "stc8g.h"
#include "intrins.h"
/*
0x01 ����ѹ
0x02 ����ѹ
0x03 ����
0x04 �¶ȸ�λ
0x05 �¶ȵ�λ
0x11 �������߰�λ
0x12 �������߰�λ

0x07 ������
0x08 tft����
0x09 sensor ��Դ
0x10 stc8g ˯��
*/
#define Systolic_bit 0x01
#define Diastolic_bit 0x02
#define heart_bit 0x03
#define temp_h 0x04
#define temp_l 0x05
#define wave_h 0x11
#define wave_l 0x12

#define status 0x07
#define tft_bit 0x08
#define sensor_bit 0x09
#define stc_sleep_bit 0x10
sbit 		sensor_power_control = P3^3;
sbit 		tft_power_control = P3^2;

#define MAIN_Fosc		11059200L	//������ʱ��
typedef unsigned char u8;
/*MK0908�����*/
unsigned char get_data[] ={0xFD,0X00,0X00,0X00,0X00,0X00};//��ȡ����Ѫѹ�¶�
unsigned char get_wave[] ={0xFC,0X00,0X00,0X00,0X00,0X00};//��������ȡ
unsigned char get_status[] ={0xF8,0X00,0X00,0X00,0X00,0X00};//ģ�鹤��״̬��ȡ

unsigned char receive_buffer[6];
unsigned char data_i = 0;//�ֽڼ���
unsigned char receive_success = 0;//�������


bit isda;                                       //�豸��ַ��־
bit isma;                                       //�洢��ַ��־
unsigned char addr;
unsigned char pdata buffer[256];

void Delay_ms(unsigned int ms)		//@11.0592MHz
{
	unsigned char i, j,k;
	for(k=0;k<ms;k++)
	{
		i = 15;
		j = 90;
		do
		{
			while (--j);
		} while (--i);
	}
}

void I2C_Isr() interrupt 24
{
    if (I2CSLST & 0x40)
    {
        I2CSLST &= ~0x40;                       //����START�¼�
    }
    else if (I2CSLST & 0x20)
    {
        I2CSLST &= ~0x20;                       //����RECV�¼�
        if (isda)
        {
            isda = 0;                           //����RECV�¼���RECV DEVICE ADDR��
        }
        else if (isma)
        {
            isma = 0;                           //����RECV�¼���RECV MEMORY ADDR��
            addr = I2CRXD;
            I2CTXD = buffer[addr];
        }
        else
        {
            buffer[addr++] = I2CRXD;            //����RECV�¼���RECV DATA��
        }
    }
    else if (I2CSLST & 0x10)
    {
        I2CSLST &= ~0x10;                       //����SEND�¼�
        if (I2CSLST & 0x02)
        {
            I2CTXD = 0xff;
        }
        else
        {
            I2CTXD = buffer[++addr];
        }
    }
    else if (I2CSLST & 0x08)
    {
        I2CSLST &= ~0x08;                       //����STOP�¼�
        isda = 1;
        isma = 1;
    }
   // _pop_(P_SW2);
}
void mk0809_send_command(unsigned char command[])
{
	unsigned char i = 0;
	for(i=0;i<6;i++)
	{
		SBUF = command[i];//����MK0908����
		while(!TI);
		TI = 0;
	}
}
void gpio_init()
{
		P3M0 = 0x00;                                //����P3.0~P3.7Ϊ˫���ģʽ
    P3M1 = 0x00;
		P5M0 = 0x00;                                //����P5.0~P5.7Ϊ˫���ģʽ
    P5M1 = 0x00;
		P_SW2 = 0x10;                               //IIC�����л�
		P_SW2 |= 0x80;                              //����д��IIC��ؼĴ���
}
void UART1_Isr() interrupt 4
{
    if (TI)
    {
      TI = 0;                                 //���жϱ�־
    }
    if (RI)
    {
			receive_buffer[data_i] = SBUF;
			data_i++;
			if(data_i>=6)
			{
				data_i = 0;
				receive_success = 1;
			}
      RI = 0;                                 //���жϱ�־
    }
}

void main()
{		
		u8 time_over = 0;
		u8 data_iic = 0x55;
		gpio_init();
		/*UART-115200*/
		SCON = 0x50;		//8λ����,�ɱ䲨����
		AUXR |= 0x40;		//��ʱ��1ʱ��ΪFosc,��1T
		AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
		TMOD &= 0x0F;		//�趨��ʱ��1Ϊ16λ�Զ���װ��ʽ
		TL1 = 0xE8;		//�趨��ʱ��ֵ
		TH1 = 0xFF;		//�趨��ʱ��ֵ
		ET1 = 0;		//��ֹ��ʱ��1�ж�
		TR1 = 1;		//������ʱ��1
		ES=1;
		/*iic*/
	  I2CCFG = 0x81;                              //ʹ��I2C�ӻ�ģʽ
    I2CSLADR = 0x5a;                            //���ôӻ��豸��ַΪ5A
    I2CSLST = 0x00;
    I2CSLCR = 0x78;                             //ʹ�ܴӻ�ģʽ�ж�
    EA = 1;
    isda = 1;                                   //�û�������ʼ��
    isma = 1;
    addr = 0;
    I2CTXD = buffer[addr];
	  buffer[tft_bit] = 1;//����Ĭ�Ͽ���	
		buffer[sensor_bit] = 0;//��������ԴĬ�Ϲر�
		buffer[stc_sleep_bit] = 0;//stc����˯��ģʽ�ر�
		
		buffer[Systolic_bit] = 0;
		buffer[Diastolic_bit] = 0;
		buffer[heart_bit] = 0;
		buffer[temp_h] = 0;
		buffer[temp_l] = 0;
		buffer[wave_h] = 0;
		buffer[wave_l] = 0;
		buffer[status] = 0;
    while (1)
		{	
			if(buffer[tft_bit] == 1)
			{
				tft_power_control = 1;
			}
			else
			{
				tft_power_control = 0;
			}
			
			if(buffer[sensor_bit] == 1)
			{
				sensor_power_control = 1;
			}
			else
			{
				sensor_power_control = 0;
			}
			
			if(buffer[stc_sleep_bit] == 1)
			{
					PCON = 0X02; //�������ģʽ
			}
			else
			{
				
			}
			/*������γ���ָ��Խ�������(�ؼҵ�)*/
			if(buffer[sensor_bit] == 1)
			{
					mk0809_send_command(get_data);//��ȡ����
					//while(!(receive_success==1));//�ȴ����������ȥ��������
					Delay_ms(200);
					time_over = 0; //�����ʱ������־
					if(receive_success == 1)
					{
						buffer[Systolic_bit]=receive_buffer[1];
						buffer[Diastolic_bit]=receive_buffer[2];
						buffer[heart_bit]=receive_buffer[3];
						buffer[temp_h]=receive_buffer[4];
						buffer[temp_l]=receive_buffer[5];
						receive_success=0;
					}
					
					/*
					mk0809_send_command(get_wave);//��ȡ��
					Delay_ms(200);
					//while(!(receive_success==1));//�ȴ����������ȥ��������
					time_over = 0; //�����ʱ������־
					if(receive_success == 1)
					{
						buffer[wave_h]=receive_buffer[2];
						buffer[wave_l]=receive_buffer[3];
						receive_success=0;
					}
					
					
					mk0809_send_command(get_status);//�����״̬
					Delay_ms(200);
					//while(!(receive_success==1));//�ȴ����������ȥ��������
					time_over = 0; //�����ʱ������־
					if(receive_success == 1)
					{
						buffer[status]=receive_buffer[3];
						receive_success=0;
					}
					*/
					Delay_ms(50);//�����ӳ�50ms
			}
	}
}