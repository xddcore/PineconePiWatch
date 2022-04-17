#include "stc8g.h"
#include "intrins.h"
/*
0x01 收缩压
0x02 舒张压
0x03 心率
0x04 温度高位
0x05 温度低位
0x11 脉搏波高八位
0x12 脉搏波高八位

0x07 佩戴检测
0x08 tft背光
0x09 sensor 电源
0x10 stc8g 睡眠
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

#define MAIN_Fosc		11059200L	//定义主时钟
typedef unsigned char u8;
/*MK0908命令表*/
unsigned char get_data[] ={0xFD,0X00,0X00,0X00,0X00,0X00};//获取心率血压温度
unsigned char get_wave[] ={0xFC,0X00,0X00,0X00,0X00,0X00};//脉搏波读取
unsigned char get_status[] ={0xF8,0X00,0X00,0X00,0X00,0X00};//模块工作状态读取

unsigned char receive_buffer[6];
unsigned char data_i = 0;//字节计数
unsigned char receive_success = 0;//接收完成


bit isda;                                       //设备地址标志
bit isma;                                       //存储地址标志
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
        I2CSLST &= ~0x40;                       //处理START事件
    }
    else if (I2CSLST & 0x20)
    {
        I2CSLST &= ~0x20;                       //处理RECV事件
        if (isda)
        {
            isda = 0;                           //处理RECV事件（RECV DEVICE ADDR）
        }
        else if (isma)
        {
            isma = 0;                           //处理RECV事件（RECV MEMORY ADDR）
            addr = I2CRXD;
            I2CTXD = buffer[addr];
        }
        else
        {
            buffer[addr++] = I2CRXD;            //处理RECV事件（RECV DATA）
        }
    }
    else if (I2CSLST & 0x10)
    {
        I2CSLST &= ~0x10;                       //处理SEND事件
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
        I2CSLST &= ~0x08;                       //处理STOP事件
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
		SBUF = command[i];//发送MK0908命令
		while(!TI);
		TI = 0;
	}
}
void gpio_init()
{
		P3M0 = 0x00;                                //设置P3.0~P3.7为双向口模式
    P3M1 = 0x00;
		P5M0 = 0x00;                                //设置P5.0~P5.7为双向口模式
    P5M1 = 0x00;
		P_SW2 = 0x10;                               //IIC引脚切换
		P_SW2 |= 0x80;                              //允许写入IIC相关寄存器
}
void UART1_Isr() interrupt 4
{
    if (TI)
    {
      TI = 0;                                 //清中断标志
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
      RI = 0;                                 //清中断标志
    }
}

void main()
{		
		u8 time_over = 0;
		u8 data_iic = 0x55;
		gpio_init();
		/*UART-115200*/
		SCON = 0x50;		//8位数据,可变波特率
		AUXR |= 0x40;		//定时器1时钟为Fosc,即1T
		AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
		TMOD &= 0x0F;		//设定定时器1为16位自动重装方式
		TL1 = 0xE8;		//设定定时初值
		TH1 = 0xFF;		//设定定时初值
		ET1 = 0;		//禁止定时器1中断
		TR1 = 1;		//启动定时器1
		ES=1;
		/*iic*/
	  I2CCFG = 0x81;                              //使能I2C从机模式
    I2CSLADR = 0x5a;                            //设置从机设备地址为5A
    I2CSLST = 0x00;
    I2CSLCR = 0x78;                             //使能从机模式中断
    EA = 1;
    isda = 1;                                   //用户变量初始化
    isma = 1;
    addr = 0;
    I2CTXD = buffer[addr];
	  buffer[tft_bit] = 1;//背光默认开启	
		buffer[sensor_bit] = 0;//传感器电源默认关闭
		buffer[stc_sleep_bit] = 0;//stc掉电睡眠模式关闭
		
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
					PCON = 0X02; //进入掉电模式
			}
			else
			{
				
			}
			/*以下这段程序指针越界会跑死(回家调)*/
			if(buffer[sensor_bit] == 1)
			{
					mk0809_send_command(get_data);//获取数据
					//while(!(receive_success==1));//等待接收完成再去解析数据
					Delay_ms(200);
					time_over = 0; //清除超时跳过标志
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
					mk0809_send_command(get_wave);//获取波
					Delay_ms(200);
					//while(!(receive_success==1));//等待接收完成再去解析数据
					time_over = 0; //清除超时跳过标志
					if(receive_success == 1)
					{
						buffer[wave_h]=receive_buffer[2];
						buffer[wave_l]=receive_buffer[3];
						receive_success=0;
					}
					
					
					mk0809_send_command(get_status);//获佩戴状态
					Delay_ms(200);
					//while(!(receive_success==1));//等待接收完成再去解析数据
					time_over = 0; //清除超时跳过标志
					if(receive_success == 1)
					{
						buffer[status]=receive_buffer[3];
						receive_success=0;
					}
					*/
					Delay_ms(50);//总体延迟50ms
			}
	}
}