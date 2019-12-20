#include "i2c.h"

/*
 @brief     : 初始化I2C总线
 @para      : 无
 @return    : 无
 */
void I2C::Init(void)
{
	pinMode(SCL, OUTPUT);
	pinMode(SDA, OUTPUT);
	I2C_SCL_H;
	I2C_SDA_H;
}

/*
 @brief     : I2C数据线配置输出模式
 @para      : 无
 @return    : 无
 */
void I2C::SDA_Out(void)
{
	pinMode(SDA, OUTPUT);
}

/*
 @brief     : I2C数据线配置输入模式
 @para      : 无
 @return    : 无
 */
void I2C::SDA_In(void)
{
	pinMode(SDA, INPUT_PULLUP);
}

/*
 @brief     : I2C开始传输数据时序
 @para      : 无
 @return    : 无
 */
void I2C::Start(void)
{
	SDA_Out();
	I2C_SDA_H;
	I2C_SCL_H;
	delayMicroseconds(5);
	I2C_SDA_L;
	delayMicroseconds(6);
	I2C_SCL_L;
}

/*
 @brief     : I2C结束传输数据时序
 @para      : 无
 @return    : 无
 */
void I2C::Stop(void)
{
	SDA_Out();
	I2C_SCL_L;
	I2C_SDA_L;
	I2C_SCL_H;
	delayMicroseconds(6);
	I2C_SDA_H;
	delayMicroseconds(6);
}

/*
 @brief     : 主机数据线拉低应答
 @para      : 无
 @return    : 无
 */
void I2C::Ack(void)
{
	I2C_SCL_L;
	SDA_Out();
	I2C_SDA_L;
	delayMicroseconds(2);
	I2C_SCL_H;
	delayMicroseconds(5);
	I2C_SCL_L;
}

/*
 @brief     : 主机数据线拉高非应答
 @para      : 无
 @return    : 无
 */
void I2C::NAck(void)
{
	I2C_SCL_L;
	SDA_Out();
	I2C_SDA_H;
	delayMicroseconds(2);
	I2C_SCL_H;
	delayMicroseconds(5);
	I2C_SCL_L;
}

/*
 @brief     : 等待从机应答， 从机返回1接收应答失败，返回0接收应答成功
 @para      : 无
 @return    : 等待应答结果
 */
unsigned char I2C::Wait_Ack(void)
{
	unsigned char tempTime = 0;
	SDA_In();
	I2C_SDA_H;
	delayMicroseconds(1);
	I2C_SCL_H;
	delayMicroseconds(1);

	while (digitalRead(SDA))
	{
		tempTime++;
		if (tempTime > 250) //等待从机返回0失败
		{
			Stop();
			return 1;
		}
	}
	I2C_SCL_L;
	return 0;
}

/*
 @brief     : I2C发送一个字节
 @para      : 1byte
 @return    : 无
 */
void I2C::Send_Byte(unsigned char data)
{
	unsigned char i = 0;
	SDA_Out();
	I2C_SCL_L; //拉低时钟线，允许数据线上电平变化

	for (i = 0; i < 8; i++)
	{
		(data & 0x80) > 0 ? I2C_SDA_H : I2C_SDA_L;//从一个字节的高位开始传送
		data <<= 1;
		I2C_SCL_H; //时钟线拉高，这时数据线电平不能变化，让从机读取线上的电平
		delayMicroseconds(2);
		I2C_SCL_L;
		delayMicroseconds(2);
	}
}

/*
 @brief     : I2C读取一个字节
 @para      : 选择应答或非应答
 @return    : 1 byte
 */
unsigned char I2C::Read_Byte(unsigned char ack)
{
	unsigned char i = 0, receive = 0;
	SDA_In();

	for (i = 0; i < 8; i++)
	{
		I2C_SCL_L;
		delayMicroseconds(2);
		I2C_SCL_H; //拉高时钟线，去读从机回过来的数据
		receive <<= 1;

		if (digitalRead(SDA))
			receive++;
		delayMicroseconds(1);
	}
	ack == 0 ? NAck() : Ack();

	return receive;
}