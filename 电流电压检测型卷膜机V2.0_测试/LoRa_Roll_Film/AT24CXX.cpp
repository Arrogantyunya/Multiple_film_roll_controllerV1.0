/************************************************************************************
 *
 * 代码与注释：卢科青
 * 日期：2019/8/27
 *
 * 该文件主要功能是操作EEPROM储存芯片，提供了初始化I2C总线，读取一个字节、写一个字节等函数接口。
 *
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "AT24CXX.h"

AT24Cxx AT24C_02;

/*
 @brief     : 初始化I2C总线
 @param     : 无
 @return    : 无
 */
void AT24Cxx::I2C_Init(void)
{
	Init();
}

/*
 @brief     : 从EEPROM读取一个字节
 @param     : 无
 @return    : 1 byte
 */
unsigned char AT24Cxx::AT24CXX_ReadOneByte(unsigned int addr)
{
	unsigned char Temp = 0;
	Start();

	if (EE_TYPE > AT24C16)
	{
		Send_Byte(0xA0);
		Wait_Ack();
		Send_Byte(addr >> 8);
	}
	else
		Send_Byte(0xA0 + ((addr / 256) << 1)); //设备地址+数据地址

	Wait_Ack();
	Send_Byte(addr % 256);
	Wait_Ack();

	Start();
	Send_Byte(0xA1);
	Wait_Ack();

	Temp = Read_Byte(0);
	NAck();
	Stop();
	return Temp;
}

/*
 @brief     : 写一个字节到EEPROM
 @param     : 1.写入数据地址
			  2.数据值
 @return    : 无
 */
void AT24Cxx::AT24CXX_WriteOneByte(unsigned int addr, unsigned char dt)
{
	Start();

	if (EE_TYPE > AT24C16)
	{
		Send_Byte(0xA0);
		Wait_Ack();
		Send_Byte(addr >> 8);
	}
	else
		Send_Byte(0xA0 + ((addr / 256) << 1));

	Wait_Ack();
	Send_Byte(addr % 256);
	Wait_Ack();

	Send_Byte(dt);
	Wait_Ack();
	Stop();
	delay(10);
}