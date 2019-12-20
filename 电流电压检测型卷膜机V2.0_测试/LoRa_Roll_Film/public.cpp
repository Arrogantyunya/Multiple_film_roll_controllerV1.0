#include "public.h"
#include "Arduino.h"

TypeConv Type_Conv;

/*
 @brief     : HEX字节转DEC字节
 @param     : 1 byte
 @return    : 1 byte
 */
unsigned char TypeConv::Hex_To_Dec(unsigned char hex)
{
	unsigned char Temp1, Temp2;
	Temp1 = hex / 10;
	Temp2 = hex % 10;
	Temp2 = Temp1 * 16 + Temp2;
	return Temp2;
}

/*
 @brief     : HEX字节转DEC字节
 @param     : 1 byte
 @return    : 无
 */
void TypeConv::Hex_To_Dec(unsigned char *hex)
{
	unsigned char Temp1, Temp2;
	Temp1 = *hex / 10;
	Temp2 = *hex % 10;
	*hex = Temp1 * 16 + Temp2;
}

/*
 @brief     : DEC字节转HEX字节
 @param     : 1 byte
 @return    : 1 byte
 */
unsigned char TypeConv::Dec_To_Hex(unsigned char dec)
{
	unsigned char Temp1, Temp2;
	Temp1 = dec / 16;
	Temp2 = dec % 16;
	Temp2 = Temp1 * 10 + Temp2;
	return Temp2;
}

/*
 @brief     : DEC字节转HEX字节
 @param     : 1 byte
 @return    : 无
 */
void TypeConv::Dec_To_Hex(unsigned char *dec)
{
	unsigned char Temp1, Temp2;
	Temp1 = *dec / 16;
	Temp2 = *dec % 16;
	*dec = Temp1 * 10 + Temp2;
}

void MyDelayMs(unsigned int d)
{
	unsigned long WaitNum = millis();

	while (millis() <= (WaitNum + d))
	{
		/*
		  *millis()函数自程序运行，开始计时，大约50天后溢出到0，再计时。
		  *为了防止发生溢出后，延时函数出错，当检测到毫秒计时小于赋值的毫秒数，
		  *再次重新赋值，重新判断。
		 */
		if (millis() < WaitNum)
			WaitNum = millis();
	}
}