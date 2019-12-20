#ifndef _AT24CXX_H
#define _AT24CXX_H

#include "i2c.h"

/*EEPROM chip*/
#define AT24C01     127
#define AT24C02     255
#define AT24C04     511
#define AT24C08     1023
#define AT24C16     2047
#define AT24C32     4095
#define AT24C64     8191
#define AT24C128    16383
#define AT24C256    32767

/*选择的设备为AT24C02*/
#define EE_TYPE     AT24C02

class AT24Cxx : public I2C {
public:
	void I2C_Init(void);
protected:
	unsigned char AT24CXX_ReadOneByte(unsigned int addr);
	void AT24CXX_WriteOneByte(unsigned int addr, unsigned char dt);
};

extern AT24Cxx AT24C_02;

#endif
