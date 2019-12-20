
#ifndef _I2C_H
#define _I2C_H

#include <Arduino.h>

#define SDA     PB7
#define SCL     PB6

#define I2C_SDA_H       digitalWrite(SDA, HIGH)
#define I2C_SDA_L       digitalWrite(SDA, LOW)
#define I2C_SCL_H       digitalWrite(SCL, HIGH)
#define I2C_SCL_L       digitalWrite(SCL, LOW)

class I2C {
protected:
	void Init(void);
	void SDA_Out(void);
	void SDA_In(void);
	void Start(void);
	void Stop(void);
	void Ack(void);
	void NAck(void);
	unsigned char Wait_Ack(void);
	void Send_Byte(unsigned char data);
	unsigned char Read_Byte(unsigned char ack);
};

#endif

