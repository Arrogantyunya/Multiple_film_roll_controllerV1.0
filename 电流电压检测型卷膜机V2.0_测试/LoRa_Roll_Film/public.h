#ifndef _PUBLIC_H
#define _PUBLIC_H

class TypeConv {
public:
	unsigned char Hex_To_Dec(unsigned char hex);
	void Hex_To_Dec(unsigned char *hex);
	unsigned char Dec_To_Hex(unsigned char dec);
	void Dec_To_Hex(unsigned char *dec);
};

void MyDelayMs(unsigned int d);

extern TypeConv Type_Conv;

#endif