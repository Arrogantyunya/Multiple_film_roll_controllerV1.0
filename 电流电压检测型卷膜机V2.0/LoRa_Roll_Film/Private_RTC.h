#ifndef _PRIVATE_RTC_H
#define _PRIVATE_RTC_H

#include <RTClock.h>

class date{
public:
    void Update_RTC(unsigned char *buffer);
    void Get_RTC(unsigned char *buffer);
};

extern date Private_RTC;

#endif