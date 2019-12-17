#include "Private_RTC.h"
#include "User_Clock.h"

/*RCT object*/
RTClock Date(RTCSEL_LSE); 
UTCTimeStruct RtcTime;

date Private_RTC;

/*
 @brief   : 更新本机RTC
 @param   : 新的RTC
 @return  : 无
 */
void date::Update_RTC(unsigned char *buffer)
{
    RtcTime.year = buffer[0] * 100 + buffer[1];
    RtcTime.month = buffer[2];
    RtcTime.day = buffer[3];
    RtcTime.hour = buffer[4];
    RtcTime.minutes = buffer[5];
    RtcTime.seconds = buffer[6];

    UTCTime CurrentSec = osal_ConvertUTCSecs(&RtcTime);
    bkp_enable_writes();
    Date.setTime(CurrentSec);
    bkp_disable_writes();
}

/*
 @brief   : 得到本机的RTC
 @param   : RTC缓存
 @return  : 无
 */
void date::Get_RTC(unsigned char *buffer)
{
  UTCTime CurrentSec = 0;
  CurrentSec = Date.getTime();
  osal_ConvertUTCTime(&RtcTime, CurrentSec);

  buffer[0] = RtcTime.year / 1000;
  buffer[1] = RtcTime.year % 1000;
  buffer[2] = RtcTime.month;
  buffer[3] = RtcTime.day;
  buffer[4] = RtcTime.hour;
  buffer[5] = RtcTime.minutes;
  buffer[6] = RtcTime.seconds;
}