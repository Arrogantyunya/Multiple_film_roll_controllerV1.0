#include "Private_Timer.h"
#include "fun_periph.h"
#include "Motor.h"
#include "Security.h"
#include <Arduino.h>

/*Timer timing time*/
#define TIMER_NUM             1000000L * 1 //1S
#define CHECK_TIMER_NUM       1000000L

volatile static unsigned int gSelfCheckNum;

/*
 @brief   : 使用定时器2初始化卷膜行程计时参数
 @param   : 无
 @return  : 无
 */
void Roll_Timer_Init(void)
{
  Timer2.setPeriod(TIMER_NUM); // in microseconds，1S
  Timer2.attachCompare1Interrupt(Timer2_Interrupt);   
  Timer2.setCount(0);
  Timer2.pause(); 
}

/*
 @brief   : 使用定时器3初始化自检参数功能自检周期
 @param   : 无
 @return  : 无
 */
void Self_Check_Parameter_Timer_Init(void)
{
  Timer3.setPeriod(TIMER_NUM); // in microseconds，1S
  Timer3.attachCompare1Interrupt(Timer3_Interrupt);   
  Timer3.setCount(0);
}

/*
 @brief   : 开始卷膜计时
 @param   : 无
 @return  : 无
 */
void Start_Roll_Timing(void)
{
  gRollingTime = 0;
  gRollingTimeVarFlag = false;
  Timer2.setCount(0);
  Timer2.resume();
}

/*
 @brief   : 开始自检周期计时
 @param   : 无
 @return  : 无
 */
void Start_Self_Check_Timing(void)
{
  Timer3.resume();
  Timer3.setCount(0);
}

/*
 @brief   : 停止卷膜计时
 @param   : 无
 @return  : 无
 */
void Stop_Roll_Timing(void)
{
  Timer2.pause();
}

/*
 @brief   : 停止自检周期计时
 @param   : 无
 @return  : 无
 */
void Stop_Self_Check_Timing(void)
{
  Timer3.pause();
}

/*
 @brief   : 卷膜计时定时器2计时中断处理函数
 @param   : 无
 @return  : 无
 */
void Timer2_Interrupt(void)
{
  gRollingTime++;
  gRollingTimeVarFlag = true;
}

/*
 @brief   : 自检计时定时器3计时中断处理函数
 @param   : 无
 @return  : 无
 */
void Timer3_Interrupt(void)
{
  gSelfCheckNum++;
  if (gSelfCheckNum >= 14400) //4 hours
  {
      gSelfCheckNum = 0;
      gCheckStoreParamFlag = true;
  }
}