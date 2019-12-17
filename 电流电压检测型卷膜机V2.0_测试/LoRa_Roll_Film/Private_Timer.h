#ifndef _PRIVATE_TIMER_H
#define _PRIVATE_TIMER_H

void Roll_Timer_Init(void);
void Self_Check_Parameter_Timer_Init(void);
void Start_Roll_Timing(void);
void Start_Self_Check_Timing(void);
void Stop_Roll_Timing(void);
void Stop_Self_Check_Timing(void);
void Timer2_Interrupt(void);
void Timer3_Interrupt(void);


#endif