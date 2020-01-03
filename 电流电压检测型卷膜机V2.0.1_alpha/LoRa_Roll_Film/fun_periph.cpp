/************************************************************************************
 * 
 * 代码与注释：卢科青
 * 日期：2019/3/17
 * 
 * 配置一些外设功能。如蜂鸣器、LED灯、按钮等。这些外设用来作为程序调试和人机交互。
 * 头文件中提供了各个类的公共接口。
 * 
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "fun_periph.h"
#include <libmaple/iwdg.h>
#include "pwm.h"

Some_Peripherals Some_Peripheral;

LED Which_LED;
unsigned int LED_Freq = 0;  //LED_Freq * 100ms

/*
 @brief   : 设置蜂鸣器，LED，功能按钮等引脚 
 @param   : 无
 @return  : 无
 */
void Some_Peripherals::Peripheral_GPIO_Config(void)
{
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(SW_FUN1, INPUT);
  pinMode(SW_FUN2, INPUT);
  GREEN1_OFF;
  GREEN2_OFF;
  RED1_OFF;
  RED2_OFF;
  BUZZ_OFF;
}

/*
 @brief   : 选择要显示的LED灯（颜色），和闪烁的频率
 @param   : 1.哪一个LED
            2.频率值（100ms/次）
 @return  : 无
 */
void Some_Peripherals::LED_Display(LED which_led, unsigned char freq)
{ 
  Which_LED = which_led;
  LED_Freq = freq;
  Timer4.setPeriod(100000); // in microseconds, 100ms
  Timer4.attachCompare1Interrupt(LED_Interrupt); 
  Timer4.setCount(0);
}

/*
 @brief   : 暂停所有LED闪烁，同时暂停定时器4
 @param   : 无
 @return  : 无
*/
void Some_Peripherals::Stop_LED(void)
{
  Timer4.pause();
  GREEN1_OFF;
  GREEN2_OFF;
  RED1_OFF;
  RED2_OFF;
}

/*
 @brief   : 恢复定时器4计数，LED开始闪烁
 @param   : 无
 @return  : 无
 */
void Some_Peripherals::Start_LED(void)
{
  Timer4.resume();
}

/*
 @brief   : 蜂鸣器按照固定频率发出响声，表明按键被按下。
 @param   : d ---> cycle time, 0.7ms one time.
 @return  : 无
 */
void Some_Peripherals::Key_Buzz(unsigned int d) //700ms  1000 times.
{
  for (unsigned int i = 0; i < d; i++)
  {
    BUZZ_ON;
    delayMicroseconds(500);
    BUZZ_OFF;
    delayMicroseconds(200);
  }
}

/*
 @brief   : LED闪烁中断函数
 @param   : 无
 @return  : 无
 */
void LED_Interrupt(void)
{
  static unsigned char LED_StatusFlag = 0; //LED灯状态标志位
  static unsigned int LED_Num = 0;
  LED_Num++;
  if (LED_Num >= LED_Freq)
  {
      LED_Num = 0;
      LED_StatusFlag = ~LED_StatusFlag; 
      switch (Which_LED)
      {
        case GREEN1 : LED_StatusFlag == 0 ? digitalWrite(LED1_PIN, HIGH) : digitalWrite(LED1_PIN, LOW);
                      digitalWrite(LED2_PIN, LOW); digitalWrite(LED3_PIN, LOW); digitalWrite(LED4_PIN, LOW); break;

        case RED1   : LED_StatusFlag == 0 ? digitalWrite(LED2_PIN, HIGH) : digitalWrite(LED2_PIN, LOW);
                      digitalWrite(LED1_PIN, LOW); digitalWrite(LED3_PIN, LOW); digitalWrite(LED4_PIN, LOW); break;

        case GREEN2 : LED_StatusFlag == 0 ? digitalWrite(LED3_PIN, HIGH) : digitalWrite(LED3_PIN, LOW);
                      digitalWrite(LED1_PIN, LOW); digitalWrite(LED2_PIN, LOW); digitalWrite(LED4_PIN, LOW); break;

        case RED2   : LED_StatusFlag == 0 ? digitalWrite(LED4_PIN, HIGH) : digitalWrite(LED4_PIN, LOW);
                      digitalWrite(LED1_PIN, LOW); digitalWrite(LED2_PIN, LOW); digitalWrite(LED3_PIN, LOW); break;
      }
  }
}

