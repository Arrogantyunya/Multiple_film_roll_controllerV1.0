/************************************************************************************
 * 
 * 代码与注释：卢科青
 * 日期：2019/8/27
 * 
 * 主函数文件。
 * setup部分的功能有：复用调试引脚、开启独立看门狗、设备串口波特率、初始化各类总线
 * 工程模式、注册服务器申请、校正因突发断电的电机开度卷膜等。
 * loop部分的功能有：矫正开度误差、LoRa监听服务器指令、手动卷膜监测、定时自检参数等。
 * 
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "i2c.h"
#include "AT24CXX.h"
#include <libmaple/iwdg.h>
#include "receipt.h"
#include "Command_Analysis.h"
#include "Memory.h"
#include "Motor.h"
#include "LoRa.h"
#include "fun_periph.h"
#include "IAP.h"
#include "Private_Timer.h"
#include "Security.h"
#include "public.h"

/*测试宏，清零上一次开度、本次开度、实时开度*/
#define OPENING_DEBUG         0
/*使能宏，使能写入软件和硬件版本*/
#define SOFT_HARD_VERSION     0

/*功能函数声明*/
void Request_Access_Network(void);
void Project_Debug(void);
void Key_Clear_Current_Value(void);

unsigned char gSN_Code[9] = {0x00}; //设备出厂默认SN码全为0

void setup() 
{
  /*Serial Wire debug only (JTAG-DP disabled, SW-DP enabled)*/
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  /*配置IWDG*/
  iwdg_init(IWDG_PRE_256, 1000);  //6.5ms * 1000 = 6500ms.

  Serial.begin(115200); //USART1, 当使用USART下载程序：USART--->USART1
  LoRa_MHL9LF.BaudRate(9600);
  bkp_init(); //备份寄存器初始化使能

  Motor_Operation.Motor_GPIO_Config();
  Motor_Operation.Direction_Selection(Stop);
  EEPROM_Operation.EEPROM_GPIO_Config();
  Some_Peripheral.Peripheral_GPIO_Config();
  iwdg_feed();

  LoRa_MHL9LF.LoRa_GPIO_Config();
  LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
  /*
   *上电后LoRa模块会发送厂家信息过来
   *这个时候配置的第一个参数在校验回车换行等参数
   *的时候会受到影响。必须先接收厂家信息并清空缓存
   */
  delay(2000);
  gIsHandleMsgFlag = false;
  LoRa_Command_Analysis.Receive_LoRa_Cmd();
  gIsHandleMsgFlag = true;
  iwdg_feed();

  //Initialize LoRa parameter.
  LoRa_MHL9LF.Parameter_Init(false);
  LoRa_Para_Config.Save_LoRa_Config_Flag();

#if SOFT_HARD_VERSION
  Vertion.Save_Software_version(0x00, 0x01);
  Vertion.Save_hardware_version(0x00, 0x01);
#endif

  Roll_Timer_Init();

  Project_Debug();

  SN.Clear_SN_Access_Network_Flag();
  /*Request access network(request gateway to save the device's SN code and channel)*/
  Request_Access_Network();

  while (SN.Self_check(gSN_Code) == false)
  {
    LED_SELF_CHECK_ERROR;
    Serial.println("Verify SN code failed, try to Retrieving SN code...");
    Message_Receipt.Request_Device_SN_and_Channel();
    LoRa_Command_Analysis.Receive_LoRa_Cmd();
    MyDelayMs(3000);
    iwdg_feed();
  }
  Serial.println("SN self_check success...");
  LED_RUNNING;
  iwdg_feed(); 

#if OPENING_DEBUG
  Roll_Operation.Clear_All_Opening_Value();
#endif

  Set_Motor_Status(NOT_INITIALIZED);

  Motor_Operation.Adjust_deviation();
  iwdg_feed();   

  Self_Check_Parameter_Timer_Init();                                                                                                                                                                                                                                                                                                                                                                                                                                
}

void loop() 
{
  iwdg_feed(); 
  LoRa_Command_Analysis.Receive_LoRa_Cmd();

  Motor_Operation.Detect_Manual_Rolling();
  Motor_Operation.Trace_Opening();

  Motor_Operation.Adjust_Opening();

  Check_Store_Param_And_LoRa();

  Key_Clear_Current_Value();
}

/*
 @brief   : 检测是否已经注册到服务器成功，如果没有注册，则配置相关参数为默认参数，然后注册到服务器。
            没有注册成功，红灯1每隔500ms闪烁。
            Checks whether registration with the server was successful, and if not, 
            configures the relevant parameters as default parameters and registers with the server.
            Failing registration, red light flashes every 500ms.
 @para    : None
 @return  : None
 */
void Request_Access_Network(void)
{
  if (SN.Verify_SN_Access_Network_Flag() == false)
  {
    gAccessNetworkFlag = false;

    if (SN.Save_SN_Code(gSN_Code) && SN.Save_BKP_SN_Code(gSN_Code))
      Serial.println("Write SN success...");

    if(Roll_Operation.Clear_Area_Number() && Roll_Operation.Clear_Group_Number())
      Serial.println("Already Clear area number and grouop number...");

    unsigned char Default_WorkGroup[5] = {0x01, 0x00, 0x00, 0x00, 0x00};
    if(Roll_Operation.Save_Group_Number(Default_WorkGroup))
      Serial.println("Save gourp number success...");

   LED_NO_REGISTER;
  }
  while (SN.Verify_SN_Access_Network_Flag() == false)
  {
    iwdg_feed();
    if (digitalRead(SW_FUN1) == LOW)
    {
      MyDelayMs(5000);
      iwdg_feed();
      if (digitalRead(SW_FUN1) == LOW)
      {
        Some_Peripheral.Key_Buzz(600);
        Message_Receipt.Report_General_Parameter();

        while (digitalRead(SW_FUN1) == LOW)
          iwdg_feed();
      }
    }
    LoRa_Command_Analysis.Receive_LoRa_Cmd();
  } 
  gAccessNetworkFlag = true;
}

/*
 @brief   : 工程模式。用于在单机工作的情况下。手动卷膜，可以测量脉冲数、电机电流、上下限位等。
            通过按键1，可以测试重置行程。
 @para    : 无
 @return  : 无
 */
void Project_Debug(void)
{
  if (digitalRead(SW_FUN1) == LOW)
  {
    MyDelayMs(100);
    if (digitalRead(SW_FUN1) == LOW)
    {
      Some_Peripheral.Key_Buzz(600);
      unsigned char PrintInterval = 0;
      Serial.println("<This is engineering mode, not normal mode>");

      while (1)
      {
        if (PrintInterval++ >= 3)
        {
          PrintInterval = 0;
          Serial.println("<This is engineering mode, not normal mode>");
        }

        iwdg_feed();
        MyDelayMs(3000);
        iwdg_feed();

        if (digitalRead(SW_FUN2) == LOW)
        {
          MyDelayMs(100);
          if (digitalRead(SW_FUN2) == LOW)
          {
            Some_Peripheral.Key_Buzz(600);
            //Motor_Operation.Reset_Motor_Route();

            MANUAL_ROLL_OFF;
            detachInterrupt(DEC_MANUAL_DOWN_PIN);
            detachInterrupt(DEC_MANUAL_UP_PIN);
            Motor_Operation.Reset_Motor_Route();
            attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
            attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
            MANUAL_ROLL_ON;

            while (digitalRead(SW_FUN2) == LOW)
              iwdg_feed();
          }
        }
      }
    }
    while (digitalRead(SW_FUN1) == LOW)
      iwdg_feed();
  }
}

void Key_Clear_Current_Value(void)
{
  if (digitalRead(SW_FUN2) == LOW)
  {
    iwdg_feed();
    MyDelayMs(5000);
    iwdg_feed();
    if (digitalRead(SW_FUN2) == LOW)
    {
      Some_Peripheral.Key_Buzz(600);
      Serial.println("Prepare clear current threshold...");
      Roll_Operation.Clear_Current_Flag();
    }
    while (digitalRead(SW_FUN2) == LOW)
      iwdg_feed();
  }
}

void Key_Reset_LoRa_Parameter(void)
{
  if (digitalRead(SW_FUN1) == LOW)
  {
      MyDelayMs(100);
      if (digitalRead(SW_FUN1) == LOW)
      {
        Some_Peripheral.Key_Buzz(600);
        MyDelayMs(3000);
        iwdg_feed();
        if (digitalRead(SW_FUN2) == LOW)
        {
          MyDelayMs(100);
          if (digitalRead(SW_FUN2) == LOW)
          {
            Some_Peripheral.Key_Buzz(600);
            LoRa_Para_Config.Clear_LoRa_Config_Flag();
            Serial.println("Clear LoRa configuration flag SUCCESS... <Key_Reset_LoRa_Parameter>");
            iwdg_feed();
          }
        }
      }
    }
}