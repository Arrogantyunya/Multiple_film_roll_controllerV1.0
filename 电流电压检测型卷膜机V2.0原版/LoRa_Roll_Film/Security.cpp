#include "Security.h"
#include <libmaple/iwdg.h>
#include "Memory.h"
#include "Command_Analysis.h"
#include "receipt.h"
#include "Private_Timer.h"
#include "fun_periph.h"
#include "LoRa.h"

#define WAIT_TIME       3000

unsigned char SelfCheckTryNum = 0;

volatile bool gCheckStoreParamFlag = false;

/*
 @brief     : 检测LoRa各参数和模块是否正常
 @param     : 无
 @return    : 无
 */
void Check_LoRa_Parameter(void)
{
    LoRa_MHL9LF.Parameter_Init(false);
}

/*
 @brief     : 自检SN码、区域号、工作组号等参数
 @param     : 无
 @return    : 无
 */
void Check_Store_Parameter(void)
{
    unsigned char SN_Temp[9];

    while(!SN.Self_check(SN_Temp))
    {
        Serial.println("SN code check ERROR !!! Applying SN code to server... <Check_Store_Parameter>");
        Message_Receipt.Request_Device_SN_and_Channel();
        LoRa_Command_Analysis.Receive_LoRa_Cmd();
        delay(WAIT_TIME);
        iwdg_feed();
        SelfCheckTryNum++;

        if (SelfCheckTryNum > 50)
        {
            nvic_sys_reset(); 
        }
    }
    SelfCheckTryNum = 0;

    while (!Roll_Operation.Check_Group_Number())
    {
        Serial.println("Group number check ERROR !!! Applying group number to server... <Check_Store_Parameter>");
        Message_Receipt.Request_Set_Group_Number();
        LoRa_Command_Analysis.Receive_LoRa_Cmd();
        delay(WAIT_TIME);
        iwdg_feed();
        SelfCheckTryNum++;

        if (SelfCheckTryNum > 50)
        {
            nvic_sys_reset(); 
        }
    }
    SelfCheckTryNum = 0;

    while (!Roll_Operation.Check_Area_Number())
    {
        Serial.println("Area number check ERROR !!! Applying area number to server... <Check_Store_Parameter>");
        Message_Receipt.Request_Device_SN_and_Channel();
        LoRa_Command_Analysis.Receive_LoRa_Cmd();
        delay(WAIT_TIME);
        iwdg_feed();
        SelfCheckTryNum++;

        if (SelfCheckTryNum > 50)
        {
            nvic_sys_reset(); 
        }
    }
    SelfCheckTryNum = 0;
}

/*
 @brief     : 每隔一段时间，自检LoRa模块、SN码、区域号、工作组号等参数，任一参数自检失败，都会向服务器请求一份重新保存
 @param     : 无
 @return    : 无
 */
void Check_Store_Param_And_LoRa(void)
{
    if (gCheckStoreParamFlag)
    {
        gCheckStoreParamFlag = false;
        Stop_Self_Check_Timing();
        LED_SELF_CHECK_ERROR;

        Check_LoRa_Parameter();
        Check_Store_Parameter();

        Start_Self_Check_Timing();
        LED_RUNNING;
        Serial.println("All parameters check SUCCESS... <Check_Store_Parameter>");
    }
}

