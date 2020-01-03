/************************************************************************************
 * 
 * 代码与注释：卢科青
 * 日期：2019/8/27
 * 
 * 该文件的作用是接收服务器通过LoRa无线模块发送过来的指令数据，然后解析这些指令。指令包括通用指令
 * （绑定SN，设置区域号，设置工作组号，查询本机状态等），私有指令（卷膜机重置行程，设置开度卷膜
 * 设置卷膜工作阈值等）。接收的指令都要校验CRC8，有些指令要校验区域号或工作组号本机才会执行相应
 * 功能。
 * 头文件中提供了各个类的公共接口。
 * 
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "Command_Analysis.h"
#include <libmaple/iwdg.h>
#include "LoRa.h"
#include "User_CRC8.h"
#include "Memory.h"
#include "Motor.h"
#include "receipt.h"

Command_Analysis LoRa_Command_Analysis;

unsigned char gReceiveCmd[128];   //接收LoRa数据缓存
unsigned char gReceiveLength;     //接收LoRa数据长度
bool gAccessNetworkFlag = true;   //是否已经注册到服务器标志位
bool gStopWorkFlag = false;       //是否要强制停止标志位
bool gMassCommandFlag = false;    //接收的消息是否是群发标志位

bool gIsHandleMsgFlag = true;     //是否接收到LoRa消息然后解析处理，还是只接收不解析处理（刚上电LoRa模块发的厂家信息）

/*
 @brief   : 从网关接收LoRa数据（网关 ---> 本机），接受的指令有通用指令和本设备私有指令。
            每条指令以0xFE为帧头，0x0D 0x0A 0x0D 0x0A 0x0D 0x0A，6个字节为帧尾。最大接受指令长度为128字节，超过将清空接收的数据和长度。
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Receive_LoRa_Cmd(void)
{
  unsigned char EndNum = 0;  //帧尾数量计数值
  bool EndNumFlag = false;  //检测到第一个帧尾标志位
  bool ReceiveEndFlag = false;  //正确接收到一笔数据标志位
  unsigned char FrameHeadDex = 0;
  gReceiveLength = 0;
  iwdg_feed();

  while (LoRa_Serial.available() > 0)
  {
    iwdg_feed();
    gReceiveCmd[gReceiveLength++] = LoRa_Serial.read();
    delay(3);
    Serial.print(gReceiveCmd[gReceiveLength - 1], HEX);
    Serial.print(" ");

    /*数据超出可以接收的范围*/
    if (gReceiveLength >= 128)
    {
      gReceiveLength = 0;
      memset(gReceiveCmd, 0x00, sizeof(gReceiveCmd));
    }

    //记录帧头所在接收的数据包里的位置（因为噪音的干扰，第一个字节可能不是帧头）
    if (gReceiveCmd[gReceiveLength - 1] == 0xFE)
    {
      FrameHeadDex = gReceiveLength - 1;
    }

    /*验证帧尾: 0D 0A 0D 0A 0D 0A*/
    if (EndNumFlag == false)
    {
      if (gReceiveCmd[gReceiveLength - 1] == 0x0D)  //如果检测到第一个帧尾
        EndNumFlag = true;
    }
    /*接收校验剩余的帧尾*/
    if (EndNumFlag == true)
    {
      switch (EndNum)
      {
        case 0 : gReceiveCmd[gReceiveLength - 1] == 0x0A ? EndNum += 1: EndNum = 0; break;
        case 1 : gReceiveCmd[gReceiveLength - 1] == 0x0D ? EndNum += 1: EndNum = 0; break;
        case 2 : gReceiveCmd[gReceiveLength - 1] == 0x0A ? EndNum += 1: EndNum = 0; break;
        case 3 : gReceiveCmd[gReceiveLength - 1] == 0x0D ? EndNum += 1: EndNum = 0; break;
        case 4 : gReceiveCmd[gReceiveLength - 1] == 0x0A ? EndNum += 1: EndNum = 0; break;
      }
    }
    if (EndNum == 5)  //帧尾校验正确
    {
      EndNum = 0;
      EndNumFlag = false;
      ReceiveEndFlag = true;
      Serial.println("Get frame end... <Receive_LoRa_Cmd>");
      break;
    }
  }

  if (ReceiveEndFlag)
  {
    Serial.println("Parsing LoRa command... <Receive_LoRa_Cmd>");
    ReceiveEndFlag = false;

    if (FrameHeadDex != 0)  //第一个字节不是0xFE，说明有噪音干扰，重新从0xFE开始组合出一帧
    {
      unsigned char HeadStart = FrameHeadDex;
      for (unsigned char i = 0; i < (gReceiveLength - HeadStart); i++)
      {
        gReceiveCmd[i] = gReceiveCmd[FrameHeadDex++];
      }
    }

    if (gIsHandleMsgFlag)
    {
      Receive_Data_Analysis();
    }
    gReceiveLength = 0;
    iwdg_feed();
  }
  else
    gReceiveLength = 0;
}

/*
 @brief     : 根据验证接收的帧ID，决定返回对应的指令枚举
 @param     : 无
 @return    : frame id type(enum) 
 */
Frame_ID Command_Analysis::FrameID_Analysis(void)
{
  unsigned int FrameID = ((gReceiveCmd[1] << 8) | gReceiveCmd[2]);
  switch (FrameID)
  {
    case 0xA011 : return Work_Para;       break;
    case 0xA012 : return Set_Group_Num;   break;
    case 0xA013 : return SN_Area_Channel; break;
    case 0xA014 : return Work_Status;     break;
    case 0xA015 : return Stop_Work;       break;
    case 0xA020 : return ResetRoll;       break;
    case 0xA021 : return Opening;         break;
    case 0xA022 : return Work_Limit;      break;

    default     : memset(gReceiveCmd, 0x00, sizeof(gReceiveCmd)); break;
  }
}

/*
 @brief     : 验证接收到的LoRa数据里的CRC8校验码
 @param     : 1.验证数据的起始地址
              2.验证数据的长度
 @return    : true or false.
 */
bool Command_Analysis::Verify_CRC8(unsigned char verify_data_base_addr, unsigned char verify_data_len)
{
  unsigned char ReceiveCRC8 = GetCrc8(&gReceiveCmd[verify_data_base_addr], verify_data_len);
  if (ReceiveCRC8 == gReceiveCmd[gReceiveLength - 7])
    return true;
  else
    return false;
}

/*
 @brief   : 验证接收的设备ID与本机是否相同
 @param   : 无
 @return  : true or false            
 */
bool Command_Analysis::Verify_Device_Type_ID(void)
{
  unsigned int DeviceTypeID = ((gReceiveCmd[4] << 8) | gReceiveCmd[5]);
  if (DeviceTypeID == 0x5555)
    return true;

  if (DeviceTypeID == DEVICE_TYPE_ID)
    return true;
  else  
    return false;
}

/*
 @brief   : 验证接收的指令是否是群发指令
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Verify_Mass_Commands(void)
{
  gReceiveCmd[6] == 0x55 ? gMassCommandFlag = true : gMassCommandFlag = false;
}

/*
 @brief   : 验证接收的区域号与本地是否相同
 @param   : 无
 @return  : true or false            
 */
bool Command_Analysis::Verify_Area_Number(void)
{
  if (gReceiveCmd[7] == 0x55) return true;  //0x55:群控指令，忽略区域号
    
  unsigned char LocalAreaNumber = Roll_Operation.Read_Area_Number();
  if (gReceiveCmd[7] == LocalAreaNumber || LocalAreaNumber == 0) 
    return true;
  else  
    return false;
}

/*
 @brief   : 验证接收的工作组号是否在本机组控列表内。
            如果接收的组号是0x55，表明此指令忽略组控，发送给区域内所有的设备
            如果本设备还未申请注册过服务器，不用校验组号。
 @param   : 无
 @return  : true or false
 */
bool Command_Analysis::Verify_Work_Group(void)
{
  if (gReceiveCmd[8] == 0x55) return true;  //0x55:群控指令，忽略工作组号

  unsigned char LocalGroupNumber[5], ReceiveGroupSingleNumber = gReceiveCmd[8];
  unsigned char UndefinedGroupNum = 0;
  Roll_Operation.Read_Group_Number(&LocalGroupNumber[0]);

  for (unsigned char i = 0; i < 5; i++)
  {
    if (ReceiveGroupSingleNumber == LocalGroupNumber[i]) return true;

    /*全为0，说明是未初始化的组号，算校验通过*/
    if (LocalGroupNumber[i] == 0x00)
    {
      UndefinedGroupNum++;
      if (UndefinedGroupNum == 5)
        return true;
      }
  }
  return false;
}

/*
 @brief   : 验证接收的指令CRC8校验、设备类型码、区域号、工作组是否合法。
            可以通过形参决定是否屏蔽验证区域号和工作组。
 @param   : 1.验证的数据起始地址
            2.验证的数据长度
            3.是否要验证区域号标志位
            4.是否要验证工作组号标志位
 @return  : true or false.
 */
bool Command_Analysis::Verify_Frame_Validity(unsigned char verify_data_base_addr, unsigned char verify_data_len, bool area_flag = true, bool group_flag = true)
{
  if (Verify_CRC8(verify_data_base_addr, verify_data_len) == true)
  {
    if (Verify_Device_Type_ID() == true)
    {
      Verify_Mass_Commands();
      if (Verify_Area_Number() == true || area_flag == false)
      {
        if (Verify_Work_Group() == true || group_flag == false)
          return true;
        else
          Serial.println("Not this device group number... <Verify_Frame_Validity>");
      }
      else
        Serial.println("Not this device area number... <Verify_Frame_Validity>");
    }
    else
      Serial.println("Device type ID ERROR! <Verify_Frame_Validity>");
  }
  else
    Serial.println("CRC8 ERROR! <Verify_Frame_Validity>");
  return false;
}

/*
 @brief   : 根据帧ID分析判断执行哪一个接收到的通用指令或私有指令
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Receive_Data_Analysis(void)
{
  switch (FrameID_Analysis())
  {
    /*通用指令*/
    case Work_Para        : Query_Current_Work_Param();  break;
    case Set_Group_Num    : Set_Group_Number();         break;
    case SN_Area_Channel  : Set_SN_Area_Channel();      break;
    case Work_Status      : Detailed_Work_Status();     break;
    case Stop_Work        : Stop_Work_Command();        break;
    /*卷膜机私有指令*/
    case ResetRoll        : ResetRoll_Command();        break;
    case Opening          : Opening_Command();          break;
    case Work_Limit       : Working_Limit_Command();    break;
  }
}

/*
 @brief   : 服务器查询当前群控相关参数，如所在区域、SN码、路数、工作组、等。（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Query_Current_Work_Param(void)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 | 申号标志 |  查询角色 | 采集时间间隔      |  时间   |  预留位     |  校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number | intent   |  channel | collect interval  |  RTC   |   allocate  |  CRC8   |  Frame end
  //  1 byte       2 byte      1 byte          2 byte        1 byte       1 byte       1 byte      1 byte      2 byte           7 byte      8 byte     1 byte      6 byte
  
  if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

  if (Verify_Frame_Validity(4, 23, true, false) == true)
  {
    if (gReceiveCmd[8] == 0x01) //配置参数标志（LoRa大棚传感器是0x00配置采集时间）
    {
      /* 预留位第一字节用来设置LoRa的通信模式 */
      if(LoRa_Para_Config.Save_LoRa_Com_Mode(gReceiveCmd[19]))
      {
        Message_Receipt.General_Receipt(SetLoRaModeOk, 1);
        LoRa_MHL9LF.Parameter_Init(true);
        Message_Receipt.Working_Parameter_Receipt(true, 2);
      }
      else 
      {
        Message_Receipt.General_Receipt(SetLoRaModeErr, 2);
        Serial.println("Set LoRa Mode Err! <Query_Current_Work_Param>");
      }
    }
    else  //回执状态标志
    {
      Message_Receipt.Report_General_Parameter();
    }
  }
  memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief   : 设置本设备的工作组号，不需要验证本设备原有工作组号（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Set_Group_Number(void)
{ 
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   |所在执行区域号 | 工作组号   | 设备路数 |  校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID | mass flag   |  Area number |  workgroup |  channel |   CRC8 |  |  Frame end
  //  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte        5 byte       1 byte    1 byte      6 byte
  
  if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

  if (Verify_Frame_Validity(4, 10, true, false) == true)
  {
    if(Roll_Operation.Save_Group_Number(&gReceiveCmd[8]) == true)
    {
      Serial.println("Save group number success... <Set_Group_Number>");
      Message_Receipt.General_Receipt(AssignGroupIdArrayOk, 2);
    }
    else
    {
      Serial.println("Save group number failed !!! <Set_Group_Number>");
      Set_Motor_Status(STORE_EXCEPTION);
      Message_Receipt.General_Receipt(AssignGroupIdArrayErr, 1);
    }
  }
  memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief   : 设置本设备的SN码、区域号、设备路数等参数（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Set_SN_Area_Channel(void)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   | 所在执行区域号 |  设备路数      |  子设备总路数           |  SN码       | 校验码   |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID | mass flag   |   Area number | Device channel |  subordinate channel   | SN code     |  CRC8   |  Frame end
  //  1 byte       2 byte      1 byte          2 byte        1 byte          1 byte          1 byte           1 byte                9 byte       1 byte      6 byte

  if (Verify_Frame_Validity(4, 15, false, false) == true)
  {
    if (SN.Save_SN_Code(&gReceiveCmd[10]) == true && SN.Save_BKP_SN_Code(&gReceiveCmd[10]) == true)
    {
      Serial.println("Set SN code success... <Set_SN_Area_Channel>");
      if (Roll_Operation.Save_Area_Number(gReceiveCmd[7]) == true)
      {
        Serial.println("Save area number success... <Set_SN_Area_Channel>");
        Message_Receipt.General_Receipt(SetSnAndSlaverCountOk, 1);
        SN.Set_SN_Access_Network_Flag();
      }
      else
      {
        Serial.println("Save area number ERROR !!! <Set_SN_Area_Channel>");
        Set_Motor_Status(STORE_EXCEPTION);
        Message_Receipt.General_Receipt(SetSnAndSlaverCountErr, 1);
      }
    }
    else
    {
      Serial.println("Save SN code ERROR !!! <Set_SN_Area_Channel>");
      Set_Motor_Status(STORE_EXCEPTION);
      Message_Receipt.General_Receipt(SetSnAndSlaverCountErr, 1);
    }
  }
  memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 查询本设备详细工作状态（服务器 ---> 本设备）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Detailed_Work_Status(void)
{  
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 |  设备路数 |  校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number |   channel |   CRC8 |  |  Frame end
  //  1 byte       2 byte      1 byte          2 byte        1 byte        1 byte         1 byte    1 byte      6 byte
  
  if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

  if (Verify_Frame_Validity(4, 5, true, false) == true)
    Message_Receipt.Working_Parameter_Receipt(true, 1);

  memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief   : 强制停止当前设备的工作（服务器 ---> 本设备）
 @param   : 无
 @return  : 无
 */
void Command_Analysis::Stop_Work_Command(void)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 |  设备路数 |  校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number |   channel |   CRC8 |  |  Frame end
  //  1 byte       2 byte      1 byte          2 byte        1 byte        1 byte         1 byte    1 byte      6 byte
  
  if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

  if (Verify_Frame_Validity(4, 6, true, true) == true)
  {
    gStopWorkFlag = true;
    Message_Receipt.General_Receipt(TrunOffOk, 1);
    MANUAL_ROLL_ON;  //使能手动
  }
  memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 重置卷膜测量行程
 @param     : 无
 @return    : 无
 */
void Command_Analysis::ResetRoll_Command(void)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 |所在执行区域号 |  工作组号   | 设备路数 |  校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID |  mass flag  |  Area number |   workgroup | channel |   CRC8 |  |  Frame end
  //  1 byte       2 byte      1 byte          2 byte        1 byte          1 byte         1 byte      1 byte    1 byte      6 byte
  
  if (gAccessNetworkFlag == false)  return;  //如果设备还未注册到服务器，无视该指令

  if (Verify_Frame_Validity(4, 6, true, true) == true)
  {
    /*如果电机手动卷膜按键电路异常，禁止自动卷膜，等待更换设备*/
    if (gManualKeyExceptionFlag)
    {
      Serial.println("Manual roll key exception !!! <ResetRoll_Command>");
      return;
    }
    /*如果当前正在卷膜，不进行二次卷膜*/
    else if (gResetRollWorkingFlag == true || gOpeningWorkingFlag == true || gForceRollWorkingFlag == true)
    {
      Serial.println("The motor is resetting the distance... <ResetRoll_Command>");
      return;
    }
    /*如果当前正在手动卷膜。拒绝执行自动卷膜*/
    else if (gManualUpDetectFlag || gManualDownDetectFlag)
    {
      if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)  gManualUpDetectFlag = false;
      if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW) gManualDownDetectFlag = false;
      if (gManualUpDetectFlag || gManualDownDetectFlag)
      {
        Serial.println("Detect manual rolling... <ResetRoll_Command>");
        /*
          *待处理事情
        */
        return;
      }
    }
    else
    {
      MANUAL_ROLL_OFF;
      detachInterrupt(DEC_MANUAL_DOWN_PIN);
      detachInterrupt(DEC_MANUAL_UP_PIN);
      Message_Receipt.General_Receipt(RestRollerOk, 1);
      Motor_Operation.Reset_Motor_Route();
      attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
      attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
      MANUAL_ROLL_ON;
      iwdg_feed();
    }
  }
  memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 设置卷膜开度
 @param     : 无
 @return    : 无            
 */
void Command_Analysis::Opening_Command(void)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   |所在执行区域号 |  工作组号   | 设备路数 |  开度    | 校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID |  mass flag  |  Area number |   workgroup | channel |  oepning | CRC8 |  |  Frame end
  //  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte         1 byte      1 byte    1 byte    1 byte      6 byte

  if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

  if (Verify_Frame_Validity(4, 7, true, true) == true)  //如果校验通过（CRC校验、区域号校验、组号校验）
  {
    /*如果电机手动卷膜按键电路异常，禁止自动卷膜*/
    if (gManualKeyExceptionFlag)
    {
      Set_Motor_Status(MANUAL_KEY_EXCEPTION);
      Message_Receipt.Working_Parameter_Receipt(false, 2); 
      Serial.println("Manual roll key exception !!! <Opening_Command>");
      return;
    }
    /*如果当前正在手动卷膜。拒绝执行自动卷膜*/
    else if (gManualUpDetectFlag || gManualDownDetectFlag)
    {
      if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)  gManualUpDetectFlag = false;
      if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW) gManualDownDetectFlag = false;
      if (gManualUpDetectFlag || gManualDownDetectFlag)
      {
        Serial.println("Detect manual rolling... <Opening_Command>");
        /*
          *待处理事情
        */
        return;
      }
    }
    /*如果当前正在卷膜，不进行二次卷膜*/
    else if (gOpeningWorkingFlag == true || gResetRollWorkingFlag == true || gForceRollWorkingFlag == true)
    {
      Serial.println("Currently the motor is opening, and others opening cannot to do !!! <Opening_Command>");
      return;
    }

    /*失能手动卷膜， 失能检测手动卷膜按键中断*/
    detachInterrupt(DEC_MANUAL_DOWN_PIN);
    detachInterrupt(DEC_MANUAL_UP_PIN);
    MANUAL_ROLL_OFF;

    Message_Receipt.General_Receipt(OpenRollerOk, 1); //通用回执，告诉服务器接收到了开度卷膜命令

    volatile unsigned char opening_value = gReceiveCmd[10]; //获取从服务器接收的开度值

    /* 
      *如果是开度卷膜，且要求全关或全开。本次当前开度正好已经是全关或全开了
      *那么就不需要再次进入到卷膜函数里。
      *应用在于：假如冬天不能开棚，本来也确实关着。服务器发送一个关棚开度，
      *程序会先判断有没有重置行程，假如有些因素导致还要重置行程，那么就会打开棚了。
      *当然，不适用于强制卷膜
      *还有，假如发来的开度是0或100，说明需要开棚和关棚，一般用于天冷了或天热了，这个时候就
      *不去判断是否发来的和保存的是否一至了，而是发来的只要是关棚或开棚，同时需要重置行程，
      *就会用强制开棚和强制关棚来操作，最大限度不去重置行程。
      *而其他的开度值，就会去判断上一次的开度和本次的是否相等，如果相等，就不动作
      *如果不相等，同时又需要重置行程，那么只能必须先重置行程才能开到某个开度了。
     */
    if (opening_value >= 0 && opening_value <= 100)
    {
      unsigned char RealTimeOpenTemp =  Roll_Operation.Read_RealTime_Opening_Value();

      if (!Roll_Operation.Read_Route_Save_Flag())
      {
        if (opening_value == 0 || opening_value == 100)
        {
          if (opening_value == 0)
            opening_value = 0xF0;
          else
            opening_value = 0xF1;

          Serial.println("Prepare Force Open or Close. Be careful... <Opening_Command>");
          Motor_Operation.Force_Open_or_Close(opening_value);
          memset(gReceiveCmd, 0x00, gReceiveLength);
          /*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
          attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
          attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
          MANUAL_ROLL_ON;
          return;
        }
      }

      if (RealTimeOpenTemp == opening_value)
      {
        Serial.println("Film has been rolled to the current opening, do not repeat the film... <Opening_Command>");
        Set_Motor_Status(ROLL_OK);
        Message_Receipt.Working_Parameter_Receipt(true, 2);

        attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
        attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
        MANUAL_ROLL_ON;
        return;
      }
    }

    //正常开度值范围是0到100，如果是F0，表明是强制关棚，如果是F1，表明是强制开棚
    if (opening_value == 0xF0 || opening_value == 0xF1)
    {
      Serial.println("Prepare Force Open or Close. Be careful... <Opening_Command>");
      Motor_Operation.Force_Open_or_Close(opening_value);
      memset(gReceiveCmd, 0x00, gReceiveLength);
      /*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
      attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
      attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
      MANUAL_ROLL_ON;
      return;
    }

    if (Roll_Operation.Read_Route_Save_Flag())  //如果已经重置行程过
    {
      if(Roll_Operation.Save_Current_Opening_Value(opening_value))  //保存当前开度值
      {
        Serial.println("Begin to coiling... <Opening_Command>");
        Motor_Operation.Motor_Coiling();  //开始卷膜
        iwdg_feed();
      }
      else  //保存开度值异常
      {
        Serial.println("Save current opening value ERROR !!! <Opening_Command>");
        Set_Motor_Status(STORE_EXCEPTION);
        Message_Receipt.Working_Parameter_Receipt(false, 2);
      }
    }
    else  //或者没有重置行程
    {
      Serial.println("The film has not measured the distance, first measure, then roll the film... <Opening_Command>");
      if(Motor_Operation.Reset_Motor_Route() == true) //先重置行程，再开度卷膜
      {
        Serial.println("Roll OK, motor begin coiling... <Opening_Command>");

        if(Roll_Operation.Save_Current_Opening_Value(opening_value))  //保存当前开度值
        {
          gAdjustOpeningFlag = false;
          Motor_Operation.Motor_Coiling();  //开始卷膜
        }
        else  //保存开度值操作异常
        {
          Serial.println("Save current opening value ERROR !!! <Opening_Command>");
          Set_Motor_Status(STORE_EXCEPTION);
          Message_Receipt.Working_Parameter_Receipt(false, 2);
        }
      }
      else  //重置行程失败
        Serial.println("Reset motor route failed !!! <Opening_Command>");
      iwdg_feed();
    }
    /*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
    attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
    attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
    MANUAL_ROLL_ON;
  }
  memset(gReceiveCmd, 0x00, gReceiveLength);
}

/*
 @brief     : 电机工作电压阈值、上报状态间隔值设置（网关 ---> 本机）
 @param     : 无
 @return    : 无
 */
void Command_Analysis::Working_Limit_Command(void)
{ 
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  | 所在执行区域号 |  工作组号   | 设备路数 | 低电压阈值       |   高电压阈值      | 状态上报间隔     |校验码 | 帧尾 
  //Frame head | Frame ID | Data Length | Device type ID |  mass flag |   Area number |   workgroup | channel | LowVolThreshold | HighVolThreshold |  ReprotInterval | CRC8 |  Frame end
  //  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte         1 byte      1 byte     2 byte             2 byte              1 byte        1 byte  6 byte
  
  if (gAccessNetworkFlag == false)  return;  //如果本设备还没有注册到服务器，不理会该命令

  if (Verify_Frame_Validity(4, 11, true, true) == true)
  {
    if(Roll_Operation.Save_Roll_Work_Voltage_and_Report_Interval(&gReceiveCmd[10]) == true)
      Message_Receipt.General_Receipt(LimitRollerOk, 1);
    else
    {
      Serial.println("Save working threshold ERROR !");
      Message_Receipt.General_Receipt(LimitRollerErr, 1);
      Set_Motor_Status(STORE_EXCEPTION);
      Message_Receipt.Working_Parameter_Receipt(false, 2);
    }
  }
  memset(gReceiveCmd, 0x00, gReceiveLength);
}
