#include "receipt.h"
#include "User_CRC8.h"
#include <libmaple/iwdg.h>
#include "LoRa.h"
#include "Motor.h"
#include "Memory.h"
#include "Command_Analysis.h"
#include "public.h"

Receipt Message_Receipt;

#define CLEAR_BUFFER_FLAG   0 //是否清除服务器数据

#define SEND_DATA_DELAY     200 //发送完一帧数据后延时时间（ms）

unsigned char gMotorStatus = MotorFactoryMode;  //每次开机默认状态未初始化

unsigned char gLoRaCSQ[2] = {0};  //接收LoRa发送和接收的信号强度

/*
 @brief   : 清除服务器上一次接收的LoRa数据缓存
 @param   : 无
 @return  : 无
*/
void Receipt::Clear_Server_LoRa_Buffer(void)
{
  /*发送一帧帧尾，让服务器认为接收到了完成数据，清空缓存*/
  unsigned char Buffer[6] = {0x0D, 0x0A, 0x0D, 0x0A, 0x0D, 0x0A};
  LoRa_Serial.write(Buffer, 6);
  delay(SEND_DATA_DELAY);
}

/*
 @brief   : 随机生成回执消息的回执发送微秒延时
 @param   : 随机延时（ms）
 @return  : 无
 */
void Receipt::Receipt_Random_Wait_Value(unsigned long int *random_value)
{
  unsigned char RandomSeed;
  SN.Read_Random_Seed(&RandomSeed); //读取随机数种子
  /*RandomSeed * 1ms, 1.5S + RandomSeed * 0.1ms, 200ms*/
  *random_value = random(RandomSeed * 1000, 1500000) + random(RandomSeed *100, 200000);
}

/*
 @brief   : 上报本设备通用设置参数,包括区域号、SN码、子设备路数、工作组号、采集间隔等（本机 ---> 服务器）
            该回执帧也用作设备第一次申请注册服务器。
 @param   : 无
 @return  : 无        
 */
void Receipt::Report_General_Parameter(void)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   |群发标志位 | 所在执行区域号  | 工作组号   | SN码    | 查询角色 | 采集时间间隔      |  时间   |  预留位     |  校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID | mass flag | Area number   | workgroup | SN code | channel | collect interval  |  RTC   |   allocate  |  CRC8   |  Frame end
  //  1 byte       2 byte      1 byte          2 byte       1 byte       1 byte          5 byte     9 byte    1 byte      2 byte           7 byte      8 byte     1 byte      6 byte
  
  unsigned char ReportFrame[64] = {0};
  unsigned char FrameLength = 0; 
  unsigned char DataTemp[10];
  unsigned long int RandomSendInterval = 0;
    
  Receipt_Random_Wait_Value(&RandomSendInterval);
  delayMicroseconds(RandomSendInterval);
  iwdg_feed();

#if CLEAR_BUFFER_FLAG
  Clear_Server_LoRa_Buffer();
#endif

  ReportFrame[FrameLength++] = 0xFE; //帧头
  ReportFrame[FrameLength++] = 0xE0; //帧ID
  ReportFrame[FrameLength++] = 0x11;  
  ReportFrame[FrameLength++] = 0x24; //帧有效数据长度
  /*设备类型*/
  ReportFrame[FrameLength++] = highByte(DEVICE_TYPE_ID);
  ReportFrame[FrameLength++] = lowByte(DEVICE_TYPE_ID);
  /*是否是群发*/
  gMassCommandFlag == true ? ReportFrame[FrameLength++] = 0x55 : ReportFrame[FrameLength++] = 0x00;
  /*区域号*/
  ReportFrame[FrameLength++] = Roll_Operation.Read_Area_Number();
  /*组号*/
  Roll_Operation.Read_Group_Number(&DataTemp[0]);
  for (unsigned char i = 0; i < 5; i++)
    ReportFrame[FrameLength++] = DataTemp[i];
  /*SN码*/
  SN.Read_SN_Code(&DataTemp[0]);
  for (unsigned char i = 0; i < 9; i++)
    ReportFrame[FrameLength++] = DataTemp[i];
  /*路数*/
  ReportFrame[FrameLength++] = 0x01; //卷膜机默认只有一路
  /*采集状态间隔*/
  ReportFrame[FrameLength++] = 0x00; 
  ReportFrame[FrameLength++] = 0x00; 
  /*RTC*/
  for (unsigned char i = 0; i < 7; i++)
    ReportFrame[FrameLength++] = 0x00; 
  /*预留的8个字节*/
  for (unsigned char i = 0; i < 8; i++)  
    ReportFrame[FrameLength++] = 0x00;
  /*CRC8*/
  ReportFrame[FrameLength++] = GetCrc8(&ReportFrame[4], 0x24);
  /*帧尾*/
  for (unsigned char i = 0; i < 6; i++)
    i % 2 == 0 ? ReportFrame[FrameLength++] = 0x0D : ReportFrame[FrameLength++] = 0x0A;

  Serial.println("Report general parameter...");
  /*打印要发送的数据帧*/
  Print_Debug(&ReportFrame[0], FrameLength);

  Some_Peripheral.Stop_LED();
  LoRa_Serial.write(&ReportFrame[0], FrameLength);  
  delay(SEND_DATA_DELAY);
  Some_Peripheral.Start_LED();
}

/*
 @brief   : 当本地工作组号丢失，向服务器申请本机的工作组号（本设备 ---> 服务器）
 @param   : 无
 @return  : 无
 */
void Receipt::Request_Set_Group_Number(void)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   |群发标志位 | 所在执行区域号 |  设备路数      | 校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID | mass flag | Area number  | Device channel |  CRC8  |  Frame end
  //  1 byte        2 byte      1 byte          2 byte       1 byte       1 byte          1 byte       1 byte     6 byte
  
  unsigned char RequestFrame[20] = {0};
  unsigned char FrameLength = 0;
  unsigned char RandomSeed;
  unsigned long int RandomSendInterval = 0;
    
  Receipt_Random_Wait_Value(&RandomSendInterval);
  delayMicroseconds(RandomSendInterval);
  iwdg_feed();

#if CLEAR_BUFFER_FLAG
  Clear_Server_LoRa_Buffer();
#endif

  RequestFrame[FrameLength++] = 0xFE; //帧头
  RequestFrame[FrameLength++] = 0xE0; //帧ID
  RequestFrame[FrameLength++] = 0x12;
  RequestFrame[FrameLength++] = 0x05; //帧有效数据长度
  /*设备ID*/
  RequestFrame[FrameLength++] = highByte(DEVICE_TYPE_ID); 
  RequestFrame[FrameLength++] = lowByte(DEVICE_TYPE_ID);
  /*是否是群发*/
  gMassCommandFlag == true ? RequestFrame[FrameLength++] = 0x55 : RequestFrame[FrameLength++] = 0x00;
  /*区域号*/
  RequestFrame[FrameLength++] = Roll_Operation.Read_Area_Number();  
  /*设备路数*/
  RequestFrame[FrameLength++] = 0x01; 
  /*CRC8*/
  RequestFrame[FrameLength++] = GetCrc8(&RequestFrame[4], 0x05);
  /*帧尾*/
  for (unsigned char i = 0; i < 6; i++)
    i % 2 == 0 ? RequestFrame[FrameLength++] = 0x0D : RequestFrame[FrameLength++] = 0x0A;

  Serial.println("Requeset SN code to access network...");
  /*打印要发送的数据帧*/
  Print_Debug(&RequestFrame[0], FrameLength);

  Some_Peripheral.Stop_LED();
  LoRa_Serial.write(&RequestFrame[0], FrameLength);
  delay(SEND_DATA_DELAY); 
  Some_Peripheral.Start_LED(); 
}

/*
 @brief   : 当本地SN码丢失，向服务器申请本机的SN码（本设备 ---> 服务器）
 @param   : 无
 @return  : 无
 */
void Receipt::Request_Device_SN_and_Channel(void)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 | 所在执行区域号 |  设备路数      | 校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number | Device channel |  CRC8  |  Frame end
  //  1 byte        2 byte      1 byte          2 byte        1 byte      1 byte          1 byte       1 byte     6 byte

  unsigned char RequestFrame[20] = {0};
  unsigned char FrameLength = 0;
  unsigned char RandomSeed;
  unsigned long int RandomSendInterval = 0;
    
  Receipt_Random_Wait_Value(&RandomSendInterval);
  delayMicroseconds(RandomSendInterval);
  iwdg_feed();

#if CLEAR_BUFFER_FLAG
  Clear_Server_LoRa_Buffer();
#endif

  RequestFrame[FrameLength++] = 0xFE; //帧头
  RequestFrame[FrameLength++] = 0xE0; //帧ID
  RequestFrame[FrameLength++] = 0x13;
  RequestFrame[FrameLength++] = 0x05; //帧有效数据长度
  /*设备ID*/
  RequestFrame[FrameLength++] = highByte(DEVICE_TYPE_ID); 
  RequestFrame[FrameLength++] = lowByte(DEVICE_TYPE_ID);
  /*是否是群发*/
  gMassCommandFlag == true ? RequestFrame[FrameLength++] = 0x55 : RequestFrame[FrameLength++] = 0x00;
  /*区域号*/
  RequestFrame[FrameLength++] = Roll_Operation.Read_Area_Number();  
  /*设备路数*/
  RequestFrame[FrameLength++] = 0x01; 
  /*CRC8*/
  RequestFrame[FrameLength++] = GetCrc8(&RequestFrame[4], 0x05);
  /*帧尾*/
  for (unsigned char i = 0; i < 6; i++)
    i % 2 == 0 ? RequestFrame[FrameLength++] = 0x0D : RequestFrame[FrameLength++] = 0x0A;

  Serial.println("Requeset SN code to access network...");
  /*打印要发送的数据帧*/
  Print_Debug(&RequestFrame[0], FrameLength);

  Some_Peripheral.Stop_LED();
  LoRa_Serial.write(&RequestFrame[0], FrameLength);
  delay(SEND_DATA_DELAY);
  Some_Peripheral.Start_LED();
}

/*
 @brief   : 上报实时详细工作状态（本机 ---> 服务器）
 @param   : 上报次数
 @return  : 无
 */
void Receipt::Working_Parameter_Receipt(bool use_random_wait, unsigned char times)
{  
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 | 所在执行区域号 | 设备路数       | 设备状态        | 电压     |  RSSI  |  CSQ    | 预留位  | 校验码  | 帧尾 
  //Frame head | Frame ID | Data Length | Device type ID | mass flag | Area number   | Device channel |  device status |  voltage |                              CRC8   | Frame end
  //  1 byte        2 byte      1 byte          2 byte       1 byte      1 byte             1 byte       1 byte           2 byte     1 byte  1 byte    16 byte   1 byte     6 byte
 
  unsigned char ReceiptFrame[64] = {0};
  unsigned char ReceiptLength = 0;
  unsigned char RandomSeed;
  unsigned long int RandomSendInterval = 0;

  iwdg_feed();
  if (use_random_wait) 
  {
    //随机等待一段时间后再发送，避免大量设备发送堵塞。
    Receipt_Random_Wait_Value(&RandomSendInterval);
    delayMicroseconds(RandomSendInterval);
  }
  iwdg_feed();

#if CLEAR_BUFFER_FLAG
  Clear_Server_LoRa_Buffer();
#endif
  
  ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
  ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
  ReceiptFrame[ReceiptLength++] = 0x14;
  ReceiptFrame[ReceiptLength++] = 0x1A; //数据长度
  /*设备类型*/
  ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
  ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);
  /*验证服务器命令是否是群发*/
  gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;
  /*区域ID*/
  ReceiptFrame[ReceiptLength++] = Roll_Operation.Read_Area_Number();
  /*设备路数*/
  ReceiptFrame[ReceiptLength++] = 0x01;
  /*该设备当前电机状态*/
  ReceiptFrame[ReceiptLength++] = Read_Motor_Status();
  /*该设备电机当前电压值*/
  int VolTemp = Motor_Operation.Voltage_Detection();
  if (VolTemp < 0) VolTemp *= -1;
  ReceiptFrame[ReceiptLength++] = highByte(VolTemp);
  ReceiptFrame[ReceiptLength++] = lowByte(VolTemp);
  /*SNR 和 RSSI*/ 
  ReceiptFrame[ReceiptLength++] = Type_Conv.Dec_To_Hex(gLoRaCSQ[0]);  //信号发送强度
  ReceiptFrame[ReceiptLength++] = Type_Conv.Dec_To_Hex(gLoRaCSQ[1]);  //信号接收强度

  /*协议预留的16个字节(它们中的一些在该帧里用来表示电机实时信息)*/
  ReceiptFrame[ReceiptLength++] = Roll_Operation.Read_RealTime_Opening_Value(); //电机实时开度
  unsigned int Temp = Roll_Operation.Read_Roll_Low_Voltage_Limit_Value(); //读取最小电压阈值
  ReceiptFrame[ReceiptLength++] = highByte(Temp);
  ReceiptFrame[ReceiptLength++] = lowByte(Temp);
  Temp = Roll_Operation.Read_Roll_High_Current_Limit_Value(); //读取最大电流阈值
  ReceiptFrame[ReceiptLength++] = highByte(Temp);
  ReceiptFrame[ReceiptLength++] = lowByte(Temp);
  Temp = Motor_Operation.Current_Detection(); //得到电机当前电流值
  ReceiptFrame[ReceiptLength++] = highByte(Temp);
  ReceiptFrame[ReceiptLength++] = lowByte(Temp);
  ReceiptFrame[ReceiptLength++] = Type_Conv.Dec_To_Hex(Roll_Operation.Read_Roll_Report_Status_Interval_Value()); //得到上报状态频率

  /* 预留的后8个字节，第一个字节用来上传当前LoRa的通信模式 */
  ReceiptFrame[ReceiptLength++] = LoRa_Para_Config.Read_LoRa_Com_Mode();
  /* 第二个字节用来表达软件版本，默认只有一位有效小数位 */
  ReceiptFrame[ReceiptLength++] = SOFT_VERSION;
  /* 第三个字节用来表达硬件版本，默认只有一位有效小数位 */
  ReceiptFrame[ReceiptLength++] = HARD_VERSION;
  for (unsigned char i = 0; i < 5; i++)
    ReceiptFrame[ReceiptLength++] = 0x00;
  /*CRC8*/
  ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], 0x1A);
  /*帧尾*/
  for (unsigned char i = 0; i < 6; i++)
    i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

  Serial.println("LoRa parameter receipt...");
  Print_Debug(&ReceiptFrame[0], ReceiptLength);

  for (unsigned char i = 0; i < times; i++)
  {
    iwdg_feed();
    Some_Peripheral.Stop_LED();
    LoRa_Serial.write(&ReceiptFrame[0], ReceiptLength);
    delayMicroseconds(SEND_DATA_DELAY * 1000);
    Some_Peripheral.Start_LED();
  }
}

/*
 @brief   : 发送通用回执信息给服务器。（本设备 ---> 服务器）
            在大多数情况下，当接受到服务器的指令后，需要发送本条通用回执
 @param   : 1.回执状态
            2.回执次数
 @return  : 无
 */
void Receipt::General_Receipt(unsigned char status, unsigned char send_times)
{
  //  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 | 所在执行区域号 |  设备路数      | 回执状态       |  预留位    | 校验码  |     帧尾 
  //Frame head | Frame ID | Data Length | Device type ID | mass flag | Area number | Device channel | receipt status |  allocate | CRC8    |  Frame end
  //  1 byte        2 byte      1 byte          2 byte       1 byte        1 byte          1 byte       1 byte          8 byte      1 byte     6 byte
  
  unsigned char ReceiptFrame[25] = {0};
  unsigned char ReceiptLength = 0;
  unsigned char RandomSeed;
  unsigned long int RandomSendInterval = 0;
    
  Receipt_Random_Wait_Value(&RandomSendInterval);
  delayMicroseconds(RandomSendInterval);
  iwdg_feed();

  /*读取LoRa模块的 SNR and RSSI，为了防止影响性能，只获取一次信号值*/
  if (gLoRaCSQ[0] == 0 || gLoRaCSQ[1] == 0)
    LoRa_MHL9LF.LoRa_AT(gLoRaCSQ, true, AT_CSQ_, 0);

#if CLEAR_BUFFER_FLAG
  Clear_Server_LoRa_Buffer();
#endif

  ReceiptFrame[ReceiptLength++] = 0xFE; //帧头
  ReceiptFrame[ReceiptLength++] = 0xE0; //帧ID
  ReceiptFrame[ReceiptLength++] = 0x15;
  ReceiptFrame[ReceiptLength++] = 0x0E; //帧有效数据长度
  /*设备类型*/
  ReceiptFrame[ReceiptLength++] = highByte(DEVICE_TYPE_ID);
  ReceiptFrame[ReceiptLength++] = lowByte(DEVICE_TYPE_ID);
  /*是否是群发*/
  gMassCommandFlag == true ? ReceiptFrame[ReceiptLength++] = 0x55 : ReceiptFrame[ReceiptLength++] = 0x00;
  /*区域号*/
  ReceiptFrame[ReceiptLength++] = Roll_Operation.Read_Area_Number();
  /*路数*/
  ReceiptFrame[ReceiptLength++] = 0x01; 
  /*回执状态*/
  ReceiptFrame[ReceiptLength++] = status;
  /*预留的8个字节*/
  /*SNR and RSSI*/
  ReceiptFrame[ReceiptLength++] = Type_Conv.Dec_To_Hex(gLoRaCSQ[0]);  //发送信号强度
  ReceiptFrame[ReceiptLength++] = Type_Conv.Dec_To_Hex(gLoRaCSQ[1]);  //接收信号强度
  for (unsigned char i = 0; i < 6; i++)
    ReceiptFrame[ReceiptLength++] = 0x00;
  /*CRC8*/
  ReceiptFrame[ReceiptLength++] = GetCrc8(&ReceiptFrame[4], 0x0E);
  /*帧尾*/
  for (unsigned char i = 0; i < 6; i++)
    i % 2 == 0 ? ReceiptFrame[ReceiptLength++] = 0x0D : ReceiptFrame[ReceiptLength++] = 0x0A;

  Serial.println("Send General Receipt...");
  Print_Debug(&ReceiptFrame[0], ReceiptLength);

  Some_Peripheral.Stop_LED();
  for (unsigned char i = 0; i < send_times; i++)
  {
    iwdg_feed();
    LoRa_Serial.write(ReceiptFrame, ReceiptLength);
    delay(SEND_DATA_DELAY);
  }
  Some_Peripheral.Start_LED();
}

/*
 @brief   : 串口打印16进制回执信息
 @param   : 1.数据起始地址
            2.数据长度s
 @return  : 无
 */
void Receipt::Print_Debug(unsigned char *base_addr, unsigned char len)
{
  for (unsigned char i = 0; i < len; i++)
  {
    Serial.print(base_addr[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
