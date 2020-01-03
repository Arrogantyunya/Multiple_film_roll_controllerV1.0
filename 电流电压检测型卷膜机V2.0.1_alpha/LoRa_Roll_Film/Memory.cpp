/************************************************************************************
 * 
 * 代码与注释：卢科青
 * 日期：2019/3/11
 * 该文件主要功能是依托EEPROM和芯片自带备份寄存器来保存、校验、操作、读取设备相关的信息，
 * 如通用的SN码，软件、硬件版本号，区域号，工作组号等。私有信息如电机卷膜开度、测量行程标志
 * 位等。头文件中提供了各个类的公共接口。
 * 
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "Memory.h"
#include "User_CRC8.h"
#include <libmaple/bkp.h>
#include <libmaple/iwdg.h>
#include "Motor.h"
#include "public.h"

/*创建EEPROM操作对象*/
EEPROM_Operations EEPROM_Operation;
/*创建SN码操作对象*/
SN_Operations SN;
/*创建LoRa配置对象*/
LoRa_Config LoRa_Para_Config;
/*创建卷膜操作对象*/
Roll_Operations Roll_Operation;
/*创建软件硬件版本对象*/
Soft_Hard_Vertion Vertion;

/*
 @brief     : 设置EEPROM读写引脚
 @para      : 无
 @return    : 无
 */
void EEPROM_Operations::EEPROM_GPIO_Config(void)
{
    pinMode(EP_WP_PIN, OUTPUT);
    digitalWrite(EP_WP_PIN, HIGH);
    I2C_Init();
}

/*
 @brief     : 写SN码到EEPROM
 @para      : SN码数组
 @return    : true or false
 */
bool SN_Operations::Save_SN_Code(unsigned char *sn_code)
{
    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 9; i++)
        AT24CXX_WriteOneByte(SN_BASE_ADDR + i, sn_code[i]);

    unsigned char SN_Verify = GetCrc8(&sn_code[0], 9); //得到SN码的CRC8
    AT24CXX_WriteOneByte(SN_VERIFY_ADDR, SN_Verify);
    
    /*读出写入的SN码，生成一个CRC8，和写入的CRC8校验，判断写入是否成功*/
    unsigned char SN_CodeTemp[9];
    for (unsigned char i = 0; i < 9; i++)
        SN_CodeTemp[i] = AT24CXX_ReadOneByte(SN_BASE_ADDR + i);

    unsigned char SN_VerifyTemp = GetCrc8(&SN_CodeTemp[0], 9);
    if (SN_VerifyTemp != SN_Verify)
    {
        EEPROM_Write_Disable();
        return false;
    }
    else
    {
        if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) != 0x55)
            AT24CXX_WriteOneByte(SN_OPERATION_FLAG_ADDR, 0x55); //校验成功，SN写入成功标志位置0x55。

        EEPROM_Write_Disable();
        return true;
    }
}

/*
 @brief     : 写SN码到EEPROM的SN备份地址
              注释参考 Save_SN_Code 函数
 @para      : SN 数组
 @return    : true or false
 */
bool SN_Operations::Save_BKP_SN_Code(unsigned char *sn_code)
{
    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 9; i++)
        AT24CXX_WriteOneByte(SN_BKP_BASE_ADDR + i, sn_code[i]);

    unsigned char SN_BKP_Verify = GetCrc8(&sn_code[0], 9);
    AT24CXX_WriteOneByte(SN_BKP_VERIFY_ADDR, SN_BKP_Verify);

    unsigned char SN_CodeTemp[9];
    for (unsigned char i = 0; i < 9; i++)
        SN_CodeTemp[i] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + i);

    unsigned char SN_BKP_VerifyTemp = GetCrc8(&SN_CodeTemp[0], 9);
    if (SN_BKP_VerifyTemp != SN_BKP_Verify)
    {
        EEPROM_Write_Disable();
        return false;
    }
    else
    {
        if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) != 0x55)
            AT24CXX_WriteOneByte(SN_BKP_OPERATION_FLAG_ADDR, 0x55);

        EEPROM_Write_Disable();
        return true;
    }
}

/*
 @brief     : 从EEPROM读取SN码
 @para      : SN数组
 @return    : true or false
 */
bool SN_Operations::Read_SN_Code(unsigned char *sn_code)
{
    unsigned char SN_Temp[9] = {0};

    for (unsigned char i = 0; i < 9; i++)
        SN_Temp[i] = AT24CXX_ReadOneByte(SN_BASE_ADDR + i);

    unsigned char SN_Verify = AT24CXX_ReadOneByte(SN_VERIFY_ADDR);
    unsigned char SN_VerifyTemp = GetCrc8(&SN_Temp[0], 9);
    
    /*校验数据CRC8*/
    if (SN_Verify != SN_VerifyTemp)
        return false;
    else
    {
        for (unsigned char i = 0; i < 9; i++)
            sn_code[i] = SN_Temp[i];
        return true;
    }
}

/*
 @brief     : 从EEPROM读取备份的SN码
              注释参考 Read_SN_Code 函数
 @para      : SN数组
 @return    : true or false
 */
bool SN_Operations::Read_BKP_SN_Code(unsigned char *sn_code)
{
    unsigned char SN_BKP_Temp[9] = {0};

    for (unsigned char i = 0; i < 9; i++)
        SN_BKP_Temp[i] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + i);

    unsigned char SN_BKP_Verify = AT24CXX_ReadOneByte(SN_BKP_VERIFY_ADDR);
    unsigned char SN_BKP_VerifyTemp = GetCrc8(&SN_BKP_Temp[0], 9);

    if (SN_BKP_Verify != SN_BKP_VerifyTemp)
        return false;
    else
    {
        for (unsigned char i = 0; i < 9; i++)
            sn_code[i] = SN_BKP_Temp[i];
        return true;
    }
}

/*
 @brief     : 取保存在SN备份地址的SN码最后两位，生成随机数种子。
              该随机数种子用于防止堵塞的回执应答随机延时。
 @para      : 随机种子指针变量
 @return    : 无
 */
void SN_Operations::Read_Random_Seed(unsigned char *random_seed)
{
    unsigned char RandomTemp[2];
    RandomTemp[0] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + 7);
    RandomTemp[1] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + 8);
    *random_seed = GetCrc8(RandomTemp, 2);
}

/*
 @brief     : 验证是否已经写入SN码，0x55表示已经写入SN码
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Verify_Save_SN_Code(void)
{
    if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 验证是否已经写入备份SN码，0x55表示已经写入SN码
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Verify_Save_BKP_SN_Code(void)
{
    if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 清除保存SN码标志位，慎用！
              如果清除了该标志位，下一次写入SN事件会无视已保存的SN码，覆盖它。
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Clear_SN_Save_Flag(void)
{
    if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x00)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SN_OPERATION_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();
    /*判断是否已成功清除该标志位*/
    if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x00)
        return true;
    else
        return false;
}

/*
 @brief     : 清除保存备份SN码标志位，慎用！
              如果清除了该标志位，下一次写入SN事件会无视已保存的SN码，覆盖它。
              注释参考 Clear_SN_Save_Flag 函数
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Clear_BKP_SN_Save_Flag(void)
{
    if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x00)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SN_BKP_OPERATION_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();

    if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x00)
        return true;
    else
        return false;
}

/*
 @brief     : 如果注册到服务器成功，设置注册成功标志位
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Set_SN_Access_Network_Flag(void)
{
    if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x55)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SN_ACCESS_NETWORK_FLAG_ADDR, 0X55);
    EEPROM_Write_Disable();
    /*判断标志位是否写入成功*/
    if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 清除注册到服务器标志位，这将意味着本机将重新请求注册到服务器，慎用！
 @para      : 无
 @return    : true or false
 */
bool SN_Operations::Clear_SN_Access_Network_Flag(void)
{
    if (digitalRead(SW_FUN2) == LOW)    //如果功能按键2按下
    {
        iwdg_feed();
        delay(5000);    //保持按下5S
        iwdg_feed();
        if (digitalRead(SW_FUN2) == LOW)
        {
            Some_Peripheral.Key_Buzz(1000);
            if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) != 0x00)
            {
                EEPROM_Write_Enable();
                AT24CXX_WriteOneByte(SN_ACCESS_NETWORK_FLAG_ADDR, 0x00);
                EEPROM_Write_Disable();
            }
            /*验证标志位是否清除成功*/
            if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x00)
                return true;
            else
                return false;  
        }
    } 
}

/*
 @brief     : 验证是否已经注册到服务器，0x55表示已经注册。
 @para      : 无
 @return    : true or false              
 */
bool SN_Operations::Verify_SN_Access_Network_Flag(void)
{
    bool BoolValue;
    AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x55 ? BoolValue = true : BoolValue = false;
    return BoolValue;
}

/*
 @brief     : 验证读取的SN是否错误，如果其中一个SN失败，则用另一个完整的SN覆盖它。
 @para      : SN数组
 @return    : true or false
 */
bool SN_Operations::Self_check(unsigned char *sn_code)
{
    bool BoolValue;
    if (Read_SN_Code(sn_code) && Read_BKP_SN_Code(sn_code)) //如果读取SN码和备份SN码都正确
    {
        Serial.println("Read SN code and read backup SN code success...");
        BoolValue = true;
    }
    else if (Read_SN_Code(sn_code)) //如果仅仅是读取SN码成功，将SN码覆盖修正备份SN码保存区
    {
        Serial.println("Read SN code success but read backup SN code failed...");
        Save_BKP_SN_Code(sn_code) == true ? BoolValue = true : BoolValue = false;   //覆盖后判断覆盖是否成功
    }
    else if (Read_BKP_SN_Code(sn_code)) //如果仅仅是读取备份SN码成功，将SN码覆盖修正SN码保存区
    {
        Serial.println("Read backup SN code success but read SN code failed...");
        Save_SN_Code(sn_code) == true ? BoolValue = true : BoolValue = false; //覆盖后判断覆盖是否成功
    }
    else    //如果SN码和备份SN码都损坏，清除已保存SN码标志位，已保存备份SN码标志位，设备将需要重新申请一份SN码
    { 
        Serial.println("All SN store ERROR!");
        Clear_SN_Save_Flag();
        Clear_BKP_SN_Save_Flag();
    }
    return BoolValue;   //返回操作的最终结果
}

/*
 @brief     : 配置LoRa参数完成标志位
 @para      : 无
 @return    : true or false
 */
bool LoRa_Config::Save_LoRa_Config_Flag(void)
{
    if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x55)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(LORA_PARA_CONFIG_FLAG_ADDR, 0x55);
    EEPROM_Write_Disable();

    if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 验证是否已经配置了LoRa参数
 @para      : 无
 @return    : true or false
 */
bool LoRa_Config::Verify_LoRa_Config_Flag(void)
{
    if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 清除配置LoRa成功标志位
 @para      : 无
 @return    : true or false
 */
bool LoRa_Config::Clear_LoRa_Config_Flag(void)
{
    if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x00)
        return true;
    
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(LORA_PARA_CONFIG_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();   

    if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x00)
        return true;
    else
        return false;
}

bool LoRa_Config::Save_LoRa_Com_Mode_Flag(void)
{
    if (AT24CXX_ReadOneByte(LORA_COM_MODE_FLAG_ADDR) == 0x55)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(LORA_COM_MODE_FLAG_ADDR, 0x55);
    EEPROM_Write_Disable();
    
    if (AT24CXX_ReadOneByte(LORA_COM_MODE_FLAG_ADDR == 0x55))
        return true;
    else
        return false;
}

bool LoRa_Config::Clear_LoRa_Com_Mode_Flag(void)
{
    if (AT24CXX_ReadOneByte(LORA_COM_MODE_FLAG_ADDR) == 0x00)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(LORA_COM_MODE_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();
    
    if (AT24CXX_ReadOneByte(LORA_COM_MODE_FLAG_ADDR == 0x00))
        return true;
    else
        return false;
}

bool LoRa_Config::Save_LoRa_Com_Mode(unsigned char mode)
{
    unsigned char ModeCrc8;

    if (mode == 0xF0 || mode == 0xF1)
    {
        if (AT24CXX_ReadOneByte(LORA_COM_MODE_ADDR) == mode)
            return true;
        
        EEPROM_Write_Enable();
        AT24CXX_WriteOneByte(LORA_COM_MODE_ADDR, mode);
        ModeCrc8 = GetCrc8(&mode, 1);
        AT24CXX_WriteOneByte(LORA_COM_MODE_VERIFY_ADDR, ModeCrc8);

        if (AT24CXX_ReadOneByte(LORA_COM_MODE_ADDR) == mode)
        {
            Save_LoRa_Com_Mode_Flag();
            EEPROM_Write_Disable();
            return true;
        }
        else
        {
            EEPROM_Write_Disable();
            return false;
        }
    }
    else
    {
        Serial.println("Save LoRa communication mode Err <Save_LoRa_Com_Mode>");
        return false;
    }
}

unsigned char LoRa_Config::Read_LoRa_Com_Mode(void)
{   
    unsigned char ComTemp;
    unsigned char Comcrc8;

    ComTemp = AT24CXX_ReadOneByte(LORA_COM_MODE_ADDR);
    Comcrc8 = GetCrc8(&ComTemp, 1);

    if (Comcrc8 == AT24CXX_ReadOneByte(LORA_COM_MODE_VERIFY_ADDR))
    {
        return ComTemp;
    }
    else
    {
        Save_LoRa_Com_Mode(0xF0);
        return 0xF0;
    }
}

bool LoRa_Config::Save_LoRa_Addr(unsigned char *addr)
{
    unsigned char TempBuf[8];
    unsigned char AddrCrc8, AddrTempCrc8;

    EEPROM_Write_Enable();
    AddrCrc8 = GetCrc8(&addr[0], 8);
    for (unsigned char i = 0; i < 8; i++)
    {
        AT24CXX_WriteOneByte(EP_LORA_ADDR_BASE_ADDR + i, addr[i]);
    }
    AT24CXX_WriteOneByte(EP_LORA_ADDR_VERIFY_ADDR, AddrCrc8);

    for (unsigned char i = 0; i < 8; i++)
    {
        TempBuf[i] = AT24CXX_ReadOneByte(EP_LORA_ADDR_BASE_ADDR + i);
    }
    AddrTempCrc8 = GetCrc8(&TempBuf[0], 8);

    if (AddrCrc8 != AddrTempCrc8)
    { 
        Clear_LoRa_Addr_Flag();
        EEPROM_Write_Disable();
        return false;
    }
    else
    {
        Serial.println("Save LoRa Addr OK...");
        Save_LoRa_Addr_Flag();
        return true;
    }
}

bool LoRa_Config::Read_LoRa_Addr(unsigned char *addr)
{
    unsigned char TempBuf[8];
    unsigned char AddrCrc8, AddrTempCrc8;

    for (unsigned char i = 0; i < 8; i++)
    {
        TempBuf[i] = AT24CXX_ReadOneByte(EP_LORA_ADDR_BASE_ADDR + i);
    }
    AddrCrc8 = GetCrc8(&TempBuf[0], 8);
    AddrTempCrc8 = AT24CXX_ReadOneByte(EP_LORA_ADDR_VERIFY_ADDR);

    if (AddrCrc8 != AddrTempCrc8)
        return false;
    
    for (unsigned char i = 0; i < 8; i++)
    {
        addr[i] = TempBuf[i];
    }
    Serial.println("Read LoRa Addr OK...");
    return true;
}

void LoRa_Config::Save_LoRa_Addr_Flag(void)
{
    if (AT24CXX_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0X55)
        return;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR, 0x55);
    EEPROM_Write_Disable();
}

void LoRa_Config::Clear_LoRa_Addr_Flag(void)
{
    if (AT24CXX_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0x00)
        return;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();
}

bool LoRa_Config::Verify_LoRa_Addr_Flag(void)
{
    if (AT24CXX_ReadOneByte(EP_LORA_ADDR_SAVED_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 保存该设备的软件版本号
 @para      : 软件包本号高8位，低8位
 @return    : 无
 */
void Soft_Hard_Vertion::Save_Software_version(unsigned char number_high, unsigned char number_low)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SOFT_HARD_VERSION_BASE_ADDR, number_high);
    AT24CXX_WriteOneByte(SOFT_HARD_VERSION_BASE_ADDR + 1,  number_low);
    EEPROM_Write_Disable();
}

/*
 @brief     : 保存该设备的硬件版本号
 @para      : 硬件版本号高8位，低8位
 @return    : 无
 */
void Soft_Hard_Vertion::Save_hardware_version(unsigned char number_high, unsigned char number_low)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SOFT_HARD_VERSION_BASE_ADDR + 2, number_high);
    AT24CXX_WriteOneByte(SOFT_HARD_VERSION_END_ADDR, number_low);
    EEPROM_Write_Disable();
}

/*
 @brief     : 电机测量行程完成，保存总行程完成，设置该标志位
 @para      : 无
 @return    : true or false
 */
bool Roll_Operations::Set_Route_Save_Flag(void)
{
    if (AT24CXX_ReadOneByte(ROUTE_FLAG_ADDR) == 0x55)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(ROUTE_FLAG_ADDR, 0x55);
    EEPROM_Write_Disable();

    if (AT24CXX_ReadOneByte(ROUTE_FLAG_ADDR) != 0x55)
        return false;
    else
        return true;
}

/*
 @brief     : 读取保存行程标志位,以决定是否需要重新测量行程，或可以开始按照预设开度卷膜
 @para      : 无
 @return    : true or false
 */
bool Roll_Operations::Read_Route_Save_Flag(void)
{
    if (AT24CXX_ReadOneByte(ROUTE_FLAG_ADDR) == 0x55)
        return true;
    else   
        return false;
}

/*
 @brief     : 清除总行程保存标志位,这将意味着需要重新测量行程。慎用！
 @para      : 无
 @return    : true or false
 */
bool Roll_Operations::Clear_Route_Save_Flag(void)
{    
    if (AT24CXX_ReadOneByte(ROUTE_FLAG_ADDR) == 0x00)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(ROUTE_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();

    if (AT24CXX_ReadOneByte(ROUTE_FLAG_ADDR) == 0x00)
        return true;
    else
        return false;
}

/*
 @brief     : 保存卷膜整个行程所需时间
 @para      : time(S)
 @return    : true or false
 */
bool Roll_Operations::Save_Rolling_Time(unsigned int time)
{
    unsigned char TimeBuffer[2];
    unsigned char CRC8, CRC8_Temp;

    TimeBuffer[0] = time >> 8;
    TimeBuffer[1] = time & 0xFF;

    /*
      *如果卷膜总行程时间小于15秒或大于8分钟，说明有异常，需要重新重置行程。
      *实验调试测试的行程可能会小于15秒。但是现实的花卉大棚不可能行程小于15秒。
    */
    if (time < 15 || time > 480) return false;

    /*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
    if (TimeBuffer[0] == AT24CXX_ReadOneByte(ROLL_TIME_HIGH_ADDR) && TimeBuffer[1] == AT24CXX_ReadOneByte(ROLL_TIME_LOW_ADDR))
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(ROLL_TIME_HIGH_ADDR, TimeBuffer[0]);
    AT24CXX_WriteOneByte(ROLL_TIME_LOW_ADDR, TimeBuffer[1]);

    CRC8 = GetCrc8(TimeBuffer, sizeof(TimeBuffer));

    AT24CXX_WriteOneByte(ROLL_TIME_VERIFY_ADDR, CRC8);
    EEPROM_Write_Disable();

    TimeBuffer[0] = AT24CXX_ReadOneByte(ROLL_TIME_HIGH_ADDR);
    TimeBuffer[1] = AT24CXX_ReadOneByte(ROLL_TIME_LOW_ADDR);
    CRC8_Temp = GetCrc8(TimeBuffer, sizeof(TimeBuffer));

    if (CRC8_Temp == CRC8)
        return true;
    else
        return false;
}

/*
 @brief     : 读取卷膜总行程时间
 @para      : 无
 @return    : time(S)
 */
unsigned int Roll_Operations::Read_Rolling_Time(void)
{
    unsigned char TimeBuffer[2];
    unsigned char CRC8, CRC8_Temp;

    TimeBuffer[0] = AT24CXX_ReadOneByte(ROLL_TIME_HIGH_ADDR);
    TimeBuffer[1] = AT24CXX_ReadOneByte(ROLL_TIME_LOW_ADDR);

    CRC8 = AT24CXX_ReadOneByte(ROLL_TIME_VERIFY_ADDR);
    CRC8_Temp = GetCrc8(TimeBuffer, sizeof(TimeBuffer));

    if (CRC8 == CRC8_Temp)
        return (TimeBuffer[0] << 8 | TimeBuffer[1]);
    else
        return 0xFFFF;  //ERROR
}

/*
 @brief     : 保存上一次卷膜完成开度值
 @para      : 开度值 (0 - 100)
 @return    : true or false
 */
bool Roll_Operations::Save_Last_Opening_Value(unsigned char opening_value)
{
    unsigned char BKP_CRC8;

    if (opening_value > 100)    
        return false;

    /*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
    unsigned char ReadOpening = bkp_read(BKP_MOTOR_LAST_OPENING_ADDR);
    if ((ReadOpening == opening_value) || ((ReadOpening - 1) == opening_value))
        return true;
    
    /*
      *因为开度为0，数值是0。0的CRC8校验也是0。并且BKP备份寄存器数据丢失后复位的值也是0.
      *这样。保存了0值进去后，等读出来使用数据时，就不知道这个数据是真是的0值，还是复位错值。
      *所以0值就让它暂时为1，CRC8就不会为0。等读出来校验没错后，又把这个值变成0。
      *也就是0值写入备份寄存器的值是1，读出来又把他还原成0。
      *并且在开度上0%开度和1%开度没有什么区别。
     */
    if (opening_value == 0) opening_value = 1;
    BKP_CRC8 = GetCrc8(&opening_value, 1);

    bkp_enable_writes();
    bkp_write(BKP_MOTOR_LAST_OPENING_ADDR, opening_value);
    bkp_write(BKP_MOTOR_LAST_OPENING_CRC_ADDR, BKP_CRC8);
    bkp_disable_writes();

    if (bkp_read(BKP_MOTOR_LAST_OPENING_CRC_ADDR) == BKP_CRC8)
        return true;
    else
        return false;
}

/*
 @brief     : 读取上一次卷膜完成开度的值。优先读取备份寄存器的开度值，如果备份寄存器数据损坏，
              读取EEPROM开度值，并修复备份寄存器数据。如果两个保存地址都损坏，则返回0xFF
 @para      : 无
 @return    : opening value (0 - 100)
 */
unsigned char Roll_Operations::Read_Last_Opening_Value(void)
{
    unsigned char LastOpeningTemp;
    unsigned char CRC8_Temp ;

    LastOpeningTemp = bkp_read(BKP_MOTOR_LAST_OPENING_ADDR);
    CRC8_Temp = GetCrc8(&LastOpeningTemp, 1);

    if (CRC8_Temp == bkp_read(BKP_MOTOR_LAST_OPENING_CRC_ADDR)) //如果备份寄存器数据正常
    {
        /*参考 Save_Last_Opening_Value 函数*/
        if (LastOpeningTemp == 1) LastOpeningTemp = 0;

        return LastOpeningTemp;
    }
    else
    {
        return 0xFF; //Error
    }
    
}

/*
 @brief     : 保存最近一次卷膜完成的开度值
 @para      : Opening value(0 - 100)
 @return    : true or false
 */
bool Roll_Operations::Save_Current_Opening_Value(unsigned char opening_value)
{
    /*参考 Save_Last_Opening_Value 函数*/

    unsigned char BKP_CRC8;
    unsigned char ReadOpening;

    if (opening_value > 100)    
        return false;

    /*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
    ReadOpening = bkp_read(BKP_MOTOR_RECENT_OPENING_ADDR);
    if ((ReadOpening == opening_value) || ((ReadOpening - 1) == opening_value))
       return true;
    
    if (opening_value == 0) opening_value = 1;
    BKP_CRC8 = GetCrc8(&opening_value, 1);

    bkp_enable_writes();
    bkp_write(BKP_MOTOR_RECENT_OPENING_ADDR, opening_value);
    bkp_write(BKP_MOTOR_RECENT_OPENING_CRC_ADDR, BKP_CRC8);
    bkp_disable_writes();  

    if (bkp_read(BKP_MOTOR_RECENT_OPENING_CRC_ADDR) == BKP_CRC8)
        return true;
    else
        return false;
}

/*
 @brief     : 读取最近一次卷膜完成后的开度值
 @para      : None
 @return    : opening value(0 - 100)
 */
unsigned char Roll_Operations::Read_Current_Opening_Value(void)
{
    unsigned char RecentOpeningTemp;
    unsigned char CRC8_Temp ;

    RecentOpeningTemp = bkp_read(BKP_MOTOR_RECENT_OPENING_ADDR);
    CRC8_Temp = GetCrc8(&RecentOpeningTemp, 1);

    if (CRC8_Temp == bkp_read(BKP_MOTOR_RECENT_OPENING_CRC_ADDR)) //如果备份寄存器数据正常
    {
        /*参考 Save_Last_Opening_Value 函数*/
        if (RecentOpeningTemp == 1) RecentOpeningTemp = 0;

        return RecentOpeningTemp;
    }
    else
    {
        return 0xFF;    //Error
    }
}

/*
 @brief     : 保存实时卷膜开度值
 @para      : opening value(0 - 100)
 @return    : true or false
 */
bool Roll_Operations::Save_RealTime_Opening_Value(unsigned char opening_value)
{
    /*参考 Save_Last_Opening_Value 函数*/
    unsigned char BKP_CRC8;

    if (opening_value > 100)    
        return false;

    /*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
    unsigned char ReadOpening = bkp_read(BKP_MOTOR_REALTIME_OPENING_ADDR);
    if ((ReadOpening == opening_value) || ((ReadOpening - 1) == opening_value))
        return true;

    if (opening_value == 0) opening_value = 1;

    BKP_CRC8 = GetCrc8(&opening_value, 1);

    bkp_enable_writes();
    bkp_write(BKP_MOTOR_REALTIME_OPENING_ADDR, opening_value);
    bkp_write(BKP_MOTOR_REALTIME_OPENING_CRC_ADDR, BKP_CRC8);
    bkp_disable_writes();

    if (bkp_read(BKP_MOTOR_REALTIME_OPENING_CRC_ADDR) == BKP_CRC8)
        return true;
    else
        return false;
}

/*
 @brief     : 读取实时卷膜开度值
 @para      : None
 @return    : opening value(0 - 100)
 */
unsigned char Roll_Operations::Read_RealTime_Opening_Value(void)
{
    unsigned char RealTimeOpeningTemp;
    unsigned char CRC8_Temp ;

    RealTimeOpeningTemp = bkp_read(BKP_MOTOR_REALTIME_OPENING_ADDR);
    CRC8_Temp = GetCrc8(&RealTimeOpeningTemp, 1);

    if (CRC8_Temp == bkp_read(BKP_MOTOR_REALTIME_OPENING_CRC_ADDR)) //如果备份寄存器数据正常
    {
        /*参考 Save_Last_Opening_Value 函数*/
        if (RealTimeOpeningTemp == 1) RealTimeOpeningTemp = 0;

        return RealTimeOpeningTemp;
    }
    else
    {
        return 0xFF;
    }
}

/*
 @brief     : 清除所有卷膜开度值，慎用！
 @para      : 无
 @return    : true or false
 */
bool Roll_Operations::Clear_All_Opening_Value(void)
{
    bkp_enable_writes();
    for (unsigned char i = BKP_MOTOR_LAST_OPENING_ADDR; i <= BKP_MOTOR_REALTIME_OPENING_CRC_ADDR; i++)
    {
        if (bkp_read(i) != 0)
        {
            bkp_write(i, 0x00);
            if (bkp_read(i) != 0)
            {
                bkp_disable_writes();
                return false;
            }
        }
    }
    bkp_disable_writes();

    EEPROM_Write_Enable();
    for (unsigned char i = EP_MOTOR_LAST_OPENING_ADDR; i <= EP_MOTOR_REALTIME_OPENING_CRC_ADDR; i++)
    {
        if (AT24CXX_ReadOneByte(i) != 0x00) //原来的开度值不是0，才去清零，避免重复擦除EEPROM
        {
            AT24CXX_WriteOneByte(i, 0x00);
            if (AT24CXX_ReadOneByte(i) != 0x00)
            {
                EEPROM_Write_Disable();
                return false;
            }
        }
    }
    EEPROM_Write_Disable();

    return true;
}

/*
 @brief     : 保存工作组号
 @para      : group number(array, 5byte)
 @return    : true or false
 */
bool Roll_Operations::Save_Group_Number(unsigned char *group_num)
{
    bool SaveGroupFlag = false;

    /*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
    for (unsigned char i = 0; i < 5; i++)
    {
        if (group_num[i] != AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i))
        {
            SaveGroupFlag = true;
            break;
        }
    }
    if (SaveGroupFlag == false) return true;

    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 5; i++)
        AT24CXX_WriteOneByte(GROUP_NUMBER_BASE_ADDR + i, group_num[i]);

    unsigned char CRC8 = GetCrc8(&group_num[0], 5);
    AT24CXX_WriteOneByte(GROUP_NUMBER_VERIFY_ADDR, CRC8);

    /*判断是否保存数据成功*/
    for (unsigned char i = 0; i < 5; i++)
    {
        if (AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i) != group_num[i])
        {
            EEPROM_Write_Disable();
             return false;
        }
    }

    if (AT24CXX_ReadOneByte(GROUP_NUMBER_VERIFY_ADDR) == CRC8)
    {
        if (AT24CXX_ReadOneByte(GROUP_NUMBER_FLAG_ADDR) != 0x55)
        {
            AT24CXX_WriteOneByte(GROUP_NUMBER_FLAG_ADDR, 0x55); //保存数据成功，置标志位
        }
        EEPROM_Write_Disable();
        return true;
    }
    else
    {
        EEPROM_Write_Disable();
        return false;
    }
}

/*
 @brief     : 读取工作组号
 @para      : workgroup number(array 5byte)
 @return    : true or false
 */
bool Roll_Operations::Read_Group_Number(unsigned char *group_num)
{
    unsigned char GroupNumber[5];
    unsigned char CRC8;
    for (unsigned char i = 0; i < 5; i++)
        GroupNumber[i] = AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i);

    CRC8 = GetCrc8(&GroupNumber[0], 5);
    if (CRC8 != AT24CXX_ReadOneByte(GROUP_NUMBER_VERIFY_ADDR))
        return false;
    else
    {
        for (unsigned char i = 0; i < 5; i++)
            group_num[i] = GroupNumber[i];
    }
    return true;
}

/*
 @brief     : 验证保存的工作组号是否损坏
 @para      : None
 @return    : true or false
*/
bool Roll_Operations::Check_Group_Number(void)
{
    unsigned char GroupTemp[5]; 

    for (unsigned char i = 0; i < 5; i++)
        GroupTemp[i] =AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i);

    unsigned char VerifyCRC8_Temp = GetCrc8(&GroupTemp[0], 5);
    if (VerifyCRC8_Temp == AT24CXX_ReadOneByte(GROUP_NUMBER_VERIFY_ADDR))
        return true;
    else
        return false;
}

/*
 @brief     : 验证是否已经保存过工作组号
 @para      : None
 @return    : true or false
 */
bool Roll_Operations::Verify_Group_Number_Flag(void)
{
    bool BoolValue;
    AT24CXX_ReadOneByte(GROUP_NUMBER_FLAG_ADDR) == 0X55 ? BoolValue = true : BoolValue = false;
    return BoolValue;
}

/*
 @brief     : 清除本机的工作组号，慎用！
 @para      : None
 @return    : true or false              
 */
bool Roll_Operations::Clear_Group_Number(void)
{
    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 5; i++)
    {
        if (AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i) != 0x00)
        {
            AT24CXX_WriteOneByte(GROUP_NUMBER_BASE_ADDR + i, 0x00);
            if (AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i) != 0x00)
            {
                EEPROM_Write_Disable();
                return false;
            }
        }
    }
    if (AT24CXX_ReadOneByte(GROUP_NUMBER_FLAG_ADDR) != 0x00)    //保护EP，防止重复擦写。
        AT24CXX_WriteOneByte(GROUP_NUMBER_FLAG_ADDR, 0x00); //同时清除保存组号标志位

    EEPROM_Write_Disable();
    return true;
}

/*
 @brief     : 保存区域号
 @para      : area number
 @return    : true or false              
 */
bool Roll_Operations::Save_Area_Number(unsigned char area_num)
{
    /*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
    if (area_num == AT24CXX_ReadOneByte(AREA_ADDR))
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(AREA_ADDR, area_num);
    AT24CXX_WriteOneByte(AREA_VERIFY_ADDR, GetCrc8(&area_num, 1));
    /*验证是否保存数据成功*/
    if (AT24CXX_ReadOneByte(AREA_ADDR) == area_num)
    {
        if (AT24CXX_ReadOneByte(AREA_FLAG_ADDR) != 0x55)
        {
            AT24CXX_WriteOneByte(AREA_FLAG_ADDR, 0x55);
        }
        EEPROM_Write_Disable();
        return true;
    }
    else
    {
        EEPROM_Write_Disable();
        return false;
    }
}

/*
 @brief     : 读取区域号
 @para      : None
 @return    : Area number.
 */
unsigned char Roll_Operations::Read_Area_Number(void)
{
    return (AT24CXX_ReadOneByte(AREA_ADDR));
}

/*
 @brief     : 验证保存的区域号是否损坏
 @para      : None
 @return    : true or false
*/
bool Roll_Operations::Check_Area_Number(void)
{
    unsigned char AreaTemp = AT24CXX_ReadOneByte(AREA_ADDR);
    unsigned char VerifyTemp = GetCrc8(&AreaTemp, 1);

    if (VerifyTemp == AT24CXX_ReadOneByte(AREA_VERIFY_ADDR))
        return true;
    else
        return false;
}

/*
 @brief     : 验证区域号是否已经保存
 @para      : None
 @return    : true or false
 */
bool Roll_Operations::Verify_Area_Number_Flag(void)
{
    bool BoolValue;
    AT24CXX_ReadOneByte(AREA_FLAG_ADDR) == 0x55 ? BoolValue = true : BoolValue = false;
    return BoolValue;
}

/*
 @brief     : 清除区域号，慎用！
 @para      : None
 @return    : true or fasle
 */
bool Roll_Operations::Clear_Area_Number(void)
{
    if (AT24CXX_ReadOneByte(AREA_ADDR) == 0x00)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(AREA_ADDR, 0X00);  //默认区域号
    if (AT24CXX_ReadOneByte(AREA_ADDR) != 0x00)
    {
        EEPROM_Write_Disable();
        return false;
    }
    else
    {
        EEPROM_Write_Disable();
        return true;
    }
}

/*
 @brief     : 保存卷膜工作电压阈值和状态上报间隔
 @para      : 电压阈值、状态上报频率等，5bytes
 @return    : true or false                       
 */
bool Roll_Operations::Save_Roll_Work_Voltage_and_Report_Interval(unsigned char *threshold_value)
{
    /*如果本次要保存的数据与已经保存的数据相同，为了维护储存器，不再重复保存*/
    bool SaveFlag = false;

    for (unsigned char i = 0; i < 5; i++)
    {
        if (threshold_value[i] != AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + i))
        {
            SaveFlag = true;
            break;
        }
    }
    if (!SaveFlag)
        return true;

    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 5; i++)
    {
        AT24CXX_WriteOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + i, threshold_value[i]);
        if (AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + i) != threshold_value[i])
        {
            EEPROM_Write_Disable();
            return false;
        }
    }
    if (AT24CXX_ReadOneByte(SAVE_ROLL_THRESHOLD_FLAG_ADDR) != 0x55)
        AT24CXX_WriteOneByte(SAVE_ROLL_THRESHOLD_FLAG_ADDR, 0x55);
        
    EEPROM_Write_Disable();
    return true;
}

/*
 @brief     : 读取最小电压阈值
 @para      : 无
 @return    : 最小电压值
 */
unsigned char Roll_Operations::Read_Roll_Low_Voltage_Limit_Value(void)
{
    if (AT24CXX_ReadOneByte(SAVE_ROLL_THRESHOLD_FLAG_ADDR) != 0x55)
        return 20;

    unsigned char Low_Current_Value = AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + 1);
    
    if (Low_Current_Value < 1 || Low_Current_Value > 100)
        Low_Current_Value = 20;

    return Low_Current_Value;
}

/*
 @brief     : 读取最大电流阈值。阈值步进0.1倍，最大10倍
 @para      : 无
 @return    : 电流阈值
*/
unsigned char Roll_Operations::Read_Roll_High_Current_Limit_Value(void)
{
    if (AT24CXX_ReadOneByte(SAVE_ROLL_THRESHOLD_FLAG_ADDR) != 0x55)
    return 20;  //默认2.0倍电流阈值
    
    unsigned char HighCurrentValue = AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + 3);

    if (HighCurrentValue < 1 || HighCurrentValue > 100) //步进0.1倍，最大10倍
        HighCurrentValue = 20;  //如果设置的电流阈值小于0.1倍或大于10倍，默认2倍。

    return HighCurrentValue;
}

/*
 @brief     : 读取上报实时状态间隔值
              Read report interval value of realtime status.
 @para      : None
 @return    : interval value.
 */
unsigned char Roll_Operations::Read_Roll_Report_Status_Interval_Value(void)
{
    unsigned char interval = AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + 4);
    if (interval >= 1 && interval <= 10)
        return interval;
    else
        return 3;
}

/*
 @brief     : 保存卷膜电压值
 @param     : 电压值
 @return    : true or false
 */
bool Roll_Operations::Save_Roll_Voltage(unsigned int voltage)
{
    unsigned char VoltageBuffer[2];
    unsigned char VoltageVerify, VoltageVerifyTemp;
    unsigned int VolTemp;

    /* 如果保存的电压值和要保存的电压值相差100mV以内，视为相等，不重复保存 */
    VolTemp = ((AT24CXX_ReadOneByte(ROLL_VOLTAGE_HIGH_ADDR) << 8) | AT24CXX_ReadOneByte(ROLL_VOLTAGE_LOW_ADDR));
    if (((voltage - VolTemp) <= 100) || ((VolTemp - voltage) <= 100))
        return true;

    VoltageBuffer[0] = highByte(voltage);
    VoltageBuffer[1] = lowByte(voltage);

    VoltageVerify = GetCrc8(&VoltageBuffer[0], sizeof(VoltageBuffer));

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(ROLL_VOLTAGE_HIGH_ADDR, VoltageBuffer[0]);
    AT24CXX_WriteOneByte(ROLL_VOLTAGE_LOW_ADDR,  VoltageBuffer[1]);
    AT24CXX_WriteOneByte(ROLL_VOLTAGE_VERIFY_ADDR, VoltageVerify);    
    EEPROM_Write_Disable();

    VoltageBuffer[0] = AT24CXX_ReadOneByte(ROLL_VOLTAGE_HIGH_ADDR);
    VoltageBuffer[1] = AT24CXX_ReadOneByte(ROLL_VOLTAGE_LOW_ADDR);
    VoltageVerifyTemp = GetCrc8(&VoltageBuffer[0], sizeof(VoltageBuffer));


    if (AT24CXX_ReadOneByte(ROLL_VOLTAGE_VERIFY_ADDR) == VoltageVerifyTemp)
        return true;
    else
        return false;
}

/*
 @brief     : 读取卷膜电压值
 @para      : *voltage
 @return    : true or false
 */
bool Roll_Operations::Read_Roll_Voltage(unsigned int *voltage)
{
    unsigned char VoltageBuffer[2];
    unsigned char VoltageCRC8;

    VoltageBuffer[0] = AT24CXX_ReadOneByte(ROLL_VOLTAGE_HIGH_ADDR);
    VoltageBuffer[1] = AT24CXX_ReadOneByte(ROLL_VOLTAGE_LOW_ADDR);

    VoltageCRC8 = GetCrc8(VoltageBuffer, sizeof(VoltageBuffer));
    if (VoltageCRC8 != AT24CXX_ReadOneByte(ROLL_VOLTAGE_VERIFY_ADDR))
        return false;
    else
    {
        *voltage = ((VoltageBuffer[0] << 8) | VoltageBuffer[1]);
        /*如果电压值小于14V，返回假*/
        if (*voltage < 14000)
            return false;
        else
        return true;
    }
}

/*
 @brief     : 保存开棚卷膜电流值
 @para      : current value
 @return    : true or false
 */
bool Roll_Operations::Save_Roll_Up_Current(unsigned int current)
{
    unsigned char CurrentBuffer[2];
    unsigned char CurrentVerify, CurrentVerifyTemp;
    unsigned int CurrentTemp;
    int CurrentDiffer;

    /*采集的电流值与保存的电流值在正负100mA之间，视为相等，不重复保存*/
    CurrentTemp = ((AT24CXX_ReadOneByte(ROLL_UP_CURRENT_HIGH_ADDR) << 8) | AT24CXX_ReadOneByte(ROLL_UP_CURRENT_LOW_ADDR));
    CurrentDiffer = (int)CurrentTemp - (int)current;

    if ((CurrentDiffer <= 100) && (CurrentDiffer>= 0))
        return true;
    if ((CurrentDiffer >= -100) && (CurrentDiffer <= 0))
        return true;

    CurrentBuffer[0] = highByte(current);
    CurrentBuffer[1] = lowByte(current);

    CurrentVerify = GetCrc8(&CurrentBuffer[0], sizeof(CurrentBuffer));

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(ROLL_UP_CURRENT_HIGH_ADDR, CurrentBuffer[0]);
    AT24CXX_WriteOneByte(ROLL_UP_CURRENT_LOW_ADDR,  CurrentBuffer[1]);
    AT24CXX_WriteOneByte(ROLL_UP_CURRENT_VERIFY_ADDR, CurrentVerify);    

    CurrentBuffer[0] = AT24CXX_ReadOneByte(ROLL_UP_CURRENT_HIGH_ADDR);
    CurrentBuffer[1] = AT24CXX_ReadOneByte(ROLL_UP_CURRENT_LOW_ADDR);
    CurrentVerifyTemp = GetCrc8(&CurrentBuffer[0], sizeof(CurrentBuffer));

    if (AT24CXX_ReadOneByte(ROLL_UP_CURRENT_VERIFY_ADDR) == CurrentVerifyTemp)
    {
        if (AT24CXX_ReadOneByte(SAVE_UP_CURRENT_FLAG_ADDR) != 0x55)
        {
            AT24CXX_WriteOneByte(SAVE_UP_CURRENT_FLAG_ADDR, 0x55);
        }
        EEPROM_Write_Disable();
        return true;
    }
    else
    {
        EEPROM_Write_Disable();
        return false;
    }
}

/*
 @brief     : 读取开棚卷膜电流值
 @para      : *current
 @return    : true or false
 */
bool Roll_Operations::Read_Roll_Up_Current(unsigned int *current)
{
    unsigned char CurrentBuffer[2];
    unsigned char CurrentCRC8;

    CurrentBuffer[0] = AT24CXX_ReadOneByte(ROLL_UP_CURRENT_HIGH_ADDR);
    CurrentBuffer[1] = AT24CXX_ReadOneByte(ROLL_UP_CURRENT_LOW_ADDR);

    CurrentCRC8 = GetCrc8(CurrentBuffer, sizeof(CurrentBuffer));
    if (CurrentCRC8 != AT24CXX_ReadOneByte(ROLL_UP_CURRENT_VERIFY_ADDR))
        return false;
    else
    {
        *current = ((CurrentBuffer[0] << 8) | CurrentBuffer[1]);
        /*如果因为某些原因采集电流过低，默认赋值800mA*/
        if (*current < 300) *current = 800;
        return true;
    }
}

/*
 @brief     : 保存关棚卷膜电流值
 @para      : current value
 @return    : true or false
 */
bool Roll_Operations::Save_Roll_Down_Current(unsigned int current)
{
    unsigned char CurrentBuffer[2];
    unsigned char CurrentVerify, CurrentVerifyTemp;
    unsigned int CurrentTemp;
    int CurrentDiffer;

    /*采集的电流值与保存的电流值在正负100mA之间，视为相等，不重复保存*/
    CurrentTemp = ((AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_HIGH_ADDR) << 8) | AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_LOW_ADDR));
    CurrentDiffer = (int)CurrentTemp - (int)current;

    if ((CurrentDiffer <= 100) && (CurrentDiffer>= 0))
        return true;
    if ((CurrentDiffer >= -100) && (CurrentDiffer <= 0))
        return true;

    CurrentBuffer[0] = highByte(current);
    CurrentBuffer[1] = lowByte(current);

    CurrentVerify = GetCrc8(&CurrentBuffer[0], sizeof(CurrentBuffer));

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(ROLL_DOWN_CURRENT_HIGH_ADDR, CurrentBuffer[0]);
    AT24CXX_WriteOneByte(ROLL_DOWN_CURRENT_LOW_ADDR,  CurrentBuffer[1]);
    AT24CXX_WriteOneByte(ROLL_DOWN_CURRENT_VERIFY_ADDR, CurrentVerify);    

    CurrentBuffer[0] = AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_HIGH_ADDR);
    CurrentBuffer[1] = AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_LOW_ADDR);
    CurrentVerifyTemp = GetCrc8(&CurrentBuffer[0], sizeof(CurrentBuffer));

    if (AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_VERIFY_ADDR) == CurrentVerifyTemp)
    {
        if (AT24CXX_ReadOneByte(SAVE_DOWN_CURRENT_FLAG_ADDR) != 0x55)
        {
            AT24CXX_WriteOneByte(SAVE_DOWN_CURRENT_FLAG_ADDR, 0x55);
        }
        EEPROM_Write_Disable();
        return true;
    }
    else
    {
        EEPROM_Write_Disable();
        return false;
    }
}

/*
 @brief     : 读取关棚卷膜电流值
 @para      : *current value
 @return    : true or false
 */
bool Roll_Operations::Read_Roll_Down_Current(unsigned int *current)
{
    unsigned char CurrentBuffer[2];
    unsigned char CurrentCRC8;

    CurrentBuffer[0] = AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_HIGH_ADDR);
    CurrentBuffer[1] = AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_LOW_ADDR);

    CurrentCRC8 = GetCrc8(CurrentBuffer, sizeof(CurrentBuffer));
    if (CurrentCRC8 != AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_VERIFY_ADDR))
        return false;
    else
    {
        *current = ((CurrentBuffer[0] << 8) | CurrentBuffer[1]);
        /*如果因为某些原因采集电流过低，默认赋值800mA*/
        if (*current < 300) *current = 800;
        return true;
    }
}

/*
 @brief     : 验证是否已保存过卷膜电流值
 @para      : None
 @return    : true or false
 */
bool Roll_Operations::Verify_Current_Flag(void)
{
    unsigned char UpFlag = 0, DownFlag = 0;
    bool BoolVal;
    UpFlag = AT24CXX_ReadOneByte(SAVE_UP_CURRENT_FLAG_ADDR);
    DownFlag = AT24CXX_ReadOneByte(SAVE_DOWN_CURRENT_FLAG_ADDR);

    UpFlag == 0x55 ? (DownFlag == 0x55 ? BoolVal = true : BoolVal = false) : BoolVal = false;

    return BoolVal;
}

/*
 @brief     : 清除保存电流值标志位
 @para      : 无
 @return    : 无
 */
bool Roll_Operations::Clear_Current_Flag(void)
{
    if (AT24CXX_ReadOneByte(SAVE_UP_CURRENT_FLAG_ADDR) == 0x00 && AT24CXX_ReadOneByte(SAVE_DOWN_CURRENT_FLAG_ADDR) == 0x00)
        return true;

    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SAVE_UP_CURRENT_FLAG_ADDR, 0x00);
    AT24CXX_WriteOneByte(SAVE_DOWN_CURRENT_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();
    return true;
}

/*
 @brief     : 自检保存的卷膜电流值是否有效
 @para      : 无
 @return    : true or false
 */
bool Roll_Operations::Current_Self_Check(void)
{
    unsigned char CurrentBuffer[2];
    unsigned char CurrentVerify, CurrentVerifyTemp;

    /*自检开棚电流值*/
    if (AT24CXX_ReadOneByte(SAVE_UP_CURRENT_FLAG_ADDR) == 0x55)
    {
        CurrentBuffer[0] = AT24CXX_ReadOneByte(ROLL_UP_CURRENT_HIGH_ADDR);
        CurrentBuffer[1] = AT24CXX_ReadOneByte(ROLL_UP_CURRENT_LOW_ADDR);    

        CurrentVerify = GetCrc8(CurrentBuffer, sizeof(CurrentBuffer));
        CurrentVerifyTemp = AT24CXX_ReadOneByte(ROLL_UP_CURRENT_VERIFY_ADDR);

        if (CurrentVerify != CurrentVerifyTemp)  return false;
    }
    /*自检关棚电流值*/
    if (AT24CXX_ReadOneByte(SAVE_DOWN_CURRENT_FLAG_ADDR) == 0x55)
    {
        CurrentBuffer[0] = AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_HIGH_ADDR);
        CurrentBuffer[1] = AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_LOW_ADDR);    

        CurrentVerify = GetCrc8(CurrentBuffer, sizeof(CurrentBuffer));
        CurrentVerifyTemp = AT24CXX_ReadOneByte(ROLL_DOWN_CURRENT_VERIFY_ADDR);

        if (CurrentVerify != CurrentVerifyTemp)  return false;
    }
    return true;
}