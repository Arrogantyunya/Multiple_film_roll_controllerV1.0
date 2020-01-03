#ifndef _Memory_H
#define _Memory_H

#include "AT24CXX.h"
#include <libmaple/bkp.h>
#include <Arduino.h>

#define EP_WP_PIN                               PB5

/*使用EEPROM储存芯片的宏定义地址*/
/*EEPROM硬件特性*/
#define EEPROM_MIN_ADDR                         0
#define EEPROM_MAX_ADDR                         255 

/*SN码相关操作保存地址*/
#define SN_OPERATION_FLAG_ADDR                  11
#define SN_BKP_OPERATION_FLAG_ADDR              12
#define SN_BASE_ADDR                            13
#define SN_END_ADDR                             21
#define SN_VERIFY_ADDR                          22
#define SN_BKP_BASE_ADDR                        23
#define SN_BKP_END_ADDR                         31
#define SN_BKP_VERIFY_ADDR                      32
#define SN_ACCESS_NETWORK_FLAG_ADDR             33
/*软件版本和硬件版本保存地址*/
#define SOFT_HARD_VERSION_BASE_ADDR             34
#define SOFT_HARD_VERSION_END_ADDR              37
/*成功重置卷膜总行程标志位保存地址*/
#define ROUTE_FLAG_ADDR                         38
/*卷膜总行程时长保存地址*/
#define ROLL_TIME_HIGH_ADDR                     39
#define ROLL_TIME_LOW_ADDR                      40
#define ROLL_TIME_VERIFY_ADDR                   41
/*电机相关电压、电流、上报频率等参数保存地址*/
#define ROLL_THRESHOLD_VALUE_BASE_ADDR          42 
#define ROLL_THRESHOLD_VALUE_END_ADDR           46
/*卷膜机工作组号相关操作保存地址*/
#define GROUP_NUMBER_BASE_ADDR                  47
#define GROUP_NUMBER_END_ADDR                   51
#define GROUP_NUMBER_VERIFY_ADDR                52
#define GROUP_NUMBER_FLAG_ADDR                  53
/*卷膜机工作区域号相关操作保存地址*/
#define AREA_ADDR                               54
#define AREA_VERIFY_ADDR                        55
#define AREA_FLAG_ADDR                          56
/*卷膜机开棚工作电流保存地址*/
#define ROLL_UP_CURRENT_HIGH_ADDR               57
#define ROLL_UP_CURRENT_LOW_ADDR                58
#define ROLL_UP_CURRENT_VERIFY_ADDR             59
#define SAVE_UP_CURRENT_FLAG_ADDR               60
/*卷膜机关棚工作电流保存地址*/
#define ROLL_DOWN_CURRENT_HIGH_ADDR             61
#define ROLL_DOWN_CURRENT_LOW_ADDR              62
#define ROLL_DOWN_CURRENT_VERIFY_ADDR           63
#define SAVE_DOWN_CURRENT_FLAG_ADDR             64
/*卷膜机上一次开度值*/
#define EP_MOTOR_LAST_OPENING_ADDR              65
#define EP_MOTOR_LAST_OPENING_CRC_ADDR          66
/*卷膜机本次开度值*/
#define EP_MOTOR_RECENT_OPENING_ADDR            67
#define EP_MOTOR_RECENT_OPENING_CRC_ADDR        68
/*卷膜机实时开度值*/
#define EP_MOTOR_REALTIME_OPENING_ADDR          69
#define EP_MOTOR_REALTIME_OPENING_CRC_ADDR      70
/*卷膜相关阈值参数保存标志位保存地址*/
#define SAVE_ROLL_THRESHOLD_FLAG_ADDR           71
/*配置LoRa参数完成标志位保存地址*/
#define LORA_PARA_CONFIG_FLAG_ADDR              72  
/*卷膜工作电机电压保存地址*/
#define ROLL_VOLTAGE_HIGH_ADDR                  73            
#define ROLL_VOLTAGE_LOW_ADDR                   74
#define ROLL_VOLTAGE_VERIFY_ADDR                75

/* LoRa通信模式配置成网关还是节点 0xF0节点; 0xF1网关 */
#define LORA_COM_MODE_ADDR                      76
#define LORA_COM_MODE_FLAG_ADDR                 77
#define LORA_COM_MODE_VERIFY_ADDR               78

#define EP_LORA_ADDR_BASE_ADDR                  79
#define EP_LORA_ADDR_END_ADDR                   86
#define EP_LORA_ADDR_VERIFY_ADDR                87
#define EP_LORA_ADDR_SAVED_FLAG_ADDR            88

/*使用芯片自带备份寄存器的宏定义地址*/
/*上一次开度值保存地址（0 - 100）*/
#define BKP_MOTOR_LAST_OPENING_ADDR             1
/*上一次开度值CRC8保存地址*/
#define BKP_MOTOR_LAST_OPENING_CRC_ADDR         2
/*本次开度值保存地址（0 - 100）*/
#define BKP_MOTOR_RECENT_OPENING_ADDR           3
/*本次开度值CRC8保存地址*/
#define BKP_MOTOR_RECENT_OPENING_CRC_ADDR       4
/*实时开度值保存地址*/
#define BKP_MOTOR_REALTIME_OPENING_ADDR         5 
/*实时开度值CRC8保存地址*/
#define BKP_MOTOR_REALTIME_OPENING_CRC_ADDR     6    

/*
 @brief     : 上拉该引脚，禁止EEPROM写操作
 @para      : 无
 @return    : 无
 */
inline void EEPROM_Write_Disable(void)
{
    digitalWrite(EP_WP_PIN, HIGH);
}

/*
 @brief     : 下拉该引脚，允许EEPROM写操作
 @para      : 无
 @return    : 无
 */
inline void EEPROM_Write_Enable(void)
{
    digitalWrite(EP_WP_PIN, LOW);
}

class EEPROM_Operations : protected AT24Cxx{
public:
    void EEPROM_GPIO_Config(void);
};

class SN_Operations : public EEPROM_Operations{
public:
    bool Save_SN_Code(unsigned char *sn_code);
    bool Save_BKP_SN_Code(unsigned char *sn_code);
    bool Read_SN_Code(unsigned char *sn_code);
    bool Read_BKP_SN_Code(unsigned char *sn_code);
    bool Verify_Save_SN_Code(void);
    bool Verify_Save_BKP_SN_Code(void);
    bool Clear_SN_Save_Flag(void);
    bool Clear_BKP_SN_Save_Flag(void);
    bool Set_SN_Access_Network_Flag(void);
    bool Clear_SN_Access_Network_Flag(void);
    bool Verify_SN_Access_Network_Flag(void);
    bool Self_check(unsigned char *dat);

    void Read_Random_Seed(unsigned char *random_seed);
};

class LoRa_Config : public EEPROM_Operations{
public:
    bool Save_LoRa_Config_Flag(void);
    bool Verify_LoRa_Config_Flag(void);
    bool Clear_LoRa_Config_Flag(void);

    bool Save_LoRa_Com_Mode_Flag(void);
    bool Clear_LoRa_Com_Mode_Flag(void);
    bool Save_LoRa_Com_Mode(unsigned char mode);
    unsigned char Read_LoRa_Com_Mode(void);

    void Clear_LoRa_Addr_Flag(void);
    void Save_LoRa_Addr_Flag(void);
    bool Verify_LoRa_Addr_Flag(void);
    bool Read_LoRa_Addr(unsigned char *addr);
    bool Save_LoRa_Addr(unsigned char *addr);
};

class Soft_Hard_Vertion : public EEPROM_Operations{
public:
    void Save_hardware_version(unsigned char number_high, unsigned char number_low);
    void Save_Software_version(unsigned char number_high, unsigned char number_low);
};

class Roll_Operations : public EEPROM_Operations{
public:
    bool Set_Route_Save_Flag(void);
    bool Read_Route_Save_Flag(void);
    bool Clear_Route_Save_Flag(void);

    bool Save_Rolling_Time(unsigned int time);
    unsigned int Read_Rolling_Time(void);

    bool Save_Last_Opening_Value(unsigned char opening_value);
    bool Save_Current_Opening_Value(unsigned char opening);
    bool Save_RealTime_Opening_Value(unsigned char opening);
    unsigned char Read_Last_Opening_Value(void);
    unsigned char Read_Current_Opening_Value(void);
    unsigned char Read_RealTime_Opening_Value(void);
    bool Clear_All_Opening_Value(void);

    bool Save_Group_Number(unsigned char *group_num);
    bool Read_Group_Number(unsigned char *group_num);
    bool Check_Group_Number(void);
    bool Verify_Group_Number_Flag(void);
    bool Clear_Group_Number(void);

    bool Save_Area_Number(unsigned char area_num);
    unsigned char Read_Area_Number(void);
    bool Check_Area_Number(void);
    bool Verify_Area_Number_Flag(void);
    bool Clear_Area_Number(void);

    bool Save_Roll_Work_Voltage_and_Report_Interval(unsigned char *voltage_value);
    unsigned char Read_Roll_Low_Voltage_Limit_Value(void);
    unsigned char Read_Roll_High_Current_Limit_Value(void);
    unsigned char Read_Roll_Report_Status_Interval_Value(void);

    bool Save_Roll_Voltage(unsigned int voltage);
    bool Read_Roll_Voltage(unsigned int *voltage);

    bool Save_Roll_Up_Current(unsigned int current);
    bool Read_Roll_Up_Current(unsigned int *current);
    bool Save_Roll_Down_Current(unsigned int current);
    bool Read_Roll_Down_Current(unsigned int *current);
    bool Verify_Current_Flag(void);
    bool Clear_Current_Flag(void);
    bool Current_Self_Check(void);
};

/*Create EEPROM object*/
extern EEPROM_Operations EEPROM_Operation;
/*Create SN code operation object*/
extern SN_Operations SN;
/*Create LoRa Config object*/
extern LoRa_Config LoRa_Para_Config;
/*Create roll operation object*/
extern Roll_Operations Roll_Operation;
/*Create software and hardware object*/
extern Soft_Hard_Vertion Vertion;

#endif
