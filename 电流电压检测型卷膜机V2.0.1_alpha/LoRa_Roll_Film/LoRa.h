#ifndef _LORA_H
#define _LORA_H

#include <Arduino.h>

#define USE_LORA_RESET  1

//GPIO definition
#define RESET_PIN       PB4
#define WAKEUP_PIN      PB3
#define AT_CMD_PIN      PD2

#define LORA_PWR_PIN    PA1

#define LoRa_Serial     Serial1  //USART2 --> USART1, when use serial upload

#define LORA_PWR_ON     (digitalWrite(LORA_PWR_PIN, HIGH))
#define LORA_PWR_OFF    (digitalWrite(LORA_PWR_PIN, LOW))

#define SOFT_AT                     "+++"
#define SOFT_PATH                   "ATT\r\n"

/*LoRa AT Command*/
//Inquiry command
#define AT_ADDR_                    "AT+ADDR?\r\n"  //return 4 byte
#define AT_MADDR_                   "AT+MADDR?\r\n" //return 4 byte
#define AT_ID_                      "AT+ID?\r\n"    //return 8 byte
#define AT_SYNC_                    "AT+SYNC?\r\n"  //return 1 byte
#define AT_POW_                     "AT+POW?\r\n"   //return 1 byte 
#define AT_BW_                      "AT+BW?\r\n"    //return 1 byte
#define AT_CR_                      "AT+CR?\r\n"    //return 1 byte
#define AT_CRC_                     "AT+CRC?\r\n"   //return 1 byte
#define AT_TFREQ_                   "AT+TFREQ?\r\n" //return 4 byte
#define AT_RFREQ_                   "AT+RFREQ?\r\n" //return 4 byte
#define AT_TSF_                     "AT+TSF?\r\n"   //return 1 byte
#define AT_RSF_                     "AT+RSF?\r\n"   //return 1 byte
#define AT_CSQ_                     "AT+CSQ?\r\n"   //return 2 byte
#define AT_CFG_                     "AT+CFG?\r\n"
#define AT_TIQ_                      "AT+TIQ?\r\n"
#define AT_RIQ_                     "AT+RIQ?\r\n"
#define AT_NET_                     "AT+NET?\r\n"
#define AT_SIP_                     "AT+SIP?\r\n"

#define AT_INQUIRE_PARA(at)         at"\r\n"   

//Set command
#define AT_NET                      "AT+NET="
#define AT_AK                       "AT+AK="
#define AT_ADDR                     "AT+ADDR="    
#define AT_MADDR                    "AT+MADDR="
#define AT_MODE                     "AT+MODE="
#define AT_PREM                     "AT+PREM="
#define AT_LDR                      "AT+LDR="
#define AT_SYNC                     "AT+SYNC="
#define AT_POW                      "AT+POW="
#define AT_BW                       "AT+BW="
#define AT_CR                       "AT+CR="
#define AT_CRC                      "AT+CRC="
#define AT_TFREQ                    "AT+TFREQ="
#define AT_RFREQ                    "AT+RFREQ="
#define AT_TSF                      "AT+TSF="
#define AT_RSF                      "AT+RSF="
#define AT_TIQ                      "AT+TIQ="
#define AT_RIQ                      "AT+RIQ="
#define AT_SIP                      "AT+SIP="
#define AT_ACK                      "AT+ACK="
#define AT_BRATE                    "AT+BRATE="
#define AT_EL                       "AT+EL="

#define AT_CONFIG_PARA(at, para)    at#para"\r\n"

enum LoRa_Mode{
    AT = 0, PASS_THROUGH_MODE
};

enum Receive_Type{
    OK = 0, ERROR, Bytes, Invalid
};

enum Error_Mark{
    Grammar_Err = 0, Para_Err, Execute_Failed, Channel_Busy, Length_Err, Save_Failed, Buffer_Full, OverTime, Set_Refuse, Unreadable,
    No_Err
};

enum Cmd{
    CMOMON = 0, CSQ
};

class LoRa{
public:
    void LoRa_GPIO_Config(void);
    void LoRa_Shutdown(void);
    void LoRa_Restart(void);
    void BaudRate(unsigned int baudrate);
    //AT mode or pass-through mode
    bool Mode(LoRa_Mode AT_status);
    void IsReset(bool Is_reset);
    bool LoRa_AT(unsigned char *data_buffer, bool is_query, const char *cmd, const char *para);
    
    bool Rewrite_ID(void);
    void Parameter_Init(bool only_net);

private:
    unsigned char Detect_Error_Receipt(unsigned char *verify_data);
    bool Detect_OK_Receipt(unsigned char *verify_data, unsigned char *data_buffer);
    //Read LoRa parameters.
    Receive_Type AT_Query_Cmd(const char *cmd, unsigned char *data_buffer, unsigned char *data_len);
    //Set LoRa parameters.
    Receive_Type AT_Config_Cmd(const char *cmd, const char * para, unsigned char *data_buffer);
    bool Parse_Command(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer, unsigned char **data_len);
    //SNR and RSSI
    void Get_CSQ(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer);
    bool String_to_Hex(unsigned char *str, unsigned char len);

    bool Param_Check(const char *cmd, const char *para, bool only_set);
};

/*Create LoRa object*/
extern LoRa LoRa_MHL9LF;

#endif
