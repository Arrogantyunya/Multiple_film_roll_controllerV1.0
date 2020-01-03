#ifndef _COMMAND_ANALYSIS_H
#define _COMMAND_ANALYSIS_H

#include <Arduino.h>

enum Frame_ID{
  Work_Para, Set_Group_Num, SN_Area_Channel, Work_Status, ResetRoll, Opening, Work_Limit, Stop_Work
};

class Command_Analysis{
public:
  void Receive_LoRa_Cmd(void);

private:
  Frame_ID FrameID_Analysis(void);
  bool Verify_CRC8(unsigned char verify_data_base_addr, unsigned char verify_data_len);
  bool Verify_Device_Type_ID(void);
  void Verify_Mass_Commands(void);
  bool Verify_Area_Number(void);
  bool Verify_Work_Group(void);
  bool Verify_Frame_Validity(unsigned char verify_data_base_addr, unsigned char verify_data_len, bool area_flag, bool group_flag);
  
private:
  void Receive_Data_Analysis(void);
  void Query_Current_Work_Param(void);
  void Set_Group_Number(void);
  void Set_SN_Area_Channel(void);
  void Detailed_Work_Status(void);
  void ResetRoll_Command(void);
  void Opening_Command(void);
  void Working_Limit_Command(void);  
  void Stop_Work_Command(void);
};

/*Create command analysis project*/
extern Command_Analysis LoRa_Command_Analysis;

extern bool gAccessNetworkFlag;  
extern bool gStopWorkFlag;       
extern bool gMassCommandFlag;
extern bool gIsHandleMsgFlag;

#endif