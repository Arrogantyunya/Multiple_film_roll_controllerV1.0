#ifndef _Motor_H
#define _Motor_H

#include "fun_periph.h"

/*两路电机电压采集引脚*/
#define AVIN_CH1_PIN          PA4
#define AVIN_CH2_PIN          PA5
/*电机电流采集引脚*/
#define AIIN_PIN              PA6
/*检测手动开棚引脚和手动关棚引脚*/
#define DEC_MANUAL_UP_PIN     PB15
#define DEC_MANUAL_DOWN_PIN   PC6
/*电机正反转供电使能引脚*/
#define MOTOR_A_PIN           PA7
#define MOTOR_B_PIN           PC4
/*使能/失能手动卷膜引脚*/
#define MANUAL_ROLL_PIN       PB0

/*电机正反转使能宏*/
#define MOTOR_A_ON            (digitalWrite(MOTOR_A_PIN, HIGH))
#define MOTOR_A_OFF           (digitalWrite(MOTOR_A_PIN, LOW))
#define MOTOR_B_ON            (digitalWrite(MOTOR_B_PIN, HIGH))
#define MOTOR_B_OFF           (digitalWrite(MOTOR_B_PIN, LOW))

/*使能/失能手动卷膜*/
#define MANUAL_ROLL_ON        (digitalWrite(MANUAL_ROLL_PIN, HIGH))
#define MANUAL_ROLL_OFF       (digitalWrite(MANUAL_ROLL_PIN, LOW))

/*卷膜超时时间*/
#define ROLL_OVERTIME         600 //(s)

/*ADC分辨率*/
#define V_RESOLUTION          (3.3 / 4096 * 1000)

#define DETECT_START_ROLL_TIME          3

#define CURRENT_COLLECTION_FREQ         4

#define REALTIME_OPEN_FREQ              5

#define MAX_OPENING_VALUE               480

#define OPENING_THRESHOLD               10

/*
  *当读取的电压压差为负数，说明电机往是开棚方向
  *当读取的电压压差的绝对值和保存的电压值相减
  *如果是负值（NEG），说明当前卷膜电压小于重置行程
  *时候的电压，要增加卷膜时间。（卷膜速度变慢）
  *如果是正值（POS），说明当前卷膜电压大于重置行程
  *时候的电压，要减少卷膜时间。（卷膜速度变快）
 */
#define NEG_ADJUST_ROLL_UP_TIME_MULTIPLE    1.2 //速度慢，往上卷，增加时间倍数基数大点。
#define POS_ADJUST_ROLL_UP_TIME_MULTIPLE    0.9 //速度快，往上卷，增加时间倍数基数小点（因为是相减，所以大了，就提早卷完，有误差）

#define NEG_ADJUST_ROLL_DOWN_TIME_MULTIPLE  1.1
#define POS_ADJUST_ROLL_DOWN_TIME_MULTIPLE  0.8

/*电机方向控制*/
enum Motor_forward{
    A, B, Stop
};

enum Limit_Detection{
  Open = 0, Close, Detection 
};

enum Roll_Action{
  Reset_Roll = 0, Opening_Roll, Force_Open, Force_Close
};

/*卷膜电流状态*/
enum Roll_Current{
  Current_Normal, Detection_OverCurrent, Current_Exception, Current_Uninit
};

extern volatile bool gResetRollWorkingFlag;
extern volatile bool gOpeningWorkingFlag;
extern volatile bool gForceRollWorkingFlag;
extern volatile unsigned int gRollingTime;   
extern volatile bool gRollingTimeVarFlag;

extern volatile bool gManualUpDetectFlag;  
extern volatile bool gManualDownDetectFlag;  
extern volatile bool gManualKeyExceptionFlag;

class Motor_Operations{
public:
  void Motor_GPIO_Config(void);

  void Direction_Selection(Motor_forward direction);

  void Detect_Manual_Rolling(void);

  void Adjust_deviation(void);

  void Adjust_Opening(void);

  bool Force_Open_or_Close(unsigned char opening_value);
  bool Reset_Motor_Route(void);
  bool Motor_Coiling(void);
  void Finish_Rolling(void);
  bool Force_Stop_Work(Roll_Action act, unsigned char realtime_opening);

  unsigned int Current_Detection(void);
  int Voltage_Detection(void);

  bool Trace_Opening(void);

private:

  bool Detect_Motor_Limit(unsigned char *current_opening, Limit_Detection dec, Roll_Action act, unsigned char roll_opening, unsigned int real_roll_time);
  bool Detect_Motor_Overtime(Limit_Detection act);

  bool Verify_Reset_OK(unsigned char *current_opening, bool first_A_direction_flag, bool is_first_stage);

  void Dynamic_Adjust_Roll_Time(unsigned int roll_time_temp, unsigned int *roll_time);

  Roll_Current Motor_Current_Init(float *threshold, unsigned int *saved_current, Limit_Detection act);
  Roll_Current Detect_Motor_OverCurrent(float threshold, unsigned int saved_current, unsigned char status);
  bool Motor_Current_Status(float threshold, unsigned int saved_current, unsigned char status);

  void Calculate_Voltage(unsigned int *voltage_collect_num, unsigned long *voltage_value, int *voltage_value_temp, unsigned int *voltage_calib);
  bool Calculate_and_Save_Voltage(unsigned int *voltage_collect_num, unsigned long *voltage_value, int *voltage_value_temp, unsigned int *voltage_calib);

  void Collect_Current(unsigned int *current_collect_num, unsigned int *current_value, unsigned int *current_value_temp, unsigned int *current_calib);
  bool Calculate_and_Save_Current(unsigned int *current_collect_num, unsigned int *current_value, unsigned int *current_value_temp, unsigned int *current_calib, Limit_Detection act);
};

void Manual_Up_Change_Interrupt(void);
void Manual_Down_Change_Interrupt(void);

extern Motor_Operations Motor_Operation;

extern volatile bool gAdjustOpeningFlag;

#endif
