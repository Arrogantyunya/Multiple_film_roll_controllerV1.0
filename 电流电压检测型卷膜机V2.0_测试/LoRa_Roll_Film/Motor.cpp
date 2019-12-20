/************************************************************************************
 *
 * 代码与注释：卢科青
 * 日期：2019/4/18
 * 该文件是电机运动的核心，主要功能有测量卷膜杆长度、根据开度指令开关到指定的开度值、卷膜突发
 * 断电恢复开度、手动卷膜检测机制等。这些功能都是基于机械和硬件上的行程开关检测，霍尔脉冲计算等。
 *
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "Motor.h"
#include <libmaple/iwdg.h>
#include "Memory.h"
#include <libmaple/nvic.h>
#include "receipt.h"
#include "Command_Analysis.h"
#include "Private_Timer.h"
#include "public.h"

/*Create Motor operations object*/
Motor_Operations Motor_Operation;

volatile uint8_t gCurrentOpening = 0;           //当前开度值
volatile uint8_t gLastOpening = 0;              //上一次开度值

volatile bool gNeedResetRollFlag = false;       //开度卷膜出现误差，需要重置行程

volatile bool gResetRollWorkingFlag = false;    //正在重置行程标志位
volatile bool gOpeningWorkingFlag = false;      //正在开度卷膜标志位
volatile bool gForceRollWorkingFlag = false;    //正在强制卷膜标志位

volatile unsigned int gRollingTime = 0;         //实时卷膜时间
volatile bool gRollingTimeVarFlag = false;

volatile unsigned char gCurrentOverNum = 0;   //电流超阈值计数次数

volatile unsigned int gLowCurrentTime = 0;    //电流过低计数时间

volatile unsigned int gVerifyRollOK_LowCurrentTime = 0;  //验证卷膜完成低电流计数时间

volatile bool gDetectMotorLimitFlag = false;  //是否允许检测电机限位标志位
volatile bool gDetectMotorOverTimeFlag = false;    //是否允许检测电机超时标志位

volatile bool gManualUpDetectFlag = false;    //检测是否有手动开棚行为
volatile bool gManualDownDetectFlag = false;  //检测是否有手动关棚行为
volatile bool gManualKeyExceptionFlag = false;  //手动卷膜按键电路异常
volatile unsigned int gManualLowCurrentNum = 0; //手动卷膜低电压检测计数

volatile bool gStartTraceOpeningFlag = false; //初始化手动卷膜各参数设置标志位
volatile bool gIsTraceUpOpeningFlag = false;  //开度追踪，判断目前的手动开度是开棚方向还是关棚方向
volatile bool gTraceOpeningOKFlag = false;    //检测到手动卷膜完成标志位
volatile unsigned int gVerifyTraceOpeningNum = 0;    //验证是否真的手动卷膜完成，举例：当前开度0%，继续关棚卷膜，实际上电机没动，开度还是0%，验证失败。

/*
 *是否需要矫正开度。有时要求卷膜到某个不为0和100的开度，但卷膜机却跑到了0或者100，说明开度有误差了
 *这个时候就需要置位该位，然后重新从0或者100卷到需要的开度
 */
volatile bool gAdjustOpeningFlag = false;

void Manual_Down_Change_Interrupt(void);
void Manual_Up_Change_Interrupt(void);

/*
 @brief   : 配置电机相关引脚
 @param   : 无
 @return  : 无
 */
void Motor_Operations::Motor_GPIO_Config(void)
{
	pinMode(MOTOR_A_PIN, OUTPUT);
	pinMode(MOTOR_B_PIN, OUTPUT);

	pinMode(MANUAL_ROLL_PIN, OUTPUT);

	pinMode(AIIN_PIN, INPUT_ANALOG);
	pinMode(AVIN_CH1_PIN, INPUT_ANALOG);
	pinMode(AVIN_CH2_PIN, INPUT_ANALOG);

	pinMode(DEC_MANUAL_UP_PIN, INPUT);
	pinMode(DEC_MANUAL_DOWN_PIN, INPUT);

	MANUAL_ROLL_ON;

	attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
	attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);

	/*
	  *虽然打开了双边沿触发的手动卷膜判断。但是假如在设备上电的时候按键就已经按下了，是没办法进入到边沿检测的。
	  *就会认为现在没有手动卷膜，但其实用户已经在手动卷膜了。虽然这种事情发生的概率很小，但本人有着就算全宇宙毁灭
	  *也能正常卷膜的思想，在电机上电的时候，检测这两个按键引脚的状态来判断是否正在手动卷膜。
	 */
	digitalRead(DEC_MANUAL_UP_PIN) == HIGH ? gManualUpDetectFlag = true : gManualUpDetectFlag = false;
	digitalRead(DEC_MANUAL_DOWN_PIN) == HIGH ? gManualDownDetectFlag = true : gManualDownDetectFlag = false;
}

/*
 @brief   : 选择电机转向或停止
 @param   : 电机方向
 @return  : 无
 */
void Motor_Operations::Direction_Selection(Motor_forward direction)
{
	switch (direction)
	{
	case A: MOTOR_A_OFF;  MOTOR_B_ON;   break;  //电机开棚
	case B: MOTOR_A_ON;   MOTOR_B_OFF;  break;  //电机关棚
	case Stop: MOTOR_A_ON;   MOTOR_B_ON;   break;
	default: MOTOR_A_ON;   MOTOR_B_ON;   break;
	}
}

/*
 @brief   : 检测电机是否到达指定开度限位,只有这几种情况触发该函数---》
			1.当电机运动到下限位（0%）或上限位（100%）时，电机硬件限位卡停，检测到电流为0，说明到达了上限位或下限位
			2.举例：当电机运动的开度是80%时，但却触碰到了上限位，说明开度有偏差，需要校正开度。
 @param   : 1.当前开度
			2.当前卷膜方向（开棚或关棚）
			3.当前卷膜方式（重置行程，开度卷膜，强制卷膜）
			4.总卷膜时间
 @return  : true or false
*/
bool Motor_Operations::Detect_Motor_Limit(unsigned char *current_opening, Limit_Detection dec, Roll_Action act, unsigned char roll_opening, unsigned int real_roll_time)
{
	if (gManualUpDetectFlag || gManualDownDetectFlag)
		return false;

	if (gRollingTime < DETECT_START_ROLL_TIME) return false;

	gDetectMotorOverTimeFlag = true;  //允许接下来的检测电机超时判断
	/*电机刚运动的前三秒，不检测*/

	/*如果电流小于100mA*/
	if (Current_Detection() <= 100)
	{
		if (gLowCurrentTime == 0)
			gLowCurrentTime = gRollingTime;

		if (gRollingTime >= (gLowCurrentTime + 3))  //连续检测到电流小于100mA 3秒
		{
			gLowCurrentTime = 0;
			switch (dec)
			{
				/*卷膜方向是开棚的情况下*/
			case Open:  Serial.println("Reach Up limit到达上限位... <Detect_Motor_Limit>");
				if (act == Reset_Roll)  //如果当前是重置行程
				{
					*current_opening = 100;
				}
				// else  //如果当前是开度卷膜或强制卷膜,说明需要校正开度
				// {
				//   if (*current_opening < 100)
				//   {
				//     Serial.println("There is an error in the opening film, and the trip needs to be reset <Detect_Motor_Limit>");
				//     gNeedResetRollFlag = true;
				//     return false;
				//     break;
				//   }
				// }

				//if (*current_opening < 100)
				gAdjustOpeningFlag = true;

				return true; break;

				/*卷膜方向是关棚的情况下*/
			case Close:  Serial.println("Reach Down limit到达下限位... <Detect_Motor_Limit>");
				//if (act == Reset_Roll)
				//{
				*current_opening = 0;
				//}
				// else
				// {
				//   if (*current_opening > 0)
				//   {
				//     Serial.println("There is an error in the opening film, and the trip needs to be reset <Detect_Motor_Limit>");
				//     gNeedResetRollFlag = true;
				//     return false;

				//     gAdjustOpeningFlag = true;

				//     break;
				//   }
				// }

				//if (*current_opening > 0)
				gAdjustOpeningFlag = true;

				return true; break;

			default: Serial.println("Direction ERROR方向错误!!!"); return false; break;
			}
		}
		else
			return false;
	}
	else
	{
		gLowCurrentTime = gRollingTime;
		return false;
	}
}

/*
 @brief   : 卷膜超时处理。如果在卷膜过程中电流小于正常值，说明异常；如果卷膜时间超过最大时间阈值，也是异常
 @para    : 当前动作
 @return  : true or false
 */
bool Motor_Operations::Detect_Motor_Overtime(Limit_Detection act)
{
	bool LowCurrentFlag = false, RollOverTimeFlag = false;

	if (!gDetectMotorOverTimeFlag)
		return false;
	else
		gDetectMotorOverTimeFlag = false;

	/*如果电机正在运行，同时电流又小于100mA，视为电机异常，关闭电机。*/
	if (gResetRollWorkingFlag || gOpeningWorkingFlag || gForceRollWorkingFlag)
	{
		if (gNeedResetRollFlag)
		{
			gNeedResetRollFlag = false;
			LowCurrentFlag = true;
		}
	}
	//如果电机卷膜时间超过了规定的最大卷膜时间,视为限位异常，或者保存的时间异常
	if (gRollingTime >= ROLL_OVERTIME)
		RollOverTimeFlag = true;

	if (LowCurrentFlag == true || RollOverTimeFlag == true)
	{
		Finish_Rolling();
		LED_RUNNING;

		if (!Roll_Operation.Clear_All_Opening_Value())
		{
			Serial.println("Clear all opening value ERROR !!!<Detect_Motor_Overtime>");
			Serial.println("清除所有开度值错误!!!<Detect_Motor_Overtime>");
			Set_Motor_Status(STORE_EXCEPTION);
			Message_Receipt.Working_Parameter_Receipt(false, 2);
			return true;
		}
		else
		{
			Serial.println("Clear all opening value SUCCESS...<Detect_Motor_Overtime>");
			Serial.println("清除所有开度值成功...<Detect_Motor_Overtime>");
		}

		if (!Roll_Operation.Clear_Route_Save_Flag())
		{
			Serial.println("Clear route flag failed清除路径失败 !!!<Detect_Motor_Overtime>");
			Serial.println(" !!!<Detect_Motor_Overtime>");
			Set_Motor_Status(STORE_EXCEPTION);
			Message_Receipt.Working_Parameter_Receipt(false, 2);
			return true;
		}
		else
			Serial.println("Clear route flag success清除路径成功...<Detect_Motor_Overtime>");

		if (LowCurrentFlag == true)
		{
			Serial.println("Motor exception电机异常 !!!<Detect_Motor_Overtime>");
			Set_Motor_Status(MOTOR_EXCEPTION);
		}
		else if (RollOverTimeFlag == true)
		{
			if (act == Open)
			{
				Serial.println("Motor high limit exception电机上限位异常 !!!<Detect_Motor_Overtime>");
				Set_Motor_Status(HIGH_POSITION_LIMIT_EXCEPTION);
			}
			else if (act == Close)
			{
				Serial.println("Motor low limit exception电机下限位异常 !!!<Detect_Motor_Overtime>");
				Set_Motor_Status(LOW_POSITION_LIMIT_EXCEPTION);
			}
		}
		Message_Receipt.Working_Parameter_Receipt(false, 2);
		return true;
	}
	return false;
}

/*
 @brief   : 初始化电流阈值检测机制，获取阈值电流值，采样电流值
 @para    : 电流阈值，采样电流值，电机当前动作
 @return  : 初始化状态
 */
Roll_Current Motor_Operations::Motor_Current_Init(float *threshold, unsigned int *savedcurrent, Limit_Detection act)
{
	if (!Roll_Operation.Verify_Current_Flag())  //如果上次没有保存过采样电流，退出电流初始化
		return Current_Uninit;

	if (!Roll_Operation.Current_Self_Check())  //如果保存的电流值自检失败，说明EP储存出问题
	{
		Roll_Operation.Clear_Current_Flag();
		return Current_Exception;
	}

	unsigned int CurrentValue;

	*threshold = Roll_Operation.Read_Roll_High_Current_Limit_Value() / 10.0;  //获取电流阈值（倍数）

	if (act == Open) //如果是开棚
	{
		Roll_Operation.Read_Roll_Up_Current(&CurrentValue); //获取开棚采集电流
		*threshold *= CurrentValue; //阈值倍数 * 采集电流 = 最大阈值电流
	}
	else if (act == Close)  //如果是关棚
	{
		Roll_Operation.Read_Roll_Down_Current(&CurrentValue); //获取关棚采集电流
		*threshold *= CurrentValue;
	}
	*savedcurrent = CurrentValue;

	return Current_Normal;
}

/*
 @brief   : 卷膜电流阈值检测。如果卷膜电流超过设置的百分比阈值，停止卷膜。
			Film current threshold detection.If the film current exceeds
			the set percentage threshold, stop the film.
 @para    : act ---> Open or Close.
 @return  : current status.
 */
Roll_Current Motor_Operations::Detect_Motor_OverCurrent(float threshold, unsigned int saved_current, unsigned char status)
{
	if (status == Current_Uninit) return Current_Uninit;
	if (status == Current_Exception) return Current_Exception;

	unsigned int CurrentCollectValue;

	CurrentCollectValue = Current_Detection();

	if (CurrentCollectValue > (saved_current + threshold))
	{
		CurrentCollectValue = Current_Detection();
		if (CurrentCollectValue > (saved_current + threshold))
		{
			gCurrentOverNum++;
			// Serial.print("+");
			// Serial.println(gCurrentOverNum);

			if (gCurrentOverNum >= 100)
			{
				Serial.println("Roll film current overcurrent卷膜电流超出 !!!");
				gCurrentOverNum = 0;
				return Detection_OverCurrent;
			}
			else
				return Current_Normal;
		}
		else
		{
			gCurrentOverNum = 0;
			return Current_Normal;
		}
	}
	else
		return Current_Normal;
}

/*
 @brief   : 判断卷膜电流状态是否过流或正常。
 @para    : act ---> Open or Close
 @return  : true or false
 */
bool Motor_Operations::Motor_Current_Status(float threshold, unsigned int saved_current, unsigned char status)
{
	if (gManualUpDetectFlag || gManualDownDetectFlag)
		return false;

	if (gRollingTime <= 3) return false;

	if (gResetRollWorkingFlag || gOpeningWorkingFlag || gForceRollWorkingFlag)
	{
		unsigned char Current_Status = Detect_Motor_OverCurrent(threshold, saved_current, status);

		switch (Current_Status)
		{
		case Current_Uninit: return false; break;
		case Current_Normal: return false; break;
		case Detection_OverCurrent:
			Finish_Rolling();
			Set_Motor_Status(MOTOR_CURRENT_EXCEPTION);
			Message_Receipt.Working_Parameter_Receipt(false, 2);
			LED_RUNNING;
			gRollingTime = 0;
			return true;  break;

		case Current_Exception:
			if (gMotorStatus != STORE_EXCEPTION) {
				Serial.println("Current self-check ERROR当前自检异常 !!! <Motor_Current_Status>");
				Set_Motor_Status(STORE_EXCEPTION);
				Message_Receipt.Working_Parameter_Receipt(false, 2);
			}
			return false; break;
		}
	}
	else
	{
		return false;
	}
}

/*
 @brief   : 手动卷膜检测。如果检测到手动卷膜，当恢复自动卷膜时，需要重置行程
			Manual film detection. If detect manual filmg is detected,
			the motor needs to be reset when automatic film is resumed.
 @para    : None
 @return  : None
 */
void Motor_Operations::Detect_Manual_Rolling(void)
{
	iwdg_feed();
	if (!gManualKeyExceptionFlag)
	{
		if (gManualUpDetectFlag || gManualDownDetectFlag)
		{
			if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)
				gManualUpDetectFlag = false;
			if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW)
				gManualDownDetectFlag = false;
			/*
			  *如果在手动卷膜期间，电流连续3分钟都小于100mA，
			  *说明是触碰开关电路本身故障，并不是人为卷膜，
			  *发送手动按键故障。禁止自动卷膜（如果坏的方向和自动卷膜方向
			  *相反，那么后果很严重），等待维修人员维修或更换。
			*/
			if (Current_Detection() < 100)
			{
				gManualLowCurrentNum++;
				//Serial.println(gManualLowCurrentNum);
				MyDelayMs(100);
			}
			if (gManualLowCurrentNum >= 18000)
			{
				detachInterrupt(DEC_MANUAL_UP_PIN);
				detachInterrupt(DEC_MANUAL_DOWN_PIN);
				gManualLowCurrentNum = 0;
				gManualKeyExceptionFlag = true;
				Serial.println("Manual key exception!");
				/**/
				if (!Roll_Operation.Clear_All_Opening_Value())
				{
					Serial.println("Clear all opening value ERROR清除所有开度值错误 !!! <Force_Stop_Work>");
					Set_Motor_Status(STORE_EXCEPTION);
					Message_Receipt.Working_Parameter_Receipt(false, 2);
				}

				Set_Motor_Status(MANUAL_KEY_EXCEPTION);
				Message_Receipt.Working_Parameter_Receipt(false, 2);
			}
		}
		else
			gManualLowCurrentNum = 0;
	}
}

/*
 @brief   : 非正常断电后，恢复先前目标开度
			Recovery of previous opening correction after abnormal power down of motor in operation
 @para    : None
 @return  : None
 */
void Motor_Operations::Adjust_deviation(void)
{
	iwdg_feed();
	unsigned char Current_Opening_Temp = Roll_Operation.Read_Current_Opening_Value();
	Serial.print("Current_Opening当前开度 <Adjust_deviation>: ");  Serial.println(Current_Opening_Temp);

	unsigned char RealTime_Opeing_Temp = Roll_Operation.Read_RealTime_Opening_Value();
	Serial.print("RealTime_Opening当前开度 <Adjust_deviation>: "); Serial.println(RealTime_Opeing_Temp);

	unsigned char Adjust_Flag = false;

	if (RealTime_Opeing_Temp % 10 != 0) {
		if (RealTime_Opeing_Temp > Current_Opening_Temp) {
			if (RealTime_Opeing_Temp - Current_Opening_Temp <= 5)
				Adjust_Flag = true;

		}
		else {
			if (Current_Opening_Temp - RealTime_Opeing_Temp <= 5) {
				Adjust_Flag = true;
			}
		}
		if (Adjust_Flag == true) {
			Serial.println("compensate opening补偿开度...<Adjust_deviation>");
			RealTime_Opeing_Temp = Current_Opening_Temp;
		}
	}

	if (RealTime_Opeing_Temp > Current_Opening_Temp || RealTime_Opeing_Temp < Current_Opening_Temp) {
		Roll_Operation.Save_Last_Opening_Value(RealTime_Opeing_Temp);
		Roll_Operation.Save_Current_Opening_Value(Current_Opening_Temp);

		detachInterrupt(DEC_MANUAL_DOWN_PIN);
		detachInterrupt(DEC_MANUAL_UP_PIN);
		MANUAL_ROLL_OFF;
		Motor_Coiling();
		attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
		attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
		MANUAL_ROLL_ON;
	}
}

/*
 @brief   : 开度误差校正。当卷膜完成后与目标开度有偏差，重新矫正开度。
			Opening error correction. When there is a deviation between the film and the target opening after
			the completion of the film, re-adjust the opening.
 @para    : None
 @return  : None
 */
void Motor_Operations::Adjust_Opening(void)
{
	if (gAdjustOpeningFlag == true)
	{
		gAdjustOpeningFlag = false;
		Serial.println("Adjust opening调整开度 ! <Adjust_Opening>");

		if (gManualKeyExceptionFlag)
		{
			Serial.println("Manual roll key exception手动滚动键异常 !!! <Opening_Command>");
			return;
		}

		/*如果当前正在手动卷膜。拒绝执行自动卷膜*/
		else if (gManualUpDetectFlag || gManualDownDetectFlag)
		{
			if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)  gManualUpDetectFlag = false;
			if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW) gManualDownDetectFlag = false;
			if (gManualUpDetectFlag || gManualDownDetectFlag)
			{
				Serial.println("Detect manual rolling检测手动卷膜... <Opening_Command>");
				/*
				  *待处理事情
				*/
				return;
			}
		}

		/*失能手动卷膜， 失能检测手动卷膜按键中断*/
		detachInterrupt(DEC_MANUAL_DOWN_PIN);
		detachInterrupt(DEC_MANUAL_UP_PIN);
		MANUAL_ROLL_OFF;

		Motor_Coiling();

		/*自动卷膜完成后，使能手动卷膜，打开检测手动卷膜按键中断*/
		attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
		attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
		MANUAL_ROLL_ON;
	}
}

/*
 @brief   : 结束电机卷膜。关闭中断，停止计时，设置相关标志位
			End the motor roll. Turn off interrupts, stop timing, and set flags.
 @para    : None
 @return  : None
 */
void Motor_Operations::Finish_Rolling(void)
{
	Stop_Roll_Timing();
	Motor_Operation.Direction_Selection(Stop);
	gResetRollWorkingFlag = false;
	gOpeningWorkingFlag = false;
	gForceRollWorkingFlag = false;
	Start_Self_Check_Timing();
}

/*
 @brief   : 强制结束电机卷膜。当从服务器接收到强制停止工作指令时执行该函数
 @para    : Open or Close
			realtime opening.
 @return  : true or false.
 */
bool Motor_Operations::Force_Stop_Work(Roll_Action act, unsigned char realtime_opening = 0)
{
	if (gStopWorkFlag == true)
	{
		gStopWorkFlag = false;
		Finish_Rolling();
		Set_Motor_Status(FORCE_STOP);
		LED_RUNNING;

		/*
		  *原则上，当卷膜机重置行程时，并不知道当前自己所在位置，所以这个时候强制停止
		  *如果下次想开度卷膜，需要再次重置行程。
		 */
		if (act == Reset_Roll)
		{
			Serial.println("Stop Motor reset roll停止电机重置行程...<Force_Stop_Work>");
			// if (!Roll_Operation.Clear_All_Opening_Value())
			// {
			//   Serial.println("Clear all opening value ERROR !!! <Force_Stop_Work>");
			//   Set_Motor_Status(STORE_EXCEPTION);
			//   Message_Receipt.Working_Parameter_Receipt(false, 2);          
			// }
		}
		else if (act == Opening_Roll)
		{
			Serial.println("Stop Motor opening roll停止电机开棚...<Force_Stop_Work>");
			Roll_Operation.Save_Last_Opening_Value(realtime_opening);  //被强制停止后，应保存当前的卷膜实时开度值
		}
		/*
		  *在强制开棚和关棚运动中，会无视当前卷到什么位置，直至卷到上限位或下限位。
		  *所以从逻辑上来讲，这时被强制停止了，系统并不知道当前处于什么位置，需要再次重置行程。
		*/
		else if (act == Force_Open || act == Force_Close)
		{
			Serial.println("Stop motor force close停止电机全开全关...<Force_Stop_Work>");
			// if (!Roll_Operation.Clear_All_Opening_Value())
			// {
			//   Serial.println("Clear all opening value ERROR !!! <Force_Stop_Work>");
			//   Set_Motor_Status(STORE_EXCEPTION);
			//   Message_Receipt.Working_Parameter_Receipt(false, 2);
			// }
			if (!Roll_Operation.Clear_Route_Save_Flag())
			{
				Serial.println("Clear route save flag ERROR !!! <Force_Stop_Work>");
				Set_Motor_Status(STORE_EXCEPTION);
				Message_Receipt.Working_Parameter_Receipt(false, 2);
			}
		}
		return true;
	}
	return false;
}

bool Motor_Operations::Verify_Reset_OK(unsigned char *current_opening, bool first_A_direction_flag, bool is_first_stage)
{
	unsigned char LowCurrentNum = 0;
	Finish_Rolling();
	iwdg_feed();
	Stop_Roll_Timing();
	gResetRollWorkingFlag = true;

	gVerifyRollOK_LowCurrentTime = 0;
	gLowCurrentTime = 0;

	MyDelayMs(1000);
	iwdg_feed();

	if (is_first_stage)
	{
		if (first_A_direction_flag)
			Motor_Operation.Direction_Selection(B);
		else
			Motor_Operation.Direction_Selection(A);
	}
	else
	{
		if (first_A_direction_flag)
			Motor_Operation.Direction_Selection(A);
		else
			Motor_Operation.Direction_Selection(B);
	}

	MyDelayMs(1000);  //防止下面的电流阈值判断误判
	iwdg_feed();
	Stop_Self_Check_Timing();
	Start_Roll_Timing();

	do {
		iwdg_feed();

		if ((Current_Detection() < 100))
		{
			if (gVerifyRollOK_LowCurrentTime == 0)
				gVerifyRollOK_LowCurrentTime = gRollingTime;

			if (gRollingTime >= gVerifyRollOK_LowCurrentTime + 3)
			{
				gVerifyRollOK_LowCurrentTime = 0;
				Serial.println("The current is less than the threshold! <Verify_Reset_OK>");
				return false;
			}
		}
		else
		{
			gVerifyRollOK_LowCurrentTime = gRollingTime;
			if (gRollingTime >= 8)
				break;
		}
	} while (1);

	Finish_Rolling();
	MyDelayMs(1000);
	iwdg_feed();

	if (is_first_stage)
	{
		if (first_A_direction_flag)
			Motor_Operation.Direction_Selection(A);
		else
			Motor_Operation.Direction_Selection(B);
	}
	else
	{
		if (first_A_direction_flag)
			Motor_Operation.Direction_Selection(B);
		else
			Motor_Operation.Direction_Selection(A);
	}

	iwdg_feed();
	Stop_Self_Check_Timing();
	Start_Roll_Timing();
	MyDelayMs(1000);

	do {
		iwdg_feed();
		if (is_first_stage)
		{
			if (first_A_direction_flag)
			{
				if (Detect_Motor_Limit(current_opening, Open, Reset_Roll, 0, 0)) break;
				if (Detect_Motor_Overtime(Open)) return false;
			}
			else
			{
				if (Detect_Motor_Limit(current_opening, Close, Reset_Roll, 0, 0)) break;
				if (Detect_Motor_Overtime(Close)) return false;
			}
		}
		else
		{
			if (first_A_direction_flag)
			{
				if (Detect_Motor_Limit(current_opening, Close, Reset_Roll, 0, 0)) break;
				if (Detect_Motor_Overtime(Close)) return false;
			}
			else
			{
				if (Detect_Motor_Limit(current_opening, Open, Reset_Roll, 0, 0)) break;
				if (Detect_Motor_Overtime(Open)) return false;
			}
		}

		if (gRollingTime >= 10)
			break;
	} while (1);

	return true;
}

/*
 @brief   : 开度追踪机制。在设备空闲的时候，同时已经重置行程完毕，当检测到用户手动卷膜，实时追踪卷膜开度，同步到自动卷膜。
 @para    : 无
 @return  : true or false
 */
bool Motor_Operations::Trace_Opening(void)
{
	/*如果检测到手动卷膜，准备追踪开度,该条件语句中执行的作用是初始化追踪开度的相关操作*/
	if (gManualUpDetectFlag || gManualDownDetectFlag)
	{
		iwdg_feed();
		if (Roll_Operation.Read_Route_Save_Flag())
		{
			/*开始打开卷膜计时器*/
			if (!gStartTraceOpeningFlag && !gTraceOpeningOKFlag)
			{
				Start_Roll_Timing();
				gStartTraceOpeningFlag = true;

				if (gManualUpDetectFlag)
					gIsTraceUpOpeningFlag = true;

				else if (gManualDownDetectFlag)
					gIsTraceUpOpeningFlag = false;
			}
		}
	}

	if (gStartTraceOpeningFlag && !gTraceOpeningOKFlag)
	{
		/*
		  *如果电流小于100mA，说明手动卷膜结束
		  *根据卷膜的方向是开棚还是关棚来计算开度走向。
		 */
		gVerifyTraceOpeningNum++;
		if (Current_Detection() < 100)
		{
			MyDelayMs(10);  //防电流抖动
			if (Current_Detection() < 100)
			{
				gStartTraceOpeningFlag = false;
				//Stop_Roll_Timing();

				if (gVerifyTraceOpeningNum > 10)
				{
					unsigned int TotolRollTime = Roll_Operation.Read_Rolling_Time();
					unsigned char ManualOpening = gRollingTime * 100 / TotolRollTime;

					if (gIsTraceUpOpeningFlag)
					{
						ManualOpening += Roll_Operation.Read_Last_Opening_Value();
						if (ManualOpening > 100) ManualOpening = 100;
						Serial.print("Up roll, manual opening is上卷手动开度为:"); Serial.println(ManualOpening);
					}
					else
					{
						ManualOpening = Roll_Operation.Read_Last_Opening_Value() - ManualOpening;
						if (ManualOpening > 100) ManualOpening = 0;
						Serial.print("Down roll, manual opening is下卷手动开度为:"); Serial.println(ManualOpening);
					}

					Roll_Operation.Save_Last_Opening_Value(ManualOpening);
					Roll_Operation.Save_Current_Opening_Value(ManualOpening);
					Roll_Operation.Save_RealTime_Opening_Value(ManualOpening);
					Set_Motor_Status(MANUAL_ROLL_OK);
					Message_Receipt.Working_Parameter_Receipt(false, 1);
				}
				gVerifyTraceOpeningNum = 0;
				gTraceOpeningOKFlag = true;
			}
		}
	}
}

/*
 @brief   : 强制卷膜机不顾行程、不顾超时检测，全开或全关棚膜。慎用！
			Force roll film regardless of route, regardless of overtime detection, full open or closed shed film. Use with caution !
 @para    : opening_value ---> 0xF0(full close shed film); 0xF1(full open shed film)
 @return  : true or false
 */
bool Motor_Operations::Force_Open_or_Close(unsigned char opening_value)
{
	unsigned char RecentOpening = 0;
	bool OpenFlag = false, CloseFlag = false;

	bool IsFirst_A_Direction;

	gForceRollWorkingFlag = true; //正在强制卷膜标志位置位

	float CurrentThreshold;
	unsigned char CurrentStatus;
	unsigned int SavedCurrent;

	gStopWorkFlag = false; //清除强制停止标志位

	Set_Motor_Status(ROLLING);
	Message_Receipt.Working_Parameter_Receipt(true, 2);

	LED_FORCE_OPENING;

	/*0xF0 : 强制关棚； 0xF1 : 强制开棚*/
	if (opening_value == 0xF0)
	{
		Serial.println("Force Close全关...");
		CloseFlag = true;
		CurrentStatus = Motor_Current_Init(&CurrentThreshold, &SavedCurrent, Close);
		Motor_Operation.Direction_Selection(B);
		IsFirst_A_Direction = false;
	}
	else if (opening_value == 0xF1)
	{
		Serial.println("Force Open全开...");
		OpenFlag = true;
		CurrentStatus = Motor_Current_Init(&CurrentThreshold, &SavedCurrent, Open);
		Motor_Operation.Direction_Selection(A);
		IsFirst_A_Direction = true;
	}

	Stop_Self_Check_Timing();
	Start_Roll_Timing();

	do {
		iwdg_feed();

		if (OpenFlag)
		{
			if (Detect_Motor_Limit(&RecentOpening, Open, Reset_Roll, 0, 0)) break;
			if (Force_Stop_Work(Force_Open) == true) return true;
			if (Motor_Current_Status(CurrentThreshold, SavedCurrent, CurrentStatus) == true) return false;
		}
		else if (CloseFlag)
		{
			if (Detect_Motor_Limit(&RecentOpening, Close, Reset_Roll, 0, 0)) break;
			if (Force_Stop_Work(Force_Close) == true) return true;
			if (Motor_Current_Status(CurrentThreshold, SavedCurrent, CurrentStatus) == true) return false;
		}
		LoRa_Command_Analysis.Receive_LoRa_Cmd();

	} while (1);

	if (Verify_Reset_OK(&RecentOpening, IsFirst_A_Direction, true))
	{
		Serial.println("Verify force roll OK...");
		Finish_Rolling();
	}
	else
	{
		Serial.println("Verify force roll failed!!!");
		//为了触发 Detect_Motor_Overtime函数，让这个函数检测出电流过低，然后报电机异常
		gForceRollWorkingFlag = true;
		//为了触发 Detect_Motor_Overtime函数，让这个函数检测出电流过低，然后报电机异常

		do {
			iwdg_feed();
			gDetectMotorOverTimeFlag = true;
			gNeedResetRollFlag = true;
			if (CloseFlag)
			{
				if (Detect_Motor_Overtime(Close))
					return false;
			}
			else if (OpenFlag)
			{
				if (Detect_Motor_Overtime(Open))
					return false;
			}
		} while (1);
	}

	Roll_Operation.Save_Last_Opening_Value(RecentOpening); // Save current opening to the current path
	Roll_Operation.Save_Current_Opening_Value(RecentOpening);
	Roll_Operation.Save_RealTime_Opening_Value(RecentOpening);
	Set_Motor_Status(ROLL_OK);
	Message_Receipt.Working_Parameter_Receipt(true, 2);
	gForceRollWorkingFlag = false;
	LED_RUNNING;
	return true;
}

/*
 @brief   : 该函数是该文件核心功能，用来重置卷膜行程。
			This function is the core function of the file and is used to reset roll.
 @para    : None
 @return  : true or false
 */
bool Motor_Operations::Reset_Motor_Route(void)
{
	unsigned char RecentOpening = 0;

	unsigned long VoltageValue = 0;
	int VoltageValueTemp;
	unsigned int VoltageCollectNum = 0;
	unsigned int VoltageCalibration = 0;

	unsigned int CurrentValue = 0, CurrentValueTemp;
	unsigned int CurrentCollectNum = 0;
	unsigned int CurrentCalibration = 0;
	float CurrentThreshold;
	unsigned char CurrentStatus;
	unsigned int SavedCurrent;

	bool ResetFirstDIRFlag = false;
	bool IsFirst_A_Direction;

	Serial.println("Begin to reset motor开始复位电机...<Reset_Motor_Route>");
	Set_Motor_Status(RESET_ROLLING);
	Message_Receipt.Working_Parameter_Receipt(true, 2);
	iwdg_feed();
	gStopWorkFlag = false; //清除强制停止标志位
	gLowCurrentTime = 0;

	/*
	  *每次决定重置行程，都将会清除上一次重置行程标志位
	  *也就是假如本次重置行程失败，那么执行开度卷膜，依然必须要重置行程成功。
	  *如果清除重置行程标志位失败，说明EP储存出错，发出储存异常报警，退出重置行程
	*/
	if (!Roll_Operation.Clear_Route_Save_Flag())
	{
		Serial.println("Clear route save flag failed清除路线保存标志失败!!! <Reset_Motor_Route>");
		Set_Motor_Status(STORE_EXCEPTION);
		Message_Receipt.Working_Parameter_Receipt(false, 2);
		return false;
	}

	gResetRollWorkingFlag = true;
	LED_RESET_ROUTE;
	CurrentStatus = Motor_Current_Init(&CurrentThreshold, &SavedCurrent, Open);

	/*
	  *判断上一次卷膜开度在哪。如果在接近全开的位置（90% - 100 %），为了能完全测量到开棚和关棚的各项信息
	  *先关棚，再开棚。（默认先开棚，再关棚）
	*/
	if ((Roll_Operation.Read_Current_Opening_Value()) >= 90 && (Roll_Operation.Read_Current_Opening_Value() <= 100))
		ResetFirstDIRFlag = true;

	if (ResetFirstDIRFlag)
	{
		Motor_Operation.Direction_Selection(B);
		IsFirst_A_Direction = false;
	}
	else
	{
		Motor_Operation.Direction_Selection(A);
		IsFirst_A_Direction = true;
	}

	/*开始计时卷膜时间，暂停自检计时*/
	Start_Roll_Timing();
	Stop_Self_Check_Timing();

	do {
		iwdg_feed();

		if (ResetFirstDIRFlag)
		{
			if (Detect_Motor_Limit(&RecentOpening, Close, Reset_Roll, 0, 0)) break;
			if (Detect_Motor_Overtime(Close)) return false;
			if (Motor_Current_Status(CurrentThreshold, SavedCurrent, CurrentStatus)) return false;
		}
		else
		{
			if (Detect_Motor_Limit(&RecentOpening, Open, Reset_Roll, 0, 0)) break;
			if (Detect_Motor_Overtime(Open)) return false;
			if (Motor_Current_Status(CurrentThreshold, SavedCurrent, CurrentStatus)) return false;
		}
		if (Force_Stop_Work(Reset_Roll)) return true;

		Collect_Current(&CurrentCollectNum, &CurrentValue, &CurrentValueTemp, &CurrentCalibration);

		LoRa_Command_Analysis.Receive_LoRa_Cmd();

	} while (1);

	if (Verify_Reset_OK(&RecentOpening, IsFirst_A_Direction, true))
	{
		Serial.println("Verify reset OK确认重置好了...");
		Finish_Rolling();
		if (ResetFirstDIRFlag)
			Calculate_and_Save_Current(&CurrentCollectNum, &CurrentValue, &CurrentValueTemp, &CurrentCalibration, Close);
		else
			Calculate_and_Save_Current(&CurrentCollectNum, &CurrentValue, &CurrentValueTemp, &CurrentCalibration, Open);
	}
	else
	{
		Serial.println("Verify reset failed确认重置失败!!!");
		//为了触发 Detect_Motor_Overtime函数，让这个函数检测出电流过低，然后报电机异常
		gResetRollWorkingFlag = true;

		do {
			iwdg_feed();
			gDetectMotorOverTimeFlag = true;
			gNeedResetRollFlag = true;

			if (ResetFirstDIRFlag)
			{
				if (Detect_Motor_Overtime(Close))
					return false;
			}
			else
			{
				if (Detect_Motor_Overtime(Open))
					return false;
			}
		} while (1);
	}

	/*-------------------------------------Roll Down--------------------------------------------*/

	gResetRollWorkingFlag = true;
	Serial.println("Prepare Reverse motor电机准备反向... <Reset_Motor_Route>");
	MyDelayMs(1000);
	iwdg_feed();

	CurrentStatus = Motor_Current_Init(&CurrentThreshold, &SavedCurrent, Close);

	if (ResetFirstDIRFlag)
		Motor_Operation.Direction_Selection(A);
	else
		Motor_Operation.Direction_Selection(B);

	Start_Roll_Timing();
	Stop_Self_Check_Timing();

	do {
		iwdg_feed();

		if (ResetFirstDIRFlag) {
			if (Detect_Motor_Limit(&RecentOpening, Open, Reset_Roll, 0, 0)) break;
			if (Detect_Motor_Overtime(Open)) return false;
			if (Motor_Current_Status(CurrentThreshold, SavedCurrent, CurrentStatus)) return false;
		}
		else
		{
			if (Detect_Motor_Limit(&RecentOpening, Close, Reset_Roll, 0, 0)) break;
			if (Detect_Motor_Overtime(Close) == true) return false;
			if (Motor_Current_Status(CurrentThreshold, SavedCurrent, CurrentStatus) == true) return false;
		}

		if (Force_Stop_Work(Reset_Roll)) return true;

		Calculate_Voltage(&VoltageCollectNum, &VoltageValue, &VoltageValueTemp, &VoltageCalibration);
		Collect_Current(&CurrentCollectNum, &CurrentValue, &CurrentValueTemp, &CurrentCalibration);

		LoRa_Command_Analysis.Receive_LoRa_Cmd();

	} while (1);

	//保存测量的行程时间
	if (gRollingTime % 2 != 0) gRollingTime += 1;
	unsigned int SavedRollingTime = gRollingTime;

	if (Verify_Reset_OK(&RecentOpening, IsFirst_A_Direction, false))
	{
		Serial.println("Verify reset OK确认重置完成...");
		Finish_Rolling();
		Calculate_and_Save_Voltage(&VoltageCollectNum, &VoltageValue, &VoltageValueTemp, &VoltageCalibration);

		if (ResetFirstDIRFlag)
			Calculate_and_Save_Current(&CurrentCollectNum, &CurrentValue, &CurrentValueTemp, &CurrentCalibration, Open);
		else
			Calculate_and_Save_Current(&CurrentCollectNum, &CurrentValue, &CurrentValueTemp, &CurrentCalibration, Close);

		if (!Roll_Operation.Save_Rolling_Time(SavedRollingTime))
		{
			Serial.println("Save rolling time ERROR!!! <Reset_Motor_Route>");
			Set_Motor_Status(STORE_EXCEPTION);
			Message_Receipt.Working_Parameter_Receipt(false, 2);
			return false;
		}
		Serial.print("Reset rolling time重置卷膜时间 <Reset_Motor_Route> = ");  Serial.println(SavedRollingTime);
	}
	else
	{
		Serial.println("Verify reset failed确认重置失败!!!");
		//为了触发 Detect_Motor_Overtime函数，让这个函数检测出电流过低，然后报电机异常
		gResetRollWorkingFlag = true;

		do {
			iwdg_feed();
			gDetectMotorOverTimeFlag = true;
			gNeedResetRollFlag = true;

			if (ResetFirstDIRFlag)
			{
				if (Detect_Motor_Overtime(Open))
					return false;
			}
			else
			{
				if (Detect_Motor_Overtime(Close))
					return false;
			}
		} while (1);
	}

	Serial.println("Reset rolling success OK重置卷膜成功...<Reset_Motor_Route>");

	Roll_Operation.Save_Last_Opening_Value(RecentOpening); // Save current opening to the current path
	Roll_Operation.Save_Current_Opening_Value(RecentOpening);
	Roll_Operation.Save_RealTime_Opening_Value(RecentOpening);
	Roll_Operation.Set_Route_Save_Flag();
	Set_Motor_Status(RESET_ROLLOK);
	Message_Receipt.Working_Parameter_Receipt(true, 2);
	LED_RUNNING;
	return true;
}

/*
 @brief   : 该函数是该文件核心功能，用来根据服务器设置的目标开度卷膜
 @para    : 无
 @return  : 无
 */
bool Motor_Operations::Motor_Coiling(void)
{
	unsigned char LastOpening = 0, RecentOpening = 0, OpeningTemp = 0; //上一次开度、本次开度、记录实时开度中间暂存量
	unsigned char RollOpening = 0;  //经过上一次和本次开度的计算，得到实际要卷的开度。

	unsigned int TotalOpeningTime = 0;  //整个卷膜行程总时长
	unsigned int LastRollTime = 0;      //用来防止在一秒内出现多次保存实时状态和电压调节操作
	unsigned int RealRollTime = 0;      //实际需要卷膜开度的时间
	unsigned int RollTimeTemp = 0;      //实际卷膜时间的中间暂存量

	bool OpenFlag = false, CloseFlag = false; //关棚标志位和开棚标志位

	bool IsFirst_A_Direction; //是否第一次卷膜方向是A方向

	/* 上报实时状态间隔时间系数 */
	unsigned char IntervalThreshold = Roll_Operation.Read_Roll_Report_Status_Interval_Value();

	unsigned int IntervalLastTime = 0; //用来防止在一秒内出现多次上报状态操作

	float Current_Threshold;  //电流阈值
	unsigned char Current_Status; //电流状态
	unsigned int Saved_Current; //保存的采集电流值

	unsigned char DyTimeNum = 0;  //实时调整动态电压间隔次数

	iwdg_feed();

	/*如果已经重置行程成功，满足开度卷膜条件*/
	if (Roll_Operation.Read_Route_Save_Flag() == true)
	{
		LastOpening = Roll_Operation.Read_Last_Opening_Value();
		RecentOpening = Roll_Operation.Read_Current_Opening_Value();

		gNeedResetRollFlag = false;

		TotalOpeningTime = Roll_Operation.Read_Rolling_Time(); //得到整个膜杆行程需要的卷膜时间
		/*根据 Read_Rolling_Time() 函数的判断，如果返回的是0xFFFF，说明储存异常*/
		if (TotalOpeningTime == 0xFFFF)
		{
			Serial.println("Saved rolling time is ERROR保存卷膜时间错误 ! <Motor_Coiling>");

			if (!Roll_Operation.Clear_All_Opening_Value())  //清除重置行程标志位，让其下次必须重置行程
			{
				Serial.println("Clear all opening value ERROR清除所有开度值错误 !!! <Force_Stop_Work>");
				Set_Motor_Status(STORE_EXCEPTION);
				Message_Receipt.Working_Parameter_Receipt(false, 2);
			}
			return false;
		}

		Serial.print("Last_opening前次开度为 <Motor_Coiling>:");        Serial.println(LastOpening);
		Serial.print("Recent_opening <Motor_Coiling>:");      Serial.println(RecentOpening);
		Serial.print("Total_Roll_Time总卷膜时间 <Motor_Coiling>:");     Serial.println(TotalOpeningTime);

		iwdg_feed();

		/*边界判断，如果这三个参数的值大于边界范围，视为储存异常*/
		if (LastOpening > 100 || RecentOpening > 100 || TotalOpeningTime > MAX_OPENING_VALUE)
		{
			Serial.println("Opening roll data Error开棚数据错误! <Motor_Coiling>");
			Set_Motor_Status(STORE_EXCEPTION);
			Message_Receipt.Working_Parameter_Receipt(false, 2);

			bool status1 = Roll_Operation.Clear_Route_Save_Flag();
			bool status2 = Roll_Operation.Clear_All_Opening_Value();

			if (status1 == false || status2 == false)
			{
				Serial.println("Clear some parameters ERROR清除一些参数错误 !!! <Motor_Coiling>");
			}
			return false;
		}
		else  //前面的参数初始化一切正常，准备卷膜
		{
			Set_Motor_Status(ROLLING);
			Message_Receipt.Working_Parameter_Receipt(true, 2);
		}

		iwdg_feed();

		gOpeningWorkingFlag = true; //正在卷膜标志位置位
		gStopWorkFlag = false;      //强制停止标志位清零

		gLowCurrentTime = 0;

		/*判断本次卷膜的方向是开棚还是关棚或是什么都不做*/
		if (RecentOpening > LastOpening)
			OpenFlag = true;
		else if (LastOpening > RecentOpening)
			CloseFlag = true;
		else if (RecentOpening == LastOpening)
		{
			Finish_Rolling();
			Set_Motor_Status(ROLL_OK);
			Message_Receipt.Working_Parameter_Receipt(true, 2);
			return true;
		}
		iwdg_feed();

		if (OpenFlag)
		{
			RollOpening = RecentOpening - LastOpening;
			Current_Status = Motor_Current_Init(&Current_Threshold, &Saved_Current, Open);  //电流阈值检测初始化
		}
		else if (CloseFlag)
		{
			RollOpening = LastOpening - RecentOpening;
			Current_Status = Motor_Current_Init(&Current_Threshold, &Saved_Current, Close);
		}

		/*这里先粗糙的计算本次开度需要的卷膜时间，实际上要根据功率卷膜算法来动态计算当前需要的卷膜时间*/
		RealRollTime = RollOpening * 0.01 * TotalOpeningTime + 0.5;
		RollTimeTemp = RealRollTime;
		Serial.print("Voltage original need time电压原始需要时间 <Motor_Coiling> = "); Serial.println(RollTimeTemp);
		iwdg_feed();

		/*使能正转或反转*/
		if (OpenFlag)
		{
			Motor_Operation.Direction_Selection(A);
			IsFirst_A_Direction = true;
		}
		else if (CloseFlag)
		{
			Motor_Operation.Direction_Selection(B);
			IsFirst_A_Direction = false;
		}

		LED_OPENING;  //LED灯开度卷膜状态

		Stop_Self_Check_Timing(); //停止自检计时
		Start_Roll_Timing();  //开始卷膜计时

		do {
			iwdg_feed();

			/*除了0%和100%开度外其他设置的开度卷膜，如果到了该开度值，则停止卷膜，本次开度卷膜完成*/
			if (RecentOpening > 0 && RecentOpening < 100)
			{
				if (gRollingTime >= RealRollTime)
				{
					Serial.println("Rolling reached opening卷膜到达开度...");
					break;
				}
			}

			if (OpenFlag)
			{
				if (Detect_Motor_Limit(&RecentOpening, Open, Opening_Roll, RollOpening, RealRollTime)) break;
				if (Detect_Motor_Overtime(Open)) return false;
				if (Force_Stop_Work(Opening_Roll, OpeningTemp) == true) return true;
				if (Motor_Current_Status(Current_Threshold, Saved_Current, Current_Status) == true) return false;
			}
			else if (CloseFlag)
			{
				if (Detect_Motor_Limit(&RecentOpening, Close, Opening_Roll, RollOpening, RealRollTime)) break;
				if (Detect_Motor_Overtime(Close)) return false;
				if (Force_Stop_Work(Opening_Roll, OpeningTemp) == true) return true;
				if (Motor_Current_Status(Current_Threshold, Saved_Current, Current_Status) == true) return false;
			}

			/*每隔一秒保存一次实时开度*/
			if (gRollingTime > LastRollTime)
			{
				LastRollTime = gRollingTime;

				DyTimeNum++;

				/*实时计算出当前开度值，并保存*/
				if (OpenFlag)
				{
					iwdg_feed();
					OpeningTemp = (gRollingTime / (float)TotalOpeningTime * 100) + LastOpening;
					/*
					 *假如卷膜开度是50%，实际会出现卷膜完成后显示51%，这里过滤这1%
					 */
					if (OpeningTemp % 10 >= 1) OpeningTemp -= 1;
					Roll_Operation.Save_RealTime_Opening_Value(OpeningTemp);
					Roll_Operation.Save_Last_Opening_Value(OpeningTemp);
				}
				else if (CloseFlag)
				{
					iwdg_feed();
					OpeningTemp = LastOpening - (gRollingTime / (float)TotalOpeningTime * 100);
					if (OpeningTemp % 10 == 1) OpeningTemp -= 1;
					Roll_Operation.Save_RealTime_Opening_Value(OpeningTemp);
					Roll_Operation.Save_Last_Opening_Value(OpeningTemp);
				}

				/*每三秒检测实时电压，动态调节所需卷膜时间*/
				if (DyTimeNum >= 3)
				{
					DyTimeNum = 0;
					Dynamic_Adjust_Roll_Time(RollTimeTemp, &RealRollTime);
				}
			}

			/*根据设置的上报状态频率阈值，上报电机实时状态给服务器*/
			if ((gRollingTime % (IntervalThreshold * 10) == 0) && (gRollingTime != IntervalLastTime))
			{
				IntervalLastTime = gRollingTime;
				Message_Receipt.Working_Parameter_Receipt(false, 1);
			}

			LoRa_Command_Analysis.Receive_LoRa_Cmd();

		} while (1);

		if (OpenFlag)
		{
			Serial.print("Opening rolling time <Motor_Coiling>= "); Serial.println(gRollingTime);
			Serial.println("# Open shed finishied...<Motor_Coiling>");
		}
		else if (CloseFlag)
		{
			Serial.print("Close rolling time <Motor_Coiling>= "); Serial.println(gRollingTime);
			Serial.println("Close shed finishied...<Motor_Coiling>");
		}

		if (RecentOpening == 0 || RecentOpening == 100 || gAdjustOpeningFlag == true)
		{
			if (Verify_Reset_OK(&RecentOpening, IsFirst_A_Direction, true))
			{
				Serial.println("Verify opening roll OK确认开始卷膜OK...");
				Finish_Rolling();
			}
			else
			{
				Serial.println("Verify opening roll failed确认开始卷膜失败!!!");
				//为了触发 Detect_Motor_Overtime函数，让这个函数检测出电流过低，然后报电机异常
				gOpeningWorkingFlag = true;

				do {
					iwdg_feed();
					gDetectMotorOverTimeFlag = true;
					gNeedResetRollFlag = true;

					if (CloseFlag)
					{
						if (Detect_Motor_Overtime(Close))
							return false;
					}
					else if (OpenFlag)
					{
						if (Detect_Motor_Overtime(Open))
							return false;
					}
				} while (1);
			}
		}
		else
		{
			Finish_Rolling();
		}

		iwdg_feed();

		Roll_Operation.Save_Last_Opening_Value(RecentOpening);
		Roll_Operation.Save_RealTime_Opening_Value(RecentOpening);
		Set_Motor_Status(ROLL_OK);
		Message_Receipt.Working_Parameter_Receipt(true, 2);
		LED_RUNNING;
		return true;
	}
	else
	{
		Serial.println("# No reset route没有复位的路线...<Motor_Coiling>");
		Serial.println("# You must to Reset route before open shed or close shed必须在打开或关闭棚之前重置路线...<Motor_Coiling>");
	}
	return false;
}

/*
 @brief   : 检测电机继电器供电电流，采用中值滤波算法
 @para    : 无
 @return  : 无
 */
unsigned int Motor_Operations::Current_Detection(void)
{
	unsigned int CurrentValueBuff[11] = { 0 };
	unsigned int CurrentTemp;
	unsigned char Length = sizeof(CurrentValueBuff) / sizeof(unsigned int);
	iwdg_feed();

	for (unsigned char i = 0; i < Length; i++)
	{
		CurrentValueBuff[i] = analogRead(AIIN_PIN);
	}

	for (unsigned char i = 0; i < Length; i++)
	{
		for (unsigned char j = 0; j < Length - 1; j++)
		{
			if (CurrentValueBuff[j] > CurrentValueBuff[j + 1])
			{
				CurrentTemp = CurrentValueBuff[j + 1];
				CurrentValueBuff[j + 1] = CurrentValueBuff[j];
				CurrentValueBuff[j] = CurrentTemp;
			}
		}
	}
	return (CurrentValueBuff[Length / 2 + 1] * V_RESOLUTION * 20 + 0.5); //  voltage / 0.05
}

int Motor_Operations::Voltage_Detection(void)
{
	unsigned int VoltageValueBuffCH1[11] = { 0 };
	unsigned int VoltageValueBuffCH2[11] = { 0 };
	unsigned int VoltageTemp;
	int DifferValue;
	unsigned char Length = sizeof(VoltageValueBuffCH1) / sizeof(int);
	iwdg_feed();

	for (unsigned char i = 0; i < Length; i++)
	{
		VoltageValueBuffCH1[i] = analogRead(AVIN_CH1_PIN);
		VoltageValueBuffCH2[i] = analogRead(AVIN_CH2_PIN);
	}

	for (unsigned char i = 0; i < Length; i++)
	{
		for (unsigned char j = 0; j < Length - 1; j++)
		{
			if (VoltageValueBuffCH1[j] > VoltageValueBuffCH1[j + 1])
			{
				VoltageTemp = VoltageValueBuffCH1[j + 1];
				VoltageValueBuffCH1[j + 1] = VoltageValueBuffCH1[j];
				VoltageValueBuffCH1[j] = VoltageTemp;
			}
		}
	}

	for (unsigned char i = 0; i < Length; i++)
	{
		for (unsigned char j = 0; j < Length - 1; j++)
		{
			if (VoltageValueBuffCH2[j] > VoltageValueBuffCH2[j + 1])
			{
				VoltageTemp = VoltageValueBuffCH2[j + 1];
				VoltageValueBuffCH2[j + 1] = VoltageValueBuffCH2[j];
				VoltageValueBuffCH2[j] = VoltageTemp;
			}
		}
	}

	DifferValue = ((VoltageValueBuffCH1[Length / 2 + 1] * V_RESOLUTION * 11) - (VoltageValueBuffCH2[Length / 2 + 1] * V_RESOLUTION * 11));
	return DifferValue;
}

void Motor_Operations::Calculate_Voltage(unsigned int *voltage_collect_num, unsigned long *voltage_value, int *voltage_value_temp, unsigned int *voltage_calib)
{
	if ((gRollingTime % CURRENT_COLLECTION_FREQ == 0) && (gRollingTimeVarFlag == true))
	{
		*voltage_value_temp = Voltage_Detection();
		*voltage_collect_num += 1;
		if (*voltage_value_temp < 0) *voltage_value_temp *= -1;

		/*如果单次采集的电压大于14V，采集本次电压，如果没有，校正值加1*/
		if (*voltage_value_temp > 14000)
			*voltage_value += *voltage_value_temp;
		else
			*voltage_calib += 1;
	}
}

bool Motor_Operations::Calculate_and_Save_Voltage(unsigned int *voltage_collect_num, unsigned long *voltage_value, int *voltage_value_temp, unsigned int *voltage_calib)
{
	*voltage_value /= (*voltage_collect_num - *voltage_calib);

	if (*voltage_value == 0)
	{
		Serial.println("Collect Voltage is empty采集电压为空 !");
		return false;
	}

	unsigned int SavedVoltage = *voltage_value;

	if (!Roll_Operation.Save_Roll_Voltage(SavedVoltage))
	{
		Set_Motor_Status(STORE_EXCEPTION);
		Message_Receipt.Working_Parameter_Receipt(false, 2);
		return false;
	}

	Serial.print("Collect motor voltage value采集电机电压值(mV) = ");
	Serial.println(SavedVoltage);

	*voltage_value = 0;
	*voltage_collect_num = 0;
	*voltage_value_temp = 0;
	*voltage_calib = 0;

	return true;
}


void Motor_Operations::Dynamic_Adjust_Roll_Time(unsigned int roll_time_temp, unsigned int *roll_time)
{
	int DycVoltage, VoltageDiffer;
	unsigned int EP_Voltage;
	bool DyNegFlag = false; //判断实时采集的电压值，得出是否是向上卷
	bool NegFlag = true; //判断得出的电压差是否是负数

	iwdg_feed();

	DycVoltage = Voltage_Detection();
	if (DycVoltage < 0)
	{
		DyNegFlag = true;
		DycVoltage *= -1;
	}

	Roll_Operation.Read_Roll_Voltage(&EP_Voltage);

	/*
	  *如果目前采集的动态电压小于重置行程时候采集的电压
	  *说明电机运动时间相对重置行程加长，要加上秒数
	  *如果目前采集的动态电压大于重置行程时候采集的电压
	  *说明电机运动时间相对重置行程的时候要快，要减少运动秒数
	 */
	VoltageDiffer = DycVoltage - EP_Voltage;
	if (VoltageDiffer < 0)
	{
		VoltageDiffer *= -1;
		NegFlag = false;
	}

	//Serial.print("VoltageDiffer = ");
	//Serial.println(VoltageDiffer);

	if (VoltageDiffer > 10000)
	{
		Serial.println("Voltage ERROR电压错位 ! <Dynamic_Adjust_Roll_Time>");
		return;
	}
	if (VoltageDiffer < 500)
	{
		return;
	}

	Serial.print("Voltage adjust need time电压调整需要时间 <Dynamic_Adjust_Roll_Time> = ");

	/*如果是开棚*/
	if (DyNegFlag)
	{
		/*如果采集的电压大于重置行程的电压，电机运动变快*/
		if (NegFlag)
		{
			/*如果压差达到了4V以上，原来的倍数可能显大了，减小0.1倍*/
			if (VoltageDiffer >= 4000)
				*roll_time = roll_time_temp - ((POS_ADJUST_ROLL_UP_TIME_MULTIPLE - 0.1) * VoltageDiffer / 500);
			else
				*roll_time = roll_time_temp - (POS_ADJUST_ROLL_UP_TIME_MULTIPLE * VoltageDiffer / 500);
		}
		else
		{
			/*如果压差达到了4V以上，原来的倍数可能显小了，增加0.2的倍*/
			if (VoltageDiffer >= 4000)
				*roll_time = roll_time_temp + ((NEG_ADJUST_ROLL_UP_TIME_MULTIPLE + 0.4) * VoltageDiffer / 500);
			else
				*roll_time = roll_time_temp + (NEG_ADJUST_ROLL_UP_TIME_MULTIPLE * VoltageDiffer / 500);
		}
	}
	else //关棚
	{
		/*如果采集的电压大于重置行程的电压，电机运动变快*/
		if (NegFlag)
		{
			/*如果压差达到了4V以上，原来的倍数可能显大了，减小0.1倍*/
			if (VoltageDiffer >= 4000)
				*roll_time = roll_time_temp - ((POS_ADJUST_ROLL_DOWN_TIME_MULTIPLE - 0.1) * VoltageDiffer / 500);
			else
				*roll_time = roll_time_temp - (POS_ADJUST_ROLL_DOWN_TIME_MULTIPLE * VoltageDiffer / 500);
		}
		else
		{
			if (VoltageDiffer >= 4000)
				*roll_time = roll_time_temp + ((NEG_ADJUST_ROLL_DOWN_TIME_MULTIPLE + 0.4) * VoltageDiffer / 500);
			else
				*roll_time = roll_time_temp + (NEG_ADJUST_ROLL_DOWN_TIME_MULTIPLE * VoltageDiffer / 500);
		}
	}

	Serial.println(*roll_time);
}

/*
 @brief   : 在重置行程中，每过一段指定的时间，累加采集一次当前电流值
 @para    : *roll_pulse_temp ---> realtime pulse
			*current_value ---> total current value
			*current_value_temp ---> once current
 @return  : None
 */
void Motor_Operations::Collect_Current(unsigned int *current_collect_num, unsigned int *current_value, unsigned int *current_value_temp, unsigned int *current_calib)
{
	if ((gRollingTime % CURRENT_COLLECTION_FREQ == 0) && (gRollingTimeVarFlag == true))
	{
		gRollingTimeVarFlag = false;
		*current_value_temp = Current_Detection();
		*current_collect_num += 1;
		/*如果单次采集的电流要于电机空载最低电流*/
		if (*current_value_temp > 300)
			*current_value += *current_value_temp;
		else
			*current_calib += 1;
	}
}

/*
 @brief   : 在重置好行程后，计算平均卷膜电流值并保存相关参数

 @para    : *roll_pulse_temp ---> roll pulse numbers.
			*current_value ---> total current value.
			*current_value_temp ---> single current value.
			act ---> Open or Close.
 @return  : true or false
 */
bool Motor_Operations::Calculate_and_Save_Current(unsigned int *current_collect_num, unsigned int *current_value, unsigned int *current_value_temp, unsigned int *current_calib, Limit_Detection act)
{
	bool UpBoolValue = true, DownBoolValue = true;
	unsigned int CurrentTemp = 0;

	if (act == Close)
	{
		Roll_Operation.Read_Roll_Up_Current(&CurrentTemp);
		if (CurrentTemp > 0 && (CurrentTemp <= 3000))
		{
			*current_value = CurrentTemp;
		}
		else
		{
			/*如果开棚平均电流为0，给关棚平均电流默认设置为800mA*/
			*current_value = 800;
		}
	}
	else
	{
		*current_value /= (*current_collect_num - *current_calib);
		if (*current_value == 0)
		{
			Serial.println("Collect current is empty !");
			return false;
		}
	}

	switch (act)
	{
	case Open:  UpBoolValue = Roll_Operation.Save_Roll_Up_Current(*current_value);

		if (UpBoolValue == false)
		{
			Serial.println("Save roll up current value ERROR保存上卷当前值错误! <Calculate_and_Save_Current>");
			Set_Motor_Status(STORE_EXCEPTION);
			Message_Receipt.Working_Parameter_Receipt(false, 2);
		}
		else
		{
			Serial.print("Roll up current value上卷电流值(mA) : "); Serial.println(*current_value);
			float Current_Coefficient = Roll_Operation.Read_Roll_High_Current_Limit_Value() / 10.0;
			Serial.print("Current_Coefficient当前系数 = "); Serial.println(Current_Coefficient);
			Serial.println(*current_value * Current_Coefficient);
		}
		break;

	case Close:  DownBoolValue = Roll_Operation.Save_Roll_Down_Current(*current_value);

		if (DownBoolValue == false)
		{
			Serial.println("Save roll down current value ERROR保存下卷当前值错误! <Calculate_and_Save_Current>");
			Set_Motor_Status(STORE_EXCEPTION);
			Message_Receipt.Working_Parameter_Receipt(false, 2);
		}
		else
		{
			Serial.print("Roll down current value下卷电流值(mA) : "); Serial.println(*current_value);
			float Current_Coefficient = Roll_Operation.Read_Roll_High_Current_Limit_Value() / 10.0;
			Serial.print("Current_Coefficient当前系数 = "); Serial.println(Current_Coefficient);
			Serial.println(*current_value * Current_Coefficient);
		}
		break;
	}

	*current_value = 0;
	*current_collect_num = 0;
	*current_value_temp = 0;
	*current_calib = 0;

	if (UpBoolValue == false || DownBoolValue == false)
		return false;
	else
		return true;
}

/*
 @brief   : 检测是否有手动开棚行为
 @para    : 无
 @return  : 无
 */
void Manual_Up_Change_Interrupt(void)
{
	detachInterrupt(DEC_MANUAL_UP_PIN);

	if (digitalRead(DEC_MANUAL_UP_PIN) == HIGH)
	{
		delayMicroseconds(1000 * 10);
		if (digitalRead(DEC_MANUAL_UP_PIN) == HIGH)
		{
			gManualUpDetectFlag = true;
			gTraceOpeningOKFlag = false;
		}
	}
	else if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)
	{
		delayMicroseconds(1000 * 10);
		if (digitalRead(DEC_MANUAL_UP_PIN) == LOW)
		{
			gManualUpDetectFlag = false;
			Stop_Roll_Timing();
		}
	}
	attachInterrupt(DEC_MANUAL_UP_PIN, Manual_Up_Change_Interrupt, CHANGE);
}

/*
 @brief   : 检测是否有手动关棚行为
 @para    : 无
 @return  : 无
 */
void Manual_Down_Change_Interrupt(void)
{
	detachInterrupt(DEC_MANUAL_DOWN_PIN);

	if (digitalRead(DEC_MANUAL_DOWN_PIN) == HIGH)
	{
		delayMicroseconds(1000 * 10);
		if (digitalRead(DEC_MANUAL_DOWN_PIN) == HIGH)
		{
			gManualDownDetectFlag = true;
			gTraceOpeningOKFlag = false;
		}
	}
	else if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW)
	{
		delayMicroseconds(1000 * 10);
		if (digitalRead(DEC_MANUAL_DOWN_PIN) == LOW)
		{
			gManualDownDetectFlag = false;
			Stop_Roll_Timing();
		}
	}
	attachInterrupt(DEC_MANUAL_DOWN_PIN, Manual_Down_Change_Interrupt, CHANGE);
}
