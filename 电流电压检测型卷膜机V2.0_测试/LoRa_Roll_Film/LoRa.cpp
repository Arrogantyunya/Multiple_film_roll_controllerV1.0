/************************************************************************************
 *
 * 代码与注释：卢科青
 * 日期：2019/3/18
 *
 * 该文件仅仅是针对仁钰科技的MHL9LF型号的LoRa无线模块所编写，不一定适用于其他型号，或其他厂商
 * 的LoRa模块。该文件主要功能有配置MHL9LF型号LoRa模块的引脚模式、通过AT指令读取各类LoRa参数
 * 信息、通过AT指令配置各类LoRa参数等。对于该LoRa模块，需要注意的是，每一次发送的AT指令都要
 * 留意返回的数据是否是ERROR标志，如果是，建议做一些相应的异常处理。
 * 更多MHL9LF型号的LoRa无线模块信息，请参考仁钰公司的技术文档。
 * 头文件中提供了各个类的公共接口。
 *
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "LoRa.h"
#include "BCD_CON.h"
#include <libmaple/iwdg.h>
#include "public.h"
#include "fun_periph.h"
#include "Memory.h"

/*Create LoRa object*/
LoRa LoRa_MHL9LF;

/*
 @brief     : 配置LoRa模块相关引脚
 @param     : 无
 @return    : 无
 */
void LoRa::LoRa_GPIO_Config(void)
{
	pinMode(LORA_PWR_PIN, OUTPUT);
	LORA_PWR_ON;
	pinMode(AT_CMD_PIN, OUTPUT);
	pinMode(WAKEUP_PIN, OUTPUT);
	pinMode(RESET_PIN, OUTPUT);
	digitalWrite(WAKEUP_PIN, LOW);
	digitalWrite(RESET_PIN, HIGH);
	digitalWrite(AT_CMD_PIN, LOW);  //引脚上一直默认透传模式
}

/*
 @brief     : 配置LoRa串口波特率
 @param     : baudrate(such as 4800, 9600, 115200)
 @return    : 无
 */
void LoRa::BaudRate(unsigned int baudrate)
{
	LoRa_Serial.begin(baudrate);
}

/*
 @brief     : 配置LoRa模式。
 @param     : AT_status : 高电平是AT模式，低电平是透传模式
 @return    : 无
 */
bool LoRa::Mode(LoRa_Mode AT_status)
{
	unsigned char RcvBuf[6];
	unsigned char i = 0;

	if (AT_status == AT)
		LoRa_Serial.print(SOFT_AT);
	else
		LoRa_Serial.print(SOFT_PATH);

	delay(100);
	while (LoRa_Serial.available() > 0)
	{
		if (i >= 6) return false;
		RcvBuf[i++] = LoRa_Serial.read();
	}
	if (i > 0)
	{
		if (RcvBuf[2] == 'O' && RcvBuf[3] == 'K')
			return true;
	}
	else
		return false;
}

/*
 @brief     : 决定LoRa是否重启
 @param     : Is_reset  : 软件重启LoRa模块。拉低不少于150ms，然后给一个高电平
 @return    : 无
 */
void LoRa::IsReset(bool Is_reset)
{
	if (Is_reset == true)
	{
		digitalWrite(RESET_PIN, LOW);
		delay(150); //150ms
		digitalWrite(RESET_PIN, HIGH);
	}
}

#if USE_LORA_RESET
/*
 @brief     : 将LoRa模块完全断电
 @param     : 无
 @return    : 无
 */
void LoRa::LoRa_Shutdown(void)
{
	LORA_PWR_OFF;
	/*注意！，如果下面这个end()已经执行过一次，并且中途没有重启打开begin，第二次end()，会死机！*/
	LoRa_Serial.end();
	pinMode(PA2, OUTPUT); //TX
	pinMode(PA3, OUTPUT); //RX
	digitalWrite(PA2, LOW);
	digitalWrite(PA3, LOW);
	digitalWrite(RESET_PIN, LOW);
}

/*
 @brief     : 将完全断电的LoRa模块重启
 @param     : 无
 @return    : 无
 */
void LoRa::LoRa_Restart(void)
{
	LORA_PWR_ON;
	BaudRate(9600);
	//digitalWrite(RESET_PIN, HIGH);
	IsReset(true);
	Mode(PASS_THROUGH_MODE);
}
#endif

/*
 @brief     : 检测是否收到错误回执
 @param     : 1.要检验的数据缓存
			  2.接收信息缓存
 @return    : 错误或正确
 */
unsigned char LoRa::Detect_Error_Receipt(unsigned char *verify_data)
{
	unsigned char Error;
	if (verify_data[0] == 'E' && verify_data[1] == 'R')
	{
		Error = (verify_data[2] - '0') * 10 + (verify_data[3] - '0');   //解析出错误代号，如：01, 02等
		Serial.println(Error);  //打印出该错误代号
		return Error;
	}
	else
		return No_Err;
}

/*
 @brief     : 用于设置LoRa参数后，如果设置参数成功，接收到回执“OK”
 @param     : 1.要验证的数据缓存
			  2.接收信息缓存
 @return    : true or false
 */
bool LoRa::Detect_OK_Receipt(unsigned char *verify_data, unsigned char *data_buffer)
{
	/*校验接收回执，是否成功设置参数*/
	if (verify_data[0] == 'O' && verify_data[1] == 'K')
	{
		data_buffer[0] = verify_data[0];
		data_buffer[1] = verify_data[1];
		return true;
	}
	else
		return false;
}

/*
 @brief     : LoRa模块核心函数，用来发送AT指令给LoRa模块，同时也包含了判断是否接到查询数据、或设置OK，
			  或设置失败等信息。该函数用来查询参数。
 @param     : 1.AT命令
			  2.接收返回的信息
			  3.接收信息长度
 @return    : received data type (bytes, error, OK)
 */
Receive_Type LoRa::AT_Query_Cmd(const char *cmd, unsigned char *data_buffer, unsigned char *data_len = 0)
{
	unsigned char ReceiveLen = 0, CopyLen = 0;
	unsigned char AddrTemp[100] = { 0 };
	bool IsValid = false;

	/*发送指令给LoRa，同时接收返回的数据*/
	LoRa_Serial.print(cmd);
	delay(100);
	while (LoRa_Serial.available() > 0)
	{
		if (ReceiveLen >= 100)
		{
			return Invalid;
		}
		AddrTemp[ReceiveLen++] = LoRa_Serial.read();
	}
	/*判断接收的数据长度是否有效*/
	ReceiveLen > 0 ? IsValid = true : IsValid = false;

	if (ReceiveLen == 0)
		Serial.println("No receive data !!!");

	if (IsValid)
	{
		/*验证帧头帧尾格式是否合法*/
		if ((AddrTemp[0] == '\r' && AddrTemp[1] == '\n') && (AddrTemp[ReceiveLen - 1] == '\n' && AddrTemp[ReceiveLen - 2] == '\r'))
		{
			/*接收除了帧头帧尾的有效数据*/
			for (unsigned char i = 2; i < ReceiveLen - 2; i++)
			{
				AddrTemp[CopyLen++] = AddrTemp[i];
			}

			/*判断接收的回执是否是ERROR*/
			if (Detect_Error_Receipt(&AddrTemp[0]) != No_Err) return ERROR;

			/*分析接收的回执*/
			if (Parse_Command(AddrTemp, CopyLen, data_buffer, &data_len) == true)
				return Bytes;
			else
				return Invalid;
		}
		else
		{
			return Invalid;
		}
	}
	else
	{
		return Invalid;
	}
}

/*
 @brief     : LoRa模块核心函数，用来发送AT指令给LoRa模块，同时也包含了判断是否接到查询数据、或设置OK，
			  或设置失败等信息。
			  该函数用来设置参数
 @param     : 1.AT指令
			  2.要设置的参数
			  3.接收的数据缓存
 @return    : received data type (bytes, error, OK)
 */
Receive_Type LoRa::AT_Config_Cmd(const char *cmd, const char * para, unsigned char *data_buffer)
{
	/*参考 AT_Query_Cmd 函数*/
	unsigned char ReceiveLen = 0, CopyLen = 0;
	unsigned char AddrTemp[100] = { 0 };
	char AT_Cmd[100] = { 0 };
	unsigned char i = 0, j = 0;
	bool IsValid = false;

	for (; cmd[i] != '\0'; i++)
		AT_Cmd[i] = cmd[i];

	for (; para[j] != '\0'; j++)
		AT_Cmd[i++] = para[j];

	/*加入AT指令帧尾格式*/
	AT_Cmd[i++] = '\r';
	AT_Cmd[i++] = '\n';

	Serial.write(AT_Cmd);

	/*发送指令给LoRa，同时接收返回的数据*/
	LoRa_Serial.write(AT_Cmd, i);
	delay(100);
	while (LoRa_Serial.available() > 0)
	{
		if (ReceiveLen >= 100) return Invalid;
		AddrTemp[ReceiveLen++] = LoRa_Serial.read();
	}
	ReceiveLen > 0 ? IsValid = true : IsValid = false;

	if (IsValid)
	{
		if ((AddrTemp[0] == '\r' && AddrTemp[1] == '\n') && (AddrTemp[ReceiveLen - 1] == '\n' && AddrTemp[ReceiveLen - 2] == '\r'))
		{
			for (unsigned char i = 2; i < ReceiveLen - 2; i++)
			{
				AddrTemp[CopyLen++] = AddrTemp[i];
			}
			if (Detect_Error_Receipt(&AddrTemp[0]) != No_Err) return ERROR;
			if (Detect_OK_Receipt(&AddrTemp[0], data_buffer) == true)
				return OK;
			else
				return ERROR;
		}
		else
			return Invalid;
	}
	else
		return Invalid;
}

/*
 @brief     : 在AT指令查询参数后，用于判断返回的参数是何种参数，然后传递给对应的函数解析获取参数
 @param     : 1.接收的回执信息
			  2.回执信息的长度
			  3.返回分析的数据缓存
			  4.返回分析的数据缓存长度
 @return    : true or false
 */
bool LoRa::Parse_Command(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer, unsigned char **data_len)
{
	unsigned char WhichCmd;
	unsigned char i = 0, j = 0;

	/*判断是否命令是查询信号强度命令*/
	if (addr_temp[1] == 'C' && addr_temp[2] == 'S' && addr_temp[3] == 'Q')
		WhichCmd = CSQ;
	else
		WhichCmd = CMOMON;

	/*回执的有效数据在 : 后面，所以这里截取 : 后面的数据*/
	while (addr_temp[i] != ':')
		i++;
	i++;
	for (; i <= len; i++)
		addr_temp[j++] = addr_temp[i];

	if (WhichCmd == CMOMON)
	{
		for (i = 0; i < j; i++)
			data_buffer[i] = addr_temp[i];
	}
	else if (WhichCmd == CSQ)
	{
		**data_len = 2;
		Get_CSQ(addr_temp, j - 1, data_buffer);
	}

	return true;
}

/*
 @brief     : 得到信噪比和接收信号强度参数。
 @param     : 1.接收的数据回执
			  2.接收数据回执的长度
			  3.返回分析的数据缓存
 @return    : 无
 */
void LoRa::Get_CSQ(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer)
{
	unsigned char i = 0, j = 0;

	/*信噪比和接收强度两个值中间有逗号隔开，这里去除逗号，得到数值*/
	for (; addr_temp[i] != ','; i++)
		data_buffer[i] = addr_temp[i];

	data_buffer[i++] = 0x55;

	for (j = i; j < len; j++)
		data_buffer[j] = addr_temp[j];
}

/*
 @brief     : LoRa返回的回执信息是ASCLL码，将ASCLL码转换成HEX。
 @param     : 1.ASCLL码
			  2.ASCLL码长度
 @return    : true or false
 */
bool LoRa::String_to_Hex(unsigned char *str, unsigned char len)
{
	for (unsigned char i = 0; i < len; i++)
	{
		if (str[i] >= '0' && str[i] <= '9')
			str[i] -= '0';
		else if (str[i] >= 'A' && str[i] <= 'F')
			str[i] -= '7';
		else if (str[i] >= 'a' && str[i] <= 'f')
			str[i] -= 'W';
		else
			return false;
	}
	return true;
}

/*
 @brief     : 发送AT指令接口。可以通过该函数设置LoRa模块，或是查询设置信息。
 @param     : 1.接收返回的数据
			  2.是查询指令还是设置指令
			  3.命令名字
			  4.命令参数（如果是查询指令，忽略命令参数）
 @return    : true or false
 */
bool LoRa::LoRa_AT(unsigned char *data_buffer, bool is_query, const char *cmd, const char *para)
{
	unsigned char ReceiveLength = 0;
	unsigned char ReturnType;

	Mode(AT);

	iwdg_feed();

	/*根据指令是查询指令还是设置参数指令，分别进入不同的指令函数*/
	if (is_query == true)
		ReturnType = AT_Query_Cmd(cmd, data_buffer, &ReceiveLength);
	else
		ReturnType = AT_Config_Cmd(cmd, para, data_buffer);

	/*根据指令执行情况，返回对应状态。*/
	switch (ReturnType)
	{
	case ERROR: Serial.println("Receipt ERROR...");  Mode(PASS_THROUGH_MODE); return false; break;
	case Invalid: Serial.println("Involid!"); Mode(PASS_THROUGH_MODE); return false; break;

	case OK: Serial.println("Set para OK"); break;
		case Bytes
			:
				for (unsigned char i = 0; i < ReceiveLength; i++)
				{
					Serial.print(data_buffer[i], HEX);
					Serial.print(" ");
				}
				Serial.println();
				break;
	}

	Mode(PASS_THROUGH_MODE);
	return true;
	iwdg_feed();
}

/*
 @brief     : 由于对该LoRa模块产商出产配置的初始LoRa地址极其不信任，故该函数的功能就是
			  将LoRa地址读取出来，再写入进去。确保“表里如一”。
 @param     : 无
 @return    : 无
 */
bool LoRa::Rewrite_ID(void)
{
	unsigned char RcvBuffer[8];
	char EP_Buffer[9] = { 0 };
	char WriteAddr[9];
	unsigned char i, j = 0;
	bool VerifyFlag = true;

	/*读取LoRa通信地址*/
	if (!LoRa_AT(RcvBuffer, true, AT_ADDR_, 0))
	{
		Serial.println("Read LoRa ADDR Err <Rewrite_ID>");
		Serial.println("读取LORA地址出错<Rewrite_ID>");
		return false;
	}

	for (unsigned char i = 0; i < 8; i++)
	{
		WriteAddr[i] = RcvBuffer[i];

		if (WriteAddr[i] >= 0 && WriteAddr[i] <= 9)
			WriteAddr[i] += '0';
		else if (WriteAddr[i] >= 10 && WriteAddr[i] <= 15)
			WriteAddr[i] += '7';
	}
	WriteAddr[8] = '\0';

	if (LoRa_Para_Config.Verify_LoRa_Addr_Flag())
	{
		Serial.println("Verify LoRa Addr <Rewrite_ID>");
		Serial.println("验证LORA地址 <Rewrite_ID>");

		/* 如果读出来的LoRa地址正确 */
		if (LoRa_Para_Config.Read_LoRa_Addr((unsigned char *)EP_Buffer))
		{
			for (i = 0; i < 8; i++)
			{
				/* 如果查询的LoRa地址和EP储存的不同 */
				if (WriteAddr[i] != EP_Buffer[i])
				{
					VerifyFlag = false;
					break;
				}
			}
			/* 重新写入EP储存的正确的地址 */
			if (!VerifyFlag)
			{
				Serial.println("LoRa addr for AT Error!, write EP addr to LoRa! <Rewrite_ID>");
				if (!LoRa_AT(RcvBuffer, false, AT_ADDR, EP_Buffer))
					return false;
			}
			else
			{
				Serial.println("LoRa addr for AT Correct, OK <Rewrite_ID>");
				Serial.println("LoRa地址正确, OK <Rewrite_ID>");
			}
		}
		/* EP储存的地址不幸出错，选择相信从AT指令读出来的，重新写入EP保存 */
		else
		{
			Serial.println("EP saved err, rewrite! <Rewrite_ID>");
			if (!LoRa_Para_Config.Save_LoRa_Addr((unsigned char *)WriteAddr))
				return false;
		}
	}
	/* 如果是设备是第一次运行 */
	else
	{
		/*写入读出来的地址*/
		if (!LoRa_AT(RcvBuffer, false, AT_ADDR, WriteAddr))
			return false;

		if (!LoRa_Para_Config.Save_LoRa_Addr((unsigned char *)WriteAddr))
			return false;
	}

	return true;
}

bool LoRa::Param_Check(const char *cmd, const char *para, bool only_set)
{
	unsigned char RcvBuffer[10] = { 0 };
	unsigned char copy_len = 0;
	unsigned char param_len = 0;
	bool QueryOKFlag = true;
	unsigned char j, k;

	if (!only_set)
	{
		/* 查询该设置下的参数是否和预设的吻合 */
		if (!LoRa_AT(RcvBuffer, true, cmd, para))
		{
			Serial.println("Query parameters Faild! <Param_Check>");
			Serial.println("查询参数失败！ <Param_Check>");
			return false;
		}

		for (unsigned char i = 0; para[i] != '\0'; i++)
			param_len++;

		char CopyBuffer[param_len];

		for (unsigned char i = 0; i < param_len; i++)
		{
			CopyBuffer[i] = RcvBuffer[i];
		}

		for (unsigned char i = 0; i < param_len; i++)
		{
			if (CopyBuffer[i] >= 0 && CopyBuffer[i] <= 9)
				CopyBuffer[i] += '0';
			else if (CopyBuffer[i] >= 10 && CopyBuffer[i] <= 15)
				CopyBuffer[i] += '7';
		}

		for (unsigned char i = 0; i < param_len; i++)
		{
			if (CopyBuffer[i] != para[i])    // 如果查询的参数和预设的不吻合，重新写入预设的参数。
			{
				Serial.println("The parameters are different from what is expected. Reset the parameters <Param_Check>");
				Serial.println("这些参数与预期的不同。重新设置参数 <Param_Check>");
				QueryOKFlag = false;
				break;
			}
		}
		if (QueryOKFlag)
		{
			Serial.println("Read param correct...读取参数正确");
			return true;
		}
	}

	if (QueryOKFlag == false || only_set == true)
	{
		for (unsigned char i = 0; cmd[i] != '\n'; i++)
			copy_len++;

		char set_cmd[copy_len - 1];
		for (unsigned char i = 0; i < (copy_len - 1); i++)
			set_cmd[i] = cmd[i];

		set_cmd[copy_len - 1] = '\0';
		set_cmd[copy_len - 2] = '=';

		if (LoRa_AT(RcvBuffer, false, set_cmd, para))
		{
			return true;
		}
		else
		{
			Serial.println("Parameters set Err! 参数设置错误!<Param_Check>");
			return false;
		}
	}
}

/*
 @brief     : 初始化LoRa配置。如果没有配置，无法与网关通信。
 @param     : 无
 @return    : 无
 */
void LoRa::Parameter_Init(bool only_net)
{
	unsigned char StatusBuffer[15] = { 0 };
	unsigned char i = 0;
	unsigned char AlarmLEDRunNum = 0;
	bool SetStatusFlag;
	unsigned char c;

	Serial.println("Configurate LoRa parameters...配置LORA参数");

	do {
		i = 0;
		SetStatusFlag = true;
		iwdg_feed();
		if (!only_net)
		{
			StatusBuffer[i++] = Rewrite_ID();
			StatusBuffer[i++] = Param_Check(AT_MADDR_, "71000000", false);
			StatusBuffer[i++] = Param_Check(AT_RIQ_, "00", true);
			StatusBuffer[i++] = Param_Check(AT_TFREQ_, "1C578DE0", false);
			StatusBuffer[i++] = Param_Check(AT_RFREQ_, "1C03AE80", false);
			StatusBuffer[i++] = Param_Check(AT_SYNC_, "34", false);

#if USE_LORA_RESET
			if (LoRa_Para_Config.Read_LoRa_Com_Mode() == 0xF1)
				StatusBuffer[i++] = Param_Check(AT_NET_, "01", true);    //配置成网关模式
			else
				StatusBuffer[i++] = Param_Check(AT_NET_, "00", true);    //配置成节点模式

#else
			StatusBuffer[i++] = Param_Check(AT_NET_, "01", true);
#endif

			StatusBuffer[i++] = Param_Check(AT_TSF_, "09", false);
			StatusBuffer[i++] = Param_Check(AT_RSF_, "09", false);
			StatusBuffer[i++] = Param_Check(AT_SIP_, "01", true);
			StatusBuffer[i++] = Param_Check(AT_BW_, "07", false);
			StatusBuffer[i++] = Param_Check(AT_POW_, "14", false);
			StatusBuffer[i++] = Param_Check(AT_TIQ_, "00", true);
		}
		else
		{
#if USE_LORA_RESET
			if (LoRa_Para_Config.Read_LoRa_Com_Mode() == 0xF1)
				StatusBuffer[i++] = Param_Check(AT_NET_, "01", true);    //配置成网关模式
			else
				StatusBuffer[i++] = Param_Check(AT_NET_, "00", true);    //配置成节点模式    

#else
			StatusBuffer[i++] = Param_Check(AT_NET_, "01", true);
#endif
		}

		for (unsigned char j = 0; j < i; j++)
		{
			if (StatusBuffer[j] == 0)
			{
				SetStatusFlag = false;
				Serial.println("Param init Err, Try again...");
				Serial.println("参数初始化错误, Try again...");

#if USE_LORA_RESET
				AlarmLEDRunNum++;
				LoRa_Restart();
				LoRa_Shutdown();
#endif
				iwdg_feed();

				break;
			}
		}
		if (SetStatusFlag)  return;

#if USE_LORA_RESET
		LoRa_Restart();
		delay(2000);
		iwdg_feed();
		while (LoRa_Serial.available() > 0)
			c = LoRa_Serial.read();
#endif

	}
#if USE_LORA_RESET
	while (AlarmLEDRunNum < 10);
#else
	while (AlarmLEDRunNum < 1);
#endif


	LED_SET_LORA_PARA_ERROR;
	Serial.println("LoRa parameters set ERROR! <Parameter_Init>");
	Serial.println("LORA参数配置错误! <Parameter_Init>");
	delay(3000);
	iwdg_feed();

	nvic_sys_reset();
}
