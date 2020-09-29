/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
 * All rights reserved.
 *
 * @file Functions.c
 * 基本子函数处理文件.
 * 包含输入命令处理，踏板速度处理函数
 * 
 *
 * @author yemj
 * @version 1.0
 * @date 2010-6-23
 *
 */


#include "DSP2803x_Device.h"			// DSP2803x Headerfile Include File
#include "GlobalVars.h"
#include "DMCvars.h"
#include "IO_Def.h"
#include "Pedal.h"
#include "hmi.h"
#include "par.h"


const Uint16 TABLE_PEDALSPEED[3][NUM_PEDAL_POINT] = 
{
// OPT0 标准加速曲线
	  0,	  1,	  2,	  3,	  4,	  5,	  6,	  7,
	  8,	  9,	 10,	 11,	 12,	 13,	 14,	 15,
	 16,	 17,	 18,	 19,	 20,	 21,	 22,	 23,
	 24,	 25,	 26,	 27,	 28,	 29,	 30,	 31,
	 32,	 33,	 34,	 35,	 36,	 37,	 38,	 39,
	 40,	 41,	 42,	 43,	 44,	 45,	 46,	 47,
	 48,	 49,	 50,	 51,	 52,	 53,	 54,	 55,
	 56,	 57,	 58,	 59,	 60,	 61,	 62,	 63,
	 64,	 65,	 66,	 67,	 68,	 69,	 70,	 71,
	 72,	 73,	 74,	 75,	 76,	 77,	 78,	 79,
	 80,	 81,	 82,	 83,	 84,	 85,	 86,	 87,
	 88,	 89,	 90,	 91,	 92,	 93,	 94,	 95,
	 96,	 97,	 98,	 99,	100,	101,	102,	103,
	104,	105,	106,	107,	108,	109,	110,	111,
	112,	113,	114,	115,	116,	117,	118,	119,
	120,	121,	122,	123,	124,	125,	126,	127,
	128,	129,	130,	131,	132,	133,	134,	135,
	136,	137,	138,	139,	140,	141,	142,	143,
	144,	145,	146,	147,	148,	149,	150,	151,
	152,	153,	154,	155,	156,	157,	158,	159,
	160,	161,	162,	163,	164,	165,	166,	167,
	168,	169,	170,	171,	172,	173,	174,	175,
	176,	177,	178,	179,	180,	181,	182,	183,
	184,	185,	186,	187,	188,	189,	190,	191,
	192,	193,	194,	195,	196,	197,	198,	199,
	200,	201,	202,	203,	204,	205,	206,	207,
	208,	209,	210,	211,	212,	213,	214,	215,
	216,	217,	218,	219,	220,	221,	222,	223,
	224,	225,	226,	227,	228,	229,	230,	231,
	232,	233,	234,	235,	236,	237,	238,	239,
	240,	241,	242,	243,	244,	245,	246,	247,
	248,	249,	250,	251,	252,	253,	254,	255,
// OPT1 慢加速曲线
	  0,	  0,	  1,	  1,	  1,	  2,	  2,	  2,
	  2,	  3,	  3,	  3,	  4,	  4,	  4,	  5,
	  5,	  5,	  5,	  6,	  6,	  6,	  7,	  7,
	  7,	  8,	  8,	  8,	  8,	  9,	  9,	  9,
	 10,	 10,	 10,	 11,	 11,	 11,	 11,	 12,
	 12,	 12,	 13,	 13,	 13,	 14,	 14,	 14,
	 14,	 15,	 15,	 15,	 16,	 16,	 16,	 17,
	 17,	 17,	 17,	 18,	 18,	 18,	 19,	 19,
	 19,	 20,	 20,	 20,	 20,	 21,	 21,	 21,
	 22,	 22,	 22,	 23,	 23,	 23,	 23,	 24,
	 24,	 24,	 25,	 25,	 25,	 26,	 26,	 26,
	 26,	 27,	 27,	 27,	 28,	 28,	 28,	 29,
	 29,	 29,	 29,	 30,	 30,	 30,	 31,	 31,
	 32,	 32,	 33,	 33,	 34,	 35,	 35,	 36,
	 36,	 37,	 38,	 38,	 39,	 39,	 40,	 41,
	 41,	 42,	 42,	 43,	 44,	 44,	 45,	 45,
	 46,	 47,	 47,	 48,	 48,	 49,	 50,	 50,
	 51,	 51,	 52,	 53,	 53,	 54,	 54,	 55,
	 56,	 56,	 57,	 57,	 58,	 59,	 59,	 60,
	 60,	 61,	 62,	 63,	 64,	 65,	 66,	 67,
	 68,	 68,	 69,	 70,	 71,	 72,	 73,	 74,
	 75,	 76,	 77,	 77,	 78,	 79,	 80,	 81,
	 82,	 83,	 84,	 85,	 86,	 87,	 89,	 90,
	 92,	 93,	 95,	 96,	 98,	 99,	101,	102,
	104,	105,	107,	108,	110,	111,	113,	114,
	116,	117,	119,	120,	122,	124,	126,	128,
	130,	132,	134,	136,	138,	140,	142,	144,
	146,	148,	150,	152,	154,	156,	158,	160,
	162,	165,	167,	170,	172,	175,	178,	180,
	183,	185,	188,	191,	193,	196,	198,	201,
	204,	207,	211,	214,	218,	221,	224,	228,
	231,	235,	238,	241,	245,	248,	252,	255,
// OPT2 快加速曲线
	  0,	  3,	  6,	  9,	 12,	 15,	 18,	 21,
	 24,	 27,	 30,	 33,	 36,	 39,	 42,	 45,
	 48,	 51,	 54,	 57,	 60,	 63,	 66,	 69,
	 72,	 75,	 77,	 79,	 81,	 83,	 85,	 87,
	 89,	 91,	 93,	 95,	 97,	 99,	101,	103,
	105,	107,	109,	111,	113,	115,	117,	119,
	121,	123,	125,	127,	128,	130,	131,	133,
	134,	135,	137,	138,	140,	141,	142,	144,
	145,	147,	148,	149,	151,	152,	154,	155,
	156,	158,	159,	161,	162,	163,	165,	166,
	168,	169,	170,	172,	173,	175,	176,	177,
	179,	180,	181,	182,	183,	183,	184,	185,
	186,	187,	187,	188,	189,	190,	191,	191,
	192,	193,	194,	195,	195,	196,	197,	198,
	199,	199,	200,	201,	202,	203,	203,	204,
	205,	206,	207,	207,	208,	209,	210,	211,
	211,	212,	212,	213,	213,	214,	214,	215,
	215,	216,	216,	217,	217,	218,	218,	219,
	219,	220,	220,	221,	221,	222,	222,	223,
	223,	224,	224,	225,	225,	226,	226,	227,
	227,	228,	228,	229,	229,	230,	230,	231,
	231,	231,	232,	232,	232,	233,	233,	233,
	233,	234,	234,	234,	235,	235,	235,	236,
	236,	236,	236,	237,	237,	237,	238,	238,
	238,	239,	239,	239,	239,	240,	240,	240,
	241,	241,	241,	242,	242,	242,	242,	243,
	243,	243,	243,	244,	244,	244,	244,	245,
	245,	245,	245,	246,	246,	246,	246,	247,
	247,	247,	248,	248,	248,	248,	249,	249,
	249,	249,	250,	250,	250,	250,	251,	251,
	251,	251,	252,	252,	252,	252,	253,	253,
	253,	253,	254,	254,	254,	254,	255,	255,
};

Uint16 PedalCmdRead_MoniP(void);

#define PEDAL_CMD_STOP	0
#define PEDAL_CMD_RUN	1
//#define PEDAL_CMD_YJUP	2
//#define PEDAL_CMD_YJDN	3
//#define PEDAL_CMD_TRIM	4
#define PEDAL_CMD_SETUP	5

void Command_Read_AutoTest(void)
{
	Uint16 PedalCmd;
	static Uint16 cntPedalUnlock=0;

//有故障或翻抬开关有效或HMI禁止运行
//	if((ErrorCode != 0) || (flagSew.bit.Fantai == 1))
	if(ErrorCode != 0||(hmi_common_vars.run_forbid == 1))
	{
		RunCmd = CMD_STOP;
		HmiCmd = CMD_STOP;
		return;
	}
	
	// 踏板信号读取
	PedalCmd = PedalCmdRead_MoniP();

	if(hmi_common_vars.age_start == 1)
	{
		HmiCmd = CMD_RUN;
	}
	else if(hmi_common_vars.age_start == 0)
	{
		HmiCmd = CMD_STOP;
	}

	switch(PedalCmd)
	{//RunCmd
		case PEDAL_CMD_STOP:
		default:
			RunCmd = CMD_STOP;
			break;
		case PEDAL_CMD_RUN:
			RunCmd = CMD_RUN;
			break;
		case PEDAL_CMD_SETUP:
			RunCmd = CMD_SETUP;
			break;
	}//end of switch(PedalCmd)

	// 踏板命令解锁处理
	if(flagSew.bit.PedalUnlock == 0)
	{
		if(RunCmd != CMD_STOP)
		{//踩踏板上电不运行，S、W缝一段结束不运行
			RunCmd = CMD_STOP;
			cntPedalUnlock = 0;
		}
		else
		{//踏板回到待机态时踏板命令恢复有效
			cntPedalUnlock++;
			if(cntPedalUnlock > 50)
			{//50ms防抖处理
				flagSew.bit.PedalUnlock = 1;
			}
		}
	}
	else
	{
		cntPedalUnlock = 0;
	}

}
/**
 * Command_Read 
 * 命令输入处理函数
 * 
 * 输出：RunCmd,flagSew.bit.Fantai
 */

#define NC_KEY_ON			0
#define NC_KEY_OFF			1
#define NC_KEY_STATE		GpioDataRegs.GPADAT.bit.GPIO23
extern Uint16 HmiNcCmd;	
void Command_Read(void)
{
	Uint16 PedalCmd;
	static Uint16 cntPedalUnlock = 0;
	static Uint16 cntHmiNcDelay = 0;

//有故障或翻抬开关有效或HMI禁止运行
//	if((ErrorCode != 0) || (flagSew.bit.Fantai == 1))
	if(ErrorCode != 0||(hmi_common_vars.run_forbid == 1))
	{
		RunCmd = CMD_STOP;
		return;
	}

// 踏板信号读取
	PedalCmd = PedalCmdRead_MoniP();

	// HMI补针信号读取
	if(NC_KEY_STATE == NC_KEY_OFF)
	{
		if((cntHmiNcDelay > 5) && (cntHmiNcDelay < 200))
		{
			HmiNcCmd = CMD_NC_UPDN;
			cntHmiNcDelay = 0;
		}
		else
		{
			HmiNcCmd = CMD_NC_OFF;
			cntHmiNcDelay = 0;
		}
	}
	else
	{
		cntHmiNcDelay++;
		if(cntHmiNcDelay > 200)
		{
			cntHmiNcDelay = 200;
			HmiNcCmd = CMD_NC_LIANXU;
		}
		else if(cntHmiNcDelay > 5)
		{
			HmiNcCmd = CMD_NC_ON;
		}
	}

// 生成运行命令
	if(hmi_common_vars.hmi_states == T_MOTOR)	//电机角度测试
	{
		if(hmi_common_vars.test_angle == 1)				//角度测试开始
		{
			RunCmd = CMD_RUN;		//自动测试DHMI按键运行
		//	flagSew.bit.PedalUnlock = 1;
		}
		else
		{
			RunCmd = CMD_STOP;
		}
	}
	else
	{
		switch(PedalCmd)
		{//RunCmd
			case PEDAL_CMD_STOP:
			default:
				RunCmd = CMD_STOP;
				break;
			case PEDAL_CMD_RUN:
				RunCmd = CMD_RUN;
				break;
			case PEDAL_CMD_SETUP:
				RunCmd = CMD_SETUP;
				break;
		}//end of switch(PedalCmd)

		// 踏板命令解锁处理
		if(flagSew.bit.PedalUnlock == 0)
		{
			if(RunCmd != CMD_STOP)
			{//踩踏板上电不运行，S、W缝一段结束不运行
				RunCmd = CMD_STOP;
				cntPedalUnlock = 0;
			}
			else
			{//踏板回到待机态时踏板命令恢复有效
				cntPedalUnlock++;
				if(cntPedalUnlock > 50)
				{//50ms防抖处理
					flagSew.bit.PedalUnlock = 1;
				}
			}
		}
		else
		{
			cntPedalUnlock = 0;
		}
	}
}//end of Command_Read()

#if(PEDAL_TYPE == PEDAL_L)
/**
 * PedalCmdRead_MoniP
 * 模拟踏板命令输入处理函数
 * 
 * 
 */
Uint16 PedalCmdRead_MoniP(void)
{
	if(PedaleCapCounter.NowHZ_Q15 > PedaleCapCounter.Run)
	{
		GpioDataRegs.GPASET.bit.GPIO28 = 1;
		return PEDAL_CMD_RUN;
	}
	else if(PedaleCapCounter.NowHZ_Q15 > PedaleCapCounter.SetUp)
	{
		GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
		return PEDAL_CMD_STOP;
	}
	else //if(adc_meas1.pedalAD < PedalMoniAD.SetUp)
	{
		return PEDAL_CMD_SETUP;
	}

}//end of PedalCmdRead_MoniP()

Uint16 Pedal_Speed(Uint16 PedalAD, Uint16 PedalZoom, Uint16 PedalOpt, \
				   Uint16 SpeedMin, Uint16 SpeedMax)
{
	Uint16 SpeedPedal,idx;
	Uint16 AdMin,AdMax,AdIn;

	AdMin = PedaleCapCounter.Acc - PedaleCapCounter.Zero;
	AdMax = PedaleCapCounter.Max - PedaleCapCounter.Zero;
	
	if(PedalAD > PedaleCapCounter.Zero)
	{
		AdIn =  PedalAD - PedaleCapCounter.Zero;
	}
	else
	{
		AdIn = 0;
	}

	SpeedMax = (Uint32)SpeedMax * PedalZoom / 100;
	if(SpeedMax < SpeedMin)
	{
		SpeedMax = SpeedMin;
	}

	if(AdIn <= AdMin)
	{
		SpeedPedal = SpeedMin;
	}
	else if(AdIn >= AdMax)
	{
		SpeedPedal = SpeedMax;
	}
	else
	{
		idx = (Uint32)(AdIn - AdMin) * NUM_PEDAL_POINT / (AdMax - AdMin);
		SpeedPedal = SpeedMin + (Uint32)(SpeedMax - SpeedMin) * TABLE_PEDALSPEED[PedalOpt][idx] / RESOLUTION_PEDAL_SPEED;
	}
	return SpeedPedal;

}//end of Pedal_Speed()

#else
/**
 * PedalCmdRead_MoniP
 * 模拟踏板命令输入处理函数
 * 
 * 
 */
Uint16 PedalCmdRead_MoniP(void)
{
	if(adc_meas1.pedalAD > PedalMoniAD.Run)
	{
		return PEDAL_CMD_RUN;
	}
	else if(adc_meas1.pedalAD > PedalMoniAD.SetUp)
	{
		return PEDAL_CMD_STOP;
	}
	else //if(adc_meas1.pedalAD < PedalMoniAD.SetUp)
	{
		return PEDAL_CMD_SETUP;
	}

}//end of PedalCmdRead_MoniP()


/**
 * Pedal_Speed
 * 踏板速度处理
 * 
 * 输入：
 * 		PedalZoom	0~100 放大倍率
 * 		PedalOpt	0 标准；1 慢；2 快
 * 输出：
 */
Uint16 Pedal_Speed(Uint16 PedalAD, Uint16 PedalZoom, Uint16 PedalOpt, \
				   Uint16 SpeedMin, Uint16 SpeedMax)
{
	Uint16 SpeedPedal,idx;
	Uint16 AdMin,AdMax,AdIn;

	AdMin = PedalMoniAD.Acc - PedalMoniAD.Zero;
	AdMax = PedalMoniAD.Max - PedalMoniAD.Zero;
	if(PedalAD > PedalMoniAD.Zero)
	{
		AdIn =  PedalAD - PedalMoniAD.Zero;
	}
	else
	{
		AdIn = 0;
	}

	SpeedMax = (Uint32)SpeedMax * PedalZoom / 100;
	if(SpeedMax < SpeedMin)
	{
		SpeedMax = SpeedMin;
	}

	if(AdIn <= AdMin)
	{
		SpeedPedal = SpeedMin;
	}
	else if(AdIn >= AdMax)
	{
		SpeedPedal = SpeedMax;
	}
	else
	{
		idx = (Uint32)(AdIn - AdMin) * NUM_PEDAL_POINT / (AdMax - AdMin);
		SpeedPedal = SpeedMin + (Uint32)(SpeedMax - SpeedMin) * TABLE_PEDALSPEED[PedalOpt][idx] / RESOLUTION_PEDAL_SPEED;
	}
	return SpeedPedal;

}//end of Pedal_Speed()
#endif

/**
 * PedalZero_Autoset 
 * 模拟踏板零位校正处理函数
 * 
 * 
 */
#if(PEDAL_TYPE == PEDAL_L)
void PedalZero_Autoset(void)
{
	static Uint16 cntDelay = 0;
	int16 PedalZero;

	if(hmi_common_vars.test_pedal == 0)
	{
		return;
	}
//PEDAL_ID_MONI_Pぐ逍Ａ?	
	if(flagPedalTest.bit.Tested == 0)
	{//只测一次
		cntDelay++;
		if(cntDelay > 1000)		//上电延时1s
		{
			flagPedalTest.bit.Tested = 1;		//零位值已读取标志置1
			//y=6.8+0.116x,X为位置mm，y频率KHz;x=y/0.116-6.8/0.116=60000/T/0.116-58.6=517241.4/T-58.6
			if(PedaleCapCounter.NowCounter == 0)
			{
				PedalZero =0;	
			}
			else
			{
//				PedalZero = (((int32)(5172414/PedaleCapCounter.NowCounter)) - 586)/10 + 100;	//转换成存储值(单位: ( -100)*0.1度)
				PedalZero = ((int32)(60000000l/PedaleCapCounter.NowCounter) - (PEDAL_ZERO_HZ*1000)) / (0.02*1000) + 100;
			}	

			if((PedalZero < 89) || (PedalZero > 108))
			{//超出范围 
				ErrorCode = ERR_PDLAD;
				flagPedalTest.bit.Error = 1;
			}
			else
			{//保存测量值
				//=====yuming====================
				hmi_common_vars.test_pedal = 0;
				PARA_BUF[24] = PedalZero;
				if(ChangeParaBuf.Cnt > 7) 
			    {
			   		ChangeParaBuf.Err = 1;
			    }
			    else
			    {
			   		ChangeParaBuf.Par_Cnt[ChangeParaBuf.Cnt] = 24;
			   		ChangeParaBuf.Cnt++;
			    }
				//===============================
			}
		}
	}

}//end of PedalZero_Autoset()
#else
void PedalZero_Autoset(void)
{
	static Uint16 cntDelay = 0;
	int16 PedalZero;

	if(hmi_common_vars.test_pedal == 0)
	{
		return;
	}
//PEDAL_ID_MONI_Pぐ逍Ａ?	
	if(flagPedalTest.bit.Tested == 0)
	{//只测一次
		cntDelay++;
		if(cntDelay > 1000)		//上电延时1s
		{
			flagPedalTest.bit.Tested = 1;		//零位值已读取标志置1
			PedalZero = _IQ15mpy((adc_meas1.pedalAD - PEDAL_MONI_P_K2),PEDAL_MONI_P_K3) + 100;		//转换成存储值(单位: ( -100)*0.1度)
			if((PedalZero < 85) || (PedalZero > 115))
			{//超出范围 (-1.5度 ~ +1.5度)
				ErrorCode = ERR_PDLAD;
				flagPedalTest.bit.Error = 1;
			}
			else
			{//保存测量值
				//=====yuming====================
				hmi_common_vars.test_pedal = 0;
				hmi_common_vars.ParFlg = 1;
				PARA_BUF[24] = PedalZero;
				//===============================
			}
		}
	}

}//end of PedalZero_Autoset()
#endif

#if(PEDAL_TYPE == PEDAL_L)
/*
Uint32 Max = 0;
Uint32 Min = 10000;		//9492,9470
void PedalTest(Uint32* pdata)
{
	static Uint32 delay = 0;
	Uint16 i=0;

	delay++;
	if(delay > 10000)
	{
		delay = 10000;

		if(*pdata > Max)
		{
			Max = *pdata;
		}
		else if(*pdata < Min)
		{
			Min = *pdata;
		}
	}
}
*/
Uint32 PedalCounterRead(PEDAL_ECAP_COUNTER *pPedalCounter)
{
	Uint32 temp[4] = {0,0,0,0};
	Uint32 tempcouter = 0;
	Uint16 i = 0;
	Uint32 PedalOut = 0;
	static Uint32 PedalErrTimer = 0;
	
	#define FILTER_COUNTER		20		//MAX:20
	static Uint32 PedalFilter[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
	temp[0] = ECap1Regs.CAP1;
	temp[1] = ECap1Regs.CAP2;
	temp[2] = ECap1Regs.CAP3;
	temp[3] = ECap1Regs.CAP4;
	tempcouter = (temp[0]+temp[1]+temp[2]+temp[3])>>2;

	if(tempcouter < pPedalCounter->Err_Max || tempcouter > pPedalCounter->Err_Min)	//频率大于最大值或者小于最小值
	{
		PedalErrTimer++;
		if(PedalErrTimer > 200)	//频率大于10K过200ms
		{
			PedalErrTimer = 3001;
			if(hmi_common_vars.hmi_states != AGING)
			{
				ErrorCode = ERR_PDLAD;
			}
			//return PEDAL_ZERO_COUNTER;
		}
		return 0;
	}
	
	if(ErrorCode == ERR_PDLAD)
	{
		ErrorCode = 0;
	}
	PedalErrTimer = 0;
	for(i=0;i<(FILTER_COUNTER-1);i++)		//滤波
	{
		PedalFilter[i] = PedalFilter[i+1];
		PedalOut += PedalFilter[i];
	}
	PedalFilter[i] = tempcouter;
	PedalOut += PedalFilter[i];
	PedalOut = PedalOut / FILTER_COUNTER;
	
	return PedalOut;
}
#endif
//===========================================================================
// No more.
//===========================================================================

