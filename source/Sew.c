/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
 * All rights reserved.
 *
 * @file Sew.c
 * 缝纫功能函数处理文件.
 * 
 * 
 *
 * @author yemj
 * @version 1.0
 * @date 2010-7-28
 *
 */


#include "DSP2803x_Device.h"			// DSP2803x Headerfile Include File

#include "GlobalVars.h"
#include "DMCvars.h"
#include "Defines.h"
#include "build.h"
#include "IO_Def.h"
#include "hmi.h"


void SewPro_Idle(void);
void SewPro_Run(void);
void SewRun_Free(void);
void SewRun_Simple(void);
void SewStop_Pos(void);

void SewSoftStart(void);
void StitchCount(void);
void SpeedLimit(void);
void SpeedCurve(void);
void SpeedStop(int16 stitchleft, int16 speedmin);
void stop_clear(void);
void pid3_clear(PIDREG3 *p);
void SewRun_AutoRun(void);
void SewStopDN_UP(void);
void SewPro_Nc(void);

void Curve_Stop(int32 dest1, int32 spd1, int32 dest2, int32 spd2);

int32 DEGREE_to_PULSE(int32 degree)		//度 -> 脉冲
	{return ((int32)(degree)*ENC_RESOLUTION*4/360);}
int32 RPM_to_PPMS(int32 rpm)			//rpm(Q0)  -> pulse/ms(Q11)
	{return (_IQ11mpyIQX((rpm),0,_IQ30((ENC_RESOLUTION*4)/60/1000),30));}
int32 PPMS_to_RPM(int32 ppms)			//pulse/ms(Q11)  -> rpm(Q0)
	{return (_IQ11toF(ppms)*60*1000/(ENC_RESOLUTION*4));}
int32 RPMPS_to_PPMSPMS(int32 rpmps)		//rpm/s(Q0) -> pulse/ms/ms(Q11)
	{return (_IQ11mpyIQX((rpmps),0,_IQ30((ENC_RESOLUTION*4)/60/1000000),30));}
Uint32 MechThetaDeltaCal(int16 mechtheta_target)	//计算停针偏差角度 Q32=360
	{return ((Uint32)(_IQ24(mechtheta_target/360.0)<<8) - (Uint32)(qep1.MechTheta<<8));}

extern Uint16 HmiNcCmd;					// 补针命令

void SewStopDN_UP_FlagClr(void)
{
	flagSew.bit.SewStopDNToUPFlag = 0;
}
void SewStopDN_UP_FlagSet(void)
{
	flagSew.bit.SewStopDNToUPFlag = 1;
}
Uint16 SewStopDN_UP_FlagState(void)
{
	return flagSew.bit.SewStopDNToUPFlag;
}
/**
 * SewProcess
 * 
 * 缝纫功能入口程序
 * 
 * 
 */
void SewProcess(void)
{
	if(ErrorCode != 0)
	{//故障时的处理
		SewState = SEW_STATE_IDLE;
		stop_clear();
		return;
	}

	switch(SewState)
	{//判断各个缝纫工况
		case SEW_STATE_IDLE:
		default:
			SewPro_Idle();
			break;
		case SEW_STATE_RUN:
			SewPro_Run();
			SewStopDN_UP_FlagClr();
			break;
		case SEW_STATE_DN_TO_UP:
			SewStopDN_UP();
			break;
		case SEW_STATE_NC:
			SewPro_Nc();
			break;
	}//end of switch

	SpeedLimit();				//最高速限制
	SpeedCurve();

}//end of SewProcess()


/**
 * SewPro_Nc
 * 
 * 缝纫补针工况
 * 
 * 
 */
void SewPro_Nc(void)
{
	static Uint16 flagPosDetected = 0;
	static Uint16 flagPosTarget = 0;
	Uint16 temp1 = 0;
	Uint16 temp2 = 0;

	if(flagPos.bit.PosMode == POS_MODE_NC)
	{//停针处理
		flagPos.bit.PosMode = POS_MODE_NC;
		flagPos.bit.OneStitchStop = 1;
		SewStop_Pos();			//定位处理
		flagPosDetected = 0;
	}
	else
	{//启动判断
		if((HmiNcCmd == CMD_NC_LIANXU))
		{//连续补针 运行
			SpeedSel = SpeedPedalMin;
			flagMotorState = MOTOR_STATE_RUN;
		}
		else if((HmiNcCmd == CMD_NC_OFF))
		{//连续补针 停针
			flagPos.bit.Updn = flagSet.bit.Updn;
			flagPos.bit.PosMode = POS_MODE_NC;
			flagPos.bit.OneStitchStop = 1;
			SewStop_Pos();			//定位处理
		}
		else if((HmiNcCmd == CMD_NC_ON))
		{//启动针位判断
			SpeedSel = SpeedPedalMin;
			flagMotorState = MOTOR_STATE_RUN;
			if(flagPosDetected == 0)
			{
				flagPosDetected = 1;
				temp1 = (((MechThetaUpSet + 45)>=360)?((MechThetaUpSet + 45)-360):(MechThetaUpSet + 45));
				temp2 = (((MechThetaUpSet + 195)>=360)?((MechThetaUpSet + 195)-360):(MechThetaUpSet + 195));
				if((MechThetaUpSet + 45) >= 360)	//temp1和temp2在0~195du
				{
					if(qep1.MechThetaDegree > temp1 && qep1.MechThetaDegree < temp2)
					{
						//针在布下
						flagPosTarget = 0;	//上针位停针
					}
					else
					{
						//针在布上
						flagPosTarget = 1;	//下针位停针
					}
				}
				else if((MechThetaUpSet + 45) < 360 && (MechThetaUpSet + 195) >= 360)	//在315到360，0到150
				{
					if(((qep1.MechThetaDegree > temp1)&&(qep1.MechThetaDegree <= 360))||\
					  (qep1.MechThetaDegree < temp2))
					{
						//针在布下
						flagPosTarget = 0;	//上针位停针
					}
					else
					{
						//针在布上
						flagPosTarget = 1;	//下针位停针
					}
				}
				else	//temp1到temp2之间,和第一种一样，为了明确才这样写
				{
					if(qep1.MechThetaDegree > temp1 && qep1.MechThetaDegree < temp2)
					{
						//针在布下
						flagPosTarget = 0;	//上针位停针
					}
					else
					{
						//针在布上
						flagPosTarget = 1;	//下针位停针
					}
				}
			}
		}
		else
		{//上下针位切换
			flagPos.bit.Updn = flagPosTarget;
			flagPos.bit.PosMode = POS_MODE_NC;
			flagPos.bit.OneStitchStop = 1;
			SewStop_Pos();			//定位处理
		}
	}
}

/**
 * SewPro_Idle
 * 
 * 缝纫待机工况
 * 
 * 
 */
#define DELTA_MECH_DEGREE	25		//如果在上停针+-这个角度内都不再做后踩走上停针的过程
void SewPro_Idle(void)
{
	Uint16 temp1,temp2 = 0;

	switch(RunCmd)
	{
		case CMD_RUN:
			SewState = SEW_STATE_RUN;	//进入缝纫运行工况
			//SewStopDN_UP_FlagClr();
			break;
		case CMD_SETUP:
			if(flagSew.bit.SewStopPos == 1&&flagSet.bit.Updn == 1&&(SewMode != SEW_MODE_SIMPLE))
			{
				if(MechThetaUpSet < DELTA_MECH_DEGREE)		//上停针位置减去区间大小已经退到360以后了(小于)
				{
					temp1 = MechThetaUpSet + DELTA_MECH_DEGREE;
					temp2 = MechThetaUpSet + 360 - DELTA_MECH_DEGREE;
					if(qep1.MechThetaDegree > temp1 && qep1.MechThetaDegree < temp2)
					{
						SewState = SEW_STATE_DN_TO_UP;
					}
				}
				else
				{
					temp1 = MechThetaUpSet + DELTA_MECH_DEGREE;
					temp2 = MechThetaUpSet - DELTA_MECH_DEGREE;
					if(qep1.MechThetaDegree > temp1 || qep1.MechThetaDegree < temp2)
					{
						SewState = SEW_STATE_DN_TO_UP;
					}
				}
			}
			else
			{
			}
			break;
		case CMD_STOP:
		default:
		//	if((HmiNcCmd == CMD_NC_ON) && (flagSew.bit.Sewed == 1))
		//	SewStopDN_UP_FlagClr();
			if((HmiNcCmd == CMD_NC_ON) && (flagSew.bit.MechThetaOk == 1) && (SewMode != SEW_MODE_SIMPLE))
			{
				SewState = SEW_STATE_NC;			//进入缝纫补针工况
			}
			break;
	}//end of switch

	if(SewState == SEW_STATE_IDLE)
	{
		stop_clear();			//变量清零
	}

}//end of SewPro_Idle()


/**
 * Power_Go2Up
 * 
 * 上电找上针位
 * 
 * 
 */
void Power_Go2Up(void)
{
	static Uint16 delay = 0;
	if((ErrorCode != 0) || (flagSet.bit.Go2Up == 0) || \
	   (SewMode == SEW_MODE_SIMPLE) || (flagPower.bit.Go2UpOver == 1))
	{//有故障、参数设定值无效、简易缝、上电特殊界面,则不需找上针位
		flagPower.bit.Go2UpOver = 1;
	}
/*	else if((flagMotorState == MOTOR_STATE_STOP) && (UP == 0))
	{//初始判断: 若UP信号为低电平(已在上针位),则不需找上针位
		flagPower.bit.Go2UpOver = 1;
		qep1.MechThetaDegree = MechThetaUpSet;	//机械角度修正
		qep1.MechTheta =  _IQ24(qep1.MechThetaDegree/BASE_THETA);
		flagSew.bit.MechThetaOk = 1;			//置上电机械角度已修正标志
	}*/
	else if(AngleInit.AngleInitFlag != 1)
	{
		if(AngleInitSearch() == 1)
		{
			qep1.ElecTheta = 0;
			qep1.TotalEnc = 0;
			qep1.CntOld = 0;
			qep1.CntUpdate = 0;
			EQep1Regs.QPOSCNT = 0;
			stop_clear();
			qep1.CalibratedAngle = AngleInit.AngleResult;
			delay++;
			if(delay>=1)
			{
				delay = 0;
				AngleInit.AngleInitFlag = 1;		//搜索结束
			}
		}
	}
	else if(flagSew.bit.MechThetaOk == 0)
	{//低速找UP信号下降沿
		stop_clear();
		SpeedSel = 150;				//rpm
		flagMotorState = MOTOR_STATE_RUN;
	}
	else
	{//位置环处理
		flagPos.bit.Updn = 0;	//上停针
		flagPos.bit.PosMode = POS_MODE_GO2UP;
		flagPos.bit.Step = STEP_PLAST_CAL;		//自动找上针位,直接做沿位置环处理
		SewStop_Pos();			//定位处理
		if(flagMotorState == MOTOR_STATE_STOP)
		{//停针结束
			flagPower.bit.Go2UpOver = 1;
		}
	}
	SpeedLimit();				//最高速限制
	SpeedCurve();

}//end of Power_Go2Up()


/**
 * SewPro_Run
 * 
 * 缝纫运行工况
 * 
 * 
 */
void SewPro_Run(void)
{
	switch(SewMode)
	{
		case SEW_MODE_FREE:
		default:
			SewRun_Free();				//自由缝处理
			SewSoftStart();				//软启动处理
			break;
		case SEW_MODE_SIMPLE:
			SewRun_Simple();			//简易缝处理
			break;
	}

}//end of SewPro_Run()


/**
 * SewRun_Free
 * 
 * 缝纫运行工况 自由缝
 * 
 * 
 */
void SewRun_Free(void)
{
	if(RunCmd == CMD_RUN)
	{
		flagSew.bit.Sewed = 1;
		SpeedSel = SpeedPedal;
		flagMotorState = MOTOR_STATE_RUN;
		stop_clear();
	}
	else if(RunCmd == CMD_SETUP)
	{
		flagPos.bit.Updn = 0;					//上停针
		flagSew.bit.SoftStarted = 0;			//清除软启动标志,下次软启动有效
		SewState = SEW_STATE_DN_TO_UP;			//走上针位
		stop_clear();
	}
	else
	{
		flagPos.bit.Updn = flagSet.bit.Updn;	//上下针位
		if(flagPos.bit.Updn == 0)		//上停针
		{
			flagSew.bit.SoftStarted = 0;			//清除软启动标志,下次软启动有效
		}
		else	//下停针
		{
			flagSew.bit.SoftStarted = 1;			//置软启动标志,下次软启动无效
		}
		flagPos.bit.OneStitchStop = 0;			//非一针内停
		SewStop_Pos();							//定位处理
	}

}//end of SewRun_Free()

Uint16 Astep[10] = {1};
Uint16 Acount = 1;

/**
* SewRun_AutoRun
*
* 自动老化，老化速度，运行时间，停止时间由参数决定
*
*踏板一次运行，再踩一次停止
*/
void SewRun_AutoRun_Mode0(void)
{
	#define INIT			1
	#define HMI_START		2
	#define PEDAL_START		3
	#define STOP			4
	#define GO_INIT			5
	#define HMI_START_ANGLE_INIT 6
	#define PEDAL_START_ANGLE_INIT 7
	static Uint16 AtStep = INIT;

	#define STEP_STOP			1
	#define STEP_RUN			2
	#define STEP_RUN_TO_STOP	3
	static Uint16 FlagStep = STEP_STOP;	

	#define UNLOCK		0
	#define LOCK		1
	static Uint16 PedalLockFlag = LOCK;	//
	static Uint32 count = 0;

	if(ErrorCode != 0)
	{//故障时的处理
		SewState = SEW_STATE_IDLE;
		stop_clear();
		return;
	}

	switch(AtStep)
	{
		case INIT:
		{
			if(HmiCmd == CMD_RUN)
			{
				if(AngleInit.AngleInitFlag != 1)
				{
					AtStep = HMI_START_ANGLE_INIT;
					break;
				}
				SpeedSel = SpeedAutoTest;	//自动测试速度
				flagMotorState = MOTOR_STATE_RUN;
				AtStep = HMI_START;
			}
			else if(RunCmd == CMD_RUN)
			{
				if(AngleInit.AngleInitFlag != 1)
				{
					AtStep = PEDAL_START_ANGLE_INIT;
					break;
				}
				SpeedSel = SpeedAutoTest;	//自动测试速度
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//替HMI置标志位
				AtStep = PEDAL_START;
			}
			FlagStep = STEP_RUN;
			count = 0;
			stop_clear();
			break;
		}
		case HMI_START_ANGLE_INIT:
		{
			if(AngleInitSearch() == 1)
			{
				qep1.ElecTheta = 0;
				qep1.TotalEnc = 0;
				qep1.CntOld = 0;
				qep1.CntUpdate = 0;
				EQep1Regs.QPOSCNT = 0;
				stop_clear();
				qep1.CalibratedAngle = AngleInit.AngleResult;
				count++;
				if(count>=1)
				{
					count = 0;
					AngleInit.AngleInitFlag = 1;		//搜索结束
					
					SpeedSel = SpeedAutoTest;	//自动测试速度
				//	stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					AtStep = HMI_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case PEDAL_START_ANGLE_INIT:
		{
			if(AngleInitSearch() == 1)
			{
				qep1.ElecTheta = 0;
				qep1.TotalEnc = 0;
				qep1.CntOld = 0;
				qep1.CntUpdate = 0;
				EQep1Regs.QPOSCNT = 0;
				stop_clear();
				qep1.CalibratedAngle = AngleInit.AngleResult;
				count++;
				if(count>=1)
				{
					count = 0;
					AngleInit.AngleInitFlag = 1;		//搜索结束
					
					SpeedSel = SpeedAutoTest;	//自动测试速度
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//替HMI置标志位
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him 启动，那么有两种情况1.hmi停止，2.踏板停止
		{
			count++;
			if(HmiCmd == CMD_STOP || RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			else
			{
				if(FlagStep == STEP_STOP)
				{
					if(count > TimeAutoTest)		//大于停顿的时间
					{
						SpeedSel = SpeedAutoTest;	//自动测试速度
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//大于运行时间
					{
						FlagStep = STEP_RUN_TO_STOP;
						SpeedSel = 0;
						count = 0;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					flagPos.bit.Updn = flagSet.bit.Updn;	//上下针位
					flagPos.bit.OneStitchStop = 0;				//非一针内停
					SewStop_Pos();												//定位处理
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			break;
		}
		case PEDAL_START:		//踏板启动，那么有按键停止，踏板上升再踩下停止
		{
			count++;
			if(HmiCmd == CMD_STOP)
			{
				AtStep = STOP;
			}
			else
			{
				if(FlagStep == STEP_STOP)
				{
					if(count > TimeAutoTest)		//大于停顿的时间
					{
						SpeedSel = SpeedAutoTest;	//自动测试速度
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//大于运行时间
					{
						FlagStep = STEP_RUN_TO_STOP;
						SpeedSel = 0;
						count = 0;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					flagPos.bit.Updn = flagSet.bit.Updn;	//上下针位
					flagPos.bit.OneStitchStop = 0;				//非一针内停
					SewStop_Pos();												//定位处理
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			if(RunCmd == CMD_STOP)
			{
				PedalLockFlag = UNLOCK;		//踏板解锁，再踩一次就可以停止老化
			}
			if(PedalLockFlag ==  UNLOCK && RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			break;
		}
		case STOP:
		{
			flagPos.bit.Updn = flagSet.bit.Updn;	//上下针位
			flagPos.bit.OneStitchStop = 0;				//非一针内停
			SewStop_Pos();												//定位处理
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				count = 0;
				AtStep = GO_INIT;
				FlagStep = STEP_STOP;
				PedalLockFlag = LOCK;
				hmi_common_vars.age_start = 0;		//替HMI清标志位
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//确保踏板是停止的
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//最高速限制
	SpeedCurve();
}

Uint16 SewAutoRunSpeedLimit_Mode1(void)
{
	const Uint16 Speed_Line[5] = {2500,2000,1500,1000,500};
	
	if(StitchAutoTest == 1||StitchAutoTest == 2||StitchAutoTest == 3)
	{
		return 500;
	}
	else if(StitchAutoTest <= 5)
	{
		if((StitchAutoTest - CountStitch - 1) > 2)
		{
			return 1500;
		}
		else
		{
			return 500;
		}
	}
	else
	{
		if((StitchAutoTest - CountStitch - 1) > 5)	//剩余针数大于5针
		{
			return SpeedAutoTest;
		}
		else if((StitchAutoTest - CountStitch - 1) == 5)
		{
			return ((SpeedAutoTest > Speed_Line[0])?(Speed_Line[0]):(SpeedAutoTest));
		}
		else if((StitchAutoTest - CountStitch - 1) == 4)
		{
			return ((SpeedAutoTest > Speed_Line[1])?(Speed_Line[1]):(SpeedAutoTest));
		}
		else if((StitchAutoTest - CountStitch - 1) == 3)
		{
			return ((SpeedAutoTest > Speed_Line[2])?(Speed_Line[2]):(SpeedAutoTest));
		}
		else if((StitchAutoTest - CountStitch - 1) == 2)
		{
			return ((SpeedAutoTest > Speed_Line[3])?(Speed_Line[3]):(SpeedAutoTest));
		}
		else if((StitchAutoTest - CountStitch - 1) == 1)
		{
			return ((SpeedAutoTest > Speed_Line[4])?(Speed_Line[4]):(SpeedAutoTest));
		}
	}
	return 200;
}
void SewRun_AutoRun_Mode1(void)
{
	#define INIT			1
	#define HMI_START		2
	#define PEDAL_START		3
	#define STOP			4
	#define GO_INIT			5
	#define HMI_START_ANGLE_INIT 6
	#define PEDAL_START_ANGLE_INIT 7
	static Uint16 AtStep = INIT;

	#define STEP_STOP			1
	#define STEP_RUN			2
	#define STEP_RUN_TO_STOP	3
	static Uint16 FlagStep = STEP_STOP;	

	#define UNLOCK		0
	#define LOCK		1
	static Uint16 PedalLockFlag = LOCK;	//
	static Uint32 count = 0;

	if(ErrorCode != 0)
	{//故障时的处理
		SewState = SEW_STATE_IDLE;
		stop_clear();
		return;
	}

	StitchCount();	//记针数
	switch(AtStep)
	{
		case INIT:
		{
			if(HmiCmd == CMD_RUN)
			{
				if(AngleInit.AngleInitFlag != 1)
				{
					AtStep = HMI_START_ANGLE_INIT;
					break;
				}
				
				SpeedSel = SewAutoRunSpeedLimit_Mode1();	//自动测试速度
				flagMotorState = MOTOR_STATE_RUN;
				AtStep = HMI_START;
			}
			else if(RunCmd == CMD_RUN)
			{
				if(AngleInit.AngleInitFlag != 1)
				{
					AtStep = PEDAL_START_ANGLE_INIT;
					break;
				}
				
				SpeedSel = SewAutoRunSpeedLimit_Mode1();	//自动测试速度
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//替HMI置标志位
				AtStep = PEDAL_START;
			}
			FlagStep = STEP_RUN;
			count = 0;
			stop_clear();
			break;
		}
		case HMI_START_ANGLE_INIT:
		{
			if(AngleInitSearch() == 1)
			{
				qep1.ElecTheta = 0;
				qep1.TotalEnc = 0;
				qep1.CntOld = 0;
				qep1.CntUpdate = 0;
				EQep1Regs.QPOSCNT = 0;
				stop_clear();
				qep1.CalibratedAngle = AngleInit.AngleResult;
				count++;
				if(count>=1)
				{
					count = 0;
					AngleInit.AngleInitFlag = 1;		//搜索结束
					
					SpeedSel = SpeedAutoTest;	//自动测试速度
				//	stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					AtStep = HMI_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case PEDAL_START_ANGLE_INIT:
		{
			if(AngleInitSearch() == 1)
			{
				qep1.ElecTheta = 0;
				qep1.TotalEnc = 0;
				qep1.CntOld = 0;
				qep1.CntUpdate = 0;
				EQep1Regs.QPOSCNT = 0;
				stop_clear();
				qep1.CalibratedAngle = AngleInit.AngleResult;
				count++;
				if(count>=1)
				{
					count = 0;
					AngleInit.AngleInitFlag = 1;		//搜索结束
					
					SpeedSel = SpeedAutoTest;	//自动测试速度
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//替HMI置标志位
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him 启动，那么有两种情况1.hmi停止，2.踏板停止
		{
			if(HmiCmd == CMD_STOP || RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			else
			{
				if(FlagStep == STEP_STOP)
				{
					count++;
					if(count > TimeAutoTest)		//大于停顿的时间
					{
						SpeedSel = SewAutoRunSpeedLimit_Mode1();
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					SpeedSel = SewAutoRunSpeedLimit_Mode1();
					if((StitchAutoTest - CountStitch) == 1)
					{
						FlagStep = STEP_RUN_TO_STOP;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					flagPos.bit.Updn = flagSet.bit.Updn;	//上下针位
					if(StitchAutoTest == 1)
					{
						flagPos.bit.OneStitchStop = 1;				//一针内停
					}
					else
					{
						flagPos.bit.OneStitchStop = 0;				//非一针内停
					}
					SewStop_Pos();												//定位处理
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			break;
		}
		case PEDAL_START:		//踏板启动，那么有按键停止，踏板上升再踩下停止
		{
			if(HmiCmd == CMD_STOP)
			{
				AtStep = STOP;
			}
			else
			{
				if(FlagStep == STEP_STOP)
				{
					count++;
					if(count > TimeAutoTest)		//大于停顿的时间
					{
						SpeedSel = SewAutoRunSpeedLimit_Mode1();	//自动测试速度
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					SpeedSel = SewAutoRunSpeedLimit_Mode1();
					if((StitchAutoTest - CountStitch) == 1)
					{
						FlagStep = STEP_RUN_TO_STOP;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					flagPos.bit.Updn = flagSet.bit.Updn;	//上下针位
					if(StitchAutoTest == 1)
					{
						flagPos.bit.OneStitchStop = 1;				//一针内停
					}
					else
					{
						flagPos.bit.OneStitchStop = 0;				//非一针内停
					}
					SewStop_Pos();												//定位处理
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			if(RunCmd == CMD_STOP)
			{
				PedalLockFlag = UNLOCK;		//踏板解锁，再踩一次就可以停止老化
			}
			if(PedalLockFlag ==  UNLOCK && RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			break;
		}
		case STOP:
		{
			flagPos.bit.Updn = flagSet.bit.Updn;	//上下针位
			flagPos.bit.OneStitchStop = 0;				//非一针内停
			SewStop_Pos();												//定位处理
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				count = 0;
				AtStep = GO_INIT;
				FlagStep = STEP_STOP;
				PedalLockFlag = LOCK;
				hmi_common_vars.age_start = 0;		//替HMI清标志位
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//确保踏板是停止的
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//最高速限制
	SpeedCurve();
}

void SewRun_AutoRun_Mode2(void)
{
	#define INIT			1
	#define HMI_START		2
	#define PEDAL_START		3
	#define STOP			4
	#define GO_INIT			5
	#define HMI_START_ANGLE_INIT 6
	#define PEDAL_START_ANGLE_INIT 7
	static Uint16 AtStep = INIT;

	#define STEP_STOP			1
	#define STEP_RUN			2
	#define STEP_RUN_TO_STOP	3
	static Uint16 FlagStep = STEP_STOP;	

	#define UNLOCK		0
	#define LOCK		1
	static Uint16 PedalLockFlag = LOCK;	//
	static Uint32 count = 0;

	if(ErrorCode != 0)
	{//故障时的处理
		SewState = SEW_STATE_IDLE;
		stop_clear();
		return;
	}

	if(Acount<10)
	{
		if(Astep[(Acount-1)] != AtStep)
		{
			Astep[Acount] = AtStep;
			Acount++;
		}
	}
	switch(AtStep)
	{
		case INIT:
		{
			if(HmiCmd == CMD_RUN)
			{
				if(AngleInit.AngleInitFlag != 1)
				{
					AtStep = HMI_START_ANGLE_INIT;
					break;
				}
				
				SpeedSel = SpeedAutoTest;	//自动测试速度
				flagMotorState = MOTOR_STATE_RUN;
				AtStep = HMI_START;
			}
			else if(RunCmd == CMD_RUN)
			{
				if(AngleInit.AngleInitFlag != 1)
				{
					AtStep = PEDAL_START_ANGLE_INIT;
					break;
				}
				
				SpeedSel = SpeedAutoTest;	//自动测试速度
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//替HMI置标志位
				AtStep = PEDAL_START;
			}
			FlagStep = STEP_RUN;
			count = 0;
			stop_clear();
			break;
		}
		case HMI_START_ANGLE_INIT:
		{
			if(AngleInitSearch() == 1)
			{
				qep1.ElecTheta = 0;
				qep1.TotalEnc = 0;
				qep1.CntOld = 0;
				qep1.CntUpdate = 0;
				EQep1Regs.QPOSCNT = 0;
				stop_clear();
				qep1.CalibratedAngle = AngleInit.AngleResult;
				count++;
				if(count>=1)
				{
					count = 0;
					AngleInit.AngleInitFlag = 1;		//搜索结束
					
					SpeedSel = SpeedAutoTest;	//自动测试速度
				//	stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					AtStep = HMI_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case PEDAL_START_ANGLE_INIT:
		{
			if(AngleInitSearch() == 1)
			{
				qep1.ElecTheta = 0;
				qep1.TotalEnc = 0;
				qep1.CntOld = 0;
				qep1.CntUpdate = 0;
				EQep1Regs.QPOSCNT = 0;
				stop_clear();
				qep1.CalibratedAngle = AngleInit.AngleResult;
				count++;
				if(count>=1)
				{
					count = 0;
					AngleInit.AngleInitFlag = 1;		//搜索结束
					
					SpeedSel = SpeedAutoTest;	//自动测试速度
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//替HMI置标志位
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him 启动，那么有两种情况1.hmi停止，2.踏板停止
		{
			count++;
			if(HmiCmd == CMD_STOP || RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			else
			{
				if(FlagStep == STEP_STOP)
				{
					if(count > TimeAutoTest)		//大于停顿的时间
					{
						SpeedSel = SpeedAutoTest;	//自动测试速度
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//大于运行时间
					{
						FlagStep = STEP_RUN_TO_STOP;
						SpeedSel = 0;
						count = 0;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					if(SpeedMeas.SpeedRpm <= 50)
					{
						flagMotorState = MOTOR_STATE_STOP;
						FlagStep = STEP_STOP;
						stop_clear();
					}
					
				}
			}
			break;
		}
		case PEDAL_START:		//踏板启动，那么有按键停止，踏板上升再踩下停止
		{
			count++;
			if(HmiCmd == CMD_STOP)
			{
				AtStep = STOP;
			}
			else
			{
				if(FlagStep == STEP_STOP)
				{
					if(count > TimeAutoTest)		//大于停顿的时间
					{
						SpeedSel = SpeedAutoTest;	//自动测试速度
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//大于运行时间
					{
						FlagStep = STEP_RUN_TO_STOP;
						SpeedSel = 0;
						count = 0;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					if(SpeedMeas.SpeedRpm <= 50)
					{
						flagMotorState = MOTOR_STATE_STOP;
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			if(RunCmd == CMD_STOP)
			{
				PedalLockFlag = UNLOCK;		//踏板解锁，再踩一次就可以停止老化
			}
			if(PedalLockFlag ==  UNLOCK && RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			break;
		}
		case STOP:
		{
			SpeedSel = 0;
			if(SpeedMeas.SpeedRpm <= 50)
			{
				count = 0;
				AtStep = GO_INIT;
				FlagStep = STEP_STOP;
				flagMotorState = MOTOR_STATE_STOP;
				PedalLockFlag = LOCK;
				hmi_common_vars.age_start = 0;		//替HMI清标志位
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//确保踏板是停止的
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//最高速限制
	SpeedCurve();
}

void SewRun_AutoRun_Mode3(void)
{
	static Uint32 Timer = 0;		//总时间计数器
	
	#define RUN_FEN			15
	#define RUN_TIME		((Uint32)RUN_FEN*60*1000)
	
	//修正运行时间，停止时间，运行速度
	//580 ,1s,1s,4000
	if((HmiCmd == CMD_RUN)||(RunCmd == CMD_RUN))
	{
		Timer++;
	}
	if(Timer >= RUN_TIME)
	{
		Timer = RUN_TIME;
		HmiCmd = CMD_STOP;
		RunCmd = CMD_STOP;
	}
	
	SpeedAutoTest = 4000;	//运行速度
	SpeedMax = 4000;
	TimeAutoTest = 1000;		//停顿1s
	TimeAutoRun = 1000;		//运行1s
	
	SewRun_AutoRun_Mode2();
}

void SewRun_AutoRun(void)
{
	if(AutoTestMode == 0)				//带停针的时间模式
	{
		SewRun_AutoRun_Mode0();
	}
	else if(AutoTestMode == 1)	//针数模式
	{
		SewRun_AutoRun_Mode1();
	}
	else if(AutoTestMode == 2)	//简易时间模式
	{
		SewRun_AutoRun_Mode2();
	}
	else		//工厂模式
	{
		SewRun_AutoRun_Mode3();
	}
}

/**
 * SewRun_Simple
 * 
 * 缝纫运行工况 简易缝
 * 
 * 
 */
void SewRun_Simple(void)
{
	if(RunCmd == CMD_RUN)
	{//运行
//		flagSew.bit.Sewed = 1;
		SpeedSel = SpeedPedal;
		flagMotorState = MOTOR_STATE_RUN;
		stop_clear();
	}
	else
	{//停止(无停针位处理)
		SpeedSel = 0;
		if(SpeedMeas.SpeedRpm <= 50)
		{
			flagMotorState = MOTOR_STATE_STOP;
			stop_clear();
		}
	}

}//end of SewRun_Simple()

/**
 * SewStopDN_UP
 * 
 * 
 * 
 * 
 */
void SewStopDN_UP(void)
{
	static Uint16 step = 0;

	if(RunCmd == CMD_RUN)
	{
		flagSew.bit.Sewed = 1;
		SpeedSel = SpeedPedal;
		flagMotorState = MOTOR_STATE_RUN;
		SewState = SEW_STATE_RUN;
		flagPos.bit.PosMode = 0;
		stop_clear();
	}
	else 
	{
		if(step == 0 && SewStopDN_UP_FlagState()==0)
		{
			step = 1;
			SewStopDN_UP_FlagSet();
		}
		else if(step == 1)
		{
			flagPos.bit.Updn = 0;			//上停针
			flagSew.bit.SoftStarted = 0;			//清除软启动标志,下次软启动有效
			flagPos.bit.PosMode = POS_MODE_TRIM;
			SewStop_Pos();					//定位处理
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				step = 0;
			}
		}
	}
}

/**
 * SewStop_Pos
 * 
 * 缝纫停针定位子程序
 * 为实现以角度作为位置量，用0~FFFFFFFF表示0~360 (只能做一圈以内停针)
 * 输入：flagPos
 * 
 */
void SewStop_Pos(void)
{
	static Uint16 cntDelayVibrate = 0;
	static Uint16 cntDelaySafe = 0;
	static Uint16 SpdStop;					//稳速速度
	int32 MechThetaDelta;					//停针计算用机械角度偏差			IQ32 pu
	int32 PulseDest;						//目标点的脉冲位置量
	Uint16 tempdgree = 0;

//如果机头机械角上电未校零则不停针
	if(flagSew.bit.MechThetaOk == 0)
	{
		return;
	}

//计算目标位置脉冲
	if(flagPos.bit.Fztz == 1)
	{//反转提针
		if(MechThetaUpSet > MechThetaFZTZ)
		{
			tempdgree = MechThetaUpSet - MechThetaFZTZ;
		}
		else
		{
			tempdgree = MechThetaUpSet + 360 - MechThetaFZTZ;
		}
		PulseDest 		= DEGREE_to_PULSE(tempdgree);
		MechThetaDelta 	= MechThetaDeltaCal(tempdgree);
	}
	else if(flagPos.bit.Updn == 0)
	{//上针位
		PulseDest 		= DEGREE_to_PULSE(MechThetaUpSet);
		MechThetaDelta 	= MechThetaDeltaCal(MechThetaUpSet);
	}
	else
	{//下针位
		tempdgree = ((MechThetaUpSet+MechThetaDnSet)>=360)?(MechThetaUpSet+MechThetaDnSet-360):(MechThetaUpSet+MechThetaDnSet);

		PulseDest 		= DEGREE_to_PULSE(tempdgree);
		MechThetaDelta 	= MechThetaDeltaCal(tempdgree);
	}

//分步骤停针操作处理
	switch(flagPos.bit.Step)
	{
		case STEP_START:
			if(flagPos.bit.OneStitchStop == 1)
			{//一针内停 不需稳速
				SpdStop = SpeedPedalMin+100;	//踏板低速
			}
			else
			{//非一针内停
				SpdStop = MachinePos.spd_stop;
			}
			flagPos.bit.Step = STEP_SPEED_CHECK;
		case STEP_SPEED_CHECK:
			if(curve1.over >= 1)
			{//稳速条件满足
				flagPos.bit.Step = STEP_PLAST_ENTRY;
			}
			break;
		case STEP_FZTZ:				//反转提针处理
			SpeedSel = -150;
			if(_IQ24abs(MechThetaDelta>>8) < _IQ24(5/360.0))
			{//沿附近判断
				flagPos.bit.Step = STEP_PLAST_CAL;
				cntDelayVibrate = 0;
				cntDelaySafe = 0;
			}
			break;
		case STEP_PLAST_ENTRY:		//位置规划
			if(((_IQ24abs(MechThetaDelta>>8) < _IQ24(10/360.0)) && (MechThetaDelta > 0)) || \
			   (curve1.times>50))
			{//沿附近判断(350~0)
				flagPos.bit.Step = STEP_PLAST_CAL;
				cntDelayVibrate = 0;
				cntDelaySafe = 0;
			}
			break;
		case STEP_PLAST_CAL:		//沿位置环
			pid1_pos2.Ref = MechThetaDelta;		//放大了2^8
			pid1_pos2.Fdb = 0;					//放大了2^8
			pid1_pos2.calc(&pid1_pos2);
			SpeedSel = _IQtoF(pid1_pos2.Out) * BASE_SPEED;

			if(_IQ24abs(MechThetaDelta>>8) < _IQ24(MachinePos.thetaPos/360.0))
			{
				cntDelayVibrate++;
			}
			else
			{
				cntDelayVibrate = 0;
			}
			cntDelaySafe++;

			if((cntDelayVibrate > MachinePos.StopTime1) || (cntDelaySafe > MachinePos.StopTime2))
			{
				cntDelayVibrate = 0;
				cntDelaySafe = 0;
				
				if((flagPos.bit.PosMode == POS_MODE_TRIM) && \
				   (flagSet.bit.optFZTZ == 1) && (flagPos.bit.Fztz == 0))
				{//反转提针有效,继续走反转提针
					flagPos.bit.Fztz = 1;				//反转提针处理
					flagPos.bit.Step = STEP_FZTZ;		//直接反转处理
					pid3_clear(&pid1_pos2);				//清停针位置环运算量
				}
				else
				{//停针过程结束
					flagMotorState = MOTOR_STATE_STOP;
					flagSew.bit.SewStopPos = 1;
					stop_clear();			//进入缝纫待机工况
					return;
				}
			}
			break;
		default:
			break;
	}//end of switch

	if((flagPos.bit.Step == STEP_FZTZ) || (flagPos.bit.Step == STEP_PLAST_CAL))
	{//反转提针或停针沿位置环处理
		flagPos.bit.CurveStop = 0;
	}
	else	//此处else注释后PLAST位置环无效
	{//其他工况 走位置规划处理
		flagPos.bit.CurveStop = 1;
		Curve_Stop((PulseDest-DEGREE_to_PULSE(MachinePos.thetaRef)),SpdStop,PulseDest,0);
	}

}//end of SewStop_Pos()


/**
 * Curve_Stop
 *
 * 停针时的位置规划
 *
 *
 */
void Curve_Stop(int32 dest1, int32 spd1, int32 dest2, int32 spd2)
{
	int32 n = 0;
	int32 b1 = 0;
//	int32 bn = 0;
	int32 s0 = 0;
	int32 d  = 0;
	int32 s2 = 0;
	int32 cpsAll = 0;

	if(curve1.run == 0)
	{
		curve1.p_max = RPM_to_PPMS(1000);
		curve1.p_in  = RPM_to_PPMS(0);
		curve1.p_out = RPM_to_PPMS(spd1);
		curve1.p_acc = RPMPS_to_PPMSPMS(MachinePos.acc1);
		curve1.p_dec = RPMPS_to_PPMSPMS(MachinePos.acc1);

		curve1.s = dest1 - qep1.TotalEncPos - (pid1_pos.Err>>11) - (pid1_pos.fir_err>>11) + (((qep1.EncPos<<11) - pid1_pos.Fdb)>>11);
		while(curve1.s <= 0l)curve1.s += (ENC_RESOLUTION*4);
		curve1.s <<= 11l;
		curve1.p_in = SpeedRef_P;
		curve1.dPos = curve1.p_in;
		curve1.p_max= SpeedRef_P;
		if(curve1.p_max < curve1.p_out)curve1.p_max = curve1.p_out;
		
		curve1.state = 0;
		curve1.err = 0;
		curve1.dir = 0;
		curve1.times = 0;
		curve1.run = 1;

		if(RPM_to_PPMS(speed1.SpeedRpm) <= abs(curve1.p_out))
		{//低速停针 直接做二次规划
			curve1.s = 0;
		}
	}

	if(curve1.s != 0)
	{
		if(curve1.state == 0)
		{
			if(curve1.dPos > curve1.p_max)
			{
				d = -curve1.p_dec;
				curve1.dPos += d;
				if(curve1.dPos < curve1.p_max)curve1.dPos = curve1.p_max;
			}
			else if(curve1.dPos < curve1.p_max)
			{
				d = curve1.p_acc;
				curve1.dPos += d;
				if(curve1.dPos > curve1.p_max)curve1.dPos = curve1.p_max;
			}
			else
			{
			}
			if(curve1.dPos > curve1.p_out)
			{
				d = -curve1.p_dec;
				n = (curve1.p_out - curve1.dPos)/d;
				if((curve1.p_out-curve1.dPos)<n*d)n++;
			}
			else if(curve1.dPos < curve1.p_out)
			{
				d = curve1.p_acc;
				n = (curve1.p_out - curve1.dPos)/d;
				if((curve1.p_out-curve1.dPos)>n*d)n++;
			}
			else
			{
				n = 0;
			}
			s0 = curve1.dPos*n+n*(n-1)*d/2;

			if( n != 0)
			{
				if((((curve1.s - s0)<(curve1.dPos>>1))&&(curve1.dPos>=0)) || \
				   (((curve1.s - s0)>(curve1.dPos>>1))&&(curve1.dPos<0)))
				{
					b1 = curve1.s/n-(n-1)*d/2;
					if((((b1 - curve1.dPos)<=(d<<1))&&(d>=0))||(((b1 - curve1.dPos)>=(d<<1))&&(d<0)))
					{
						s2 = b1*n+n*(n-1)*d/2;
				//		bn = b1+(n-1)*d;
						curve1.dPos = b1;
						curve1.state = 1;//切换状态
						cpsAll =  (curve1.s-s2)/n;
						curve1.err = (curve1.s-s2)-cpsAll*n;
						curve1.dPos += cpsAll;
					}
					else
					{
						if(curve1.s>0)
						{
							curve1.s+=((int32)(ENC_RESOLUTION*4)<<11);
						}
						else if(curve1.s<0)
						{
							curve1.s-=((int32)(ENC_RESOLUTION*4)<<11);
						}
					}

				}
			}

		}
		else if(curve1.state == 1)
		{
			if(curve1.dPos > curve1.p_out)
			{
				d = -curve1.p_dec;
				curve1.dPos += d;
			}
			else if(curve1.dPos < curve1.p_out)
			{
				d = curve1.p_acc;
				curve1.dPos += d;
			}
		}

		curve1.dPos_out = curve1.dPos;
		if(curve1.err > 0)
		{
			curve1.err--;
			curve1.dPos_out++;
		}
		if(curve1.err < 0)
		{
			curve1.err++;
			curve1.dPos_out--;
		}

		if(curve1.dir == 0)
		{
			if(curve1.s < 0)
			{
				curve1.dPos_out = curve1.s;
			}

		}

		curve1.s -= curve1.dPos_out;
	}
	else
	{//二次运动规划
		curve1.times++;
		if(curve1.times == 1)
		{
			curve1.p_max = curve1.p_out;
			curve1.p_in  = curve1.p_out;
			curve1.dPos  = curve1.p_out;
			curve1.p_out = RPM_to_PPMS(spd2);
			curve1.p_acc = RPMPS_to_PPMSPMS(MachinePos.acc2);
			curve1.p_dec = RPMPS_to_PPMSPMS(MachinePos.acc2);

	        curve1.s = dest2 - qep1.TotalEncPos - (pid1_pos.Err>>11) - (pid1_pos.fir_err>>11) + (((qep1.EncPos<<11) - pid1_pos.Fdb)>>11);
			while(curve1.s < 0)curve1.s += (ENC_RESOLUTION*4);
			curve1.s <<= 11;
			curve1.state = 0;
			curve1.err = 0;
			curve1.run = 1;
			curve1.over = 1;
		}
		else
		{
			curve1.dPos_out = curve1.p_out;//0
			curve1.over = 2;
		}
	}

	SpeedRef_P = curve1.dPos_out;
	flagMotorState = MOTOR_STATE_RUN;

}//end of Curve_Stop()


/**
 * SewSoftStart()
 * 
 * 软启动子程序
 * 
 * 
 */
void SewSoftStart(void)
{
	int16 speed1st,speed2nd,speed3rd;
	int16 speedsoft;

//	if(flagPos.bit.Updn == 1)		//下停针
//	{
//		return;
//	}
	if((flagSew.bit.SoftStarted == 0) && \
	   (flagSet.bit.SoftStartEn == 1) && (StitchSoftStart > 0)&&(CountStitch < StitchSoftStart))
	{
		StitchCount();		//计针数
//		if(CountStitch >= StitchSoftStart)
//		{//设定总针数到，软启动结束
//			flagSew.bit.SoftStarted = 1;
//		}
//		else
		{
			//确保 speed1st < speed2nd < speed3rd
			speed1st = SpeedSoftStart1;
			speed2nd = SpeedSoftStart2;
			if(speed1st > speed2nd)
			{
				speed2nd = speed1st;
			}
			speed3rd = SpeedSoftStart3;
			if(speed2nd > speed3rd)
			{
				speed3rd = speed2nd;
			}

			if(CountStitch <= 0)
			{//第1针
				speedsoft = speed1st;
			}
			else if(CountStitch == 1)
			{//第2针
				speedsoft = speed2nd;
			}
			else
			{//第3~9针
				speedsoft = speed3rd;
			}

			if(SpeedSel > speedsoft)
			{
				SpeedSel = speedsoft;
			}
		}
	}

}//end of SoftStartSew()


//减速曲线数据表
const Uint16 CURVE_SPEED_STOP_1[10][6] = // 加速度a: [0~3000rpm 200ms] = 250rpss = 900000rpmm
{//	   360~  300~  240~  180~  120~   60~	stitchleft
	{  430,  300,  200,  200,  200,  200},	//0
	{ 1342, 1225, 1095,  949,  775,  548},	//1
	{ 1897, 1817, 1732, 1643, 1549, 1449},	//2
	{ 2324, 2258, 2191, 2121, 2049, 1975},	//3
	{ 2683, 2627, 2569, 2510, 2449, 2387},	//4
	{ 3000, 2950, 2898, 2846, 2793, 2739},	//5
	{ 3286, 3240, 3194, 3146, 3098, 3050},	//6
	{ 3550, 3507, 3464, 3421, 3376, 3332},	//7
	{ 3795, 3755, 3715, 3674, 3633, 3592},	//8
	{ 4000, 3987, 3950, 3912, 3873, 3834},	//9
};
const Uint16 CURVE_SPEED_STOP_2[10][6] = // 加速度a: [0~3000rpm 150ms] = 333rpss = 1200000rpmm
{//	   360~  300~  240~  180~  120~   60~	stitchleft
	{  400,  200,  200,  200,  200,  200},	//0
	{ 1549, 1414, 1265, 1095,  894,  632},	//1
	{ 2191, 2098, 2000, 1897, 1789, 1673},	//2
	{ 2683, 2608, 2530, 2449, 2366, 2280},	//3
	{ 3098, 3033, 2966, 2898, 2828, 2757},	//4
	{ 3464, 3406, 3347, 3286, 3225, 3162},	//5
	{ 3795, 3742, 3688, 3633, 3578, 3521},	//6
	{ 4000, 4000, 4000, 3950, 3899, 3847},	//7
	{ 4000, 4000, 4000, 4000, 4000, 4000},	//8
	{ 4000, 4000, 4000, 4000, 4000, 4000},	//9
};
/**
 * SpeedStop
 * 
 * 停针提前限速处理(0~15针以内)
 * 
 * 输入：stitchleft, SpeedSel, speedmin
 * 输出篠peedSel
 */
void SpeedStop(int16 stitchleft, int16 speedmin)
{
	Uint16 speedlimit;
	Uint16 stitchleft_nosign;

	if(stitchleft > 15)
	{//15针以上不做处理
		return;
	}
	else if(stitchleft > 9)
	{
		speedlimit = 4000;
	}
	else
	{
		if(stitchleft >= 0)
		{
			stitchleft_nosign = stitchleft;
		}
		else
		{//负数处理为0
			stitchleft_nosign = 0;
		}
		switch(MachinePos.CurveStop)
		{
			case 1:
			default:
				speedlimit = CURVE_SPEED_STOP_1[stitchleft_nosign][qep1.MechThetaDegree/60];
				break;
			case 2:
				speedlimit = CURVE_SPEED_STOP_2[stitchleft_nosign][qep1.MechThetaDegree/60];
				break;
		}
	}

	if(speedlimit < speedmin)
	{
		speedlimit = speedmin;
	}

	if(SpeedSel > speedlimit)
	{
		SpeedSel = speedlimit;
	}

}//end of SpeedStop()


/**
 * SpeedLimit
 * 
 * 最高速度限制子程序
 * 
 * 输入：SpeedSel，SpeedMax
 * 输出：SpeedSet
 */
void SpeedLimit(void)
{
	if(abs(SpeedSel) > SpeedMax)
	{
		SpeedSet = SpeedMax;
	}
	else
	{
		SpeedSet = SpeedSel;
	}

}//end of SpeedLimit()


/**
 * SpeedCurve
 * 
 * 加减速曲线限制子程序
 * 
 * 输入：SpeedSet
 * 输出：SpeedRef
 */
void SpeedCurve(void)
{
	rc2.TargetValue = _IQ(SpeedSet/BASE_SPEED);
	rc2.calc(&rc2);
	SpeedRef = _IQtoF(rc2.SetpointValue) * BASE_SPEED;
//	SpeedRef = SpeedSet;

/*----------------------------
非位置规划: SpeedSel->SpeedSet->SpeedRef->SpeedRef_P
  位置规划: SpeedRef_P->SpeedSel->SpeedSet->SpeedRef
----------------------------*/
	if(flagPos.bit.CurveStop == 0)
	{//非停针位置规划
		SpeedRef_P = RPM_to_PPMS(SpeedRef);
	}
	else
	{//停针位置规划
		SpeedSel = PPMS_to_RPM(SpeedRef_P);
	}

}//end of SpeedCurve()


/**
 * stop_clear
 * 
 * 待机或者运行时恍┳刺曛竞捅淞康幕指?
 * 根据不同的工况作相应的处理
 * 
 */
void stop_clear(void)
{
	static Uint16 flagPidCleared = 0;

	if(flagMotorState == MOTOR_STATE_STOP)
	{
		if(SewState == SEW_STATE_IDLE)
		{//待机态
			SpeedSel = 0;
			
			//清flagSew相关标志
			flagSew.bit.Sewed = 0;
			CountStitch = -1;				//重置针数
			flagStitch.bit.Counted = 0;

		}//end of 待机
		else
		{//运行、剪线、补针
			SewState = SEW_STATE_IDLE;		//返回待机态

			if(flagPos.bit.PosMode == POS_MODE_TRIM)
			{//剪完线进入待机
				//清flagSew相关标志
				flagSew.bit.Sewed = 0;
				CountStitch = -1;				//重置针数
				flagStitch.bit.Counted = 0;
			}//end of if(flagPos.bit.PosMode == POS_MODE_TRIM)
		}//end of if(SewState == SEW_STATE_IDLE)

		pid3_clear(&pid1_spd);	//清速度环运算量
		pid3_clear(&pid1_iq);	//清iq电流环运算量
		pid3_clear(&pid1_id);	//清id电流环运算量

//清测速运算量
		speed1.Speed = 0;
		speed1.SpeedPos = 0;
		speed1.SpeedRpm = 0;
		speed1.Enc1ms = 0;
		speed1.Enc4ms = 0;
		speed1.Enc4msCnt = 0;

		SysInfo.Speed = 0;
		SpeedRef_P = 0;

		flagPidCleared = 0;

	}//end of if(flagMotorState == MOTOR_STATE_STOP)
	else //flagMotorState == MOTOR_STATE_RUN
	{
		if(flagPidCleared == 0)
		{//运行时清一次
			flagPidCleared = 1;
			pid3_clear(&pid1_spd);	//清速度环运算量
			pid3_clear(&pid1_iq);	//清iq缌骰吩怂懔�
			pid3_clear(&pid1_id);	//清id电流环运算量
		}
	}

	flagPos.all = 0;		//运行态清停针过程所用标志
	curve1.run = 0;
	curve1.over = 0;
	pid3_clear(&pid1_pos2);	//清停针位置环运算量


}//end of stop_clear()


/**
 * StitchCount
 * 
 * 计针数子程序
 * 
 * 
 */
void StitchCount(void)
{
	int16 MechThetaDeltaCheck;		//角度范围判断值 ->0(360)<-
	int16 temp = 0;

	if(flagStitch.bit.Counted == 1)
	{//针已计
		temp = ((MechThetaUpSet + MachinePos.MechThetaStitchClear)>=360)?(MechThetaUpSet + MachinePos.MechThetaStitchClear-360):(MechThetaUpSet + MachinePos.MechThetaStitchClear);
		MechThetaDeltaCheck	= (int16)(qep1.MechThetaDegree - temp);
		if(abs(MechThetaDeltaCheck) <= 30)
		{
			flagStitch.bit.Counted = 0;		//清针已计标志
		}
	}
	else if(flagMotorState == MOTOR_STATE_RUN)
	{//针未计且运行态
		temp = ((MechThetaUpSet + MachinePos.MechThetaStitchSet)>=360)?(MechThetaUpSet + MachinePos.MechThetaStitchSet-360):(MechThetaUpSet + MachinePos.MechThetaStitchSet);
		MechThetaDeltaCheck	= (int16)(qep1.MechThetaDegree - temp);
		if(abs(MechThetaDeltaCheck) <= 30)
		{
			flagStitch.bit.Counted = 1;		//置针已计标志
			CountStitch ++;					//针数累加
			GpioDataRegs.GPATOGGLE.bit.GPIO16 = 1;
		}
	}

}//end of StitchCount()


/**
 * pid3_clear
 * 
 * pid3变量清除
 * 
 * 
 */
void pid3_clear(PIDREG3 *p)
{
	
	p->Ref = 0;		//PIDREG3
	p->Fdb = 0;
	p->Out = 0;
	p->Up = 0;
	p->Ui = 0;
	p->Ud = 0;
	p->OutPreSat = 0;

}//end of pid3_clear()


//===========================================================================
// No more.
//===========================================================================
