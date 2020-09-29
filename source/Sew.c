/**
 * Copyright (c) 2010, …œ∫£ª˝÷¬µÁ◊”ø∆ºº”–œﬁπ´Àæ
 * All rights reserved.
 *
 * @file Sew.c
 * ∑Ï»“π¶ƒ‹∫Ø ˝¥¶¿ÌŒƒº˛.
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

int32 DEGREE_to_PULSE(int32 degree)		//∂» -> ¬ˆ≥Â
	{return ((int32)(degree)*ENC_RESOLUTION*4/360);}
int32 RPM_to_PPMS(int32 rpm)			//rpm(Q0)  -> pulse/ms(Q11)
	{return (_IQ11mpyIQX((rpm),0,_IQ30((ENC_RESOLUTION*4)/60/1000),30));}
int32 PPMS_to_RPM(int32 ppms)			//pulse/ms(Q11)  -> rpm(Q0)
	{return (_IQ11toF(ppms)*60*1000/(ENC_RESOLUTION*4));}
int32 RPMPS_to_PPMSPMS(int32 rpmps)		//rpm/s(Q0) -> pulse/ms/ms(Q11)
	{return (_IQ11mpyIQX((rpmps),0,_IQ30((ENC_RESOLUTION*4)/60/1000000),30));}
Uint32 MechThetaDeltaCal(int16 mechtheta_target)	//º∆À„Õ£’Î∆´≤ÓΩ«∂» Q32=360
	{return ((Uint32)(_IQ24(mechtheta_target/360.0)<<8) - (Uint32)(qep1.MechTheta<<8));}

extern Uint16 HmiNcCmd;					// ≤π’Î√¸¡Ó

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
 * ∑Ï»“π¶ƒ‹»Îø⁄≥Ã–Ú
 * 
 * 
 */
void SewProcess(void)
{
	if(ErrorCode != 0)
	{//π ’œ ±µƒ¥¶¿Ì
		SewState = SEW_STATE_IDLE;
		stop_clear();
		return;
	}

	switch(SewState)
	{//≈–∂œ∏˜∏ˆ∑Ï»“π§øˆ
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

	SpeedLimit();				//◊Ó∏ﬂÀŸœﬁ÷∆
	SpeedCurve();

}//end of SewProcess()


/**
 * SewPro_Nc
 * 
 * ∑Ï»“≤π’Îπ§øˆ
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
	{//Õ£’Î¥¶¿Ì
		flagPos.bit.PosMode = POS_MODE_NC;
		flagPos.bit.OneStitchStop = 1;
		SewStop_Pos();			//∂®Œª¥¶¿Ì
		flagPosDetected = 0;
	}
	else
	{//∆Ù∂Ø≈–∂œ
		if((HmiNcCmd == CMD_NC_LIANXU))
		{//¡¨–¯≤π’Î ‘À––
			SpeedSel = SpeedPedalMin;
			flagMotorState = MOTOR_STATE_RUN;
		}
		else if((HmiNcCmd == CMD_NC_OFF))
		{//¡¨–¯≤π’Î Õ£’Î
			flagPos.bit.Updn = flagSet.bit.Updn;
			flagPos.bit.PosMode = POS_MODE_NC;
			flagPos.bit.OneStitchStop = 1;
			SewStop_Pos();			//∂®Œª¥¶¿Ì
		}
		else if((HmiNcCmd == CMD_NC_ON))
		{//∆Ù∂Ø’ÎŒª≈–∂œ
			SpeedSel = SpeedPedalMin;
			flagMotorState = MOTOR_STATE_RUN;
			if(flagPosDetected == 0)
			{
				flagPosDetected = 1;
				temp1 = (((MechThetaUpSet + 45)>=360)?((MechThetaUpSet + 45)-360):(MechThetaUpSet + 45));
				temp2 = (((MechThetaUpSet + 195)>=360)?((MechThetaUpSet + 195)-360):(MechThetaUpSet + 195));
				if((MechThetaUpSet + 45) >= 360)	//temp1∫Õtemp2‘⁄0~195du
				{
					if(qep1.MechThetaDegree > temp1 && qep1.MechThetaDegree < temp2)
					{
						//’Î‘⁄≤ºœ¬
						flagPosTarget = 0;	//…œ’ÎŒªÕ£’Î
					}
					else
					{
						//’Î‘⁄≤º…œ
						flagPosTarget = 1;	//œ¬’ÎŒªÕ£’Î
					}
				}
				else if((MechThetaUpSet + 45) < 360 && (MechThetaUpSet + 195) >= 360)	//‘⁄315µΩ360£¨0µΩ150
				{
					if(((qep1.MechThetaDegree > temp1)&&(qep1.MechThetaDegree <= 360))||\
					  (qep1.MechThetaDegree < temp2))
					{
						//’Î‘⁄≤ºœ¬
						flagPosTarget = 0;	//…œ’ÎŒªÕ£’Î
					}
					else
					{
						//’Î‘⁄≤º…œ
						flagPosTarget = 1;	//œ¬’ÎŒªÕ£’Î
					}
				}
				else	//temp1µΩtemp2÷Æº‰,∫Õµ⁄“ª÷÷“ª—˘£¨Œ™¡À√˜»∑≤≈’‚—˘–¥
				{
					if(qep1.MechThetaDegree > temp1 && qep1.MechThetaDegree < temp2)
					{
						//’Î‘⁄≤ºœ¬
						flagPosTarget = 0;	//…œ’ÎŒªÕ£’Î
					}
					else
					{
						//’Î‘⁄≤º…œ
						flagPosTarget = 1;	//œ¬’ÎŒªÕ£’Î
					}
				}
			}
		}
		else
		{//…œœ¬’ÎŒª«–ªª
			flagPos.bit.Updn = flagPosTarget;
			flagPos.bit.PosMode = POS_MODE_NC;
			flagPos.bit.OneStitchStop = 1;
			SewStop_Pos();			//∂®Œª¥¶¿Ì
		}
	}
}

/**
 * SewPro_Idle
 * 
 * ∑Ï»“¥˝ª˙π§øˆ
 * 
 * 
 */
#define DELTA_MECH_DEGREE	25		//»Áπ˚‘⁄…œÕ£’Î+-’‚∏ˆΩ«∂»ƒ⁄∂º≤ª‘Ÿ◊ˆ∫Û≤»◊ﬂ…œÕ£’Îµƒπ˝≥Ã
void SewPro_Idle(void)
{
	Uint16 temp1,temp2 = 0;

	switch(RunCmd)
	{
		case CMD_RUN:
			SewState = SEW_STATE_RUN;	//Ω¯»Î∑Ï»“‘À––π§øˆ
			//SewStopDN_UP_FlagClr();
			break;
		case CMD_SETUP:
			if(flagSew.bit.SewStopPos == 1&&flagSet.bit.Updn == 1&&(SewMode != SEW_MODE_SIMPLE))
			{
				if(MechThetaUpSet < DELTA_MECH_DEGREE)		//…œÕ£’ÎŒª÷√ºı»•«¯º‰¥Û–°“—æ≠ÕÀµΩ360“‘∫Û¡À(–°”⁄)
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
				SewState = SEW_STATE_NC;			//Ω¯»Î∑Ï»“≤π’Îπ§øˆ
			}
			break;
	}//end of switch

	if(SewState == SEW_STATE_IDLE)
	{
		stop_clear();			//±‰¡ø«Â¡„
	}

}//end of SewPro_Idle()


/**
 * Power_Go2Up
 * 
 * …œµÁ’“…œ’ÎŒª
 * 
 * 
 */
void Power_Go2Up(void)
{
	static Uint16 delay = 0;
	if((ErrorCode != 0) || (flagSet.bit.Go2Up == 0) || \
	   (SewMode == SEW_MODE_SIMPLE) || (flagPower.bit.Go2UpOver == 1))
	{//”–π ’œ°¢≤Œ ˝…Ë∂®÷µŒﬁ–ß°¢ºÚ“◊∑Ï°¢…œµÁÃÿ ‚ΩÁ√Ê,‘Ú≤ª–Ë’“…œ’ÎŒª
		flagPower.bit.Go2UpOver = 1;
	}
/*	else if((flagMotorState == MOTOR_STATE_STOP) && (UP == 0))
	{//≥ı º≈–∂œ: »ÙUP–≈∫≈Œ™µÕµÁ∆Ω(“—‘⁄…œ’ÎŒª),‘Ú≤ª–Ë’“…œ’ÎŒª
		flagPower.bit.Go2UpOver = 1;
		qep1.MechThetaDegree = MechThetaUpSet;	//ª˙–µΩ«∂»–ﬁ’˝
		qep1.MechTheta =  _IQ24(qep1.MechThetaDegree/BASE_THETA);
		flagSew.bit.MechThetaOk = 1;			//÷√…œµÁª˙–µΩ«∂»“—–ﬁ’˝±Í÷æ
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
				AngleInit.AngleInitFlag = 1;		//À—À˜Ω· ¯
			}
		}
	}
	else if(flagSew.bit.MechThetaOk == 0)
	{//µÕÀŸ’“UP–≈∫≈œ¬Ωµ—ÿ
		stop_clear();
		SpeedSel = 150;				//rpm
		flagMotorState = MOTOR_STATE_RUN;
	}
	else
	{//Œª÷√ª∑¥¶¿Ì
		flagPos.bit.Updn = 0;	//…œÕ£’Î
		flagPos.bit.PosMode = POS_MODE_GO2UP;
		flagPos.bit.Step = STEP_PLAST_CAL;		//◊‘∂Ø’“…œ’ÎŒª,÷±Ω”◊ˆ—ÿŒª÷√ª∑¥¶¿Ì
		SewStop_Pos();			//∂®Œª¥¶¿Ì
		if(flagMotorState == MOTOR_STATE_STOP)
		{//Õ£’ÎΩ· ¯
			flagPower.bit.Go2UpOver = 1;
		}
	}
	SpeedLimit();				//◊Ó∏ﬂÀŸœﬁ÷∆
	SpeedCurve();

}//end of Power_Go2Up()


/**
 * SewPro_Run
 * 
 * ∑Ï»“‘À––π§øˆ
 * 
 * 
 */
void SewPro_Run(void)
{
	switch(SewMode)
	{
		case SEW_MODE_FREE:
		default:
			SewRun_Free();				//◊‘”…∑Ï¥¶¿Ì
			SewSoftStart();				//»Ì∆Ù∂Ø¥¶¿Ì
			break;
		case SEW_MODE_SIMPLE:
			SewRun_Simple();			//ºÚ“◊∑Ï¥¶¿Ì
			break;
	}

}//end of SewPro_Run()


/**
 * SewRun_Free
 * 
 * ∑Ï»“‘À––π§øˆ ◊‘”…∑Ï
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
		flagPos.bit.Updn = 0;					//…œÕ£’Î
		flagSew.bit.SoftStarted = 0;			//«Â≥˝»Ì∆Ù∂Ø±Í÷æ,œ¬¥Œ»Ì∆Ù∂Ø”––ß
		SewState = SEW_STATE_DN_TO_UP;			//◊ﬂ…œ’ÎŒª
		stop_clear();
	}
	else
	{
		flagPos.bit.Updn = flagSet.bit.Updn;	//…œœ¬’ÎŒª
		if(flagPos.bit.Updn == 0)		//…œÕ£’Î
		{
			flagSew.bit.SoftStarted = 0;			//«Â≥˝»Ì∆Ù∂Ø±Í÷æ,œ¬¥Œ»Ì∆Ù∂Ø”––ß
		}
		else	//œ¬Õ£’Î
		{
			flagSew.bit.SoftStarted = 1;			//÷√»Ì∆Ù∂Ø±Í÷æ,œ¬¥Œ»Ì∆Ù∂ØŒﬁ–ß
		}
		flagPos.bit.OneStitchStop = 0;			//∑«“ª’Îƒ⁄Õ£
		SewStop_Pos();							//∂®Œª¥¶¿Ì
	}

}//end of SewRun_Free()

Uint16 Astep[10] = {1};
Uint16 Acount = 1;

/**
* SewRun_AutoRun
*
* ◊‘∂Ø¿œªØ£¨¿œªØÀŸ∂»£¨‘À–– ±º‰£¨Õ£÷π ±º‰”…≤Œ ˝æˆ∂®
*
*Ã§∞Â“ª¥Œ‘À––£¨‘Ÿ≤»“ª¥ŒÕ£÷π
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
	{//π ’œ ±µƒ¥¶¿Ì
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
				SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
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
				SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//ÃÊHMI÷√±Í÷æŒª
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
					AngleInit.AngleInitFlag = 1;		//À—À˜Ω· ¯
					
					SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
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
					AngleInit.AngleInitFlag = 1;		//À—À˜Ω· ¯
					
					SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//ÃÊHMI÷√±Í÷æŒª
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him ∆Ù∂Ø£¨ƒ«√¥”–¡Ω÷÷«Èøˆ1.hmiÕ£÷π£¨2.Ã§∞ÂÕ£÷π
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
					if(count > TimeAutoTest)		//¥Û”⁄Õ£∂Ÿµƒ ±º‰
					{
						SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//¥Û”⁄‘À–– ±º‰
					{
						FlagStep = STEP_RUN_TO_STOP;
						SpeedSel = 0;
						count = 0;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					flagPos.bit.Updn = flagSet.bit.Updn;	//…œœ¬’ÎŒª
					flagPos.bit.OneStitchStop = 0;				//∑«“ª’Îƒ⁄Õ£
					SewStop_Pos();												//∂®Œª¥¶¿Ì
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			break;
		}
		case PEDAL_START:		//Ã§∞Â∆Ù∂Ø£¨ƒ«√¥”–∞¥º¸Õ£÷π£¨Ã§∞Â…œ…˝‘Ÿ≤»œ¬Õ£÷π
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
					if(count > TimeAutoTest)		//¥Û”⁄Õ£∂Ÿµƒ ±º‰
					{
						SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//¥Û”⁄‘À–– ±º‰
					{
						FlagStep = STEP_RUN_TO_STOP;
						SpeedSel = 0;
						count = 0;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					flagPos.bit.Updn = flagSet.bit.Updn;	//…œœ¬’ÎŒª
					flagPos.bit.OneStitchStop = 0;				//∑«“ª’Îƒ⁄Õ£
					SewStop_Pos();												//∂®Œª¥¶¿Ì
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			if(RunCmd == CMD_STOP)
			{
				PedalLockFlag = UNLOCK;		//Ã§∞ÂΩ‚À¯£¨‘Ÿ≤»“ª¥ŒæÕø…“‘Õ£÷π¿œªØ
			}
			if(PedalLockFlag ==  UNLOCK && RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			break;
		}
		case STOP:
		{
			flagPos.bit.Updn = flagSet.bit.Updn;	//…œœ¬’ÎŒª
			flagPos.bit.OneStitchStop = 0;				//∑«“ª’Îƒ⁄Õ£
			SewStop_Pos();												//∂®Œª¥¶¿Ì
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				count = 0;
				AtStep = GO_INIT;
				FlagStep = STEP_STOP;
				PedalLockFlag = LOCK;
				hmi_common_vars.age_start = 0;		//ÃÊHMI«Â±Í÷æŒª
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//»∑±£Ã§∞Â «Õ£÷πµƒ
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//◊Ó∏ﬂÀŸœﬁ÷∆
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
		if((StitchAutoTest - CountStitch - 1) > 5)	// £”‡’Î ˝¥Û”⁄5’Î
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
	{//π ’œ ±µƒ¥¶¿Ì
		SewState = SEW_STATE_IDLE;
		stop_clear();
		return;
	}

	StitchCount();	//º«’Î ˝
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
				
				SpeedSel = SewAutoRunSpeedLimit_Mode1();	//◊‘∂Ø≤‚ ‘ÀŸ∂»
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
				
				SpeedSel = SewAutoRunSpeedLimit_Mode1();	//◊‘∂Ø≤‚ ‘ÀŸ∂»
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//ÃÊHMI÷√±Í÷æŒª
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
					AngleInit.AngleInitFlag = 1;		//À—À˜Ω· ¯
					
					SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
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
					AngleInit.AngleInitFlag = 1;		//À—À˜Ω· ¯
					
					SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//ÃÊHMI÷√±Í÷æŒª
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him ∆Ù∂Ø£¨ƒ«√¥”–¡Ω÷÷«Èøˆ1.hmiÕ£÷π£¨2.Ã§∞ÂÕ£÷π
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
					if(count > TimeAutoTest)		//¥Û”⁄Õ£∂Ÿµƒ ±º‰
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
					flagPos.bit.Updn = flagSet.bit.Updn;	//…œœ¬’ÎŒª
					if(StitchAutoTest == 1)
					{
						flagPos.bit.OneStitchStop = 1;				//“ª’Îƒ⁄Õ£
					}
					else
					{
						flagPos.bit.OneStitchStop = 0;				//∑«“ª’Îƒ⁄Õ£
					}
					SewStop_Pos();												//∂®Œª¥¶¿Ì
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			break;
		}
		case PEDAL_START:		//Ã§∞Â∆Ù∂Ø£¨ƒ«√¥”–∞¥º¸Õ£÷π£¨Ã§∞Â…œ…˝‘Ÿ≤»œ¬Õ£÷π
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
					if(count > TimeAutoTest)		//¥Û”⁄Õ£∂Ÿµƒ ±º‰
					{
						SpeedSel = SewAutoRunSpeedLimit_Mode1();	//◊‘∂Ø≤‚ ‘ÀŸ∂»
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
					flagPos.bit.Updn = flagSet.bit.Updn;	//…œœ¬’ÎŒª
					if(StitchAutoTest == 1)
					{
						flagPos.bit.OneStitchStop = 1;				//“ª’Îƒ⁄Õ£
					}
					else
					{
						flagPos.bit.OneStitchStop = 0;				//∑«“ª’Îƒ⁄Õ£
					}
					SewStop_Pos();												//∂®Œª¥¶¿Ì
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			if(RunCmd == CMD_STOP)
			{
				PedalLockFlag = UNLOCK;		//Ã§∞ÂΩ‚À¯£¨‘Ÿ≤»“ª¥ŒæÕø…“‘Õ£÷π¿œªØ
			}
			if(PedalLockFlag ==  UNLOCK && RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			break;
		}
		case STOP:
		{
			flagPos.bit.Updn = flagSet.bit.Updn;	//…œœ¬’ÎŒª
			flagPos.bit.OneStitchStop = 0;				//∑«“ª’Îƒ⁄Õ£
			SewStop_Pos();												//∂®Œª¥¶¿Ì
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				count = 0;
				AtStep = GO_INIT;
				FlagStep = STEP_STOP;
				PedalLockFlag = LOCK;
				hmi_common_vars.age_start = 0;		//ÃÊHMI«Â±Í÷æŒª
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//»∑±£Ã§∞Â «Õ£÷πµƒ
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//◊Ó∏ﬂÀŸœﬁ÷∆
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
	{//π ’œ ±µƒ¥¶¿Ì
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
				
				SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
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
				
				SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//ÃÊHMI÷√±Í÷æŒª
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
					AngleInit.AngleInitFlag = 1;		//À—À˜Ω· ¯
					
					SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
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
					AngleInit.AngleInitFlag = 1;		//À—À˜Ω· ¯
					
					SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//ÃÊHMI÷√±Í÷æŒª
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him ∆Ù∂Ø£¨ƒ«√¥”–¡Ω÷÷«Èøˆ1.hmiÕ£÷π£¨2.Ã§∞ÂÕ£÷π
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
					if(count > TimeAutoTest)		//¥Û”⁄Õ£∂Ÿµƒ ±º‰
					{
						SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//¥Û”⁄‘À–– ±º‰
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
		case PEDAL_START:		//Ã§∞Â∆Ù∂Ø£¨ƒ«√¥”–∞¥º¸Õ£÷π£¨Ã§∞Â…œ…˝‘Ÿ≤»œ¬Õ£÷π
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
					if(count > TimeAutoTest)		//¥Û”⁄Õ£∂Ÿµƒ ±º‰
					{
						SpeedSel = SpeedAutoTest;	//◊‘∂Ø≤‚ ‘ÀŸ∂»
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//¥Û”⁄‘À–– ±º‰
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
				PedalLockFlag = UNLOCK;		//Ã§∞ÂΩ‚À¯£¨‘Ÿ≤»“ª¥ŒæÕø…“‘Õ£÷π¿œªØ
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
				hmi_common_vars.age_start = 0;		//ÃÊHMI«Â±Í÷æŒª
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//»∑±£Ã§∞Â «Õ£÷πµƒ
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//◊Ó∏ﬂÀŸœﬁ÷∆
	SpeedCurve();
}

void SewRun_AutoRun_Mode3(void)
{
	static Uint32 Timer = 0;		//◊‹ ±º‰º∆ ˝∆˜
	
	#define RUN_FEN			15
	#define RUN_TIME		((Uint32)RUN_FEN*60*1000)
	
	//–ﬁ’˝‘À–– ±º‰£¨Õ£÷π ±º‰£¨‘À––ÀŸ∂»
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
	
	SpeedAutoTest = 4000;	//‘À––ÀŸ∂»
	SpeedMax = 4000;
	TimeAutoTest = 1000;		//Õ£∂Ÿ1s
	TimeAutoRun = 1000;		//‘À––1s
	
	SewRun_AutoRun_Mode2();
}

void SewRun_AutoRun(void)
{
	if(AutoTestMode == 0)				//¥¯Õ£’Îµƒ ±º‰ƒ£ Ω
	{
		SewRun_AutoRun_Mode0();
	}
	else if(AutoTestMode == 1)	//’Î ˝ƒ£ Ω
	{
		SewRun_AutoRun_Mode1();
	}
	else if(AutoTestMode == 2)	//ºÚ“◊ ±º‰ƒ£ Ω
	{
		SewRun_AutoRun_Mode2();
	}
	else		//π§≥ßƒ£ Ω
	{
		SewRun_AutoRun_Mode3();
	}
}

/**
 * SewRun_Simple
 * 
 * ∑Ï»“‘À––π§øˆ ºÚ“◊∑Ï
 * 
 * 
 */
void SewRun_Simple(void)
{
	if(RunCmd == CMD_RUN)
	{//‘À––
//		flagSew.bit.Sewed = 1;
		SpeedSel = SpeedPedal;
		flagMotorState = MOTOR_STATE_RUN;
		stop_clear();
	}
	else
	{//Õ£÷π(ŒﬁÕ£’ÎŒª¥¶¿Ì)
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
			flagPos.bit.Updn = 0;			//…œÕ£’Î
			flagSew.bit.SoftStarted = 0;			//«Â≥˝»Ì∆Ù∂Ø±Í÷æ,œ¬¥Œ»Ì∆Ù∂Ø”––ß
			flagPos.bit.PosMode = POS_MODE_TRIM;
			SewStop_Pos();					//∂®Œª¥¶¿Ì
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
 * ∑Ï»“Õ£’Î∂®Œª◊”≥Ã–Ú
 * Œ™ µœ÷“‘Ω«∂»◊˜Œ™Œª÷√¡ø£¨”√0~FFFFFFFF±Ì æ0~360 (÷ªƒ‹◊ˆ“ª»¶“‘ƒ⁄Õ£’Î)
 *  ‰»Î£∫flagPos
 * 
 */
void SewStop_Pos(void)
{
	static Uint16 cntDelayVibrate = 0;
	static Uint16 cntDelaySafe = 0;
	static Uint16 SpdStop;					//Œ»ÀŸÀŸ∂»
	int32 MechThetaDelta;					//Õ£’Îº∆À„”√ª˙–µΩ«∂»∆´≤Ó			IQ32 pu
	int32 PulseDest;						//ƒø±Íµ„µƒ¬ˆ≥ÂŒª÷√¡ø
	Uint16 tempdgree = 0;

//»Áπ˚ª˙Õ∑ª˙–µΩ«…œµÁŒ¥–£¡„‘Ú≤ªÕ£’Î
	if(flagSew.bit.MechThetaOk == 0)
	{
		return;
	}

//º∆À„ƒø±ÍŒª÷√¬ˆ≥Â
	if(flagPos.bit.Fztz == 1)
	{//∑¥◊™Ã·’Î
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
	{//…œ’ÎŒª
		PulseDest 		= DEGREE_to_PULSE(MechThetaUpSet);
		MechThetaDelta 	= MechThetaDeltaCal(MechThetaUpSet);
	}
	else
	{//œ¬’ÎŒª
		tempdgree = ((MechThetaUpSet+MechThetaDnSet)>=360)?(MechThetaUpSet+MechThetaDnSet-360):(MechThetaUpSet+MechThetaDnSet);

		PulseDest 		= DEGREE_to_PULSE(tempdgree);
		MechThetaDelta 	= MechThetaDeltaCal(tempdgree);
	}

//∑÷≤Ω÷ËÕ£’Î≤Ÿ◊˜¥¶¿Ì
	switch(flagPos.bit.Step)
	{
		case STEP_START:
			if(flagPos.bit.OneStitchStop == 1)
			{//“ª’Îƒ⁄Õ£ ≤ª–ËŒ»ÀŸ
				SpdStop = SpeedPedalMin+100;	//Ã§∞ÂµÕÀŸ
			}
			else
			{//∑«“ª’Îƒ⁄Õ£
				SpdStop = MachinePos.spd_stop;
			}
			flagPos.bit.Step = STEP_SPEED_CHECK;
		case STEP_SPEED_CHECK:
			if(curve1.over >= 1)
			{//Œ»ÀŸÃıº˛¬˙◊„
				flagPos.bit.Step = STEP_PLAST_ENTRY;
			}
			break;
		case STEP_FZTZ:				//∑¥◊™Ã·’Î¥¶¿Ì
			SpeedSel = -150;
			if(_IQ24abs(MechThetaDelta>>8) < _IQ24(5/360.0))
			{//—ÿ∏ΩΩ¸≈–∂œ
				flagPos.bit.Step = STEP_PLAST_CAL;
				cntDelayVibrate = 0;
				cntDelaySafe = 0;
			}
			break;
		case STEP_PLAST_ENTRY:		//Œª÷√πÊªÆ
			if(((_IQ24abs(MechThetaDelta>>8) < _IQ24(10/360.0)) && (MechThetaDelta > 0)) || \
			   (curve1.times>50))
			{//—ÿ∏ΩΩ¸≈–∂œ(350~0)
				flagPos.bit.Step = STEP_PLAST_CAL;
				cntDelayVibrate = 0;
				cntDelaySafe = 0;
			}
			break;
		case STEP_PLAST_CAL:		//—ÿŒª÷√ª∑
			pid1_pos2.Ref = MechThetaDelta;		//∑≈¥Û¡À2^8
			pid1_pos2.Fdb = 0;					//∑≈¥Û¡À2^8
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
				{//∑¥◊™Ã·’Î”––ß,ºÃ–¯◊ﬂ∑¥◊™Ã·’Î
					flagPos.bit.Fztz = 1;				//∑¥◊™Ã·’Î¥¶¿Ì
					flagPos.bit.Step = STEP_FZTZ;		//÷±Ω”∑¥◊™¥¶¿Ì
					pid3_clear(&pid1_pos2);				//«ÂÕ£’ÎŒª÷√ª∑‘ÀÀ„¡ø
				}
				else
				{//Õ£’Îπ˝≥ÃΩ· ¯
					flagMotorState = MOTOR_STATE_STOP;
					flagSew.bit.SewStopPos = 1;
					stop_clear();			//Ω¯»Î∑Ï»“¥˝ª˙π§øˆ
					return;
				}
			}
			break;
		default:
			break;
	}//end of switch

	if((flagPos.bit.Step == STEP_FZTZ) || (flagPos.bit.Step == STEP_PLAST_CAL))
	{//∑¥◊™Ã·’ÎªÚÕ£’Î—ÿŒª÷√ª∑¥¶¿Ì
		flagPos.bit.CurveStop = 0;
	}
	else	//¥À¥¶else◊¢ Õ∫ÛPLASTŒª÷√ª∑Œﬁ–ß
	{//∆‰À˚π§øˆ ◊ﬂŒª÷√πÊªÆ¥¶¿Ì
		flagPos.bit.CurveStop = 1;
		Curve_Stop((PulseDest-DEGREE_to_PULSE(MachinePos.thetaRef)),SpdStop,PulseDest,0);
	}

}//end of SewStop_Pos()


/**
 * Curve_Stop
 *
 * Õ£’Î ±µƒŒª÷√πÊªÆ
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
		{//µÕÀŸÕ£’Î ÷±Ω”◊ˆ∂˛¥ŒπÊªÆ
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
						curve1.state = 1;//«–ªª◊¥Ã¨
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
	{//∂˛¥Œ‘À∂ØπÊªÆ
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
 * »Ì∆Ù∂Ø◊”≥Ã–Ú
 * 
 * 
 */
void SewSoftStart(void)
{
	int16 speed1st,speed2nd,speed3rd;
	int16 speedsoft;

//	if(flagPos.bit.Updn == 1)		//œ¬Õ£’Î
//	{
//		return;
//	}
	if((flagSew.bit.SoftStarted == 0) && \
	   (flagSet.bit.SoftStartEn == 1) && (StitchSoftStart > 0)&&(CountStitch < StitchSoftStart))
	{
		StitchCount();		//º∆’Î ˝
//		if(CountStitch >= StitchSoftStart)
//		{//…Ë∂®◊‹’Î ˝µΩ£¨»Ì∆Ù∂ØΩ· ¯
//			flagSew.bit.SoftStarted = 1;
//		}
//		else
		{
			//»∑±£ speed1st < speed2nd < speed3rd
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
			{//µ⁄1’Î
				speedsoft = speed1st;
			}
			else if(CountStitch == 1)
			{//µ⁄2’Î
				speedsoft = speed2nd;
			}
			else
			{//µ⁄3~9’Î
				speedsoft = speed3rd;
			}

			if(SpeedSel > speedsoft)
			{
				SpeedSel = speedsoft;
			}
		}
	}

}//end of SoftStartSew()


//ºıÀŸ«˙œﬂ ˝æ›±Ì
const Uint16 CURVE_SPEED_STOP_1[10][6] = // º”ÀŸ∂»a: [0~3000rpm 200ms] = 250rpss = 900000rpmm
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
const Uint16 CURVE_SPEED_STOP_2[10][6] = // º”ÀŸ∂»a: [0~3000rpm 150ms] = 333rpss = 1200000rpmm
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
 * Õ£’ÎÃ·«∞œﬁÀŸ¥¶¿Ì(0~15’Î“‘ƒ⁄)
 * 
 *  ‰»Î£∫stitchleft, SpeedSel, speedmin
 *  ‰≥ˆ∫SpeedSel
 */
void SpeedStop(int16 stitchleft, int16 speedmin)
{
	Uint16 speedlimit;
	Uint16 stitchleft_nosign;

	if(stitchleft > 15)
	{//15’Î“‘…œ≤ª◊ˆ¥¶¿Ì
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
		{//∏∫ ˝¥¶¿ÌŒ™0
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
 * ◊Ó∏ﬂÀŸ∂»œﬁ÷∆◊”≥Ã–Ú
 * 
 *  ‰»Î£∫SpeedSel£¨SpeedMax
 *  ‰≥ˆ£∫SpeedSet
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
 * º”ºıÀŸ«˙œﬂœﬁ÷∆◊”≥Ã–Ú
 * 
 *  ‰»Î£∫SpeedSet
 *  ‰≥ˆ£∫SpeedRef
 */
void SpeedCurve(void)
{
	rc2.TargetValue = _IQ(SpeedSet/BASE_SPEED);
	rc2.calc(&rc2);
	SpeedRef = _IQtoF(rc2.SetpointValue) * BASE_SPEED;
//	SpeedRef = SpeedSet;

/*----------------------------
∑«Œª÷√πÊªÆ: SpeedSel->SpeedSet->SpeedRef->SpeedRef_P
  Œª÷√πÊªÆ: SpeedRef_P->SpeedSel->SpeedSet->SpeedRef
----------------------------*/
	if(flagPos.bit.CurveStop == 0)
	{//∑«Õ£’ÎŒª÷√πÊªÆ
		SpeedRef_P = RPM_to_PPMS(SpeedRef);
	}
	else
	{//Õ£’ÎŒª÷√πÊªÆ
		SpeedSel = PPMS_to_RPM(SpeedRef_P);
	}

}//end of SpeedCurve()


/**
 * stop_clear
 * 
 * ¥˝ª˙ªÚ’ﬂ‘À–– ±¨“ª–©◊¥Ã¨±Í÷æ∫Õ±‰¡øµƒª÷∏?
 * ∏˘æ›≤ªÕ¨µƒπ§øˆ◊˜œ‡”¶µƒ¥¶¿Ì
 * 
 */
void stop_clear(void)
{
	static Uint16 flagPidCleared = 0;

	if(flagMotorState == MOTOR_STATE_STOP)
	{
		if(SewState == SEW_STATE_IDLE)
		{//¥˝ª˙Ã¨
			SpeedSel = 0;
			
			//«ÂflagSewœ‡πÿ±Í÷æ
			flagSew.bit.Sewed = 0;
			CountStitch = -1;				//÷ÿ÷√’Î ˝
			flagStitch.bit.Counted = 0;

		}//end of ¥˝ª˙
		else
		{//‘À––°¢ºÙœﬂ°¢≤π’Î
			SewState = SEW_STATE_IDLE;		//∑µªÿ¥˝ª˙Ã¨

			if(flagPos.bit.PosMode == POS_MODE_TRIM)
			{//ºÙÕÍœﬂΩ¯»Î¥˝ª˙
				//«ÂflagSewœ‡πÿ±Í÷æ
				flagSew.bit.Sewed = 0;
				CountStitch = -1;				//÷ÿ÷√’Î ˝
				flagStitch.bit.Counted = 0;
			}//end of if(flagPos.bit.PosMode == POS_MODE_TRIM)
		}//end of if(SewState == SEW_STATE_IDLE)

		pid3_clear(&pid1_spd);	//«ÂÀŸ∂»ª∑‘ÀÀ„¡ø
		pid3_clear(&pid1_iq);	//«ÂiqµÁ¡˜ª∑‘ÀÀ„¡ø
		pid3_clear(&pid1_id);	//«ÂidµÁ¡˜ª∑‘ÀÀ„¡ø

//«Â≤‚ÀŸ‘ÀÀ„¡ø
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
		{//‘À–– ±«Â“ª¥Œ
			flagPidCleared = 1;
			pid3_clear(&pid1_spd);	//«ÂÀŸ∂»ª∑‘ÀÀ„¡ø
			pid3_clear(&pid1_iq);	//«ÂiqÁ¡˜ª∑‘ÀÀ„¡ø
			pid3_clear(&pid1_id);	//«ÂidµÁ¡˜ª∑‘ÀÀ„¡ø
		}
	}

	flagPos.all = 0;		//‘À––Ã¨«ÂÕ£’Îπ˝≥ÃÀ˘”√±Í÷æ
	curve1.run = 0;
	curve1.over = 0;
	pid3_clear(&pid1_pos2);	//«ÂÕ£’ÎŒª÷√ª∑‘ÀÀ„¡ø


}//end of stop_clear()


/**
 * StitchCount
 * 
 * º∆’Î ˝◊”≥Ã–Ú
 * 
 * 
 */
void StitchCount(void)
{
	int16 MechThetaDeltaCheck;		//Ω«∂»∑∂Œß≈–∂œ÷µ ->0(360)<-
	int16 temp = 0;

	if(flagStitch.bit.Counted == 1)
	{//’Î“—º∆
		temp = ((MechThetaUpSet + MachinePos.MechThetaStitchClear)>=360)?(MechThetaUpSet + MachinePos.MechThetaStitchClear-360):(MechThetaUpSet + MachinePos.MechThetaStitchClear);
		MechThetaDeltaCheck	= (int16)(qep1.MechThetaDegree - temp);
		if(abs(MechThetaDeltaCheck) <= 30)
		{
			flagStitch.bit.Counted = 0;		//«Â’Î“—º∆±Í÷æ
		}
	}
	else if(flagMotorState == MOTOR_STATE_RUN)
	{//’ÎŒ¥º∆«“‘À––Ã¨
		temp = ((MechThetaUpSet + MachinePos.MechThetaStitchSet)>=360)?(MechThetaUpSet + MachinePos.MechThetaStitchSet-360):(MechThetaUpSet + MachinePos.MechThetaStitchSet);
		MechThetaDeltaCheck	= (int16)(qep1.MechThetaDegree - temp);
		if(abs(MechThetaDeltaCheck) <= 30)
		{
			flagStitch.bit.Counted = 1;		//÷√’Î“—º∆±Í÷æ
			CountStitch ++;					//’Î ˝¿€º”
			GpioDataRegs.GPATOGGLE.bit.GPIO16 = 1;
		}
	}

}//end of StitchCount()


/**
 * pid3_clear
 * 
 * pid3±‰¡ø«Â≥˝
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
