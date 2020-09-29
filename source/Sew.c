/**
 * Copyright (c) 2010, �Ϻ����µ��ӿƼ����޹�˾
 * All rights reserved.
 *
 * @file Sew.c
 * ���ҹ��ܺ��������ļ�.
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

int32 DEGREE_to_PULSE(int32 degree)		//�� -> ����
	{return ((int32)(degree)*ENC_RESOLUTION*4/360);}
int32 RPM_to_PPMS(int32 rpm)			//rpm(Q0)  -> pulse/ms(Q11)
	{return (_IQ11mpyIQX((rpm),0,_IQ30((ENC_RESOLUTION*4)/60/1000),30));}
int32 PPMS_to_RPM(int32 ppms)			//pulse/ms(Q11)  -> rpm(Q0)
	{return (_IQ11toF(ppms)*60*1000/(ENC_RESOLUTION*4));}
int32 RPMPS_to_PPMSPMS(int32 rpmps)		//rpm/s(Q0) -> pulse/ms/ms(Q11)
	{return (_IQ11mpyIQX((rpmps),0,_IQ30((ENC_RESOLUTION*4)/60/1000000),30));}
Uint32 MechThetaDeltaCal(int16 mechtheta_target)	//����ͣ��ƫ��Ƕ� Q32=360
	{return ((Uint32)(_IQ24(mechtheta_target/360.0)<<8) - (Uint32)(qep1.MechTheta<<8));}

extern Uint16 HmiNcCmd;					// ��������

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
 * ���ҹ�����ڳ���
 * 
 * 
 */
void SewProcess(void)
{
	if(ErrorCode != 0)
	{//����ʱ�Ĵ���
		SewState = SEW_STATE_IDLE;
		stop_clear();
		return;
	}

	switch(SewState)
	{//�жϸ������ҹ���
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

	SpeedLimit();				//���������
	SpeedCurve();

}//end of SewProcess()


/**
 * SewPro_Nc
 * 
 * ���Ҳ��빤��
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
	{//ͣ�봦��
		flagPos.bit.PosMode = POS_MODE_NC;
		flagPos.bit.OneStitchStop = 1;
		SewStop_Pos();			//��λ����
		flagPosDetected = 0;
	}
	else
	{//�����ж�
		if((HmiNcCmd == CMD_NC_LIANXU))
		{//�������� ����
			SpeedSel = SpeedPedalMin;
			flagMotorState = MOTOR_STATE_RUN;
		}
		else if((HmiNcCmd == CMD_NC_OFF))
		{//�������� ͣ��
			flagPos.bit.Updn = flagSet.bit.Updn;
			flagPos.bit.PosMode = POS_MODE_NC;
			flagPos.bit.OneStitchStop = 1;
			SewStop_Pos();			//��λ����
		}
		else if((HmiNcCmd == CMD_NC_ON))
		{//������λ�ж�
			SpeedSel = SpeedPedalMin;
			flagMotorState = MOTOR_STATE_RUN;
			if(flagPosDetected == 0)
			{
				flagPosDetected = 1;
				temp1 = (((MechThetaUpSet + 45)>=360)?((MechThetaUpSet + 45)-360):(MechThetaUpSet + 45));
				temp2 = (((MechThetaUpSet + 195)>=360)?((MechThetaUpSet + 195)-360):(MechThetaUpSet + 195));
				if((MechThetaUpSet + 45) >= 360)	//temp1��temp2��0~195du
				{
					if(qep1.MechThetaDegree > temp1 && qep1.MechThetaDegree < temp2)
					{
						//���ڲ���
						flagPosTarget = 0;	//����λͣ��
					}
					else
					{
						//���ڲ���
						flagPosTarget = 1;	//����λͣ��
					}
				}
				else if((MechThetaUpSet + 45) < 360 && (MechThetaUpSet + 195) >= 360)	//��315��360��0��150
				{
					if(((qep1.MechThetaDegree > temp1)&&(qep1.MechThetaDegree <= 360))||\
					  (qep1.MechThetaDegree < temp2))
					{
						//���ڲ���
						flagPosTarget = 0;	//����λͣ��
					}
					else
					{
						//���ڲ���
						flagPosTarget = 1;	//����λͣ��
					}
				}
				else	//temp1��temp2֮��,�͵�һ��һ����Ϊ����ȷ������д
				{
					if(qep1.MechThetaDegree > temp1 && qep1.MechThetaDegree < temp2)
					{
						//���ڲ���
						flagPosTarget = 0;	//����λͣ��
					}
					else
					{
						//���ڲ���
						flagPosTarget = 1;	//����λͣ��
					}
				}
			}
		}
		else
		{//������λ�л�
			flagPos.bit.Updn = flagPosTarget;
			flagPos.bit.PosMode = POS_MODE_NC;
			flagPos.bit.OneStitchStop = 1;
			SewStop_Pos();			//��λ����
		}
	}
}

/**
 * SewPro_Idle
 * 
 * ���Ҵ�������
 * 
 * 
 */
#define DELTA_MECH_DEGREE	25		//�������ͣ��+-����Ƕ��ڶ��������������ͣ��Ĺ���
void SewPro_Idle(void)
{
	Uint16 temp1,temp2 = 0;

	switch(RunCmd)
	{
		case CMD_RUN:
			SewState = SEW_STATE_RUN;	//����������й���
			//SewStopDN_UP_FlagClr();
			break;
		case CMD_SETUP:
			if(flagSew.bit.SewStopPos == 1&&flagSet.bit.Updn == 1&&(SewMode != SEW_MODE_SIMPLE))
			{
				if(MechThetaUpSet < DELTA_MECH_DEGREE)		//��ͣ��λ�ü�ȥ�����С�Ѿ��˵�360�Ժ���(С��)
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
				SewState = SEW_STATE_NC;			//������Ҳ��빤��
			}
			break;
	}//end of switch

	if(SewState == SEW_STATE_IDLE)
	{
		stop_clear();			//��������
	}

}//end of SewPro_Idle()


/**
 * Power_Go2Up
 * 
 * �ϵ�������λ
 * 
 * 
 */
void Power_Go2Up(void)
{
	static Uint16 delay = 0;
	if((ErrorCode != 0) || (flagSet.bit.Go2Up == 0) || \
	   (SewMode == SEW_MODE_SIMPLE) || (flagPower.bit.Go2UpOver == 1))
	{//�й��ϡ������趨ֵ��Ч�����׷졢�ϵ��������,����������λ
		flagPower.bit.Go2UpOver = 1;
	}
/*	else if((flagMotorState == MOTOR_STATE_STOP) && (UP == 0))
	{//��ʼ�ж�: ��UP�ź�Ϊ�͵�ƽ(��������λ),����������λ
		flagPower.bit.Go2UpOver = 1;
		qep1.MechThetaDegree = MechThetaUpSet;	//��е�Ƕ�����
		qep1.MechTheta =  _IQ24(qep1.MechThetaDegree/BASE_THETA);
		flagSew.bit.MechThetaOk = 1;			//���ϵ��е�Ƕ���������־
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
				AngleInit.AngleInitFlag = 1;		//��������
			}
		}
	}
	else if(flagSew.bit.MechThetaOk == 0)
	{//������UP�ź��½���
		stop_clear();
		SpeedSel = 150;				//rpm
		flagMotorState = MOTOR_STATE_RUN;
	}
	else
	{//λ�û�����
		flagPos.bit.Updn = 0;	//��ͣ��
		flagPos.bit.PosMode = POS_MODE_GO2UP;
		flagPos.bit.Step = STEP_PLAST_CAL;		//�Զ�������λ,ֱ������λ�û�����
		SewStop_Pos();			//��λ����
		if(flagMotorState == MOTOR_STATE_STOP)
		{//ͣ�����
			flagPower.bit.Go2UpOver = 1;
		}
	}
	SpeedLimit();				//���������
	SpeedCurve();

}//end of Power_Go2Up()


/**
 * SewPro_Run
 * 
 * �������й���
 * 
 * 
 */
void SewPro_Run(void)
{
	switch(SewMode)
	{
		case SEW_MODE_FREE:
		default:
			SewRun_Free();				//���ɷ촦��
			SewSoftStart();				//����������
			break;
		case SEW_MODE_SIMPLE:
			SewRun_Simple();			//���׷촦��
			break;
	}

}//end of SewPro_Run()


/**
 * SewRun_Free
 * 
 * �������й��� ���ɷ�
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
		flagPos.bit.Updn = 0;					//��ͣ��
		flagSew.bit.SoftStarted = 0;			//�����������־,�´���������Ч
		SewState = SEW_STATE_DN_TO_UP;			//������λ
		stop_clear();
	}
	else
	{
		flagPos.bit.Updn = flagSet.bit.Updn;	//������λ
		if(flagPos.bit.Updn == 0)		//��ͣ��
		{
			flagSew.bit.SoftStarted = 0;			//�����������־,�´���������Ч
		}
		else	//��ͣ��
		{
			flagSew.bit.SoftStarted = 1;			//����������־,�´���������Ч
		}
		flagPos.bit.OneStitchStop = 0;			//��һ����ͣ
		SewStop_Pos();							//��λ����
	}

}//end of SewRun_Free()

Uint16 Astep[10] = {1};
Uint16 Acount = 1;

/**
* SewRun_AutoRun
*
* �Զ��ϻ����ϻ��ٶȣ�����ʱ�䣬ֹͣʱ���ɲ�������
*
*̤��һ�����У��ٲ�һ��ֹͣ
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
	{//����ʱ�Ĵ���
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
				SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
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
				SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//��HMI�ñ�־λ
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
					AngleInit.AngleInitFlag = 1;		//��������
					
					SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
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
					AngleInit.AngleInitFlag = 1;		//��������
					
					SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//��HMI�ñ�־λ
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him ��������ô���������1.hmiֹͣ��2.̤��ֹͣ
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
					if(count > TimeAutoTest)		//����ͣ�ٵ�ʱ��
					{
						SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//��������ʱ��
					{
						FlagStep = STEP_RUN_TO_STOP;
						SpeedSel = 0;
						count = 0;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					flagPos.bit.Updn = flagSet.bit.Updn;	//������λ
					flagPos.bit.OneStitchStop = 0;				//��һ����ͣ
					SewStop_Pos();												//��λ����
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			break;
		}
		case PEDAL_START:		//̤����������ô�а���ֹͣ��̤�������ٲ���ֹͣ
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
					if(count > TimeAutoTest)		//����ͣ�ٵ�ʱ��
					{
						SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//��������ʱ��
					{
						FlagStep = STEP_RUN_TO_STOP;
						SpeedSel = 0;
						count = 0;
					}
				}
				else if(FlagStep == STEP_RUN_TO_STOP)
				{
					flagPos.bit.Updn = flagSet.bit.Updn;	//������λ
					flagPos.bit.OneStitchStop = 0;				//��һ����ͣ
					SewStop_Pos();												//��λ����
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			if(RunCmd == CMD_STOP)
			{
				PedalLockFlag = UNLOCK;		//̤��������ٲ�һ�ξͿ���ֹͣ�ϻ�
			}
			if(PedalLockFlag ==  UNLOCK && RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			break;
		}
		case STOP:
		{
			flagPos.bit.Updn = flagSet.bit.Updn;	//������λ
			flagPos.bit.OneStitchStop = 0;				//��һ����ͣ
			SewStop_Pos();												//��λ����
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				count = 0;
				AtStep = GO_INIT;
				FlagStep = STEP_STOP;
				PedalLockFlag = LOCK;
				hmi_common_vars.age_start = 0;		//��HMI���־λ
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//ȷ��̤����ֹͣ��
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//���������
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
		if((StitchAutoTest - CountStitch - 1) > 5)	//ʣ����������5��
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
	{//����ʱ�Ĵ���
		SewState = SEW_STATE_IDLE;
		stop_clear();
		return;
	}

	StitchCount();	//������
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
				
				SpeedSel = SewAutoRunSpeedLimit_Mode1();	//�Զ������ٶ�
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
				
				SpeedSel = SewAutoRunSpeedLimit_Mode1();	//�Զ������ٶ�
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//��HMI�ñ�־λ
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
					AngleInit.AngleInitFlag = 1;		//��������
					
					SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
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
					AngleInit.AngleInitFlag = 1;		//��������
					
					SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//��HMI�ñ�־λ
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him ��������ô���������1.hmiֹͣ��2.̤��ֹͣ
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
					if(count > TimeAutoTest)		//����ͣ�ٵ�ʱ��
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
					flagPos.bit.Updn = flagSet.bit.Updn;	//������λ
					if(StitchAutoTest == 1)
					{
						flagPos.bit.OneStitchStop = 1;				//һ����ͣ
					}
					else
					{
						flagPos.bit.OneStitchStop = 0;				//��һ����ͣ
					}
					SewStop_Pos();												//��λ����
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			break;
		}
		case PEDAL_START:		//̤����������ô�а���ֹͣ��̤�������ٲ���ֹͣ
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
					if(count > TimeAutoTest)		//����ͣ�ٵ�ʱ��
					{
						SpeedSel = SewAutoRunSpeedLimit_Mode1();	//�Զ������ٶ�
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
					flagPos.bit.Updn = flagSet.bit.Updn;	//������λ
					if(StitchAutoTest == 1)
					{
						flagPos.bit.OneStitchStop = 1;				//һ����ͣ
					}
					else
					{
						flagPos.bit.OneStitchStop = 0;				//��һ����ͣ
					}
					SewStop_Pos();												//��λ����
					if(flagMotorState == MOTOR_STATE_STOP)
					{
						FlagStep = STEP_STOP;
						stop_clear();
					}
				}
			}
			if(RunCmd == CMD_STOP)
			{
				PedalLockFlag = UNLOCK;		//̤��������ٲ�һ�ξͿ���ֹͣ�ϻ�
			}
			if(PedalLockFlag ==  UNLOCK && RunCmd == CMD_RUN)
			{
				AtStep = STOP;
			}
			break;
		}
		case STOP:
		{
			flagPos.bit.Updn = flagSet.bit.Updn;	//������λ
			flagPos.bit.OneStitchStop = 0;				//��һ����ͣ
			SewStop_Pos();												//��λ����
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				count = 0;
				AtStep = GO_INIT;
				FlagStep = STEP_STOP;
				PedalLockFlag = LOCK;
				hmi_common_vars.age_start = 0;		//��HMI���־λ
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//ȷ��̤����ֹͣ��
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//���������
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
	{//����ʱ�Ĵ���
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
				
				SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
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
				
				SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
				flagMotorState = MOTOR_STATE_RUN;
				hmi_common_vars.age_start = 1;		//��HMI�ñ�־λ
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
					AngleInit.AngleInitFlag = 1;		//��������
					
					SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
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
					AngleInit.AngleInitFlag = 1;		//��������
					
					SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
//					stop_clear();
					flagMotorState = MOTOR_STATE_RUN;
					hmi_common_vars.age_start = 1;		//��HMI�ñ�־λ
					AtStep = PEDAL_START;
					
					FlagStep = STEP_RUN;
					count = 0;
					stop_clear();
				}
			}
			break;
		}
		case HMI_START:		//Him ��������ô���������1.hmiֹͣ��2.̤��ֹͣ
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
					if(count > TimeAutoTest)		//����ͣ�ٵ�ʱ��
					{
						SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//��������ʱ��
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
		case PEDAL_START:		//̤����������ô�а���ֹͣ��̤�������ٲ���ֹͣ
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
					if(count > TimeAutoTest)		//����ͣ�ٵ�ʱ��
					{
						SpeedSel = SpeedAutoTest;	//�Զ������ٶ�
						flagMotorState = MOTOR_STATE_RUN;
						FlagStep = STEP_RUN;
						count = 0;
						stop_clear();
					}
				}
				else if(FlagStep == STEP_RUN)
				{
					if(count > TimeAutoRun)			//��������ʱ��
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
				PedalLockFlag = UNLOCK;		//̤��������ٲ�һ�ξͿ���ֹͣ�ϻ�
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
				hmi_common_vars.age_start = 0;		//��HMI���־λ
				stop_clear();
			}
			break;
		}
		case GO_INIT:		//ȷ��̤����ֹͣ��
		{
			if(RunCmd == CMD_STOP)
			{
				AtStep = INIT;
			}
			break;
		}
	}
	SpeedLimit();				//���������
	SpeedCurve();
}

void SewRun_AutoRun_Mode3(void)
{
	static Uint32 Timer = 0;		//��ʱ�������
	
	#define RUN_FEN			15
	#define RUN_TIME		((Uint32)RUN_FEN*60*1000)
	
	//��������ʱ�䣬ֹͣʱ�䣬�����ٶ�
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
	
	SpeedAutoTest = 4000;	//�����ٶ�
	SpeedMax = 4000;
	TimeAutoTest = 1000;		//ͣ��1s
	TimeAutoRun = 1000;		//����1s
	
	SewRun_AutoRun_Mode2();
}

void SewRun_AutoRun(void)
{
	if(AutoTestMode == 0)				//��ͣ���ʱ��ģʽ
	{
		SewRun_AutoRun_Mode0();
	}
	else if(AutoTestMode == 1)	//����ģʽ
	{
		SewRun_AutoRun_Mode1();
	}
	else if(AutoTestMode == 2)	//����ʱ��ģʽ
	{
		SewRun_AutoRun_Mode2();
	}
	else		//����ģʽ
	{
		SewRun_AutoRun_Mode3();
	}
}

/**
 * SewRun_Simple
 * 
 * �������й��� ���׷�
 * 
 * 
 */
void SewRun_Simple(void)
{
	if(RunCmd == CMD_RUN)
	{//����
//		flagSew.bit.Sewed = 1;
		SpeedSel = SpeedPedal;
		flagMotorState = MOTOR_STATE_RUN;
		stop_clear();
	}
	else
	{//ֹͣ(��ͣ��λ����)
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
			flagPos.bit.Updn = 0;			//��ͣ��
			flagSew.bit.SoftStarted = 0;			//�����������־,�´���������Ч
			flagPos.bit.PosMode = POS_MODE_TRIM;
			SewStop_Pos();					//��λ����
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
 * ����ͣ�붨λ�ӳ���
 * Ϊʵ���ԽǶ���Ϊλ��������0~FFFFFFFF��ʾ0~360 (ֻ����һȦ����ͣ��)
 * ���룺flagPos
 * 
 */
void SewStop_Pos(void)
{
	static Uint16 cntDelayVibrate = 0;
	static Uint16 cntDelaySafe = 0;
	static Uint16 SpdStop;					//�����ٶ�
	int32 MechThetaDelta;					//ͣ������û�е�Ƕ�ƫ��			IQ32 pu
	int32 PulseDest;						//Ŀ��������λ����
	Uint16 tempdgree = 0;

//�����ͷ��е���ϵ�δУ����ͣ��
	if(flagSew.bit.MechThetaOk == 0)
	{
		return;
	}

//����Ŀ��λ������
	if(flagPos.bit.Fztz == 1)
	{//��ת����
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
	{//����λ
		PulseDest 		= DEGREE_to_PULSE(MechThetaUpSet);
		MechThetaDelta 	= MechThetaDeltaCal(MechThetaUpSet);
	}
	else
	{//����λ
		tempdgree = ((MechThetaUpSet+MechThetaDnSet)>=360)?(MechThetaUpSet+MechThetaDnSet-360):(MechThetaUpSet+MechThetaDnSet);

		PulseDest 		= DEGREE_to_PULSE(tempdgree);
		MechThetaDelta 	= MechThetaDeltaCal(tempdgree);
	}

//�ֲ���ͣ���������
	switch(flagPos.bit.Step)
	{
		case STEP_START:
			if(flagPos.bit.OneStitchStop == 1)
			{//һ����ͣ ��������
				SpdStop = SpeedPedalMin+100;	//̤�����
			}
			else
			{//��һ����ͣ
				SpdStop = MachinePos.spd_stop;
			}
			flagPos.bit.Step = STEP_SPEED_CHECK;
		case STEP_SPEED_CHECK:
			if(curve1.over >= 1)
			{//������������
				flagPos.bit.Step = STEP_PLAST_ENTRY;
			}
			break;
		case STEP_FZTZ:				//��ת���봦��
			SpeedSel = -150;
			if(_IQ24abs(MechThetaDelta>>8) < _IQ24(5/360.0))
			{//�ظ����ж�
				flagPos.bit.Step = STEP_PLAST_CAL;
				cntDelayVibrate = 0;
				cntDelaySafe = 0;
			}
			break;
		case STEP_PLAST_ENTRY:		//λ�ù滮
			if(((_IQ24abs(MechThetaDelta>>8) < _IQ24(10/360.0)) && (MechThetaDelta > 0)) || \
			   (curve1.times>50))
			{//�ظ����ж�(350~0)
				flagPos.bit.Step = STEP_PLAST_CAL;
				cntDelayVibrate = 0;
				cntDelaySafe = 0;
			}
			break;
		case STEP_PLAST_CAL:		//��λ�û�
			pid1_pos2.Ref = MechThetaDelta;		//�Ŵ���2^8
			pid1_pos2.Fdb = 0;					//�Ŵ���2^8
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
				{//��ת������Ч,�����߷�ת����
					flagPos.bit.Fztz = 1;				//��ת���봦��
					flagPos.bit.Step = STEP_FZTZ;		//ֱ�ӷ�ת����
					pid3_clear(&pid1_pos2);				//��ͣ��λ�û�������
				}
				else
				{//ͣ����̽���
					flagMotorState = MOTOR_STATE_STOP;
					flagSew.bit.SewStopPos = 1;
					stop_clear();			//������Ҵ�������
					return;
				}
			}
			break;
		default:
			break;
	}//end of switch

	if((flagPos.bit.Step == STEP_FZTZ) || (flagPos.bit.Step == STEP_PLAST_CAL))
	{//��ת�����ͣ����λ�û�����
		flagPos.bit.CurveStop = 0;
	}
	else	//�˴�elseע�ͺ�PLASTλ�û���Ч
	{//�������� ��λ�ù滮����
		flagPos.bit.CurveStop = 1;
		Curve_Stop((PulseDest-DEGREE_to_PULSE(MachinePos.thetaRef)),SpdStop,PulseDest,0);
	}

}//end of SewStop_Pos()


/**
 * Curve_Stop
 *
 * ͣ��ʱ��λ�ù滮
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
		{//����ͣ�� ֱ�������ι滮
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
						curve1.state = 1;//�л�״̬
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
	{//�����˶��滮
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
 * �������ӳ���
 * 
 * 
 */
void SewSoftStart(void)
{
	int16 speed1st,speed2nd,speed3rd;
	int16 speedsoft;

//	if(flagPos.bit.Updn == 1)		//��ͣ��
//	{
//		return;
//	}
	if((flagSew.bit.SoftStarted == 0) && \
	   (flagSet.bit.SoftStartEn == 1) && (StitchSoftStart > 0)&&(CountStitch < StitchSoftStart))
	{
		StitchCount();		//������
//		if(CountStitch >= StitchSoftStart)
//		{//�趨��������������������
//			flagSew.bit.SoftStarted = 1;
//		}
//		else
		{
			//ȷ�� speed1st < speed2nd < speed3rd
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
			{//��1��
				speedsoft = speed1st;
			}
			else if(CountStitch == 1)
			{//��2��
				speedsoft = speed2nd;
			}
			else
			{//��3~9��
				speedsoft = speed3rd;
			}

			if(SpeedSel > speedsoft)
			{
				SpeedSel = speedsoft;
			}
		}
	}

}//end of SoftStartSew()


//�����������ݱ�
const Uint16 CURVE_SPEED_STOP_1[10][6] = // ���ٶ�a: [0~3000rpm 200ms] = 250rpss = 900000rpmm
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
const Uint16 CURVE_SPEED_STOP_2[10][6] = // ���ٶ�a: [0~3000rpm 150ms] = 333rpss = 1200000rpmm
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
 * ͣ����ǰ���ٴ���(0~15������)
 * 
 * ���룺stitchleft, SpeedSel, speedmin
 * ����SpeedSel
 */
void SpeedStop(int16 stitchleft, int16 speedmin)
{
	Uint16 speedlimit;
	Uint16 stitchleft_nosign;

	if(stitchleft > 15)
	{//15�����ϲ�������
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
		{//��������Ϊ0
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
 * ����ٶ������ӳ���
 * 
 * ���룺SpeedSel��SpeedMax
 * �����SpeedSet
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
 * �Ӽ������������ӳ���
 * 
 * ���룺SpeedSet
 * �����SpeedRef
 */
void SpeedCurve(void)
{
	rc2.TargetValue = _IQ(SpeedSet/BASE_SPEED);
	rc2.calc(&rc2);
	SpeedRef = _IQtoF(rc2.SetpointValue) * BASE_SPEED;
//	SpeedRef = SpeedSet;

/*----------------------------
��λ�ù滮: SpeedSel->SpeedSet->SpeedRef->SpeedRef_P
  λ�ù滮: SpeedRef_P->SpeedSel->SpeedSet->SpeedRef
----------------------------*/
	if(flagPos.bit.CurveStop == 0)
	{//��ͣ��λ�ù滮
		SpeedRef_P = RPM_to_PPMS(SpeedRef);
	}
	else
	{//ͣ��λ�ù滮
		SpeedSel = PPMS_to_RPM(SpeedRef_P);
	}

}//end of SpeedCurve()


/**
 * stop_clear
 * 
 * ������������ʱ�һЩ״̬��־�ͱ����Ļָ?
 * ���ݲ�ͬ�Ĺ�������Ӧ�Ĵ���
 * 
 */
void stop_clear(void)
{
	static Uint16 flagPidCleared = 0;

	if(flagMotorState == MOTOR_STATE_STOP)
	{
		if(SewState == SEW_STATE_IDLE)
		{//����̬
			SpeedSel = 0;
			
			//��flagSew��ر�־
			flagSew.bit.Sewed = 0;
			CountStitch = -1;				//��������
			flagStitch.bit.Counted = 0;

		}//end of ����
		else
		{//���С����ߡ�����
			SewState = SEW_STATE_IDLE;		//���ش���̬

			if(flagPos.bit.PosMode == POS_MODE_TRIM)
			{//�����߽������
				//��flagSew��ر�־
				flagSew.bit.Sewed = 0;
				CountStitch = -1;				//��������
				flagStitch.bit.Counted = 0;
			}//end of if(flagPos.bit.PosMode == POS_MODE_TRIM)
		}//end of if(SewState == SEW_STATE_IDLE)

		pid3_clear(&pid1_spd);	//���ٶȻ�������
		pid3_clear(&pid1_iq);	//��iq������������
		pid3_clear(&pid1_id);	//��id������������

//�����������
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
		{//����ʱ��һ��
			flagPidCleared = 1;
			pid3_clear(&pid1_spd);	//���ٶȻ�������
			pid3_clear(&pid1_iq);	//��iq�����������
			pid3_clear(&pid1_id);	//��id������������
		}
	}

	flagPos.all = 0;		//����̬��ͣ��������ñ�־
	curve1.run = 0;
	curve1.over = 0;
	pid3_clear(&pid1_pos2);	//��ͣ��λ�û�������


}//end of stop_clear()


/**
 * StitchCount
 * 
 * �������ӳ���
 * 
 * 
 */
void StitchCount(void)
{
	int16 MechThetaDeltaCheck;		//�Ƕȷ�Χ�ж�ֵ ->0(360)<-
	int16 temp = 0;

	if(flagStitch.bit.Counted == 1)
	{//���Ѽ�
		temp = ((MechThetaUpSet + MachinePos.MechThetaStitchClear)>=360)?(MechThetaUpSet + MachinePos.MechThetaStitchClear-360):(MechThetaUpSet + MachinePos.MechThetaStitchClear);
		MechThetaDeltaCheck	= (int16)(qep1.MechThetaDegree - temp);
		if(abs(MechThetaDeltaCheck) <= 30)
		{
			flagStitch.bit.Counted = 0;		//�����ѼƱ�־
		}
	}
	else if(flagMotorState == MOTOR_STATE_RUN)
	{//��δ��������̬
		temp = ((MechThetaUpSet + MachinePos.MechThetaStitchSet)>=360)?(MechThetaUpSet + MachinePos.MechThetaStitchSet-360):(MechThetaUpSet + MachinePos.MechThetaStitchSet);
		MechThetaDeltaCheck	= (int16)(qep1.MechThetaDegree - temp);
		if(abs(MechThetaDeltaCheck) <= 30)
		{
			flagStitch.bit.Counted = 1;		//�����ѼƱ�־
			CountStitch ++;					//�����ۼ�
			GpioDataRegs.GPATOGGLE.bit.GPIO16 = 1;
		}
	}

}//end of StitchCount()


/**
 * pid3_clear
 * 
 * pid3�������
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
