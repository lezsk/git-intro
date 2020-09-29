/**
 * Copyright (c) 2010, �Ϻ����µ��ӿƼ����޹�˾
 * All rights reserved.
 *
 * @file Display.c
 * DHMI��ʾ�����ļ�.
 * 
 * 
 *
 * @author yemj
 * @version 1.0
 * @date 2010-6-17
 *
 */


#include "DSP2803x_Device.h"			// DSP2803x Headerfile Include File
#include "GlobalVars.h"
#include "DMCparameter.h"
#include "DMCvars.h"
#include "build.h"
//#include "E2prom.h"
#include "IO_Def.h"
#include "hmi.h"

extern HMI_Vars hmi_common_vars;

void motortest_THTA(void);
void MotorTest_DCPWM_Enter(void);
void MotorTest_DCPWM_Exit(void);

/**
 * MotorTest
 * 
 * �������
 * 
 * 
 */
void MotorTest(void)
{
	motortest_THTA();

}//end of MotorTest()


/**
 * motortest_THTA
 * 
 * ������� THTA
 * 
 * 
 */
#define THTA_OFFSET			330			// ֱ��λ�õĽǶ�
#define ISQ_REF_DC			5.0			// ͱֱ����С	A
#define TIME_SPDZERO_DC		300			// ����ֱ��ʱ��
#define TIME_MAX_DC			3000		// ֱ���ʱ��
#define ISQ_REF_RUN			1.0			// ��HALL������С	A
#define THTA_STATE_IDLE		0
#define THTA_STATE_TEST		1
#define THTA_STATE_CAL		2
#define THTA_STATE_SAVE		3

void motortest_THTA(void)
{
	static Uint16 state = THTA_STATE_IDLE;		//����״̬
	static Uint16 cntDelayS = 0;
	static Uint16 cntDelayL = 0;
	static Uint16 cmdLocked = 1;				//���������ֹ��־(��ֹ������ͣ�Ĳ���)
	Uint16 sumofTheta;

	modlmt_dq.ModLenMax = _IQ(0.10);			//PWM���ģ������

	flagThetaTest.bit.TestEn = 1;

	switch(state)
	{
		case THTA_STATE_IDLE:	//����
			if((ErrorCode == 0) && (RunCmd == CMD_RUN) && (cmdLocked == 0))
			{
				//CmdHmiRun = 0;
				state = THTA_STATE_TEST;			//������Խ׶�
				flagThetaTest.bit.Tested = 0;
				pid3_clear(&pid1_iq);	//��iq������������
				pid3_clear(&pid1_id);	//��id������������
			}
			else
			{
				flagMotorState = MOTOR_STATE_STOP;
				MotorTest_DCPWM_Exit();
				ModeParaUpdate();
				if(RunCmd != CMD_RUN)
				{
					cmdLocked = 0;
				}
			}
			break;
		case THTA_STATE_TEST:	//������
			if(flagThetaTest.bit.Calc == 1)
			{//���Խ���
				flagThetaTest.bit.Calc = 0;
				flagMotorState = MOTOR_STATE_STOP;
				state = THTA_STATE_CAL;
			}
			else
			{//���Թ���
				if(flagThetaTest.bit.Step == 0)
				{//��ֱ��
					MotorTest_DCPWM_Enter();
					flagMotorState = MOTOR_STATE_RUN;
					pid1_iq.Ref = _IQ(ISQ_REF_DC/BASE_CURRENT);
					cntDelayL++;
					if(SpeedMeas.SpeedRpm==0)
					{
						cntDelayS++;
					}
					else
					{
						cntDelayS = 0;
					}
					if((cntDelayS>TIME_SPDZERO_DC) || (cntDelayL>TIME_MAX_DC))
					{
						MotorTest_DCPWM_Exit();
						cntDelayS = 0;
						cntDelayL = 0;
						qep1.TotalEnc = 0;
						qep1.DeltaEnc = 0;
						qep1.I2cAngle = 0;
						qep1.CalibratedAngle = 0;
						flagThetaTest.bit.Step = 1;	//��HALL_A�׶� ICP
					}
				}
				else
				{//��HALL_A�׶� 200Rpm
					pid1_iq.Ref = _IQ(ISQ_REF_RUN/BASE_CURRENT);
				}
			}
			break;
		case THTA_STATE_CAL:	//������
			while(ThetaTest1st >= 360)
			{
				ThetaTest1st -= 360;
			}
			while(ThetaTest2nd >= 360)
			{
				ThetaTest2nd -= 360;
			}
			while(ThetaTest3rd >= 360)
			{
				ThetaTest3rd -= 360;
			}
			sumofTheta = ThetaTest1st;
			sumofTheta += ThetaTest2nd;
			sumofTheta += ThetaTest3rd;
			ThetaTest = sumofTheta/3;
			if((abs(ThetaTest-ThetaTest1st)>12) || (abs(ThetaTest-ThetaTest2nd)>12) || \
			   (abs(ThetaTest-ThetaTest3rd)>12))// || (ThetaTest>=360))
			{//ƫ��̫��
		//		ErrorCode = ERR_TSTTHTA0;		//���� (������������Զ�β���)
		//		flagThetaTest.bit.Error = 1;
				state = THTA_STATE_IDLE;		//���ش���
				cmdLocked = 1;
			}
			else
			{//���Խ���
				ThetaTest += THTA_OFFSET;
				while(ThetaTest >= 360)
				{//����ƫ����
					ThetaTest -= 360;
				}
				I2cAngle = ThetaTest;
				state = THTA_STATE_SAVE;		//������
				
				//===============yuming================
				hmi_common_vars.test_angle = 0;
				hmi_common_vars.motor_agl = ThetaTest;
				//=====================================
		//		if(MotorThtaMode == MOTOR_THTA_ENABLE)
		//		{// ��Thta0�Ƕȴ浽����������洢оƬ
		//			flagMotorWrite.bit.Write = 1;
		//		}
			}
			break;
		case THTA_STATE_SAVE:	//��������
		//	if(((MotorThtaMode == MOTOR_THTA_ENABLE) && (flagMotorWrite.bit.Write==0)) || \
		//	   (MotorThtaMode == MOTOR_THTA_DISABLE))
			{//�������
				flagThetaTest.bit.Tested = 1;
				state = THTA_STATE_IDLE;		//���ش���
				cmdLocked = 1;
			}
			break;
		default:
			break;
	}//end of switch(state)
}//end of motortest_THTA()

void motortest_THTA_old(void)
{
	static Uint16 state = THTA_STATE_IDLE;		//����״̬
	static Uint16 cntDelayS = 0;
	static Uint16 cntDelayL = 0;
	static Uint16 cmdLocked = 1;	//���������ֹ��־(��ֹ������ͣ�Ĳ���)
	Uint16 sumofTheta;

	modlmt_dq.ModLenMax = _IQ(0.10);			//PWM���ģ������
	
	flagThetaTest.bit.TestEn = 1;
	
	switch(state)
	{
		case THTA_STATE_IDLE:	//����
			if((ErrorCode == 0) && (RunCmd == CMD_RUN) && (cmdLocked == 0))
			{
				flagMotorState = MOTOR_STATE_RUN;
				state = THTA_STATE_TEST;			//������Խ׶�
				flagThetaTest.bit.Tested = 0;
			}
			else
			{
				flagMotorState = MOTOR_STATE_STOP;
				if(RunCmd != CMD_RUN)
				{
					cmdLocked = 0;
				}
			}
			break;
		case THTA_STATE_TEST:	//������
			if(flagThetaTest.bit.Calc == 1)
			{//���Խ���
				flagThetaTest.bit.Calc = 0;
				flagMotorState = MOTOR_STATE_STOP;
				state = THTA_STATE_CAL;
			}
			else
			{//���Թ���
				if(flagThetaTest.bit.Step == 0)
				{//��ֱ��
					pid1_iq.Ref = _IQ(ISQ_REF_DC/BASE_CURRENT);
					cntDelayL++;
					if(SpeedMeas.SpeedRpm==0)
					{
						cntDelayS++;
					}
					if((cntDelayS>TIME_SPDZERO_DC) || (cntDelayL>TIME_MAX_DC))
					{
						cntDelayS = 0;
						cntDelayL = 0;
						qep1.TotalEnc = 0;
						qep1.DeltaEnc = 0;
						qep1.I2cAngle = 0;
						qep1.CalibratedAngle = 0;
						flagThetaTest.bit.Step = 1;	//��HALL_A�׶� ICP
					}
				}
				else
				{//��HALL_A�׶� 200Rpm
					pid1_iq.Ref = _IQ(ISQ_REF_RUN/BASE_CURRENT);
				}
			}
			break;
		case THTA_STATE_CAL:	//������
			sumofTheta = ThetaTest1st;
			sumofTheta += ThetaTest2nd;
			sumofTheta += ThetaTest3rd;
			ThetaTest = sumofTheta/3;
			if((abs(ThetaTest-ThetaTest1st)>12) || (abs(ThetaTest-ThetaTest2nd)>12) || \
			   (abs(ThetaTest-ThetaTest3rd)>12) || (ThetaTest>=360))
			{//ƫ��̫��
				ErrorCode = ERR_TSTTHTA0;		//���� (������������Զ�β���)
				flagThetaTest.bit.Error = 1;
				state = THTA_STATE_IDLE;		//���ش���
				cmdLocked = 1;
			}
			else
			{//���Խ���
			/*	if(MotorThtaMode == MOTOR_THTA_ENABLE)
				{// ��Thta0�Ƕȴ浽����������洢оƬ
					flagMotorWrite.bit.Write = 1;
				}*/
				state = THTA_STATE_SAVE;		//������
				I2cAngle = ThetaTest;
				//===============yuming================
				hmi_common_vars.test_angle = 0;
				hmi_common_vars.motor_agl = ThetaTest;
				//=====================================
			}
			break;
		case THTA_STATE_SAVE:	//��������
		//	if(((MotorThtaMode == MOTOR_THTA_ENABLE) && (flagMotorWrite.bit.Write==0)) || \
		//	   (MotorThtaMode == MOTOR_THTA_DISABLE))
			{//�������
				flagThetaTest.bit.Tested = 1;
				state = THTA_STATE_IDLE;		//���ش���
				cmdLocked = 1;
			}
			break;
		default:
			break;
	}//end of switch(state)

}//end of motortest_THTA()

/**
 * 280xEPWMģ���ʼ������(��ֱ����) ����
 * up/down����ģʽ��PWM�͵�ƽ��Ч
 * A+(ACTIVE HIGH),B- (FORCE LOW),OTHERS(FORCE HIGH)
 */
void MotorTest_DCPWM_Enter(void)
{
//************************************************
//	EPWM Module 1 config
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;	// Sync disable
//	Set actions
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;			// set actions for ePWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
	EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
	EPwm1Regs.AQCTLB.bit.CAU = AQ_SET;				// set actions for ePWM1B
	EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
	EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm1Regs.AQCTLB.bit.PRD = AQ_SET;
//	Set dead-times
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;		// disable Dead-band module
//************************************************
//	EPWM Module 2 config
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;	// Sync disable
//	Set actions
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;				// set actions for ePWM2A
	EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;
	EPwm2Regs.AQCTLA.bit.PRD = AQ_SET;
	EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;				// set actions for ePWM2B
	EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;
	EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
	EPwm2Regs.AQCTLB.bit.PRD = AQ_CLEAR;
//	Set dead-time
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;		// disable Dead-band module
//************************************************
//	EPWM Module 3 config
//	Setup counter mode
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;	// Sync disable
//	Set actions
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;				// set actions for ePWM3A
	EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
	EPwm3Regs.AQCTLA.bit.PRD = AQ_SET;
	EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;				// set actions for ePWM3B
	EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;
	EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm3Regs.AQCTLB.bit.PRD = AQ_SET;
//	Set dead-time
	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;		// disable Dead-band module

}//end of MotorTest_DCPWM_Enter()


/**
 * 280xEPWMģ���ʼ������(��ֱ����) �˳�
 * up/down����ģʽ��PWM�͵�ƽ��Ч
 * 
 */
void MotorTest_DCPWM_Exit(void)
{
//************************************************
//	EPWM Module 1 config
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;		// Sync down-stream module
//	Set actions
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;				// set actions for ePWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
	EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
	EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;			// set actions for ePWM1B
	EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
	EPwm1Regs.AQCTLB.bit.ZRO = AQ_NO_ACTION;
	EPwm1Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;
//	Set dead-time
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;	// enable Dead-band module
//************************************************
//	EPWM Module 2 config
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;		// Sync flow-through
//	Set actions
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;				// set actions for ePWM2A
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm2Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
	EPwm2Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
	EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;			// set actions for ePWM2B
	EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;
	EPwm2Regs.AQCTLB.bit.ZRO = AQ_NO_ACTION;
	EPwm2Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;
//	Set dead-time
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;	// enable Dead-band module
//************************************************
//	EPWM Module 3 config
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;		// Sync flow-through
//	Set actions
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;				// set actions for ePWM3A
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
	EPwm3Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
	EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;			// set actions for ePWM3B
	EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;
	EPwm3Regs.AQCTLB.bit.ZRO = AQ_NO_ACTION;
	EPwm3Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;
//	Set dead-time
	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;	// enable Dead-band module

}//end of MotorTest_DCPWM_Exit()

//===========================================================================
// No more.
//===========================================================================
