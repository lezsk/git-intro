/**
 * Copyright (c) 2010, �Ϻ����µ��ӿƼ����޹�˾
 * All rights reserved.
 *
 * @file MainISR.c
 * ���жϺ���
 * 
 * 
 * 
 * 
 *
 * @author yemj
 * @version 0.1
 * @date 2010-3-12
 *
 */


#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "IQmathLib.h"

#include "build.h"
#include "Defines.h"
#include "IO_Def.h"
#include "DMCparameter.h"
#include "DMCobjects.h"
#include "DMCvars.h"
#include "GlobalVars.h"

#if (BUILD_MODE == FLASH_MODE)
	#pragma CODE_SECTION(MainISR, "ramfuncs");
	#pragma CODE_SECTION(SecondaryISR, "ramfuncs");
#endif

extern void SCI_RxData(void);
extern void SCI_TxData(void);



/**
 * MainISR�жϺ���
 * 
 * 
 * 
 * 
 */
interrupt void MainISR(void)
{
	int32 ElecTheta;

	counterMainISR++;				// �жϼ���������


//����
	adc_meas1.read(&adc_meas1);	//418 total 2348

//QEP����
	qep1.calc(&qep1);	//446

//��Ƕȼ���
	if(flagThetaTest.bit.TestEn == 1)
	{
		if(flagThetaTest.bit.Step == 0)//������ʼ�� ֱ���׶�
		{
			ElecTheta = _IQ((-90.0+330.0)/360.0);	//THTA_OFFSET 330
		}
		else
		{//ת���׶�
			ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(330.0/360.0);	//THTA_OFFSET 330
		}
	}
	else if(AngleInit.AngleInitFlag != 1)		//�Ƕ���Ѱδ���
	{
		ElecTheta = AngleInit.ElecTheta;
	}
	else 
	{//�ƶ�����
		//�ǵ����ʼ��ģʽ
		switch(flagBrake)
		{
			case 0://���ƶ�����
			default:
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta);	//18
				break;
			case 1://�ƶ�����1
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-30.0/360.0);
				break;
			case 2://�ƶ�����2
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-45.0/360.0);
				break;
			case 3://�ƶ�����3
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-60.0/360.0);
				break;
			case 4://�ƶ�����4
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-75.0/360.0);
				break;
			case 5://�ƶ�����5
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-90.0/360.0);
				break;
		}
	}

//�����任
	clarke1.As = _IQ15toIQ((int32)adc_meas1.ImeasA);	//7
	clarke1.Bs = _IQ15toIQ((int32)adc_meas1.ImeasB);	//5
	clarke1.calc(&clarke1);	//55

	park1.Alpha = clarke1.Alpha;
	park1.Beta = clarke1.Beta;
	park1.Angle = ElecTheta;
	park1.calc(&park1);		//up4 200

//����PID
	if(flagThetaTest.bit.TestEn == 0 && AngleInit.AngleInitFlag == 1)
	{//�ǲ�����ʼ��
		pid1_iq.Ref = pid1_spd.Out + feedforward_torque.Out;
		if(flagFeedForwardDn == 1)
		{//��ǰ������ ����
			if(MotorDir == MOTOR_DIR_POS)
			{//��ת
				pid1_iq.Ref = pid1_spd.Out + CurrentFeedForwardDn;
			}
			else
			{//��ת
				pid1_iq.Ref = pid1_spd.Out - CurrentFeedForwardDn;
			}
		}
		pid1_iq.Ref = _IQsat(pid1_iq.Ref, pid1_spd.OutMax, pid1_spd.OutMin);		/* Saturate the output */
	}
	pid1_id.Ref = _IQ(0);
	pid1_iq.Fdb = park1.Qs;
	pid1_id.Fdb = park1.Ds;
	if(AngleInit.AngleInitFlag == 2)
	{
		pid1_id.Ref = AngleInit.Id_Ref;
		pid1_iq.Ref = 0;
	}
	pid1_iq.calc(&pid1_iq);	//up 190
	pid1_id.calc(&pid1_id);	//up 151

//ģ������ ��ѹ����
	modlmt_dq.X = pid1_id.Out;
	modlmt_dq.Y = pid1_iq.Out;
	modlmt_dq.VdcCoeff = adc_meas1.VdcCoeff;
	modlmt_dq.calc(&modlmt_dq);	//up 213

//��ѹPARK���任
	ipark1.Ds = modlmt_dq.Xlimited;
	ipark1.Qs = modlmt_dq.Ylimited;	
	ipark1.Angle = ElecTheta;
	ipark1.calc(&ipark1);	//up 200

//SVPWM���� ���
	svgen_dq1.Ualpha = ipark1.Alpha;
	svgen_dq1.Ubeta = ipark1.Beta;
	svgen_dq1.calc(&svgen_dq1);	//up 201

	pwm1.MfuncC1 = (int16)_IQtoIQ15(svgen_dq1.Ta); // MfuncC1 is in Q15
	pwm1.MfuncC2 = (int16)_IQtoIQ15(svgen_dq1.Tb); // MfuncC2 is in Q15  
	pwm1.MfuncC3 = (int16)_IQtoIQ15(svgen_dq1.Tc); // MfuncC3 is in Q15
	pwm1.update(&pwm1,ZERO_INT);

//PWM���ʹ��
	drv1.EnableFlag = flagMotorState;
	drv1.update(&drv1);	//up 58


	adc_meas1.read(&adc_meas1);

/*---- Prepare to exit the ISR -----*/
// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;
//	EPwm2Regs.ETCLR.bit.INT = 1;
// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;	//up 14

}


/**
 * SecondaryISR�жϺ���
 * 
 * 
 * 
 * 
 */
interrupt void SecondaryISR(void)
{
	pwm1.update(&pwm1,PERIOD_INT);

	EPwm2Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


//===========================================================================
// No more.
//===========================================================================
