/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
 * All rights reserved.
 *
 * @file MainISR.c
 * 主中断函数
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
 * MainISR中断函数
 * 
 * 
 * 
 * 
 */
interrupt void MainISR(void)
{
	int32 ElecTheta;

	counterMainISR++;				// 中断计数器增加


//采样
	adc_meas1.read(&adc_meas1);	//418 total 2348

//QEP计算
	qep1.calc(&qep1);	//446

//电角度计算
	if(flagThetaTest.bit.TestEn == 1)
	{
		if(flagThetaTest.bit.Step == 0)//测电机初始角 直流阶段
		{
			ElecTheta = _IQ((-90.0+330.0)/360.0);	//THTA_OFFSET 330
		}
		else
		{//转动阶段
			ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(330.0/360.0);	//THTA_OFFSET 330
		}
	}
	else if(AngleInit.AngleInitFlag != 1)		//角度搜寻未完成
	{
		ElecTheta = AngleInit.ElecTheta;
	}
	else 
	{//制动处理
		//非电机初始角模式
		switch(flagBrake)
		{
			case 0://非制动处理
			default:
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta);	//18
				break;
			case 1://制动处理1
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-30.0/360.0);
				break;
			case 2://制动处理2
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-45.0/360.0);
				break;
			case 3://制动处理3
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-60.0/360.0);
				break;
			case 4://制动处理4
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-75.0/360.0);
				break;
			case 5://制动处理5
				ElecTheta = _IQ24toIQ((int32)qep1.ElecTheta) + _IQ(-90.0/360.0);
				break;
		}
	}

//电流变换
	clarke1.As = _IQ15toIQ((int32)adc_meas1.ImeasA);	//7
	clarke1.Bs = _IQ15toIQ((int32)adc_meas1.ImeasB);	//5
	clarke1.calc(&clarke1);	//55

	park1.Alpha = clarke1.Alpha;
	park1.Beta = clarke1.Beta;
	park1.Angle = ElecTheta;
	park1.calc(&park1);		//up4 200

//电流PID
	if(flagThetaTest.bit.TestEn == 0 && AngleInit.AngleInitFlag == 1)
	{//非测电机初始角
		pid1_iq.Ref = pid1_spd.Out + feedforward_torque.Out;
		if(flagFeedForwardDn == 1)
		{//加前馈电流 下针
			if(MotorDir == MOTOR_DIR_POS)
			{//正转
				pid1_iq.Ref = pid1_spd.Out + CurrentFeedForwardDn;
			}
			else
			{//反转
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

//模长限制 电压补偿
	modlmt_dq.X = pid1_id.Out;
	modlmt_dq.Y = pid1_iq.Out;
	modlmt_dq.VdcCoeff = adc_meas1.VdcCoeff;
	modlmt_dq.calc(&modlmt_dq);	//up 213

//电压PARK反变换
	ipark1.Ds = modlmt_dq.Xlimited;
	ipark1.Qs = modlmt_dq.Ylimited;	
	ipark1.Angle = ElecTheta;
	ipark1.calc(&ipark1);	//up 200

//SVPWM生成 输出
	svgen_dq1.Ualpha = ipark1.Alpha;
	svgen_dq1.Ubeta = ipark1.Beta;
	svgen_dq1.calc(&svgen_dq1);	//up 201

	pwm1.MfuncC1 = (int16)_IQtoIQ15(svgen_dq1.Ta); // MfuncC1 is in Q15
	pwm1.MfuncC2 = (int16)_IQtoIQ15(svgen_dq1.Tb); // MfuncC2 is in Q15  
	pwm1.MfuncC3 = (int16)_IQtoIQ15(svgen_dq1.Tc); // MfuncC3 is in Q15
	pwm1.update(&pwm1,ZERO_INT);

//PWM输出使能
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
 * SecondaryISR中断函数
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
