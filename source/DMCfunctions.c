/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
 * All rights reserved.
 *
 * @file DMCfunctions.c
 * Digital Motor Control函数文件.
 *
 *
 *
 *
 *
 * @author yemj
 * @version 0.1
 * @date 2010-3-10
 *
 */

#include "DMCobjects.h"
#include "DMCparameter.h"
#include "build.h"
#include "IO_Def.h"
#include "f2803xbmsk.h"
#include "GlobalVars.h"
#include "Defines.h"
#include "math.h"


#if	(BUILD_MODE == FLASH_MODE)
	#pragma CODE_SECTION(clarke_calc, "ramfuncs");
	#pragma CODE_SECTION(park_calc, "ramfuncs");
	#pragma CODE_SECTION(ipark_calc, "ramfuncs");
	#pragma CODE_SECTION(modlmt_calc, "ramfuncs");
	#pragma CODE_SECTION(svgendq_calc, "ramfuncs");
	#pragma CODE_SECTION(pid_reg3_calc, "ramfuncs");
	#pragma CODE_SECTION(PWM_limit, "ramfuncs");
	#pragma CODE_SECTION(F2803X_PWM_Update, "ramfuncs");
	#pragma CODE_SECTION(F2803X_adc_meas_read, "ramfuncs");
	#pragma CODE_SECTION(F2803X_QEP_Calc, "ramfuncs");

#endif

Uint16 PWM_limit(Uint16 PwmCmp,Uint16 TypeFlag);

/**
 * Clarke变换函数
 * abc三相坐标 变换为 Alpha Beta两相静止坐标
 * 输入：	v->As 		IQ
 *			v->Bs		IQ
 * 输出：	v->Alpha	IQ
 *			v->Beta		IQ
 */
void clarke_calc(CLARKE *v)
{
	v->Alpha = v->As;
 										// 1/sqrt(3) = 0.57735026918963
	v->Beta = _IQmpy((v->As + _IQmpy(_IQ(2),v->Bs)),_IQ(0.57735026918963));
}


/**
 * Park变换函数
 * Alpha Beta静止坐标 变换为 DQ旋转坐标
 * 输入：	v->Angle 	IQ
 *			v->Alpha	IQ
 *			v->Beta		IQ
 * 输出：	v->Ds		IQ
 *			v->Qs		IQ
 */
void park_calc(PARK *v)
{
	_iq Cosine,Sine;

//	Using look-up IQ sine table
	Sine = _IQsinPU(v->Angle);
	Cosine = _IQcosPU(v->Angle);

	v->Ds = _IQmpy(v->Alpha,Cosine) + _IQmpy(v->Beta,Sine);
	v->Qs = _IQmpy(v->Beta,Cosine) - _IQmpy(v->Alpha,Sine);
}


/**
 * 反Park变换函数
 * DQ旋转坐标 变换为 Alpha Beta静止坐标
 * 输入：	v->Angle 	IQ
 *			v->Ds		IQ
 *			v->Qs		IQ
 * 输出：	v->Alpha	IQ
 *			v->Beta		IQ
 */
void ipark_calc(IPARK *v)
{
	_iq Cosine,Sine;

//	Using look-up IQ sine table
	Sine = _IQsinPU(v->Angle);
	Cosine = _IQcosPU(v->Angle);

	v->Alpha = _IQmpy(v->Ds,Cosine) - _IQmpy(v->Qs,Sine);
	v->Beta = _IQmpy(v->Qs,Cosine) + _IQmpy(v->Ds,Sine);
}


/**
 * 模长限制和电压补偿函数
 *
 * 输入：	v->X 			IQ
 *			v->Y			IQ
 *			v->VdcCoeff		IQ
 *			v->ModLenMax	IQ
 * 输出：	v->Xlimited		IQ
 *			v->Ylimited		IQ
 */
void modlmt_calc(MODLMT *v)
{
	_iq k,sqrX,sqrY;

// Vdc Coeff
//	v->X = _IQmpy(v->X,v->VdcCoeff);
//	v->Y = _IQmpy(v->Y,v->VdcCoeff);

// calculate the actual modulus length
	sqrX = _IQmpy(v->X,v->X);
	sqrY = _IQmpy(v->Y,v->Y);
	v->ModLen = _IQsqrt(sqrX+sqrY);

// modulus length max limt and x&y axis limit
	if (v->ModLen > v->ModLenMax)
	{
		k = _IQdiv(v->ModLenMax,v->ModLen);		// scaler
		v->Xlimited = _IQmpy(v->X,k);
		v->Ylimited = _IQmpy(v->Y,k);
	}
	else
	{
		v->Xlimited = v->X;
		v->Ylimited = v->Y;
	}
}


/**
 * SVPWM比较器时间计算函数
 * 根据alpha beta电压计算时间，并限制最大最小脉宽
 * 输入：	v->Ualpha 		IQ
 *			v->Ubeta		IQ
 *			v->Tlimit 		IQ
 * 输出：	v->Ta			IQ(-1)~IQ(1)
 *			v->Tb			IQ(-1)~IQ(1)
 *			v->Tc			IQ(-1)~IQ(1)
 */
void svgendq_calc(SVGENDQ *v)
{
	_iq X,Y,Z,t1,t2;
	_iq sqrt3Ualpha;
	Uint32 Sector = 0;  // Sector is treated as Q0 - independently with global Q

	sqrt3Ualpha = _IQmpy(_IQ(1.7320508),v->Ualpha);	// 1.7320508 =  tan(60) = sqrt(3)

// 60 degree Sector determination
	if (v->Ubeta >= _IQ(0))
	{//Sector 123
		if (v->Ualpha >= _IQ(0))
		{//Sector 12
			if (v->Ubeta <= sqrt3Ualpha)
			{
				Sector = 1;						// 0~60
			}
			else
			{
				Sector = 2;						// 60~120
			}
		}
		else
		{//Sector 23
			if ((v->Ubeta + sqrt3Ualpha) >= _IQ(0))
			{
				Sector = 2;						// 60~120
			}
			else
			{
				Sector = 3;						// 120~180
			}
		}
	}
	else
	{//Sector 456
		if (v->Ualpha <= _IQ(0))
		{//Sector 45
			if (v->Ubeta >= sqrt3Ualpha)
			{
				Sector = 4;						// 180~240
			}
			else
			{
				Sector = 5;						// 240~300
			}
		}
		else
		{//Sector 56
			if ((v->Ubeta + sqrt3Ualpha) <= _IQ(0))
			{
				Sector = 5;						// 240~300
			}
			else
			{
				Sector = 6;						// 300~0
			}
		}
	}

// X,Y,Z (Va,Vb,Vc) calculations
	X = v->Ubeta;
	Y = (v->Ubeta + sqrt3Ualpha) >> 1;
	Z = (v->Ubeta - sqrt3Ualpha) >> 1;

	switch (Sector)
	{
	case 1:  		// Sector 1: t1=-Z and t2=X		(abc ---> Ta,Tb,Tc)
		t1 = -Z;
		t2 = X;
		if ((t1 + t2) > v->Tlimit)
		{
			t1 = _IQdiv(_IQmpy(t1,v->Tlimit),(t1 + t2));
			t2 = _IQdiv(_IQmpy(t2,v->Tlimit),(t1 + t2));
		}
		v->Ta = (_IQ(1)-t1-t2) >> 1;				// taon = (1-t1-t2)/2
		v->Tb = v->Ta + t1;							// tbon = taon+t1
		v->Tc = v->Tb + t2;							// tcon = tbon+t2
		break;
	case 2:  		// Sector 2: t1=Z and t2=Y		(abc ---> Tb,Ta,Tc)
		t1 = Z;
		t2 = Y;
		if ((t1 + t2) > v->Tlimit)
		{
			t1 = _IQdiv(_IQmpy(t1,v->Tlimit),(t1 + t2));
			t2 = _IQdiv(_IQmpy(t2,v->Tlimit),(t1 + t2));
		}
		v->Tb = (_IQ(1)-t1-t2) >> 1;				// taon = (1-t1-t2)/2
		v->Ta = v->Tb + t1;							// tbon = taon+t1
		v->Tc = v->Ta + t2;							// tcon = tbon+t2
		break;
	case 3:  		// Sector 3: t1=X and t2=-Y		(abc ---> Tb,Tc,Ta)
		t1 = X;
		t2 = -Y;
		if ((t1 + t2) > v->Tlimit)
		{
			t1 = _IQdiv(_IQmpy(t1,v->Tlimit),(t1 + t2));
			t2 = _IQdiv(_IQmpy(t2,v->Tlimit),(t1 + t2));
		}
		v->Tb = (_IQ(1)-t1-t2) >> 1;				// taon = (1-t1-t2)/2
		v->Tc = v->Tb + t1;							// tbon = taon+t1
		v->Ta = v->Tc + t2;							// tcon = tbon+t2
		break;
	case 4:  		// Sector 4: t1=-X and t2=Z		(abc ---> Tc,Tb,Ta)
		t1 = -X;
		t2 = Z;
		if ((t1 + t2) > v->Tlimit)
		{
			t1 = _IQdiv(_IQmpy(t1,v->Tlimit),(t1 + t2));
			t2 = _IQdiv(_IQmpy(t2,v->Tlimit),(t1 + t2));
		}
		v->Tc = (_IQ(1)-t1-t2) >> 1;				// taon = (1-t1-t2)/2
		v->Tb = v->Tc + t1;							// tbon = taon+t1
		v->Ta = v->Tb + t2;							// tcon = tbon+t2
		break;
	case 5:  		// Sector 5: t1=-Y and t2=-Z	(abc ---> Tc,Ta,Tb)
		t1 = -Y;
		t2 = -Z;
		if ((t1 + t2) > v->Tlimit)
		{
			t1 = _IQdiv(_IQmpy(t1,v->Tlimit),(t1 + t2));
			t2 = _IQdiv(_IQmpy(t2,v->Tlimit),(t1 + t2));
		}
		v->Tc = (_IQ(1)-t1-t2) >> 1;				// taon = (1-t1-t2)/2
		v->Ta = v->Tc + t1;							// tbon = taon+t1
		v->Tb = v->Ta + t2;							// tcon = tbon+t2
		break;
	case 6:  		// Sector 6: t1=Y and t2=-X		(abc ---> Ta,Tc,Tb)
		t1 = Y;
		t2 = -X;
		if ((t1 + t2) > v->Tlimit)
		{
			t1 = _IQdiv(_IQmpy(t1,v->Tlimit),(t1 + t2));
			t2 = _IQdiv(_IQmpy(t2,v->Tlimit),(t1 + t2));
		}
		v->Ta = (_IQ(1)-t1-t2) >> 1;				// taon = (1-t1-t2)/2
		v->Tc = v->Ta + t1;							// tbon = taon+t1
		v->Tb = v->Tc + t2;							// tcon = tbon+t2
		break;
	default:
		break;
	} //end of switch

// Convert the unsigned GLOBAL_Q format (ranged (0,1)) -> signed GLOBAL_Q format (ranged (-1,1))
	v->Ta = (v->Ta - _IQ(0.5)) << 1;
	v->Tb = (v->Tb - _IQ(0.5)) << 1;
	v->Tc = (v->Tc - _IQ(0.5)) << 1;

}


/**
 * 280xEPWM模块初始化函数
 * up/down计数模式，PWM低电平有效，有死区
 * 输入：	p->PeriodMax
 */
void F2803X_PWM_Init(PWMGEN *p)
{
//************************************************
//	EPWM Module 1 config
//	Setup TBCLK
	EPwm1Regs.TBPRD = p->PeriodMax;					// Set period TBCLKs
	EPwm1Regs.TBPHS.half.TBPHS = 0x0000;			// Phase is 0
	EPwm1Regs.TBCTR = 0x0000;						// Clear counter
//	Set Compare values
	EPwm1Regs.CMPA.half.CMPA = p->PeriodMax>>1;		//Set compare A value
//	Setup counter mode
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;	// Count up-down, Symmetrical mode
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;			// Disable phase loading, Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;		// Sync down-stream module
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		// Clock ratio to SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
//	Setup shadowing
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
//	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;	// Load on Zero
//	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;	//下溢和周期都更新占空比
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
//	Set actions
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;				// set actions for ePWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//	Set dead-time
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;	// enable Dead-band module
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;		// Active Lo complementary
	EPwm1Regs.DBFED = SVPWM_DEADTIME_SET;			// deadtime TBCLKs
	EPwm1Regs.DBRED = SVPWM_DEADTIME_SET;			// deadtime TBCLKs
//	Interrupt where we will change the Compare Values
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;		// Select INT on Zero event
	EPwm1Regs.ETSEL.bit.INTEN = 1;					// Enable EPWM1INT generation
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;				// Generate INT on 1st event
	EPwm1Regs.ETCLR.bit.INT = 1;					// Enable more interrupts

	EPwm1Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL	= 1;		// Enable event time-base counter equal to zero
    EPwm1Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event
//************************************************
//	EPWM Module 2 config
//	Setup TBCLK
	EPwm2Regs.TBPRD = p->PeriodMax;					// Set period TBCLKs
	EPwm2Regs.TBPHS.half.TBPHS = 0x0000;			// Phase is 0
	EPwm2Regs.TBCTR = 0x0000;						// Clear counter
//	Set Compare values
	EPwm2Regs.CMPA.half.CMPA = p->PeriodMax>>1;		// Set compare A value
//	Setup counter mode
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;	// Count up-down, Symmetrical mode
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;			// Enable phase loading, Slave module
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;		// Sync flow-through
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		// Clock ratio to SYSCLKOUT
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
//	Setup shadowing
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
//	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;	// Load on Zero
//	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;	//下溢和周期都更新占空比
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
//	Set actions
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;				// set actions for ePWM2A
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//	Set dead-time
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;	// enable Dead-band module
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;		// Active Lo complementary
	EPwm2Regs.DBFED = SVPWM_DEADTIME_SET;			// deadtime TBCLKs
	EPwm2Regs.DBRED = SVPWM_DEADTIME_SET;			// deadtime TBCLKs
//	Interrupt where we will change the Compare Values
	EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;		// Select INT on Period event
	EPwm2Regs.ETSEL.bit.INTEN = 1;					// Enable EPWM2INT generation
	EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;				// Generate INT on 1st event
	EPwm2Regs.ETCLR.bit.INT = 1;					// Enable more interrupts

//	EPwm2Regs.ETSEL.bit.SOCBEN	= 0;		// Enable SOC on A group
//  EPwm2Regs.ETSEL.bit.SOCBSEL	= 2;		// Enable event time-base counter equal to period
//  EPwm2Regs.ETPS.bit.SOCBPRD 	= 1;		// Generate pulse on 1st event
//************************************************
//	EPWM Module 3 config
//	Setup TBCLK
	EPwm3Regs.TBPRD = p->PeriodMax;					// Set period TBCLKs
	EPwm3Regs.TBPHS.half.TBPHS = 0x0000;			// Phase is 0
	EPwm3Regs.TBCTR = 0x0000;						// Clear counter
//	Set Compare values
	EPwm3Regs.CMPA.half.CMPA = p->PeriodMax>>1;		// Set compare A value
//	Setup counter mode
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;	// Count up-down, Symmetrical mode
	EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;			// Enable phase loading, Slave module
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;		// Sync flow-through
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		// Clock ratio to SYSCLKOUT
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
//	Setup shadowing
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
//	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;	// Load on Zero
//	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;	//下溢和周期都更新占空比
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
//	Set actions
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;			// set actions for ePWM3A
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//	Set dead-time
	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;	// enable Dead-band module
	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;		// Active Lo complementary
	EPwm3Regs.DBFED = SVPWM_DEADTIME_SET;			// deadtime TBCLKs
	EPwm3Regs.DBRED = SVPWM_DEADTIME_SET;			// deadtime TBCLKs
//************************************************
//	Trip Zone setup
	EALLOW;                       // Enable EALLOW

   // Enable TZ1 as one shot trip sources
	EPwm1Regs.TZSEL.bit.OSHT1 = 1;
	EPwm2Regs.TZSEL.bit.OSHT1 = 1;
	EPwm3Regs.TZSEL.bit.OSHT1 = 1;

   // What do we want the TZ1 and TZ2 to do?
	EPwm1Regs.TZCTL.bit.TZA = TZ_HIZ;
	EPwm1Regs.TZCTL.bit.TZB = TZ_HIZ;
	EPwm2Regs.TZCTL.bit.TZA = TZ_HIZ;
	EPwm2Regs.TZCTL.bit.TZB = TZ_HIZ;
	EPwm3Regs.TZCTL.bit.TZA = TZ_HIZ;
	EPwm3Regs.TZCTL.bit.TZB = TZ_HIZ;

   // Enable EPWM1_TZINT interrupt
	EPwm1Regs.TZEINT.bit.OST = 1;

	EDIS;                         // Disable EALLOW

}


/**
 * SVPWM模块CMPR值更新函数
 *
 * 输入：	p->PeriodMax
 * 			p->MfuncPeriod
 * 			p->MfuncC1		IQ(-1)~IQ(1)
 * 			p->MfuncC2		IQ(-1)~IQ(1)
 * 			p->MfuncC3		IQ(-1)~IQ(1)
 */
void F2803X_PWM_Update(PWMGEN *p,Uint16 TypeFlag)
{
	int16 MPeriod;
	int32 Tmp;
	static Uint16 PwmCmp[3] = {0,0,0};

	if(TypeFlag == ZERO_INT)
	{
		// Compute the timer period (Q0) from the period modulation input (Q15)
		Tmp = (int32)p->PeriodMax*(int32)p->MfuncPeriod;           // Q15 = Q0*Q15
		MPeriod = (int16)(Tmp>>16) + (int16)(p->PeriodMax>>1);     // Q0 = (Q15->Q0)/2 + (Q0/2)

		EPwm1Regs.TBPRD = MPeriod;
		EPwm2Regs.TBPRD = MPeriod;
		EPwm3Regs.TBPRD = MPeriod;

		// Compute the compare A (Q0) from the EPWM1AO & EPWM1BO duty cycle ratio (Q15)
		Tmp = (int32)MPeriod*(int32)p->MfuncC1;                    // Q15 = Q0*Q15
		PwmCmp[0] = (int16)(Tmp>>16) + (int16)(MPeriod>>1);   // Q0 = (Q15->Q0)/2 + (Q0/2)

		// Compute the compare B (Q0) from the EPWM2AO & EPWM2BO duty cycle ratio (Q15)
		Tmp = (int32)MPeriod*(int32)p->MfuncC2;                   // Q15 = Q0*Q15
		PwmCmp[1] = (int16)(Tmp>>16) + (int16)(MPeriod>>1);  // Q0 = (Q15->Q0)/2 + (Q0/2)	

		// Compute the compare C (Q0) from the EPWM3AO & EPWM3BO duty cycle ratio (Q15)
		Tmp = (int32)MPeriod*(int32)p->MfuncC3;                   // Q15 = Q0*Q15
		PwmCmp[2] = (int16)(Tmp>>16) + (int16)(MPeriod>>1);  // Q0 = (Q15->Q0)/2 + (Q0/2)
		
	}

	EPwm1Regs.CMPA.half.CMPA = PWM_limit(PwmCmp[0],TypeFlag);
	EPwm2Regs.CMPA.half.CMPA = PWM_limit(PwmCmp[1],TypeFlag);
	EPwm3Regs.CMPA.half.CMPA = PWM_limit(PwmCmp[2],TypeFlag);

}


/*
下溢中断的更新值是后半周期的比较值，应该用后半周期的限制值
周期中断的更新值是前半周期的比较值，应该用前半周期的限制值
*/	
Uint16 PWM_limit(Uint16 PwmCmp,Uint16 TypeFlag)
{
	if(TypeFlag == ZERO_INT)
	{
		if(PwmCmp < BACK_PERIOD_COMPARE_LIMIT_MIN)
		{
			return BACK_PERIOD_COMPARE_LIMIT_MIN;
		}
		else if(PwmCmp > BACK_PERIOD_COMPARE_LIMIT_MAX)
		{
			return BACK_PERIOD_COMPARE_LIMIT_MAX;
		}
	}
	else if(TypeFlag == PERIOD_INT)
	{
		if(PwmCmp < FRONT_PERIOD_COMPARE_LIMIT_MIN)
		{
			return FRONT_PERIOD_COMPARE_LIMIT_MIN;
		}
		else if(PwmCmp > FRONT_PERIOD_COMPARE_LIMIT_MAX)
		{
			return FRONT_PERIOD_COMPARE_LIMIT_MAX;
		}
	}	

	return PwmCmp;
}


/**
 * 电流前馈处理
 * 
 * 
 */
void FEEDFORWARD_TORQUE_calc(FEEDFORWARD_TORQUE *v)
{
	_iq Temp;

	v->dIn  = v->In - v->In_1;
	Temp  = v->K * v->dIn;
	v->In_1 = v->In;

	v->Out += (Temp - v->Out)>>2;	// 滤波
}


/**
 * 速度前馈处理
 * 
 * 
 */
void FEEDFORWARD_SPD_calc(FEEDFORWARD_SPD *v)
{
	_iq Temp;
	Temp =_IQmpy((v->K * v->In),v->Kp);

	v->Out += (Temp - v->Out)>>2;	// 滤波
}


/**
 * 位置环PID模块函数
 *
 * 输入：	v->Ref			IQ
 * 			v->Fdb			IQ
 * 			v->Kp			IQ
 * 			v->Ki			IQ
 * 			v->Kd			IQ
 * 			v->Kc			IQ
 * 			v->OutMax		IQ
 * 			v->OutMin		IQ
 * 输出：	v->Out			IQ
 */
void pid_reg4_calc(PIDREG4 *v)
{
	v->d_Fdb = v->Fdb - v->Fdb_1;
	v->Fdb_1 = v->Fdb;
	if(v->PosErr_CLSW)
	{
		v->Err = 0;
	}
	else
	{
		v->Err += (v->Ref - v->d_Fdb);
	}
	if(v->Err>= v->ErrMax) v->Err= v->ErrMax;
	if(v->Err<=-v->ErrMax) v->Err=-v->ErrMax;
	v->OutPreSat = _IQmpyIQX(v->Err,11,_IQ30mpyIQX(v->Kp,30,_IQ30(1/ENC_RESOLUTION),30),30);				/* Compute the proportional output */
	v->Out = _IQsat(v->OutPreSat, v->OutMax, v->OutMin);		/* Saturate the output */
}


/**
 * PID模块函数
 *
 * 输入：	v->Ref			IQ
 * 			v->Fdb			IQ
 * 			v->Kp			IQ
 * 			v->Ki			IQ
 * 			v->Kd			IQ
 * 			v->Kc			IQ
 * 			v->OutMax		IQ
 * 			v->OutMin		IQ
 * 输出：	v->Out			IQ
 */
void pid_reg3_calc(PIDREG3 *v)
{
	// Compute the error
	v->Err = v->Ref - v->Fdb;

	// Compute the proportional output
	v->Up = _IQmpy(v->Kp,v->Err);

	// Compute the integral output
	v->Ui = v->Ui + _IQmpy(v->Ki,v->Up) + _IQmpy(v->Kc,v->SatErr);

	// Compute the derivative output
	v->Ud = _IQmpy(v->Kd,(v->Up - v->Up1));

	// Compute the pre-saturated output
	v->OutPreSat = v->Up + v->Ui + v->Ud;

	// Saturate the output
	if (v->OutPreSat > v->OutMax)
	{
		v->Out =  v->OutMax;
	}
	else if (v->OutPreSat < v->OutMin)
	{
		v->Out =  v->OutMin;
	}
	else
	{
		v->Out = v->OutPreSat;
	}

	// Compute the saturate difference
	v->SatErr = v->Out - v->OutPreSat;

	// Update the previous proportional output
	v->Up1 = v->Up;

}


/**
 * 爬坡曲线计算函数
 *
 * 输入：	v->StepAngleMax		IQ
 * 			v->Resolution		IQ
 * 			v->RampDelayMax		IQ
 * 			v->RampHighLimit	IQ
 * 			v->RampLowLimit		IQ
 * 输出：	v->SetpointValue	IQ
 * 			v->EqualFlag
 */
void rmp_cntl_calc(RMPCNTL *v)
{
	_iq tmp;

	tmp = v->TargetValue - v->SetpointValue;

	if (_IQabs(tmp) > v->Resolution)
 	{
 		v->RampDelayCount += 1;
 		if (v->RampDelayCount >= v->RampDelayMax)
 		{
 			if (v->TargetValue >= v->SetpointValue)
			{
				v->SetpointValue += v->Resolution;
				if (v->SetpointValue > v->RampHighLimit)
				{
					v->SetpointValue = v->RampHighLimit;
				}
				v->RampDelayCount = 0;
			}
			else
			{
				v->SetpointValue -= v->Resolution;
				if (v->SetpointValue < v->RampLowLimit)
				{
					v->SetpointValue = v->RampLowLimit;
				}
				v->RampDelayCount = 0;
			}
		}
	}
	else
	{
		if (v->TargetValue < v->RampLowLimit)
		{
			v->SetpointValue = v->RampLowLimit;
		}
		else if (v->TargetValue > v->RampHighLimit)
		{
			v->SetpointValue = v->RampHighLimit;
		}
		else
		{
			v->SetpointValue = v->TargetValue;
		}
		v->EqualFlag = 0x7FFFFFFF;
	}
}


/**
 * M法测试计算函数
 *
 * 输入：	v->Enc1ms		IQ
 * 			v->K1			IQ
 * 			v->SpeedLow		IQ
 * 			v->SpeedHigh	IQ
 * 			v->BaseRpm		IQ
 * 			v->Direction	IQ
 * 输出：	v->Speed		IQ
 * 			v->SpeedRpm
 */
void speed_frq_calc(SPEED_MEAS_QEP *v)
{
	static Uint16 cntEnc4msDelay = 0;
	static int32 buffer[8]={0,0,0,0,0,0,0,0};	//低速滤波Buffer
	int32 TempSpeed;
	Uint16 i;

	v->Enc4msCnt += v->Enc1ms;
	cntEnc4msDelay++;
	if(cntEnc4msDelay>=4)
	{
		cntEnc4msDelay = 0;
		v->Enc4ms = v->Enc4msCnt;
		v->Enc4msCnt = 0;
	}

	if(v->SpeedRpm > v->SpeedHigh)
	{//高速
		TempSpeed = v->K1 * v->Enc1ms;
		if (v->Direction == MOTOR_DIR_NEG)
		{//反向
		        TempSpeed = - TempSpeed;
		}
		v->SpeedPos += (TempSpeed - v->SpeedPos)>>2;	// 滤波
	}
	else if(v->SpeedRpm > v->SpeedLow)
	{//中速
		TempSpeed = (v->K1>>2) * v->Enc4ms;
		if (v->Direction == MOTOR_DIR_NEG)
		{//反向
		        TempSpeed = - TempSpeed;
		}
		v->SpeedPos += (TempSpeed - v->SpeedPos)>>2;	// 滤波
	}
	else
	{//低速
		TempSpeed = (v->K1>>2) * v->Enc4ms;
		if (v->Direction == MOTOR_DIR_NEG)
		{//反向
		        TempSpeed = - TempSpeed;
		}
	//8次滑动求平均值
		for(i=7; i>0; i--)
		{
			buffer[i] = buffer[i-1];
		}
		buffer[0] = TempSpeed;
		TempSpeed = 0;
		for(i=0; i<8; i++)
		{
			TempSpeed += buffer[i];			//求和
		}
		v->SpeedPos = TempSpeed>>3;			//平均值
	}

	if (v->SpeedPos > _IQ(1))
	{
		v->SpeedPos = _IQ(1);
	}
	else if (v->SpeedPos < _IQ(-1))
    {
		v->SpeedPos = _IQ(-1);
	}

	if (v->Direction == MOTOR_DIR_POS)
	{//正向
		v->Speed = _IQabs(v->SpeedPos);
	}
	else
	{//反向
		v->Speed = - _IQabs(v->SpeedPos);
	}

// Change motor speed from pu value to rpm value (GLOBAL_Q -> Q0)
// Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
	v->SpeedRpm = _IQmpy(v->BaseRpm,_IQabs(v->Speed));			//无符号

}


/**
 * ADC初始化校准函数
 *
 */
void F2803X_adc_meas_init(ADCMEAS *p)
{
	int16 DatQ15;
	int32 Tmp;
	volatile Uint16 *p_result;

//---------A相电流采样回路零位校准--------------
	EALLOW;
	AdcRegs.ADCSOC0CTL.bit.CHSEL  = 0;	//Set SOC0 channel select to ADCINA0
	AdcRegs.ADCSOC1CTL.bit.CHSEL  = 0;	//Set SOC1 channel select to ADCINA0
	AdcRegs.ADCSOC2CTL.bit.CHSEL  = 0;	//Set SOC2 channel select to ADCINA0
	AdcRegs.ADCSOC3CTL.bit.CHSEL  = 0;	//Set SOC3 channel select to ADCINA0
	AdcRegs.ADCSOC4CTL.bit.CHSEL  = 0;	//Set SOC4 channel select to ADCINA0
	AdcRegs.ADCSOC5CTL.bit.CHSEL  = 0;	//Set SOC5 channel select to ADCINA0
	AdcRegs.ADCSOC6CTL.bit.CHSEL  = 0;	//Set SOC6 channel select to ADCINA0
	AdcRegs.ADCSOC7CTL.bit.CHSEL  = 0;	//Set SOC7 channel select to ADCINA0
	AdcRegs.ADCSOC8CTL.bit.CHSEL  = 0;	//Set SOC8 channel select to ADCINA0
	AdcRegs.ADCSOC9CTL.bit.CHSEL  = 0;	//Set SOC9 channel select to ADCINA0
	AdcRegs.ADCSOC10CTL.bit.CHSEL = 0;	//Set SOC10 channel select to ADCINA0
	AdcRegs.ADCSOC11CTL.bit.CHSEL = 0;	//Set SOC11 channel select to ADCINA0
	AdcRegs.ADCSOC12CTL.bit.CHSEL = 0;	//Set SOC12 channel select to ADCINA0
	AdcRegs.ADCSOC13CTL.bit.CHSEL = 0;	//Set SOC13 channel select to ADCINA0
	AdcRegs.ADCSOC14CTL.bit.CHSEL = 0;	//Set SOC14 channel select to ADCINA0
	AdcRegs.ADCSOC15CTL.bit.CHSEL = 0;	//Set SOC15 channel select to ADCINA0
	AdcRegs.ADCSOC0CTL.bit.ACQPS  = 23;	//Set SOC0 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC1CTL.bit.ACQPS  = 23;	//Set SOC1 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC2CTL.bit.ACQPS  = 23;	//Set SOC2 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC3CTL.bit.ACQPS  = 23;	//Set SOC3 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC4CTL.bit.ACQPS  = 23;	//Set SOC4 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC5CTL.bit.ACQPS  = 23;	//Set SOC5 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC6CTL.bit.ACQPS  = 23;	//Set SOC6 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC7CTL.bit.ACQPS  = 23;	//Set SOC7 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC8CTL.bit.ACQPS  = 23;	//Set SOC8 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC9CTL.bit.ACQPS  = 23;	//Set SOC9 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC10CTL.bit.ACQPS = 23;	//Set SOC10 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC11CTL.bit.ACQPS = 23;	//Set SOC11 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC12CTL.bit.ACQPS = 23;	//Set SOC12 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC13CTL.bit.ACQPS = 23;	//Set SOC13 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC14CTL.bit.ACQPS = 23;	//Set SOC14 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC15CTL.bit.ACQPS = 23;	//Set SOC15 acquisition period to 24 ADCCLK
	AdcRegs.INTSEL1N2.bit.INT1SEL = 15;	//Connect ADCINT1 to EOC15
	AdcRegs.INTSEL1N2.bit.INT1E  =  1;	//Enable ADCINT1
	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;//ADCINTs trigger at end of conversion
	EDIS;

	AdcRegs.ADCSOCFRC1.all = 0xFFFF;	//Force start of conversion on SOC0~15

	while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0)
	{};										// Wait until ADC conversion is completed
	AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//Clear ADCINT1

	Tmp = 0;
	for(p_result=&AdcResult.ADCRESULT0;p_result<=&AdcResult.ADCRESULT15;p_result++)
	{
		Tmp += *(p_result);
	}
	Tmp /= 16;									// 求16次采样的平均值

	DatQ15 = (Tmp<<4)^0x8000;					// Convert raw result to Q15 (bipolar signal)
	Tmp = (int32)p->ImeasAGain*(int32)DatQ15;	// Tmp = gain*dat => Q28 = Q13*Q15
	p->ImeasAOffset = -(int16)(Tmp>>13);		// Convert Q28 to Q15

//---------B相电流采样回路零位校准--------------
	EALLOW;
	AdcRegs.ADCSOC0CTL.bit.CHSEL  = 8;	//Set SOC0 channel select to ADCINB0
	AdcRegs.ADCSOC1CTL.bit.CHSEL  = 8;	//Set SOC1 channel select to ADCINB0
	AdcRegs.ADCSOC2CTL.bit.CHSEL  = 8;	//Set SOC2 channel select to ADCINB0
	AdcRegs.ADCSOC3CTL.bit.CHSEL  = 8;	//Set SOC3 channel select to ADCINB0
	AdcRegs.ADCSOC4CTL.bit.CHSEL  = 8;	//Set SOC4 channel select to ADCINB0
	AdcRegs.ADCSOC5CTL.bit.CHSEL  = 8;	//Set SOC5 channel select to ADCINB0
	AdcRegs.ADCSOC6CTL.bit.CHSEL  = 8;	//Set SOC6 channel select to ADCINB0
	AdcRegs.ADCSOC7CTL.bit.CHSEL  = 8;	//Set SOC7 channel select to ADCINB0
	AdcRegs.ADCSOC8CTL.bit.CHSEL  = 8;	//Set SOC8 channel select to ADCINB0
	AdcRegs.ADCSOC9CTL.bit.CHSEL  = 8;	//Set SOC9 channel select to ADCINB0
	AdcRegs.ADCSOC10CTL.bit.CHSEL = 8;	//Set SOC10 channel select to ADCINB0
	AdcRegs.ADCSOC11CTL.bit.CHSEL = 8;	//Set SOC11 channel select to ADCINB0
	AdcRegs.ADCSOC12CTL.bit.CHSEL = 8;	//Set SOC12 channel select to ADCINB0
	AdcRegs.ADCSOC13CTL.bit.CHSEL = 8;	//Set SOC13 channel select to ADCINB0
	AdcRegs.ADCSOC14CTL.bit.CHSEL = 8;	//Set SOC14 channel select to ADCINB0
	AdcRegs.ADCSOC15CTL.bit.CHSEL = 8;	//Set SOC15 channel select to ADCINB0
	AdcRegs.ADCSOC0CTL.bit.ACQPS  = 23;	//Set SOC0 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC1CTL.bit.ACQPS  = 23;	//Set SOC1 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC2CTL.bit.ACQPS  = 23;	//Set SOC2 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC3CTL.bit.ACQPS  = 23;	//Set SOC3 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC4CTL.bit.ACQPS  = 23;	//Set SOC4 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC5CTL.bit.ACQPS  = 23;	//Set SOC5 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC6CTL.bit.ACQPS  = 23;	//Set SOC6 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC7CTL.bit.ACQPS  = 23;	//Set SOC7 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC8CTL.bit.ACQPS  = 23;	//Set SOC8 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC9CTL.bit.ACQPS  = 23;	//Set SOC9 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC10CTL.bit.ACQPS = 23;	//Set SOC10 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC11CTL.bit.ACQPS = 23;	//Set SOC11 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC12CTL.bit.ACQPS = 23;	//Set SOC12 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC13CTL.bit.ACQPS = 23;	//Set SOC13 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC14CTL.bit.ACQPS = 23;	//Set SOC14 acquisition period to 24 ADCCLK
	AdcRegs.ADCSOC15CTL.bit.ACQPS = 23;	//Set SOC15 acquisition period to 24 ADCCLK
	AdcRegs.INTSEL1N2.bit.INT1SEL = 15;	//Connect ADCINT1 to EOC15
	AdcRegs.INTSEL1N2.bit.INT1E  =  1;	//Enable ADCINT1
	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;//ADCINTs trigger at end of conversion
	EDIS;

	AdcRegs.ADCSOCFRC1.all = 0xFFFF;	//Force start of conversion on SOC0~15

	while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0)
	{};										// Wait until ADC conversion is completed
	AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//Clear ADCINT1

	Tmp = 0;
	for(p_result=&AdcResult.ADCRESULT0;p_result<=&AdcResult.ADCRESULT15;p_result++)
	{
		Tmp += *(p_result);
	}
	Tmp /= 16;									// 求16次采样的平均值

	DatQ15 = (Tmp<<4)^0x8000;					// Convert raw result to Q15 (bipolar signal)
	Tmp = (int32)p->ImeasBGain*(int32)DatQ15;	// Tmp = gain*dat => Q28 = Q13*Q15
	p->ImeasBOffset = -(int16)(Tmp>>13);		// Convert Q28 to Q15

	if( (p->ImeasAOffset < p->IOffsetMin) || (p->ImeasAOffset > p->IOffsetMax) \
	  ||(p->ImeasBOffset < p->IOffsetMin) || (p->ImeasBOffset > p->IOffsetMax) )
	{
		p->Error = 1;							// 电流检测回路故障
	}
	else
	{
		p->Error = 0;
	}

//---------定时中断采样排序初始化--------------
	EALLOW;
	AdcRegs.ADCCTL1.bit.TEMPCONV = 0;			//A5 normal
	AdcRegs.ADCCTL1.bit.VREFLOCONV = 0;			//B5 normal
	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;		//EOC的脉冲发出在数据开始转换之后
	
	AdcRegs.ADCCTL2.bit.CLKDIV2EN = 0;
	AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 0;

	AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0x10;		// 关闭优先级

	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN0 = 1;		//并联SOC
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN2 = 1;
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN4 = 0;
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN6 = 0;
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN8 = 0;		//非并联
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN10 = 0;	
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN12 = 0;
	AdcRegs.ADCSAMPLEMODE.bit.SIMULEN14 = 0;

	AdcRegs.ADCINTSOCSEL1.all = 0;
	AdcRegs.ADCINTSOCSEL2.all = 0;

	AdcRegs.INTSEL1N2.all = 0;
	AdcRegs.INTSEL1N2.bit.INT1E = 1;
	AdcRegs.INTSEL1N2.bit.INT1SEL = 3;

	AdcRegs.INTSEL1N2.bit.INT2E = 1;
	AdcRegs.INTSEL1N2.bit.INT2SEL = 5;

	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0;	//A0,B0 pair
	AdcRegs.ADCSOC1CTL.bit.CHSEL = 0;	//A0,B0 pair
	AdcRegs.ADCSOC2CTL.bit.CHSEL = 0;	//A0,B0 pair
	AdcRegs.ADCSOC3CTL.bit.CHSEL = 0;	//A0,B0 pair
	AdcRegs.ADCSOC4CTL.bit.CHSEL  = 1;	//A1	BUS_V
	AdcRegs.ADCSOC5CTL.bit.CHSEL  = 9;	//B1	Pedal

	AdcRegs.ADCSOC0CTL.bit.ACQPS  = 23;
	AdcRegs.ADCSOC1CTL.bit.ACQPS  = 23;	
	AdcRegs.ADCSOC2CTL.bit.ACQPS  = 23;	
	AdcRegs.ADCSOC3CTL.bit.ACQPS  = 23;	
	AdcRegs.ADCSOC4CTL.bit.ACQPS  = 23;	
	AdcRegs.ADCSOC5CTL.bit.ACQPS  = 23;	
	AdcRegs.ADCSOC6CTL.bit.ACQPS  = 23;	
	AdcRegs.ADCSOC7CTL.bit.ACQPS  = 23;	
	AdcRegs.ADCSOC8CTL.bit.ACQPS  = 23;	
	AdcRegs.ADCSOC9CTL.bit.ACQPS  = 23;	

	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC6CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC7CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC8CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
	AdcRegs.ADCSOC9CTL.bit.TRIGSEL = 5;	//PWM1 SOCA
		
	EDIS;

	AdcRegs.ADCINTFLGCLR.all = 0xFFFF;

}


/**
 * ADC采样函数
 *
 */
void F2803X_adc_meas_read(ADCMEAS *p)
{
	int16 DatQ15,adTmp;
	int32 Tmp;
	static Uint16 Flag = 0;
	Uint16 temp[6] = {0,0,0,0,0,0};

	if(Flag == 0)
	{
		while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0)
		{};										// Wait until ADC conversion is completed
		AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//Clear ADCINT1

		Tmp = (AdcResult.ADCRESULT0 + AdcResult.ADCRESULT2)>>1;
//		temp[2] = ((Uint32)Tmp * 1335 + 4763719ul)>>12;//A_R
		temp[2] = Tmp;
		Tmp = (AdcResult.ADCRESULT1 + AdcResult.ADCRESULT3)>>1;
//		temp[3] = ((Uint32)Tmp * 1335 + 4763719ul)>>12;//B_R
		temp[3] = Tmp;
		
		DatQ15 = (temp[2]<<4)^0x8000;	// Convert raw result to Q15 (bipolar signal)
		Tmp = (int32)p->ImeasAGain*(int32)DatQ15;	// Tmp = gain*dat => Q28 = Q13*Q15
		p->ImeasA = (int16)(Tmp>>13);				// Convert Q28 to Q15
		p->ImeasA += p->ImeasAOffset;				// Add offset
	//	p->ImeasA *= -1;							// Positive direction, current flows to motor
		
		DatQ15 = (temp[3]<<4)^0x8000;	// Convert raw result to Q15 (bipolar signal)
		Tmp = (int32)p->ImeasBGain*(int32)DatQ15;	// Tmp = gain*dat => Q28 = Q13*Q15
		p->ImeasB = (int16)(Tmp>>13);				// Convert Q28 to Q15
		p->ImeasB += p->ImeasBOffset;				// Add offset
	//	p->ImeasB *= -1;							// Positive direction, current flows to motor

		p->ImeasC = -(p->ImeasA + p->ImeasB);		// Compute phase-c current
	
		Flag = 1;
	}
	else
	{
		while(AdcRegs.ADCINTFLG.bit.ADCINT2 == 0)
		{};										// Wait until ADC conversion is completed
		AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;	//Clear ADCINT1

		DatQ15 = (AdcResult.ADCRESULT4<<3)&0x7FFF;	// Convert raw result to Q15 (unipolar signal)
		Tmp = (int32)p->VdcMeasGain*(int32)DatQ15;	// Tmp = gain*dat => Q28 = Q13*Q15
		if (Tmp > 0x0FFFFFFF)						// Limit Tmp to 1.0 in Q28
			Tmp = 0x0FFFFFFF;
		adTmp = (int16)(Tmp>>13);					// Convert Q28 to Q15
		adTmp += p->VdcMeasOffset;					// Add offset
		p->VdcMeas += (adTmp - p->VdcMeas)>>3;		// 滤波(pu)
		p->VdcCoeff = _IQdiv(_IQ(VDC_FULL/BASE_VDC),_IQ15toIQ(p->VdcMeas));		//求电压调制系数
		
		adTmp = (AdcResult.ADCRESULT5<<3)&0x7FFF;		// Convert raw result to Q15 (unipolar signal)
		p->pedalAD += (adTmp - p->pedalAD)>>3;		// 滤波

		Flag = 0;
	}

}


/**
 * Drive模块刷新
 *
 * 输入：	p->EnableFlag
 * 输出：	PWM_EN(IO)
 */
void F2803X_DRIVE_Update(DRIVE *p)
{
// ENdrive DISdrive
	EALLOW;                       // Enable EALLOW
	if (p->EnableFlag==1)   
	{// Enable PWM drive
		PWM_EN_ON;
		EPwm1Regs.TZCLR.bit.OST = 1;
		EPwm2Regs.TZCLR.bit.OST = 1;
		EPwm3Regs.TZCLR.bit.OST = 1;
		EPwm1Regs.TZCLR.bit.INT = 1;
	}
	else// if (p->EnableFlag==0)
	{// Disable PWM drive
		PWM_EN_OFF;
		EPwm1Regs.TZFRC.bit.OST = 1;
		EPwm2Regs.TZFRC.bit.OST = 1;
		EPwm3Regs.TZFRC.bit.OST = 1;
	}
	EDIS;                         // Disable EALLOW

}


/**
 * QEP模块初始化
 *
 */
void  F2803X_QEP_Init(QEP *p)
{

	EQep1Regs.QDECCTL.all = QDECCTL_INIT_STATE;
	EQep1Regs.QEPCTL.all = QEPCTL_INIT_STATE;
	EQep1Regs.QPOSCTL.all = QPOSCTL_INIT_STATE;
	EQep1Regs.QUPRD = 1000000;		        	// Unit Timer for 100Hz
	EQep1Regs.QCAPCTL.all = QCAPCTL_INIT_STATE;
//	EQep1Regs.QPOSMAX = 4*p->LineEncoder;
	EQep1Regs.QPOSMAX = 0xFFFFFFFF;

//	EALLOW;                       // Enable EALLOW
//	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;  // GPIO20 is EQEP1A
//	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;  // GPIO21 is EQEP1B
//	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;  // GPIO23 is EQEP1I
//	EDIS;                         // Disable EALLOW

}


/**
 * QEP模块测角度计算函数
 *
 * 输入：	p->MechScaler
 * 			p->TotalEnc
 * 			p->PolePairs
 * 			p->CalibratedAngle		IQ24
 * 输出：	p->DirectionQep
 * 			p->ElecTheta			IQ24
 * 			p->MechTheta			IQ24
 * 			p->EncSpeed				IQ24
 */
void F2803X_QEP_Calc(QEP *p)
{

// Check the position counter for EQEP1
	p->CntUpdate = EQep1Regs.QPOSCNT;
	p->DeltaEnc = p->CntUpdate - p->CntOld;
	p->CntOld = p->CntUpdate;
	
	p->TotalEnc += p->DeltaEnc;
	p->EncSpeed += p->DeltaEnc;
	if(MotorDir == MOTOR_DIR_POS)
	{
		p->EncPos += p->DeltaEnc; 
	}
	else
	{
		p->EncPos -= p->DeltaEnc; 
	}

// Check the rotational direction
//	p->DirectionQep = EQep1Regs.QEPSTS.bit.QDF;
	if(p->DeltaEnc > 0)
	{
		p->DirectionQep = MOTOR_DIR_POS;	// 正向
	}
	else if(p->DeltaEnc < 0)
	{
		p->DirectionQep = MOTOR_DIR_NEG;	// 反向
	}

// Compute the electrical angle in Q24
	p->ElecTheta = __qmpy32by16(p->MechScaler,(int16)p->TotalEnc,31);	// Q15 = Q30*Q0
	p->ElecTheta = p->PolePairs * p->ElecTheta; 						// Q15 = Q0*Q15
	p->ElecTheta <<= 9;													// Q15 -> Q24
	p->ElecTheta += p->CalibratedAngle;
	p->ElecTheta &= 0x00FFFFFF;                          				// Wrap around 0x00FFFFFF

// Compute the mechanical angle in Q24
	if(MotorDir == MOTOR_DIR_POS)
	{
		p->DeltaMechTheta = __qmpy32by16(p->MechScaler,(int16)p->DeltaEnc,31);		// Q15 = Q30*Q0
		p->DeltaEncAll = (int16)p->DeltaEnc;		
	}
	else
	{
		p->DeltaMechTheta = __qmpy32by16(p->MechScaler,(int16)(-p->DeltaEnc),31);	// Q15 = Q30*Q0
		p->DeltaEncAll = (int16)(-p->DeltaEnc);		
	}
	p->DeltaMechTheta <<= 9;											// Q15 -> Q24
	p->DeltaMechTheta = (int64)p->DeltaMechTheta * RatioWheel / 1000;
	p->MechTheta += p->DeltaMechTheta;
	p->MechTheta &= 0x00FFFFFF;                          				// Wrap around 0x00FFFFFF
	p->MechThetaDegree =  _IQ24mpy(p->MechTheta, BASE_THETA);

	p->TotalEncAll += p->DeltaEncAll;
	p->MechThetaAll = (Uint64)p->TotalEncAll * RatioWheel / 1000 / (p->LineEncoder * 4 / 360);

	p->TotalEncPos += p->DeltaEncAll;
	if(p->TotalEncPos>=(p->LineEncoder*4)) {p->TotalEncPos -= (p->LineEncoder*4);}
	else if(p->TotalEncPos<0) {p->TotalEncPos += (p->LineEncoder*4);}

// Check an index occurrence
//	if (EQep1Regs.QFLG.bit.IEL == 1)                   
//	{  
//		p->IndexSyncFlag = 0x00F0;
//		p->QepCountIndex = EQep1Regs.QPOSILAT; 
//		EQep1Regs.QCLR.bit.IEL = 1;					// Clear interrupt flag
//	}

// Check unit Time out-event for speed calculation:
// Unit Timer is configured for 100Hz in INIT function
//	if(EQep1Regs.QFLG.bit.UTO == 1)
//	{
//			//**** Low Speed Calculation   ****//
//		if((EQep1Regs.QEPSTS.bit.COEF || EQep1Regs.QEPSTS.bit.CDEF))
//		{	// Capture Counter overflowed, hence do no compute speed
//			EQep1Regs.QEPSTS.all = 0x000C;
//		}
//		else
//		{	// Compute lowspeed using capture counter value
//			p->QepPeriod = EQep1Regs.QCPRDLAT;
//		}
//	}

}


/**
 * QEP模块中断函数
 * 电角度校正
 * 输入：	p->I2cAngle
 * 			
 * 输出：	p->TotalEnc
 * 			p->CalibratedAngle
 */
void F2803X_QEP_Isr(QEP *p)
{
	int32 ThetaSet,DeltaTheta;
	static Uint16 cntTestRepeat = 0;
	Uint16 cntEnc;

	if(flagThetaTest.bit.TestEn == 1)
	{//测初始角模式
		if((flagThetaTest.bit.Step==1) && (p->DirectionQep == MOTOR_DIR_POS))
		{//转动阶段 测3次
			cntTestRepeat++;
			cntEnc = abs(p->TotalEnc);
			switch(cntTestRepeat)
			{
				case 1:
					ThetaTest1st = (Uint32)cntEnc * MOTOR_POLE * 45 / ENC_RESOLUTION;
					flagThetaTest.bit.Step = 0;			//转动结束，下次加直流
					break;
				case 2:
					ThetaTest2nd = (Uint32)cntEnc * MOTOR_POLE * 45 / ENC_RESOLUTION;
					flagThetaTest.bit.Step = 0;			//转动结束，下次加直流
					break;
				case 3:
					ThetaTest3rd = (Uint32)cntEnc * MOTOR_POLE * 45 / ENC_RESOLUTION;
					flagThetaTest.bit.Step = 0;			//转动结束，下次加直流
					cntTestRepeat = 0;
					flagThetaTest.bit.Calc = 1;			//一轮测试完成 需要计算
					break;
				default:
					break;
			}//end of switch(cntTestRpt)
			p->TotalEnc = 0;			
		}
	}
	else
	{//非测初始角模式
		ThetaSet = p->I2cAngle;
		if(p->DirectionQep == MOTOR_DIR_NEG)	//反向
		{
			ThetaSet += _IQ24(60.0/360);	//+180du
			ThetaSet &= 0x00FFFFFF;	// Wrap around 0x00FFFFF
		}
		DeltaTheta = _IQ24abs(p->ElecTheta - ThetaSet);
		if((DeltaTheta <= _IQ24(0.25)) || (DeltaTheta >= _IQ24(0.75)))	//当前角度值与校正值相差大于90度时不校正
		{
			p->TotalEnc = 0;
			p->CalibratedAngle = ThetaSet;
		}
	}

}//end of F280X_QEP_Isr()


/**
 * HALL模块初始化函数
 * 
 * 输入：	HALL_A				IO pin
 * 			HALL_B				IO pin		
 * 			HALL_C				IO pin
 * 			p->I2cAngle			IQ24
 * 输出：	p->CalibratedAngle	IQ(-1)~IQ(1)
 * 			p->Error
 */
void HALL_Calc(HALL *p)
{

	p->Error = 0x0;
	p->HallState = HALL_C;
	p->HallState <<= 1;
	p->HallState += HALL_B;
	p->HallState <<= 1;
	p->HallState += HALL_A;

	switch(p->HallState)
	{
	case 1:		// 001
		p->CalibratedAngle = p->I2cAngle + _IQ24(90.0/360.0);
		break;
	case 2:		// 010
		p->CalibratedAngle = p->I2cAngle + _IQ24(210.0/360.0);
		break;
	case 3:		// 011
		p->CalibratedAngle = p->I2cAngle + _IQ24(150.0/360.0);
		break;
	case 4:		// 100
		p->CalibratedAngle = p->I2cAngle + _IQ24(330.0/360.0);
		break;
	case 5:		// 101
		p->CalibratedAngle = p->I2cAngle + _IQ24(30.0/360.0);
		break;
	case 6:		// 110
		p->CalibratedAngle = p->I2cAngle + _IQ24(270.0/360.0);
		break;
	default:	// 000 111
		p->Error = 0x1;
		break;
	}//end of switch
	if(p->CalibratedAngle > _IQ24(1.0))
	{
		p->CalibratedAngle -= _IQ24(1.0);
	}
	else if (p->CalibratedAngle < _IQ24(-1.0))
	{
		p->CalibratedAngle += _IQ24(1.0);
	}

}//end of HALL_Calc()






//===========================================================================
// No more.
//===========================================================================
