/**
 * Copyright (c) 2010, �Ϻ����µ��ӿƼ����޹�˾
 * All rights reserved.
 *
 * @file Main.c
 * ����ں����ļ�.
 * ���ļ��ǳ������ں����ļ�����main���������������.
 * �ṩmain�������õ�DSP��ģ��ĳ�ʼ�����ú���.
 * ϵͳ��ģ��ĳ�ʼ�����ú���.
 * 
 *
 * @author yemj
 * @version 0.1
 * @date 2010-2-25
 *
 */


#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "Flash2803x_API_Config.h"
#include "Flash2803x_API_Library.h"
#include "IQmathLib.h"

#include "Defines.h"
#include "build.h"
#include "GlobalVars.h"
#include "DMCvars.h"
#include "DMCparameter.h"
//#include "E2prom.h"
#include "IO_Def.h"
#include "hmi.h"
#include "Pedal.h"
//begin... These are defined by the linker (see F28035.cmd) //��д
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
//end...   These are defined by the linker (see F28035.cmd) //��д

// Prototype statements for functions found within this file.
extern interrupt void MainISR(void);
extern interrupt void SecondaryISR(void);
interrupt void epwm1_tzint_isr(void);
interrupt void xint1_isr(void);
interrupt void xint2_isr(void);
interrupt void xint3_isr(void);

extern interrupt void i2c_int1a_isr(void);
extern void I2CA_Init(void);

extern void System_Init(void);
extern void Synchro_Detect(void);
extern void Synchro_Clear(void);
extern void PoffDetect(void);
extern void FaultDetect(void);
extern void FaultProcess(void);
extern void Motor_Overload(void);
extern void Brake(void);
extern Uint16 Pedal_Speed(Uint16 PedalAD, Uint16 PedalZoom, Uint16 PedalOpt, Uint16 SpeedMin, Uint16 SpeedMax);
extern void GetSysInfo(void);

extern void Power_Go2Up(void);
extern void SewProcess(void);
extern void ModeParaUpdate(void);
extern void Speed_Detect(void);
extern void Speed_PI_Reg(void);
extern void Pos_P_Reg(void);

extern void PedalZero_Autoset(void);
extern void MotorTest(void);

extern void SaveDataPoff(void);
extern Uint16 led_drv_init(void);
extern HMI_Vars hmi_common_vars;

extern Command_Read_AutoTest(void);
void debug_cal(void);

extern Uint32 PedalCounterRead(PEDAL_ECAP_COUNTER *pPedalCounter);
extern Uint16 AngleInitSearch(void);
/**
 * DSP2808����ں���.
 * �������ȳ�ʼ�������衢���裻���ø�IO�Լ��жϣ�
 * Ȼ�������жϽ���IDLEѭ��.
 * 
 * - ʹ��EVA��ʱ��2����200us��ʱ�ж�
 * - ѡ��SCIB��ΪͨѶ����
 * - ���ֳɹ����жϽ����жϺ���
 */
#define POWER_STEP_START		0
#define POWER_STEP_HALL			1
#define POWER_STEP_GO2UP		2
#define POWER_STEP_SEW			3
#define POWER_STEP_PEDAL		4
#define POWER_STEP_MOTOR		5
#define POWER_STEP_AGING		6
#define POWER_STEP_MODE			7
#define POWER_STEP_DELAY		8

#define POWER_STEP_HMI_INIT		0
#define POWER_STEP_HMI_RUN		1

void main(void)
{
	static Uint16 PowerStep = POWER_STEP_START;		// �ϵ粽��
	static Uint16 PowerDelay = 0;
	static Uint16 PowerStepHmiFlag = 0;

/* Step 1.	Initialize System Control:
	 		PLL, WatchDog, enable Peripheral Clocks */
	InitSysCtrl();

/* Step 2.	Initialize GPIO */ 
	InitGpio();
	
	#if(PEDAL_TYPE == PEDAL_L)
	InitECapture();
	#endif
		
/* Step 3.	Clear all interrupts and initialize PIE vector table: */
	DINT;					// Disable CPU interrupts 
	InitPieCtrl();			// Initialize the PIE control registers.(all disabled and cleared)
	IER = 0x0000;			// Disable CPU interrupts
	IFR = 0x0000;			// Clear all CPU interrupt flags
	InitPieVectTable();		// Initialize the PIE vector table to the shell ISRs

/* Interrupts used  are re-mapped to ISR functions */
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.EPWM1_INT = &MainISR;
	PieVectTable.EPWM2_INT = &SecondaryISR;
	PieVectTable.EPWM1_TZINT = &epwm1_tzint_isr;
//	PieVectTable.XINT1 = &xint1_isr;
	PieVectTable.XINT2 = &xint2_isr;
	PieVectTable.XINT3 = &xint3_isr;
	PieVectTable.I2CINT1A = &i2c_int1a_isr;
	EDIS;    // This is needed to disable write to EALLOW protected registers
	
/* Step 4.	Initialize all the Device Peripherals: */
#if (BUILD_MODE == FLASH_MODE)
	// Copy time critical code and Flash setup code to RAM
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM
	InitFlash();
#endif

//	InitPeripherals(); // Not required for this example
	InitAdc();				//ADCģ�鹤����ʽ��ʼ��

/* Step 5.	User specific code, enable interrupts: */
	DELAY_US(500000L);		//�ڲ���λ֮�����ʱ�����ȴ�Ӳ����·�ȶ�
	I2CA_Init();			//I2CAģ���ʼ��
//========Flash API Init====================
//system_Init�������ж�ȡд��flash�ĺ���,��ر���Ҫ������
	EALLOW;
	Flash_CPUScaleFactor = SCALE_FACTOR;
    EDIS;

	EALLOW;
    Flash_CallbackPtr = NULL; 
    EDIS;
//=============================================
	System_Init();			//ϵͳ��ʼ��

#if (WATCHDOG_MODE == WATCHDOG_ENABLE)
	EALLOW;						// Enable the watchdog
	SysCtrlRegs.WDCR = 0x0028;	// ���ʱ�䣺2^8*512*2^(bit210-1)/20M = 6.5536 ms
	EDIS;
//	DELAY_US(65600L);			// check���ʱ�� for debug
#else
	DisableDog();
#endif

/* Enable CPU INTx Group which is used */
	IER |= M_INT1;			// Enable CPU INT1 for XINT1 & XINT2
	IER |= M_INT2;			// Enable CPU INT2 for EPWM1_TZINT
	IER |= M_INT3;			// Enable CPU INT3 for EPWM1_INT
	IER |= M_INT8;			// Enable CPU INT8 for I2CINT1A
	IER |= M_INT12;			// Enable CPU INT12 for XINT3

/* Enable PIE bits which is used for each CPU INTx Group */
	PieCtrlRegs.PIEIER1.all = M_INT5;//M_INT4+M_INT5;		// Enable XINT2:
	PieCtrlRegs.PIEIER2.all = M_INT1;				// Enable EPWM1_TZINT: Group 2.1
	PieCtrlRegs.PIEIER3.all = M_INT1+M_INT2;		// Enable EPWM1_INT&EPWM2_INT: Group 3.1&2
	PieCtrlRegs.PIEIER8.all = M_INT1;				// Enable I2CINT1A: Group 8.1
	PieCtrlRegs.PIEIER12.all = M_INT1;				// Enable XINT3: Group 12.1

/* Enable global Interrupts and higher priority real-time debug events: */
	EINT;   				// Enable Global interrupt INTM
	ERTM;   				// Enable Global realtime interrupt DBGM

	KEY_PowerOn();
/* Step 6. IDLE loop. Just sit and loop forever (optional): */
	for(;;)
	{
	#if (WATCHDOG_MODE == WATCHDOG_ENABLE)
		ServiceDog();		// KickDog
	#endif
		if (counterMainISR >= NUM_CNT_1MS)
		{//1msʱ��Ƭ������

			counterMainISR = 0;			// ���жϼ���������
			GetSysInfo();				// ϵͳ��Ϣ��ȡ

			switch(PowerStep)			// �ϵ�ʱ�ֲ���������
			{
				case POWER_STEP_START:
				default:
					if(1 ==  DHMI_init()) 
					{
						PowerStep = POWER_STEP_DELAY;
					}
					break;
				case POWER_STEP_DELAY:
					PowerDelay++;
					if(PowerDelay > 1500)	//1500  2012.11.5
					{
						PowerStep = POWER_STEP_HALL;
						PowerStepHmiFlag = 1;	// ��ʼ�����
					}
					break;
				case POWER_STEP_HALL:
					
					qep1.I2cAngle = _IQ24(I2cAngle/BASE_THETA);
					qep1.CalibratedAngle = 0;

					qep1.ElecTheta = 0;
					qep1.TotalEnc = 0;
					qep1.CntOld = 0;
					qep1.CntUpdate = 0;
					EQep1Regs.QPOSCNT = 0;

					PowerStep = POWER_STEP_MODE;
					break;
				case POWER_STEP_MODE:
					if(hmi_common_vars.hmi_states == T_MOTOR)
					{
						PowerStep = POWER_STEP_MOTOR;	//������ʼ��
					}
					else if(hmi_common_vars.hmi_states == T_PEDAL)
					{	
						PowerStep = POWER_STEP_PEDAL;	//̤���ʼ��
					}
					else if(hmi_common_vars.hmi_states == AGING)
					{
						PowerStep = POWER_STEP_AGING;
					}
					else
					{
						PowerStep = POWER_STEP_GO2UP;
					}
					break;
				case POWER_STEP_GO2UP:
					
					Power_Go2Up();				// �ϵ�������λ
					if(AngleInit.AngleInitFlag == 1)
					{
						Speed_Detect();				// �ٶȼ��
						Pos_P_Reg();					// λ�û�����
						Speed_PI_Reg();				// �ٶȻ�����(����PI�ߵ����л�)
						Brake();							// ɲ������	
						Synchro_Detect();			// ͣ���źŶ�ʧ���
					}
					if(flagPower.bit.Go2UpOver == 1)
					{
						PowerStep = POWER_STEP_SEW;
					}
					break;
				case POWER_STEP_SEW:
					Command_Read();				// ����ɨ��
					//flagSew.bit.PedalUnlock = 1;
					//RunCmd = CMD_RUN;
					//SpeedPedal = 500;
					//RunCmd = CMD_RUN;
					if(AngleInit.AngleInitFlag != 1 && RunCmd == CMD_RUN)		//δ��ɶ�λ
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
							
							AngleInit.AngleInitFlag = 1;		//��������
						}
					}					
					else
					{
						#if(PEDAL_TYPE == PEDAL_L)
						SpeedPedal = Pedal_Speed(PedaleCapCounter.NowHZ_Q15, SpeedPedalZoom, optPedalCurve, SpeedPedalMin, SpeedPedalMax);
						//SpeedPedal = 1000;
						#else
						SpeedPedal = Pedal_Speed(adc_meas1.pedalAD, SpeedPedalZoom, optPedalCurve, SpeedPedalMin, SpeedPedalMax);
						#endif
						
						SewProcess();					// ���Ҵ���

						Speed_Detect();				// �ٶȼ��
						Pos_P_Reg();					// λ�û�����
						Speed_PI_Reg();				// �ٶȻ�����(����PI�ߵ����л�)
						Brake();							// ɲ������
						//SaveDataPoff();			// ���籣������
						Synchro_Detect();			// ͣ���HALLA�źŶ�ʧ���
					}
					//Speed_Detect();
					break;
				case POWER_STEP_MOTOR:			//����ǶȲ���
					Command_Read();				// ����ɨ��
					MotorTest();
					Speed_Detect();				// �ٶȼ��
					//Speed_PI_Reg();				// �ٶȻ�����(����PI�ߵ����л�)
					Brake();					// ɲ������
					break;
				case POWER_STEP_PEDAL:			//ģ��̤��У��
					PedalZero_Autoset();		// ģ��̤��У��
					break;
				case POWER_STEP_AGING:
					Command_Read_AutoTest();				
					SewRun_AutoRun();
					if(AngleInit.AngleInitFlag == 1)
					{
						Speed_Detect();				// �ٶȼ��
						Pos_P_Reg();					// λ�û�����
						Speed_PI_Reg();				// �ٶȻ�����(����PI�ߵ����л�)
						Brake();							// ɲ������
						Synchro_Detect();			// ͣ���źŶ�ʧ���
					}
					break;
			}//end of switch

			if(PowerStepHmiFlag == 1)
			{
				DHMI_Process();				// �������СHMI���洦��
				#if TEST_SUV_UPDATE
				updata_suv_data();					// ����ˢ�¼��ֵ����
				#endif
			}			
			KEY_Read();							// ������ʱɨ�账��

			PoffDetect();						// ������
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				EEPROM_ParaWrite(PARA_BUF,&ChangeParaBuf);
			}
			FaultDetect();						// 1ms��ʱ���ϼ��
			Motor_Overload();					// ������ع��ϼ��
			#if(PEDAL_TYPE == PEDAL_L)
			PedaleCapCounter.NowCounter = PedalCounterRead(&PedaleCapCounter);//̤����
			if(PedaleCapCounter.NowCounter == 0)
			{
				PedaleCapCounter.NowHZ_Q15 = 0;
			}
			else
			{
				PedaleCapCounter.NowHZ_Q15 = PEDAL_COUNTER_TO_HZ_Q15(PedaleCapCounter.NowCounter);
			}
			#endif
			
//			EEPROM_Test();

		}//end of 1msʱ��Ƭ

	//��ѭ��������
		FaultProcess();							// ����������
		Synchro_Clear();						// ͣ���HALLA�źż�����������
		ModeParaUpdate();						// ��������

	}//end of ��ѭ��

}//end of Main()


/**
 * epwm1_tzint_isr
 * PB_FLT
 * 
 * 
 * 
 */
interrupt void epwm1_tzint_isr(void)
{
	if(PB_FLT == 0)
	{//ȷ��ΪӲ��PB_FLT��Ч
		PWM_EN_OFF;				// ���ʹ�ܹ�
		ErrorCode = ERR_HOC;
	}

	// Acknowledge this interrupt to receive more interrupts from group 2
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}//end of epwm1_tzint_isr()



/**
 * xint2_isr
 * OZ signal
 * 
 * 
 * 
 */
interrupt void xint2_isr(void)
{
	flagPower.bit.cntCL = 0;
	CntOZdelay = 0;

	if((!CL_ISON) && (SysInfo.Vdc > UDC_CLON))
	{
		CL_ON;			//�̵�������
	}

	// Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}//end of xint2_isr()


/**
 * xint3_isr
 * UP signal
 * 
 * 
 * 
 */
interrupt void xint3_isr(void)
{
	if(AngleInit.AngleInitFlag == 1 || flagThetaTest.bit.TestEn == 1)	//ֻ��������ɻ��߲���ģʽ��������
	{
		qep1.isr(&qep1);
		if((MotorDir == qep1.DirectionQep) && (flagPos.bit.Step != STEP_PLAST_CAL))
		{//����У��
			flagSew.bit.MechThetaOk = 1;	//���ϵ��е�Ƕ���������־
			qep1.MechTheta = _IQ24(MECHTHETA_ZSIGNAL/360.0);		//��ͷ��е�Ƕ�У��
			qep1.MechThetaDegree =  _IQ24mpy(qep1.MechTheta, BASE_THETA);
			qep1.TotalEncPos = MECHTHETA_ZSIGNAL * (ENC_RESOLUTION*4) / 360;
		}
	}
	// Acknowledge this interrupt to get more from group 12
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

}//end of xint3_isr()


/**
 * debug_cal
 * 
 * ����
 * 
 * 
 */
void debug_cal(void)
{
/*	int32 tmp1 = 0;					//ͣ������û�е�Ƕ�Ref IQ32 pu
	int32 tmp2 = 0;					//ͣ������û�е�Ƕ�Fdb IQ32 pu
	int32 tmp3 = 0;					//ͣ������û�е�Ƕ�Fdb IQ32 pu
	int32 tmp4 = 0;					//ͣ������û�е�Ƕ�Delta IQ32 pu
	int16 value0 = 0;
	int16 value1 = 0;
	int16 max0 = 0;
	int16 max1 = 0;

	
	tmp3 = (Uint32)tmp1 - (Uint32)tmp2;
	if((Uint32)tmp3 < (Uint32)tmp4)
	{
		asm(" nop");
	}
//	StitchCount();
	pSewParaDefault = PSEWPARA_DEFAULT[0];
	
	value0 =  (* pSewParaDefault).Value;
	value1 =  (* (pSewParaDefault+1)).Value;
	max0 =  (* pSewParaDefault).Max;
	max1 =  (* (pSewParaDefault+1)).Max;
*/

}



//===========================================================================
// No more.
//===========================================================================
