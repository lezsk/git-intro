/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
 * All rights reserved.
 *
 * @file Main.c
 * 主入口函数文件.
 * 该文件是程序的入口函数文件，由main函数进入程序运行.
 * 提供main函数调用的DSP各模块的初始化配置函数.
 * 系统各模块的初始化配置函数.
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
//begin... These are defined by the linker (see F28035.cmd) //烧写
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
//end...   These are defined by the linker (see F28035.cmd) //烧写

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
 * DSP2808主入口函数.
 * 主函数先初始化各内设、外设；设置各IO以及中断；
 * 然后启动中断进入IDLE循环.
 * 
 * - 使用EVA定时器2产生200us定时中断
 * - 选择SCIB作为通讯串口
 * - 握手成功后开中断进入中断函数
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
	static Uint16 PowerStep = POWER_STEP_START;		// 上电步骤
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
	InitAdc();				//ADC模块工作方式初始化

/* Step 5.	User specific code, enable interrupts: */
	DELAY_US(500000L);		//内部复位之后的延时处理，等待硬件电路稳定
	I2CA_Init();			//I2CA模块初始化
//========Flash API Init====================
//system_Init函数中有读取写入flash的函数,相关变量要先设置
	EALLOW;
	Flash_CPUScaleFactor = SCALE_FACTOR;
    EDIS;

	EALLOW;
    Flash_CallbackPtr = NULL; 
    EDIS;
//=============================================
	System_Init();			//系统初始化

#if (WATCHDOG_MODE == WATCHDOG_ENABLE)
	EALLOW;						// Enable the watchdog
	SysCtrlRegs.WDCR = 0x0028;	// 溢出时间：2^8*512*2^(bit210-1)/20M = 6.5536 ms
	EDIS;
//	DELAY_US(65600L);			// check溢出时间 for debug
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
		{//1ms时间片程序区

			counterMainISR = 0;			// 主中断计数器清零
			GetSysInfo();				// 系统信息获取

			switch(PowerStep)			// 上电时分步骤任务处理
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
						PowerStepHmiFlag = 1;	// 初始化完成
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
						PowerStep = POWER_STEP_MOTOR;	//测电机初始角
					}
					else if(hmi_common_vars.hmi_states == T_PEDAL)
					{	
						PowerStep = POWER_STEP_PEDAL;	//踏板初始化
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
					
					Power_Go2Up();				// 上电找上针位
					if(AngleInit.AngleInitFlag == 1)
					{
						Speed_Detect();				// 速度检测
						Pos_P_Reg();					// 位置环处理
						Speed_PI_Reg();				// 速度环处理(包括PI高低速切换)
						Brake();							// 刹车处理	
						Synchro_Detect();			// 停针信号丢失检测
					}
					if(flagPower.bit.Go2UpOver == 1)
					{
						PowerStep = POWER_STEP_SEW;
					}
					break;
				case POWER_STEP_SEW:
					Command_Read();				// 命令扫描
					//flagSew.bit.PedalUnlock = 1;
					//RunCmd = CMD_RUN;
					//SpeedPedal = 500;
					//RunCmd = CMD_RUN;
					if(AngleInit.AngleInitFlag != 1 && RunCmd == CMD_RUN)		//未完成定位
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
							
							AngleInit.AngleInitFlag = 1;		//搜索结束
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
						
						SewProcess();					// 缝纫处理

						Speed_Detect();				// 速度检测
						Pos_P_Reg();					// 位置环处理
						Speed_PI_Reg();				// 速度环处理(包括PI高低速切换)
						Brake();							// 刹车处理
						//SaveDataPoff();			// 掉电保存数据
						Synchro_Detect();			// 停针和HALLA信号丢失检测
					}
					//Speed_Detect();
					break;
				case POWER_STEP_MOTOR:			//电机角度测试
					Command_Read();				// 命令扫描
					MotorTest();
					Speed_Detect();				// 速度检测
					//Speed_PI_Reg();				// 速度环处理(包括PI高低速切换)
					Brake();					// 刹车处理
					break;
				case POWER_STEP_PEDAL:			//模拟踏板校正
					PedalZero_Autoset();		// 模拟踏板校零
					break;
				case POWER_STEP_AGING:
					Command_Read_AutoTest();				
					SewRun_AutoRun();
					if(AngleInit.AngleInitFlag == 1)
					{
						Speed_Detect();				// 速度检测
						Pos_P_Reg();					// 位置环处理
						Speed_PI_Reg();				// 速度环处理(包括PI高低速切换)
						Brake();							// 刹车处理
						Synchro_Detect();			// 停针信号丢失检测
					}
					break;
			}//end of switch

			if(PowerStepHmiFlag == 1)
			{
				DHMI_Process();				// 机箱面板小HMI界面处理
				#if TEST_SUV_UPDATE
				updata_suv_data();					// 滚动刷新监控值函数
				#endif
			}			
			KEY_Read();							// 按键定时扫描处理

			PoffDetect();						// 掉电检测
			if(flagMotorState == MOTOR_STATE_STOP)
			{
				EEPROM_ParaWrite(PARA_BUF,&ChangeParaBuf);
			}
			FaultDetect();						// 1ms定时故障检测
			Motor_Overload();					// 电机过载故障检测
			#if(PEDAL_TYPE == PEDAL_L)
			PedaleCapCounter.NowCounter = PedalCounterRead(&PedaleCapCounter);//踏板检测
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

		}//end of 1ms时间片

	//主循环程序区
		FaultProcess();							// 出错保护处理
		Synchro_Clear();						// 停针和HALLA信号检测计数器清零
		ModeParaUpdate();						// 参数解释

	}//end of 主循环

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
	{//确定为硬件PB_FLT有效
		PWM_EN_OFF;				// 电机使能关
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
		CL_ON;			//继电器吸合
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
	if(AngleInit.AngleInitFlag == 1 || flagThetaTest.bit.TestEn == 1)	//只有找零完成或者测试模式才能清零
	{
		qep1.isr(&qep1);
		if((MotorDir == qep1.DirectionQep) && (flagPos.bit.Step != STEP_PLAST_CAL))
		{//反向不校零
			flagSew.bit.MechThetaOk = 1;	//置上电机械角度已修正标志
			qep1.MechTheta = _IQ24(MECHTHETA_ZSIGNAL/360.0);		//机头机械角度校正
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
 * 测试
 * 
 * 
 */
void debug_cal(void)
{
/*	int32 tmp1 = 0;					//停针计算用机械角度Ref IQ32 pu
	int32 tmp2 = 0;					//停针计算用机械角度Fdb IQ32 pu
	int32 tmp3 = 0;					//停针计算用机械角度Fdb IQ32 pu
	int32 tmp4 = 0;					//停针计算用机械角度Delta IQ32 pu
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
