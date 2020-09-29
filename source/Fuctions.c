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

#include "build.h"
#include "GlobalVars.h"
#include "DMCvars.h"
#include "Defines.h"
#include "IO_Def.h"
#include "DMCparameter.h"
#include "IQmathLib.h"
#include "Pedal.h"
#include "hmi.h"
#include "par.h"
#include "math.h"

int16 PedalMoniAD_Cal(int16 para, _iq scale, _iq offset);
void AutoStepClearZero(void);

extern Uint16 ParaRead(Uint16 *pBuf);
extern Uint16 ParaWrite(void);
extern const PARAM_UNIT SEWPARA_DEFAULT[];
extern Uint16 par_adv_value[PAR_ADV_DEF_SIZE];
extern HMI_Vars hmi_common_vars;
Uint32 PedalKHz_Cal(int16 length);

/**
 * System_Init
 * 
 * 
 * 
 * 
 */
extern Uint16 UdcRatio;
extern Uint16 CntOZdelay;
void System_Init(void)
{
//全局变量初始化
	counterMainISR = 0;			// 主中断计数器
	UdcRatio = 207;
	CntOZdelay = 0;
	ErrorCode = 0;
	RunCmd = CMD_STOP;
	SewMode = SEW_MODE_FREE;
	SewState = SEW_STATE_IDLE;
	flagMotorState = MOTOR_STATE_STOP;
	HmiRunCmd = 0;


#if (MACHINE_WR580BS||MACHINE_WR587BS)
	//I2cAngle = 268;//
	I2cAngle = 4;
#else
	I2cAngle = 252;
#endif

	SpeedPedalZoom = 100;

	flagPedalTest.all = 0;
	flagThetaTest.all = 0;
	flagPower.all = 0;
	flagSew.all = 0;
	flagPos.all = 0;
	flagStitch.all = 0;
	CountStitch = 0;
	flagSet.all = 0;

	hmi_common_vars.run_forbid = 0;
	hmi_common_vars.age_start = 0;
	hmi_common_vars.test_angle = 0;
	hmi_common_vars.test_pedal = 0;

//==============参数读取======================
	if(EEPROM_ParaRead(PARA_BUF) == FALSE)		//EEPROM版本不对
	{
		EEPROM_ParaRecover(PARA_BUF);
	}

//==========================================

//	Initialize ADC module

//  - 传感器接法 【2.5 ±(0.625/6A)*I】*0.6 (中点在1.5v, 0.6 = 1.5/2.5)
//  - 即电流采样倍数 = 6/(0.625*0.6) = 16
	adc_meas1.ImeasAGain = _IQ13(BASE_ADCREF/(BASE_CURRENT*2/16));
	adc_meas1.ImeasBGain = adc_meas1.ImeasAGain;
//        - 电压采样倍数 = 207
//	temp = BASE_VDC / UdcRatio;
//	temp = BASE_ADCREF / temp;
//	adc_meas1.VdcMeasGain = temp * 8192;		//Q13
	adc_meas1.VdcMeasGain = _IQ13(BASE_ADCREF/(BASE_VDC/UdcRatio));	
		
	adc_meas1.init(&adc_meas1);
	if(adc_meas1.Error == 1)
	{
		ErrorCode = ERR_CURROFF;
	}

//	Initialize SVGEN_DQ module 最小脉宽限制
	svgen_dq1.Tlimit = _IQ(1.0 - (SVPWM_DEADTIME+SVPWM_LIMITIME) * SVPWM_FREQUENCY * 2 / 1000);

//	Initialize PWM module
	pwm1.PeriodMax = SVPWM_PERIOD;// /2;  // Perscaler X1 (T1), ISR period = T x 1
	pwm1.init(&pwm1); 

//	Initialize enable drive module    
//	drv1.init(&drv1);

//	Initialize QEP module
	qep1.LineEncoder = ENC_RESOLUTION;
	qep1.MechScaler = _IQ30(0.25/qep1.LineEncoder);
	qep1.PolePairs = MOTOR_POLE/2;
	qep1.ElecTheta = 0;
	qep1.MechTheta = 0;
	qep1.TotalEnc = 0;
	qep1.CntOld = 0;
	qep1.CntUpdate = 0;
	qep1.init(&qep1);

//	Initialize the Speed module for QEP based speed calculation
	speed1.K1 = _IQ(1/(BASE_SPEED/60*T*NUM_CNT_1MS*ENC_RESOLUTION*4));
	speed1.BaseRpm = BASE_SPEED;
	speed1.SpeedLow = 450;			//低速测试阈值 rpm
	speed1.SpeedHigh = 1800;		//高速测试阈值 rpm

//	Initialize the feedforward_spd module for QEP based speed calculation
	feedforward_torque.K = 16;

//	Initialize the feedforward_spd module for QEP based speed calculation
	feedforward_spd.K = _IQ13(1/(BASE_SPEED/60*T*NUM_CNT_1MS*ENC_RESOLUTION*4));
	feedforward_spd.Kp = _IQ(0.5);//前馈50%

//	Initialize the PID_REG3 module for Id 
	pid1_id.Kp = _IQ(2.5);			//_IQ(0.75);  2.5
	pid1_id.Ki = _IQ(T/0.0016);		//_IQ(T/0.0005); 0.0016
	pid1_id.Kd = _IQ(0/T);
	pid1_id.Kc = _IQ(0.05);	//0.2
	pid1_id.OutMax = _IQ(1.0);		//_IQ(0.30);
	pid1_id.OutMin = _IQ(-1.0);		//_IQ(-0.30);
 
//	Initialize the PID_REG3 module for Iq 
	pid1_iq.Kp = _IQ(2.5);			//_IQ(0.75);	2.5
	pid1_iq.Ki = _IQ(T/0.0016);		//_IQ(T/0.0005);	0.0016
	pid1_iq.Kd = _IQ(0/T);
	pid1_iq.Kc = _IQ(0.05);	//0.2
	pid1_iq.OutMax = _IQ(1.0);		//_IQ(0.95);
	pid1_iq.OutMin = _IQ(-1.0);		//_IQ(-0.95);

//	Initialize the PID_REG3 module for speed
	pid1_spd.Kp = _IQ(2.5);			//_IQ(1);                  
	pid1_spd.Ki = _IQ(T*NUM_CNT_1MS/0.2);	//0.3
	pid1_spd.Kd = _IQ(0/(T*NUM_CNT_1MS));
	pid1_spd.Kc = _IQ(0.05);	//越大调节平缓 否则激烈
	pid1_spd.OutMax = _IQ(12.0/BASE_CURRENT);		//12A
	pid1_spd.OutMin = _IQ(-12.0/BASE_CURRENT); 		//12A

//	Initialize the PID_REG3 module for pos2
	pid1_pos2.Kp = _IQ(0.20/256);					// 0.5 (停针效果)               
	pid1_pos2.Ki = _IQ(0);//T*NUM_CNT_1MS/0.6/256);	//0.6 (停针效果)
	pid1_pos2.Kd = _IQ(0/(T*NUM_CNT_1MS));
	pid1_pos2.Kc = _IQ(0);	//0.001				//(过大容易造成PID计算溢出)
//	pid1_pos2.OutMax = _IQ(300.0/BASE_SPEED);
//	pid1_pos2.OutMin = _IQ(-300.0/BASE_SPEED);
#if(MACHINE_WR580BS && MACHINE_MODE_SHULIAO)
	pid1_pos2.OutMax = _IQ(200.0/BASE_SPEED);
	pid1_pos2.OutMin = _IQ(-200.0/BASE_SPEED);
#else
	pid1_pos2.OutMax = _IQ(300.0/BASE_SPEED);//_IQ(200.0/BASE_SPEED);塑料手轮
	pid1_pos2.OutMin = _IQ(-300.0/BASE_SPEED);//_IQ(-200.0/BASE_SPEED);塑料手轮
#endif

//	Initialize the RMPCNTL module
	rc2.RampDelayMax = 1;				//每周期操作
	rc2.Resolution = _IQ(1.0/300);		//300*1ms 到 BASE_SPEED


}//end of System_Init()
/*
*/
Uint16 AngleInitSearch(void)
{	
	#define DELAY_TIME		8		//ms

	#define TEST_CURRENT_ONE	4.0			//第一次搜寻位置电流
	#define TEST_CURRENT_TWO	6.0			//第二次搜寻位置电流
	#define TEST_CURRENT_THREE	8.0			//第三次搜寻位置电流

	#define STALL_ANGLE			1.5														//堵转角度
	#define STALL_P				((Uint16)(STALL_ANGLE * (ENC_RESOLUTION * 4) / 360))	//堵转脉冲数

	#define SECTOR_ANGLE		11														//初始扇区调整角度
	#define SECTOR_P			((Uint16)(SECTOR_ANGLE * (ENC_RESOLUTION * 4) / 360))	//扇区调整的角度
	
	int32 temp = 0;
	static Uint16 ErrCounter = 0;		//零位寻找出错的计数器
	
	#define WAIT_FOR_STOP_TIME		10	//等待电机停止时间,ms
	static Uint16 delaytimer = 0;		//延时计数器
	
/*	if(AngleInit.AngleInitFlag == 3)
	{
		AngleInit.AngleInitFlag = 1;		//搜索结束
		flagMotorState = MOTOR_STATE_STOP;
		AngleInit.AngleResult = qep1.CalibratedAngle;
		pid3_clear(&pid1_iq);	//清iq电流环运算量
		pid3_clear(&pid1_id);	//清id电流环运算量
		return TRUE;
	}*/
	switch(AngleInit.step)
	{
		case 0:		//第一阶段
			{
				pid3_clear(&pid1_iq);	//清iq电流环运算量
				pid3_clear(&pid1_id);	//清id电流环运算量
				qep1.ElecTheta = 0;
				qep1.TotalEnc = 0;
				qep1.CntOld = 0;
				qep1.CntUpdate = 0;
				EQep1Regs.QPOSCNT = 0;

				AngleInit.AngleInitFlag = 2;
				AngleInit.Cnt = AngleInit.CntOld = EQep1Regs.QPOSCNT;
				if(ErrCounter == 0)
				{
					AngleInit.Id_Ref = _IQ(TEST_CURRENT_ONE/BASE_CURRENT);	//4A电流
				}
				else if(ErrCounter == 1)
				{
					AngleInit.Id_Ref = _IQ(TEST_CURRENT_TWO/BASE_CURRENT);	//6A电流
				}
				else if(ErrCounter == 2)
				{
					AngleInit.Id_Ref = _IQ(TEST_CURRENT_THREE/BASE_CURRENT);	//8A电流
				}
				AngleInit.ElecTheta = 0;									//0度
				AngleInit.Counter = 0;
				flagMotorState = MOTOR_STATE_RUN;	
				AngleInit.step = 1;
				break;
			}
		case 1:
			{
				AngleInit.Counter++;
				if(AngleInit.Counter >= DELAY_TIME)
				{
					//AngleInit.Counter = DELAY_TIME;
					AngleInit.Cnt = EQep1Regs.QPOSCNT;
					//AngleInit.Cnt = qep1.CntUpdate;
					AngleInit.DeltaCnt1 = AngleInit.Cnt - AngleInit.CntOld;
					
					AngleInit.Cnt = AngleInit.CntOld = EQep1Regs.QPOSCNT;
					AngleInit.Counter = 0;
					if(AngleInit.DeltaCnt1 > 0)		//180~360,加270电流矢量
					{
						AngleInit.ElecTheta = _IQ(270.0/360.0);									//270度						
					}
					else				//<=0,加+90的电流矢量,0~180
					{
						AngleInit.ElecTheta = _IQ(90.0/360.0);									//90度
					}
					AngleInit.step = 11;
				}
				break;
			}
		case 11:
			{
				AngleInit.Counter++;
				if(AngleInit.Counter >= DELAY_TIME)
				{
					AngleInit.Counter = DELAY_TIME;
					
					flagMotorState = MOTOR_STATE_STOP;
					AngleInit.Counter = DELAY_TIME;
					AngleInit.Cnt = EQep1Regs.QPOSCNT;
					AngleInit.DeltaCnt2 = AngleInit.Cnt - AngleInit.CntOld;
					
					if((abs(AngleInit.DeltaCnt2) <= STALL_P)&&(abs(AngleInit.DeltaCnt1) <= STALL_P))		//两个矢量产生的脉冲都比较少，堵转
					{
						if(ErrCounter == 2)			//三次失败报错处理
						{
							ErrorCode = ERR_STALL;
							AngleInit.AngleInitFlag = 0;		//搜索失败
							return TRUE;
						}
						else
						{
							delaytimer++;
							if(delaytimer > WAIT_FOR_STOP_TIME)
							{
								delaytimer = WAIT_FOR_STOP_TIME;
								if(AngleInit.Cnt == EQep1Regs.QPOSCNT)		//电机静止
								{
									ErrCounter++;
									delaytimer = 0;
									AngleInit.step = 0;
									AngleInit.Cnt = 0;
									AngleInit.CntOld = 0;
									AngleInit.DeltaCnt1 = 0;
									AngleInit.DeltaCnt2 = 0;
									AngleInit.Counter = 0;
								}
								else
								{
									AngleInit.Cnt = EQep1Regs.QPOSCNT;	//等待停止的时间内刷新编码器计数器
									delaytimer = 0;
								}
							}
						}
					}
					else
					{
						if(AngleInit.DeltaCnt1 > 0)			//3，4象限
						{
							AngleInit.step = 3;
						}
						else		//1，2象限
						{
							AngleInit.step = 2;
						}
					}
				}
				break;
			}
		case 2:				//0~180
			{
				if(AngleInit.DeltaCnt1 == 0)		//有可能在0度或者180度附近
				{
					if(AngleInit.DeltaCnt2 < 0)			//180度附近，减掉走过的角度就是真正的角度
					{
						temp = __qmpy32by16(qep1.MechScaler,(int16)AngleInit.DeltaCnt2,31);
						temp = qep1.PolePairs * temp;
						temp = temp << 9;
						AngleInit.AngleResult = _IQ24(180.0/360) + temp;
						AngleInit.AngleResult &= 0x00FFFFFF; 
						flagMotorState = MOTOR_STATE_STOP;
						return TRUE;
					}
					else			//0度附近，加走过的角度就是真正的角度
					{
						temp = __qmpy32by16(qep1.MechScaler,(int16)AngleInit.DeltaCnt2,31);
						temp = qep1.PolePairs * temp;
						temp = temp << 9;
						AngleInit.AngleResult = _IQ24(0/360) + temp;
						AngleInit.AngleResult &= 0x00FFFFFF; 
						flagMotorState = MOTOR_STATE_STOP;
						return TRUE;
					}
				}
				else
				{
					//判断1，2象限
					//1求加速度
					AngleInit.V1 = AngleInit.A1 = AngleInit.DeltaCnt1 * 2;
					AngleInit.V2 = AngleInit.DeltaCnt2 * 2 - AngleInit.V1;
					AngleInit.A2 = AngleInit.V2 - AngleInit.V1;
					
					AngleInit.A1_abs = abs(AngleInit.A1);
					AngleInit.A2_abs = abs(AngleInit.A2);
					
					if(AngleInit.A2 < 0)		//第二象限
					{
						if(AngleInit.A2_abs < AngleInit.A1_abs)		// A2<A1
						{
							temp = AngleInit.A2_abs + (AngleInit.A2_abs >> 1);
							if(AngleInit.A1_abs < temp)
							{
								if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)		//22,180线，小于11度
								{
									//初始角取135
									AngleInit.AngleResult = _IQ24(135.0/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
								else
								{
									//初始角取112.5
									AngleInit.AngleResult = _IQ24(112.5/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
							}
							else		//90~135
							{
								if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)		//22,180线，小于11度
								{
									//初始角取112.5
									AngleInit.AngleResult = _IQ24(112.5/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
								else
								{
									//初始角取90
									AngleInit.AngleResult = _IQ24(90.0/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
							}
						}
						else
						{
							temp = AngleInit.A1_abs + (AngleInit.A1_abs >> 1);
							if(AngleInit.A2_abs < temp)		//A2<1.5*A1
							{
								if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)		//22,180线，小于11度
								{
									//初始角取135
									AngleInit.AngleResult = _IQ24(135.0/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
								else		//大于45度，要跳一个扇区
								{
									//初始角取112.5度
									AngleInit.AngleResult = _IQ24(112.5/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
							}
							else	//A2>=1.5*A1
							{
								if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)		//22,180线，小于11度
								{
									//初始角取157.5
									AngleInit.AngleResult = _IQ24(157.5/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
								else		//大于45度，要跳一个扇区
								{
									//初始角取135度
									AngleInit.AngleResult = _IQ24(135.0/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
							}
						}
					}
					else		//第一象限
					{
						if(AngleInit.A2_abs < AngleInit.A1_abs)		// A2<A1
						{
							temp = AngleInit.A2_abs + (AngleInit.A2_abs >> 1);
							if(AngleInit.A1_abs < temp)		//A1<1.5*A2
							{
								//初始角度45
								AngleInit.AngleResult = _IQ24(45.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//初始角度67.5
								AngleInit.AngleResult = _IQ24(67.5/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
						}
						else
						{
							temp = AngleInit.A1_abs + (AngleInit.A1_abs >> 1);
							if(AngleInit.A2_abs < temp)		//A2<1.5*A1
							{
								//初始角度45
								AngleInit.AngleResult = _IQ24(45.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//初始角度22.5
								AngleInit.AngleResult = _IQ24(22.5/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
						}
					}
				}
				//break;
			}
		case 3:
			{
				//1求加速度
				AngleInit.V1 = AngleInit.A1 = AngleInit.DeltaCnt1 * 2;
				AngleInit.V2 = AngleInit.DeltaCnt2 * 2 - AngleInit.V1;
				AngleInit.A2 = AngleInit.V2 - AngleInit.V1;
				
				AngleInit.A1_abs = abs(AngleInit.A1);
				AngleInit.A2_abs = abs(AngleInit.A2);
				if(AngleInit.A2 < 0)		//第四象限
				{
					if(AngleInit.A2_abs < AngleInit.A1_abs)		// A2<A1
					{
						temp = AngleInit.A2_abs + (AngleInit.A2_abs >> 1);
						if(AngleInit.A1_abs < temp)		//A1<1.5*A2
						{
							//315
							AngleInit.AngleResult = _IQ24(315.0/360);
							flagMotorState = MOTOR_STATE_STOP;
							return TRUE;
						}
						else
						{
							//292.5
							AngleInit.AngleResult = _IQ24(292.5/360);
							flagMotorState = MOTOR_STATE_STOP;
							return TRUE;
						}
					}
					else
					{
						temp = AngleInit.A1_abs + (AngleInit.A1_abs >> 1);
						if(AngleInit.A2_abs < temp)		//A2<1.5*A1
						{
							//315
							AngleInit.AngleResult = _IQ24(315.0/360);
							flagMotorState = MOTOR_STATE_STOP;
							return TRUE;
						}
						else
						{
							//337.5
							AngleInit.AngleResult = _IQ24(337.5/360);
							flagMotorState = MOTOR_STATE_STOP;
							return TRUE;
						}
					}
				}
				else		//第三象限
				{
					if(AngleInit.A2_abs < AngleInit.A1_abs)		// A2<A1
					{
						temp = AngleInit.A2_abs + (AngleInit.A2_abs >> 1);
						if(AngleInit.A1_abs < temp)		//A1<1.5*A2
						{
							if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)
							{
								//247.5
								AngleInit.AngleResult = _IQ24(247.5/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//225
								AngleInit.AngleResult = _IQ24(225.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
						}
						else
						{
							if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)
							{
								//247.5
								AngleInit.AngleResult = _IQ24(247.5/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//270
								AngleInit.AngleResult = _IQ24(270.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
						}
					}
					else
					{
						temp = AngleInit.A1_abs + (AngleInit.A1_abs >> 1);
						if(AngleInit.A2_abs < temp)		//A2<1.5*A1
						{
							if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)
							{
								//初始角度225
								AngleInit.AngleResult = _IQ24(225.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//初始角度247.5
								AngleInit.AngleResult = _IQ24(247.5/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
						}
						else
						{
							if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)
							{
								//初始角度202.5
								AngleInit.AngleResult = _IQ24(202.5/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//初始角度225
								AngleInit.AngleResult = _IQ24(225.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
						}
					}
				}	
			}
		default:
			{
				break;
			}
	}
	return FALSE;
}

/**
 * GetSysInfo
 * 
 * 
 * 
 * 
 */
void GetSysInfo(void)
{
	static int32 IqFilter1 = 0;
	static int32 IqFilter2 = 0;
	static int32 SpdFilter = 0;
	static Uint32 HmiDataDlay = 0;
	static Uint32 FTKGCounter = 0;

	IqFilter1 += (park1.Qs - IqFilter1)>>3;					// 滤波 弱
	IqFilter2 += (park1.Qs - IqFilter2)>>6;					// 滤波 强
	SpdFilter += (SpeedMeas.Speed - SpdFilter)>>6;			// 滤波

	SysInfo.Vdc = _IQ15mpy(adc_meas1.VdcMeas, BASE_VDC);
	SysInfo.Iq = _IQmpy(IqFilter1, BASE_CURRENT*100);
	SysInfo.IqLed = abs(_IQmpy(IqFilter2, BASE_CURRENT*10/1.414));	//有效值
	SysInfo.Speed = abs(_IQmpy(SpdFilter, BASE_SPEED));
	SysInfo.MechTheta = (qep1.MechThetaDegree>=MechThetaUpSet)?(qep1.MechThetaDegree-MechThetaUpSet):(qep1.MechThetaDegree+360-MechThetaUpSet);
	SysInfo.PedalAD = ((Uint32)(adc_meas1.pedalAD)*300)>>15;

	// 翻抬开关信号读取
/*	if(((flagSet.bit.FTKGmode == 0) && (FTKG_ISON)) || \
	   ((flagSet.bit.FTKGmode == 1) && (!FTKG_ISON)))
	{
		flagSew.bit.Fantai = 1;		// 翻抬抬起
	}
	else
	{
		flagSew.bit.Fantai = 0;		// 翻抬放下
	}
*/
	if(flagSet.bit.FTKGmode == 2)		//如果翻台开关禁止，则不检测信号
	{
		flagSew.bit.Fantai = 0;		// 翻抬放下
	}
	else
	{
		if(flagSew.bit.Fantai == 0)	//如果翻台开关信号本来是关闭的
		{
			//翻台开关模式0,1
			if(((flagSet.bit.FTKGmode == 0) && (FTKG_ISON)) || \
				((flagSet.bit.FTKGmode == 1) && (!FTKG_ISON)))
			{	
				FTKGCounter++;
			}
			else 
			{
				FTKGCounter = 0;
			}

			if(FTKGCounter > 200)	//50ms
			{
				flagSew.bit.Fantai = 1;
				FTKGCounter = 0;
			}
		}
		else	//本来翻台开关是打开的 =1
		{
			//翻台开关模式0,1
			if(((flagSet.bit.FTKGmode == 0) && (!FTKG_ISON)) || \
				((flagSet.bit.FTKGmode == 1) && (FTKG_ISON)))
			{	
				FTKGCounter++;
			}
			else 
			{
				FTKGCounter = 0;
			}

			if(FTKGCounter > 200)	//50ms
			{
				flagSew.bit.Fantai = 0;
				FTKGCounter = 0;
			}
		}
	}
	#if FTKG_EN
	hmi_common_vars.FTKG = flagSew.bit.Fantai;		//翻台开关状态
	#else
	hmi_common_vars.FTKG = 0;
	#endif
	HmiDataDlay++;
	if(HmiDataDlay == 20)
	{
		hmi_common_vars.motor_spd = SysInfo.Speed;
		HmiDataDlay = 0;
	}
	hmi_common_vars.motor_spd = SysInfo.Speed;
	hmi_common_vars.board_cur = SysInfo.IqLed;
	hmi_common_vars.board_vol = SysInfo.Vdc;
	hmi_common_vars.motor_agl = I2cAngle;
	hmi_common_vars.error_code = ErrorCode;
	hmi_common_vars.motor = flagMotorState;			//电机状态
	if(PedaleCapCounter.NowCounter == 0)
	{
		hmi_common_vars.pedal_KHz = 0;
	}
	else
	{
		hmi_common_vars.pedal_KHz = CPU_FREQUENCY * 1000000uL / PedaleCapCounter.NowCounter;		//HZ
	}
}//end of GetSysInfo()


/**
 * PoffDetect
 * 掉电检测
 * 
 * 
 *
 */
void PoffDetect(void)
{
	if(CL_ISON)
	{
		flagPower.bit.cntCL++;				// 在Oz中断中清零
		if(flagPower.bit.cntCL > 200) 		// 200ms = 20ms(50Hz)*10次
		{
			flagPower.bit.cntCL = 200;
			CL_OFF;							// 关继电器
			flagPower.bit.Poff = 1;			// 掉电
		}
		else
		{
			flagPower.bit.Poff = 0;
		}
	}

}//end of PoffDetect()


/**
 * FaultDetect
 * 系统故障1ms定时检测
 * 
 * 
 * 输出：ErrorCode
 */
void FaultDetect(void)
{
	static Uint16 cntPoff = 0;
	static Uint16 cntOC = 0;
	static Uint16 cntVdc0 = 0;
	static Uint16 cntVdc1 = 0;
	static Uint16 cntVdc11 = 0;
	static Uint16 cntStall = 0;
	static Uint16 cntOverSpd = 0;
	static Uint16 cntNegSpd = 0;


//PoFF故障判断处理=======================
	if(ErrorCode == ERR_POFF)
	{
		if (flagPower.bit.Poff == 0)
		{
			cntPoff++;
			if(cntPoff > 10)
			{
				cntPoff = 0;
				ErrorCode = ErrorCodeOld;
			}
		}
		else
		{
			cntPoff = 0;
		}
	}
	else 
	{
		if (flagPower.bit.Poff == 1)
		{
			cntPoff++;
			if(cntPoff > 10)
			{
				cntPoff = 0;
				ErrorCodeOld = ErrorCode;
				ErrorCode = ERR_POFF;
				flagSew.bit.PedalUnlock = 0;	//封锁踏板
			}
		}
		else
		{
			cntPoff = 0;
		}
	}

//其他故障判断=======================
	if(ErrorCode != 0)
	{//已有其他故障，不再检测
		return;						//已有故障后不再检测
	}
//OZ信号丢失检测
	CntOZdelay++;					//在Oz中断中清零
	if(CntOZdelay > 1000)
	{
		CntOZdelay = 0;
		ErrorCode = ERR_OZ;			//持续1S未进OZ中断 ,可认为OZ故障
		return;
	}
//软件过流检测
	if(abs(SysInfo.Iq) > I_MAX)
	{
		cntOC++;
		if(cntOC > 10)
		{
			cntOC = 0;
			ErrorCode = ERR_OC;		//持续10MS大于IMAX,可认为软件过流故障
			return;
		}
	}
	else
	{
		cntOC = 0;
	}
//过压、欠压检测
	if(flagMotorState == MOTOR_STATE_STOP)
	{//待机
		if(SysInfo.Vdc < UDC_LOW0)
		{
			cntVdc0++;
			if(cntVdc0 > 3000)
			{
				cntVdc0 = 0;
				ErrorCode = ERR_LVSTOP;		//母线电压持续3秒<UDC_LOW0,可认为欠压故障
				return;
			}
		}
		else if(SysInfo.Vdc > UDC_MAX0)
		{
			cntVdc0++;
			if(cntVdc0 > 3000)
			{
				cntVdc0 = 0;
				ErrorCode = ERR_OVSTOP;		//母线电压持续3秒>UDC_MAX0,可认为过压故障
				return;
			}
		}
		else
		{
			cntVdc0 = 0;
		}
	}
	else
	{//运行
		if(SysInfo.Vdc < UDC_LOW1)
		{
			cntVdc1++;
			if(cntVdc1 > 10)
			{
				cntVdc1 = 0;
				ErrorCode = ERR_LVRUN;		//母线电压持续10ms<UDC_LOW1,可认为欠压故障
				return;
			}
		}
		else if(SysInfo.Vdc > UDC_MAX0)
		{
			cntVdc11++;
			if(cntVdc11 > 3000)
			{
				cntVdc11 = 0;
				ErrorCode = ERR_OVSTOP;		//母线电压持续3秒>UDC_MAX0,可认为过压故障
				return;
			}
			if(SysInfo.Vdc > UDC_MAX1)
			{
				cntVdc1++;
				if(cntVdc1 > 10)
				{
					cntVdc1 = 0;
					ErrorCode = ERR_OVRUN;		//母线电压持续10ms>UDC_MAX1,可认为过压故障
					return;
				}
			}
		}
		else
		{
			cntVdc1 = 0;
			cntVdc11 = 0;
		}
	}
//电机超负荷(堵转)检测
	if(flagMotorState == MOTOR_STATE_RUN)		//EnableFlag
	{
		if((SpeedMeas.SpeedRpm<25) || (abs(SysInfo.Iq) > 1000))
		{//速度低于25rpm或电流大于10A
			cntStall++;
			if(cntStall>2000)
			{
				cntStall = 0;
				ErrorCode = ERR_STALL;		//持续2s 堵转故障
				return;
			}
		}
		else
		{
			cntStall = 0;
		}
	}
	else
	{
		cntStall = 0;
	}
//电机运转异常(�)检测
	if(SpeedMeas.SpeedRpm>(SpeedMax+SPD_MAXERR))
	{
		cntOverSpd++;
		if(cntOverSpd>20)
		{
			cntOverSpd = 0;
			ErrorCode = ERR_OSPD;		//电机速度持续20ms>SPD_MAX+SPD_MAXERR,可认为超速故障
			return;
		}
	}
	else
	{
		cntOverSpd = 0;
	}
//电机运转异常(反转)检测
	if((flagMotorState == MOTOR_STATE_RUN) && (flagThetaTest.bit.TestEn == 0))
	{//待机、测电机初始角时不检测
		if(MotorDir ^ SpeedMeas.Direction)
		{
			if(SpeedMeas.SpeedRpm>750)
			{
				cntNegSpd++;
				if(cntNegSpd>100)
				{
					cntNegSpd = 0;
					ErrorCode = ERR_NSPD;		//电机速度持�100ms反转750rpm,可认为反转故障
					return;
				}
			}
			else
			{
				cntNegSpd = 0;
			}
		}
		else
		{
			cntNegSpd = 0;
		}
	}
	else
	{
		cntNegSpd = 0;
	}

}//end of FaultDetect()


/**
 * Motor_Overload
 * 电机过载故障故障检测
 * 
 * 
 * 
 */
const Uint16 TABLE_MOTOR_OVERLOAD[8][2] = 
{
//	时间0.1s	斜率(tn-tn')/9
	36000,		2000,		//114%In 60分钟过载
	18000,		1333,		//123%In 30分钟过载
	 6000,		 333,		//132%In 10分钟过载
	 3000,		 200,		//141%In  5分钟过载
	 1200,		  72,		//150%In  2分钟过载
	  550,		  28,		//159%In   55秒过载
	  300,		  22,		//168%In   30秒过载
	  100,		   0,		//177%In   10秒过载
};
#define IQ_RATED	4.0		//额度电流(A)
void Motor_Overload(void)
{
	static int16 cntMsDelay = 0;		//计数器 ms
	static int32 cntMotorDelay = 0;		//保护时间计数 (0~36000)*0.1s
	Uint16 IqOverload;					//电流(Iq/In*100)
	Uint16 TimeTabled;					//曲线计算值 (0~36000)*0.1s
	Uint16 temp1,temp2;

	if(ErrorCode != 0)
	{
		return;							//已有故障后不再检测
	}
	cntMsDelay++;
	if(cntMsDelay < 100)
	{
		return;							//不到0.1s跳出
	}
	cntMsDelay = 0;

	IqOverload = abs(SysInfo.Iq) / IQ_RATED;
	if(IqOverload < 114)
	{//低载不检查
		cntMotorDelay -= 360;			//36000/100
		if(cntMotorDelay < 0)
		{
			cntMotorDelay = 0;
		}
	}
	else
	{//过载检查 114~
 		if(IqOverload > 177)
		{
			IqOverload = 177;			//限定在114~195
		}
		temp1 = (IqOverload-114)/9;		//商	表中点间隔9
		temp2 = (IqOverload-114)%9;		//余数
		TimeTabled = TABLE_MOTOR_OVERLOAD[temp1][0] - TABLE_MOTOR_OVERLOAD[temp1][1] * temp2;
		cntMotorDelay += 36000 / TimeTabled;
		if(cntMotorDelay > 36000)
		{
			cntMotorDelay = 36000;
			ErrorCode = ERR_OVERLOAD;	//电机过载故障
		}
	}

}//end of Motor_Overload()


/**
 * Synchro_Detect
 * 同步信号故障检测
 * 
 * 
 * 
 */
void Synchro_Detect(void)
{
	if(ErrorCode != 0)
	{
		return;							//已有故障后不再检测
	}
//UP/DN信号检测-----------------------
	if(SewMode == SEW_MODE_SIMPLE || flagMotorState == MOTOR_STATE_STOP)
	{//简易缝模式、测电机初始角，不检测停针信号(up/dn)故障
		Synchro.cntUP0 = 0;
		Synchro.cntUP1 = 0;
	}
	else if(SpeedMeas.SpeedRpm > 50)
	{//电平计时
		if(UP==1)
		{
			Synchro.cntUP1++;
		}
		else
		{
			Synchro.cntUP0++;
		}
	}
	if((Synchro.cntUP0>1000) || (Synchro.cntUP1>1000))
	{
		ErrorCode = ERR_SYNCHRO;	//转速50RPM以上时持续1s为高或低，报停针信号故障
	}

}//end of Synchro_Detect()


/**
 * Synchro_Clear
 * 同步信号计数清零
 * 
 * 
 * 
 */
void Synchro_Clear(void)
{
	if(UP==1)
	{
		Synchro.cntUP0 = 0;
	}
	else
	{
		Synchro.cntUP1 = 0;
	}

/*	if(DN==1)
	{
		Synchro.cntDN0 = 0;
	}
	else
	{
		Synchro.cntDN1 = 0;
	}
*/
	if(HALL_A==1)
	{
		Synchro.cntHallA0 = 0;
	}
	else
	{
		Synchro.cntHallA1 = 0;
	}

}//end of Synchro_Clear()


/**
 * Brake()
 * 刹车放电子程序
 * 
 * 
 * 
 */
void Brake(void)
{
//刹车处理
	if((flagMotorState == MOTOR_STATE_RUN) && \
	   (SpeedMeas.SpeedRpm > SPEED_DB) && \
	   (((MotorDir == MOTOR_DIR_POS) && (pid1_spd.Out<0)) || \
	    ((MotorDir == MOTOR_DIR_NEG) && (pid1_spd.Out>0))))
	{//滞环处理
		if(SysInfo.Vdc > (UDC_DB+20))
		{
			flagBrake = 5;
		}
		else if(SysInfo.Vdc > (UDC_DB+10))
		{
			flagBrake = 4;
		}
		else if(SysInfo.Vdc > UDC_DB)
		{
			flagBrake = 3;
		}
		else if((SysInfo.Vdc < (UDC_DB-30)) && (flagBrake == 1))
		{
			flagBrake = 0;
		}
		else if((SysInfo.Vdc < (UDC_DB-25)) && (flagBrake == 2))
		{
			flagBrake = 1;
		}
		else if((SysInfo.Vdc < (UDC_DB-20)) && (flagBrake >= 3))
		{
			flagBrake = 2;
		}
	}
	else
	{
		flagBrake = 0;		//待机时或速度低于SPEED_DB不开制动
	}
}//end of Brake()

/**
 * FaultProcess()
 * 故障保护处理及淞壳辶�
 * 
 * 
 * 
 */
void FaultProcess(void)
{
	if(ErrorCode != 0)
	{//有故障
		if(ErrorCode == ERR_POFF)
		{//Poff故障
			
		}
		flagMotorState = MOTOR_STATE_STOP;
		PWM_EN_OFF;				// 电机使能关
	}

}//end of FaultProcess()


/**
 * Speed_Detect
 * 速度检测
 * 
 * 
 * 
 */
void Speed_Detect(void)
{
	speed1.Enc1ms = abs(qep1.EncSpeed);
	qep1.EncSpeed = 0;						//计数清零
	speed1.Direction = qep1.DirectionQep;	//转向赋值
	speed1.calc(&speed1);					//速度计算

	SpeedMeas.Direction = qep1.DirectionQep;
	SpeedMeas.Speed = speed1.Speed;
	SpeedMeas.SpeedRpm = speed1.SpeedRpm;

}//end of Speed_Detect()


/**
 * Pos_P_Reg
 * 位置环P调节
 *
 *
 *
 */
void Pos_P_Reg(void)
{
	int16 i;
	int32 SpeedRef_P_FIR;

	if(flagMotorState == MOTOR_STATE_RUN)
	{
		pid1_pos.PosErr_CLSW = 0;
	}
	else
	{
		pid1_pos.PosErr_CLSW = 1;
	}

	pid1_pos.TempSpeed = SpeedRef_P;

	//16次滑动求平均值
	for(i=15; i>0; i--)
	{
		pid1_pos.buffer[i] = pid1_pos.buffer[i-1];
	}
	pid1_pos.buffer[0] = pid1_pos.TempSpeed;
	pid1_pos.sum = pid1_pos.remainder;
	for(i=0; i<16; i++)
	{
		pid1_pos.sum += pid1_pos.buffer[i];			//求和
	}
	pid1_pos.remainder = pid1_pos.sum&0x000f;
	SpeedRef_P_FIR = pid1_pos.sum>>4;			//平均值
	pid1_pos.fir_err += (pid1_pos.TempSpeed - SpeedRef_P_FIR);

	pid1_pos.ErrMax = _IQ11mpyIQX(_IQmpy(_IQ((SpeedMax+200)/BASE_SPEED),_IQ(1)-feedforward_spd.Kp),GLOBAL_Q,_IQ16div(_IQ16(ENC_RESOLUTION),_IQtoIQ16(_IQ30toIQ(pid1_pos.Kp))),16);
	pid1_pos.Ref = SpeedRef_P_FIR;
	pid1_pos.Fdb = (qep1.EncPos<<11);
	pid1_pos.calc(&pid1_pos);

	feedforward_spd.In = SpeedRef_P_FIR;
	feedforward_spd.calc(&feedforward_spd);

}//end of Pos_P_Reg()


/**
 * Speed_PI_Reg
 * 速度环PI调节
 * 
 * 
 * 
 */
void Speed_PI_Reg(void)
{
	_iq SpeedR,SpeedL,SpeedH;
	_iq KpL,KpH,KiL,KiH;

	if(MotorDir == MOTOR_DIR_POS)
	{//正转
		pid1_spd.OutMax = _IQ( 14.0/BASE_CURRENT);
		pid1_spd.OutMin = _IQ(-10.0/BASE_CURRENT);
		pid1_spd.Ref = pid1_pos.Out + feedforward_spd.Out;
	}
	else
	{//反转
		pid1_spd.OutMax = _IQ( 10.0/BASE_CURRENT);
		pid1_spd.OutMin = _IQ(-14.0/BASE_CURRENT);
		pid1_spd.Ref = - pid1_pos.Out - feedforward_spd.Out;
	}
	pid1_spd.Ref = _IQsat(pid1_spd.Ref, pid1_pos.OutMax, pid1_pos.OutMin);	/* Saturate the Ref */
	pid1_spd.Fdb = SpeedMeas.Speed;

//高低速PI参数曲线计算
	SpeedR = _IQabs(pid1_spd.Ref);
	SpeedL = _IQ(MachinePos.spd_Low/BASE_SPEED);			//300rpm
	SpeedH = _IQ(MachinePos.spd_High/BASE_SPEED);			//2000rpm
	KpL = MachinePos.spd_KpL;
	KpH = MachinePos.spd_KpH;
	KiL = MachinePos.spd_KiL;
	KiH = MachinePos.spd_KiH;

	if(SpeedR < SpeedL)
	{
		pid1_spd.Kp = KpL;
		pid1_spd.Ki = KiL;
	}
	else if(SpeedR > SpeedH)
	{
		pid1_spd.Kp = KpH;
		pid1_spd.Ki = KiH;
	}
	else
	{
		pid1_spd.Kp = KpL + _IQdiv(_IQmpy((SpeedR-SpeedL),(KpH-KpL)),(SpeedH-SpeedL));
		pid1_spd.Ki = KiL + _IQdiv(_IQmpy((SpeedR-SpeedL),(KiH-KiL)),(SpeedH-SpeedL));
	}

//速度环PI调节
	pid1_spd.calc(&pid1_spd);	//速度环PID调节

	feedforward_torque.In = pid1_spd.Ref;
	feedforward_torque.calc(&feedforward_torque);

//下针加力处理
	if((SewMode != SEW_MODE_SIMPLE) && (flagMotorState == MOTOR_STATE_RUN) && \
	   (speed1.SpeedRpm < 450) && (flagPos.bit.Step != STEP_PLAST_CAL) && \
	   (qep1.MechThetaDegree > (((MechThetaUpSet+5)>=360)?(MechThetaUpSet+5-360):(MechThetaUpSet+5))) 
	   && (qep1.MechThetaDegree < (((MechThetaUpSet+60)>=360)?(MechThetaUpSet+60-360):(MechThetaUpSet+60))))
	{
		flagFeedForwardDn = 1;
	}
	else
	{
		flagFeedForwardDn = 0;
	}

}//end of Speed_PI_Reg()


/**
 * SaveDataPoff
 * 掉电保存数据处理
 * 
 * 
 * 
 */
extern interrupt void MainISR(void);
extern interrupt void SecondaryISR(void);
extern interrupt void epwm1_tzint_isr(void);
extern interrupt void xint1_isr(void);
extern interrupt void xint2_isr(void);
extern interrupt void xint3_isr(void);
extern interrupt void i2c_int1a_isr(void);
#pragma CODE_SECTION(SaveDataPoff, "ramfuncs");
void SaveDataPoff(void)
{
	Uint16 i = 0;
	Uint16 buf[PAR_ADV_DEF_SIZE] = {0,};
	static Uint32 Delay = 0;		//写入数据需要延时，保证能显示poff
	#define EEPROM_INIT				0
	#define EEPROM_START			1
	#define EEPROM_WORK				2
	#define EEPROM_DELAY			3
	static Uint16 EEPROM_Step = EEPROM_INIT;

	switch(EEPROM_Step)
	{
		case EEPROM_INIT:
		{
			if(flagPower.bit.Poff == 1 && hmi_common_vars.ParFlg == 1)	//有掉电信号且参数表有改动
			{
				EEPROM_Step = EEPROM_START;
			}
			break;
		}
		case EEPROM_START:
		{
			//读现有的参数表
			//DINT;
			ParaRead(buf);
			//与内存中的参数表对比
			for(;i<(PAR_ADV_DEF_SIZE-1);i++)
			{
				if(buf[i] != par_adv_value[i])
				{
					EEPROM_Step = EEPROM_DELAY;
					break;
				}
			}
			if(EEPROM_Step != EEPROM_WORK)		//如果参数一样，就把步骤清零，参数更改标志位清零
			{
				hmi_common_vars.ParFlg = 0;
				EEPROM_Step = EEPROM_DELAY;
			}
			//EINT;
			break;
		}
		case EEPROM_DELAY:
		{
			if(ErrorCode == ERR_POFF)
			{
				Delay++;
				if(Delay > 10)
				{
					EEPROM_Step = EEPROM_WORK;
				}
			}
			break;
		}
		case EEPROM_WORK:		//慈�
		{
			//计算校验和
			par_adv_value[(PAR_ADV_DEF_SIZE-1)] = 0;
			for(i=0;i<(PAR_ADV_DEF_SIZE-1);i++)
			{
				par_adv_value[(PAR_ADV_DEF_SIZE-1)] += par_adv_value[i];
			}
			//写入参数表
			if(ParaWrite() == FALSE)
			{
				ErrorCode = ERR_EEP;
			}
			else	//校验参数
			{
				ParaRead(buf);
				for(i=0;i<PAR_ADV_DEF_SIZE;i++)
				{
					if(buf[i] != par_adv_value[i])
					{
						ErrorCode = ERR_EEP;
						break;
					}
				}
				hmi_common_vars.ParFlg = 0;
			}
			Delay = 0;
			EEPROM_Step = EEPROM_INIT;
			for(;;)
			{
				if(OZ == 1)
				{
					//延时1ms
					DELAY_US(1000);
					//再检测是否有信号
					if(OZ == 0)//poff cancelled,reset DSP
					{
						EALLOW;						// Enable the watchdog
						SysCtrlRegs.WDCR = 0x0028;	// 溢出时间：2^8*512*2^(bit210-1)/20M = 6.5536 ms
						EDIS;

						EALLOW;						// Enable the watchdog
						SysCtrlRegs.WDCR = 0x0000;	// 溢出时间：2^8*512*2^(bit210-1)/20M = 6.5536 ms
						EDIS;
					}
					else	
					{
						CL_OFF;							// 关继电器
						flagPower.bit.Poff = 1;			// 掉电
					}
				}
			}
		}
	}
}//end of SaveDataPoff()


/**
 * ModeParaUpdate
 * 系统参数解释刷新
 * 
 * 
 * 
 */
void ModeParaUpdate(void)
{
	static Uint16 Flag = 0;		//0,第一次更新

	if(flagMotorState == MOTOR_STATE_RUN)
	{//运行时需要解释的参数在此处理
		SpeedPedalMax =			PARA_BUF[0] * SEWPARA_DEFAULT[0].v_step;			//用户设定最高缝纫速度
		return;
	}

//==================解释参数数据=============================
	if(Flag == 0)
	{
		flagSet.bit.Updn = 			PARA_BUF[ADDR_PARA_PON];
		flagSet.bit.SoftStartEn = 	PARA_BUF[ADDR_PARA_SOFTSTART_EN];

		hmi_common_vars.ndl_stop = flagSet.bit.Updn;
		hmi_common_vars.soft_start = flagSet.bit.SoftStartEn;
		hmi_common_vars.ndl_light = PARA_BUF[P112_NDL_LIGHT];
		Flag = 1;
	}
	else
	{
		flagSet.bit.Updn = hmi_common_vars.ndl_stop;
		flagSet.bit.SoftStartEn = hmi_common_vars.soft_start;

		PARA_BUF[ADDR_PARA_PON] = flagSet.bit.Updn;
		PARA_BUF[ADDR_PARA_SOFTSTART_EN] = flagSet.bit.SoftStartEn;
	}

	SpeedPedalMax =			PARA_BUF[0] * SEWPARA_DEFAULT[0].v_step;			//用户设定最高缝纫速度

	StitchSoftStart =		PARA_BUF[1] * SEWPARA_DEFAULT[1].v_step;			//软启动针数
//	SpeedAuto =				bufSewPara[3] * SEWPARA_DEFAULT[3].v_step;			// 定长缝最高速

	SpeedSoftStart1 =		PARA_BUF[2] * SEWPARA_DEFAULT[2].v_step;
	SpeedSoftStart2 =		PARA_BUF[3] * SEWPARA_DEFAULT[3].v_step;
	SpeedSoftStart3 =		PARA_BUF[4] * SEWPARA_DEFAULT[4].v_step;

	AccTime = PARA_BUF[5] * 20;
	DecTime = PARA_BUF[6] * 20;

	flagSet.bit.Go2Up =		PARA_BUF[8];										//上电找上针位
	flagSet.bit.FTKGmode = 	PARA_BUF[9];										// 0 常开; 1 常闭 

	SpeedPedalMin =			200;	// 踏板最低速,根据机型码来
	optPedalCurve =			PARA_BUF[10] * SEWPARA_DEFAULT[10].v_step;			//踏板曲线

#if(PEDAL_TYPE == PEDAL_L)
/*	PedaleCapCounter.Zero = PedalECapCounter_Cal(PARA_BUF[24]);
	PedaleCapCounter.Run = 	PedalECapCounter_Cal((PARA_BUF[24]-100+PARA_BUF[25]));
	PedaleCapCounter.Acc = 	PedalECapCounter_Cal((PARA_BUF[24]-100+PARA_BUF[26]));
	PedaleCapCounter.Max = 	PedalECapCounter_Cal((PARA_BUF[24]-100+PARA_BUF[27]));
	PedaleCapCounter.SetUp = 	PedalECapCounter_Cal((PARA_BUF[24]-100+PARA_BUF[28]));*/
	PedaleCapCounter.Zero = PedalKHz_Cal(PARA_BUF[24]);
	PedaleCapCounter.Run = 	PedalKHz_Cal((PARA_BUF[24]-100+PARA_BUF[25]));
	PedaleCapCounter.Acc = 	PedalKHz_Cal((PARA_BUF[24]-100+PARA_BUF[26]));
	PedaleCapCounter.Max = 	PedalKHz_Cal((PARA_BUF[24]-100+PARA_BUF[27]));
	PedaleCapCounter.SetUp = 	PedalKHz_Cal((PARA_BUF[24]-100+PARA_BUF[28]));
#else
	PedalMoniAD.Zero = 	PedalMoniAD_Cal(PARA_BUF[24],PEDAL_MONI_P_K1,PEDAL_MONI_P_K2); 
	PedalMoniAD.Run = 	PedalMoniAD_Cal((PARA_BUF[24]-100+PARA_BUF[25]),PEDAL_MONI_P_K1,PedalMoniAD.Zero);
	PedalMoniAD.Acc = 	PedalMoniAD_Cal((PARA_BUF[24]-100+PARA_BUF[26]),PEDAL_MONI_P_K1,PedalMoniAD.Zero);
	PedalMoniAD.Max = 	PedalMoniAD_Cal((PARA_BUF[24]-100+PARA_BUF[27]),PEDAL_MONI_P_K1,PedalMoniAD.Zero);
	PedalMoniAD.SetUp = PedalMoniAD_Cal((PARA_BUF[24]-100+PARA_BUF[28]),PEDAL_MONI_P_K1,PedalMoniAD.Zero);
#endif
	MechThetaDnSet =		177;	//下停针角
	flagSet.bit.optFZTZ =	0;		//反转提针使能
	MechThetaFZTZ =			20;		// 反转提针角度

	SpeedMax =				PARA_BUF[20] * SEWPARA_DEFAULT[20].v_step;						// 系统最高速		
	CurrentFeedForwardDn = _IQ(PARA_BUF[21] * SEWPARA_DEFAULT[21].v_step/BASE_CURRENT/2); 	//加重功能

	MechThetaStop = (PARA_BUF[22] * SEWPARA_DEFAULT[22].v_step * 3) >> 1;		//停针补偿角度

	MechThetaUpSet = ((MECHTHETA_UP + MechThetaStop)>=360)?((MECHTHETA_UP + MechThetaStop)-360):(MECHTHETA_UP + MechThetaStop);
	RatioWheel = 1000;	
	
	AutoTestMode = 			PARA_BUF[29];	//老化模式
	SpeedAutoTest =			PARA_BUF[30] * SEWPARA_DEFAULT[30].v_step;							//自动测试速度
	TimeAutoTest = 			PARA_BUF[32] * SEWPARA_DEFAULT[32].v_step * 100;				// 自动测试停顿时间
	TimeAutoRun = 			PARA_BUF[31] * SEWPARA_DEFAULT[31].v_step * 100;				//自动测试运行时间
	StitchAutoTest = 		PARA_BUF[33];
	
//	StitchAutoTest = 		bufSewPara[97] * SEWPARA_DEFAULT[97].v_step;

	//和手轮参数有关变量
#if((MACHINE_WR580BS||MACHINE_WR587BS) && MACHINE_MODE_SHULIAO)
	MotorDir = MOTOR_DIR_POS;
	MachinePos.CurveStop = 1;
	rc2.Resolution = _IQ(1.0/300);
	pid1_pos.Kp = _IQ30(0.36);				//位置环	//0.36 44叠厚
	pid1_pos2.Kp = _IQ(0.40/256);			//停针位置环//0.40 44叠厚
	MachinePos.spd_Low = 300;
	MachinePos.spd_High = 3000;
	MachinePos.spd_KpL = _IQ(1.5);			//2.0 44叠厚
	MachinePos.spd_KiL = _IQ(T*NUM_CNT_1MS/0.20);
	MachinePos.spd_KpH = _IQ(1.0);
	MachinePos.spd_KiH = _IQ(T*NUM_CNT_1MS/0.40);
	MachinePos.spd_stop = 300;
	MachinePos.thetaRef = 40;
	MachinePos.acc1 = 4500/0.1;
	MachinePos.acc2 = 2000/0.1;
	MachinePos.thetaPos =  1.0;
	MachinePos.StopTime1 = 30;
	MachinePos.StopTime2 = 150;
#elif((MACHINE_WR580BS||MACHINE_WR587BS) && MACHINE_MODE_TIE)
	MotorDir = MOTOR_DIR_POS;
	MachinePos.CurveStop = 1;
	rc2.Resolution = _IQ(1.0/300);
	pid1_pos.Kp = _IQ30(0.36);
	MachinePos.spd_Low = 300;
	MachinePos.spd_High = 3000;
	MachinePos.spd_KpL = _IQ(2.0);
	MachinePos.spd_KiL = _IQ(T*NUM_CNT_1MS/0.20);
	MachinePos.spd_KpH = _IQ(1.0);
	MachinePos.spd_KiH = _IQ(T*NUM_CNT_1MS/0.40);
	pid1_pos2.Kp = _IQ(0.40/256);
	MachinePos.spd_stop = 400;
	MachinePos.thetaRef = 40;
	MachinePos.acc1 = 2000/0.1;
	MachinePos.acc2 = 1500/0.1;
	MachinePos.thetaPos =  1.0;
	MachinePos.StopTime1 = 30;
	MachinePos.StopTime2 = 200;
#else
	MotorDir = MOTOR_DIR_POS;
	MachinePos.CurveStop = 1;
	rc2.Resolution = _IQ(1.0/300);
	pid1_pos.Kp = _IQ30(0.34);				//位置环	//0.36 44叠厚
	pid1_pos2.Kp = _IQ(0.40/256);			//停针位置环//0.40 44叠厚
	MachinePos.spd_Low = 300;
	MachinePos.spd_High = 3000;
	MachinePos.spd_KpL = _IQ(1.0);			//2.0 44叠厚
	MachinePos.spd_KiL = _IQ(T*NUM_CNT_1MS/0.22);
	MachinePos.spd_KpH = _IQ(1.0);
	MachinePos.spd_KiH = _IQ(T*NUM_CNT_1MS/0.40);
	MachinePos.spd_stop = 300;
	MachinePos.thetaRef = 40;
	MachinePos.acc1 = 2000/0.1;
	MachinePos.acc2 = 1500/0.1;
	MachinePos.thetaPos =  1.0;
	MachinePos.StopTime1 = 30;
	MachinePos.StopTime2 = 150;
#endif

	MachinePos.MechThetaPin1 = 45;
	MachinePos.MechThetaPin2 = 195;
	MachinePos.MechThetaFeed = 75;	//75~255
	MachinePos.MechThetaStitchSet = 210;
	MachinePos.MechThetaStitchClear = 30;

//==================解释缝纫模式数据=============================
	if(PARA_BUF[7] == 1)
	{//简易缝参数设定有效
		SewMode = SEW_MODE_SIMPLE;
		flagSew.bit.Sewed = 0;
	}
	else
	{
		SewMode = SEW_MODE_FREE;
		flagSet.bit.OneShot = 0;
	}
	
}//end of ModeParaUpdate()



/**
 * PedalMoniAD_Cal
 * 模拟踏板参数转换
 * 转换角度值到电压值
 * 
 * 
 */
int16 PedalMoniAD_Cal(int16 para, _iq scale, _iq offset)
{
	return (_IQ15mpy(scale,_IQ15((para-100)/10)) + offset);

}//end of PedalMoniAD_Cal()

/**
*电感踏板的行程转到频率计数值
*
*
*/
#if(PEDAL_TYPE == PEDAL_L)
/*Uint32 PedalECapCounter_Cal(int16 length)
{
	//return(PEDAL_ZERO_COUNTER + PEDAL_KP1_COUNTER*(length-100)/10);
	Uint32 KHZ_Q15 = 0;
	int32 temp = 0;
	temp = ((int32)PEDAL_KP1_HZ_Q15)*((int32)(length-100))/10;
	KHZ_Q15 = PEDAL_ZERO_HZ_Q15 + temp;
	return((PEDAL_BASE_COUNTER*32768L)/KHZ_Q15);
}
*/
Uint32 PedalKHz_Cal(int16 deta_KHZ)
{
	//return(PEDAL_ZERO_COUNTER + PEDAL_KP1_COUNTER*(length-100)/10);  //-13,6.16
	int32 KHZ_1000 = (deta_KHZ-100) * 20;		//0.02 * 1000
	Uint32 temp = PEDAL_HZ1000_TO_Q15(KHZ_1000) + PEDAL_ZERO_HZ_Q15;
	
	return temp;
}
//length=0.5202*(CPU_FREQUENCY / (1000*A))^3 
//- 12.23*(CPU_FREQUENCY / (1000*A))^2 
//+ 100.72*(CPU_FREQUENCY / (1000*A))
//-274.97
/*#define K1			(0.5202*(CPU_FREQUENCY * 1000000 / 1000)*(CPU_FREQUENCY * 1000000 / 1000)*(CPU_FREQUENCY * 1000000 / 1000))
Uint32 PedalECapLength(Uint32 PedalCounter)
{
}*/
#endif
//===========================================================================
// No more.
//===========================================================================
