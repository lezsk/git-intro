/**
 * Copyright (c) 2010, ÉÏº£»ýÖÂµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾
 * All rights reserved.
 *
 * @file Functions.c
 * »ù±¾×Óº¯Êý´¦ÀíÎÄ¼þ.
 * °üº¬ÊäÈëÃüÁî´¦Àí£¬Ì¤°åËÙ¶È´¦Àíº¯Êý
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
//È«¾Ö±äÁ¿³õÊ¼»¯
	counterMainISR = 0;			// Ö÷ÖÐ¶Ï¼ÆÊýÆ÷
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

//==============²ÎÊý¶ÁÈ¡======================
	if(EEPROM_ParaRead(PARA_BUF) == FALSE)		//EEPROM°æ±¾²»¶Ô
	{
		EEPROM_ParaRecover(PARA_BUF);
	}

//==========================================

//	Initialize ADC module

//  - ´«¸ÐÆ÷½Ó·¨ ¡¾2.5 ¡À(0.625/6A)*I¡¿*0.6 (ÖÐµãÔÚ1.5v, 0.6 = 1.5/2.5)
//  - ¼´µçÁ÷²ÉÑù±¶Êý = 6/(0.625*0.6) = 16
	adc_meas1.ImeasAGain = _IQ13(BASE_ADCREF/(BASE_CURRENT*2/16));
	adc_meas1.ImeasBGain = adc_meas1.ImeasAGain;
//        - µçÑ¹²ÉÑù±¶Êý = 207
//	temp = BASE_VDC / UdcRatio;
//	temp = BASE_ADCREF / temp;
//	adc_meas1.VdcMeasGain = temp * 8192;		//Q13
	adc_meas1.VdcMeasGain = _IQ13(BASE_ADCREF/(BASE_VDC/UdcRatio));	
		
	adc_meas1.init(&adc_meas1);
	if(adc_meas1.Error == 1)
	{
		ErrorCode = ERR_CURROFF;
	}

//	Initialize SVGEN_DQ module ×îÐ¡Âö¿íÏÞÖÆ
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
	speed1.SpeedLow = 450;			//µÍËÙ²âÊÔãÐÖµ rpm
	speed1.SpeedHigh = 1800;		//¸ßËÙ²âÊÔãÐÖµ rpm

//	Initialize the feedforward_spd module for QEP based speed calculation
	feedforward_torque.K = 16;

//	Initialize the feedforward_spd module for QEP based speed calculation
	feedforward_spd.K = _IQ13(1/(BASE_SPEED/60*T*NUM_CNT_1MS*ENC_RESOLUTION*4));
	feedforward_spd.Kp = _IQ(0.5);//Ç°À¡50%

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
	pid1_spd.Kc = _IQ(0.05);	//Ô½´óµ÷½ÚÆ½»º ·ñÔò¼¤ÁÒ
	pid1_spd.OutMax = _IQ(12.0/BASE_CURRENT);		//12A
	pid1_spd.OutMin = _IQ(-12.0/BASE_CURRENT); 		//12A

//	Initialize the PID_REG3 module for pos2
	pid1_pos2.Kp = _IQ(0.20/256);					// 0.5 (Í£ÕëÐ§¹û)               
	pid1_pos2.Ki = _IQ(0);//T*NUM_CNT_1MS/0.6/256);	//0.6 (Í£ÕëÐ§¹û)
	pid1_pos2.Kd = _IQ(0/(T*NUM_CNT_1MS));
	pid1_pos2.Kc = _IQ(0);	//0.001				//(¹ý´óÈÝÒ×Ôì³ÉPID¼ÆËãÒç³ö)
//	pid1_pos2.OutMax = _IQ(300.0/BASE_SPEED);
//	pid1_pos2.OutMin = _IQ(-300.0/BASE_SPEED);
#if(MACHINE_WR580BS && MACHINE_MODE_SHULIAO)
	pid1_pos2.OutMax = _IQ(200.0/BASE_SPEED);
	pid1_pos2.OutMin = _IQ(-200.0/BASE_SPEED);
#else
	pid1_pos2.OutMax = _IQ(300.0/BASE_SPEED);//_IQ(200.0/BASE_SPEED);ËÜÁÏÊÖÂÖ
	pid1_pos2.OutMin = _IQ(-300.0/BASE_SPEED);//_IQ(-200.0/BASE_SPEED);ËÜÁÏÊÖÂÖ
#endif

//	Initialize the RMPCNTL module
	rc2.RampDelayMax = 1;				//Ã¿ÖÜÆÚ²Ù×÷
	rc2.Resolution = _IQ(1.0/300);		//300*1ms µ½ BASE_SPEED


}//end of System_Init()
/*
*/
Uint16 AngleInitSearch(void)
{	
	#define DELAY_TIME		8		//ms

	#define TEST_CURRENT_ONE	4.0			//µÚÒ»´ÎËÑÑ°Î»ÖÃµçÁ÷
	#define TEST_CURRENT_TWO	6.0			//µÚ¶þ´ÎËÑÑ°Î»ÖÃµçÁ÷
	#define TEST_CURRENT_THREE	8.0			//µÚÈý´ÎËÑÑ°Î»ÖÃµçÁ÷

	#define STALL_ANGLE			1.5														//¶Â×ª½Ç¶È
	#define STALL_P				((Uint16)(STALL_ANGLE * (ENC_RESOLUTION * 4) / 360))	//¶Â×ªÂö³åÊý

	#define SECTOR_ANGLE		11														//³õÊ¼ÉÈÇøµ÷Õû½Ç¶È
	#define SECTOR_P			((Uint16)(SECTOR_ANGLE * (ENC_RESOLUTION * 4) / 360))	//ÉÈÇøµ÷ÕûµÄ½Ç¶È
	
	int32 temp = 0;
	static Uint16 ErrCounter = 0;		//ÁãÎ»Ñ°ÕÒ³ö´íµÄ¼ÆÊýÆ÷
	
	#define WAIT_FOR_STOP_TIME		10	//µÈ´ýµç»úÍ£Ö¹Ê±¼ä,ms
	static Uint16 delaytimer = 0;		//ÑÓÊ±¼ÆÊýÆ÷
	
/*	if(AngleInit.AngleInitFlag == 3)
	{
		AngleInit.AngleInitFlag = 1;		//ËÑË÷½áÊø
		flagMotorState = MOTOR_STATE_STOP;
		AngleInit.AngleResult = qep1.CalibratedAngle;
		pid3_clear(&pid1_iq);	//ÇåiqµçÁ÷»·ÔËËãÁ¿
		pid3_clear(&pid1_id);	//ÇåidµçÁ÷»·ÔËËãÁ¿
		return TRUE;
	}*/
	switch(AngleInit.step)
	{
		case 0:		//µÚÒ»½×¶Î
			{
				pid3_clear(&pid1_iq);	//ÇåiqµçÁ÷»·ÔËËãÁ¿
				pid3_clear(&pid1_id);	//ÇåidµçÁ÷»·ÔËËãÁ¿
				qep1.ElecTheta = 0;
				qep1.TotalEnc = 0;
				qep1.CntOld = 0;
				qep1.CntUpdate = 0;
				EQep1Regs.QPOSCNT = 0;

				AngleInit.AngleInitFlag = 2;
				AngleInit.Cnt = AngleInit.CntOld = EQep1Regs.QPOSCNT;
				if(ErrCounter == 0)
				{
					AngleInit.Id_Ref = _IQ(TEST_CURRENT_ONE/BASE_CURRENT);	//4AµçÁ÷
				}
				else if(ErrCounter == 1)
				{
					AngleInit.Id_Ref = _IQ(TEST_CURRENT_TWO/BASE_CURRENT);	//6AµçÁ÷
				}
				else if(ErrCounter == 2)
				{
					AngleInit.Id_Ref = _IQ(TEST_CURRENT_THREE/BASE_CURRENT);	//8AµçÁ÷
				}
				AngleInit.ElecTheta = 0;									//0¶È
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
					if(AngleInit.DeltaCnt1 > 0)		//180~360,¼Ó270µçÁ÷Ê¸Á¿
					{
						AngleInit.ElecTheta = _IQ(270.0/360.0);									//270¶È						
					}
					else				//<=0,¼Ó+90µÄµçÁ÷Ê¸Á¿,0~180
					{
						AngleInit.ElecTheta = _IQ(90.0/360.0);									//90¶È
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
					
					if((abs(AngleInit.DeltaCnt2) <= STALL_P)&&(abs(AngleInit.DeltaCnt1) <= STALL_P))		//Á½¸öÊ¸Á¿²úÉúµÄÂö³å¶¼±È½ÏÉÙ£¬¶Â×ª
					{
						if(ErrCounter == 2)			//Èý´ÎÊ§°Ü±¨´í´¦Àí
						{
							ErrorCode = ERR_STALL;
							AngleInit.AngleInitFlag = 0;		//ËÑË÷Ê§°Ü
							return TRUE;
						}
						else
						{
							delaytimer++;
							if(delaytimer > WAIT_FOR_STOP_TIME)
							{
								delaytimer = WAIT_FOR_STOP_TIME;
								if(AngleInit.Cnt == EQep1Regs.QPOSCNT)		//µç»ú¾²Ö¹
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
									AngleInit.Cnt = EQep1Regs.QPOSCNT;	//µÈ´ýÍ£Ö¹µÄÊ±¼äÄÚË¢ÐÂ±àÂëÆ÷¼ÆÊýÆ÷
									delaytimer = 0;
								}
							}
						}
					}
					else
					{
						if(AngleInit.DeltaCnt1 > 0)			//3£¬4ÏóÏÞ
						{
							AngleInit.step = 3;
						}
						else		//1£¬2ÏóÏÞ
						{
							AngleInit.step = 2;
						}
					}
				}
				break;
			}
		case 2:				//0~180
			{
				if(AngleInit.DeltaCnt1 == 0)		//ÓÐ¿ÉÄÜÔÚ0¶È»òÕß180¶È¸½½ü
				{
					if(AngleInit.DeltaCnt2 < 0)			//180¶È¸½½ü£¬¼õµô×ß¹ýµÄ½Ç¶È¾ÍÊÇÕæÕýµÄ½Ç¶È
					{
						temp = __qmpy32by16(qep1.MechScaler,(int16)AngleInit.DeltaCnt2,31);
						temp = qep1.PolePairs * temp;
						temp = temp << 9;
						AngleInit.AngleResult = _IQ24(180.0/360) + temp;
						AngleInit.AngleResult &= 0x00FFFFFF; 
						flagMotorState = MOTOR_STATE_STOP;
						return TRUE;
					}
					else			//0¶È¸½½ü£¬¼Ó×ß¹ýµÄ½Ç¶È¾ÍÊÇÕæÕýµÄ½Ç¶È
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
					//ÅÐ¶Ï1£¬2ÏóÏÞ
					//1Çó¼ÓËÙ¶È
					AngleInit.V1 = AngleInit.A1 = AngleInit.DeltaCnt1 * 2;
					AngleInit.V2 = AngleInit.DeltaCnt2 * 2 - AngleInit.V1;
					AngleInit.A2 = AngleInit.V2 - AngleInit.V1;
					
					AngleInit.A1_abs = abs(AngleInit.A1);
					AngleInit.A2_abs = abs(AngleInit.A2);
					
					if(AngleInit.A2 < 0)		//µÚ¶þÏóÏÞ
					{
						if(AngleInit.A2_abs < AngleInit.A1_abs)		// A2<A1
						{
							temp = AngleInit.A2_abs + (AngleInit.A2_abs >> 1);
							if(AngleInit.A1_abs < temp)
							{
								if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)		//22,180Ïß£¬Ð¡ÓÚ11¶È
								{
									//³õÊ¼½ÇÈ¡135
									AngleInit.AngleResult = _IQ24(135.0/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
								else
								{
									//³õÊ¼½ÇÈ¡112.5
									AngleInit.AngleResult = _IQ24(112.5/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
							}
							else		//90~135
							{
								if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)		//22,180Ïß£¬Ð¡ÓÚ11¶È
								{
									//³õÊ¼½ÇÈ¡112.5
									AngleInit.AngleResult = _IQ24(112.5/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
								else
								{
									//³õÊ¼½ÇÈ¡90
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
								if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)		//22,180Ïß£¬Ð¡ÓÚ11¶È
								{
									//³õÊ¼½ÇÈ¡135
									AngleInit.AngleResult = _IQ24(135.0/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
								else		//´óÓÚ45¶È£¬ÒªÌøÒ»¸öÉÈÇø
								{
									//³õÊ¼½ÇÈ¡112.5¶È
									AngleInit.AngleResult = _IQ24(112.5/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
							}
							else	//A2>=1.5*A1
							{
								if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)		//22,180Ïß£¬Ð¡ÓÚ11¶È
								{
									//³õÊ¼½ÇÈ¡157.5
									AngleInit.AngleResult = _IQ24(157.5/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
								else		//´óÓÚ45¶È£¬ÒªÌøÒ»¸öÉÈÇø
								{
									//³õÊ¼½ÇÈ¡135¶È
									AngleInit.AngleResult = _IQ24(135.0/360);
									flagMotorState = MOTOR_STATE_STOP;
									return TRUE;
								}
							}
						}
					}
					else		//µÚÒ»ÏóÏÞ
					{
						if(AngleInit.A2_abs < AngleInit.A1_abs)		// A2<A1
						{
							temp = AngleInit.A2_abs + (AngleInit.A2_abs >> 1);
							if(AngleInit.A1_abs < temp)		//A1<1.5*A2
							{
								//³õÊ¼½Ç¶È45
								AngleInit.AngleResult = _IQ24(45.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//³õÊ¼½Ç¶È67.5
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
								//³õÊ¼½Ç¶È45
								AngleInit.AngleResult = _IQ24(45.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//³õÊ¼½Ç¶È22.5
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
				//1Çó¼ÓËÙ¶È
				AngleInit.V1 = AngleInit.A1 = AngleInit.DeltaCnt1 * 2;
				AngleInit.V2 = AngleInit.DeltaCnt2 * 2 - AngleInit.V1;
				AngleInit.A2 = AngleInit.V2 - AngleInit.V1;
				
				AngleInit.A1_abs = abs(AngleInit.A1);
				AngleInit.A2_abs = abs(AngleInit.A2);
				if(AngleInit.A2 < 0)		//µÚËÄÏóÏÞ
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
				else		//µÚÈýÏóÏÞ
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
								//³õÊ¼½Ç¶È225
								AngleInit.AngleResult = _IQ24(225.0/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//³õÊ¼½Ç¶È247.5
								AngleInit.AngleResult = _IQ24(247.5/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
						}
						else
						{
							if(abs(AngleInit.DeltaCnt1 + AngleInit.DeltaCnt2) < SECTOR_P)
							{
								//³õÊ¼½Ç¶È202.5
								AngleInit.AngleResult = _IQ24(202.5/360);
								flagMotorState = MOTOR_STATE_STOP;
								return TRUE;
							}
							else
							{
								//³õÊ¼½Ç¶È225
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

	IqFilter1 += (park1.Qs - IqFilter1)>>3;					// ÂË²¨ Èõ
	IqFilter2 += (park1.Qs - IqFilter2)>>6;					// ÂË²¨ Ç¿
	SpdFilter += (SpeedMeas.Speed - SpdFilter)>>6;			// ÂË²¨

	SysInfo.Vdc = _IQ15mpy(adc_meas1.VdcMeas, BASE_VDC);
	SysInfo.Iq = _IQmpy(IqFilter1, BASE_CURRENT*100);
	SysInfo.IqLed = abs(_IQmpy(IqFilter2, BASE_CURRENT*10/1.414));	//ÓÐÐ§Öµ
	SysInfo.Speed = abs(_IQmpy(SpdFilter, BASE_SPEED));
	SysInfo.MechTheta = (qep1.MechThetaDegree>=MechThetaUpSet)?(qep1.MechThetaDegree-MechThetaUpSet):(qep1.MechThetaDegree+360-MechThetaUpSet);
	SysInfo.PedalAD = ((Uint32)(adc_meas1.pedalAD)*300)>>15;

	// ·­Ì§¿ª¹ØÐÅºÅ¶ÁÈ¡
/*	if(((flagSet.bit.FTKGmode == 0) && (FTKG_ISON)) || \
	   ((flagSet.bit.FTKGmode == 1) && (!FTKG_ISON)))
	{
		flagSew.bit.Fantai = 1;		// ·­Ì§Ì§Æð
	}
	else
	{
		flagSew.bit.Fantai = 0;		// ·­Ì§·ÅÏÂ
	}
*/
	if(flagSet.bit.FTKGmode == 2)		//Èç¹û·­Ì¨¿ª¹Ø½ûÖ¹£¬Ôò²»¼ì²âÐÅºÅ
	{
		flagSew.bit.Fantai = 0;		// ·­Ì§·ÅÏÂ
	}
	else
	{
		if(flagSew.bit.Fantai == 0)	//Èç¹û·­Ì¨¿ª¹ØÐÅºÅ±¾À´ÊÇ¹Ø±ÕµÄ
		{
			//·­Ì¨¿ª¹ØÄ£Ê½0,1
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
		else	//±¾À´·­Ì¨¿ª¹ØÊÇ´ò¿ªµÄ =1
		{
			//·­Ì¨¿ª¹ØÄ£Ê½0,1
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
	hmi_common_vars.FTKG = flagSew.bit.Fantai;		//·­Ì¨¿ª¹Ø×´Ì¬
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
	hmi_common_vars.motor = flagMotorState;			//µç»ú×´Ì¬
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
 * µôµç¼ì²â
 * 
 * 
 *
 */
void PoffDetect(void)
{
	if(CL_ISON)
	{
		flagPower.bit.cntCL++;				// ÔÚOzÖÐ¶ÏÖÐÇåÁã
		if(flagPower.bit.cntCL > 200) 		// 200ms = 20ms(50Hz)*10´Î
		{
			flagPower.bit.cntCL = 200;
			CL_OFF;							// ¹Ø¼ÌµçÆ÷
			flagPower.bit.Poff = 1;			// µôµç
		}
		else
		{
			flagPower.bit.Poff = 0;
		}
	}

}//end of PoffDetect()


/**
 * FaultDetect
 * ÏµÍ³¹ÊÕÏ1ms¶¨Ê±¼ì²â
 * 
 * 
 * Êä³ö£ºErrorCode
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


//PoFF¹ÊÕÏÅÐ¶Ï´¦Àí=======================
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
				flagSew.bit.PedalUnlock = 0;	//·âËøÌ¤°å
			}
		}
		else
		{
			cntPoff = 0;
		}
	}

//ÆäËû¹ÊÕÏÅÐ¶Ï¦À=======================
	if(ErrorCode != 0)
	{//ÒÑÓÐÆäËû¹ÊÕÏ£¬²»ÔÙ¼ì²â
		return;						//ÒÑÓÐ¹ÊÕÏºó²»ÔÙ¼ì²â
	}
//OZÐÅºÅ¶ªÊ§¼ì²â
	CntOZdelay++;					//ÔÚOzÖÐ¶ÏÖÐÇåÁã
	if(CntOZdelay > 1000)
	{
		CntOZdelay = 0;
		ErrorCode = ERR_OZ;			//³ÖÐø1SÎ´½øOZÖÐ¶Ï ,¿ÉÈÏÎªOZ¹ÊÕÏ
		return;
	}
//Èí¼þ¹ýÁ÷¼ì²â
	if(abs(SysInfo.Iq) > I_MAX)
	{
		cntOC++;
		if(cntOC > 10)
		{
			cntOC = 0;
			ErrorCode = ERR_OC;		//³ÖÐø10MS´óÓÚIMAX,¿ÉÈÏÎªÈí¼þ¹ýÁ÷¹ÊÕÏ
			return;
		}
	}
	else
	{
		cntOC = 0;
	}
//¹ýÑ¹¡¢Ç·Ñ¹¼ì²â
	if(flagMotorState == MOTOR_STATE_STOP)
	{//´ý»ú
		if(SysInfo.Vdc < UDC_LOW0)
		{
			cntVdc0++;
			if(cntVdc0 > 3000)
			{
				cntVdc0 = 0;
				ErrorCode = ERR_LVSTOP;		//Ä¸ÏßµçÑ¹³ÖÐø3Ãë<UDC_LOW0,¿ÉÈÏÎªÇ·Ñ¹¹ÊÕÏ
				return;
			}
		}
		else if(SysInfo.Vdc > UDC_MAX0)
		{
			cntVdc0++;
			if(cntVdc0 > 3000)
			{
				cntVdc0 = 0;
				ErrorCode = ERR_OVSTOP;		//Ä¸ÏßµçÑ¹³ÖÐø3Ãë>UDC_MAX0,¿ÉÈÏÎª¹ýÑ¹¹ÊÕÏ
				return;
			}
		}
		else
		{
			cntVdc0 = 0;
		}
	}
	else
	{//ÔËÐÐ
		if(SysInfo.Vdc < UDC_LOW1)
		{
			cntVdc1++;
			if(cntVdc1 > 10)
			{
				cntVdc1 = 0;
				ErrorCode = ERR_LVRUN;		//Ä¸ÏßµçÑ¹³ÖÐø10ms<UDC_LOW1,¿ÉÈÏÎªÇ·Ñ¹¹ÊÕÏ
				return;
			}
		}
		else if(SysInfo.Vdc > UDC_MAX0)
		{
			cntVdc11++;
			if(cntVdc11 > 3000)
			{
				cntVdc11 = 0;
				ErrorCode = ERR_OVSTOP;		//Ä¸ÏßµçÑ¹³ÖÐø3Ãë>UDC_MAX0,¿ÉÈÏÎª¹ýÑ¹¹ÊÕÏ
				return;
			}
			if(SysInfo.Vdc > UDC_MAX1)
			{
				cntVdc1++;
				if(cntVdc1 > 10)
				{
					cntVdc1 = 0;
					ErrorCode = ERR_OVRUN;		//Ä¸ÏßµçÑ¹³ÖÐø10ms>UDC_MAX1,¿ÉÈÏÎª¹ýÑ¹¹ÊÕÏ
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
//µç»ú³¬¸ººÉ(¶Â×ª)¼ì²â
	if(flagMotorState == MOTOR_STATE_RUN)		//EnableFlag
	{
		if((SpeedMeas.SpeedRpm<25) || (abs(SysInfo.Iq) > 1000))
		{//ËÙ¶ÈµÍÓÚ25rpm»òµçÁ÷´óÓÚ10A
			cntStall++;
			if(cntStall>2000)
			{
				cntStall = 0;
				ErrorCode = ERR_STALL;		//³ÖÐø2s ¶Â×ª¹ÊÕÏ
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
//µç»úÔË×ªÒì³£(¬ËÙ)¼ì²â
	if(SpeedMeas.SpeedRpm>(SpeedMax+SPD_MAXERR))
	{
		cntOverSpd++;
		if(cntOverSpd>20)
		{
			cntOverSpd = 0;
			ErrorCode = ERR_OSPD;		//µç»úËÙ¶È³ÖÐø20ms>SPD_MAX+SPD_MAXERR,¿ÉÈÏÎª³¬ËÙ¹ÊÕÏ
			return;
		}
	}
	else
	{
		cntOverSpd = 0;
	}
//µç»úÔË×ªÒì³£(·´×ª)¼ì²â
	if((flagMotorState == MOTOR_STATE_RUN) && (flagThetaTest.bit.TestEn == 0))
	{//´ý»ú¡¢²âµç»ú³õÊ¼½ÇÊ±²»¼ì²â
		if(MotorDir ^ SpeedMeas.Direction)
		{
			if(SpeedMeas.SpeedRpm>750)
			{
				cntNegSpd++;
				if(cntNegSpd>100)
				{
					cntNegSpd = 0;
					ErrorCode = ERR_NSPD;		//µç»úËÙ¶È³Öø100ms·´×ª750rpm,¿ÉÈÏÎª·´×ª¹ÊÕÏ
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
 * µç»ú¹ýÔØ¹ÊÕÏ¹ÊÕÏ¼ì²â
 * 
 * 
 * 
 */
const Uint16 TABLE_MOTOR_OVERLOAD[8][2] = 
{
//	Ê±¼ä0.1s	Ð±ÂÊ(tn-tn')/9
	36000,		2000,		//114%In 60·ÖÖÓ¹ýÔØ
	18000,		1333,		//123%In 30·ÖÖÓ¹ýÔØ
	 6000,		 333,		//132%In 10·ÖÖÓ¹ýÔØ
	 3000,		 200,		//141%In  5·ÖÖÓ¹ýÔØ
	 1200,		  72,		//150%In  2·ÖÖÓ¹ýÔØ
	  550,		  28,		//159%In   55Ãë¹ýÔØ
	  300,		  22,		//168%In   30Ãë¹ýÔØ
	  100,		   0,		//177%In   10Ãë¹ýÔØ
};
#define IQ_RATED	4.0		//¶î¶ÈµçÁ÷(A)
void Motor_Overload(void)
{
	static int16 cntMsDelay = 0;		//¼ÆÊýÆ÷ ms
	static int32 cntMotorDelay = 0;		//±£»¤Ê±¼ä¼ÆÊý (0~36000)*0.1s
	Uint16 IqOverload;					//µçÁ÷(Iq/In*100)
	Uint16 TimeTabled;					//ÇúÏß¼ÆËãÖµ (0~36000)*0.1s
	Uint16 temp1,temp2;

	if(ErrorCode != 0)
	{
		return;							//ÒÑÓÐ¹ÊÕÏºó²»ÔÙ¼ì²â
	}
	cntMsDelay++;
	if(cntMsDelay < 100)
	{
		return;							//²»µ½0.1sÌø³ö
	}
	cntMsDelay = 0;

	IqOverload = abs(SysInfo.Iq) / IQ_RATED;
	if(IqOverload < 114)
	{//µÍÔØ²»¼ì²é
		cntMotorDelay -= 360;			//36000/100
		if(cntMotorDelay < 0)
		{
			cntMotorDelay = 0;
		}
	}
	else
	{//¹ýÔØ¼ì²é 114~
 		if(IqOverload > 177)
		{
			IqOverload = 177;			//ÏÞ¶¨ÔÚ114~195
		}
		temp1 = (IqOverload-114)/9;		//ÉÌ	±íÖÐµã¼ä¸ô9
		temp2 = (IqOverload-114)%9;		//ÓàÊý
		TimeTabled = TABLE_MOTOR_OVERLOAD[temp1][0] - TABLE_MOTOR_OVERLOAD[temp1][1] * temp2;
		cntMotorDelay += 36000 / TimeTabled;
		if(cntMotorDelay > 36000)
		{
			cntMotorDelay = 36000;
			ErrorCode = ERR_OVERLOAD;	//µç»ú¹ýÔØ¹ÊÕÏ
		}
	}

}//end of Motor_Overload()


/**
 * Synchro_Detect
 * Í¬²½ÐÅºÅ¹ÊÕÏ¼ì²â
 * 
 * 
 * 
 */
void Synchro_Detect(void)
{
	if(ErrorCode != 0)
	{
		return;							//ÒÑÓÐ¹ÊÕÏºó²»ÔÙ¼ì²â
	}
//UP/DNÐÅºÅ¼ì²â-----------------------
	if(SewMode == SEW_MODE_SIMPLE || flagMotorState == MOTOR_STATE_STOP)
	{//¼òÒ×·ìÄ£Ê½¡¢²âµç»ú³õÊ¼½Ç£¬²»¼ì²âÍ£ÕëÐÅºÅ(up/dn)¹ÊÕÏ
		Synchro.cntUP0 = 0;
		Synchro.cntUP1 = 0;
	}
	else if(SpeedMeas.SpeedRpm > 50)
	{//µçÆ½¼ÆÊ±
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
		ErrorCode = ERR_SYNCHRO;	//×ªËÙ50RPMÒÔÉÏÊ±³ÖÐø1sÎª¸ß»òµÍ£¬±¨Í£ÕëÐÅºÅ¹ÊÕÏ
	}

}//end of Synchro_Detect()


/**
 * Synchro_Clear
 * Í¬²½ÐÅºÅ¼ÆÊýÇåÁã
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
 * É²³µ·Åµç×Ó³ÌÐò
 * 
 * 
 * 
 */
void Brake(void)
{
//É²³µ´¦Àí
	if((flagMotorState == MOTOR_STATE_RUN) && \
	   (SpeedMeas.SpeedRpm > SPEED_DB) && \
	   (((MotorDir == MOTOR_DIR_POS) && (pid1_spd.Out<0)) || \
	    ((MotorDir == MOTOR_DIR_NEG) && (pid1_spd.Out>0))))
	{//ÖÍ»·´¦Àí
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
		flagBrake = 0;		//´ý»úÊ±»òËÙ¶ÈµÍÓÚSPEED_DB²»¿ªÖÆ¶¯
	}
}//end of Brake()

/**
 * FaultProcess()
 * ¹ÊÕÏ±£»¤´¦Àí¼°äÁ¿ÇåÁã
 * 
 * 
 * 
 */
void FaultProcess(void)
{
	if(ErrorCode != 0)
	{//ÓÐ¹ÊÕÏ
		if(ErrorCode == ERR_POFF)
		{//Poff¹ÊÕÏ
			
		}
		flagMotorState = MOTOR_STATE_STOP;
		PWM_EN_OFF;				// µç»úÊ¹ÄÜ¹Ø
	}

}//end of FaultProcess()


/**
 * Speed_Detect
 * ËÙ¶È¼ì²â
 * 
 * 
 * 
 */
void Speed_Detect(void)
{
	speed1.Enc1ms = abs(qep1.EncSpeed);
	qep1.EncSpeed = 0;						//¼ÆÊýÇåÁã
	speed1.Direction = qep1.DirectionQep;	//×ªÏò¸³Öµ
	speed1.calc(&speed1);					//ËÙ¶È¼ÆËã

	SpeedMeas.Direction = qep1.DirectionQep;
	SpeedMeas.Speed = speed1.Speed;
	SpeedMeas.SpeedRpm = speed1.SpeedRpm;

}//end of Speed_Detect()


/**
 * Pos_P_Reg
 * Î»ÖÃ»·Pµ÷½Ú
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

	//16´Î»¬¶¯ÇóÆ½¾ùÖµ
	for(i=15; i>0; i--)
	{
		pid1_pos.buffer[i] = pid1_pos.buffer[i-1];
	}
	pid1_pos.buffer[0] = pid1_pos.TempSpeed;
	pid1_pos.sum = pid1_pos.remainder;
	for(i=0; i<16; i++)
	{
		pid1_pos.sum += pid1_pos.buffer[i];			//ÇóºÍ
	}
	pid1_pos.remainder = pid1_pos.sum&0x000f;
	SpeedRef_P_FIR = pid1_pos.sum>>4;			//Æ½¾ùÖµ
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
 * ËÙ¶È»·PIµ÷½Ú
 * 
 * 
 * 
 */
void Speed_PI_Reg(void)
{
	_iq SpeedR,SpeedL,SpeedH;
	_iq KpL,KpH,KiL,KiH;

	if(MotorDir == MOTOR_DIR_POS)
	{//Õý×ª
		pid1_spd.OutMax = _IQ( 14.0/BASE_CURRENT);
		pid1_spd.OutMin = _IQ(-10.0/BASE_CURRENT);
		pid1_spd.Ref = pid1_pos.Out + feedforward_spd.Out;
	}
	else
	{//·´×ª
		pid1_spd.OutMax = _IQ( 10.0/BASE_CURRENT);
		pid1_spd.OutMin = _IQ(-14.0/BASE_CURRENT);
		pid1_spd.Ref = - pid1_pos.Out - feedforward_spd.Out;
	}
	pid1_spd.Ref = _IQsat(pid1_spd.Ref, pid1_pos.OutMax, pid1_pos.OutMin);	/* Saturate the Ref */
	pid1_spd.Fdb = SpeedMeas.Speed;

//¸ßµÍËÙPI²ÎÊýÇúÏß¼ÆËã
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

//ËÙ¶È»·PIµ÷½Ú
	pid1_spd.calc(&pid1_spd);	//ËÙ¶È»·PIDµ÷½Ú

	feedforward_torque.In = pid1_spd.Ref;
	feedforward_torque.calc(&feedforward_torque);

//ÏÂÕë¼ÓÁ¦´¦Àí
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
 * µôµç±£´æÊý¾Ý´¦Àí
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
	static Uint32 Delay = 0;		//Ð´ÈëÊý¾ÝÐèÒªÑÓÊ±£¬±£Ö¤ÄÜÏÔÊ¾poff
	#define EEPROM_INIT				0
	#define EEPROM_START			1
	#define EEPROM_WORK				2
	#define EEPROM_DELAY			3
	static Uint16 EEPROM_Step = EEPROM_INIT;

	switch(EEPROM_Step)
	{
		case EEPROM_INIT:
		{
			if(flagPower.bit.Poff == 1 && hmi_common_vars.ParFlg == 1)	//ÓÐµôµçÐÅºÅÇÒ²ÎÊý±íÓÐ¸Ä¶¯
			{
				EEPROM_Step = EEPROM_START;
			}
			break;
		}
		case EEPROM_START:
		{
			//¶ÁÏÖÓÐµÄ²ÎÊý±í
			//DINT;
			ParaRead(buf);
			//ÓëÄÚ´æÖÐµÄ²ÎÊý±í¶Ô±È
			for(;i<(PAR_ADV_DEF_SIZE-1);i++)
			{
				if(buf[i] != par_adv_value[i])
				{
					EEPROM_Step = EEPROM_DELAY;
					break;
				}
			}
			if(EEPROM_Step != EEPROM_WORK)		//Èç¹û²ÎÊýÒ»Ñù£¬¾Í°Ñ²½ÖèÇåÁã£¬²ÎÊý¸ü¸Ä±êÖ¾Î»ÇåÁã
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
		case EEPROM_WORK:		//´Èë
		{
			//¼ÆËãÐ£ÑéºÍ
			par_adv_value[(PAR_ADV_DEF_SIZE-1)] = 0;
			for(i=0;i<(PAR_ADV_DEF_SIZE-1);i++)
			{
				par_adv_value[(PAR_ADV_DEF_SIZE-1)] += par_adv_value[i];
			}
			//Ð´Èë²ÎÊý±í
			if(ParaWrite() == FALSE)
			{
				ErrorCode = ERR_EEP;
			}
			else	//Ð£Ñé²ÎÊý
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
					//ÑÓÊ±1ms
					DELAY_US(1000);
					//ÔÙ¼ì²âÊÇ·ñÓÐÐÅºÅ
					if(OZ == 0)//poff cancelled,reset DSP
					{
						EALLOW;						// Enable the watchdog
						SysCtrlRegs.WDCR = 0x0028;	// Òç³öÊ±¼ä£º2^8*512*2^(bit210-1)/20M = 6.5536 ms
						EDIS;

						EALLOW;						// Enable the watchdog
						SysCtrlRegs.WDCR = 0x0000;	// Òç³öÊ±¼ä£º2^8*512*2^(bit210-1)/20M = 6.5536 ms
						EDIS;
					}
					else	
					{
						CL_OFF;							// ¹Ø¼ÌµçÆ÷
						flagPower.bit.Poff = 1;			// µôµç
					}
				}
			}
		}
	}
}//end of SaveDataPoff()


/**
 * ModeParaUpdate
 * ÏµÍ³²ÎÊý½âÊÍË¢ÐÂ
 * 
 * 
 * 
 */
void ModeParaUpdate(void)
{
	static Uint16 Flag = 0;		//0,µÚÒ»´Î¸üÐÂ

	if(flagMotorState == MOTOR_STATE_RUN)
	{//ÔËÐÐÊ±ÐèÒª½âÊÍµÄ²ÎÊýÔÚ´Ë´¦Àí
		SpeedPedalMax =			PARA_BUF[0] * SEWPARA_DEFAULT[0].v_step;			//ÓÃ»§Éè¶¨×î¸ß·ìÈÒËÙ¶È
		return;
	}

//==================½âÊÍ²ÎÊýÊý¾Ý=============================
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

	SpeedPedalMax =			PARA_BUF[0] * SEWPARA_DEFAULT[0].v_step;			//ÓÃ»§Éè¶¨×î¸ß·ìÈÒËÙ¶È

	StitchSoftStart =		PARA_BUF[1] * SEWPARA_DEFAULT[1].v_step;			//ÈíÆô¶¯ÕëÊý
//	SpeedAuto =				bufSewPara[3] * SEWPARA_DEFAULT[3].v_step;			// ¶¨³¤·ì×î¸ßËÙ

	SpeedSoftStart1 =		PARA_BUF[2] * SEWPARA_DEFAULT[2].v_step;
	SpeedSoftStart2 =		PARA_BUF[3] * SEWPARA_DEFAULT[3].v_step;
	SpeedSoftStart3 =		PARA_BUF[4] * SEWPARA_DEFAULT[4].v_step;

	AccTime = PARA_BUF[5] * 20;
	DecTime = PARA_BUF[6] * 20;

	flagSet.bit.Go2Up =		PARA_BUF[8];										//ÉÏµçÕÒÉÏÕëÎ»
	flagSet.bit.FTKGmode = 	PARA_BUF[9];										// 0 ³£¿ª; 1 ³£±Õ 

	SpeedPedalMin =			200;	// Ì¤°å×îµÍËÙ,¸ù¾Ý»úÐÍÂëÀ´
	optPedalCurve =			PARA_BUF[10] * SEWPARA_DEFAULT[10].v_step;			//Ì¤°åÇúÏß

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
	MechThetaDnSet =		177;	//ÏÂÍ£Õë½Ç
	flagSet.bit.optFZTZ =	0;		//·´×ªÌáÕëÊ¹ÄÜ
	MechThetaFZTZ =			20;		// ·´×ªÌáÕë½Ç¶È

	SpeedMax =				PARA_BUF[20] * SEWPARA_DEFAULT[20].v_step;						// ÏµÍ³×î¸ßËÙ		
	CurrentFeedForwardDn = _IQ(PARA_BUF[21] * SEWPARA_DEFAULT[21].v_step/BASE_CURRENT/2); 	//¼ÓÖØ¹¦ÄÜ

	MechThetaStop = (PARA_BUF[22] * SEWPARA_DEFAULT[22].v_step * 3) >> 1;		//Í£Õë²¹³¥½Ç¶È

	MechThetaUpSet = ((MECHTHETA_UP + MechThetaStop)>=360)?((MECHTHETA_UP + MechThetaStop)-360):(MECHTHETA_UP + MechThetaStop);
	RatioWheel = 1000;	
	
	AutoTestMode = 			PARA_BUF[29];	//ÀÏ»¯Ä£Ê½
	SpeedAutoTest =			PARA_BUF[30] * SEWPARA_DEFAULT[30].v_step;							//×Ô¶¯²âÊÔËÙ¶È
	TimeAutoTest = 			PARA_BUF[32] * SEWPARA_DEFAULT[32].v_step * 100;				// ×Ô¶¯²âÊÔÍ£¶ÙÊ±¼ä
	TimeAutoRun = 			PARA_BUF[31] * SEWPARA_DEFAULT[31].v_step * 100;				//×Ô¶¯²âÊÔÔËÐÐÊ±¼ä
	StitchAutoTest = 		PARA_BUF[33];
	
//	StitchAutoTest = 		bufSewPara[97] * SEWPARA_DEFAULT[97].v_step;

	//ºÍÊÖÂÖ²ÎÊýÓÐ¹Ø±äÁ¿
#if((MACHINE_WR580BS||MACHINE_WR587BS) && MACHINE_MODE_SHULIAO)
	MotorDir = MOTOR_DIR_POS;
	MachinePos.CurveStop = 1;
	rc2.Resolution = _IQ(1.0/300);
	pid1_pos.Kp = _IQ30(0.36);				//Î»ÖÃ»·	//0.36 44µþºñ
	pid1_pos2.Kp = _IQ(0.40/256);			//Í£ÕëÎ»ÖÃ»·//0.40 44µþºñ
	MachinePos.spd_Low = 300;
	MachinePos.spd_High = 3000;
	MachinePos.spd_KpL = _IQ(1.5);			//2.0 44µþºñ
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
	pid1_pos.Kp = _IQ30(0.34);				//Î»ÖÃ»·	//0.36 44µþºñ
	pid1_pos2.Kp = _IQ(0.40/256);			//Í£ÕëÎ»ÖÃ»·//0.40 44µþºñ
	MachinePos.spd_Low = 300;
	MachinePos.spd_High = 3000;
	MachinePos.spd_KpL = _IQ(1.0);			//2.0 44µþºñ
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

//==================½âÊÍ·ìÈÒÄ£Ê½Êý¾Ý=============================
	if(PARA_BUF[7] == 1)
	{//¼òÒ×·ì²ÎÊýÉè¶¨ÓÐÐ§
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
 * Ä£ÄâÌ¤°å²ÎÊý×ª»»
 * ×ª»»½Ç¶ÈÖµµ½µçÑ¹Öµ
 * 
 * 
 */
int16 PedalMoniAD_Cal(int16 para, _iq scale, _iq offset)
{
	return (_IQ15mpy(scale,_IQ15((para-100)/10)) + offset);

}//end of PedalMoniAD_Cal()

/**
*µç¸ÐÌ¤°åµÄÐÐ³Ì×ªµ½ÆµÂÊ¼ÆÊýÖµ
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
