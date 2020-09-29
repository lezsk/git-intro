/**
 * Copyright (c) 2010, �Ϻ����µ��ӿƼ����޹�˾
 * All rights reserved.
 *
 * @file DMCvars.c
 * 
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

#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "Defines.h"
//#include "E2prom.h"
#include "GlobalVars.h"


/*-------------------------------------------------------------------------------
-------------------------------------------------------------------------------*/
FLAGPOWER flagPower;				// �ϵ���̴����־
Uint16 bufHmiData[25] = {0x00,0x00,4,4,4,4,4,4,4,5, \
						 12,24,48,32,0,0,0,0,0,0,0,0,0,0,0};	//0xA3 �ϻ���ǰ��̷�
//bufHmiData����
//byte1 bit3~0: 	0~1 ���ɷ� 2 ������ 3 �ϻ�һ�η�
//byte2 bit1: 		�����촥��ʹ��
//byte3~10:			reserved
//byte11~25:		������P1~PF��������

Uint16 HmiRunCmd;					// HMI��������(��������)
/*-------------------------------------------------------------------------------
 ϵͳ(I2C)�ñ���
-------------------------------------------------------------------------------*/
// I2C_EEPROM definitions
/*struct I2CMSG I2cMsgOutMotor={I2C_MSGSTAT_SEND_WITHSTOP,
                              I2C_SLAVE_ADDR_MOTOR,	// I2C_SLAVE_ADDR
                              1,					// I2C_NUMBYTES (ADDR+DATA)
                              0x00,					// I2C_EEPROM_HIGH_ADDR(no used)
                              0x00};				// I2C_EEPROM_LOW_ADDR(ҳ�׵�ַ,ÿҳ8byte)

struct I2CMSG I2cMsgInMotor={ I2C_MSGSTAT_SEND_NOSTOP,
                              I2C_SLAVE_ADDR_MOTOR,	// I2C_SLAVE_ADDR
                              1,					// I2C_NUMBYTES
                              0x00,					// I2C_EEPROM_HIGH_ADDR(no used)
                              0x00};				// I2C_EEPROM_LOW_ADDR(ҳ�׵�ַ,ÿҳ8byte)
*/
struct I2CMSG *CurrentI2CMsgPtr;					// Used in interrupts
I2CMSG_EEPROM EEPROM_Read = EEPROM_I2C_DEFAULT;
I2CMSG_EEPROM EEPROM_Write = EEPROM_I2C_DEFAULT;
/*-------------------------------------------------------------------------------
 ϵͳͨ�ñ���
-------------------------------------------------------------------------------*/
SYSINFO SysInfo = SYSINFO_DEFAULTS;	// ϵͳ�����Ϣ
SYNCHRO Synchro = SYNCHRO_DEFAULTS;
SPEEDMEAS SpeedMeas = SPEEDMEAS_DEFAULTS;

Uint16 counterMainISR;				// ���жϼ�����
Uint16 ErrorCode;					// ���ϴ���
Uint16 ErrorCodeOld;
Uint16 UdcRatio = 207;
Uint16 CntOZdelay = 0;
Uint16 I2cAngle;					//���A��ͻ���A�ļн�
Uint16 flagMotorState;				// ���״̬ 0 ���� 1 ����
Uint16 MotorDir = MOTOR_DIR_POS;	// ���ת���趨

PEDALMONIAD PedalMoniAD;			// ģ��̤���г̲���
FLAGPEDALTEST flagPedalTest;		// ģ��̤��У���־


//���ʼ���ñ���
Uint16 ThetaTest1st = 0;
Uint16 ThetaTest2nd = 0;
Uint16 ThetaTest3rd = 0;
Uint16 ThetaTest = 0;				//3��ƽ��ֵ
FLAGTHETATEST flagThetaTest;		//
PEDAL_ECAP_COUNTER PedaleCapCounter = PEDAL_ECAP_COUNTER_DEFAULT;

volatile ANGLE_INIT AngleInit = ANGLE_INIT_DEFAULT;

/*-------------------------------------------------------------------------------
 ���ҹ����ñ���
-------------------------------------------------------------------------------*/
//int16 bufSewPara[NUMOFPARA];
FLAGSET flagSet;
Uint16 SewMode;						// ����ģʽ 0 ���ɷ죻1 W�죻2 �����죻3 ���׷�
Uint16 SewState;					// ����״̬ 0 ������1 ���ң�2 ���ߣ�3 ���룻4 ��ͣ
Uint16 optPedalCurve;				// 0 ����; 1 ��; 2 �� 

Uint16 SpeedPedalMin;				// ̤�������
Uint16 SpeedPedalMax;				// ̤�������
Uint16 SpeedPedalZoom;				// ̤��ٷֱ�
Uint16 SpeedAuto;					// �����������
Uint16 SpeedMax;					// ϵͳ�����
Uint16 SpeedSoftStart1;				// ��������1���ٶ�
Uint16 SpeedSoftStart2;				// ��������1���ٶ�
Uint16 SpeedSoftStart3;				// ��������3~9���ٶ�
int16 StitchSoftStart;				// ����������
int16 NumAutoStep;					// ���������(���15��)
int16 CountAutoStep = 0;			// �������������
int16 NumAutoStitch;				// �����쵱ǰ������
int16 StitchAutoStep[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};		// �������������
Uint16 MechThetaUpSet;				//��ͣ��Ƕ�
Uint16 MechThetaDnSet;				// ��ͣ���
Uint16 MechThetaFZTZ;				// ��ת����Ƕ�
Uint16 MechThetaStop;				// ͣ�벹���Ƕ�
Uint16 RatioWheel = 1000;			// ���ӱ�

Uint16 AutoTestMode;				//�ϻ�ģʽѡ��0��3��
Uint16 TimeAutoTest;				// �Զ�����ͣ��ʱ��
Uint16 TimeAutoRun;					// �Զ���������ʱ��
Uint16 SpeedAutoTest;				// �Զ������ٶ�
int16 StitchAutoTest;				// �Զ���������(һ��)

Uint16 HmiCmd;						// Hmi����
Uint16 RunCmd;						// ��������
Uint16 HmiNcCmd;					// ��������
Uint16 SpeedPedal;					// ̤���ٶ�
int16 SpeedSel;						// �ٶȸ���(δ�������ת��)
int16 SpeedSet;						// �ٶȸ���(���������ת��)
int16 SpeedRef;						// �ٶȸ���(����������)
int32 SpeedRef_P;					// �ٶȸ���(����ÿ����)

Uint16 AccTime;				//���ٶ�
Uint16 DecTime;				//���ٶ�

FLAGSEW flagSew;
FLAGPOS flagPos;
FLAGSTITCH flagStitch;				// ��������־
int16 CountStitch;					// ����
Uint16 flagBrake = 0;				// ɲ����־
Uint16 flagFeedForward = 0;			// ǰ����־
Uint16 flagFeedForwardDn = 0;		// ����ǰ����־
int32 CurrentFeedForwardDn = 0;		// ����ǰ������(���ۻ�)

MACHINEPOS MachinePos = MACHINEPOS_DEFAULTS;	// ��ͬ��ͷ��ͣ��Ч����ر���

int16 DebugVal1 = 0;	
int16 DebugVal2 = 0;

/*================
Flash ����ʹ��
================*/
Uint16 ParaCheckSum = 0;
Uint16* PARA_ADD = 0;
//===========================================================================
// No more.
//===========================================================================
