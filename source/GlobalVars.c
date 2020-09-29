/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
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
FLAGPOWER flagPower;				// 上电过程处理标志
Uint16 bufHmiData[25] = {0x00,0x00,4,4,4,4,4,4,4,5, \
						 12,24,48,32,0,0,0,0,0,0,0,0,0,0,0};	//0xA3 老化带前后固缝
//bufHmiData含义
//byte1 bit3~0: 	0~1 自由缝 2 定长缝 3 老化一段缝
//byte2 bit1: 		定长缝触发使能
//byte3~10:			reserved
//byte11~25:		定长缝P1~PF各段针数

Uint16 HmiRunCmd;					// HMI运行命令(按键启动)
/*-------------------------------------------------------------------------------
 系统(I2C)用变量
-------------------------------------------------------------------------------*/
// I2C_EEPROM definitions
/*struct I2CMSG I2cMsgOutMotor={I2C_MSGSTAT_SEND_WITHSTOP,
                              I2C_SLAVE_ADDR_MOTOR,	// I2C_SLAVE_ADDR
                              1,					// I2C_NUMBYTES (ADDR+DATA)
                              0x00,					// I2C_EEPROM_HIGH_ADDR(no used)
                              0x00};				// I2C_EEPROM_LOW_ADDR(页首地址,每页8byte)

struct I2CMSG I2cMsgInMotor={ I2C_MSGSTAT_SEND_NOSTOP,
                              I2C_SLAVE_ADDR_MOTOR,	// I2C_SLAVE_ADDR
                              1,					// I2C_NUMBYTES
                              0x00,					// I2C_EEPROM_HIGH_ADDR(no used)
                              0x00};				// I2C_EEPROM_LOW_ADDR(页首地址,每页8byte)
*/
struct I2CMSG *CurrentI2CMsgPtr;					// Used in interrupts
I2CMSG_EEPROM EEPROM_Read = EEPROM_I2C_DEFAULT;
I2CMSG_EEPROM EEPROM_Write = EEPROM_I2C_DEFAULT;
/*-------------------------------------------------------------------------------
 系统通用变量
-------------------------------------------------------------------------------*/
SYSINFO SysInfo = SYSINFO_DEFAULTS;	// 系统监控信息
SYNCHRO Synchro = SYNCHRO_DEFAULTS;
SPEEDMEAS SpeedMeas = SPEEDMEAS_DEFAULTS;

Uint16 counterMainISR;				// 主中断计数器
Uint16 ErrorCode;					// 故障代码
Uint16 ErrorCodeOld;
Uint16 UdcRatio = 207;
Uint16 CntOZdelay = 0;
Uint16 I2cAngle;					//电机A相和霍尔A的夹角
Uint16 flagMotorState;				// 电机状态 0 待机 1 运行
Uint16 MotorDir = MOTOR_DIR_POS;	// 电机转向设定

PEDALMONIAD PedalMoniAD;			// 模拟踏板行程参数
FLAGPEDALTEST flagPedalTest;		// 模拟踏板校零标志


//测初始角用变量
Uint16 ThetaTest1st = 0;
Uint16 ThetaTest2nd = 0;
Uint16 ThetaTest3rd = 0;
Uint16 ThetaTest = 0;				//3次平均值
FLAGTHETATEST flagThetaTest;		//
PEDAL_ECAP_COUNTER PedaleCapCounter = PEDAL_ECAP_COUNTER_DEFAULT;

volatile ANGLE_INIT AngleInit = ANGLE_INIT_DEFAULT;

/*-------------------------------------------------------------------------------
 缝纫功能用变量
-------------------------------------------------------------------------------*/
//int16 bufSewPara[NUMOFPARA];
FLAGSET flagSet;
Uint16 SewMode;						// 缝纫模式 0 自由缝；1 W缝；2 定长缝；3 简易缝
Uint16 SewState;					// 缝纫状态 0 待机；1 缝纫；2 剪线；3 补针；4 暂停
Uint16 optPedalCurve;				// 0 正常; 1 慢; 2 快 

Uint16 SpeedPedalMin;				// 踏板最低速
Uint16 SpeedPedalMax;				// 踏板最高速
Uint16 SpeedPedalZoom;				// 踏板百分比
Uint16 SpeedAuto;					// 定长缝最高速
Uint16 SpeedMax;					// 系统最高速
Uint16 SpeedSoftStart1;				// 软启动第1针速度
Uint16 SpeedSoftStart2;				// 软启动第1针速度
Uint16 SpeedSoftStart3;				// 软启动第3~9针速度
int16 StitchSoftStart;				// 软启动针数
int16 NumAutoStep;					// 定长缝段数(最多15段)
int16 CountAutoStep = 0;			// 定长缝段数计数
int16 NumAutoStitch;				// 定长缝当前段针数
int16 StitchAutoStep[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};		// 定长缝各段针数
Uint16 MechThetaUpSet;				//上停针角度
Uint16 MechThetaDnSet;				// 下停针角
Uint16 MechThetaFZTZ;				// 反转提针角度
Uint16 MechThetaStop;				// 停针补偿角度
Uint16 RatioWheel = 1000;			// 轮子比

Uint16 AutoTestMode;				//老化模式选择（0～3）
Uint16 TimeAutoTest;				// 自动测试停顿时间
Uint16 TimeAutoRun;					// 自动测试运行时间
Uint16 SpeedAutoTest;				// 自动测试速度
int16 StitchAutoTest;				// 自动测试针数(一段)

Uint16 HmiCmd;						// Hmi命令
Uint16 RunCmd;						// 缝纫命令
Uint16 HmiNcCmd;					// 补针命令
Uint16 SpeedPedal;					// 踏板速度
int16 SpeedSel;						// 速度给定(未限制最高转速)
int16 SpeedSet;						// 速度给定(已限制最高转速)
int16 SpeedRef;						// 速度给定(已限制曲线)
int32 SpeedRef_P;					// 速度给定(脉冲每毫秒)

Uint16 AccTime;				//加速度
Uint16 DecTime;				//减速度

FLAGSEW flagSew;
FLAGPOS flagPos;
FLAGSTITCH flagStitch;				// 计针数标志
int16 CountStitch;					// 针数
Uint16 flagBrake = 0;				// 刹车标志
Uint16 flagFeedForward = 0;			// 前馈标志
Uint16 flagFeedForwardDn = 0;		// 下针前馈标志
int32 CurrentFeedForwardDn = 0;		// 下针前馈电流(标幺化)

MACHINEPOS MachinePos = MACHINEPOS_DEFAULTS;	// 不同机头与停针效果相关变量

int16 DebugVal1 = 0;	
int16 DebugVal2 = 0;

/*================
Flash 操作使用
================*/
Uint16 ParaCheckSum = 0;
Uint16* PARA_ADD = 0;
//===========================================================================
// No more.
//===========================================================================
