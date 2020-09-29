/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
 * All rights reserved.
 *
 * @file E2promTable.c
 * EEPROM存储默认表.
 *
 *
 * @author yemj
 * @version 0.1
 * @date 2010-6-12
 *
 */


#include "E2prom.h"
#include "Pedal.h"



//-------------------------------------------------------------
//机型默认参数表 
//注意：
//	1.不同机型的参数步距为一致
//	2.下表包含了本程序支持机型所有参数
//-------------------------------------------------------------
const EEPPARA SEWPARA_DEFAULT[NUMOFPARA] =
{//	   val  min  max  sca num flg(bit2 Reload; bit1:0 Level)
	{  35,   2,  50, 100,  1, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//00 踏板最高速
//	{  50,   2,  50, 100,  1, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//00 踏板最高速
	{   2,   0,   9,   1,  2, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//01 软启动针数
	{   0,   0,   0,   1,  3, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//02 reserved
	{  30,   2,  50, 100,  4, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//03 定长缝最高速
//	{  50,   2,  50, 100,  4, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//03 定长缝最高速
	{   0,   0,   3,   1,  5, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//04 简易模式设定 0 无效 1 有效
//	{   1,   0,   3,   1,  5, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//04 简易模式设定 0 无效 1 有效
	{   0,   0,   0,   1,  6, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//05 reserved
	{   0,   0,   0,   1,  7, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//06 reserved
	{   0,   0,   0,   1,  8, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//07 reserved
	{   0,   0,   1,   1,  9, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//08 reserved
	{   0,   0,   1,   1, 10, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//09 reserved
	{   0,   0,   1,   1, 11, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//10 reserved
	{   0,   0,   1,   1, 12, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//11 reserved
	{   0,   0,   1,   1, 13, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//12 reserved
	{   0,   0,   1,   1, 14, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//13 reserved
	{   0,   0,   1,   1, 15, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//14 reserved
	{   0,   0,   1,   1, 16, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//15 reserved
	{   0,   0,   1,   1, 17, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//16 reserved
	{   0,   0,   1,   1, 18, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//17 reserved
	{   0,   0,   1,   1, 19, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//18 reserved
	{   0,   0,   1,   1, 20, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//19 reserved
	{  12,   2,  60,  50, 21, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//20 软启动第1针速度
	{  20,   2,  60,  50, 22, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//21 软启动第2针速度
	{  30,   2,  60,  50, 23, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//22 软启动第3~9针速度
	{   0,   0,   1,   1, 24, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//23 reserved
	{   0,   0,   1,   1, 25, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//24 reserved
	{   0,   0,   1,   1, 26, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//25 reserved
	{   1,   0,   1,   1, 27, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//26 上电定位设定		0 无；1 有
	{   0,   0,   1,   1, 28, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//27 翻台开关信号模式	0 常开；1 常闭
	{   0,   0,   1,   1, 29, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//28 reserved
	{   0,   0,   1,   1, 30, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//29 reserved
	{   0,   0,   1,   1, 31, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//30 reserved
	{   0,   0,   1,   1, 32, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//31 reserved
	{   0,   0,   1,   1, 33, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//32 reserved
	{   0,   0,   1,   1, 34, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//33 reserved
	{   0,   0,   1,   1, 35, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//34 reserved
	{   0,   0,   0,   1, 36, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//35 reserved
	{   0,   0,   0,   1, 37, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//36 reserved
	{   0,   0,   0,   1, 38, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//37 reserved
	{   0,   0,   0,   1, 39, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//38 reserved
	{   0,   0,   0,   1, 40, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//39 reserved
	{  20,  10,  40,  10, 41, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//40 低速速度
	{   0,   0,   2,   1, 42, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//41 踏板曲线设定		0 正常；1 慢；2 快
	{   0,   0,   0,   1, 43, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//42 reserved
	{   0,   0,   0,   1, 44, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//43 reserved
	{   0,   0,   0,   1, 45, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//44 reserved
	{   0,   0,   0,   1, 46, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//45 reserved
	{   0,   0,   0,   1, 47, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//46 reserved
	{   0,   0,   0,   1, 48, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//47 reserved
	{   0,   0,   0,   1, 49, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//48 reserved
	{   0,   0,   0,   1, 50, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//49 reserved
	{   0,   0,   0,   1, 51, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//50 reserved
	{   0,   0,   0,   1, 52, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//51 reserved
	{   0,   0,   0,   1, 53, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//52 reserved
	{   0,   0,   0,   1, 54, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//53 reserved
	{   0,   0,   0,   1, 55, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//54 reserved
	{   0,   0,   0,   1, 56, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//55 reserved
	{   0,   0,   0,   1, 57, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//56 reserved
	{   0,   0,   0,   1, 58, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//57 reserved
	{   0,   0,   0,   1, 59, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//58 reserved
	{   0,   0,   0,   1, 60, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//59 reserved
	{   0,   0,   0,   1, 61, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//60 reserved
	{ 125, 110, 150,   1, 62, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//61 开始运行的踏行程 ( -100)0.1度
	{ 150, 110, 200,   1, 63, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//62 开始加速的踏板行程
	{ 210, 110, 250,   1, 64, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//63 高速运行的踏板行程
	{  80,   0,  90,   1, 65, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//64 压脚抬升的踏板行程
	{ 110, 105, 150,   1, 66, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//65 压脚降下的踏板行程
	{  75,   0,  90,   1, 67, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//66 开始剪线的踏板行程1
	{  40,   0,  90,   1, 68, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//67 开始剪线的踏板行程2
	{ 177, 120, 240,   1, 69, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//68 下停针角
	{   0,   0,   1,   1, 70, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//69 反转提针使能
	{  20,   0,  45,   1, 71, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//70 反转提针角度
	{   0,   0,   0,   1, 72, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//71 reserved
	{   0,   0,   0,   1, 73, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//72 reserved
	{   0,   0,   0,   1, 74, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//73 reserved
	{   0,   0,   0,   1, 75, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//74 reserved
	{   0,   0,   0,   1, 76, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//75 reserved
	{   0,   0,   0,   1, 77, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//76 reserved
	{   0,   0,   0,   1, 78, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//77 reserved
	{   0,   0,  15,   1, 79, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//78 出厂参数/隐藏参数
	{  40,   3,  50, 100, 80, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//79 max速度
//	{  50,   3,  50, 100, 80, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//79 max速度
	{   0,   0,   0,   1, 81, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//80 reserved
	{   0,   0,   0,   1, 82, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//81 reserved
	{   0,  15,   1,   1, 83, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//82 加重功能(机针穿不透布时使用)	0 无效；1~15 力度调整
	{   0,   0,   0,   1, 84, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//83 reserved
	{   0,   0,   0,   1, 85, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//84 reserved
	{   0,   0,   0,   1, 86, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//85 reserved
	{   0,   0,   0,   1, 87, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//86 reserved
	{   0,   0,   0,   1, 88, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//87 reserved
	{   0,   0,   0,   1, 89, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//88 reserved
	{ 200, 160, 240,   5, 90, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//89 轮子比(不显示)
	{   0,   0,   0,   1, 91, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//90 reserved
	{   0,   0,   0,   1, 92, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//91 reserved
	{ 100,  85, 115,   1, 93, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//92 踏板中立行程调整
	{   0,   0,   0,   1, 94, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//93 reserved
	{   0,   0,   0,   1, 95, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//94 reserved
	{  40,   3,  50, 100, 96, PARA_RESTORE_DIS+ PARA_LEVEL_FACTORY	},	//95 自动测试速度
	{   8,   0,  50, 100, 97, PARA_RESTORE_DIS+ PARA_LEVEL_FACTORY	},	//96 自动测试stop时间 ms
	{  40,   1, 200,   1, 98, PARA_RESTORE_DIS+ PARA_LEVEL_FACTORY	},	//97 自动测试一段缝针数
	{   0,   0,   0,   1, 99, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//98 reserved
//--------------------以下保存值均为参数界面下不可见------------------------------------
	{   1,   0,   1,   1,  0, PARA_RESTORE_EN + PARA_LEVEL_NULL  	},	// 99 针位设定			0 上；1 下
	{   1,   0,   1,   1,  0, PARA_RESTORE_EN + PARA_LEVEL_NULL  	},	//100 软启动设定 		0 无效 1 有效
	{ 135,   0, 180,   2,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//101 电机初始角设定
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//102 润滑提示时间计数Lo_BYTE
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//103 润滑提示时间计数Hi_BYTE
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//104 试用时间计数Lo_BYTE
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//105 试用时间计数Hi_BYTE
	{   1,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//106 密码1
	{   1,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//107 密码2
	{   1,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//108 密码3
	{   1,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//109 密码4
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//110 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//111 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//112 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//113 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//114 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//115 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//116 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//117 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//118 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//119 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//120 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//121 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//122 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//123 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//124 reserved
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//125 reserved
	{ EEP_PARA_CHK1, 0,0,1,0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//126 标识代码1
	{ EEP_PARA_CHK2, 0,0,1,0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//127 标识代码2
};


//===========================================================================
// No more.
//===========================================================================





