/**
 * Copyright (c) 2010, �Ϻ����µ��ӿƼ����޹�˾
 * All rights reserved.
 *
 * @file E2promTable.c
 * EEPROM�洢Ĭ�ϱ�.
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
//����Ĭ�ϲ����� 
//ע�⣺
//	1.��ͬ���͵Ĳ�������Ϊһ��
//	2.�±�����˱�����֧�ֻ������в���
//-------------------------------------------------------------
const EEPPARA SEWPARA_DEFAULT[NUMOFPARA] =
{//	   val  min  max  sca num flg(bit2 Reload; bit1:0 Level)
	{  35,   2,  50, 100,  1, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//00 ̤�������
//	{  50,   2,  50, 100,  1, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//00 ̤�������
	{   2,   0,   9,   1,  2, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//01 ����������
	{   0,   0,   0,   1,  3, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//02 reserved
	{  30,   2,  50, 100,  4, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//03 �����������
//	{  50,   2,  50, 100,  4, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//03 �����������
	{   0,   0,   3,   1,  5, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//04 ����ģʽ�趨 0 ��Ч 1 ��Ч
//	{   1,   0,   3,   1,  5, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//04 ����ģʽ�趨 0 ��Ч 1 ��Ч
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
	{  12,   2,  60,  50, 21, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//20 ��������1���ٶ�
	{  20,   2,  60,  50, 22, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//21 ��������2���ٶ�
	{  30,   2,  60,  50, 23, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//22 ��������3~9���ٶ�
	{   0,   0,   1,   1, 24, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//23 reserved
	{   0,   0,   1,   1, 25, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//24 reserved
	{   0,   0,   1,   1, 26, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//25 reserved
	{   1,   0,   1,   1, 27, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//26 �ϵ綨λ�趨		0 �ޣ�1 ��
	{   0,   0,   1,   1, 28, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//27 ��̨�����ź�ģʽ	0 ������1 ����
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
	{  20,  10,  40,  10, 41, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//40 �����ٶ�
	{   0,   0,   2,   1, 42, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//41 ̤�������趨		0 ������1 ����2 ��
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
	{ 125, 110, 150,   1, 62, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//61 ��ʼ���е�̤�г� ( -100)0.1��
	{ 150, 110, 200,   1, 63, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//62 ��ʼ���ٵ�̤���г�
	{ 210, 110, 250,   1, 64, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//63 �������е�̤���г�
	{  80,   0,  90,   1, 65, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//64 ѹ��̧����̤���г�
	{ 110, 105, 150,   1, 66, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//65 ѹ�Ž��µ�̤���г�
	{  75,   0,  90,   1, 67, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//66 ��ʼ���ߵ�̤���г�1
	{  40,   0,  90,   1, 68, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//67 ��ʼ���ߵ�̤���г�2
	{ 177, 120, 240,   1, 69, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//68 ��ͣ���
	{   0,   0,   1,   1, 70, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//69 ��ת����ʹ��
	{  20,   0,  45,   1, 71, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//70 ��ת����Ƕ�
	{   0,   0,   0,   1, 72, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//71 reserved
	{   0,   0,   0,   1, 73, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//72 reserved
	{   0,   0,   0,   1, 74, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//73 reserved
	{   0,   0,   0,   1, 75, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//74 reserved
	{   0,   0,   0,   1, 76, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//75 reserved
	{   0,   0,   0,   1, 77, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//76 reserved
	{   0,   0,   0,   1, 78, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//77 reserved
	{   0,   0,  15,   1, 79, PARA_RESTORE_EN + PARA_LEVEL_OPERATOR	},	//78 ��������/���ز���
	{  40,   3,  50, 100, 80, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//79 max�ٶ�
//	{  50,   3,  50, 100, 80, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//79 max�ٶ�
	{   0,   0,   0,   1, 81, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//80 reserved
	{   0,   0,   0,   1, 82, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//81 reserved
	{   0,  15,   1,   1, 83, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//82 ���ع���(���봩��͸��ʱʹ��)	0 ��Ч��1~15 ���ȵ���
	{   0,   0,   0,   1, 84, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//83 reserved
	{   0,   0,   0,   1, 85, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//84 reserved
	{   0,   0,   0,   1, 86, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//85 reserved
	{   0,   0,   0,   1, 87, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//86 reserved
	{   0,   0,   0,   1, 88, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//87 reserved
	{   0,   0,   0,   1, 89, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//88 reserved
	{ 200, 160, 240,   5, 90, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//89 ���ӱ�(����ʾ)
	{   0,   0,   0,   1, 91, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//90 reserved
	{   0,   0,   0,   1, 92, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//91 reserved
	{ 100,  85, 115,   1, 93, PARA_RESTORE_EN + PARA_LEVEL_MAINTAIN	},	//92 ̤�������г̵���
	{   0,   0,   0,   1, 94, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//93 reserved
	{   0,   0,   0,   1, 95, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//94 reserved
	{  40,   3,  50, 100, 96, PARA_RESTORE_DIS+ PARA_LEVEL_FACTORY	},	//95 �Զ������ٶ�
	{   8,   0,  50, 100, 97, PARA_RESTORE_DIS+ PARA_LEVEL_FACTORY	},	//96 �Զ�����stopʱ�� ms
	{  40,   1, 200,   1, 98, PARA_RESTORE_DIS+ PARA_LEVEL_FACTORY	},	//97 �Զ�����һ�η�����
	{   0,   0,   0,   1, 99, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//98 reserved
//--------------------���±���ֵ��Ϊ���������²��ɼ�------------------------------------
	{   1,   0,   1,   1,  0, PARA_RESTORE_EN + PARA_LEVEL_NULL  	},	// 99 ��λ�趨			0 �ϣ�1 ��
	{   1,   0,   1,   1,  0, PARA_RESTORE_EN + PARA_LEVEL_NULL  	},	//100 �������趨 		0 ��Ч 1 ��Ч
	{ 135,   0, 180,   2,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//101 �����ʼ���趨
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//102 ����ʾʱ�����Lo_BYTE
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL  	},	//103 ����ʾʱ�����Hi_BYTE
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//104 ����ʱ�����Lo_BYTE
	{   0,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//105 ����ʱ�����Hi_BYTE
	{   1,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//106 ����1
	{   1,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//107 ����2
	{   1,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//108 ����3
	{   1,   0,   0,   1,  0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//109 ����4
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
	{ EEP_PARA_CHK1, 0,0,1,0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//126 ��ʶ����1
	{ EEP_PARA_CHK2, 0,0,1,0, PARA_RESTORE_DIS+ PARA_LEVEL_NULL 	},	//127 ��ʶ����2
};


//===========================================================================
// No more.
//===========================================================================





