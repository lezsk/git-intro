/*=======================================================================
//	HMC Technology
//=======================================================================
//File Name:	FlashProgram.c
//Description:	Flash操作接口
//Author:	YuMing
//Target:	TMS320LF28031
//Created Date:	2012.07.25
//Status:	test
=======================================================================*/
#include "DSP2803x_Device.h"
#include "Flash2803x_API_Config.h"
#include "Flash2803x_API_Library.h"
#include "Defines.h"
#include "GlobalVars.h"
#include "par.h"
#include "hmi.h"

extern Uint16 par_adv_value[PAR_ADV_DEF_SIZE];
extern const PARAM_UNIT SEWPARA_DEFAULT[];

Uint16 ParaRead(Uint16 *pBuf)
{
	Uint16 i = 0;
	Uint16* pParaAdd = PARA_ADD;
	Uint16 temp = 0;
	
	for(;i<PAR_ADV_DEF_SIZE;i++)	
	{
		*pBuf = *pParaAdd;
		if(i<(PAR_ADV_DEF_SIZE-1))
		{
			temp += *pBuf;
		}
		pBuf++;
		pParaAdd++;
	}

	ParaCheckSum = *(--pBuf);

	if(temp != ParaCheckSum)
	{
		return FALSE;
	}
	return TRUE;
}

void DefaultParaRead(Uint16 *pBuf)
{
	Uint16 i = 0;

	ParaCheckSum= 0;

	for(;i<(PAR_ADV_DEF_SIZE-1);i++)	
	{
		*pBuf = SEWPARA_DEFAULT[i].v_df;
		ParaCheckSum += *pBuf;
		pBuf++;
	}
	*pBuf = ParaCheckSum;
}

#define CSMSCR       (volatile Uint16*)0x00000AEF   // CSM status and control register 
#define KEY0         (volatile Uint16*)0x00000AE0   /* low word of the 128-bit key */
#define KEY1         (volatile Uint16*)0x00000AE1   /* next word in 128-bit key */
#define KEY2         (volatile Uint16*)0x00000AE2   /* next word in 128-bit key */ 
#define KEY3         (volatile Uint16*)0x00000AE3   /* next word in 128-bit key */
#define KEY4         (volatile Uint16*)0x00000AE4   /* next word in 128-bit key */
#define KEY5         (volatile Uint16*)0x00000AE5   /* next word in 128-bit key */
#define KEY6         (volatile Uint16*)0x00000AE6   /* next word in 128-bit key */
#define KEY7         (volatile Uint16*)0x00000AE7   /* high word of the 128-bit key */
#define PWL0         (volatile Uint16*)0x003F7FF8   /* Password 0 */
#define PWL1         (volatile Uint16*)0x003F7FF9   /* Password 1 */
#define PWL2         (volatile Uint16*)0x003F7FFA   /* Password 2 */
#define PWL3         (volatile Uint16*)0x003F7FFB   /* Password 3 */
#define PWL4         (volatile Uint16*)0x003F7FFC   /* Password 4 */
#define PWL5         (volatile Uint16*)0x003F7FFD   /* Password 5 */
#define PWL6         (volatile Uint16*)0x003F7FFE   /* Password 6 */
#define PWL7         (volatile Uint16*)0x003F7FFF   /* Password 7 */
#define STATUS_FAIL          0
#define STATUS_SUCCESS       1
Uint16 FLASH_CSM[8] = {0x3340,0xD96A,0x360A,0x64D7,0x64D7,0x360A,0xD96A,0x3340};
Uint16 CsmUnlock(void)
{
    volatile Uint16 temp;
    
    // Load the key registers with the current password
    // These are defined in Example_Flash281x_CsmKeys.asm
    
    EALLOW;
    *KEY0 = 0x3340;
    *KEY1 = 0xD96A;
    *KEY2 = 0x360A;
    *KEY3 = 0x64D7;
    *KEY4 = 0x64D7;
    *KEY5 = 0x360A;
    *KEY6 = 0xD96A;
    *KEY7 = 0x3340;   
    EDIS;

    // Perform a dummy read of the password locations
    // if they match the key values, the CSM will unlock 
        
    temp = *PWL0;
    temp = *PWL1;
    temp = *PWL2;
    temp = *PWL3;
    temp = *PWL4;
    temp = *PWL5;
    temp = *PWL6;
    temp = *PWL7;
 
    // If the CSM unlocked, return succes, otherwise return
    // failure.
    if ( (*CSMSCR & 0x0001) == 0) return STATUS_SUCCESS;
    else return STATUS_FAIL;  
}
/*
SECTORA;SECTORB;SECTORC;SECTORD;SECTORE;SECTORF;SECTORG
*/
#define ERASE_SECTOR		SECTORH
#define FLASH_OPER_SUCCESS	0

#pragma CODE_SECTION(FlashErase, "ramfuncs");
Uint16 FlashErase(void)
{
	Uint16 Status = 1;
	FLASH_ST 	FlashState;

	Status = Flash_Erase(ERASE_SECTOR,&FlashState);
	if(Status != FLASH_OPER_SUCCESS) 
	{
		Status = Flash_DepRecover();
		if(Status != FLASH_OPER_SUCCESS)	//失败了，报错
		{
			return FALSE;
		}
		else	//恢复成功
		{
			Status = Flash_Erase(ERASE_SECTOR,&FlashState);
			if(Status != FLASH_OPER_SUCCESS)		//失败了，报错
			{
				return FALSE;
			}
			else
			{
				return TRUE;
			}
		}
	}
	else		//擦除成功
	{
		return TRUE;
	}
}

#pragma CODE_SECTION(WriteFlash, "ramfuncs");
Uint16 WriteFlash(void)
{
	FLASH_ST 	FlashState;
	Uint16 Status = 1;
	Uint16* ptemp = PARA_ADD;
	Uint16* pbuf = PARA_BUF;
	Uint16 i = 0;

	Status = Flash_Program(PARA_ADD,PARA_BUF,PARA_NUM,&FlashState);
	if(Status != FLASH_OPER_SUCCESS)	//失败
	{
		return FALSE;
	}
	else
	{
	}

	for(i=0;i<PARA_NUM;i++)
	{
		if(*ptemp != *pbuf)
		{
			return FALSE;
		}
		ptemp++;
		pbuf++;
	}
	return TRUE;
}
#pragma CODE_SECTION(ParaWrite, "ramfuncs");
Uint16 ParaWrite(void)
{
	DINT;

	if(CsmUnlock() == STATUS_FAIL)
	{
		//EINT;
		return FALSE;
	}

	if(FlashErase() == FALSE)
	{
		//EINT;
		return FALSE;
	}
	else if(WriteFlash() == FALSE)
	{
		//EINT;
		return FALSE;
	}

	//EINT;	//校验完再打开
	return TRUE;
}

/**
*IO模拟I2C
*read data
*Yu Ming
*Input: data address that wanted to be read
*Output: data
*/
#define SDA_IO_H		GpioDataRegs.GPASET.bit.GPIO16 = 1
#define SDA_IO_L		GpioDataRegs.GPACLEAR.bit.GPIO16 = 1
#define SDA_IO_IN		GpioCtrlRegs.GPADIR.bit.GPIO16 = 0
#define SDA_IO_OUT		GpioCtrlRegs.GPADIR.bit.GPIO16 = 1
#define SDA_IO_DATA		GpioDataRegs.GPADAT.bit.GPIO16
/*
#define SCL_IO_H		GpioDataRegs.GPASET.bit.GPIO19 = 1
#define SCL_IO_L		GpioDataRegs.GPACLEAR.bit.GPIO19 = 1
*/
#define SCL_IO_H		GpioDataRegs.GPASET.bit.GPIO7 = 1
#define SCL_IO_L		GpioDataRegs.GPACLEAR.bit.GPIO7 = 1

#define DELAY_50US	168
#define DELAY_25US	84
void SetSDAIn(void)
{
	EALLOW;
	SDA_IO_IN;
	NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
	EDIS;
}
void SetSDAOut(void)
{
	EALLOW;
	SDA_IO_OUT;
	NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
	EDIS;
}

void SendStart(void)
{
	Uint16 i=0;
	
	SDA_IO_H;					//data "1";
	NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
	SCL_IO_H;					//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	SDA_IO_L;					//data "0";
	for(i=0;i<DELAY_50US;i++)NOP;
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
}

Uint16 EEPROM_BYTE_READ(I2CMSG_EEPROM *pReadData)
{
	Uint16 Data_temp=0,i=0;
	
	//设置IO口为发送
	SetSDAOut();
	
	//发送start信号
	SendStart();
	
	//==发送选定的EEPROM地址命令;1010XXX0;
//	SCL_IO_L;	//将CLOCK置"0";
//	NOP;NOP;NOP;NOP;

//bit7;
	SDA_IO_H;		//data "1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit6;
	SDA_IO_L;	  //data "0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit5;
	SDA_IO_H;		//data "1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit4;
	SDA_IO_L;		//data "0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit3;
	SDA_IO_DATA = (pReadData->SlaveAddress >> 2) & 0x0001;		//data "A2";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit2;
	SDA_IO_DATA = (pReadData->SlaveAddress >> 1) & 0x0001;		//data "A1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit1;
	SDA_IO_DATA = (pReadData->SlaveAddress) & 0x0001;				//data "A0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit0;
	SDA_IO_L;		//data "0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;		//将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//应答;
	//设置IO口为接收
	SetSDAIn();
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;		//将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//发送要读的数据地址

	//设置IO口为发送
	SetSDAOut();
	
//bit7;
	SDA_IO_DATA = (pReadData->MemoryLowAddr >> 7) & 0x1;	//data "A7";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	 //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit6;
	SDA_IO_DATA = (pReadData->MemoryLowAddr >> 6) & 0x1;	//data "A6";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;

//bit5;
	SDA_IO_DATA = (pReadData->MemoryLowAddr >> 5) & 0x1;	//data "A5";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;

//bit4;
	SDA_IO_DATA = (pReadData->MemoryLowAddr >> 4) & 0x1;	//data "A4";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit3
	SDA_IO_DATA = (pReadData->MemoryLowAddr >> 3) & 0x1;	//data "A3";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit2;
	SDA_IO_DATA = (pReadData->MemoryLowAddr >> 2) & 0x1;	//data "A2";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit1;
	SDA_IO_DATA = (pReadData->MemoryLowAddr >> 1) & 0x1;	//data "A1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit0
	SDA_IO_DATA = pReadData->MemoryLowAddr & 0x1;	//data "A0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;

//应答信号;
	//设置IO口为接收
	SetSDAIn();
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;		//将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//==发送start位==;

	//设置IO口为发送
	SetSDAOut();
	
	SendStart();
	
//==发读命令;1010XXX1;
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit7;
	SDA_IO_H;		//data "1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit6;
	SDA_IO_L;	  //data "0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit5;
	SDA_IO_H;		//data "1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit4;
	SDA_IO_L;	//data "0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit3;
	SDA_IO_DATA = (pReadData->SlaveAddress >> 2) & 0x0001;		//data "A2";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit2;
	SDA_IO_DATA = (pReadData->SlaveAddress >> 1) & 0x0001;		//data "A1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit1;
	SDA_IO_DATA = (pReadData->SlaveAddress) & 0x0001;				//data "A0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//bit0;
	SDA_IO_H;		//data "1";	读数据;
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	  //将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//应答;
	//设置IO口为接收
	SetSDAIn();
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;		//将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
//读取数据;55H;
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//bit7;
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	Data_temp |= (SDA_IO_DATA<<7);		//data "0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//bit6;
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	Data_temp |= (SDA_IO_DATA<<6);
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//bit5
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	Data_temp |= (SDA_IO_DATA<<5);
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//bit4
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	Data_temp |= (SDA_IO_DATA<<4);
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//bit3
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	Data_temp |= (SDA_IO_DATA<<3);
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//bit2
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	Data_temp |= (SDA_IO_DATA<<2);
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//bit1
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	Data_temp |= (SDA_IO_DATA<<1);
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//bit0
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	Data_temp |= SDA_IO_DATA;
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_50US;i++)NOP;
	
//应答;
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SCL_IO_L;	//将CLOCK置"0";
	for(i=0;i<DELAY_25US;i++)NOP;

//结束位;
	//设置IO口为发送
	SetSDAOut();
	
	SDA_IO_L;		//data "0";
	for(i=0;i<DELAY_25US;i++)NOP;
	
	SCL_IO_H;		//将CLOCK置"1";
	for(i=0;i<DELAY_50US;i++)NOP;
	
	SDA_IO_H;		//data "1";
	for(i=0;i<DELAY_25US;i++)NOP;

	pReadData->MsgBuffer[0] = Data_temp;
	return TRUE;
}

Uint16 err_count=0;
Uint16 EEPROM_BYTE_WRITE(I2CMSG_EEPROM *pReadData)
{
	Uint16 i = 0;
	
	#define DELAY_TIME		4		//ms
	static Uint16 delay = 0;
	
	#define WRITE_EEPROM		0
	#define READ_EEPROM			1
	static Uint16 step = 0;
	
	#define ERR_COUNTER			5
	static Uint16 ErrCounter = 0;
	I2CMSG_EEPROM tmpmsg;
	tmpmsg.MemoryLowAddr = pReadData->MemoryLowAddr;
	tmpmsg.SlaveAddress = pReadData->SlaveAddress;

	switch(step)
	{
		case WRITE_EEPROM:
			{
				//设置IO口为发送
				SetSDAOut();
				
				//发送start信号
				SendStart();
				
			  //==发写命令;1010XXX0;
			//	SCL_IO_L;	//将CLOCK置"0";
			//	NOP;NOP;NOP;NOP;
			
			//bit7;
				SDA_IO_H;		//data "1";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit6;
				SDA_IO_L;	  //data "0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit5;
				SDA_IO_H;		//data "1";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit4;
				SDA_IO_L;		//data "0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit3;
				SDA_IO_DATA = (pReadData->SlaveAddress >> 2) & 0x0001;		//data "A2";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit2;
				SDA_IO_DATA = (pReadData->SlaveAddress >> 1) & 0x0001;		//data "A1";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit1;
				SDA_IO_DATA = (pReadData->SlaveAddress) & 0x0001;				//data "A0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit0;
				SDA_IO_L;		//data "0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;		//将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//应答;
				//设置IO口为接收
				SetSDAIn();
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;		//将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
					
			//====写数据地址;Data_Add;
				//设置IO口为发送
				SetSDAOut();
				
			//	SCL_IO_L;	//将CLOCK置"0";
			//	NOP;NOP;NOP;NOP;
			//	for(i=0;i<25;i++)NOP;
			//bit7;
				SDA_IO_DATA = (pReadData->MemoryLowAddr >> 7) & 0x1;	//data "A7";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	 //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit6;
				SDA_IO_DATA = (pReadData->MemoryLowAddr >> 6) & 0x1;	//data "A6";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
			
			//bit5;
				SDA_IO_DATA = (pReadData->MemoryLowAddr >> 5) & 0x1;	//data "A5";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
			
			//bit4;
				SDA_IO_DATA = (pReadData->MemoryLowAddr >> 4) & 0x1;	//data "A4";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit3
				SDA_IO_DATA = (pReadData->MemoryLowAddr >> 3) & 0x1;	//data "A3";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit2;
				SDA_IO_DATA = (pReadData->MemoryLowAddr >> 2) & 0x1;	//data "A2";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit1;
				SDA_IO_DATA = (pReadData->MemoryLowAddr >> 1) & 0x1;	//data "A1";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit0
				SDA_IO_DATA = pReadData->MemoryLowAddr & 0x1;	//data "A0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
			
			//应答;
				//设置IO口为接收
				SetSDAIn();
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;		//将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
			
			//====写数据;
				//设置IO口为发送
				SetSDAOut();
			//	SCL_IO_L;	//将CLOCK置"0";
			//	NOP;NOP;NOP;NOP;
			//	for(i=0;i<25;i++)NOP;
			//bit7;
				SDA_IO_DATA = (pReadData->MsgBuffer[0] >> 7) & 0x1;	//data "A7";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	 //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit6;
				SDA_IO_DATA = (pReadData->MsgBuffer[0] >> 6) & 0x1;	//data "A6";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
			
			//bit5;
				SDA_IO_DATA = (pReadData->MsgBuffer[0] >> 5) & 0x1;	//data "A5";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
			
			//bit4;
				SDA_IO_DATA = (pReadData->MsgBuffer[0] >> 4) & 0x1;	//data "A4";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit3
				SDA_IO_DATA = (pReadData->MsgBuffer[0] >> 3) & 0x1;	//data "A3";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit2;
				SDA_IO_DATA = (pReadData->MsgBuffer[0] >> 2) & 0x1;	//data "A2";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit1;
				SDA_IO_DATA = (pReadData->MsgBuffer[0] >> 1) & 0x1;	//data "A1";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
			//bit0
				SDA_IO_DATA = pReadData->MsgBuffer[0] & 0x1;	//data "A0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;	  //将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
			
			//应答;
				//设置IO口为接收
				SetSDAIn();
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SCL_IO_L;		//将CLOCK置"0";
				for(i=0;i<DELAY_25US;i++)NOP;
			
			//结束位;
				//设置IO口为发送
				SetSDAOut();
				
				SDA_IO_L;	//data "0";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				SCL_IO_H;		//将CLOCK置"1";
				for(i=0;i<DELAY_50US;i++)NOP;
				
				SDA_IO_H;		//data "1";
				for(i=0;i<DELAY_25US;i++)NOP;
				
				step = READ_EEPROM;
				break;
			}
		case READ_EEPROM:
			{
				delay++;
				if(delay >= DELAY_TIME)
				{
					delay = DELAY_TIME;
					EEPROM_BYTE_READ(&tmpmsg);
					if((pReadData->MsgBuffer[0] & 0x00FF) == (tmpmsg.MsgBuffer[0] & 0x00FF))
					{
						step = WRITE_EEPROM;
						delay = 0;
						ErrCounter = 0;
						return TRUE;
					}
					else
					{
						ErrCounter++;
						err_count++;
						if(ErrCounter >= ERR_COUNTER)
						{
							step = WRITE_EEPROM;
							delay = 0;
							return ERR_EEP;
						}
					}
				}
			}
	}
	return FALSE;
}

Uint16 EEPROM_ParaRead(Uint16 *pBuf)
{
	Uint16 i = 0;
	Uint16* pParaAdd = PARA_ADD_EEP;
	I2CMSG_EEPROM tmpmsg;
	Uint16 temp = 0;
	Uint16 Ver_H,Ver_L = 0;
	
	tmpmsg.MemoryLowAddr = (Uint32)pParaAdd;
	tmpmsg.SlaveAddress = I2C_SLAVE_ADDR_PARA;
	tmpmsg.MsgBuffer[0] = 0;

	for(;i<PAR_ADV_DEF_SIZE;i++)			//读取数据
	{
		EEPROM_BYTE_READ(&tmpmsg);
		*pBuf = tmpmsg.MsgBuffer[0];
		if(tmpmsg.MemoryLowAddr == P103_VER_H)
		{
			Ver_H = tmpmsg.MsgBuffer[0];
		}
		else if(tmpmsg.MemoryLowAddr == P104_VER_H)
		{
			Ver_L = tmpmsg.MsgBuffer[0];
		}

		pBuf++;
		tmpmsg.MemoryLowAddr++;
	}

	if(Ver_H != VER_H_CODE || Ver_L != VER_L_CODE)		//版本不对
	{
		return FALSE;
	}
	return TRUE;
}

Uint16 EEPROM_ParaRecover(Uint16* pBuf)
{
	Uint16* DefaultBuf = pBuf;
	static I2CMSG_EEPROM tmpmsg;
	Uint16 temp = 0;
	static Uint16 i = 0;
	
	if(ErrorCode == ERR_EEP)
	{
		return FALSE;
	}
	
	DefaultParaRead(DefaultBuf);		//读取默认参数

	tmpmsg.MsgBuffer[0] = (*pBuf & 0x00FF);
	tmpmsg.MemoryLowAddr = (Uint32)PARA_ADD_EEP;
	tmpmsg.SlaveAddress = I2C_SLAVE_ADDR_PARA;
	
	for(;i<PAR_ADV_DEF_SIZE;)
	{
		temp = EEPROM_BYTE_WRITE(&tmpmsg);
		if(temp == ERR_EEP)
		{
			ErrorCode = ERR_EEP;
			break;
		}
		else if(temp == TRUE)
		{
			tmpmsg.MemoryLowAddr++;
			tmpmsg.MsgBuffer[0] = (*(++pBuf) & 0x00FF);
			i++;
		}
	}
	return TRUE;
}


/*
HMI恢复参数
HMI更改参数
*/
void EEPROM_ParaWrite(Uint16* pBuf,CHANGE_PARA_BUF* pCPara)
{
	static I2CMSG_EEPROM tmpmsg;
	Uint16 i = 0;
	Uint16 temp = 0;
	
	#define INIT				0
	#define WRITE				1
	static Uint16 step = 0;
	
	if((ErrorCode == ERR_EEP)||(ErrorCode == ERR_INSIDE))
	{
		return;
	}
	
	if(HMI_PAR_RESTOR == 1)		//恢复出厂参数
	{
		pCPara->EEPROM_State = 1;
		
		tmpmsg.MsgBuffer[0] = (*pBuf & 0x00FF);
		tmpmsg.MemoryLowAddr = (Uint32)PARA_ADD_EEP;
		tmpmsg.SlaveAddress = I2C_SLAVE_ADDR_PARA;
		
		for(;i<PAR_ADV_DEF_SIZE;)
		{
			temp = EEPROM_BYTE_WRITE(&tmpmsg);
			if(temp == ERR_EEP)
			{
				ErrorCode = ERR_EEP;
				break;
			}
			else if(temp == TRUE)
			{
				tmpmsg.MemoryLowAddr++;
				tmpmsg.MsgBuffer[0] = (*(++pBuf) & 0x00ff);
				i++;
			}
		}
		HMI_PAR_RESTOR = 0;
		pCPara->EEPROM_State = 0;
	}
	else
	{
		if(pCPara->Err == 1)
		{
			ErrorCode = ERR_INSIDE;
		}
		else if((pCPara->Cnt == 0))//||(pCPara->EEPROM_State == 1))
		{
			return;
		}
		else	//写入参数
		{
			switch(step)
			{
				case INIT:
				{
					pCPara->EEPROM_State = 1;
		
					tmpmsg.SlaveAddress = I2C_SLAVE_ADDR_PARA;
					tmpmsg.MemoryLowAddr = (Uint32)PARA_ADD_EEP + pCPara->Par_Cnt[(pCPara->Cnt-1)];
					tmpmsg.MsgBuffer[0] = pBuf[(pCPara->Par_Cnt[(pCPara->Cnt-1)])];
					
					step = WRITE;
					break;
				}
				case WRITE:
				{
					temp = EEPROM_BYTE_WRITE(&tmpmsg);
					if(temp == ERR_EEP)
					{
						ErrorCode = ERR_EEP;
						break;
					}
					else if(temp == TRUE)
					{
						pCPara->Cnt--;
						if(pCPara->Cnt>0)		//大于0，表示还有没有写入的数据
						{
							tmpmsg.MemoryLowAddr = (Uint32)PARA_ADD_EEP + pCPara->Par_Cnt[(pCPara->Cnt-1)];
							tmpmsg.MsgBuffer[0] = pBuf[(pCPara->Par_Cnt[(pCPara->Cnt-1)])];
						}
						else		//=0,写入结束
						{
							pCPara->EEPROM_State = 0;
							step = INIT;
						}
					}
					break;
				}
			}
		}
	}
}

Uint16 add = 60;
Uint16 eepdata[60]={0};
Uint16 data = 0;
void EEPROM_Test(void)
{
	static I2CMSG_EEPROM tmpmsg;
	static Uint16 step = 0;
	Uint16 temp = 0;
	
	if(step==0)
	{
		tmpmsg.MsgBuffer[0] = 0x00;
		tmpmsg.MemoryLowAddr = add;
		tmpmsg.SlaveAddress = I2C_SLAVE_ADDR_PARA;
		step = 1;
	}
	
	//EEPROM_BYTE_READ(&tmpmsg);
	//data = tmpmsg.MsgBuffer[0];

/*	for(add=0;add<60;add++)
	{
		tmpmsg.MemoryLowAddr = add;
		EEPROM_BYTE_READ(&tmpmsg);
		eepdata[add] = tmpmsg.MsgBuffer[0];
	}*/

	

	temp = EEPROM_BYTE_WRITE(&tmpmsg);
	if(temp == TRUE)
	{
		if(err_count!=0){data++;err_count=0;}
		tmpmsg.MsgBuffer[0]++;
		tmpmsg.MsgBuffer[0] = tmpmsg.MsgBuffer[0] & 0x00FF;
		tmpmsg.MemoryLowAddr++;
		tmpmsg.MemoryLowAddr = tmpmsg.MemoryLowAddr & 0x00FF;
		if(tmpmsg.MemoryLowAddr<60)tmpmsg.MemoryLowAddr = 60;

	}
	else if(temp == ERR_EEP)
	{
		while(1){}
	}
}












