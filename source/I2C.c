/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
 * All rights reserved.
 *
 * @file I2C.c
 * 数码管显示处理文件.
 * 包含数码管显示所需的初始化、数据处理、数据显示函数
 * 
 *
 * @author yemj
 * @version 0.1
 * @date 2010-2-25
 *
 */


#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "GlobalVars.h"



/**
 * I2CA_Init
 * I2CA模块初始化函数
 * Enable FIFO mode
 * 
 */
void I2CA_Init(void)
{
	// Initialize I2C
	I2caRegs.I2CMDR.all = 0x0000;	// reset I2C module

	I2caRegs.I2CSAR = 0x0050;		// Slave address - EEPROM control code

	//I2C Baud = 100Khz
	I2caRegs.I2CPSC.bit.IPSC = 5;	// Prescaler - need 7-12 Mhz on module clk (10Mhz = 60Mhz/(5+1))
	I2caRegs.I2CCLKL = 45;			// NOTE: must be non zero (I2CCLKL+5)
	I2caRegs.I2CCLKH = 45;			// NOTE: must be non zero (I2CCLKH+5)
	I2caRegs.I2CIER.all = 0x24;		// Enable SCD & ARDY interrupts

	I2caRegs.I2CMDR.all = 0x0020;	// Take I2C out of reset
   									// Stop I2C when suspended

	I2caRegs.I2CFFTX.all = 0x6040;	// Enable FIFO mode and TXFIFO, clear RXFFINT 
	I2caRegs.I2CFFRX.all = 0x2040;	// Enable RXFIFO, clear RXFFINT

}


/**
 * I2CA_WriteData
 * I2CA模块写数据操作
 * 
 * 输出: I2C状态标志
 */
Uint16 I2CA_WriteData(struct I2CMSG *msg)
{
	Uint16 i;

	// Wait until the STP bit is cleared from any previous master communication.
	// Clearing of this bit by the module is delayed until after the SCD bit is
	// set. If this bit is not checked prior to initiating a new message, the
	// I2C could get confused.
	if (I2caRegs.I2CMDR.bit.STP == 1)
	{
		return I2C_STP_NOT_READY_ERROR;
	}

	// Setup slave address
	I2caRegs.I2CSAR = msg->SlaveAddress;

	// Check if bus busy
	if (I2caRegs.I2CSTR.bit.BB == 1)
	{
		return I2C_BUS_BUSY_ERROR;
	}

	// Setup number of bytes to send (MsgBuffer + Address)
	I2caRegs.I2CCNT = msg->NumOfBytes+1;//+2;

	// Setup data to send
//	I2caRegs.I2CDXR = msg->MemoryHighAddr;
	I2caRegs.I2CDXR = msg->MemoryLowAddr;
//	for (i=0; i<msg->NumOfBytes-2; i++)
	for (i=0; i<msg->NumOfBytes; i++)
	{
		I2caRegs.I2CDXR = *(msg->MsgBuffer+i);
	}

	// Send start as master transmitter
	I2caRegs.I2CMDR.all = 0x6E20;

	return I2C_SUCCESS;
}


/**
 * I2CA_ReadData
 * I2CA模块读数据操作
 * 
 * 输出: I2C状态标志
 */
Uint16 I2CA_ReadData(struct I2CMSG *msg)
{
	// Wait until the STP bit is cleared from any previous master communication.
	// Clearing of this bit by the module is delayed until after the SCD bit is
	// set. If this bit is not checked prior to initiating a new message, the
	// I2C could get confused.
	if (I2caRegs.I2CMDR.bit.STP == 1)
	{
		return I2C_STP_NOT_READY_ERROR;
	}

	// Setup slave address
	I2caRegs.I2CSAR = msg->SlaveAddress;

	if(msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
	{//发送读指令
		// Check if bus busy
		if (I2caRegs.I2CSTR.bit.BB == 1)
		{
			return I2C_BUS_BUSY_ERROR;
		}
		I2caRegs.I2CCNT = 1;//2;
//		I2caRegs.I2CDXR = msg->MemoryHighAddr;
		I2caRegs.I2CDXR = msg->MemoryLowAddr;
 		I2caRegs.I2CMDR.all = 0x2620;			// Send data to setup EEPROM address
	}
	else if(msg->MsgStatus == I2C_MSGSTAT_RESTART)
	{//准备接收数据
		I2caRegs.I2CCNT = msg->NumOfBytes;		// Setup how many bytes to expect
		I2caRegs.I2CMDR.all = 0x2C20;			// Send restart as master receiver
	}

	return I2C_SUCCESS;
}


/**
 * I2CA_Write
 * I2CA写操作
 * 
 */
void I2CA_Write(struct I2CMSG *msg)
{

	//////////////////////////////////
	// Write data to EEPROM section //
	//////////////////////////////////

	// Check the outgoing message to see if it should be sent.
	// In this example it is initialized to send with a stop bit.
	if(msg->MsgStatus == I2C_MSGSTAT_SEND_WITHSTOP)
	{
		// If communication is correctly initiated, set msg status to busy
		// and update CurrentMsgPtr for the interrupt service routine.
		// Otherwise, do nothing and try again next loop. Once message is
		// initiated, the I2C interrupts will handle the rest. Search for
		// ICINTR1A_ISR in the i2c.c file.
		if(I2CA_WriteData(msg) == I2C_SUCCESS)
		{
			CurrentI2CMsgPtr = msg;
			msg->MsgStatus = I2C_MSGSTAT_WRITE_BUSY;
		}
	}  // end of write section
}


/**
 * I2CA_Read
 * I2CA读操作
 * 
 */
void I2CA_Read(struct I2CMSG *msg)
{

	///////////////////////////////////
	// Read data from EEPROM section //
	///////////////////////////////////

	// Check incoming message status.
	if(msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
	{
		// EEPROM address setup portion
		if(I2CA_ReadData(msg) == I2C_SUCCESS)
		{
			// Update current message pointer and message status
			CurrentI2CMsgPtr = msg;
			msg->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
		}
	}

	// Once message has progressed past setting up the internal address
	// of the EEPROM, send a restart to read the data bytes from the
	// EEPROM. Complete the communique with a stop bit. MsgStatus is
	// updated in the interrupt service routine.
	else if(msg->MsgStatus == I2C_MSGSTAT_RESTART)
	{
		// Read data portion
		if(I2CA_ReadData(msg) == I2C_SUCCESS)
		{
			// Update current message pointer and message status
			CurrentI2CMsgPtr = msg;
			msg->MsgStatus = I2C_MSGSTAT_READ_BUSY;
		}
	}
}


/**
 * i2c_int1a_isr
 * I2C模块中断函数
 * 
 *  
 */
interrupt void i2c_int1a_isr(void)     // I2C-A
{
	Uint16 IntSource, i;

	// Read interrupt source
	IntSource = I2caRegs.I2CISRC.all;

	// Interrupt source = stop condition detected
	if(IntSource == I2C_SCD_ISRC)
	{
		if (CurrentI2CMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY)
		{
			// If completed message was writing data, reset msg to inactive state
			// and reset TXFIFO pointer
			CurrentI2CMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
			I2caRegs.I2CFFTX.bit.TXFFRST = 0;	// Reset TXFIFO pointer to 0000
			I2caRegs.I2CFFTX.bit.TXFFRST = 1;	// and then Enable TXFIFO
		}
		else
		{
			if(CurrentI2CMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
			{
				// If a message receives a NACK during the address setup portion of the
				// EEPROM read, the code further below included in the register access ready
				// interrupt source code will generate a stop condition. After the stop
				// condition is received (here), set the message status to try again.
				// User may want to limit the number of retries before generating an error.
				CurrentI2CMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
			}
			else if (CurrentI2CMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY)
			{
				// If completed message was reading EEPROM data, reset msg to inactive state
				// and read data from FIFO.
				CurrentI2CMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
				for(i=0; i < CurrentI2CMsgPtr->NumOfBytes; i++)
				{
					CurrentI2CMsgPtr->MsgBuffer[i] = I2caRegs.I2CDRR;
				}
				// then reset RXFIFO pointer
				I2caRegs.I2CFFRX.bit.RXFFRST = 0;	// Reset RXFIFO pointer to 0000
				I2caRegs.I2CFFRX.bit.RXFFRST = 1;	// and then Enable TXFIFO
			}
		}
	}  // end of stop condition detected

	// Interrupt source = Register Access Ready
	// This interrupt is used to determine when the EEPROM address setup portion of the
	// read data communication is complete. Since no stop bit is commanded, this flag
	// tells us when the message has been sent instead of the SCD flag. If a NACK is
	// received, clear the NACK bit and command a stop. Otherwise, move on to the read
	// data portion of the communication.
	else if(IntSource == I2C_ARDY_ISRC)
	{
		if(I2caRegs.I2CSTR.bit.NACK == 1)
		{
			I2caRegs.I2CMDR.bit.STP = 1;
			I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
		}
		else if(CurrentI2CMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
		{
			// set msg to RESTART state(Rx Data), and Reset RXFIFO pointer
			CurrentI2CMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;
			I2caRegs.I2CFFTX.bit.TXFFRST = 0;	// Reset TXFIFO pointer to 0000
			I2caRegs.I2CFFTX.bit.TXFFRST = 1;	// and then Enable TXFIFO
		}
	}  // end of register access ready

	else
	{
		// Generate some error due to invalid interrupt source
//		asm("   ESTOP0");
	}

	// Enable future I2C (PIE Group 8) interrupts
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}


//===========================================================================
// No more.
//===========================================================================
