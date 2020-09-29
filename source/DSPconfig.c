/**
 * Copyright (c) 2010, 上海积致电子科技有限公司
 * All rights reserved.
 *
 * @file DSPconfig.c
 * Dsp外设模块初始化函数文件.
 * 函数名同TI默认
 * 包括Adc模块、EPwm模块、Gpio模块、SCI通讯模块、SPI模块初始化
 * 
 * 
 *
 * @author yemj
 * @version 0.1
 * @date 2010-3-10
 *
 */


#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "DMCparameter.h"
#include "DMCvars.h"
#include "build.h"


/**
 * Gpio初始化设定
 * 
 * This function initializes the Gpio to a known state.
 *
 * 
 */
void InitGpio(void)
{

	// Each GPIO pin can be:
	// a) a GPIO input/output
	// b) peripheral function 1
	// c) peripheral function 2
	// d) peripheral function 3
	// By default, all are GPIO Inputs
	EALLOW;

	// Enable PWM1-3 on GPIO0-GPIO5 for SVPWM (PWM_A+/A-/B+/B-/C+/C-)
	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO0
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO1
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;   // Enable pullup on GPIO2
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pullup on GPIO3
	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;   // Enable pullup on GPIO4
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pullup on GPIO5
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;  // GPIO1 = PWM1B
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // GPIO2 = PWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // GPIO3 = PWM2B
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;  // GPIO4 = PWM3A
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;  // GPIO5 = PWM3B

	// Enable GPIO input on GPIO6 (OZ)
	GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;   // Enable pullup on GPIO6
	GpioDataRegs.GPASET.bit.GPIO6 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;  // GPIO6 = GPIO6
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 0;   // GPIO6 = input

	// Enable GPIO input on GPIO7 (SCL)
	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;   // Enable pullup on GPIO7
	GpioDataRegs.GPASET.bit.GPIO7 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;  // GPIO7 = GPIO7
	GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;   // GPIO7 = output

	// Enable GPIO input on GPIO8 (SW4)
	GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;   // Enable pullup on GPIO8
	GpioDataRegs.GPASET.bit.GPIO8 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;  // GPIO8 = GPIO8
	GpioCtrlRegs.GPADIR.bit.GPIO8 = 0;   // GPIO8 = input

	// Enable GPIO input on GPIO9 (SW1)
	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;   // Enable pullup on GPIO9
	GpioDataRegs.GPASET.bit.GPIO9 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;  // GPIO9 = GPIO9
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;   // GPIO9 = input

	// Enable GPIO input on GPIO10 (HALL_B)
	GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;  // Enable pullup on GPIO10
	GpioDataRegs.GPASET.bit.GPIO10 = 1;  // Load output latch
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0; // GPIO10 = GPIO10
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;   // GPIO10 = input

	// Enable GPIO input on GPIO11 (UP)
	GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;  // Enable pullup on GPIO11
	GpioDataRegs.GPASET.bit.GPIO11 = 1;  // Load output latch
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0; // GPIO11 = GPIO11
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 0;  // GPIO11 = input

	// Enable Trip Zone on GPIO12 (PB_FLT)
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pullup on GPIO12
	GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3; // asynch input
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // GPIO12 = TZ1

	// Enable GPIO input on GPIO16 (reserved)
	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;  // Enable pullup on GPIO16
	GpioDataRegs.GPASET.bit.GPIO16 = 1;  // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0; // GPIO16 = GPIO16
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;  // GPIO16 = input

	// Enable GPIO input on GPIO17 (SW3)
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pullup on GPIO17
	GpioDataRegs.GPASET.bit.GPIO17 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;  // GPIO17 = GPIO17
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;   // GPIO17 = input

	// Enable GPIO input on GPIO18 (SW2)
	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;  // Enable pullup on GPIO18
	GpioDataRegs.GPASET.bit.GPIO18 = 1;  // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0; // GPIO18 = GPIO18
	GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;  // GPIO18 = input

	// Enable GPIO input on GPIO19 (ecap1)
	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;  // Enable pullup on GPIO19
	GpioDataRegs.GPASET.bit.GPIO19 = 1;  // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;  // GPIO19 = input

	// Enable EQEP1 on GPIO20-GPIO21 (ENC_A/B)
	GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pullup on GPIO20
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pullup on GPIO21
	GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0; // Synch to SYSCLKOUT
	GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0; // Synch to SYSCLKOUT
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;  // GPIO20 is EQEP1A
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;  // GPIO21 is EQEP1B

	// Enable GPIO input on GPIO22 (FTKG)
	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO22
//	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;   // Disable pullup on GPIO22
	GpioDataRegs.GPASET.bit.GPIO22 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;  // GPIO22 = GPIO22
	GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;   // GPIO22 = input

	// Enable GPIO input on GPIO23 (NDL_LIGHT)
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO23
	GpioDataRegs.GPASET.bit.GPIO23 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;  // GPIO23 = GPIO23
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;   // GPIO23 = output

	// Enable GPIO input on GPIO24 (HALL_C)
	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pullup on GPIO24
	GpioDataRegs.GPASET.bit.GPIO24 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;  // GPIO24 = GPIO24
	GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;   // GPIO24 = input

 	// Enable SCI-A on GPIO28-GPIO29 (RXD,TXD)
	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;   // Enable pullup on GPIO28
	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3; // Asynch input
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;  // GPIO28 = SCIRXDA
	GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
/*	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;   // Enable pullup on GPIO29
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;  // GPIO29 = SCITXDA
*/
	// Enable GPIO output on GPIO30, set it high (CL)
	GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;   // Enable pullup on GPIO30
	GpioDataRegs.GPASET.bit.GPIO30 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;  // GPIO30 = GPIO30
	GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;   // GPIO30 = output

	// Enable GPIO input on GPIO31 (reserved)
	GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;   // Enable pullup on GPIO31
	GpioDataRegs.GPASET.bit.GPIO31 = 1;   // Load output latch
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;  // GPIO31 = GPIO31
	GpioCtrlRegs.GPADIR.bit.GPIO31 = 0;   // GPIO31 = input

	// Enable I2C-A on GPIO32 - GPIO33 (SDA,SCL)
	GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pullup on GPIO32
	GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3; // Asynch input
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;  // GPIO32 = SDAA
	GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pullup on GPIO33
	GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // Asynch input
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;  // GPIO33 = SCLA

	// Enable GPIO output on GPIO34, set it high (PWM_EN)
	GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;   // Enable pullup on GPIO34
	GpioDataRegs.GPBSET.bit.GPIO34 = 1;   // Load output latch
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;  // GPIO34 = GPIO34
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;   // GPIO34 = output

	// Config XINT1-XINT3
	GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 50;	// Each sampling window is 2*QUALPRD0*SYSCLKOUT
	GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 50;	// Each sampling window is 2*QUALPRD0*SYSCLKOUT
	GpioCtrlRegs.GPACTRL.bit.QUALPRD2 = 50;	// Each sampling window is 2*QUALPRD0*SYSCLKOUT
	GpioCtrlRegs.GPACTRL.bit.QUALPRD3 = 50;	// Each sampling window is 2*QUALPRD0*SYSCLKOUT
	GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0; 	// GPIO6 Synch to SYSCLKOUT
	GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 2; 	// GPIO11 Qual using 6 samples (10us)
	GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 2; 	// GPIO7 Qual using 6 samples (10us)

	// XINT GPIO SEL
	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 7;	// XINT1 is GPIO7(HALLA)
	GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 6;	// XINT2 is GPIO6(OZ)
	GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL = 11;	// XINT3 is GPIO11(UP)

	// Configure XINT Edge
	XIntruptRegs.XINT1CR.bit.POLARITY = 1;      // Rising edge interrupt
	XIntruptRegs.XINT2CR.bit.POLARITY = 0;      // Falling edge interrupt
	XIntruptRegs.XINT3CR.bit.POLARITY = 0;      // Falling edge interrupt

	// Enable XINT1-XINT3
	XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1
	XIntruptRegs.XINT2CR.bit.ENABLE = 1;        // Enable XINT2
	XIntruptRegs.XINT3CR.bit.ENABLE = 1;        // Enable XINT3

	GpioCtrlRegs.AIOMUX1.bit.AIO12 = 0;   // Configure AIO12 for B4 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO14 = 0;   // Configure AIO14 for B6 (analog input) operation
	GpioCtrlRegs.AIODIR.bit.AIO12 = 0;
	GpioCtrlRegs.AIODIR.bit.AIO14 = 0;
	EDIS;

}//end of InitGpio


//===========================================================================
// No more.
//===========================================================================



