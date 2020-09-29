// TI File $Revision: /main/7 $
// Checkin $Date: November 10, 2009   14:05:15 $
//###########################################################################
//
// FILE:   DSP2803x_SysCtrl.c
//
// TITLE:  DSP2803x Device System Control Initialization & Support Functions.
//
// DESCRIPTION:
//
//         Example initialization of system resources.
//
//###########################################################################
// $TI Release: 2803x C/C++ Header Files V1.21 $
// $Release Date: December 1, 2009 $
//###########################################################################

#include "DSP2803x_Device.h"     // Headerfile Include File

// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped to a load and
// run address using the linker cmd file.

#pragma CODE_SECTION(InitFlash, "ramfuncs");

//---------------------------------------------------------------------------
// InitSysCtrl:
//---------------------------------------------------------------------------
// This function initializes the System Control registers to a known state.
// - Disables the watchdog
// - Set the PLLCR for proper SYSCLKOUT frequency
// - Set the pre-scaler for the high and low frequency peripheral clocks
// - Enable the clocks to the peripherals

void InitSysCtrl(void)
{

   // Disable the watchdog
   DisableDog();

   // *IMPORTANT*
   // The Device_cal function, which copies the ADC & oscillator calibration values
   // from TI reserved OTP into the appropriate trim registers, occurs automatically
   // in the Boot ROM. If the boot ROM code is bypassed during the debug process, the
   // following function MUST be called for the ADC and oscillators to function according
   // to specification. The clocks to the ADC MUST be enabled before calling this
   // function.
   // See the device data manual and/or the ADC Reference
   // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1; // Enable ADC peripheral clock
        (*Device_cal)();
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0; // Return ADC clock to original state
        EDIS;

   // Select Internal Oscillator 1 as Clock Source (default), and turn off all unused clocks to
   // conserve power.
   IntOsc1Sel();
// XtalOscSel();

   // Initialize the PLL control: PLLCR and CLKINDIV
   // DSP28_PLLCR and DSP28_CLKINDIV are defined in DSP2803x_Examples.h
   InitPll(DSP28_PLLCR,DSP28_DIVSEL);
   // Initialize the peripheral clocks
   InitPeripheralClocks();
}

//---------------------------------------------------------------------------
// Example: InitFlash:
//---------------------------------------------------------------------------
// This function initializes the Flash Control registers

//                   CAUTION
// This function MUST be executed out of RAM. Executing it
// out of OTP/Flash will yield unpredictable results

void InitFlash(void)
{
   EALLOW;
   //Enable Flash Pipeline mode to improve performance
   //of code executed from Flash.
   FlashRegs.FOPT.bit.ENPIPE = 1;

   //                CAUTION
   //Minimum waitstates required for the flash operating
   //at a given CPU rate must be characterized by TI.
   //Refer to the datasheet for the latest information.

   //Set the Paged Waitstate for the Flash
   FlashRegs.FBANKWAIT.bit.PAGEWAIT = 2;

   //Set the Random Waitstate for the Flash
   FlashRegs.FBANKWAIT.bit.RANDWAIT = 2;

   //Set the Waitstate for the OTP
   FlashRegs.FOTPWAIT.bit.OTPWAIT = 2;

   //                CAUTION
   //ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
   FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
   FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;
   EDIS;

   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.

   asm(" RPT #7 || NOP");
}

//---------------------------------------------------------------------------
// Example: ServiceDog:
//---------------------------------------------------------------------------
// This function resets the watchdog timer.
// Enable this function for using ServiceDog in the application

void ServiceDog(void)
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;
}

//---------------------------------------------------------------------------
// Example: DisableDog:
//---------------------------------------------------------------------------
// This function disables the watchdog timer.

void DisableDog(void)
{
    EALLOW;
    SysCtrlRegs.WDCR= 0x0068;
    EDIS;
}

//---------------------------------------------------------------------------
// Example: InitPll:
//---------------------------------------------------------------------------
// This function initializes the PLLCR register.

void InitPll(Uint16 val, Uint16 divsel)
{
   volatile Uint16 iVol;

   // Make sure the PLL is not running in limp mode
   if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
   {
      EALLOW;
      // OSCCLKSRC1 failure detected. PLL running in limp mode.
      // Re-enable missing clock logic.
      SysCtrlRegs.PLLSTS.bit.MCLKCLR = 1;
      EDIS;
      // Replace this line with a call to an appropriate
      // SystemShutdown(); function.
      asm("        ESTOP0");     // Uncomment for debugging purposes
   }

   // DIVSEL MUST be 0 before PLLCR can be changed from
   // 0x0000. It is set to 0 by an external reset XRSn
   // This puts us in 1/4
   if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
   {
       EALLOW;
       SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
       EDIS;
   }

   // Change the PLLCR
   if (SysCtrlRegs.PLLCR.bit.DIV != val)
   {

      EALLOW;
      // Before setting PLLCR turn off missing clock detect logic
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
      SysCtrlRegs.PLLCR.bit.DIV = val;
      EDIS;

      // Optional: Wait for PLL to lock.
      // During this time the CPU will switch to OSCCLK/2 until
      // the PLL is stable.  Once the PLL is stable the CPU will
      // switch to the new PLL value.
      //
      // This time-to-lock is monitored by a PLL lock counter.
      //
      // Code is not required to sit and wait for the PLL to lock.
      // However, if the code does anything that is timing critical,
      // and requires the correct clock be locked, then it is best to
      // wait until this switching has completed.

      // Wait for the PLL lock bit to be set.

      // The watchdog should be disabled before this loop, or fed within
      // the loop via ServiceDog().

      // Uncomment to disable the watchdog
      DisableDog();

      while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)
      {
          // Uncomment to service the watchdog
          // ServiceDog();
      }

      EALLOW;
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
      EDIS;
    }

    // If switching to 1/2
    if((divsel == 1)||(divsel == 2))
    {
        EALLOW;
        SysCtrlRegs.PLLSTS.bit.DIVSEL = divsel;
        EDIS;
    }

    // If switching to 1/1
    // * First go to 1/2 and let the power settle
    //   The time required will depend on the system, this is only an example
    // * Then switch to 1/1
    if(divsel == 3)
    {
        EALLOW;
        SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
        DELAY_US(50L);
        SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;
        EDIS;
    }
}

//--------------------------------------------------------------------------
// Example: InitPeripheralClocks:
//---------------------------------------------------------------------------
// This function initializes the clocks to the peripheral modules.
// First the high and low clock prescalers are set
// Second the clocks are enabled to each peripheral.
// To reduce power, leave clocks to unused peripherals disabled
//
// Note: If a peripherals clock is not enabled then you cannot
// read or write to the registers for that peripheral

void InitPeripheralClocks(void)
{
   EALLOW;

// LOSPCP prescale register settings, normally it will be set to default values

   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;  // GPIO18 = XCLKOUT
   SysCtrlRegs.LOSPCP.all = 0x0002;

// XCLKOUT to SYSCLKOUT ratio.  By default XCLKOUT = 1/4 SYSCLKOUT
   SysCtrlRegs.XCLK.bit.XCLKOUTDIV=2;

// Peripheral clock enables set for the selected peripherals.
// If you are not using a peripheral leave the clock off
// to save on power.
//
// Note: not all peripherals are available on all 2803x derivates.
// Refer to the datasheet for your particular device.
//
// This function is not written to be an example of efficient code.

   SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;      // ADC
   SysCtrlRegs.PCLKCR3.bit.COMP1ENCLK = 0;    // COMP1
   SysCtrlRegs.PCLKCR3.bit.COMP2ENCLK = 0;    // COMP2
   SysCtrlRegs.PCLKCR3.bit.COMP3ENCLK = 0;    // COMP3
   SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 1;    // eCAP1
   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK = 0;    // eCAN-A
   SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;    // eQEP1
   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;    // ePWM1
   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;    // ePWM2
   SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;    // ePWM3
   SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0;    // ePWM4
   SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 0;    // ePWM5
   SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 0;    // ePWM6
   SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 0;    // ePWM7
   SysCtrlRegs.PCLKCR0.bit.HRPWMENCLK = 0;    // HRPWM
   SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;     // I2C
   SysCtrlRegs.PCLKCR0.bit.LINAENCLK = 0;     // LIN-A
   SysCtrlRegs.PCLKCR3.bit.CLA1ENCLK = 0;     // CLA1
   SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 0;     // SCI-A
   SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 0;     // SPI-A
   SysCtrlRegs.PCLKCR0.bit.SPIBENCLK = 0;     // SPI-B

   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;     // Enable TBCLK within the ePWM

   EDIS;
}

//---------------------------------------------------------------------------
// Example: IntOsc1Sel:
//---------------------------------------------------------------------------
// This function switches to Internal Oscillator 1 and turns off all other clock
// sources to minimize power consumption

void IntOsc1Sel (void) {
    EALLOW;
    SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0;
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL=0;  // Clk Src = INTOSC1
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF=1;     // Turn off XCLKIN
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF=1;    // Turn off XTALOSC
    SysCtrlRegs.CLKCTL.bit.INTOSC2OFF=1;    // Turn off INTOSC2
    EDIS;
}

//---------------------------------------------------------------------------
// Example: IntOsc2Sel:
//---------------------------------------------------------------------------
// This function switches to Internal oscillator 2 from External Oscillator
// and turns off all other clock sources to minimize power consumption
// NOTE: If there is no external clock connection, when switching from
//       INTOSC1 to INTOSC2, EXTOSC and XLCKIN must be turned OFF prior
//       to switching to internal oscillator 1

void IntOsc2Sel (void) {
    EALLOW;
    SysCtrlRegs.CLKCTL.bit.INTOSC2OFF = 0;     // Turn on INTOSC2
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 1;  // Switch to INTOSC2
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 1;      // Turn off XCLKIN
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 1;     // Turn off XTALOSC
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;   // Switch to Internal Oscillator 2
    SysCtrlRegs.CLKCTL.bit.WDCLKSRCSEL = 1;    // Switch Watchdog Clk Src to INTOSC2
    SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 1;     // Turn off INTOSC1
    EDIS;
}

//---------------------------------------------------------------------------
// Example: XtalOscSel:
//---------------------------------------------------------------------------
// This function switches to External CRYSTAL oscillator and turns off all other clock
// sources to minimize power consumption. This option may not be available on all
// device packages

void XtalOscSel (void)  {
     EALLOW;
     SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 0;     // Turn on XTALOSC
     SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 1;      // Turn off XCLKIN
     SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;  // Switch to external clock
     SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;   // Switch from INTOSC1 to INTOSC2/ext clk
     SysCtrlRegs.CLKCTL.bit.WDCLKSRCSEL = 1;    // Switch Watchdog Clk Src to external clock
     SysCtrlRegs.CLKCTL.bit.INTOSC2OFF = 1;     // Turn off INTOSC2
     SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 1;     // Turn off INTOSC1
     EDIS;
}


//---------------------------------------------------------------------------
// Example: ExtOscSel:
//---------------------------------------------------------------------------
// This function switches to External oscillator and turns off all other clock
// sources to minimize power consumption.

void ExtOscSel (void)  {
     EALLOW;
     SysCtrlRegs.XCLK.bit.XCLKINSEL = 1;       // 1-GPIO19 = XCLKIN, 0-GPIO38 = XCLKIN
     SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 1;    // Turn on XTALOSC
     SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 0;     // Turn on XCLKIN
     SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0; // Switch to external clock
     SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;  // Switch from INTOSC1 to INTOSC2/ext clk
     SysCtrlRegs.CLKCTL.bit.WDCLKSRCSEL = 1;   // Switch Watchdog Clk Src to external clock
     SysCtrlRegs.CLKCTL.bit.INTOSC2OFF = 1;    // Turn off INTOSC2
     SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 1;    // Turn off INTOSC1
     EDIS;
}

void InitECapture(void)
{

	// Code snippet for CAP mode Delta Time, Rising and Falling 
	// edge triggers 
	// InitializationTime 
	//======================= 
	// ECAP module 1 config 
	ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap1Regs.ECCTL1.bit.CAP2POL = EC_RISING; 
	ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;
	ECap1Regs.ECCTL1.bit.CAP4POL = EC_RISING; 
	ECap1Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
	ECap1Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE; 
	ECap1Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
	ECap1Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE; 
	ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
	ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1; 
	ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS; 
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
	ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE; 
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; 

	ECap1Regs.ECEINT.bit.CEVT1 = 0;            
	ECap1Regs.ECEINT.bit.CEVT2 = 0;
	ECap1Regs.ECEINT.bit.CEVT3 = 0;
	ECap1Regs.ECEINT.bit.CEVT4 = 0;			// 4 events = interrupt
}

//===========================================================================
// End of file.
//===========================================================================
