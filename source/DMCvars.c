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

#include "DSP2803x_Device.h"			// DSP2803x Headerfile Include File
#include "DMCparameter.h"

#include "DMCobjects.h"



// Instance a few transform objects
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;

// Instance PID regulators to regulate the d and q synchronous axis currents, and speed
PIDREG3 pid1_id = PIDREG3_DEFAULTS;
PIDREG3 pid1_iq = PIDREG3_DEFAULTS;
PIDREG3 pid1_spd = PIDREG3_DEFAULTS;
PIDREG4 pid1_pos = PIDREG4_DEFAULTS;

PIDREG3 pid1_pos2 = PIDREG3_DEFAULTS;
CURVE curve1 = CURVE_DEFAULTS;
FEEDFORWARD_TORQUE feedforward_torque = FEEDFORWARD_TORQUE_DEFAULTS;
FEEDFORWARD_SPD feedforward_spd = FEEDFORWARD_SPD_DEFAULTS;

// Instance a modlimit instance
MODLMT modlmt_dq = MODLMT_DEFAULTS;

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGENDQ svgen_dq1 = SVGENDQ_DEFAULTS;

// Instance a QEP interface driver 
QEP qep1 = QEP_DEFAULTS;

// Instance a HALL interface driver 
HALL hall1 = HALL_DEFAULTS;

// Instance a speed calculator based on QEP
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

// Instance a enable PWM drive driver (only for DMC1500) 
DRIVE drv1 = DRIVE_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc2 = RMPCNTL_DEFAULTS;			//缝纫速度曲线

// Create an instance of the current/dc-bus voltage measurement driver
ADCMEAS adc_meas1 = ADCMEAS_DEFAULTS;














//===========================================================================
// No more.
//===========================================================================
