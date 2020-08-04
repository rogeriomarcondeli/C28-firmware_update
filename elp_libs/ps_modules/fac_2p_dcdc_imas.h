/******************************************************************************
 * Copyright (C) 2020 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fac_2p_dcdc_imas.h
 * @brief FAC-2P DC/DC Stage module for IMAS
 * 
 * Module for control of two DC/DC modules of FAC power supplies used by IMAS
 * group on magnets characterization tests. It implements the controller for
 * load current.
 *
 * PWM signals are mapped as the following :
 *
 *      ePWM  =>  Signal   ( POF transmitter)
 *     channel     Name    (    on BCB      )
 *
 *     ePWM1A => Q1_MOD_1        (PWM1)
 *     ePWM1B => Q3_MOD_1        (PWM2)
 *     ePWM2A => Q2_MOD_1        (PWM3)
 *     ePWM2B => Q4_MOD_1        (PWM4)
 *     ePWM7A => Q2_MOD_1        (PWM13)
 *     ePWM7B => Q2_MOD_2        (PWM14)
 *     ePWM8A => Q2_MOD_2        (PWM15)
 *     ePWM8B => Q2_MOD_1        (PWM16)
 *
 * @author gabriel.brunheira
 * @date 19/02/2020
 *
 */

#ifndef FAC_2P_DCDC_IMAS_H_
#define FAC_2P_DCDC_IMAS_H_

extern void main_fac_2p_dcdc_imas(void);

#endif /* FAC_2P_DCDC_IMAS_H_ */
