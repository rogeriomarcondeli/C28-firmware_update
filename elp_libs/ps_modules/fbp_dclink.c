/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fbp_dclink.C
 * @brief FBP DC-Link controller module
 *
 * Module for control of DC-Link crate from FBP FAC power supplies.
 *
 * @author gabriel.brunheira
 * @date 05/06/2018
 *
 */

#include "boards/udc_c28.h"
#include "control/control.h"
#include "event_manager/event_manager.h"
#include "ipc/ipc.h"
#include "pwm/pwm.h"

#include "fbp_dclink.h"

/**
 * Configuration parameters
 */
#define TIMEOUT_DCLINK_RELAY    1000000

/**
 * Digital I/O's operations and status
 */
#define PIN_OPEN_DCLINK_RELAY           CLEAR_GPDO1
#define PIN_CLOSE_DCLINK_RELAY          SET_GPDO1

#define PIN_OPEN_EXTERNAL_RELAY         CLEAR_GPDO2
#define PIN_CLOSE_EXTERNAL_RELAY        SET_GPDO2

#define PIN_STATUS_POWER_MODULE_1_FAULT GET_GPDI1
#define PIN_STATUS_POWER_MODULE_2_FAULT GET_GPDI2
#define PIN_STATUS_POWER_MODULE_3_FAULT GET_GPDI3

#define PIN_STATUS_SMOKE_DETECTOR       GET_GPDI4
#define PIN_STATUS_EXTERNAL_INTERLOCK   GET_GPDI5

#define PIN_STATUS_ALL_PS_FAIL          g_controller_ctom.net_signals[0].u32

#define V_DCLINK_OUTPUT                 g_controller_mtoc.net_signals[0].f   // ANI0
#define V_PS1_OUTPUT                    g_controller_mtoc.net_signals[1].f   // ANI1
#define V_PS2_OUTPUT                    g_controller_mtoc.net_signals[2].f   // ANI3
#define V_PS3_OUTPUT                    g_controller_mtoc.net_signals[3].f   // ANI2

#define DIGITAL_POT_VOLTAGE             g_controller_mtoc.net_signals[4].u32

/**
 * Analog variables parameters
 */
#define MAX_V_ALL_PS                    ANALOG_VARS_MAX[0]
#define MIN_V_ALL_PS                    ANALOG_VARS_MIN[0]

#define MAX_V_PS1                       ANALOG_VARS_MAX[1]
#define MIN_V_PS1                       ANALOG_VARS_MIN[1]

#define MAX_V_PS2                       ANALOG_VARS_MAX[2]
#define MIN_V_PS2                       ANALOG_VARS_MIN[2]

#define MAX_V_PS3                       ANALOG_VARS_MAX[3]
#define MIN_V_PS3                       ANALOG_VARS_MIN[3]

/**
 * Interlocks defines
 */
typedef enum
{
    Power_Module_1_Fault,
    Power_Module_2_Fault,
    Power_Module_3_Fault,
    Total_Output_Overvoltage,
    Power_Module_1_Overvoltage,
    Power_Module_2_Overvoltage,
    Power_Module_3_Overvoltage,
    Total_Output_Undervoltage,
    Power_Module_1_Undervoltage,
    Power_Module_2_Undervoltage,
    Power_Module_3_Undervoltage,
    Smoke_Detector,
    External_Interlock
} hard_interlocks_t;

#define NUM_HARD_INTERLOCKS     External_Interlock + 1
#define NUM_SOFT_INTERLOCKS     0

#define ISR_FREQ_INTERLOCK_TIMEBASE     10000.0

/**
 * Private functions
 */
static void init_controller(void);

static void init_peripherals_drivers(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t id);
static void turn_off(uint16_t id);

static void reset_interlocks(uint16_t id);
static void check_interlocks_ps_module(uint16_t id);


void main_fbp_dclink(void)
{
    uint16_t i;

    init_controller();
    init_peripherals_drivers();
    init_interruptions();

    /**
     * This delay waits for the stabilization of power supplies after power up,
     * avoid invalid interlock checks
     */
    DELAY_US(1000000);

    /// TODO: check why first sync_pulse occurs
    g_ipc_ctom.counter_sync_pulse = 0;

    /// Enable TBCLK for buzzer and interlock LED PWM signals
    enable_pwm_tbclk();

    /// Enable interlocks time-base timer
    CpuTimer0Regs.TCR.all = 0x4000;

    /// TODO: include condition for re-initialization
    while(1)
    {
        /// Group all pin status
        PIN_STATUS_ALL_PS_FAIL = ( PIN_STATUS_POWER_MODULE_1_FAULT |
                                  (PIN_STATUS_POWER_MODULE_2_FAULT << 1) |
                                  (PIN_STATUS_POWER_MODULE_3_FAULT << 2) ) & 0x00000007;

        /// Check interlocks for specified power module
        for(i = 0; i < NUM_PS_MODULES; i++)
        {
            check_interlocks_ps_module(i);
        }

        /// Saturate and reference
        if(g_ipc_ctom.ps_module[0].ps_status.bit.openloop)
        {
            SATURATE(g_ipc_ctom.ps_module[0].ps_setpoint, MAX_REF_OL[0], MIN_REF_OL[0]);
        }
        else
        {
            /// TODO: After implementation of closed loop, remove first line
            /// below and un-comment second line
            open_loop(&g_ipc_ctom.ps_module[0]);
            ///SATURATE(g_ipc_ctom.ps_module[0].ps_setpoint, MAX_REF[0], MIN_REF[0]);
        }

        g_ipc_ctom.ps_module[0].ps_reference = g_ipc_ctom.ps_module[0].ps_setpoint;
    }

    turn_off(0);
    term_interruptions();
}

static void init_controller(void)
{
    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &turn_on, &turn_off, &isr_soft_interlock,
                   &isr_hard_interlock, &reset_interlocks);

    init_event_manager(0, ISR_FREQ_INTERLOCK_TIMEBASE,
                       NUM_HARD_INTERLOCKS, NUM_SOFT_INTERLOCKS,
                       &HARD_INTERLOCKS_DEBOUNCE_TIME,
                       &HARD_INTERLOCKS_RESET_TIME,
                       &SOFT_INTERLOCKS_DEBOUNCE_TIME,
                       &SOFT_INTERLOCKS_RESET_TIME);

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /// TODO: This line avoids this PS model to open/close loop. After
    /// implementation of control law, remove this line.
    g_ipc_ctom.ps_module[0].ps_status.bit.unlocked = LOCKED;

    g_ipc_ctom.ps_module[0].ps_setpoint = g_ipc_mtoc.ps_module[0].ps_setpoint;

}

static void init_peripherals_drivers(void)
{
    /// Initialization of timers
    InitCpuTimers();

    /// Timer for time-base of interlocks debouncing
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, (1000000.0/ISR_FREQ_INTERLOCK_TIMEBASE) );
    CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.TINT0 = &isr_interlocks_timebase;
    EDIS;

    /// Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    IER |= M_INT1;
    IER |= M_INT11;

    /// Enable global interrupts (EINT)
    EINT;
    ERTM;
}

/**
 * Turn on specified power supply.
 *
 * @param id specified power supply
 */
static void turn_on(uint16_t id)
{
    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    #endif
    {
        PIN_CLOSE_DCLINK_RELAY;
        PIN_CLOSE_EXTERNAL_RELAY;
        DELAY_US(TIMEOUT_DCLINK_RELAY);

        g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;
        g_ipc_ctom.ps_module[0].ps_setpoint = g_ipc_mtoc.ps_module[0].ps_setpoint;
        g_ipc_ctom.ps_module[0].ps_reference = g_ipc_mtoc.ps_module[0].ps_setpoint;
    }
}

/**
 * Turn off specified power supply.
 *
 * @param id specified power supply
 */
static void turn_off(uint16_t id)
{
    PIN_OPEN_DCLINK_RELAY;
    PIN_OPEN_EXTERNAL_RELAY;
    DELAY_US(TIMEOUT_DCLINK_RELAY);

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state != Interlock)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Termination of interruptions.
 */
static void term_interruptions(void)
{
    /// Disable global interrupts (EINT)
    DINT;
    DRTM;

    /// Clear enables
    IER = 0;

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT11;
}

/**
 * Reset interlocks for specified power supply.
 *
 * @param id specified power supply
 */
static void reset_interlocks(uint16_t id)
{
    g_ipc_ctom.ps_module[0].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[0].ps_soft_interlock = 0;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
static void check_interlocks_ps_module(uint16_t id)
{
    if(PIN_STATUS_SMOKE_DETECTOR)
    {
        set_hard_interlock(0, Smoke_Detector);
    }

    if(PIN_STATUS_EXTERNAL_INTERLOCK)
    {
        set_hard_interlock(0, External_Interlock);
    }

    /// Check overvoltage conditions
    if(V_DCLINK_OUTPUT > MAX_V_ALL_PS)
    {
        set_hard_interlock(0, Total_Output_Overvoltage);
    }

    DINT;

    switch(id)
    {
        case 0:
        {
            if(V_PS1_OUTPUT > MAX_V_PS1)
            {
                set_hard_interlock(0, Power_Module_1_Overvoltage);
            }

            if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                if(PIN_STATUS_POWER_MODULE_1_FAULT)
                {
                    set_hard_interlock(0, Power_Module_1_Fault);
                }

                if(V_DCLINK_OUTPUT < MIN_V_ALL_PS)
                {
                    set_hard_interlock(0, Total_Output_Undervoltage);
                }

                if(V_PS1_OUTPUT < MIN_V_PS1)
                {
                    set_hard_interlock(0, Power_Module_1_Undervoltage);
                }
            }

            else
            {
                if(!PIN_STATUS_POWER_MODULE_1_FAULT)
                {
                    set_hard_interlock(0, Power_Module_1_Fault);
                }
            }

            break;
        }

        case 1:
        {
            if(V_PS2_OUTPUT > MAX_V_PS2)
            {
                set_hard_interlock(0, Power_Module_2_Overvoltage);
            }

            if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                if(PIN_STATUS_POWER_MODULE_2_FAULT)
                {
                    set_hard_interlock(0, Power_Module_2_Fault);
                }

                if(V_DCLINK_OUTPUT < MIN_V_ALL_PS)
                {
                    set_hard_interlock(0, Total_Output_Undervoltage);
                }

                if(V_PS2_OUTPUT < MIN_V_PS2)
                {
                    set_hard_interlock(0, Power_Module_2_Undervoltage);
                }
            }

            else
            {
                if(!PIN_STATUS_POWER_MODULE_2_FAULT)
                {
                    set_hard_interlock(0, Power_Module_2_Fault);
                }
            }

            break;
        }

        case 2:
        {
            if(V_PS3_OUTPUT > MAX_V_PS3)
            {
                set_hard_interlock(0, Power_Module_3_Overvoltage);
            }

            if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                if(PIN_STATUS_POWER_MODULE_3_FAULT)
                {
                    set_hard_interlock(0, Power_Module_3_Fault);
                }

                if(V_DCLINK_OUTPUT < MIN_V_ALL_PS)
                {
                    set_hard_interlock(0, Total_Output_Undervoltage);
                }

                if(V_PS3_OUTPUT < MIN_V_PS3)
                {
                    set_hard_interlock(0, Power_Module_3_Undervoltage);
                }
            }

            else
            {
                if(!PIN_STATUS_POWER_MODULE_3_FAULT)
                {
                    set_hard_interlock(0, Power_Module_3_Fault);
                }
            }

            break;
        }
    }

    EINT;

    run_interlocks_debouncing(0);
}
