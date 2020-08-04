/******************************************************************************
 * Copyright (C) 2019 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fap_2p2s.h
 * @brief FAP-2P2S module
 * 
 * Module for control of FAP-2P2S power supplies. It implements the controller
 * for load current, current share between both parallel arms and current share
 * between 8 IGBT's.
 *
 * PWM signals are mapped as the following :
 *
 *      ePWM  =>    Signal     POF transmitter
 *     channel       Name         on BCB
 *
 *     ePWM1A => IGBT_1_MOD_1      PWM1
 *     ePWM2A => IGBT_2_MOD_1      PWM3
 *     ePWM3A => IGBT_1_MOD_2      PWM5
 *     ePWM4A => IGBT_2_MOD_2      PWM7
 *     ePWM5A => IGBT_1_MOD_3      PWM9
 *     ePWM6A => IGBT_2_MOD_3      PWM11
 *     ePWM7A => IGBT_1_MOD_4      PWM13
 *     ePWM8A => IGBT_2_MOD_4      PWM15
 *
 * @author gabriel.brunheira
 * @date 17/04/2019
 *
 */

#include <float.h>

#include "boards/udc_c28.h"
#include "common/structs.h"
#include "common/timeslicer.h"
#include "control/control.h"
#include "event_manager/event_manager.h"
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"

#include "fap_2p2s.h"

/**
 * PWM parameters
 */
#define NUM_PWM_MODULES         8

/**
 * Control parameters
 */
#define TIMESLICER_I_SHARE_CONTROLLER_IDX   2
#define TIMESLICER_I_SHARE_CONTROLLER       g_controller_ctom.timeslicer[TIMESLICER_I_SHARE_CONTROLLER_IDX]
#define I_SHARE_CONTROLLER_FREQ_SAMP        TIMESLICER_FREQ[TIMESLICER_I_SHARE_CONTROLLER_IDX]

/**
 * Analog variables parameters
 */
#define MAX_I_LOAD                              ANALOG_VARS_MAX[0]

#define MAX_I_ARMS_DIFF                         ANALOG_VARS_MAX[1]

#define MAX_DCCTS_DIFF                          ANALOG_VARS_MAX[2]

#define MAX_I_IDLE_DCCT                         ANALOG_VARS_MAX[3]
#define MIN_I_ACTIVE_DCCT                       ANALOG_VARS_MIN[3]

#define MAX_I_IGBT                              ANALOG_VARS_MAX[4]
#define MAX_IGBT_DIFF                           ANALOG_VARS_MAX[5]

#define MAX_V_DCLINK                            ANALOG_VARS_MAX[6]
#define MIN_V_DCLINK                            ANALOG_VARS_MIN[6]

#define TIMEOUT_DCLINK_CONTACTOR_CLOSED_MS      ANALOG_VARS_MAX[7]
#define TIMEOUT_DCLINK_CONTACTOR_OPENED_MS      ANALOG_VARS_MAX[8]

#define NETSIGNAL_ELEM_CTOM_BUF                 ANALOG_VARS_MAX[9]
#define NETSIGNAL_ELEM_MTOC_BUF                 ANALOG_VARS_MIN[9]

#define NETSIGNAL_CTOM_BUF      g_controller_ctom.net_signals[(uint16_t) NETSIGNAL_ELEM_CTOM_BUF].f
#define NETSIGNAL_MTOC_BUF      g_controller_mtoc.net_signals[(uint16_t) NETSIGNAL_ELEM_MTOC_BUF].f

#define NUM_DCCTs                               ANALOG_VARS_MAX[10]

#define MAX_I_ARM                               ANALOG_VARS_MAX[11]
#define I_ARMS_DIFF_MODE                        ANALOG_VARS_MAX[12]

#define RESET_PULSE_TIME_DCLINK_CONTACTOR_MS    ANALOG_VARS_MAX[13]

/**
 * Controller defines
 */

/// DSP Net Signals
#define I_LOAD_1                    g_controller_ctom.net_signals[0].f  // HRADC0
#define I_LOAD_2                    g_controller_ctom.net_signals[1].f  // HRADC1
#define I_ARM_1                     g_controller_ctom.net_signals[2].f  // HRADC2
#define I_ARM_2                     g_controller_ctom.net_signals[3].f  // HRADC3

#define I_LOAD_MEAN                 g_controller_ctom.net_signals[4].f
#define I_LOAD_ERROR                g_controller_ctom.net_signals[5].f
#define DUTY_MEAN                   g_controller_ctom.net_signals[6].f

#define I_LOAD_DIFF                 g_controller_ctom.net_signals[7].f

#define I_ARMS_DIFF                 g_controller_ctom.net_signals[8].f
#define DUTY_ARMS_DIFF              g_controller_ctom.net_signals[9].f

#define I_MOD_1                     g_controller_ctom.net_signals[10].f
#define I_MOD_2                     g_controller_ctom.net_signals[11].f
#define I_MOD_3                     g_controller_ctom.net_signals[12].f
#define I_MOD_4                     g_controller_ctom.net_signals[13].f

#define I_IGBTS_DIFF_MOD_1          g_controller_ctom.net_signals[14].f
#define I_IGBTS_DIFF_MOD_2          g_controller_ctom.net_signals[15].f
#define I_IGBTS_DIFF_MOD_3          g_controller_ctom.net_signals[16].f
#define I_IGBTS_DIFF_MOD_4          g_controller_ctom.net_signals[17].f

#define DUTY_IGBTS_DIFF_MOD_1       g_controller_ctom.net_signals[18].f
#define DUTY_IGBTS_DIFF_MOD_2       g_controller_ctom.net_signals[19].f
#define DUTY_IGBTS_DIFF_MOD_3       g_controller_ctom.net_signals[20].f
#define DUTY_IGBTS_DIFF_MOD_4       g_controller_ctom.net_signals[21].f

/// ARM Net Signals
#define I_IGBT_1_MOD_1              g_controller_mtoc.net_signals[0].f  // ANI0
#define I_IGBT_2_MOD_1              g_controller_mtoc.net_signals[1].f  // ANI1
#define I_IGBT_1_MOD_2              g_controller_mtoc.net_signals[2].f  // ANI2
#define I_IGBT_2_MOD_2              g_controller_mtoc.net_signals[3].f  // ANI3
#define I_IGBT_1_MOD_3              g_controller_mtoc.net_signals[4].f  // ANI4
#define I_IGBT_2_MOD_3              g_controller_mtoc.net_signals[5].f  // ANI5
#define I_IGBT_1_MOD_4              g_controller_mtoc.net_signals[6].f  // ANI6
#define I_IGBT_2_MOD_4              g_controller_mtoc.net_signals[7].f  // ANI7

#define V_DCLINK_MOD_1              g_controller_mtoc.net_signals[8].f  // IIB 1
#define V_DCLINK_MOD_2              g_controller_mtoc.net_signals[9].f  // IIB 2
#define V_DCLINK_MOD_3              g_controller_mtoc.net_signals[10].f // IIB 3
#define V_DCLINK_MOD_4              g_controller_mtoc.net_signals[11].f // IIB 4

#define DUTY_CYCLE_IGBT_1_MOD_1     g_controller_ctom.output_signals[0].f
#define DUTY_CYCLE_IGBT_2_MOD_1     g_controller_ctom.output_signals[1].f
#define DUTY_CYCLE_IGBT_1_MOD_2     g_controller_ctom.output_signals[2].f
#define DUTY_CYCLE_IGBT_2_MOD_2     g_controller_ctom.output_signals[3].f
#define DUTY_CYCLE_IGBT_1_MOD_3     g_controller_ctom.output_signals[4].f
#define DUTY_CYCLE_IGBT_2_MOD_3     g_controller_ctom.output_signals[5].f
#define DUTY_CYCLE_IGBT_1_MOD_4     g_controller_ctom.output_signals[6].f
#define DUTY_CYCLE_IGBT_2_MOD_4     g_controller_ctom.output_signals[7].f

/// Reference
#define I_LOAD_SETPOINT             g_ipc_ctom.ps_module[0].ps_setpoint
#define I_LOAD_REFERENCE            g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_I_LOAD_REFERENCE      &g_controller_ctom.dsp_modules.dsp_srlim[0]

#define WFMREF                      g_ipc_ctom.wfmref[0]

#define SIGGEN                      SIGGEN_CTOM[0]
#define SRLIM_SIGGEN_AMP            &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET         &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define MAX_SLEWRATE_SLOWREF            g_controller_mtoc.dsp_modules.dsp_srlim[0].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_AMP         g_controller_mtoc.dsp_modules.dsp_srlim[1].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_OFFSET      g_controller_mtoc.dsp_modules.dsp_srlim[2].coeffs.s.max_slewrate

/// Load current controller
#define ERROR_I_LOAD                        &g_controller_ctom.dsp_modules.dsp_error[0]
#define PI_CONTROLLER_I_LOAD                &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_I_LOAD_COEFFS         g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s
#define KP_I_LOAD                           PI_CONTROLLER_I_LOAD_COEFFS.kp
#define KI_I_LOAD                           PI_CONTROLLER_I_LOAD_COEFFS.ki

/// Arms current share controller
#define ERROR_I_ARMS_SHARE                  &g_controller_ctom.dsp_modules.dsp_error[1]
#define PI_CONTROLLER_I_ARMS_SHARE          &g_controller_ctom.dsp_modules.dsp_pi[1]
#define PI_CONTROLLER_I_ARMS_SHARE_COEFFS   g_controller_mtoc.dsp_modules.dsp_pi[1].coeffs.s
#define KP_I_ARMS_SHARE                     PI_CONTROLLER_I_ARMS_SHARE_COEFFS.kp
#define KI_I_ARMS_SHARE                     PI_CONTROLLER_I_ARMS_SHARE_COEFFS.ki
#define U_MAX_I_ARMS_SHARE_MODULES          PI_CONTROLLER_I_ARMS_SHARE_COEFFS.u_max
#define U_MIN_I_ARMS_SHARE_MODULES          PI_CONTROLLER_I_ARMS_SHARE_COEFFS.u_min

/// IGBTs current share controllers
#define PI_CONTROLLER_I_SHARE_MOD_1         &g_controller_ctom.dsp_modules.dsp_pi[2]
#define PI_CONTROLLER_I_SHARE_MOD_1_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[2].coeffs.s
#define KP_I_SHARE_MOD_1                    PI_CONTROLLER_I_SHARE_MOD_1_COEFFS.kp
#define KI_I_SHARE_MOD_1                    PI_CONTROLLER_I_SHARE_MOD_1_COEFFS.ki

#define PI_CONTROLLER_I_SHARE_MOD_2         &g_controller_ctom.dsp_modules.dsp_pi[3]
#define PI_CONTROLLER_I_SHARE_MOD_2_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[3].coeffs.s
#define KP_I_SHARE_MOD_2                    PI_CONTROLLER_I_SHARE_MOD_2_COEFFS.kp
#define KI_I_SHARE_MOD_2                    PI_CONTROLLER_I_SHARE_MOD_2_COEFFS.ki

#define PI_CONTROLLER_I_SHARE_MOD_3         &g_controller_ctom.dsp_modules.dsp_pi[4]
#define PI_CONTROLLER_I_SHARE_MOD_3_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[4].coeffs.s
#define KP_I_SHARE_MOD_3                    PI_CONTROLLER_I_SHARE_MOD_3_COEFFS.kp
#define KI_I_SHARE_MOD_3                    PI_CONTROLLER_I_SHARE_MOD_3_COEFFS.ki

#define PI_CONTROLLER_I_SHARE_MOD_4         &g_controller_ctom.dsp_modules.dsp_pi[5]
#define PI_CONTROLLER_I_SHARE_MOD_4_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[5].coeffs.s
#define KP_I_SHARE_MOD_4                    PI_CONTROLLER_I_SHARE_MOD_4_COEFFS.kp
#define KI_I_SHARE_MOD_4                    PI_CONTROLLER_I_SHARE_MOD_4_COEFFS.ki

/// PWM modulators
#define PWM_MODULATOR_IGBT_1_MOD_1          g_pwm_modules.pwm_regs[0]
#define PWM_MODULATOR_IGBT_2_MOD_1          g_pwm_modules.pwm_regs[1]
#define PWM_MODULATOR_IGBT_1_MOD_2          g_pwm_modules.pwm_regs[2]
#define PWM_MODULATOR_IGBT_2_MOD_2          g_pwm_modules.pwm_regs[3]
#define PWM_MODULATOR_IGBT_1_MOD_3          g_pwm_modules.pwm_regs[4]
#define PWM_MODULATOR_IGBT_2_MOD_3          g_pwm_modules.pwm_regs[5]
#define PWM_MODULATOR_IGBT_1_MOD_4          g_pwm_modules.pwm_regs[6]
#define PWM_MODULATOR_IGBT_2_MOD_4          g_pwm_modules.pwm_regs[7]

/// Scope
#define SCOPE                           SCOPE_CTOM[0]

/**
 * Digital I/O's status
 */
#define PIN_OPEN_DCLINK_CONTACTOR_MOD_1     CLEAR_GPDO1;
#define PIN_CLOSE_DCLINK_CONTACTOR_MOD_1    SET_GPDO1;
#define PIN_STATUS_DCLINK_CONTACTOR_MOD_1   GET_GPDI5

#define PIN_OPEN_DCLINK_CONTACTOR_MOD_2     CLEAR_GPDO2;
#define PIN_CLOSE_DCLINK_CONTACTOR_MOD_2    SET_GPDO2;
#define PIN_STATUS_DCLINK_CONTACTOR_MOD_2   GET_GPDI7

#define PIN_OPEN_DCLINK_CONTACTOR_MOD_3     CLEAR_GPDO3;
#define PIN_CLOSE_DCLINK_CONTACTOR_MOD_3    SET_GPDO3;
#define PIN_STATUS_DCLINK_CONTACTOR_MOD_3   GET_GPDI13

#define PIN_OPEN_DCLINK_CONTACTOR_MOD_4     CLEAR_GPDO4;
#define PIN_CLOSE_DCLINK_CONTACTOR_MOD_4    SET_GPDO4;
#define PIN_STATUS_DCLINK_CONTACTOR_MOD_4   GET_GPDI15

#define PIN_STATUS_DCCT_1_STATUS            GET_GPDI9
#define PIN_STATUS_DCCT_1_ACTIVE            GET_GPDI10
#define PIN_STATUS_DCCT_2_STATUS            GET_GPDI11
#define PIN_STATUS_DCCT_2_ACTIVE            GET_GPDI12

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    IGBT_1_Mod_1_Overcurrent,
    IGBT_2_Mod_1_Overcurrent,
    IGBT_1_Mod_2_Overcurrent,
    IGBT_2_Mod_2_Overcurrent,
    IGBT_1_Mod_3_Overcurrent,
    IGBT_2_Mod_3_Overcurrent,
    IGBT_1_Mod_4_Overcurrent,
    IGBT_2_Mod_4_Overcurrent,
    Welded_Contactor_Mod_1_Fault,
    Welded_Contactor_Mod_2_Fault,
    Welded_Contactor_Mod_3_Fault,
    Welded_Contactor_Mod_4_Fault,
    Opened_Contactor_Mod_1_Fault,
    Opened_Contactor_Mod_2_Fault,
    Opened_Contactor_Mod_3_Fault,
    Opened_Contactor_Mod_4_Fault,
    DCLink_Mod_1_Overvoltage,
    DCLink_Mod_2_Overvoltage,
    DCLink_Mod_3_Overvoltage,
    DCLink_Mod_4_Overvoltage,
    DCLink_Mod_1_Undervoltage,
    DCLink_Mod_2_Undervoltage,
    DCLink_Mod_3_Undervoltage,
    DCLink_Mod_4_Undervoltage,
    IIB_Mod_1_Itlk,
    IIB_Mod_2_Itlk,
    IIB_Mod_3_Itlk,
    IIB_Mod_4_Itlk,
    ARM_1_Overcurrent,
    ARM_2_Overcurrent
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
    Arms_High_Difference,
    IGBTs_Current_High_Difference
} soft_interlocks_t;

#define NUM_HARD_INTERLOCKS             ARM_2_Overcurrent + 1
#define NUM_SOFT_INTERLOCKS             IGBTs_Current_High_Difference + 1

/**
 *  Private variables
 */
static uint16_t decimation_factor;
static float decimation_coeff;

/**
 * Private functions
 */
#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");

static void init_peripherals_drivers(void);
static void term_peripherals_drivers(void);

static void init_controller(void);
static void reset_controller(void);
static void enable_controller();
static void disable_controller();
static interrupt void isr_init_controller(void);
static interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t dummy);
static void turn_off(uint16_t dummy);

static void reset_interlocks(uint16_t dummy);
static inline void check_interlocks(void);

/**
 * Main function for this power supply module
 */
void main_fap_2p2s(void)
{
    init_controller();
    init_peripherals_drivers();
    init_interruptions();
    enable_controller();

    /// TODO: check why first sync_pulse occurs
    g_ipc_ctom.counter_sync_pulse = 0;

    /// TODO: include condition for re-initialization
    while(1)
    {
        check_interlocks();
    }

    turn_off(0);

    disable_controller();
    term_interruptions();
    reset_controller();
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{
    uint16_t i;

    /// Initialization of HRADC boards
    stop_DMA();

    decimation_factor = (uint16_t) roundf(HRADC_FREQ_SAMP / ISR_CONTROL_FREQ);
    decimation_coeff = 1.0 / (float) decimation_factor;


    HRADCs_Info.enable_Sampling = 0;
    HRADCs_Info.n_HRADC_boards = NUM_HRADC_BOARDS;

    Init_DMA_McBSP_nBuffers(NUM_HRADC_BOARDS, decimation_factor, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);
    send_ipc_lowpriority_msg(0,Enable_HRADC_Boards);
    DELAY_US(2000000);

    for(i = 0; i < NUM_HRADC_BOARDS; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, decimation_factor,
                        buffers_HRADC[i], TRANSDUCER_GAIN[i]);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], TRANSDUCER_OUTPUT_TYPE[i],
                           HRADC_HEATER_ENABLE[i], HRADC_MONITOR_ENABLE[i]);
    }

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /**
     * Initialization of PWM modules. PWM signals are mapped as the following:
     *
     *      ePWM  =>    Signal     POF transmitter
     *     channel       Name         on BCB
     *
     *     ePWM1A => IGBT_1_MOD_1      PWM1
     *     ePWM2A => IGBT_2_MOD_1      PWM3
     *     ePWM3A => IGBT_1_MOD_2      PWM5
     *     ePWM4A => IGBT_2_MOD_2      PWM7
     *     ePWM5A => IGBT_1_MOD_3      PWM9
     *     ePWM6A => IGBT_2_MOD_3      PWM11
     *     ePWM7A => IGBT_1_MOD_4      PWM13
     *     ePWM8A => IGBT_2_MOD_4      PWM15
     *
     */

    g_pwm_modules.num_modules = NUM_PWM_MODULES;
    SATURATE(g_pwm_modules.num_modules, NUM_MAX_PWM_MODULES, 0);

    PWM_MODULATOR_IGBT_1_MOD_1 = &EPwm1Regs;
    PWM_MODULATOR_IGBT_2_MOD_1 = &EPwm2Regs;
    PWM_MODULATOR_IGBT_1_MOD_2 = &EPwm3Regs;
    PWM_MODULATOR_IGBT_2_MOD_2 = &EPwm4Regs;
    PWM_MODULATOR_IGBT_1_MOD_3 = &EPwm5Regs;
    PWM_MODULATOR_IGBT_2_MOD_3 = &EPwm6Regs;
    PWM_MODULATOR_IGBT_1_MOD_4 = &EPwm7Regs;
    PWM_MODULATOR_IGBT_2_MOD_4 = &EPwm8Regs;

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    init_pwm_module(PWM_MODULATOR_IGBT_1_MOD_1, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_IGBT_2_MOD_1, PWM_FREQ, 0, PWM_Sync_Slave, 180,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_IGBT_1_MOD_2, PWM_FREQ, 0, PWM_Sync_Slave, 90,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_IGBT_2_MOD_2, PWM_FREQ, 0, PWM_Sync_Slave, 270,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_IGBT_1_MOD_3, PWM_FREQ, 0, PWM_Sync_Slave, 45,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_IGBT_2_MOD_3, PWM_FREQ, 0, PWM_Sync_Slave, 225,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_IGBT_1_MOD_4, PWM_FREQ, 0, PWM_Sync_Slave, 135,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_IGBT_2_MOD_4, PWM_FREQ, 0, PWM_Sync_Slave, 315,
                    PWM_ChB_Independent, PWM_DEAD_TIME);

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();

    /// Initialization of timers
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
    CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void term_peripherals_drivers(void)
{
}

static void init_controller(void)
{
    /**
     * TODO: initialize WfmRef and Samples Buffer
     */

    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &turn_on, &turn_off, &isr_soft_interlock,
                   &isr_hard_interlock, &reset_interlocks);

    g_ipc_ctom.ps_module[1].ps_status.all = 0;
    g_ipc_ctom.ps_module[2].ps_status.all = 0;
    g_ipc_ctom.ps_module[3].ps_status.all = 0;

    init_event_manager(0, ISR_CONTROL_FREQ,
                       NUM_HARD_INTERLOCKS, NUM_SOFT_INTERLOCKS,
                       &HARD_INTERLOCKS_DEBOUNCE_TIME,
                       &HARD_INTERLOCKS_RESET_TIME,
                       &SOFT_INTERLOCKS_DEBOUNCE_TIME,
                       &SOFT_INTERLOCKS_RESET_TIME);

    init_control_framework(&g_controller_ctom);

    init_ipc();

    init_wfmref(&WFMREF, WFMREF_SELECTED_PARAM[0], WFMREF_SYNC_MODE_PARAM[0],
                ISR_CONTROL_FREQ, WFMREF_FREQUENCY_PARAM[0], WFMREF_GAIN_PARAM[0],
                WFMREF_OFFSET_PARAM[0], &g_wfmref_data.data, SIZE_WFMREF,
                &I_LOAD_REFERENCE);

    /***********************************************/
    /** INITIALIZATION OF SIGNAL GENERATOR MODULE **/
    /***********************************************/

    disable_siggen(&SIGGEN);

    init_siggen(&SIGGEN, ISR_CONTROL_FREQ, &I_LOAD_REFERENCE);

    cfg_siggen(&SIGGEN, SIGGEN_TYPE_PARAM, SIGGEN_NUM_CYCLES_PARAM,
               SIGGEN_FREQ_PARAM, SIGGEN_AMP_PARAM,
               SIGGEN_OFFSET_PARAM, SIGGEN_AUX_PARAM);

    /**
     *        name:     SRLIM_SIGGEN_AMP
     * description:     Signal generator amplitude slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     SIGGEN_MTOC[0].amplitude
     *         out:     SIGGEN_CTOM[0].amplitude
     */

    init_dsp_srlim(SRLIM_SIGGEN_AMP, MAX_SLEWRATE_SIGGEN_AMP, ISR_CONTROL_FREQ,
                   &SIGGEN_MTOC[0].amplitude, &SIGGEN.amplitude);

    /**
     *        name:     SRLIM_SIGGEN_OFFSET
     * description:     Signal generator offset slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     SIGGEN_MTOC[0].offset
     *         out:     SIGGEN_CTOM[0].offset
     */

    init_dsp_srlim(SRLIM_SIGGEN_OFFSET, MAX_SLEWRATE_SIGGEN_OFFSET,
                   ISR_CONTROL_FREQ, &SIGGEN_MTOC[0].offset,
                   &SIGGEN_CTOM[0].offset);

    /*************************************************/
    /** INITIALIZATION OF LOAD CURRENT CONTROL LOOP **/
    /*************************************************/

    /**
     *        name:     SRLIM_I_LOAD_REFERENCE
     * description:     Load current slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     I_LOAD_SETPOINT
     *         out:     I_LOAD_REFERENCE
     */

    init_dsp_srlim(SRLIM_I_LOAD_REFERENCE, MAX_SLEWRATE_SLOWREF, ISR_CONTROL_FREQ,
                   &I_LOAD_SETPOINT, &I_LOAD_REFERENCE);

    /**
     *        name:     ERROR_I_LOAD
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     I_LOAD_REFERENCE
     *           -:     I_LOAD_MEAN
     *         out:     I_LOAD_ERROR
     */

    init_dsp_error(ERROR_I_LOAD, &I_LOAD_REFERENCE, &I_LOAD_MEAN, &I_LOAD_ERROR);

    /**
     *        name:     PI_CONTROLLER_I_LOAD
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     I_LOAD_ERROR
     *         out:     DUTY_MEAN
     */

    init_dsp_pi(PI_CONTROLLER_I_LOAD, KP_I_LOAD, KI_I_LOAD, ISR_CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &I_LOAD_ERROR, &DUTY_MEAN);

    /****************************************************************/
    /** INITIALIZATION OF PARALLEL ARMS CURRENT SHARE CONTROL LOOP **/
    /****************************************************************/

    /**
     *        name:     ERROR_I_ARMS_SHARE
     * description:     Parallel arms current difference error
     *  dsp module:     DSP_Error
     *           +:     I_ARM_1
     *           -:     I_ARM_2
     *         out:     I_ARMS_DIFF
     */

    init_dsp_error(ERROR_I_ARMS_SHARE, &I_ARM_1, &I_ARM_2, &I_ARMS_DIFF);

    /**
     *        name:     PI_CONTROLLER_I_SHARE_MODULES
     * description:     PI controller for current share between parallel arms
     *  dsp module:     DSP_PI
     *          in:     I_ARMS_DIFF
     *         out:     DUTY_ARMS_DIFF
     */

    init_dsp_pi(PI_CONTROLLER_I_ARMS_SHARE, KP_I_ARMS_SHARE, KI_I_ARMS_SHARE,
                ISR_CONTROL_FREQ, U_MAX_I_ARMS_SHARE_MODULES,
                U_MIN_I_ARMS_SHARE_MODULES, &I_ARMS_DIFF, &DUTY_ARMS_DIFF);

    /*******************************************************/
    /** INITIALIZATION OF IGBT CURRENT SHARE CONTROL LOOP **/
    /*******************************************************/

    /**
     *        name:     PI_CONTROLLER_I_SHARE_MOD_1
     * description:     IGBT current share PI controller for module 1
     *  dsp module:     DSP_PI
     *          in:     I_IGBTS_DIFF_MOD_1
     *         out:     DUTY_DIFF_MOD_1
     */

    init_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_1, KP_I_SHARE_MOD_1, KI_I_SHARE_MOD_1,
                I_SHARE_CONTROLLER_FREQ_SAMP, PWM_LIM_DUTY_SHARE,
                -PWM_LIM_DUTY_SHARE, &I_IGBTS_DIFF_MOD_1, &DUTY_IGBTS_DIFF_MOD_1);

    /**
     *        name:     PI_CONTROLLER_I_SHARE_MOD_2
     * description:     IGBT current share PI controller for module 2
     *  dsp module:     DSP_PI
     *          in:     I_IGBTS_DIFF_MOD_2
     *         out:     DUTY_DIFF_MOD_2
     */

    init_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_2, KP_I_SHARE_MOD_2, KI_I_SHARE_MOD_2,
                I_SHARE_CONTROLLER_FREQ_SAMP, PWM_LIM_DUTY_SHARE,
                -PWM_LIM_DUTY_SHARE, &I_IGBTS_DIFF_MOD_2, &DUTY_IGBTS_DIFF_MOD_2);

    /**
     *        name:     PI_CONTROLLER_I_SHARE_MOD_3
     * description:     IGBT current share PI controller for module 3
     *  dsp module:     DSP_PI
     *          in:     I_IGBTS_DIFF_MOD_3
     *         out:     DUTY_DIFF_MOD_3
     */

    init_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_3, KP_I_SHARE_MOD_3, KI_I_SHARE_MOD_3,
                I_SHARE_CONTROLLER_FREQ_SAMP, PWM_LIM_DUTY_SHARE,
                -PWM_LIM_DUTY_SHARE, &I_IGBTS_DIFF_MOD_3, &DUTY_IGBTS_DIFF_MOD_3);

    /**
     *        name:     PI_CONTROLLER_I_SHARE_MOD_4
     * description:     IGBT current share PI controller for module 4
     *  dsp module:     DSP_PI
     *          in:     I_IGBTS_DIFF_MOD_4
     *         out:     DUTY_DIFF_MOD_4
     */

    init_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_4, KP_I_SHARE_MOD_4, KI_I_SHARE_MOD_4,
                I_SHARE_CONTROLLER_FREQ_SAMP, PWM_LIM_DUTY_SHARE,
                -PWM_LIM_DUTY_SHARE, &I_IGBTS_DIFF_MOD_4, &DUTY_IGBTS_DIFF_MOD_4);

    /************************************/
    /** INITIALIZATION OF TIME SLICERS **/
    /************************************/

    /**
     * Time-slicer for IGBT current share controller
     */
    init_timeslicer(&TIMESLICER_I_SHARE_CONTROLLER, ISR_CONTROL_FREQ);
    cfg_timeslicer(&TIMESLICER_I_SHARE_CONTROLLER, I_SHARE_CONTROLLER_FREQ_SAMP);

    /******************************/
    /** INITIALIZATION OF SCOPES **/
    /******************************/

    init_scope(&SCOPE, ISR_CONTROL_FREQ, SCOPE_FREQ_SAMPLING_PARAM[0],
               &g_buf_samples_ctom[0], SIZE_BUF_SAMPLES_CTOM,
               SCOPE_SOURCE_PARAM[0], &run_scope_shared_ram);
    /**
     * Reset all internal variables
     */
    reset_controller();
}

/**
 * Reset all internal variables from controller
 */
static void reset_controller(void)
{
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_1_MOD_1, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_2_MOD_1, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_1_MOD_2, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_2_MOD_2, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_1_MOD_3, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_2_MOD_3, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_1_MOD_4, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_2_MOD_4, 0.0);

    I_LOAD_SETPOINT = 0.0;
    I_LOAD_REFERENCE = 0.0;

    reset_dsp_srlim(SRLIM_I_LOAD_REFERENCE);

    reset_dsp_error(ERROR_I_LOAD);
    reset_dsp_pi(PI_CONTROLLER_I_LOAD);

    reset_dsp_error(ERROR_I_ARMS_SHARE);
    reset_dsp_pi(PI_CONTROLLER_I_ARMS_SHARE);

    reset_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_1);
    reset_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_2);
    reset_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_3);
    reset_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_4);

    reset_dsp_srlim(SRLIM_SIGGEN_AMP);
    reset_dsp_srlim(SRLIM_SIGGEN_OFFSET);
    disable_siggen(&SIGGEN);

    reset_wfmref(&WFMREF);
}

/**
 * Enable control ISR
 */
static void enable_controller()
{
    stop_DMA();
    DELAY_US(5);
    start_DMA();
    HRADCs_Info.enable_Sampling = 1;
    enable_pwm_tbclk();
}

/**
 * Disable control ISR
 */
static void disable_controller()
{
    disable_pwm_tbclk();
    HRADCs_Info.enable_Sampling = 0;
    stop_DMA();

    reset_controller();
}

/**
 * ISR for control initialization
 */
static interrupt void isr_init_controller(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT = &isr_controller;
    EDIS;

    PWM_MODULATOR_IGBT_1_MOD_1->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_IGBT_1_MOD_1->ETCLR.bit.INT = 1;

    PWM_MODULATOR_IGBT_2_MOD_1->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_IGBT_2_MOD_1->ETCLR.bit.INT = 1;

    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Control ISR
 */
static interrupt void isr_controller(void)
{
    static float temp[4];
    static uint16_t i;

    //CLEAR_DEBUG_GPIO1;
    SET_DEBUG_GPIO0;
    SET_DEBUG_GPIO1;

    temp[0] = 0.0;
    temp[1] = 0.0;
    temp[2] = 0.0;
    temp[3] = 0.0;

    /// Get HRADC samples
    for(i = 0; i < decimation_factor; i++)
    {
        temp[0] += (float) *(HRADCs_Info.HRADC_boards[0].SamplesBuffer++);
        temp[1] += (float) *(HRADCs_Info.HRADC_boards[1].SamplesBuffer++);
        temp[2] += (float) *(HRADCs_Info.HRADC_boards[2].SamplesBuffer++);
        temp[3] += (float) *(HRADCs_Info.HRADC_boards[3].SamplesBuffer++);
    }

    //CLEAR_DEBUG_GPIO1;

    HRADCs_Info.HRADC_boards[0].SamplesBuffer = buffers_HRADC[0];
    HRADCs_Info.HRADC_boards[1].SamplesBuffer = buffers_HRADC[1];
    HRADCs_Info.HRADC_boards[2].SamplesBuffer = buffers_HRADC[2];
    HRADCs_Info.HRADC_boards[3].SamplesBuffer = buffers_HRADC[3];

    temp[0] *= HRADCs_Info.HRADC_boards[0].gain * decimation_coeff;
    temp[0] += HRADCs_Info.HRADC_boards[0].offset;

    temp[1] *= HRADCs_Info.HRADC_boards[1].gain * decimation_coeff;
    temp[1] += HRADCs_Info.HRADC_boards[1].offset;

    temp[2] *= HRADCs_Info.HRADC_boards[2].gain * decimation_coeff;
    temp[2] += HRADCs_Info.HRADC_boards[2].offset;

    temp[3] *= HRADCs_Info.HRADC_boards[3].gain * decimation_coeff;
    temp[3] += HRADCs_Info.HRADC_boards[3].offset;

    if(NUM_DCCTs)
    {
        I_LOAD_1 = temp[0];
        I_LOAD_2 = temp[1];
        I_ARM_1 = temp[2];
        I_ARM_2 = temp[3];

        I_LOAD_MEAN = 0.5*(I_LOAD_1 + I_LOAD_2);
        I_LOAD_DIFF = I_LOAD_1 - I_LOAD_2;
    }
    else
    {
        I_LOAD_1 = temp[0];
        I_ARM_1 = temp[1];
        I_ARM_2 = temp[2];

        I_LOAD_MEAN = I_LOAD_1;
        I_LOAD_DIFF = 0;
    }

    /// Check whether power supply is ON
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
    {
        /// Calculate reference according to operation mode
        switch(g_ipc_ctom.ps_module[0].ps_status.bit.state)
        {
            case SlowRef:
            case SlowRefSync:
            {
                run_dsp_srlim(SRLIM_I_LOAD_REFERENCE, USE_MODULE);
                break;
            }
            case Cycle:
            {
                run_dsp_srlim(SRLIM_SIGGEN_AMP, USE_MODULE);
                run_dsp_srlim(SRLIM_SIGGEN_OFFSET, USE_MODULE);
                SIGGEN.p_run_siggen(&SIGGEN);
                break;
            }
            case RmpWfm:
            case MigWfm:
            {
                run_wfmref(&WFMREF);
                break;
            }
            default:
            {
                break;
            }
        }

        /// Open-loop
        if(g_ipc_ctom.ps_module[0].ps_status.bit.openloop)
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF_OL[0], MIN_REF_OL[0]);
            DUTY_CYCLE_IGBT_1_MOD_1 = 0.01 * I_LOAD_REFERENCE;
            SATURATE(DUTY_CYCLE_IGBT_1_MOD_1, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);

            DUTY_CYCLE_IGBT_2_MOD_1 = DUTY_CYCLE_IGBT_1_MOD_1;
            DUTY_CYCLE_IGBT_1_MOD_2 = DUTY_CYCLE_IGBT_1_MOD_1;
            DUTY_CYCLE_IGBT_2_MOD_2 = DUTY_CYCLE_IGBT_1_MOD_1;
            DUTY_CYCLE_IGBT_1_MOD_3 = DUTY_CYCLE_IGBT_1_MOD_1;
            DUTY_CYCLE_IGBT_2_MOD_3 = DUTY_CYCLE_IGBT_1_MOD_1;
            DUTY_CYCLE_IGBT_1_MOD_4 = DUTY_CYCLE_IGBT_1_MOD_1;
            DUTY_CYCLE_IGBT_2_MOD_4 = DUTY_CYCLE_IGBT_1_MOD_1;
        }
        /// Closed-loop
        else
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF[0], MIN_REF[0]);

            /// Load current controller
            run_dsp_error(ERROR_I_LOAD);
            run_dsp_pi(PI_CONTROLLER_I_LOAD);

            /// Arms current share controller
            if(I_ARMS_DIFF_MODE)
            {
                I_ARMS_DIFF = I_ARM_1 - 0.5*I_LOAD_MEAN;
            }
            else
            {
                run_dsp_error(ERROR_I_ARMS_SHARE);
            }
            run_dsp_pi(PI_CONTROLLER_I_ARMS_SHARE);

            /// IGBTs current share controllers
            /*********************************************/
            RUN_TIMESLICER(TIMESLICER_I_SHARE_CONTROLLER)
            /*********************************************/

                I_MOD_1 = I_IGBT_1_MOD_1 + I_IGBT_2_MOD_1;
                I_MOD_2 = I_IGBT_1_MOD_2 + I_IGBT_2_MOD_2;
                I_MOD_3 = I_IGBT_1_MOD_3 + I_IGBT_2_MOD_3;
                I_MOD_4 = I_IGBT_1_MOD_4 + I_IGBT_2_MOD_4;

                I_IGBTS_DIFF_MOD_1 = I_IGBT_1_MOD_1 - I_IGBT_2_MOD_1;
                I_IGBTS_DIFF_MOD_2 = I_IGBT_1_MOD_2 - I_IGBT_2_MOD_2;
                I_IGBTS_DIFF_MOD_3 = I_IGBT_1_MOD_3 - I_IGBT_2_MOD_3;
                I_IGBTS_DIFF_MOD_4 = I_IGBT_1_MOD_4 - I_IGBT_2_MOD_4;

                run_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_1);
                run_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_2);
                run_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_3);
                run_dsp_pi(PI_CONTROLLER_I_SHARE_MOD_4);

            /*********************************************/
            END_TIMESLICER(TIMESLICER_I_SHARE_CONTROLLER)
            /*********************************************/

            DUTY_CYCLE_IGBT_1_MOD_1 = DUTY_MEAN - DUTY_ARMS_DIFF - DUTY_IGBTS_DIFF_MOD_1;
            DUTY_CYCLE_IGBT_2_MOD_1 = DUTY_MEAN - DUTY_ARMS_DIFF + DUTY_IGBTS_DIFF_MOD_1;
            DUTY_CYCLE_IGBT_1_MOD_2 = DUTY_MEAN - DUTY_ARMS_DIFF - DUTY_IGBTS_DIFF_MOD_2;
            DUTY_CYCLE_IGBT_2_MOD_2 = DUTY_MEAN - DUTY_ARMS_DIFF + DUTY_IGBTS_DIFF_MOD_2;
            DUTY_CYCLE_IGBT_1_MOD_3 = DUTY_MEAN + DUTY_ARMS_DIFF - DUTY_IGBTS_DIFF_MOD_3;
            DUTY_CYCLE_IGBT_2_MOD_3 = DUTY_MEAN + DUTY_ARMS_DIFF + DUTY_IGBTS_DIFF_MOD_3;
            DUTY_CYCLE_IGBT_1_MOD_4 = DUTY_MEAN + DUTY_ARMS_DIFF - DUTY_IGBTS_DIFF_MOD_4;
            DUTY_CYCLE_IGBT_2_MOD_4 = DUTY_MEAN + DUTY_ARMS_DIFF + DUTY_IGBTS_DIFF_MOD_4;

            SATURATE(DUTY_CYCLE_IGBT_1_MOD_1, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_IGBT_2_MOD_1, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_IGBT_1_MOD_2, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_IGBT_2_MOD_2, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_IGBT_1_MOD_3, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_IGBT_2_MOD_3, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_IGBT_1_MOD_4, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_IGBT_2_MOD_4, PWM_MAX_DUTY, PWM_MIN_DUTY);
        }

        set_pwm_duty_chA(PWM_MODULATOR_IGBT_1_MOD_1, DUTY_CYCLE_IGBT_1_MOD_1);
        set_pwm_duty_chA(PWM_MODULATOR_IGBT_2_MOD_1, DUTY_CYCLE_IGBT_2_MOD_1);
        set_pwm_duty_chA(PWM_MODULATOR_IGBT_1_MOD_2, DUTY_CYCLE_IGBT_1_MOD_2);
        set_pwm_duty_chA(PWM_MODULATOR_IGBT_2_MOD_2, DUTY_CYCLE_IGBT_2_MOD_2);
        set_pwm_duty_chA(PWM_MODULATOR_IGBT_1_MOD_3, DUTY_CYCLE_IGBT_1_MOD_3);
        set_pwm_duty_chA(PWM_MODULATOR_IGBT_2_MOD_3, DUTY_CYCLE_IGBT_2_MOD_3);
        set_pwm_duty_chA(PWM_MODULATOR_IGBT_1_MOD_4, DUTY_CYCLE_IGBT_1_MOD_4);
        set_pwm_duty_chA(PWM_MODULATOR_IGBT_2_MOD_4, DUTY_CYCLE_IGBT_2_MOD_4);
    }

    RUN_SCOPE(SCOPE);

    SET_INTERLOCKS_TIMEBASE_FLAG(0);

    PWM_MODULATOR_IGBT_1_MOD_1->ETCLR.bit.INT = 1;
    PWM_MODULATOR_IGBT_2_MOD_1->ETCLR.bit.INT = 1;

    PieCtrlRegs.PIEACK.all |= M_INT3;

    CLEAR_DEBUG_GPIO1;
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT =  &isr_init_controller;
    PieVectTable.EPWM2_INT =  &isr_controller;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    enable_pwm_interrupt(PWM_MODULATOR_IGBT_1_MOD_1);
    enable_pwm_interrupt(PWM_MODULATOR_IGBT_2_MOD_1);

    IER |= M_INT1;
    IER |= M_INT3;
    IER |= M_INT11;

    /// Enable global interrupts (EINT)
    EINT;
    ERTM;
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
    PieCtrlRegs.PIEIER3.bit.INTx1 = 0;  /// ePWM1
    PieCtrlRegs.PIEIER3.bit.INTx2 = 0;  /// ePWM2
    disable_pwm_interrupt(PWM_MODULATOR_IGBT_1_MOD_1);
    disable_pwm_interrupt(PWM_MODULATOR_IGBT_2_MOD_1);

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}

/**
 * Turn power supply on.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_on(uint16_t dummy)
{
#ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    #endif
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Initializing;

        PIN_CLOSE_DCLINK_CONTACTOR_MOD_1;
        DELAY_US(250000);
        PIN_CLOSE_DCLINK_CONTACTOR_MOD_2;
        DELAY_US(250000);
        PIN_CLOSE_DCLINK_CONTACTOR_MOD_3;
        DELAY_US(250000);
        PIN_CLOSE_DCLINK_CONTACTOR_MOD_4;

        DELAY_US(TIMEOUT_DCLINK_CONTACTOR_CLOSED_MS*1000);

        if(V_DCLINK_MOD_1 < MIN_V_DCLINK)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Mod_1_Undervoltage);
            set_hard_interlock(0, DCLink_Mod_1_Undervoltage);
        }

        if(V_DCLINK_MOD_2 < MIN_V_DCLINK)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Mod_2_Undervoltage);
            set_hard_interlock(0, DCLink_Mod_2_Undervoltage);
        }

        if(V_DCLINK_MOD_3 < MIN_V_DCLINK)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Mod_3_Undervoltage);
            set_hard_interlock(0, DCLink_Mod_3_Undervoltage);
        }

        if(V_DCLINK_MOD_4 < MIN_V_DCLINK)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Mod_4_Undervoltage);
            set_hard_interlock(0, DCLink_Mod_4_Undervoltage);
        }

        if(!PIN_STATUS_DCLINK_CONTACTOR_MOD_1)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, Opened_Contactor_Mod_1_Fault);
            set_hard_interlock(0, Opened_Contactor_Mod_1_Fault);
        }

        else if(!PIN_STATUS_DCLINK_CONTACTOR_MOD_2)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, Opened_Contactor_Mod_2_Fault);
            set_hard_interlock(0, Opened_Contactor_Mod_2_Fault);
        }

        else if(!PIN_STATUS_DCLINK_CONTACTOR_MOD_3)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, Opened_Contactor_Mod_3_Fault);
            set_hard_interlock(0, Opened_Contactor_Mod_3_Fault);
        }

        else if(!PIN_STATUS_DCLINK_CONTACTOR_MOD_4)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, Opened_Contactor_Mod_4_Fault);
            set_hard_interlock(0, Opened_Contactor_Mod_4_Fault);
        }

        #ifdef USE_ITLK
        if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Initializing)
        {
        #endif
            reset_controller();

            g_ipc_ctom.ps_module[0].ps_status.bit.openloop = OPEN_LOOP;
            g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;

            enable_pwm_output(0);
            enable_pwm_output(1);
            enable_pwm_output(2);
            enable_pwm_output(3);
            enable_pwm_output(4);
            enable_pwm_output(5);
            enable_pwm_output(6);
            enable_pwm_output(7);

        #ifdef USE_ITLK
        }
        #endif
    }
}

/**
 * Turn off specified power supply.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_off(uint16_t dummy)
{
    disable_pwm_output(0);
    disable_pwm_output(1);
    disable_pwm_output(2);
    disable_pwm_output(3);
    disable_pwm_output(4);
    disable_pwm_output(5);
    disable_pwm_output(6);
    disable_pwm_output(7);

    PIN_OPEN_DCLINK_CONTACTOR_MOD_1;
    PIN_OPEN_DCLINK_CONTACTOR_MOD_2;
    PIN_OPEN_DCLINK_CONTACTOR_MOD_3;
    PIN_OPEN_DCLINK_CONTACTOR_MOD_4;

    DELAY_US(TIMEOUT_DCLINK_CONTACTOR_OPENED_MS*1000);

    reset_controller();

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state != Interlock)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Reset interlocks for specified power supply.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void reset_interlocks(uint16_t dummy)
{
    g_ipc_ctom.ps_module[0].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[0].ps_soft_interlock = 0;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state < Initializing)
    {
        if(PIN_STATUS_DCLINK_CONTACTOR_MOD_1)
        {
            PIN_CLOSE_DCLINK_CONTACTOR_MOD_1;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
            PIN_OPEN_DCLINK_CONTACTOR_MOD_1;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
        }

        if(PIN_STATUS_DCLINK_CONTACTOR_MOD_2)
        {
            PIN_CLOSE_DCLINK_CONTACTOR_MOD_2;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
            PIN_OPEN_DCLINK_CONTACTOR_MOD_2;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
        }

        if(PIN_STATUS_DCLINK_CONTACTOR_MOD_3)
        {
            PIN_CLOSE_DCLINK_CONTACTOR_MOD_3;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
            PIN_OPEN_DCLINK_CONTACTOR_MOD_3;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
        }

        if(PIN_STATUS_DCLINK_CONTACTOR_MOD_4)
        {
            PIN_CLOSE_DCLINK_CONTACTOR_MOD_4;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
            PIN_OPEN_DCLINK_CONTACTOR_MOD_4;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
        }

        DELAY_US(TIMEOUT_DCLINK_CONTACTOR_OPENED_MS*1000);

        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    //SET_DEBUG_GPIO1;

    if(fabs(I_LOAD_MEAN) > MAX_I_LOAD)
    {
        set_hard_interlock(0, Load_Overcurrent);
    }

    if(fabs(I_IGBT_1_MOD_1) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_1_Mod_1_Overcurrent);
    }

    if(fabs(I_IGBT_2_MOD_1) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_2_Mod_1_Overcurrent);
    }

    if(fabs(I_IGBT_1_MOD_2) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_1_Mod_2_Overcurrent);
    }

    if(fabs(I_IGBT_2_MOD_2) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_2_Mod_2_Overcurrent);
    }

    if(fabs(I_IGBT_1_MOD_3) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_1_Mod_3_Overcurrent);
    }

    if(fabs(I_IGBT_2_MOD_3) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_2_Mod_3_Overcurrent);
    }

    if(fabs(I_IGBT_1_MOD_4) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_1_Mod_4_Overcurrent);
    }

    if(fabs(I_IGBT_2_MOD_4) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_2_Mod_4_Overcurrent);
    }

    if(fabs(I_ARM_1) > MAX_I_ARM)
    {
        set_hard_interlock(0, ARM_1_Overcurrent);
    }

    if(fabs(I_ARM_2) > MAX_I_ARM)
    {
        set_hard_interlock(0, ARM_2_Overcurrent);
    }

    if(fabs(I_LOAD_DIFF) > MAX_DCCTS_DIFF)
    {
        set_soft_interlock(0, DCCT_High_Difference);
    }

    if(fabs(I_ARMS_DIFF) > MAX_I_ARMS_DIFF)
    {
        set_soft_interlock(0, Arms_High_Difference);
    }

    if(V_DCLINK_MOD_1 > MAX_V_DCLINK)
    {
        set_hard_interlock(0, DCLink_Mod_1_Overvoltage);
    }

    if(V_DCLINK_MOD_2 > MAX_V_DCLINK)
    {
        set_hard_interlock(0, DCLink_Mod_2_Overvoltage);
    }

    if(V_DCLINK_MOD_3 > MAX_V_DCLINK)
    {
        set_hard_interlock(0, DCLink_Mod_3_Overvoltage);
    }

    if(V_DCLINK_MOD_4 > MAX_V_DCLINK)
    {
        set_hard_interlock(0, DCLink_Mod_4_Overvoltage);
    }

    if(!PIN_STATUS_DCCT_1_STATUS)
    {
        set_soft_interlock(0, DCCT_1_Fault);
    }

    if( NUM_DCCTs && !PIN_STATUS_DCCT_2_STATUS )
    {
        set_soft_interlock(0, DCCT_2_Fault);
    }

    if(PIN_STATUS_DCCT_1_ACTIVE)
    {
        if(fabs(I_LOAD_1) < MIN_I_ACTIVE_DCCT)
        {
            set_soft_interlock(0, Load_Feedback_1_Fault);
        }
    }
    else
    {
        if(fabs(I_LOAD_1) > MAX_I_IDLE_DCCT)
        {
            set_soft_interlock(0, Load_Feedback_1_Fault);
        }
    }

    if(NUM_DCCTs)
    {
        if(PIN_STATUS_DCCT_2_ACTIVE)
        {
            if(fabs(I_LOAD_2) < MIN_I_ACTIVE_DCCT)
            {
                set_soft_interlock(0, Load_Feedback_2_Fault);
            }
        }
        else
        {
            if(fabs(I_LOAD_2) > MAX_I_IDLE_DCCT)
            {
                set_soft_interlock(0, Load_Feedback_2_Fault);
            }
        }
    }

    DINT;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    {
        if(PIN_STATUS_DCLINK_CONTACTOR_MOD_1)
        {
            set_hard_interlock(0, Welded_Contactor_Mod_1_Fault);
        }

        if(PIN_STATUS_DCLINK_CONTACTOR_MOD_2)
        {
            set_hard_interlock(0, Welded_Contactor_Mod_2_Fault);
        }

        if(PIN_STATUS_DCLINK_CONTACTOR_MOD_3)
        {
            set_hard_interlock(0, Welded_Contactor_Mod_3_Fault);
        }

        if(PIN_STATUS_DCLINK_CONTACTOR_MOD_4)
        {
            set_hard_interlock(0, Welded_Contactor_Mod_4_Fault);
        }
    }
    else
    {
        if(!PIN_STATUS_DCLINK_CONTACTOR_MOD_1)
        {
            set_hard_interlock(0, Opened_Contactor_Mod_1_Fault);
        }

        if(!PIN_STATUS_DCLINK_CONTACTOR_MOD_2)
        {
            set_hard_interlock(0, Opened_Contactor_Mod_2_Fault);
        }

        if(!PIN_STATUS_DCLINK_CONTACTOR_MOD_3)
        {
            set_hard_interlock(0, Opened_Contactor_Mod_3_Fault);
        }

        if(!PIN_STATUS_DCLINK_CONTACTOR_MOD_4)
        {
            set_hard_interlock(0, Opened_Contactor_Mod_4_Fault);
        }

        if(V_DCLINK_MOD_1 < MIN_V_DCLINK)
        {
            set_hard_interlock(0, DCLink_Mod_1_Undervoltage);
        }

        if(V_DCLINK_MOD_2 < MIN_V_DCLINK)
        {
            set_hard_interlock(0, DCLink_Mod_2_Undervoltage);
        }

        if(V_DCLINK_MOD_3 < MIN_V_DCLINK)
        {
            set_hard_interlock(0, DCLink_Mod_3_Undervoltage);
        }

        if(V_DCLINK_MOD_4 < MIN_V_DCLINK)
        {
            set_hard_interlock(0, DCLink_Mod_4_Undervoltage);
        }
    }

    EINT;

    //CLEAR_DEBUG_GPIO1;
    //SET_DEBUG_GPIO1;
    run_interlocks_debouncing(0);
    CLEAR_DEBUG_GPIO1;
}
