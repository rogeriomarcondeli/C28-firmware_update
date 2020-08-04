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
 * @file fap.c
 * @brief FAP module
 * 
 * Module for control of FAP power supplies. It implements the controller for
 * load current.
 *
 * @author gabriel.brunheira
 * @date 08/08/2018
 *
 */

#include <float.h>

#include "boards/udc_c28.h"
#include "common/structs.h"
#include "event_manager/event_manager.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"

#include "uninitialized.h"
/*
static interrupt void isr_init_controller(void);
static interrupt void isr_controller(void);

static void turn_on(uint16_t dummy);
static void turn_off(uint16_t dummy);

static void reset_interlocks(uint16_t dummy);
*/
static void dummy_func(void)
{
    return;
}

/**
 * Main function for this power supply module
 */
void main_uninitialized(void)
{
    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &dummy_func, &dummy_func, &dummy_func, &dummy_func,
                   &dummy_func);

    g_ipc_ctom.ps_module[1].ps_status.all = 0;
    g_ipc_ctom.ps_module[2].ps_status.all = 0;
    g_ipc_ctom.ps_module[3].ps_status.all = 0;

    init_ipc();

    /// Enable IPC interrupts
    IER |= M_INT11;

    /// Enable global interrupts (EINT)
    EINT;
    ERTM;

    while(1);

}

