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
 * @file parameters.c
 * @brief Power supply parameters bank module.
 * 
 * This module implements a data structure for initialization and configuration
 * of parameters for operation of the power supplies applications.
 *
 * @author gabriel.brunheira
 * @date 23/02/2018
 *
 */

#include <string.h>
#include "parameters.h"
#include "ipc/ipc.h"

#pragma DATA_SECTION(g_param_bank,"SHARERAMS0_1");
volatile param_bank_t g_param_bank;

void init_param(param_id_t id, param_type_t type, uint16_t num_elements,
                uint16_t *p_param)
{
    if(num_elements > 0)
    {
        g_param_bank.param_info[id].id = id;
        g_param_bank.param_info[id].type = type;
        g_param_bank.param_info[id].num_elements = num_elements;
        g_param_bank.param_info[id].p_val.u16 = p_param;
    }
}

uint16_t set_param(param_id_t id, uint16_t n, float val)
{
    if(n < g_param_bank.param_info[id].num_elements)
    {
        switch(g_param_bank.param_info[id].type)
        {
            case is_uint16_t:
            {
                *(g_param_bank.param_info[id].p_val.u16 + n) = (uint16_t) val;
                break;
            }

            case is_uint32_t:
            {
                *(g_param_bank.param_info[id].p_val.u32 + n) = (uint32_t) val;
                break;
            }

            case is_float:
            {
                *(g_param_bank.param_info[id].p_val.f + n) = val;
                break;
            }

            default:
            {
                return 0;
            }
        }

        return 1;
    }
    else
    {
        return 0;
    }
}

float get_param(param_id_t id, uint16_t n)
{
    if(n < g_param_bank.param_info[id].num_elements)
    {
        switch(g_param_bank.param_info[id].type)
        {
            case is_uint16_t:
            {
                return (float) *(g_param_bank.param_info[id].p_val.u16 + n);
            }

            case is_uint32_t:
            {
                return (float) *(g_param_bank.param_info[id].p_val.u32 + n);
            }

            case is_float:
            {
                return *(g_param_bank.param_info[id].p_val.f + n);
            }

            default:
            {
                return NAN;
            }
        }
    }
    else
    {
        return NAN;
    }
}
