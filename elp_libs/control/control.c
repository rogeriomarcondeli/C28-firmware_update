/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file control.c
 * @brief Brief description of module
 * 
 * Detailed description
 *
 * @author gabriel.brunheira
 * @date 27/11/2017
 *
 * TODO: insert comments
 */

#include <math.h>
#include "control.h"

#pragma DATA_SECTION(g_controller_mtoc,"SHARERAMS0_0");
#pragma DATA_SECTION(g_controller_ctom,"SHARERAMS1_0");

volatile control_framework_t g_controller_ctom;
volatile control_framework_t g_controller_mtoc;

void init_control_framework(volatile control_framework_t *p_controller)
{
    uint16_t i;

    for(i = 0; i < NUM_MAX_NET_SIGNALS; i++)
    {
        p_controller->net_signals[i].f = 0.0;
    }

    for(i = 0; i < NUM_MAX_OUTPUT_SIGNALS; i++)
    {
        p_controller->output_signals[i].f = 0.0;
    }
}

void set_dsp_coeffs(dsp_class_t dsp_class, uint16_t id)
{
    switch(dsp_class)
    {
        case DSP_SRLim:
        {
            cfg_dsp_srlim( &(g_controller_ctom.dsp_modules.dsp_srlim[id]),
                           g_controller_mtoc.dsp_modules.dsp_srlim[id].coeffs.f[0] );
            break;
        }

        case DSP_LPF:
        {
            cfg_dsp_lpf( &(g_controller_ctom.dsp_modules.dsp_lpf[id]),
                         g_controller_mtoc.dsp_modules.dsp_lpf[id].coeffs.f[0] );
            break;
        }

        case DSP_PI:
        {
            cfg_dsp_pi( &(g_controller_ctom.dsp_modules.dsp_pi[id]),
                        g_controller_mtoc.dsp_modules.dsp_pi[id].coeffs.f[0],
                        g_controller_mtoc.dsp_modules.dsp_pi[id].coeffs.f[1],
                        g_controller_mtoc.dsp_modules.dsp_pi[id].coeffs.f[2],
                        g_controller_mtoc.dsp_modules.dsp_pi[id].coeffs.f[3]);
            break;
       }
        case DSP_IIR_2P2Z:
        {
            cfg_dsp_iir_2p2z( &(g_controller_ctom.dsp_modules.dsp_iir_2p2z[id]),
                              g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f[0],
                              g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f[1],
                              g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f[2],
                              g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f[3],
                              g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f[4],
                              g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f[5],
                              g_controller_mtoc.dsp_modules.dsp_iir_2p2z[id].coeffs.f[6] );
            break;
        }

        case DSP_IIR_3P3Z:
        {
            cfg_dsp_iir_3p3z( &(g_controller_ctom.dsp_modules.dsp_iir_3p3z[id]),
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[0],
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[1],
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[2],
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[3],
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[4],
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[5],
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[6],
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[7],
                              g_controller_mtoc.dsp_modules.dsp_iir_3p3z[id].coeffs.f[8] );
            break;
        }

        case DSP_VdcLink_FeedForward:
        {
            cfg_dsp_vdclink_ff( &(g_controller_ctom.dsp_modules.dsp_ff[id]),
                                g_controller_mtoc.dsp_modules.dsp_ff[id].coeffs.f[0],
                                g_controller_mtoc.dsp_modules.dsp_ff[id].coeffs.f[1]);
            break;
        }

        default:
        {
            break;
        }
    }
}
