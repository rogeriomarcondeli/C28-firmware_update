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
 * @file wfmref.h
 * @brief Waveform references module
 * 
 * This module implements waveform references functionality.
 *
 * @author gabriel.brunheira
 * @date 22 de nov de 2017
 *
 */

#ifndef WFMREF_H_
#define WFMREF_H_

#include <stdint.h>
#include "common/structs.h"

#define SIZE_WFMREF             4096
#define SIZE_WFMREF_FBP         SIZE_WFMREF/4
#define NUM_WFMREF_CURVES       2

//#define WFMREF                  g_ipc_ctom.wfmref
#define TIMESLICER_WFMREF       0
#define WFMREF_FREQ             TIMESLICER_FREQ[TIMESLICER_WFMREF]

#define INTERPOLATE(a, b, f)    (a * (1.0 - f)) + (b * f)

typedef enum
{
    SampleBySample,
    SampleBySample_OneCycle,
    OneShot
} sync_mode_t;

typedef union
{
    float data[NUM_WFMREF_CURVES][SIZE_WFMREF];
    float data_fbp[4][NUM_WFMREF_CURVES][SIZE_WFMREF_FBP];
} u_wfmref_data_t;

typedef volatile struct
{
    uint16_t        counter;
    uint16_t        max_count;
    float           freq_lerp;
    float           freq_base;
    float           inv_decimation;
    float           fraction;
    float           out;
} wfmref_lerp_t;

typedef volatile struct
{
    buf_t           wfmref_data[NUM_WFMREF_CURVES];
    uint16_t        wfmref_selected;
    sync_mode_t     sync_mode;
    wfmref_lerp_t   lerp;
    float           gain;
    float           offset;
    float           *p_out;
} wfmref_t;
/*
inline void sync_wfmref(wfmref_t *p_wfmref, wfmref_t *p_wfmref_new)
{
    static uint16_t sel;

    sel = p_wfmref->wfmref_selected;

    switch(p_wfmref->sync_mode)
    {
        case SampleBySample:
        {
            if(p_wfmref->wfmref_data[sel].p_buf_idx++ >=
               p_wfmref->wfmref_data[sel].p_buf_end)
            {
                p_wfmref->wfmref_selected = p_wfmref_new->wfmref_selected;
                sel = p_wfmref->wfmref_selected;

                p_wfmref->wfmref_data[sel] = p_wfmref_new->wfmref_data[sel];

                p_wfmref->wfmref_data[sel].p_buf_idx =
                                    p_wfmref->wfmref_data[sel].p_buf_start;

                p_wfmref->gain = p_wfmref_new->gain;
                p_wfmref->offset = p_wfmref_new->offset;
                p_wfmref->sync_mode = p_wfmref_new->sync_mode;
            }

            break;
        }

        case SampleBySample_OneCycle:
        {
            if(p_wfmref->wfmref_data[sel].p_buf_idx++ ==
               p_wfmref->wfmref_data[sel].p_buf_end)
            {
                p_wfmref->wfmref_data[sel].p_buf_idx =
                        p_wfmref->wfmref_data[sel].p_buf_end;
            }
            else if(p_wfmref->wfmref_data[sel].p_buf_idx >
                    p_wfmref->wfmref_data[sel].p_buf_end)
            {
                p_wfmref->wfmref_selected = p_wfmref_new->wfmref_selected;
                sel = p_wfmref->wfmref_selected;

                p_wfmref->wfmref_data[sel] = p_wfmref_new->wfmref_data[sel];

                p_wfmref->wfmref_data[sel].p_buf_idx =
                                    p_wfmref->wfmref_data[sel].p_buf_start;

                p_wfmref->gain = p_wfmref_new->gain;
                p_wfmref->offset = p_wfmref_new->offset;
                p_wfmref->sync_mode = p_wfmref_new->sync_mode;
            }
            else
            {
                //p_wfmref->wfmref_data[sel].p_buf_idx++;
            }

            break;
        }

        case OneShot:
        {
            p_wfmref->wfmref_selected = p_wfmref_new->wfmref_selected;
            sel = p_wfmref->wfmref_selected;

            p_wfmref->wfmref_data[sel] = p_wfmref_new->wfmref_data[sel];

            p_wfmref->wfmref_data[sel].p_buf_idx =
                                    p_wfmref->wfmref_data[sel].p_buf_start;

            p_wfmref->gain = p_wfmref_new->gain;
            p_wfmref->offset = p_wfmref_new->offset;
            p_wfmref->sync_mode = p_wfmref_new->sync_mode;

            break;
        }
    }

    p_wfmref->lerp.counter = 0;
}
*/
extern volatile u_wfmref_data_t g_wfmref_data;
extern volatile wfmref_lerp_t wfmref_lerp;

extern void init_wfmref(wfmref_t *p_wfmref, uint16_t wfmref_selected,
                        sync_mode_t sync_mode, float freq_lerp, float freq_wfmref,
                        float gain, float offset, float *p_start, uint16_t size,
                        float *p_out);
extern void cfg_wfmref(wfmref_t *p_wfmref, wfmref_t *p_wfmref_new);
extern void reset_wfmref(wfmref_t *p_wfmref);
extern void update_wfmref(wfmref_t *p_wfmref, wfmref_t *p_wfmref_new);
extern void sync_wfmref(wfmref_t *p_wfmref, wfmref_t *p_wfmref_new);
extern void run_wfmref(wfmref_t *p_wfmref);

#endif /* WFMREF_H_ */
