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
 * @file wfmref.c
 * @brief Waveform references module
 * 
 * This module implements waveform references functionality.
 *
 * @author gabriel.brunheira
 * @date 22 de nov de 2017
 *
 */
#include <math.h>
#include "wfmref.h"

#pragma DATA_SECTION(g_wfmref_data,"SHARERAMS2345");
volatile u_wfmref_data_t g_wfmref_data;

#pragma CODE_SECTION(sync_wfmref,"ramfuncs");
#pragma CODE_SECTION(run_wfmref,"ramfuncs");

void init_wfmref(wfmref_t *p_wfmref, uint16_t wfmref_selected,
                 sync_mode_t sync_mode, float freq_lerp, float freq_wfmref,
                 float gain, float offset, float *p_start, uint16_t size,
                 float *p_out)
{
    uint16_t i;

    p_wfmref->wfmref_selected = wfmref_selected;
    p_wfmref->sync_mode = sync_mode;
    p_wfmref->gain = gain;
    p_wfmref->offset = offset;
    p_wfmref->p_out = p_out;

    for(i = 0; i < NUM_WFMREF_CURVES; i++)
    {
        init_buffer(&p_wfmref->wfmref_data[i], p_start + i * size, size);
        p_wfmref->wfmref_data[i].p_buf_idx   = p_wfmref->wfmref_data[i].p_buf_end + 1;
    }

    p_wfmref->lerp.counter = 0;
    p_wfmref->lerp.max_count = (uint16_t) roundf(freq_lerp / freq_wfmref);
    p_wfmref->lerp.freq_lerp = freq_lerp;
    p_wfmref->lerp.freq_base = freq_wfmref;

    /**
     * TODO: Due to the FPUFastRTS library, some accuracy has been lost in the
     * direct form of this calculating. This inverse of the division showed
     * better results.
     */
    ///p_wfmref->lerp.inv_decimation = freq_wfmref / freq_lerp;
    p_wfmref->lerp.inv_decimation = 1.0/(roundf(freq_lerp/freq_wfmref));
    p_wfmref->lerp.out = 0.0;
}

void cfg_wfmref(wfmref_t *p_wfmref, wfmref_t *p_wfmref_new)
{
    uint16_t i;

    p_wfmref->wfmref_selected = p_wfmref_new->wfmref_selected;
    p_wfmref->sync_mode = p_wfmref_new->sync_mode;
    p_wfmref->gain = p_wfmref_new->gain;
    p_wfmref->offset = p_wfmref_new->offset;

    p_wfmref->lerp.counter = 0;
    p_wfmref->lerp.freq_base = p_wfmref_new->lerp.freq_base;
    p_wfmref->lerp.max_count = (uint16_t) roundf(p_wfmref->lerp.freq_lerp /
                                                 p_wfmref_new->lerp.freq_base);

    /**
     * TODO: Due to the FPUFastRTS library, some accuracy has been lost in the
     * direct form of this calculating. This inverse of the division showed
     * better results.
     */
    ///p_wfmref->lerp.inv_decimation = freq_wfmref / freq_lerp;
    p_wfmref->lerp.inv_decimation = 1.0/(roundf(p_wfmref->lerp.freq_lerp /
                                                p_wfmref_new->lerp.freq_base));
}

void reset_wfmref(wfmref_t *p_wfmref)
{
    static uint16_t i;

    for(i = 0; i < NUM_WFMREF_CURVES; i++)
    {
        p_wfmref->wfmref_data[i].p_buf_idx = p_wfmref->wfmref_data[i].p_buf_end + 1;
    }

    p_wfmref->lerp.counter = 0;
    //p_wfmref->lerp.out = *(p_wfmref->wfmref_data[p_wfmref->wfmref_selected].p_buf_end);
}

void update_wfmref(wfmref_t *p_wfmref, wfmref_t *p_wfmref_new)
{
    static uint16_t i;

    for(i = 0; i < NUM_WFMREF_CURVES; i++)
    {
        p_wfmref->wfmref_data[i] = p_wfmref_new->wfmref_data[i];
    }

    p_wfmref->wfmref_selected   = p_wfmref_new->wfmref_selected;
    p_wfmref->gain              = p_wfmref_new->gain;
    p_wfmref->offset            = p_wfmref_new->offset;
    p_wfmref->sync_mode         = p_wfmref_new->sync_mode;
}

void sync_wfmref(wfmref_t *p_wfmref, wfmref_t *p_wfmref_new)
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

void run_wfmref(wfmref_t *p_wfmref)
{
    static uint16_t sel;

    sel = p_wfmref->wfmref_selected;

    switch(p_wfmref->sync_mode)
    {
        case SampleBySample:
        case SampleBySample_OneCycle:
        {
            if(p_wfmref->wfmref_data[sel].p_buf_idx <
               p_wfmref->wfmref_data[sel].p_buf_end)
            {
                if(p_wfmref->lerp.counter < p_wfmref->lerp.max_count)
                {
                    p_wfmref->lerp.fraction = p_wfmref->lerp.inv_decimation *
                                              p_wfmref->lerp.counter++;

                    p_wfmref->lerp.out =
                         INTERPOLATE( *(p_wfmref->wfmref_data[sel].p_buf_idx),
                                      *(p_wfmref->wfmref_data[sel].p_buf_idx+1),
                                        p_wfmref->lerp.fraction);
                }

                else
                {
                    p_wfmref->lerp.out = *(p_wfmref->wfmref_data[sel].p_buf_idx+1);
                }

                *(p_wfmref->p_out) = p_wfmref->lerp.out * p_wfmref->gain + p_wfmref->offset;
            }

            else if( p_wfmref->wfmref_data[sel].p_buf_idx ==
                     p_wfmref->wfmref_data[sel].p_buf_end)
            {
                p_wfmref->lerp.out = *(p_wfmref->wfmref_data[sel].p_buf_idx);
                *(p_wfmref->p_out) = p_wfmref->lerp.out * p_wfmref->gain + p_wfmref->offset;
            }

            break;
        }

        case OneShot:
        {
            if(p_wfmref->wfmref_data[sel].p_buf_idx <
               p_wfmref->wfmref_data[sel].p_buf_end)
            {
                if(p_wfmref->lerp.counter < p_wfmref->lerp.max_count)
                {
                    p_wfmref->lerp.fraction = p_wfmref->lerp.inv_decimation *
                                              p_wfmref->lerp.counter++;

                    p_wfmref->lerp.out =
                         INTERPOLATE( *(p_wfmref->wfmref_data[sel].p_buf_idx),
                                      *(p_wfmref->wfmref_data[sel].p_buf_idx+1),
                                        p_wfmref->lerp.fraction);

                    if(p_wfmref->lerp.counter >= p_wfmref->lerp.max_count)
                    {
                        p_wfmref->lerp.counter = 0;
                        p_wfmref->wfmref_data[sel].p_buf_idx++;
                    }

                    *(p_wfmref->p_out) = p_wfmref->lerp.out * p_wfmref->gain + p_wfmref->offset;
                }
            }

            else if( p_wfmref->wfmref_data[sel].p_buf_idx ==
                     p_wfmref->wfmref_data[sel].p_buf_end)
            {
                p_wfmref->lerp.out = *(p_wfmref->wfmref_data[sel].p_buf_idx);
                *(p_wfmref->p_out) = p_wfmref->lerp.out * p_wfmref->gain + p_wfmref->offset;
            }

            break;
        }

        default:
        {
            p_wfmref->lerp.out = 0.0;
            break;
        }
    }

    //*(p_wfmref->p_out) = p_wfmref->lerp.out * p_wfmref->gain + p_wfmref->offset;
}
