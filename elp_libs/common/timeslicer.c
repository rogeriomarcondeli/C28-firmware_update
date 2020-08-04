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
 * @file timeslicer.c
 * @brief Time Slicer Module
 * 
 * This module implements functions for time slicer functionality. It allows
 * the decimation of a periodic task within another periodic task.
 *
 * @author gabriel.brunheira
 * @date 24/11/2017
 *
 */

#include <math.h>
#include "common/timeslicer.h"

void init_timeslicer(timeslicer_t *p_ts, float freq_base)
{
    p_ts->freq_base = freq_base;
    p_ts->freq_sampling = freq_base;
    p_ts->freq_ratio = 1;
    p_ts->counter = 1;
}

void cfg_timeslicer(timeslicer_t *p_ts, float freq_sampling)
{
    p_ts->freq_ratio = (uint16_t) roundf(p_ts->freq_base / freq_sampling);

    /**
     *  TODO: Investigate one-bit accuracy-error in some cases, like:
     *          freq_base = 48000.0
     *          freq_ratio = 480
     *          freq_sampling = 100.000008
     */
    p_ts->freq_sampling = p_ts->freq_base / ((float) p_ts->freq_ratio);

    p_ts->counter = p_ts->freq_ratio;
}

void reset_timeslicer(timeslicer_t *p_ts)
{
    p_ts->counter = p_ts->freq_ratio;
}
