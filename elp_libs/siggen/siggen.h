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
 * @file siggen.h
 * @brief Signal generator module
 *
 * This module implements a real-time parametric digital signal generator. It
 * supports some broadly used signals, like sinusoidals, trapezoids, squares,
 * triangular, etc.
 *
 * TODO: Compare implementation using uint32_t representation on counter 'n'.
 *       This is suggested in order to avoid inexact representations of large
 *       integer due to float limitation.
 *       (Ref.: https://en.wikipedia.org/wiki/Single-precision_floating-point_format#Precision_limits_on_integer_values)
 *
 *
 * @author gabriel.brunheira
 * @date 11/02/2018
 *
 */


#ifndef SIGGEN_H_
#define SIGGEN_H_

#include <stdint.h>

#define NUM_SIGGEN_AUX_PARAM    4
#define NUM_SIGGEN_AUX_VAR      8

/**
 * TODO: Implement square, triangular and prbs
 */
typedef enum
{
    Sine,
    DampedSine,
    Trapezoidal,
    DampedSquaredSine,
    Square
} siggen_type_t;

typedef volatile struct siggen_t siggen_t;

struct siggen_t
{
    uint16_t 		enable;
	siggen_type_t	type;
	uint16_t		num_cycles;
	float           freq;
    float           amplitude;
    float           offset;
    float           n;
    float           num_samples;
	float			aux_param[NUM_SIGGEN_AUX_PARAM];
	float           aux_var[NUM_SIGGEN_AUX_VAR];
	float           freq_sampling;
	volatile float 	*p_out;
	void			(*p_run_siggen)(siggen_t *p_siggen);
};

extern void init_siggen(siggen_t *p_siggen, float freq_sampling,
                        volatile float *p_out);

extern void cfg_siggen(siggen_t *p_siggen, siggen_type_t sig_type,
                       uint16_t num_cycles, float freq, float amplitude,
                       float offset, float *p_aux_param);
extern void scale_siggen(siggen_t *p_siggen, float amplitude, float offset);
extern void set_siggen_freq(siggen_t *p_siggen, float freq);

extern void enable_siggen(siggen_t *p_siggen);
extern void disable_siggen(siggen_t *p_siggen);

extern void run_siggen_sine(siggen_t *p_siggen);
extern void run_siggen_dampedsine(siggen_t *p_siggen);
extern void run_siggen_trapezoidal(siggen_t *p_siggen);
extern void run_siggen_dampedsquaredsine(siggen_t *p_siggen);
extern void run_siggen_square(siggen_t *p_siggen);

#endif
