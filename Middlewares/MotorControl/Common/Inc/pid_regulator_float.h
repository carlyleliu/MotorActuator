/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PIDREGULATOR_FLOAT_H
#define __PIDREGULATOR_FLOAT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/**
 * @brief Handle of a PID component
 *
 * @detail This structure stores all the parameters needed to perform a proportional,
 * integral and derivative regulation computation. It also stores configurable limits
 * in order to saturate the integral terms and the output value. This structure is
 * passed to each PID component function.
 */
typedef struct {
    float def_kp_gain;          /**< Default p gain */
    float def_ki_gain;          /**< Default i gain */
    float kp_gain;              /**< gain used by PID component */
    float ki_gain;              /**< gain used by PID component */
    float integral_term;        /**< integral term */
    float upper_integral_limit; /**< Upper limit used to saturate the integral term*/
    float lower_integral_limit; /**< Lower limit used to saturate the integral term*/
    float upper_output_limit;   /**< Upper limit used to saturate the PID output */
    float lower_output_limit;   /**< Lower limit used to saturate the PID output */

    float def_kd_gain; /**< Default Kd gain */
    float kd_gain;     /**< Kd gain used by PID component */

    float prev_process_var_error; /**< previous process variable used by the derivative part of the PID component */
} pid_float_t;

/* This function compute the output of a PI regulator sum of its
 * proportional and integral_terms with float
 */
void pid_float_init(pid_float_t* phandle);
void pid_float_set_integral_term(pid_float_t* phandle, float integral_term_value);
void pid_float_set_lower_integral_term_limit(pid_float_t* phandle, float lower_limit);
void pid_float_set_upper_integral_term_limit(pid_float_t* phandle, float upper_limit);
void pid_float_set_lower_output_limit(pid_float_t* phandle, float lower_limit);
void pid_float_set_upper_output_limit(pid_float_t* phandle, float upper_limit);
void pid_float_set_prev_error(pid_float_t* phandle, float prev_process_var_error);

float pi_float_controller(pid_float_t* phandle, float process_var_error);
float pid_float_controller(pid_float_t* phandle, float process_var_error);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif
