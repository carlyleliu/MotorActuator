/* Includes ------------------------------------------------------------------*/
#include "pid_regulator_float.h"

#include "mc_type.h"

/**
 * @brief  It initializes the handle
 * @param  phandle: handler of the current instance of the PID component
 * @retval None
 */
__weak void pid_float_init(pid_float_t* phandle)
{
    phandle->kp_gain = phandle->def_kp_gain;
    phandle->ki_gain = phandle->def_ki_gain;
    phandle->kd_gain = phandle->def_kd_gain;
    phandle->integral_term = 0.0f;
    phandle->prev_process_var_error = 0.0f;
}

/**
 * @brief  It set a new value into the PI integral term
 * phandle: handler of the current instance of the PID component
 * @param  integral_term_value: new integral term value
 * @retval None
 */
__weak void pid_float_set_integral_term(pid_float_t* phandle, float integral_term_value)
{
    phandle->integral_term = integral_term_value;

    return;
}

/**
 * @brief  It set a new value for lower integral term limit
 * @param  phandle: handler of the current instance of the PID component
 * @param  lower_limit: new lower integral term limit value
 * @retval None
 */
__weak void pid_float_set_lower_integral_term_limit(pid_float_t* phandle, float lower_limit)
{
    phandle->lower_integral_limit = lower_limit;
}

/**
 * @brief  It set a new value for upper integral term limit
 * @param  phandle: handler of the current instance of the PID component
 * @param  upper_limit: new upper integral term limit value
 * @retval None
 */
__weak void pid_float_set_upper_integral_term_limit(pid_float_t* phandle, float upper_limit)
{
    phandle->upper_integral_limit = upper_limit;
}

/**
 * @brief  It set a new value for lower output limit
 * @param  phandle: handler of the current instance of the PID component
 * @param  lower_limit: new lower output limit value
 * @retval None
 */
__weak void pid_float_set_lower_output_limit(pid_float_t* phandle, float lower_limit)
{
    phandle->lower_output_limit = lower_limit;
}

/**
 * @brief  It set a new value for upper output limit
 * @param  phandle: handler of the current instance of the PID component
 * @param  hupper_limit: new upper output limit value
 * @retval None
 */
__weak void pid_float_set_upper_output_limit(pid_float_t* phandle, float upper_limit)
{
    phandle->upper_output_limit = upper_limit;
}

/**
 * @brief  It set a new value into the PID Previous error variable required to
 *         compute derivative term
 * @param  phandle: handler of the current instance of the PID component
 * @param  prev_process_var_error: New previous error variable
 * @retval None
 */
__weak void pid_float_set_prev_error(pid_float_t* phandle, float prev_process_var_error)
{
    phandle->prev_process_var_error = prev_process_var_error;
}

/**
 * @brief  This function compute the output of a PI regulator sum of its
 *         proportional and integral terms
 * @param  phandle: handler of the current instance of the PID component
 * @param  process_var_error: current process variable error, intended as the reference
 *         value minus the present process variable value
 * @retval computed PI output
 */
float pi_float_controller(pid_float_t* phandle, float process_var_error)
{
    float proportional_term, integral_term, output, integral_sum_temp;
    float discharge = 0;

    /* Proportional term computation*/
    proportional_term = phandle->kp_gain * process_var_error;

    /* Integral term computation */
    if (phandle->ki_gain == 0) {
        phandle->integral_term = 0;
    } else {
        integral_term = phandle->ki_gain * process_var_error;
        integral_sum_temp = phandle->integral_term + integral_term;

        if (integral_sum_temp > phandle->upper_integral_limit) {
            phandle->integral_term = phandle->upper_integral_limit;
        } else if (integral_sum_temp < phandle->lower_integral_limit) {
            phandle->integral_term = phandle->lower_integral_limit;
        } else {
            phandle->integral_term = integral_sum_temp;
        }
    }

    output = proportional_term + phandle->integral_term;

    if (output > phandle->upper_output_limit) {
        discharge = phandle->upper_output_limit - output;
        output = phandle->upper_output_limit;
    } else if (output < phandle->lower_output_limit) {
        discharge = phandle->lower_output_limit - output;
        output = phandle->lower_output_limit;
    } else { /* Nothing to do here */
    }

    phandle->integral_term += discharge;

    return output;
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  This function compute the output of a PID regulator sum of its
 *         proportional, integral and derivative terms
 * @param  phandle: handler of the current instance of the PID component
 * @param  process_var_error: current process variable error, intended as the
 *         reference value minus the present process variable value
 * @retval PID computed output
 */

__weak float pid_float_controller(pid_float_t* phandle, float process_var_error)
{
    float differential_term;
    float delta_error;
    float temp_output;

    if (phandle->kd_gain != 0) /* derivative terms not used */
    {
        delta_error = process_var_error - phandle->prev_process_var_error;
        differential_term = phandle->kd_gain * delta_error;

        phandle->prev_process_var_error = process_var_error;

        temp_output = pi_float_controller(phandle, process_var_error) + differential_term;

        if (temp_output > phandle->upper_output_limit) {
            temp_output = phandle->upper_output_limit;
        } else if (temp_output < phandle->lower_output_limit) {
            temp_output = phandle->lower_output_limit;
        } else {
        }
    } else {
        temp_output = pi_float_controller(phandle, process_var_error);
    }
    return ((float)temp_output);
}
