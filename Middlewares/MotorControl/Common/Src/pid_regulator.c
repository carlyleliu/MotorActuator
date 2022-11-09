/**
 ******************************************************************************
 * @file    pid_regulator.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the PID regulator component of the motor Control SDK:
 *
 *           * proportional, integral and derivative computation funcions
 *           * read and write gain functions
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "pid_regulator.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/**
 * @defgroup PIDRegulator PID Regulator
 * @brief PID regulator component of the motor Control SDK
 *
 * The PID regulator component implements the following control function:
 *
 * @f[
 * u(t) = K_{p} e(t) + K_{i} \int_0^t e(\tau) \,d\tau + K_{d} \frac{de(t)}{dt}
 * @f]
 *
 * with the proportional, integral and derivative gains expressed as rational numbers, with a gain and a divisor parameter :
 *
 * @f[
 * K_{p} = \frac{K_{pg}}{K_{pd}}
 * @f]
 * @f[
 * K_{i} = \frac{K_{ig}}{K_{id}}
 * @f]
 * @f[
 * K_{d} = \frac{K_{dg}}{K_{dd}}
 * @f]
 *
 *  Each of the gain and divisor parameters, @f$K_{{p}g}@f$, @f$K_{{i}g}@f$, @f$K_{{d}g}@f$, @f$K_{{p}d}@f$,
 * @f$K_{id}@f$, @f$K_{dd}@f$, can be set independently. via the pid_set_kp(), pid_set_kp_divisor_pow2(), pid_set_ki(),
 * pid_set_ki_divisor_pow2(), pid_set_kd()
 *
 * @{
 */

/**
 * @brief  It initializes the handle
 * @param  phandle: handler of the current instance of the PID component
 * @retval None
 */
__weak void pid_init(pid_integer_t* phandle)
{
    phandle->kp_gain = phandle->def_kp_gain;
    phandle->ki_gain = phandle->def_ki_gain;
    phandle->kd_gain = phandle->def_kd_gain;
    phandle->integral_term = 0x00000000UL;
    phandle->prev_process_var_error = 0x00000000UL;
}

/**
 * @brief  It updates the Kp gain
 * @param  phandle: handler of the current instance of the PID component
 * @param  kp_gain: new Kp gain
 * @retval None
 */
__weak void pid_set_kp(pid_integer_t* phandle, int16_t kp_gain)
{
    phandle->kp_gain = kp_gain;
}

/**
 * @brief  It updates the Ki gain
 * @param  phandle: handler of the current instance of the PID component
 * @param  ki_gain: new Ki gain
 * @retval None
 */
__weak void pid_set_ki(pid_integer_t* phandle, int16_t ki_gain)
{
    phandle->ki_gain = ki_gain;
}

/**
 * @brief  It returns the Kp gain
 * @param  phandle: handler of the current instance of the PID component
 * @retval Kp gain
 */
__weak int16_t pid_get_kp(pid_integer_t* phandle)
{
    return (phandle->kp_gain);
}

/**
 * @brief  It returns the Ki gain
 * @param  phandle: handler of the current instance of the PID component
 * @retval Ki gain
 */
__weak int16_t pid_get_ki(pid_integer_t* phandle)
{
    return (phandle->ki_gain);
}

/**
 * @brief  It returns the Default Kp gain
 * @param  phandle: handler of the current instance of the PID component
 * @retval default Kp gain
 */
__weak int16_t pid_get_default_kp(pid_integer_t* phandle)
{
    return (phandle->def_kp_gain);
}

/**
 * @brief  It returns the Default Ki gain of the passed PI object
 * @param  phandle: handler of the current instance of the PID component
 * @retval default Ki gain
 */
__weak int16_t pid_get_default_ki(pid_integer_t* phandle)
{
    return (phandle->def_ki_gain);
}

/**
 * @brief  It set a new value into the PI integral term
 * phandle: handler of the current instance of the PID component
 * @param  integral_term_value: new integral term value
 * @retval None
 */
__weak void pid_set_integral_term(pid_integer_t* phandle, int32_t integral_term_value)
{
    phandle->integral_term = integral_term_value;

    return;
}

/**
 * @brief  It returns the Kp gain divisor
 * @param  phandle: handler of the current instance of the PID component
 * @retval Kp gain divisor
 */
__weak uint16_t pid_get_kp_divisor(pid_integer_t* phandle)
{
    return (phandle->kp_divisor);
}

/**
 * @brief  It updates the Kp divisor
 * @param  phandle: handler of the current instance of the PID component
 * @param  kp_divisor_pow2: new Kp divisor expressed as power of 2
 * @retval None
 */
__weak void pid_set_kp_divisor_pow2(pid_integer_t* phandle, uint16_t kp_divisor_pow2)
{
    phandle->kp_divisor_pow2 = kp_divisor_pow2;
    phandle->kp_divisor = ((uint16_t)(1u) << kp_divisor_pow2);
}

/**
 * @brief  It returns the Ki gain divisor of the passed PI object
 * @param  phandle: handler of the current instance of the PID component
 * @retval Ki gain divisor
 */
__weak uint16_t pid_get_ki_divisor(pid_integer_t* phandle)
{
    return (phandle->ki_divisor);
}

/**
 * @brief  It updates the Ki divisor
 * @param  phandle: handler of the current instance of the PID component
 * @param  ki_divisor_pow2: new Ki divisor expressed as power of 2
 * @retval None
 */
__weak void pid_set_ki_divisor_pow2(pid_integer_t* phandle, uint16_t ki_divisor_pow2)
{
    int32_t ki_div = ((int32_t)(1u) << ki_divisor_pow2);
    phandle->ki_divisor_pow2 = ki_divisor_pow2;
    phandle->ki_divisor = (uint16_t)(ki_div);
    pid_set_upper_integral_term_limit(phandle, (int32_t)INT16_MAX * ki_div);
    pid_set_lower_integral_term_limit(phandle, (int32_t)-INT16_MAX * ki_div);
}

/**
 * @brief  It set a new value for lower integral term limit
 * @param  phandle: handler of the current instance of the PID component
 * @param  wlower_limit: new lower integral term limit value
 * @retval None
 */
__weak void pid_set_lower_integral_term_limit(pid_integer_t* phandle, int32_t wlower_limit)
{
    phandle->lower_integral_limit = wlower_limit;
}

/**
 * @brief  It set a new value for upper integral term limit
 * @param  phandle: handler of the current instance of the PID component
 * @param  upper_limit: new upper integral term limit value
 * @retval None
 */
__weak void pid_set_upper_integral_term_limit(pid_integer_t* phandle, int32_t upper_limit)
{
    phandle->upper_integral_limit = upper_limit;
}

/**
 * @brief  It set a new value for lower output limit
 * @param  phandle: handler of the current instance of the PID component
 * @param  hlower_limit: new lower output limit value
 * @retval None
 */
__weak void pid_set_lower_output_limit(pid_integer_t* phandle, int16_t hlower_limit)
{
    phandle->lower_output_limit = hlower_limit;
}

/**
 * @brief  It set a new value for upper output limit
 * @param  phandle: handler of the current instance of the PID component
 * @param  hupper_limit: new upper output limit value
 * @retval None
 */
__weak void pid_set_upper_output_limit(pid_integer_t* phandle, int16_t hupper_limit)
{
    phandle->upper_output_limit = hupper_limit;
}

/**
 * @brief  It set a new value into the PID Previous error variable required to
 *         compute derivative term
 * @param  phandle: handler of the current instance of the PID component
 * @param  prev_process_var_error: New previous error variable
 * @retval None
 */
__weak void pid_set_prev_error(pid_integer_t* phandle, int32_t prev_process_var_error)
{
    phandle->prev_process_var_error = prev_process_var_error;
    return;
}

/**
 * @brief  It updates the Kd gain
 * @param  phandle: handler of the current instance of the PID component
 * @param  kd_gain: new Kd gain
 * @retval None
 */
__weak void pid_set_kd(pid_integer_t* phandle, int16_t kd_gain)
{
    phandle->kd_gain = kd_gain;
}

/**
 * @brief  It returns the Kd gain
 * @param  phandle: handler of the current instance of the PID component
 * @retval Kd gain
 */
__weak int16_t pid_get_kd(pid_integer_t* phandle)
{
    return phandle->kd_gain;
}

/**
 * @brief  It returns the Kd gain divisor of the PID object passed
 * @param  phandle: handler of the current instance of the PID component
 * @retval Kd gain divisor
 */
__weak uint16_t pid_get_kd_divisor(pid_integer_t* phandle)
{
    return (phandle->kd_divisor);
}

/**
 * @brief Sets @f$K_{dd}@f$, the derivative divisor parameter of the PID component
 *
 * @param phandle handle on the instance of the PID component to update
 * @param kd_divisor_pow2
 */
__weak void pid_set_kd_divisor_pow2(pid_integer_t* phandle, uint16_t kd_divisor_pow2)
{
    phandle->kd_divisor_pow2 = kd_divisor_pow2;
    phandle->kd_divisor = ((uint16_t)(1u) << kd_divisor_pow2);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  This function compute the output of a PI regulator sum of its
 *         proportional and integral terms
 * @param  phandle: handler of the current instance of the PID component
 * @param  wprocess_var_error: current process variable error, intended as the reference
 *         value minus the present process variable value
 * @retval computed PI output
 */
__weak int16_t pi_controller(pid_integer_t* phandle, int32_t wprocess_var_error)
{
    int32_t proportional_term, integral_term, output_32, integral_sum_temp;
    int32_t discharge = 0;
    int16_t upper_output_limit = phandle->upper_output_limit;
    int16_t lower_output_limit = phandle->lower_output_limit;

    /* Proportional term computation*/
    proportional_term = phandle->kp_gain * wprocess_var_error;

    /* Integral term computation */
    if (phandle->ki_gain == 0) {
        phandle->integral_term = 0;
    } else {
        integral_term = phandle->ki_gain * wprocess_var_error;
        integral_sum_temp = phandle->integral_term + integral_term;

        if (integral_sum_temp < 0) {
            if (phandle->integral_term > 0) {
                if (integral_term > 0) {
                    integral_sum_temp = INT32_MAX;
                }
            }
        } else {
            if (phandle->integral_term < 0) {
                if (integral_term < 0) {
                    integral_sum_temp = -INT32_MAX;
                }
            }
        }

        if (integral_sum_temp > phandle->upper_integral_limit) {
            phandle->integral_term = phandle->upper_integral_limit;
        } else if (integral_sum_temp < phandle->lower_integral_limit) {
            phandle->integral_term = phandle->lower_integral_limit;
        } else {
            phandle->integral_term = integral_sum_temp;
        }
    }

#ifdef FULL_MISRA_C_COMPLIANCY
    output_32 = (proportional_term / (int32_t)phandle->kp_divisor) + (phandle->integral_term / (int32_t)phandle->ki_divisor);
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/
    output_32 = (proportional_term >> phandle->kp_divisor_pow2) + (phandle->integral_term >> phandle->ki_divisor_pow2);
#endif

    if (output_32 > upper_output_limit) {
        discharge = upper_output_limit - output_32;
        output_32 = upper_output_limit;
    } else if (output_32 < lower_output_limit) {
        discharge = lower_output_limit - output_32;
        output_32 = lower_output_limit;
    } else { /* Nothing to do here */
    }

    phandle->integral_term += discharge;

    return ((int16_t)(output_32));
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
 * @param  wprocess_var_error: current process variable error, intended as the
 *         reference value minus the present process variable value
 * @retval PID computed output
 */

__weak int16_t pid_controller(pid_integer_t* phandle, int32_t wprocess_var_error)
{
    int32_t differential_term;
    int32_t delta_error;
    int32_t temp_output;

    if (phandle->kd_gain != 0) /* derivative terms not used */
    {
        delta_error = wprocess_var_error - phandle->prev_process_var_error;
        differential_term = phandle->kd_gain * delta_error;

#ifdef FULL_MISRA_C_COMPLIANCY
        differential_term /= (int32_t)phandle->kd_divisor;
#else
        /* WARNING: the below instruction is not MISRA compliant, user should verify
         that Cortex-M3 assembly instruction ASR (arithmetic shift right)
         is used by the compiler to perform the shifts (instead of LSR
         logical shift right)*/
        differential_term >>= phandle->kd_divisor_pow2;
#endif

        phandle->prev_process_var_error = wprocess_var_error;

        temp_output = pi_controller(phandle, wprocess_var_error) + differential_term;

        if (temp_output > phandle->upper_output_limit) {
            temp_output = phandle->upper_output_limit;
        } else if (temp_output < phandle->lower_output_limit) {
            temp_output = phandle->lower_output_limit;
        } else {
        }
    } else {
        temp_output = pi_controller(phandle, wprocess_var_error);
    }
    return ((int16_t)temp_output);
}
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
