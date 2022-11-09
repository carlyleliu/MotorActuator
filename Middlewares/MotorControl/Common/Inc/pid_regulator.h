/**
 ******************************************************************************
 * @file    pid_regulator.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          PID reulator component of the motor Control SDK.
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
 * @ingroup PIDRegulator
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PIDREGULATOR_H
#define __PIDREGULATOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup PIDRegulator
 * @{
 */

/**
 * @brief Handle of a PID component
 *
 * @detail This structure stores all the parameters needed to perform a proportional,
 * integral and derivative regulation computation. It also stores configurable limits
 * in order to saturate the integral terms and the output value. This structure is
 * passed to each PID component function.
 */
typedef struct {
    int16_t def_kp_gain;            /**< Default @f$K_{pg}@f$ gain */
    int16_t def_ki_gain;            /**< Default @f$K_{ig}@f$ gain */
    int16_t kp_gain;                /**< @f$K_{pg}@f$ gain used by PID component */
    int16_t ki_gain;                /**< @f$K_{ig}@f$ gain used by PID component */
    int32_t integral_term;          /**< integral term */
    int32_t upper_integral_limit;   /**< Upper limit used to saturate the integral
                                     term given by @f$\frac{K_{ig}}{K_{id}} @f$ * integral of
                                     process variable error */
    int32_t lower_integral_limit;   /**< Lower limit used to saturate the integral
                                     term given by Ki / Ki divisor * integral of
                                     process variable error */
    int16_t upper_output_limit;     /**< Upper limit used to saturate the PI output */
    int16_t lower_output_limit;     /**< Lower limit used to saturate the PI output */
    uint16_t kp_divisor;            /**< Kp gain divisor, used in conjuction with
                                     Kp gain allows obtaining fractional values.
                                     If FULL_MISRA_C_COMPLIANCY is not defined
                                     the divisor is implemented through
                                     algebrical right shifts to speed up PI
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be
                                     executed */
    uint16_t ki_divisor;            /**< Ki gain divisor, used in conjuction with
                                     Ki gain allows obtaining fractional values.
                                     If FULL_MISRA_C_COMPLIANCY is not defined
                                     the divisor is implemented through
                                     algebrical right shifts to speed up PI
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be
                                     executed */
    uint16_t kp_divisor_pow2;       /**< Kp gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 as 2^9 = 512 */
    uint16_t ki_divisor_pow2;       /**< Ki gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 as 2^9 = 512 */
    int16_t def_kd_gain;            /**< Default Kd gain */
    int16_t kd_gain;                /**< Kd gain used by PID component */
    uint16_t kd_divisor;            /**< Kd gain divisor, used in conjuction with
                                     Kd gain allows obtaining fractional values.
                                     If FULL_MISRA_C_COMPLIANCY is not defined
                                     the divisor is implemented through
                                     algebrical right shifts to speed up PI
                                     execution. Only in this case this parameter
                                     specifies the number of right shifts to be
                                     executed */
    uint16_t kd_divisor_pow2;       /*!< Kd gain divisor expressed as power of 2.
                                     E.g. if gain divisor is 512 the value
                                     must be 9 as 2^9 = 512 */
    int32_t prev_process_var_error; /*!< previous process variable used by the
                                     derivative part of the PID component */
} pid_integer_t;

/**
 * It initializes the handle
 */
void pid_init(pid_integer_t* phandle);

/**
 * It updates the Kp gain
 */
void pid_set_kp(pid_integer_t* phandle, int16_t kp_gain);

/**
 * It updates the Ki gain
 */
void pid_set_ki(pid_integer_t* phandle, int16_t ki_gain);

/**
 *  It returns the Kp gain
 */
int16_t pid_get_kp(pid_integer_t* phandle);

/**
 * It returns the Ki gain
 */
int16_t pid_get_ki(pid_integer_t* phandle);

/**
 * It returns the Default Kp gain
 */
int16_t pid_get_default_kp(pid_integer_t* phandle);

/**
 * It returns the Default Ki gain of the passed PI object
 */
int16_t pid_get_default_ki(pid_integer_t* phandle);

/**
 * It set a new value into the PI integral term
 */
void pid_set_integral_term(pid_integer_t* phandle, int32_t integral_term_value);

/**
 * It returns the Kp gain divisor
 */
uint16_t pid_get_kp_divisor(pid_integer_t* phandle);

/**
 * It updates the Kp divisor
 */
void pid_set_kp_divisor_pow2(pid_integer_t* phandle, uint16_t kp_divisor_pow2);

/**
 * It returns the Ki gain divisor of the passed PI object
 */
uint16_t pid_get_ki_divisor(pid_integer_t* phandle);

/**
 * It updates the Ki divisor
 */
void pid_set_ki_divisor_pow2(pid_integer_t* phandle, uint16_t ki_divisor_pow2);

/**
 * It set a new value for lower integral term limit
 */
void pid_set_lower_integral_term_limit(pid_integer_t* phandle, int32_t wlower_limit);

/**
 * It set a new value for upper integral term limit
 */
void pid_set_upper_integral_term_limit(pid_integer_t* phandle, int32_t upper_limit);

/**
 * It set a new value for lower output limit
 */
void pid_set_lower_output_limit(pid_integer_t* phandle, int16_t hlower_limit);

/**
 * It set a new value for upper output limit
 */
void pid_set_upper_output_limit(pid_integer_t* phandle, int16_t hupper_limit);

/**
 * It set a new value into the PID Previous error variable required to
 * compute derivative term
 */
void pid_set_prev_error(pid_integer_t* phandle, int32_t prev_process_var_error);

/**
 * @brief  It updates the Kd gain
 */
void pid_set_kd(pid_integer_t* phandle, int16_t kd_gain);

/**
 * It returns the Kd gain
 */
int16_t pid_get_kd(pid_integer_t* phandle);

/**
 * It returns the Kd gain divisor of the PID object passed
 */
uint16_t pid_get_kd_divisor(pid_integer_t* phandle);

/**
 * Updates the Kd divisor
 */
void pid_set_kd_divisor_pow2(pid_integer_t* phandle, uint16_t kd_divisor_pow2);

/**
 * This function compute the output of a PI regulator sum of its
 * proportional and integral_terms
 */
int16_t pi_controller(pid_integer_t* phandle, int32_t wprocess_var_error);

/**
 * This function compute the output of a PID regulator sum of its
 *  proportional, integral and derivative terms
 */
int16_t pid_controller(pid_integer_t* phandle, int32_t wprocess_var_error);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__PIDREGULATOR_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
