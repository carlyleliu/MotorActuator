/**
 ******************************************************************************
 * @file    motor_power_measurement.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the motor Power Measurement component of the motor Control SDK:
 *
 *           * Calculate power of the motor
 *           * Clear power measurement
 *           * Get Power of the motor
 *           * Get average Power of the motor
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

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup motorpowermeasurement motor Power Measurement
 * @brief motor Power Measurement component of the motor Control SDK
 *
 * @todo Document the motor Power Measurement "module".
 *
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "motor_power_measurement.h"

#include "mc_type.h"

/**
 * @brief  It should be called before each motor restart. It clears the
 *         measurement buffer and initialize the index.
 * @param power handle.
 * @retval none.
 */
__weak void mpm_clear(motor_pow_meas_t* phandle)
{
    uint16_t i;
    for (i = 0u; i < MPM_BUFFER_LENGHT; i++) {
        phandle->meas_buffer[i] = 0;
    }
    phandle->next_meas_buffer_index = 0u;
    phandle->last_meas_buffer_index = 0u;
}

/**
 * @brief  This method should be called with periodicity. It computes and
 *         returns the measured motor power expressed in watt. It is also used
 *         to fill, with that measure, the buffer used to compute the average
 *         motor power.
 * @param phandle pointer on the related component instance.
 * @retval int16_t The measured motor power expressed in watt.
 */
__weak int16_t mpm_calc_el_motor_power(motor_pow_meas_t* phandle, int16_t CurrentmotorPower)
{
    uint16_t i;
    int32_t aux = 0;

    /* Store the measured values in the buffer.*/
    phandle->meas_buffer[phandle->next_meas_buffer_index] = CurrentmotorPower;
    phandle->last_meas_buffer_index = phandle->next_meas_buffer_index;
    phandle->next_meas_buffer_index++;
    if (phandle->next_meas_buffer_index >= MPM_BUFFER_LENGHT) {
        phandle->next_meas_buffer_index = 0u;
    }
    /* Compute the average measured motor power */
    for (i = 0u; i < MPM_BUFFER_LENGHT; i++) {
        aux += (int32_t)(phandle->meas_buffer[i]);
    }
    aux /= (int32_t)MPM_BUFFER_LENGHT;
    phandle->avrg_el_motor_power_w = (int16_t)(aux);
    /* Return the last measured motor power */
    return CurrentmotorPower;
}
/**
 * @brief  This method is used to get the last measured motor power
 *         (instantaneous value) expressed in watt.
 * @param phandle pointer on the related component instance.
 * @retval int16_t The last measured motor power (instantaneous value)
 *         expressed in watt.
 */
__weak int16_t mpm_get_el_motor_power_w(motor_pow_meas_t* phandle)
{
    return (phandle->meas_buffer[phandle->last_meas_buffer_index]);
}

/**
 * @brief  This method is used to get the average measured motor power
 *         expressed in watt.
 * @param phandle pointer on the related component instance.
 * @retval int16_t The average measured motor power expressed in watt.
 */
__weak int16_t mpm_get_avrg_el_motor_power_w(motor_pow_meas_t* phandle)
{
    return (phandle->avrg_el_motor_power_w);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
