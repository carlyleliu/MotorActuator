/**
 ******************************************************************************
 * @file    pqd_motor_power_measurement.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the PQD motor Power Measurement component of the motor Control SDK:
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

/* Includes ------------------------------------------------------------------*/

#include "pqd_motor_power_measurement.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup motorpowermeasurement
 * @{
 */

/** @defgroup pqd_motorpowermeasurement PQD motor Power Measurement
 * @brief PQD motor Power Measurement component of the motor Control SDK
 *
 * pqd method to measure power of the motor
 *
 * @todo Document the PQD motor Power Measurement "module".
 *
 * @{
 */

/**
 * @brief  This method should be called with periodicity. It computes and
 *         returns the measured motor power expressed in watt. It is also used
 *         to fill, with that measure, the buffer used to compute the average
 *         motor power.
 * @param power handle.
 * @retval int16_t The measured motor power expressed in watt.
 */
__weak void pqd_calc_el_motor_power(pqd_motor_pow_meas_t* phandle)
{
    int32_t aux, aux2, aux3;
    qd_t iqd = phandle->foc_vars->iqd;
    qd_t vqd = phandle->foc_vars->vqd;
    aux = ((int32_t)iqd.q * (int32_t)vqd.q) + ((int32_t)iqd.d * (int32_t)vqd.d);
    aux /= 65536;

    aux2 = phandle->conv_fact * (int32_t)vbs_get_avbus_voltage_v(phandle->ptr_vbs);
    aux2 /= 600; /* 600 is max bus voltage expressed in volt.*/

    aux3 = aux * aux2;
    aux3 *= 6; /* 6 is max bus voltage expressed in thousend of volt.*/
    aux3 /= 10;
    aux3 /= 65536;

    mpm_calc_el_motor_power(&phandle->_super, aux3);
}

/**
 * @}
 */

/**
 * @}
 */

/** @} */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
