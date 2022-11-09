/**
 ******************************************************************************
 * @file    motor_power_measurement.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          motor Power Measurement component of the motor Control SDK.
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
 * @ingroup motorpowermeasurement
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __motorPOWERMEASUREMENT_H
#define __motorPOWERMEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup motorpowermeasurement
 * @{
 */

#define MPM_BUFFER_LENGHT                        \
    128u /*!< Length of buffer used to store the \
             instantaneous measurements of motor power.*/
typedef struct {
    int16_t meas_buffer[MPM_BUFFER_LENGHT]; /*!< Buffer used by MPM object to
                                             store the instantaneous
                                             measurements of motor power. */
    uint16_t next_meas_buffer_index;        /*!< Index of the buffer that will contain the
                                             next motor power measurement. */
    uint16_t last_meas_buffer_index;        /*!< Index of the buffer that contains the last
                                             motor power measurement. */
    int16_t avrg_el_motor_power_w;          /*!< The average measured motor power expressed in
                                             watt. */
} motor_pow_meas_t;

/**
 * @brief  motorPowerMeasurement class init struct definition
 */
typedef void* ptr_mpm_init_struct_t;

/**
 * @brief  It should be called before each motor restart. It clears the
 *         measurement buffer and initialize the index.
 * @param power handle.
 * @retval none.
 */
void mpm_clear(motor_pow_meas_t* phandle);

/**
 * @brief  This method should be called with periodicity. It computes and
 *         returns the measured motor power expressed in watt. It is also used
 *         to fill, with that measure, the buffer used to compute the average
 *         motor power.
 * @param power handle.
 * @retval int16_t The measured motor power expressed in watt.
 */
int16_t mpm_calc_el_motor_power(motor_pow_meas_t* phandle, int16_t motorPower);

/**
 * @brief  This method is used to get the last measured motor power
 *         (instantaneous value) expressed in watt.
 * @param power handle.
 * @retval int16_t The last measured motor power (instantaneous value)
 *         expressed in watt.
 */
int16_t mpm_get_el_motor_power_w(motor_pow_meas_t* phandle);

/**
 * @brief  This method is used to get the average measured motor power
 *         expressed in watt.
 * @param phandle related component instance.
 * @retval int16_t The average measured motor power expressed in watt.
 */
int16_t mpm_get_avrg_el_motor_power_w(motor_pow_meas_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CCC_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
