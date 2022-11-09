/**
 ******************************************************************************
 * @file    pqd_motor_power_measurement.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          pqd_motor_power_measurement component of the motor Control SDK.
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
#ifndef __PQD_motorPOWERMEASUREMENT_H
#define __PQD_motorPOWERMEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"
#include "motor_power_measurement.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup motorpowermeasurement
 * @{
 */

/** @addtogroup pqd_motorpowermeasurement
 * @{
 */

typedef struct {
    motor_pow_meas_t _super;

    int32_t conv_fact; /* It is the conversion factor used to convert the
                        variables expressed in digit into variables expressed
                        in physical measurement unit. It is used to convert the
                        power in watts. It must be equal to
                        (1000 * 3 * Vddï¿½)/(sqrt(3) * Rshunt * Aop) */

    ptr_foc_vars_t foc_vars;       /*!< Pointer to FOC vars used by MPM.*/
    bus_voltage_sensor_t* ptr_vbs; /*!< Bus voltage sensor object used by MPM.*/
} pqd_motor_pow_meas_t;

/**
 * @brief Implementation of derived class init method. It should be called before each motor restart.
 * @param phandle related component instance.
 * @retval none.
 */
void pqd_clear(pqd_motor_pow_meas_t* phandle);

/**
 * @brief Implementation of derived class CalcElmotorPower.
 * @param phandle related component instance.
 * @retval int16_t The measured motor power expressed in watt.
 */
void pqd_calc_el_motor_power(pqd_motor_pow_meas_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

/** @} */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __CCC_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
