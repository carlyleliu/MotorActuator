/**
 ******************************************************************************
 * @file    ntc_temperature_sensor.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the Temperature Sensor component of the motor Control SDK.
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
#include "ntc_temperature_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup TemperatureSensor NTC Temperature Sensor
 * @brief Allows to read the temperature of the heat sink
 *
 * This component implements both a virtual and a real temperature sensor,
 * depending on the sensor availability.
 *
 * Access to the MCU peripherals needed to acquire the temperature (GPIO and ADC
 * used for regular conversion) is managed by the PWM component used in the motor
 * Control subsystem. As a consequence, this NTC temperature sensor implementation
 * is hardware-independent.
 *
 * If a real temperature sensor is available (Sensor Type = #REAL_SENSOR),
 * this component can handle NTC sensors or, more generally, analog temperature sensors
 * which output is related to the temperature by the following formula:
 *
 * @f[
 *               V_{out} = V_0 + \frac{dV}{dT} \cdot ( T - T_0)
 * @f]
 *
 * In case a real temperature sensor is not available (Sensor Type = #VIRTUAL_SENSOR),
 * This component will always returns a constant, programmable, temperature.
 *
 * @{
 */

/* Private function prototypes -----------------------------------------------*/
uint16_t ntc_set_fault_state(ntc_t* phandle);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Returns fault when temperature exceeds the over voltage protection threshold
 *
 *  @p phandle : Pointer on Handle structure of TemperatureSensor component
 *
 *  @r Fault status : Updated internal fault status
 */
__weak uint16_t ntc_set_fault_state(ntc_t* phandle)
{
    uint16_t fault;

    if (phandle->av_temp_d > phandle->over_temp_threshold) {
        fault = MC_OVER_TEMP;
    } else if (phandle->av_temp_d < phandle->over_temp_deact_threshold) {
        fault = MC_NO_ERROR;
    } else {
        fault = phandle->fault_state;
    }
    return fault;
}

/* Functions ---------------------------------------------------- */

/**
 * @brief Initializes temperature sensing conversions
 *
 *  @p phandle : Pointer on Handle structure of TemperatureSensor component
 *
 *  @p ptr_pwmnCurrentSensor : Handle on the PWMC component to be used for regular conversions
 */
__weak void ntc_init(ntc_t* phandle)
{
    if (phandle->sensor_type == REAL_SENSOR) {
        /* Need to be register with RegularConvManager */
        phandle->conv_handle = rcm_register_reg_conv(&phandle->temp_reg_conv);
        ntc_clear(phandle);
    } else /* case VIRTUAL_SENSOR */
    {
        phandle->fault_state = MC_NO_ERROR;
        phandle->av_temp_d = phandle->expected_temp_d;
    }
}

/**
 * @brief Initializes internal average temperature computed value
 *
 *  @p phandle : Pointer on Handle structure of TemperatureSensor component
 */
__weak void ntc_clear(ntc_t* phandle)
{
    phandle->av_temp_d = 0u;
}

/**
 * @brief Performs the temperature sensing average computation after an ADC conversion
 *
 *  @p phandle : Pointer on Handle structure of TemperatureSensor component
 *
 *  @r Fault status : Error reported in case of an over temperature detection
 */
__weak uint16_t ntc_calc_av_temp(ntc_t* phandle)
{
    uint32_t temp;
    uint16_t aux;

    if (phandle->sensor_type == REAL_SENSOR) {
        aux = rcm_exec_regular_conv(phandle->conv_handle);

        if (aux != 0xFFFFu) {
            temp = (uint32_t)(phandle->low_pass_filter_bw) - 1u;
            temp *= (uint32_t)(phandle->av_temp_d);
            temp += aux;
            temp /= (uint32_t)(phandle->low_pass_filter_bw);

            phandle->av_temp_d = (uint16_t)temp;
        }

        phandle->fault_state = ntc_set_fault_state(phandle);
    } else /* case VIRTUAL_SENSOR */
    {
        phandle->fault_state = MC_NO_ERROR;
    }

    return (phandle->fault_state);
}

/**
 * @brief  Returns latest averaged temperature measured expressed in u16Celsius
 *
 * @p phandle : Pointer on Handle structure of TemperatureSensor component
 *
 * @r AverageTemperature : Current averaged temperature measured (in u16Celsius)
 */
__weak uint16_t ntc_get_avtemp_d(ntc_t* phandle)
{
    return (phandle->av_temp_d);
}

/**
 * @brief  Returns latest averaged temperature expressed in Celsius degrees
 *
 * @p phandle : Pointer on Handle structure of TemperatureSensor component
 *
 * @r AverageTemperature : Latest averaged temperature measured (in Celsius degrees)
 */
__weak int16_t ntc_get_av_temp_c(ntc_t* phandle)
{
    int32_t temp;

    if (phandle->sensor_type == REAL_SENSOR) {
        temp = (int32_t)(phandle->av_temp_d);
        temp -= (int32_t)(phandle->v0);
        temp *= phandle->sensitivity;
        temp = temp / 65536 + (int32_t)(phandle->t0);
    } else {
        temp = phandle->expected_temp_c;
    }
    return ((int16_t)temp);
}

/**
 * @brief  Returns Temperature measurement fault status
 *
 * Fault status can be either MC_OVER_TEMP when measure exceeds the protection threshold values or
 * MC_NO_ERROR if it is inside authorized range.
 *
 * @p phandle: Pointer on Handle structure of TemperatureSensor component.
 *
 *  @r Fault status : read internal fault state
 */
__weak uint16_t ntc_check_temp(ntc_t* phandle)
{
    return (phandle->fault_state);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
