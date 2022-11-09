/**
 ******************************************************************************
 * @file    r_divider_bus_voltage_sensor.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the  features
 *          of the Resistor Divider Bus Voltage Sensor component of the motor
 *          Control SDK:
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
#include "r_divider_bus_voltage_sensor.h"

#include "regular_conversion_manager.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup BusVoltageSensor
 * @{
 */

/** @defgroup RDividerBusVoltageSensor Resistor Divider Bus Voltage Sensor
 * @brief Resistor Divider Bus Voltage Sensor implementation
 *
 * @todo Document the Resistor Divider Bus Voltage Sensor "module".
 *
 * @{
 */

/**
  * @brief  It initializes bus voltage conversion (ADC, ADC channel, conversion time.
    It must be called only after PWMC_Init.
  * @param  phandle related r_divider_t
  * @retval none
  */
__weak void rvbs_init(r_divider_t* phandle)
{
    /* Need to be register with RegularConvManager */
    phandle->conv_handle = rcm_register_reg_conv(&phandle->vbus_reg_conv);
    /* Check */
    rvbs_clear(phandle);
}

/**
 * @brief  It clears bus voltage FW variable containing average bus voltage
 *         value
 * @param  phandle related r_divider_t
 * @retval none
 */
__weak void rvbs_clear(r_divider_t* phandle)
{
    uint16_t aux;
    uint16_t index;

    aux = (phandle->over_voltage_threshold + phandle->under_voltage_threshold) / 2u;
    for (index = 0u; index < phandle->low_pass_filter_bw; index++) {
        phandle->ptr_aver_buffer[index] = aux;
    }
    phandle->_Super.latest_conv = aux;
    phandle->_Super.av_bus_voltage_d = aux;
    phandle->index = 0;
}

static uint16_t rvbs_convert_vbus_filtrered(r_divider_t* phandle)
{
    uint16_t aux;
    uint8_t vindex;
    uint16_t max = 0, min = 0;
    uint32_t tot = 0u;

    for (vindex = 0; vindex < phandle->low_pass_filter_bw;) {
        aux = rcm_exec_regular_conv(phandle->conv_handle);

        if (aux != 0xFFFFu) {
            if (vindex == 0) {
                min = aux;
                max = aux;
            } else {
                if (aux < min) {
                    min = aux;
                }
                if (aux > max) {
                    max = aux;
                }
            }
            vindex++;

            tot += aux;
        }
    }

    tot -= max;
    tot -= min;
    return (uint16_t)(tot / (phandle->low_pass_filter_bw - 2u));
}

/**
 * @brief  It actually performes the Vbus ADC conversion and updates average
 *         value
 * @param  phandle related r_divider_t
 * @retval uint16_t Fault code error
 */
__weak uint16_t rvbs_calc_av_vbus_filt(r_divider_t* phandle)
{
    uint32_t temp;
    uint16_t aux;
    uint8_t i;

    aux = rvbs_convert_vbus_filtrered(phandle);

    if (aux != 0xFFFF) {
        phandle->ptr_aver_buffer[phandle->index] = aux;
        temp = 0;
        for (i = 0; i < phandle->low_pass_filter_bw; i++) {
            temp += phandle->ptr_aver_buffer[i];
        }
        temp /= phandle->low_pass_filter_bw;
        phandle->_Super.av_bus_voltage_d = (uint16_t)temp;
        phandle->_Super.latest_conv = aux;

        if (phandle->index < phandle->low_pass_filter_bw - 1) {
            phandle->index++;
        } else {
            phandle->index = 0;
        }
    }

    phandle->_Super.fault_state = rvbs_check_fault_state(phandle);

    return (phandle->_Super.fault_state);
}

/**
 * @brief  It actually performes the Vbus ADC conversion and updates average
 *         value
 * @param  phandle related r_divider_t
 * @retval uint16_t Fault code error
 */
__weak uint16_t rvbs_calc_av_vbus(r_divider_t* phandle)
{
    uint32_t temp;
    uint16_t aux;
    uint8_t i;

    aux = rcm_exec_regular_conv(phandle->conv_handle);

    if (aux != 0xFFFF) {
        phandle->ptr_aver_buffer[phandle->index] = aux;
        temp = 0;
        for (i = 0; i < phandle->low_pass_filter_bw; i++) {
            temp += phandle->ptr_aver_buffer[i];
        }
        temp /= phandle->low_pass_filter_bw;
        phandle->_Super.av_bus_voltage_d = (uint16_t)temp;
        phandle->_Super.latest_conv = aux;

        if (phandle->index < phandle->low_pass_filter_bw - 1) {
            phandle->index++;
        } else {
            phandle->index = 0;
        }
    }

    phandle->_Super.fault_state = rvbs_check_fault_state(phandle);

    return (phandle->_Super.fault_state);
}

/**
 * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
 *         bus voltage and protection threshold values
 * @param  phandle related r_divider_t
 * @retval uint16_t Fault code error
 */
__weak uint16_t rvbs_check_fault_state(r_divider_t* phandle)
{
    uint16_t fault;

    if (phandle->_Super.av_bus_voltage_d > phandle->over_voltage_threshold) {
        fault = MC_OVER_VOLT;
    } else if (phandle->_Super.av_bus_voltage_d < phandle->under_voltage_threshold) {
        fault = MC_UNDER_VOLT;
    } else {
        fault = MC_NO_ERROR;
    }
    return fault;
}

/**
 * @}
 */

/**
 * @}
 */

/** @} */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
