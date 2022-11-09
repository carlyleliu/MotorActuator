/**
 ******************************************************************************
 * @file    bus_voltage_sensor.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the BusVoltageSensor component of the motor Control SDK.
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

#include "bus_voltage_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup BusVoltageSensor Bus Voltage Sensor
 * @brief Bus Voltage Sensor components of the motor Control SDK
 *
 * Two Bus Voltage Sensor implementations are provided:
 *
 * - The @ref RDividerBusVoltageSensor "Resistor Divider Bus Voltage Sensor" operates as the name suggests
 * - The @ref VirtualBusVoltageSensor "Virtual Bus Voltage Sensor" does not make measurement but rather
 *   returns a fixed, application defined value.
 *
 * @todo Document the Bus Voltage Sensor "module".
 *
 * @{
 */

/**
 * @brief  It return latest Vbus conversion result expressed in u16Volt
 * @param  phandle related Handle of bus_voltage_sensor_t
 * @retval uint16_t Latest Vbus conversion result in digit
 */
__weak uint16_t vbs_get_bus_voltage_d(bus_voltage_sensor_t* phandle)
{
    return (phandle->latest_conv);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It return latest averaged Vbus measurement expressed in u16Volt
 * @param  phandle related Handle of bus_voltage_sensor_t
 * @retval uint16_t Latest averaged Vbus measurement in digit
 */
__weak uint16_t vbs_get_avbus_voltage_d(bus_voltage_sensor_t* phandle)
{
    return (phandle->av_bus_voltage_d);
}

/**
 * @brief  It return latest averaged Vbus measurement expressed in Volts
 * @param  phandle related Handle of bus_voltage_sensor_t
 * @retval uint16_t Latest averaged Vbus measurement in Volts
 */
__weak uint16_t vbs_get_avbus_voltage_v(bus_voltage_sensor_t* phandle)
{
    uint32_t temp;

    temp = (uint32_t)(phandle->av_bus_voltage_d);
    temp *= phandle->conversion_factor;
    temp /= 65536u;

    return ((uint16_t)temp);
}

/**
 * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
 *         bus voltage and protection threshold values
 * @param  phandle related Handle of bus_voltage_sensor_t
 * @retval uint16_t Fault code error
 */
__weak uint16_t vbs_check_vbus(bus_voltage_sensor_t* phandle)
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
