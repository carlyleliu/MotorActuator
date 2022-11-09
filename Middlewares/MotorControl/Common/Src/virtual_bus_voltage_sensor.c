/**
 ******************************************************************************
 * @file    virtual_bus_voltage_sensor.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the Virtual Bus Voltage Sensor component of the motor Control SDK.
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
#include "virtual_bus_voltage_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup BusVoltageSensor
 * @{
 */

/** @defgroup VirtualBusVoltageSensor Virtual Bus Voltage Sensor
 * @brief Virtual Bus Voltage Sensor implementation.
 *
 * @todo Document the Virtual Bus Voltage Sensor "module".
 *
 * @{
 */

/**
 * @brief  It initializes bus voltage conversion for virtual bus voltage sensor
 * @param  phandle related Handle of virtual_bus_voltage_sensor_t
 * @retval none
 */
__weak void vvbs_init(virtual_bus_voltage_sensor_t* phandle)
{
    phandle->_Super.fault_state = MC_NO_ERROR;
    phandle->_Super.latest_conv = phandle->expected_vbus_d;
    phandle->_Super.av_bus_voltage_d = phandle->expected_vbus_d;
}

/**
 * @brief  It simply returns in virtual Vbus sensor implementation
 * @param  phandle related Handle of virtual_bus_voltage_sensor_t
 * @retval none
 */
__weak void vvbs_clear(virtual_bus_voltage_sensor_t* phandle)
{
    return;
}

/**
 * @brief  It returns MC_NO_ERROR
 * @param  phandle related Handle of virtual_bus_voltage_sensor_t
 * @retval uint16_t Fault code error: MC_NO_ERROR
 */
__weak uint16_t vvbs_no_errors(virtual_bus_voltage_sensor_t* phandle)
{
    return (MC_NO_ERROR);
}

/**
 * @}
 */

/**
 * @}
 */

/** @} */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
