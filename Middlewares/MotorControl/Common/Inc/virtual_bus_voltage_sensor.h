/**
 ******************************************************************************
 * @file    virtual_bus_voltage_sensor.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Virtual Bus Voltage Sensor component of the motor Control SDK.
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
 * @ingroup VirtualBusVoltageSensor
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VIRTUAL_BUSVOLTAGESENSOR_H
#define __VIRTUAL_BUSVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup BusVoltageSensor
 * @{
 */

/** @addtogroup VirtualBusVoltageSensor
 * @{
 */

/**
 * @brief  Virtual Vbus sensor class parameters definition
 */
typedef struct {
    bus_voltage_sensor_t _Super;

    uint16_t expected_vbus_d; /*!< Expected Vbus voltage expressed in
                               digital value
                               hover_voltage_threshold(digital value)=
                               Over Voltage Threshold (V) * 65536
                               / 500 */
} virtual_bus_voltage_sensor_t;

/* Exported functions ------------------------------------------------------- */
void vvbs_init(virtual_bus_voltage_sensor_t* phandle);
void vvbs_clear(virtual_bus_voltage_sensor_t* phandle);
uint16_t vvbs_no_errors(virtual_bus_voltage_sensor_t* phandle);

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
