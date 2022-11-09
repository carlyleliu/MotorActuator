/**
 ******************************************************************************
 * @file    r_divider_bus_voltage_sensor.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Resistor Divider Bus Voltage Sensor component of the motor Control SDK.
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
 * @ingroup RDividerBusVoltageSensor
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RDIVIDER_BUSVOLTAGESENSOR_H
#define __RDIVIDER_BUSVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"
#include "regular_conversion_manager.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup BusVoltageSensor
 * @{
 */

/** @addtogroup RDividerBusVoltageSensor
 * @{
 */

/**
 * @brief  Rdivider class parameters definition
 */
typedef struct {
    bus_voltage_sensor_t _Super;

    reg_conv_t vbus_reg_conv;
    uint16_t low_pass_filter_bw;      /*!< Use this number to configure the Vbus
                                       first order software filter bandwidth.
                                       low_pass_filter_bw = VBS_CalcBusReading
                                       call rate [Hz]/ FilterBandwidth[Hz] */
    uint16_t over_voltage_threshold;  /*!< It represents the over voltage protection
                                       intervention threshold. To be expressed
                                       in digital value through formula:
                                       hover_voltage_threshold (digital value) =
                                       Over Voltage Threshold (V) * 65536
                                       / hConversionFactor */
    uint16_t under_voltage_threshold; /*!< It represents the under voltage protection
                                       intervention threshold. To be expressed
                                       in digital value through formula:
                                       hunder_voltage_threshold (digital value)=
                                       Under Voltage Threshold (V) * 65536
                                       / hConversionFactor */
    uint16_t* ptr_aver_buffer;        /*!< Buffer used to compute average value.*/
    uint8_t elem;                     /*!< Number of stored elements in the average buffer.*/
    uint8_t index;                    /*!< Index of last stored element in the average buffer.*/
    uint8_t conv_handle;              /*!< handle to the regular conversion */

} r_divider_t;

/* Exported functions ------------------------------------------------------- */
void rvbs_init(r_divider_t* phandle);
void rvbs_clear(r_divider_t* phandle);
uint16_t rvbs_calc_av_vbus_filt(r_divider_t* phandle);
uint16_t rvbs_calc_av_vbus(r_divider_t* phandle);
uint16_t rvbs_check_fault_state(r_divider_t* phandle);

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

#endif /* __RDividerBusVoltageSensor_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
