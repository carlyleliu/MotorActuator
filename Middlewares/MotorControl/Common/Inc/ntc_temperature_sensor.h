/**
 ******************************************************************************
 * @file    ntc_temperature_sensor.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Temperature Sensor component of the motor Control SDK.
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
 * @ingroup TemperatureSensor
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TEMPERATURESENSOR_H
#define __TEMPERATURESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "regular_conversion_manager.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup TemperatureSensor
 * @{
 */

/**
 * @brief ntc_t structure used for temperature monitoring
 *
 */
typedef struct {
    sensor_type_t sensor_type; /**< Type of instanced temperature.
                                This parameter can be REAL_SENSOR or VIRTUAL_SENSOR */

    reg_conv_t temp_reg_conv;

    uint16_t av_temp_d; /**< It contains latest available average Vbus.
                         This parameter is expressed in u16Celsius */

    uint16_t expected_temp_d; /**< Default set when no sensor available (ie virtual sensor) */

    uint16_t expected_temp_c; /**< Default value when no sensor available (ie virtual sensor).
                               This parameter is expressed in Celsius */

    uint16_t fault_state; /**< Contains latest Fault code.
                           This parameter is set to MC_OVER_TEMP or MC_NO_ERROR */

    uint16_t low_pass_filter_bw;        /**< used to configure the first order software filter bandwidth.
                                         low_pass_filter_bw = NTC_CalcBusReading
                                         call rate [Hz]/ FilterBandwidth[Hz] */
    uint16_t over_temp_threshold;       /**< Represents the over voltage protection intervention threshold.
                                         This parameter is expressed in u16Celsius through formula:
                                         over_temp_threshold =
                                         (V0[V]+dV/dT[V/°C]*(OverTempThreshold[°C] - T0[°C]))* 65536 / MCU supply voltage       */
    uint16_t over_temp_deact_threshold; /**< Temperature threshold below which an active over temperature fault is cleared.
                                         This parameter is expressed in u16Celsius through formula:
                                         over_temp_deact_threshold =
                                         (V0[V]+dV/dT[V/°C]*(OverTempDeactThresh[°C] - T0[°C]))* 65536 / MCU supply voltage*/
    int16_t sensitivity;                /**< NTC sensitivity
                                         This parameter is equal to MCU supply voltage [V] / dV/dT [V/°C] */
    uint32_t v0;                        /**< V0 voltage constant value used to convert the temperature into Volts.
                                         This parameter is equal V0*65536/MCU supply
                                         Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */
    uint16_t t0;                        /**< T0 temperature constant value used to convert the temperature into Volts
                                         Used in through formula: V[V]=V0+dV/dT[V/°C]*(T-T0)[°C] */
    uint8_t conv_handle;                /*!< handle to the regular conversion */

} ntc_t;

/* Initialize temperature sensing parameters */
void ntc_init(ntc_t* phandle);

/* Clear static average temperature value */
void ntc_clear(ntc_t* phandle);

/* Temperature sensing computation */
uint16_t ntc_calc_av_temp(ntc_t* phandle);

/* Get averaged temperature measurement expressed in u16Celsius */
uint16_t ntc_get_avtemp_d(ntc_t* phandle);

/* Get averaged temperature measurement expressed in Celsius degrees */
int16_t ntc_get_av_temp_c(ntc_t* phandle);

/* Get the temperature measurement fault status */
uint16_t ntc_check_temp(ntc_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __TEMPERATURESENSOR_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
