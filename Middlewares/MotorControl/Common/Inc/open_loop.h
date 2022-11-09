/**
 ******************************************************************************
 * @file    open_loop.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Open Loop component of the motor Control SDK.
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
 * @ingroup OpenLoop
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OPENLOOPCLASS_H
#define __OPENLOOPCLASS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "virtual_speed_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup OpenLoop
 * @{
 */

/**
 * @brief  open_loop_t structure used for phases definition
 */
typedef struct {
    int16_t default_voltage; /**< Default Open loop phase voltage. */

    bool vf_mode; /**< Flag to enable Voltage versus Frequency mode (V/F mode). */

    int16_t vf_offset; /**< Minimum Voltage to apply when frequency is equal to zero. */

    int16_t vf_slope; /**< Slope of V/F curve: Voltage = (vf_slope)*Frequency + vf_offset. */

    int16_t voltage; /**< Current Open loop phase voltage. */

    virtual_speed_sensor_t* ptr_vss; /**< Allow access on mechanical speed measured. */

} open_loop_t;

/**
 * @}
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize OpenLoop variables.
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 * @param  ptr_vss: Pointer on virtual speed sensor structure.
 *  @retval none
 */
void ol_init(open_loop_t* phandle, virtual_speed_sensor_t* ptr_vss);

/**
 * @brief  Set vqd according to open loop phase voltage.
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 *  @retval Voltage components vqd conditioned values.
 */
qd_t ol_vqd_conditioning(open_loop_t* phandle);

/**
 * @brief  Allow to set new open loop phase voltage.
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 * @param  new_voltage: New voltage value to apply.
 * @retval None
 */
void ol_update_voltage(open_loop_t* phandle, int16_t new_voltage);

/**
 * @brief  Compute phase voltage to apply according to average mechanical speed (V/F Mode).
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 * @retval None
 */
void ol_calc(open_loop_t* phandle);

/**
 * @brief  Allow activation of the Voltage versus Frequency mode (V/F mode).
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 * @param  vf_enabling: Flag to enable the V/F mode.
 * @retval None
 */
void ol_vf(open_loop_t* phandle, bool vf_enabling);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __OPENLOOPCLASS_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
