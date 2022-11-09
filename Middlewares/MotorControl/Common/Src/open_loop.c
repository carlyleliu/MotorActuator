/**
 ******************************************************************************
 * @file    open_loop.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the Open Loop component.
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
#include "open_loop.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup OpenLoop Open Loop Control
 * @brief Open Loop component of the motor Control SDK
 *
 * Used to run the motor in open loop mode.
 *
 * @todo Document the Bus Open Loop "module".
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/**
 * @brief  Initialize OpenLoop variables.
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 * @param  ptr_vss: Pointer on virtual speed sensor structure.
 *  @retval none
 */
__weak void ol_init(open_loop_t* phandle, virtual_speed_sensor_t* ptr_vss)
{
    phandle->voltage = phandle->default_voltage;
    phandle->ptr_vss = ptr_vss;
}

/**
 * @brief  Set vqd according to open loop phase voltage.
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 *  @retval qd_t vqd conditioned values.
 */
__weak qd_t ol_vqd_conditioning(open_loop_t* phandle)
{
    qd_t vqd;

    vqd.q = phandle->voltage;
    vqd.d = 0;

    return (vqd);
}

/**
 * @brief  Allow to set new open loop phase voltage.
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 * @param  new_voltage: New voltage value to apply.
 * @retval None
 */
__weak void ol_update_voltage(open_loop_t* phandle, int16_t new_voltage)
{
    phandle->voltage = new_voltage;
}

/**
 * @brief  Compute phase voltage to apply according to average mechanical speed (V/F Mode).
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 * @retval None
 */
__weak void ol_calc(open_loop_t* phandle)
{
    if (phandle->vf_mode == true) {
        /* V/F mode true means enabled */
        if (phandle->ptr_vss->_Super.avr_mecspeed_unit >= 0) {
            phandle->voltage = phandle->vf_offset + (phandle->vf_slope * phandle->ptr_vss->_Super.avr_mecspeed_unit);
        } else {
            phandle->voltage = phandle->vf_offset - (phandle->vf_slope * phandle->ptr_vss->_Super.avr_mecspeed_unit);
        }
    }
}

/**
 * @brief  Allow activation of the Voltage versus Frequency mode (V/F mode).
 * @param  phandle: Pointer on Handle structure of OpenLoop feature.
 * @param  vf_enabling: Flag to enable the V/F mode.
 * @retval None
 */
__weak void ol_vf(open_loop_t* phandle, bool vf_enabling)
{
    phandle->vf_mode = vf_enabling;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
