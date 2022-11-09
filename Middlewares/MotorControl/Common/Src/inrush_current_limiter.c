/**
 ******************************************************************************
 * @file    inrush_current_limiter.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions implementing the
 *          InrushCurrentLimiter feature of the motor Control SDK :
 *
 *          * icl_init() to initialize dedicated variables
 *          * icl_exec() to manage the icL state machine
 *          * icl_get_state() to get the current icL state machine
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
#include "inrush_current_limiter.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup icL Inrush Current Limiter
 * @brief Inrush Current Limiter component of the motor Control SDK
 *
 * @todo Document the Inrush Current Limiter "module".
 *
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief  It initializes all the needed icL component variables.
 *         It shall be called only once, right after the icL instance creation.
 *         It makes the bus voltage sensor and digital output assignment needed
 *         for the icL instance usage.
 * @param  phandle: handler of the current instance of the icL component
 * @param  ptr_vbs the bus voltage sensor used by the icL.
 * @param  p_dout the digital output used by the icL.
 * @retval none.
 */
__weak void icl_init(icl_t* phandle, bus_voltage_sensor_t* ptr_vbs, dout_t* p_dout)
{
    uint32_t aux;

    phandle->ptr_vbs = ptr_vbs;
    phandle->p_dout = p_dout;
    phandle->icl_state = ICL_ACTIVE;
    dout_set_output_state(p_dout, ACTIVE);
    phandle->icl_ticks_counter = 0u;
    aux = (uint32_t)(phandle->icl_durationms);
    aux *= (uint32_t)(phandle->icl_frequency_hz);
    aux /= 1000;
    aux -= 1;
    if (aux > UINT16_MAX) {
        aux = UINT16_MAX;
    }
    if (aux < 1) {
        aux = 1;
    }
    phandle->icl_total_ticks = (uint16_t)(aux);
}

/**
 * @brief  It clocks the Inrush Current Limiter and must be called with a
 *         frequency equal to the one set in the eac_frequency_hz parameter.
 * @param  phandle: handler of the current instance of the icL component
 * @retval icl_state_t returns the current icL state machine
 */
__weak icl_state_t icl_exec(icl_t* phandle)
{
    /* icL actions.*/
    switch (phandle->icl_state) {
        case ICL_ACTIVATION: {
            /* icL activation: counting the step before pass in ICL_ACTIVE */
            if (phandle->icl_ticks_counter == 0u) {
                phandle->icl_state = ICL_ACTIVE;
            } else {
                phandle->icl_ticks_counter--;
            }
        } break;

        case ICL_DEACTIVATION: {
            /* icL deactivation: counting the step before pass in ICL_INACTIVE.*/
            if (phandle->icl_ticks_counter == 0u) {
                phandle->icl_state = ICL_INACTIVE;
            } else {
                phandle->icl_ticks_counter--;
            }
        } break;

        case ICL_ACTIVE: {
            /* icL is active: if bus is present deactivate the icL */
            if (vbs_check_vbus(phandle->ptr_vbs) != MC_UNDER_VOLT) {
                dout_set_output_state(phandle->p_dout, INACTIVE);
                phandle->icl_state = ICL_DEACTIVATION;
                phandle->icl_ticks_counter = phandle->icl_total_ticks;
            }
        } break;

        case ICL_INACTIVE: {
            /* icL is inactive: if bus is not present activate the icL */
            if (vbs_check_vbus(phandle->ptr_vbs) == MC_UNDER_VOLT) {
                dout_set_output_state(phandle->p_dout, ACTIVE);
                phandle->icl_state = ICL_ACTIVATION;
                phandle->icl_ticks_counter = phandle->icl_total_ticks;
            }
        } break;

        case ICL_IDLE:
        default: {
        } break;
    }

    return phandle->icl_state;
}

/**
 * @brief It returns the current state of the icL state machine.
 * @param  phandle: handler of the current instance of the icL component
 * @retval icl_state_t returns the current icL state machine
 */
__weak icl_state_t icl_get_state(icl_t* phandle)
{
    return phandle->icl_state;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
