/**
 ******************************************************************************
 * @file    digital_output.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the Digital
 *          Output component of the motor Control SDK:
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
#include "digital_output.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup DigitalOutput Digital Output
 * @brief digital output component of the motor Control SDK
 *
 * @{
 */

/**
 * @brief Accordingly with selected polarity, it sets to active or inactive the
 *        digital output
 * @param phandle handler address of the digital output component.
 * @param state New requested state
 * @retval none
 */
__weak void dout_set_output_state(dout_t* phandle, doutput_state_t state)
{
    /* check parameter */
    if (NULL == phandle->gpio_set_output_pin || NULL == phandle->gpio_reset_output_pin) {
        return;
    }

    if (state == ACTIVE) {
        if (phandle->doutput_polarity == DOutputActiveHigh) {
            phandle->gpio_set_output_pin(phandle->doutput_port, phandle->doutput_pin);
        } else {
            phandle->gpio_reset_output_pin(phandle->doutput_port, phandle->doutput_pin);
        }
    } else if (phandle->doutput_polarity == DOutputActiveHigh) {
        phandle->gpio_reset_output_pin(phandle->doutput_port, phandle->doutput_pin);
    } else {
        phandle->gpio_set_output_pin(phandle->doutput_port, phandle->doutput_pin);
    }
    phandle->output_state = state;
}

/**
 * @brief It returns the state of the digital output
 * @param phandle pointer on related component instance
 * @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
 */
__weak doutput_state_t dout_get_output_state(dout_t* phandle)
{
    return (phandle->output_state);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
