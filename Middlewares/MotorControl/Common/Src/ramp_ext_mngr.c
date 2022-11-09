/**
 ******************************************************************************
 * @file    ramp_ext_mngr.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the Ramp extended Manager component of the motor Control SDK:
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
#include "ramp_ext_mngr.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup RampextMngr Ramp Manager
 * @brief Ramp extended Manager component of the motor Control SDK
 *
 * @todo Document the Ramp extended Manager "module".
 *
 * @{
 */
/* Private function prototypes -----------------------------------------------*/
uint32_t get_scaling_factor(int32_t Target);

/**
 * @brief  It reset the state variable to zero.
 * @param  phandle related Handle of struct RampMngr_Handle_t
 * @retval none.
 */
void remng_init(ramp_ext_mngr_t* phandle)
{
    phandle->ext = 0;
    phandle->target_final = 0;
    phandle->ramp_remaining_step = 0u;
    phandle->inc_dec_amount = 0;
    phandle->scaling_factor = 1u;

#ifdef FASTDIV
    fd_init(&(phandle->fd));
#endif
}

/**
  * @brief  Exec the ramp calculations and returns the current value of the
            state variable.
            It must be called at fixed interval defined in the hExecFreq.
  * @param  phandle related Handle of struct RampMngr_Handle_t
  * @retval int32_t value of the state variable
  */
__weak int32_t remng_calc(ramp_ext_mngr_t* phandle)
{
    int32_t ret_val;
    int32_t current_ref;

    current_ref = phandle->ext;

    /* Update the variable and terminates the ramp if needed. */
    if (phandle->ramp_remaining_step > 1u) {
        /* Increment/decrement the reference value. */
        current_ref += phandle->inc_dec_amount;

        /* Decrement the number of remaining steps */
        phandle->ramp_remaining_step--;
    } else if (phandle->ramp_remaining_step == 1u) {
        /* Set the backup value of target_final. */
        current_ref = phandle->target_final * (int32_t)(phandle->scaling_factor);
        phandle->ramp_remaining_step = 0u;
    } else {
        /* Do nothing. */
    }

    phandle->ext = current_ref;

#ifdef FASTDIV
    ret_val = fd_fast_div(&(phandle->fd), phandle->ext, (int32_t)(phandle->scaling_factor));
#else
    ret_val = phandle->ext / (int32_t)(phandle->scaling_factor);
#endif

    return ret_val;
}

/**
 * @brief  Setup the ramp to be executed
 * @param  phandle related Handle of struct RampMngr_Handle_t
 * @param  htarget_final (signed 32bit) final value of state variable at the end
 *         of the ramp.
 * @param  durationms (unsigned 32bit) the duration of the ramp expressed in
 *         milliseconds. It is possible to set 0 to perform an instantaneous
 *         change in the value.
 * @retval bool It returns true is command is valid, false otherwise
 */
__weak bool remng_exec_ramp(ramp_ext_mngr_t* phandle, int32_t target_final, uint32_t durationms)
{
    uint32_t aux;
    int32_t aux1;
    int32_t current_ref;
    bool ret_val = true;

    /* Get current state */
#ifdef FASTDIV
    current_ref = fd_fast_div(&(phandle->fd), phandle->ext, (int32_t)(phandle->scaling_factor));
#else
    current_ref = phandle->ext / (int32_t)(phandle->scaling_factor);
#endif

    if (durationms == 0u) {
        phandle->scaling_factor = get_scaling_factor(target_final);
        phandle->ext = target_final * (int32_t)(phandle->scaling_factor);
        phandle->ramp_remaining_step = 0u;
        phandle->inc_dec_amount = 0;
    } else {
        uint32_t wscaling_factor = get_scaling_factor(target_final - current_ref);
        uint32_t wscaling_factor2 = get_scaling_factor(current_ref);
        uint32_t wscaling_factor3 = get_scaling_factor(target_final);
        uint32_t wscaling_factorMin;

        if (wscaling_factor < wscaling_factor2) {
            if (wscaling_factor < wscaling_factor3) {
                wscaling_factorMin = wscaling_factor;
            } else {
                wscaling_factorMin = wscaling_factor3;
            }
        } else {
            if (wscaling_factor2 < wscaling_factor3) {
                wscaling_factorMin = wscaling_factor2;
            } else {
                wscaling_factorMin = wscaling_factor3;
            }
        }

        phandle->scaling_factor = wscaling_factorMin;
        phandle->ext = current_ref * (int32_t)(phandle->scaling_factor);

        /* Store the target_final to be applied in the last step */
        phandle->target_final = target_final;

        /* Compute the (wramp_remaining_step) number of steps remaining to complete
    the ramp. */
        aux = durationms * (uint32_t)phandle->frequency_hz; /* Check for overflow and use prescaler */
        aux /= 1000u;
        phandle->ramp_remaining_step = aux;
        phandle->ramp_remaining_step++;

        /* Compute the increment/decrement amount (winc_dec_amount) to be applied to
    the reference value at each CalcTorqueReference. */
        aux1 = (target_final - current_ref) * (int32_t)(phandle->scaling_factor);
        aux1 /= (int32_t)(phandle->ramp_remaining_step);
        phandle->inc_dec_amount = aux1;
    }

    return ret_val;
}

/**
 * @brief  Returns the current value of the state variable.
 * @param  phandle related Handle of struct RampMngr_Handle_t
 * @retval int32_t value of the state variable
 */
__weak int32_t remng_get_value(ramp_ext_mngr_t* phandle)
{
    int32_t ret_val;
    ret_val = phandle->ext / (int32_t)(phandle->scaling_factor);
    return ret_val;
}

/**
 * @brief  Check if the settled ramp has been completed.
 * @param  phandle related Handle of struct RampMngr_Handle_t.
 * @retval bool It returns true if the ramp is completed, false otherwise.
 */
__weak bool remng_ramp_completed(ramp_ext_mngr_t* phandle)
{
    bool ret_val = false;
    if (phandle->ramp_remaining_step == 0u) {
        ret_val = true;
    }
    return ret_val;
}

/**
 * @brief  Stop the execution of the ramp keeping the last reached value.
 * @param  phandle related Handle of struct RampMngr_Handle_t.
 * @retval none
 */
__weak void remng_stop_ramp(ramp_ext_mngr_t* phandle)
{
    phandle->ramp_remaining_step = 0u;
    phandle->inc_dec_amount = 0;
}

/**
 * @brief  Calculating the scaling factor to maximixe the resolution. It
 *         perform the 2^int(31-log2(Target)) with an iterative approach.
 *         It allows to keep Target * Scaling factor inside int32_t type.
 * @param  Target Input data.
 * @retval uint32_t It returns the optimized scaling factor.
 */
__weak uint32_t get_scaling_factor(int32_t target)
{
    uint8_t i;
    uint32_t target_abs;
    int32_t aux;

    if (target < 0) {
        aux = -target;
        target_abs = (uint32_t)(aux);
    } else {
        target_abs = (uint32_t)(target);
    }
    for (i = 1u; i < 32u; i++) {
        uint32_t limit = ((uint32_t)(1) << (31u - i));
        if (target_abs >= limit) {
            break;
        }
    }
    return ((uint32_t)(1u) << (i - 1u));
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
