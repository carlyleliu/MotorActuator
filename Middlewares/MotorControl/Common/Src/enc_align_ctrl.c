/**
 ******************************************************************************
 * @file    enc_align_ctrl.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the Encoder Alignment Control component of the motor Control SDK.
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
#include "enc_align_ctrl.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup EncAlignCtrl Encoder Alignment Controller
 * @brief Encoder Alignment Controller component of the motor Control SDK
 *
 * Initial encoder calibration which comprises a rotor alignment in a given position
 * necessary to make the information coming from a quadrature encoder absolute.
 * @{
 */

/**
 * @brief  It initializes the handle
 * @param  phandle: handler of the current instance of the EncAlignCtrl component.
 * @param  ptr_stc: the speed and torque controller used by the EAC.
 * @param  ptr_vss: the virtual speed sensor used by the EAC.
 * @param  ptr_enc: the encoder used by the EAC.
 * @retval none.
 */
__weak void eac_init(enc_align_t* phandle, speedn_torq_ctrl_t* ptr_stc, virtual_speed_sensor_t* ptr_vss, encoder_t* ptr_enc)
{
    phandle->ptr_stc = ptr_stc;
    phandle->ptr_vss = ptr_vss;
    phandle->ptr_enc = ptr_enc;
    phandle->enc_aligned = false;
    phandle->enc_restart = false;
}

/**
 * @brief  It start the encoder alignment procedure.
 *     It configures the VSS with the required angle and sets the STC to
 *         execute the required torque ramp.
 * @param  phandle: handler of the current instance of the EncAlignCtrl component.
 * @retval none.
 */
__weak void eac_start_alignment(enc_align_t* phandle)
{
    uint32_t aux;

    /* Set ptr_vss mechanical speed to zero.*/
    vss_set_mec_acceleration(phandle->ptr_vss, 0, 0u);

    /* Set ptr_vss mechanical angle.*/
    vss_set_mec_angle(phandle->ptr_vss, phandle->el_angle);

    /* Set ptr_stc in STC_TORQUE_MODE.*/
    stc_set_control_mode(phandle->ptr_stc, STC_TORQUE_MODE);

    /* Set starting torque to Zero */
    stc_exec_ramp(phandle->ptr_stc, 0, 0u);

    /* Execute the torque ramp.*/
    stc_exec_ramp(phandle->ptr_stc, phandle->final_torque, (uint32_t)(phandle->durationms));

    /* Compute remaining_ticks, the number of thick of alignment phase.*/
    aux = (uint32_t)phandle->durationms * (uint32_t)phandle->eac_frequency_hz;
    aux /= 1000u;
    phandle->remaining_ticks = (uint16_t)(aux);
    phandle->remaining_ticks++;
}

/**
 * @brief  It clocks the encoder alignment controller and must be called with a
 *         frequency equal to the one settled in the parameters
 *         eac_frequency_hz. Calling this method the EAC is possible to verify
 *         if the alignment duration has been finished. At the end of alignment
 *         the encoder is set to the defined mechanical angle.
 *         Note: STC, VSS, ENC are not clocked by eac_exec.
 * @param  phandle: handler of the current instance of the EncAlignCtrl component.
 * @retval bool It returns true when the programmed alignment has been
 *         completed.
 */
__weak bool eac_exec(enc_align_t* phandle)
{
    bool ret_val = true;

    if (phandle->remaining_ticks > 0u) {
        phandle->remaining_ticks--;

        if (phandle->remaining_ticks == 0u) {
            /* Set ptr_vss mechanical angle.*/
            enc_set_mec_angle(phandle->ptr_enc, phandle->el_angle / (int16_t)(phandle->el_to_mec_ratio));
            phandle->enc_aligned = true;
            ret_val = true;
        } else {
            ret_val = false;
        }
    }

    return ret_val;
}

/**
 * @brief  It returns true if the encoder has been aligned at least
 *         one time, false if hasn't been never aligned.
 * @param  phandle: handler of the current instance of the EncAlignCtrl component.
 */
__weak bool eac_aligned(enc_align_t* phandle)
{
    return phandle->enc_aligned;
}

/**
 * @brief  It sets a restart after an encoder alignment.
 * @param  phandle: handler of the current instance of the EncAlignCtrl component.
 * @param  restart: Set to true if a restart is programmed else false
 * @ret_val none.
 */
__weak void eac_set_restart_state(enc_align_t* phandle, bool restart)
{
    phandle->enc_restart = restart;
}

/**
 * @brief  Returns true if a restart after an encoder alignment has been requested.
 * @param  phandle: handler of the current instance of the EncAlignCtrl component.
 */
__weak bool eac_get_restart_state(enc_align_t* phandle)
{
    return phandle->enc_restart;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
