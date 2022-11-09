/**
 ******************************************************************************
 * @file    flux_weakening_ctrl.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the Flux Weakening
 *          Control component of the motor Control SDK.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMicROELECTRONicS AND CONTRibUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTicULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMicROELECTRONicS OR CONTRibUTORS BE LiaBLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECiaL, EXEMPLARY, OR CONSEQUENTiaL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVicES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LiaBILITY, WHETHER IN CONTRACT, STRicT LiaBILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSibILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "flux_weakening_ctrl.h"

#include "mc_math.h"
#include "mc_type.h"
#include "pid_regulator.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup FluxWeakeningCtrl Flux Weakening Control
 * @brief Flux Weakening Control Component component of the motor Control SDK
 *
 * @todo Document the Flux Weakening Control "module".
 *
 * @{
 */

/**
 * @brief  Initializes all the object variables, usually it has to be called
 *         once right after object creation.
 * @param  phandle Flux weakening init strutcture.
 * @param  ptr_pid_peed Speed PID strutcture.
 * @param  PIDFluxWeakeningHandle FW PID strutcture.
 * @retval none.
 */
__weak void flux_weakening_init(flux_weakening_t* phandle, pid_integer_t* ptr_pid_peed, pid_integer_t* ptr_pid_flux_weakening_handle)
{
    phandle->fw_v_ref = phandle->default_fw_v_ref;

    phandle->ptr_pid_flux_weakening = ptr_pid_flux_weakening_handle;

    phandle->ptr_pid_speed = ptr_pid_peed;
}

/**
 * @brief  It should be called before each motor restart and clears the Flux
 *         weakening internal variables with the exception of the target
 *         voltage (fw_v_ref).
 * @param  phandle Flux weakening init strutcture.
 * @retval none
 */
__weak void flux_weakening_clear(flux_weakening_t* phandle)
{
    qd_t v_null = {(int16_t)0, (int16_t)0};

    pid_set_integral_term(phandle->ptr_pid_flux_weakening, (int32_t)0);
    phandle->av_volt_qd = v_null;
    phandle->av_volt_ampl = (int16_t)0;
    phandle->id_ref_offset = (int16_t)0;
}

/**
 * @brief  It computes iqd_ref according the flux weakening algorithm.  Inputs
 *         are the starting Iqref components.
 *         As soon as the speed increases beyond the nominal one, fluxweakening
 *         algorithm take place and handles Idref value. Finally, accordingly
 *         with new Idref, a new Iqref saturation value is also computed and
 *         put into speed PI.
 * @param  phandle Flux weakening init strutcture.
 * @param  iqd_ref The starting current components that have to be
 *         manipulated by the flux weakening algorithm.
 * @retval qd_t Computed iqd_ref.
 */
__weak qd_t flux_weakening_calc_curr_ref(flux_weakening_t* phandle, qd_t iqd_ref)
{
    int32_t id_ref, iq_sat_sq, iq_sat, aux1, aux2;
    uint32_t volt_limit_ref;
    int16_t id_fw;

    /* Computation of the Id contribution coming from flux weakening algorithm */
    volt_limit_ref = ((uint32_t)(phandle->fw_v_ref) * phandle->max_module) / 1000u;
    aux1 = (int32_t)(phandle->av_volt_qd.q) * phandle->av_volt_qd.q;
    aux2 = (int32_t)(phandle->av_volt_qd.d) * phandle->av_volt_qd.d;
    aux1 += aux2;

    aux1 = mcm_sqrt(aux1);
    phandle->av_volt_ampl = (int16_t)aux1;

    /* Just in case sqrt rounding exceeded INT16_MAX */
    if (aux1 > INT16_MAX) {
        aux1 = (int32_t)INT16_MAX;
    }

    id_fw = pi_controller(phandle->ptr_pid_flux_weakening, (int32_t)volt_limit_ref - aux1);

    /* If the Id coming from flux weakening algorithm (Id_fw) is positive, keep
     unchanged Idref, otherwise sum it to last Idref available when Id_fw was
     zero */
    if (id_fw >= (int16_t)0) {
        phandle->id_ref_offset = iqd_ref.d;
        id_ref = (int32_t)iqd_ref.d;
    } else {
        id_ref = (int32_t)phandle->id_ref_offset + id_fw;
    }

    /* Saturate new Idref to prevent the rotor from being demagnetized */
    if (id_ref < phandle->demag_current) {
        id_ref = phandle->demag_current;
    }

    iqd_ref.d = (int16_t)id_ref;

    /* New saturation for Iqref */
    iq_sat_sq = phandle->nominal_sq_curr - id_ref * id_ref;
    iq_sat = mcm_sqrt(iq_sat_sq);

    /* Iqref saturation value used for updating integral term limitations of
  speed PI */
    aux1 = iq_sat * (int32_t)pid_get_ki_divisor(phandle->ptr_pid_speed);

    pid_set_lower_integral_term_limit(phandle->ptr_pid_speed, -aux1);
    pid_set_upper_integral_term_limit(phandle->ptr_pid_speed, aux1);

    /* Iqref saturation value used for updating integral term limitations of
  speed PI */
    if (iqd_ref.q > iq_sat) {
        iqd_ref.q = (int16_t)iq_sat;
    } else if (iqd_ref.q < -iq_sat) {
        iqd_ref.q = -(int16_t)iq_sat;
    } else {
    }

    return (iqd_ref);
}

#if defined(__ICCARM__)
/* shift value is written during init and computed
     by MC Workbench */
#pragma cstat_disable = "ATH-shift-bounds"
#endif /* __ICCARM__ */

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It low-pass filters both the vqd voltage components. Filter
 *         bandwidth depends on vqd_low_pass_filter_bw parameter
 * @param  phandle Flux weakening init strutcture.
 * @param  vqd Voltage componets to be averaged.
 * @retval none
 */
__weak void flux_weakening_data_process(flux_weakening_t* phandle, qd_t vqd)
{
    int32_t aux;
    int32_t low_pass_filter_bw = (int32_t)(phandle->vqd_low_pass_filter_bw) - (int32_t)1;

#ifdef FULL_MISRA_C_COMPLIANCY
    aux = (int32_t)(phandle->av_volt_qd.q) * low_pass_filter_bw;
    aux += vqd.q;

    phandle->av_volt_qd.q = (int16_t)(aux / (int32_t)(phandle->vqd_low_pass_filter_bw));

    aux = (int32_t)(phandle->av_volt_qd.d) * low_pass_filter_bw;
    aux += vqd.d;

    phandle->av_volt_qd.d = (int16_t)(aux / (int32_t)phandle->vqd_low_pass_filter_bw);
#else
    aux = (int32_t)(phandle->av_volt_qd.q) * low_pass_filter_bw;
    aux += vqd.q;

    phandle->av_volt_qd.q = (int16_t)(aux >> phandle->vqd_low_pass_filter_bw_log);

    aux = (int32_t)(phandle->av_volt_qd.d) * low_pass_filter_bw;
    aux += vqd.d;
    phandle->av_volt_qd.d = (int16_t)(aux >> phandle->vqd_low_pass_filter_bw_log);

#endif
    return;
}

#if defined(__ICCARM__)
/* shift value is written during init and computed
     by MC Workbench */
#pragma cstat_enable = "ATH-shift-bounds"
#endif /* __ICCARM__ */

/**
 * @brief  Use this method to set a new value for the voltage reference used by
 *         flux weakening algorithm.
 * @param  phandle Flux weakening init strutcture.
 * @param  uint16_t New target voltage value, expressend in tenth of percentage
 *         points of available voltage.
 * @retval none
 */
__weak void flux_weakening_set_vref(flux_weakening_t* phandle, uint16_t new_vref)
{
    phandle->fw_v_ref = new_vref;
}

/**
 * @brief  It returns the present value of target voltage used by flux
 *         weakening algorihtm.
 * @param  phandle Flux weakening init strutcture.
 * @retval int16_t Present target voltage value expressed in tenth of
 *         percentage points of available voltage.
 */
__weak uint16_t flux_weakening_get_vref(flux_weakening_t* phandle)
{
    return (phandle->fw_v_ref);
}

/**
 * @brief  It returns the present value of voltage actually used by flux
 *         weakening algorihtm.
 * @param  phandle Flux weakening init strutcture.
 * @retval int16_t Present averaged phase stator voltage value, expressed
 *         in s16V (0-to-peak), where
 *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
 */
__weak int16_t flux_weakening_get_av_v_amplitude(flux_weakening_t* phandle)
{
    return (phandle->av_volt_ampl);
}

/**
 * @brief  It returns the measure of present voltage actually used by flux
 *         weakening algorihtm as percentage of available voltage.
 * @param  phandle Flux weakening init strutcture.
 * @retval uint16_t Present averaged phase stator voltage value, expressed in
 *         tenth of percentage points of available voltage.
 */
__weak uint16_t flux_weakening_get_av_v_percentage(flux_weakening_t* phandle)
{
    return (uint16_t)((uint32_t)(phandle->av_volt_ampl) * 1000u / (uint32_t)(phandle->max_module));
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
