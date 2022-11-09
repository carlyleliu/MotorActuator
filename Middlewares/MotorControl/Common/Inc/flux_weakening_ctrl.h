/**
 ******************************************************************************
 * @file    flux_weakening_ctrl.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the FLUX WEAKENING
 *          CTRL component of the motor Control SDK.
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
 * @ingroup FluxWeakeningCtrl
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLUXWEAKENINGCTRL_H
#define __FLUXWEAKENINGCTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup FluxWeakeningCtrl
 * @{
 */

/**
 * @brief  Flux Weakening Control Component handle structure
 */
typedef struct {
    pid_integer_t* ptr_pid_flux_weakening; /**< PI object used for flux weakening */
    pid_integer_t* ptr_pid_speed;          /**< PI object used for speed control */
    uint16_t fw_v_ref;                     /**< Voltage reference, tenth of
                                            percentage points */
    qd_t av_volt_qd;                       /**< Average stator voltage in qd
                                            reference frame */
    int16_t av_volt_ampl;                  /**< Average stator voltage amplitude */
    int16_t id_ref_offset;                 /**< Id reference offset */
    uint16_t max_module;                   /**< Circle limitation maximum allowed module */

    uint16_t default_fw_v_ref;           /**< Default flux weakening voltage reference,
                                          tenth of percentage points*/
    int16_t demag_current;               /**< Demagnetization current in s16A:
                                          Current(Amp) = [Current(s16A) * Vdd micro]/
                                          [65536 * Rshunt * Aop] */
    int32_t nominal_sq_curr;             /**< Squared motor nominal current in (s16A)^2
                                          where: Current(Amp) = [Current(s16A) * Vdd micro]/
                                          [65536 * Rshunt * Aop] */
    uint16_t vqd_low_pass_filter_bw;     /**< Use this parameter to configure the vqd
                                          first order software filter bandwidth.
                                          vqd_low_pass_filter_bw = FOC_CurrController
                                          call rate [Hz]/ FilterBandwidth[Hz] in
                                          case FULL_MISRA_COMPLiaNCY is defined.
                                          On the contrary, if FULL_MISRA_COMPLiaNCY
                                          is not defined, vqd_low_pass_filter_bw is
                                          equal to log with base two of previous
                                          definition */
    uint16_t vqd_low_pass_filter_bw_log; /**< vqd_low_pass_filter_bw expressed as power of 2.
                                          E.g. if gain divisor is 512 the value
                                          must be 9 because 2^9 = 512 */
} flux_weakening_t;

/**
 * @}
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initializes all the object variables, usually it has to be called
 *         once right after object creation.
 * @param  phandle Flux weakening init strutcture.
 * @param  ptr_pid_peed Speed PID structure.
 * @param  ptr_pid_flux_weakening_handle FW PID structure.
 * @retval none.
 */
void flux_weakening_init(flux_weakening_t* phandle, pid_integer_t* ptr_pid_peed, pid_integer_t* ptr_pid_flux_weakening_handle);

/**
 * @brief  It should be called before each motor restart and clears the Flux
 *         weakening internal variables with the exception of the target
 *         voltage (fw_v_ref).
 * @param  phandle Flux weakening init strutcture.
 * @retval none
 */
void flux_weakening_clear(flux_weakening_t* phandle);

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
qd_t flux_weakening_calc_curr_ref(flux_weakening_t* phandle, qd_t iqd_ref);

/**
 * @brief  It low-pass filters both the vqd voltage components. Filter
 *         bandwidth depends on vqd_low_pass_filter_bw parameter
 * @param  phandle Flux weakening init strutcture.
 * @param  vqd Voltage componets to be averaged.
 * @retval none
 */
void flux_weakening_data_process(flux_weakening_t* phandle, qd_t vqd);

/**
 * @brief  Use this method to set a new value for the voltage reference used by
 *         flux weakening algorithm.
 * @param  phandle Flux weakening init strutcture.
 * @param  uint16_t New target voltage value, expressend in tenth of percentage
 *         points of available voltage.
 * @retval none
 */
void flux_weakening_set_vref(flux_weakening_t* phandle, uint16_t new_vref);

/**
 * @brief  It returns the present value of target voltage used by flux
 *         weakening algorihtm.
 * @param  phandle Flux weakening init strutcture.
 * @retval int16_t Present target voltage value expressed in tenth of
 *         percentage points of available voltage.
 */
uint16_t flux_weakening_get_vref(flux_weakening_t* phandle);

/**
 * @brief  It returns the present value of voltage actually used by flux
 *         weakening algorihtm.
 * @param  phandle Flux weakening init strutcture.
 * @retval int16_t Present averaged phase stator voltage value, expressed
 *         in s16V (0-to-peak), where
 *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
 */
int16_t flux_weakening_get_av_v_amplitude(flux_weakening_t* phandle);

/**
 * @brief  It returns the measure of present voltage actually used by flux
 *         weakening algorihtm as percentage of available voltage.
 * @param  phandle Flux weakening init strutcture.
 * @retval uint16_t Present averaged phase stator voltage value, expressed in
 *         tenth of percentage points of available voltage.
 */
uint16_t flux_weakening_get_av_v_percentage(flux_weakening_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __FLUXWEAKENINGCTRL_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
