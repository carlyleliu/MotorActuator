/**
 ******************************************************************************
 * @file    speed_torq_ctrl.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the Speed & Torque Control component of the motor Control SDK.
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
#include "speed_torq_ctrl.h"

#include "mc_type.h"
#include "speed_pos_fdbk.h"

#define CHECK_BOUNDARY

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup SpeednTorqCtrl Speed & Torque Control
 * @brief Speed & Torque Control component of the motor Control SDK
 *
 * @todo Document the Speed & Torque Control "module".
 *
 * @{
 */

/**
 * @brief  Initializes all the object variables, usually it has to be called
 *         once right after object creation.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @param  ptr_pi the PI object used as controller for the speed regulation.
 *         It can be equal to MC_NULL if the STC is initialized in torque mode
 *         and it will never be configured in speed mode.
 * @param  ptr_spd the speed sensor used to perform the speed regulation.
 *         It can be equal to MC_NULL if the STC is used only in torque
 *         mode.
 * @retval none.
 */
__weak void stc_init(speedn_torq_ctrl_t* phandle, pid_float_t* ptr_pi, speedn_pos_fdbk_t* spd_handle)
{
    phandle->ptr_pi_speed = ptr_pi;
    phandle->ptr_spd = spd_handle;
    phandle->mode = phandle->mode_default;
    phandle->speed_ref_unit_ext = (int32_t)phandle->mec_speed_ref_unit_default * 65536;
    phandle->torque_ref = (int32_t)phandle->torque_ref_default * 65536;
    phandle->target_final = 0;
    phandle->ramp_remaining_step = 0u;
    phandle->inc_dec_amount = 0;
}

/**
 * @brief It sets in real time the speed sensor utilized by the STC.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @param spd_handle Speed sensor component to be set.
 * @retval none
 */
__weak void stc_set_speed_sensor(speedn_torq_ctrl_t* phandle, speedn_pos_fdbk_t* spd_handle)
{
    phandle->ptr_spd = spd_handle;
}

/**
 * @brief It returns the speed sensor utilized by the FOC.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval speedn_pos_fdbk_t speed sensor utilized by the FOC.
 */
__weak speedn_pos_fdbk_t* stc_get_speed_sensor(speedn_torq_ctrl_t* phandle)
{
    return (phandle->ptr_spd);
}

/**
  * @brief  It should be called before each motor restart. If STC is set in
            speed mode, this method resets the integral term of speed regulator.
  * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
  * @retval none.
  */
__weak void stc_clear(speedn_torq_ctrl_t* phandle)
{
    if (phandle->mode == STC_SPEED_MODE) {
        pid_float_set_integral_term(phandle->ptr_pi_speed, 0);
    }
}

/**
 * @brief  Get the current mechanical rotor speed reference expressed in tenths
 *         of HZ.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval int16_t current mechanical rotor speed reference expressed in tenths
 *         of HZ.
 */
__weak int16_t stc_get_mec_speed_ref_unit(speedn_torq_ctrl_t* phandle)
{
    return ((int16_t)(phandle->speed_ref_unit_ext / 65536));
}

/**
 * @brief  Get the current motor torque reference. This value represents
 *         actually the Iq current reference expressed in digit.
 *         To convert current expressed in digit to current expressed in Amps
 *         is possible to use the formula:
 *         Current(Amp) = [Current(digit) * Vdd micro] / [65536 * Rshunt * Aop]
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval int16_t current motor torque reference. This value represents
 *         actually the Iq current expressed in digit.
 */
__weak int16_t stc_get_torque_ref(speedn_torq_ctrl_t* phandle)
{
    return ((int16_t)(phandle->torque_ref / 65536));
}

/**
 * @brief  Set the modality of the speed and torque controller. Two modality
 *         are available Torque mode and Speed mode.
 *         In Torque mode is possible to set directly the motor torque
 *         reference or execute a motor torque ramp. This value represents
 *         actually the Iq current reference expressed in digit.
 *         In Speed mode is possible to set the mechanical rotor speed
 *         reference or execute a speed ramp. The required motor torque is
 *         automatically calculated by the STC.
 *         This command interrupts the execution of any previous ramp command
 *         maintaining the last value of Iq.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @param  mode modality of STC. It can be one of these two settings:
 *         STC_TORQUE_MODE to enable the Torque mode or STC_SPEED_MODE to
 *         enable the Speed mode.
 * @retval none
 */
__weak void stc_set_control_mode(speedn_torq_ctrl_t* phandle, stc_modality_t mode)
{
    phandle->mode = mode;
    phandle->ramp_remaining_step = 0u; /* Interrupts previous ramp. */
}

/**
 * @brief  Get the modality of the speed and torque controller.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval stc_modality_t It returns the modality of STC. It can be one of
 *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
 */
__weak stc_modality_t stc_get_control_mode(speedn_torq_ctrl_t* phandle)
{
    return phandle->mode;
}

/**
 * @brief  Starts the execution of a ramp using new target and duration. This
 *         command interrupts the execution of any previous ramp command.
 *         The generated ramp will be in the modality previously set by
 *         stc_set_control_mode method.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @param  target_final final value of command. This is different accordingly
 *         the STC modality.
 *         If STC is in Torque mode target_final is the value of motor torque
 *         reference at the end of the ramp. This value represents actually the
 *         Iq current expressed in digit.
 *         To convert current expressed in Amps to current expressed in digit
 *         is possible to use the formula:
 *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
 *         If STC is in Speed mode target_final is the value of mechanical
 *         rotor speed reference at the end of the ramp expressed in tenths of
 *         HZ.
 * @param  durationms the duration of the ramp expressed in milliseconds. It
 *         is possible to set 0 to perform an instantaneous change in the value.
 * @retval bool It return false if the absolute value of target_final is out of
 *         the boundary of the application (Above max application speed or max
 *         application torque or below min application speed depending on
 *         current modality of TSC) in this case the command is ignored and the
 *         previous ramp is not interrupted, otherwise it returns true.
 */
__weak bool stc_exec_ramp(speedn_torq_ctrl_t* phandle, int16_t target_final, uint32_t durationms)
{
    bool allowed_range = true;
    uint32_t aux;
    int32_t aux1;
    int16_t current_reference;

    /* Check if the target_final is out of the bound of application. */
    if (phandle->mode == STC_TORQUE_MODE) {
        current_reference = stc_get_torque_ref(phandle);
#ifdef CHECK_BOUNDARY
        if ((int32_t)target_final > (int32_t)phandle->max_positive_torque) {
            allowed_range = false;
        }
        if ((int32_t)target_final < (int32_t)phandle->min_negative_torque) {
            allowed_range = false;
        }
#endif
    } else {
        current_reference = (int16_t)(phandle->speed_ref_unit_ext / 65536);

#ifdef CHECK_BOUNDARY
        if ((int32_t)target_final > (int32_t)phandle->max_app_positive_mec_speed_unit) {
            allowed_range = false;
        } else if (target_final < phandle->min_app_negative_mec_speed_unit) {
            allowed_range = false;
        } else if ((int32_t)target_final < (int32_t)phandle->min_app_positive_mec_speed_unit) {
            if (target_final > phandle->max_app_negative_mec_speed_unit) {
                allowed_range = false;
            }
        } else {
        }
#endif
    }

    if (allowed_range == true) {
        /* Interrupts the execution of any previous ramp command */
        if (durationms == 0u) {
            if (phandle->mode == STC_SPEED_MODE) {
                phandle->speed_ref_unit_ext = (int32_t)target_final * 65536;
            } else {
                phandle->torque_ref = (int32_t)target_final * 65536;
            }
            phandle->ramp_remaining_step = 0u;
            phandle->inc_dec_amount = 0;
        } else {
            /* Store the target_final to be applied in the last step */
            phandle->target_final = target_final;

            /* Compute the (wramp_remaining_step) number of steps remaining to complete
                the ramp. */
            aux = (uint32_t)durationms * (uint32_t)phandle->stc_frequency_hz;
            aux /= 1000u;
            phandle->ramp_remaining_step = aux;
            phandle->ramp_remaining_step++;

            /* Compute the increment/decrement amount (winc_dec_amount) to be applied to
                the reference value at each Calctorque_reference. */
            aux1 = ((int32_t)target_final - (int32_t)current_reference) * 65536;
            aux1 /= (int32_t)phandle->ramp_remaining_step;
            phandle->inc_dec_amount = aux1;
        }
    }

    return allowed_range;
}

/**
 * @brief  This command interrupts the execution of any previous ramp command.
 *         If STC has been set in Torque mode the last value of Iq is
 *         maintained.
 *         If STC has been set in Speed mode the last value of mechanical
 *         rotor speed reference is maintained.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval none
 */
__weak void stc_stop_ramp(speedn_torq_ctrl_t* phandle)
{
    phandle->ramp_remaining_step = 0u;
    phandle->inc_dec_amount = 0;
}

/**
 * @brief  It is used to compute the new value of motor torque reference. It
 *         must be called at fixed time equal to hstc_frequency_hz. It is called
 *         passing as parameter the speed sensor used to perform the speed
 *         regulation.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval int16_t motor torque reference. This value represents actually the
 *         Iq current expressed in digit.
 *         To convert current expressed in Amps to current expressed in digit
 *         is possible to use the formula:
 *         Current(digit) = [Current(Amp) * 65536 * Rshunt * Aop]  /  Vdd micro
 */
__weak int16_t stc_calc_torque_reference(speedn_torq_ctrl_t* phandle)
{
    int32_t current_reference;
    int16_t torque_reference = 0;
    int16_t measured_speed;
    int16_t target_speed;
    int16_t error;

    if (phandle->mode == STC_TORQUE_MODE) {
        current_reference = phandle->torque_ref;
    } else {
        current_reference = phandle->speed_ref_unit_ext;
    }

    /* Update the speed reference or the torque reference according to the mode
     and terminates the ramp if needed. */
    if (phandle->ramp_remaining_step > 1u) {
        /* Increment/decrement the reference value. */
        current_reference += phandle->inc_dec_amount;

        /* Decrement the number of remaining steps */
        phandle->ramp_remaining_step--;
    } else if (phandle->ramp_remaining_step == 1u) {
        /* Set the backup value of target_final. */
        current_reference = (int32_t)phandle->target_final * 65536;
        phandle->ramp_remaining_step = 0u;
    } else {
        /* Do nothing. */
    }

    if (phandle->mode == STC_SPEED_MODE) {
        /* Run the speed control loop */
        /* Compute speed error */
        target_speed = (int16_t)(current_reference / 65536);
        measured_speed = spd_get_avrg_mecspeed_unit(phandle->ptr_spd);
        error = target_speed - measured_speed;
        torque_reference = pi_float_controller(phandle->ptr_pi_speed, (int32_t)error);

        phandle->speed_ref_unit_ext = current_reference;
        phandle->torque_ref = (int32_t)torque_reference * 65536;
    } else {
        phandle->torque_ref = current_reference;
        torque_reference = (int16_t)(current_reference / 65536);
    }

    return torque_reference;
}

/**
 * @brief  Get the Default mechanical rotor speed reference expressed in tenths
 *         of HZ.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval int16_t It returns the Default mechanical rotor speed. reference
 *         expressed in tenths of HZ.
 */
__weak int16_t stc_get_mec_speed_ref_unit_default(speedn_torq_ctrl_t* phandle)
{
    return phandle->mec_speed_ref_unit_default;
}

/**
 * @brief  Returns the Application maximum positive value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 */
__weak uint16_t stc_get_max_app_positive_mec_speed_unit(speedn_torq_ctrl_t* phandle)
{
    return phandle->max_app_positive_mec_speed_unit;
}

/**
 * @brief  Returns the Application minimum negative value of rotor speed. Expressed in the unit defined by #SPEED_UNIT.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 */
__weak int16_t stc_get_min_app_negative_mec_speed_unit(speedn_torq_ctrl_t* phandle)
{
    return phandle->min_app_negative_mec_speed_unit;
}

/**
 * @brief  Check if the settled speed or torque ramp has been completed.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval bool It returns true if the ramp is completed, false otherwise.
 */
__weak bool stc_ramp_completed(speedn_torq_ctrl_t* phandle)
{
    bool ret_val = false;
    if (phandle->ramp_remaining_step == 0u) {
        ret_val = true;
    }
    return ret_val;
}

/**
 * @brief  Stop the execution of speed ramp.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval bool It returns true if the command is executed, false otherwise.
 */
__weak bool stc_stop_speed_ramp(speedn_torq_ctrl_t* phandle)
{
    bool ret_val = false;
    if (phandle->mode == STC_SPEED_MODE) {
        phandle->ramp_remaining_step = 0u;
        ret_val = true;
    }
    return ret_val;
}

/**
 * @brief It returns the default values of iqd_ref.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval default values of iqd_ref.
 */
__weak qd_t stc_get_default_iqd_ref(speedn_torq_ctrl_t* phandle)
{
    qd_t iqd_ref_default;
    iqd_ref_default.q = phandle->torque_ref_default;
    iqd_ref_default.d = phandle->idref_default;
    return iqd_ref_default;
}

/**
  * @brief  Change the nominal current .
  * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
  * @param  nominal_current This value represents actually the maximum Iq current
            expressed in digit.
  * @retval none
  */
__weak void stc_set_nominal_current(speedn_torq_ctrl_t* phandle, uint16_t nominal_current)
{
    phandle->max_positive_torque = nominal_current;
    phandle->min_negative_torque = -nominal_current;
}

/**
 * @brief  Force the speed reference to the curren speed. It is used
 *         at the START_RUN state to initialize the speed reference.
 * @param  phandle: handler of the current instance of the SpeednTorqCtrl component
 * @retval none
 */
__weak void stc_force_speed_reference_to_current_speed(speedn_torq_ctrl_t* phandle)
{
    phandle->speed_ref_unit_ext = (int32_t)spd_get_avrg_mecspeed_unit(phandle->ptr_spd) * (int32_t)65536;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
