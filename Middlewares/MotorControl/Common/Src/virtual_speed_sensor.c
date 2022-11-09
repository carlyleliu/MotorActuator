/**
 ******************************************************************************
 * @file    virtual_speed_sensor.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the Virtual Speed Sensor component of the motor Control SDK.
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
#include "virtual_speed_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/** @defgroup VirtualSpeedSensor Virtual Speed & Position Feedback
 * @brief Virtual Speed Speed & Position Feedback implementation
 *
 * This component provides a "virtual" implementation of the speed and position feedback features.
 * This implementation provides a theoretical estimation of the speed and position of the rotor of
 * the motor based on a mechanical acceleration and an initial angle set by the application.
 *
 * This component is used during the revup phases of the motor in a sensorless subsystem.
 *
 * @todo Document the Virtual Speed Sensor "module".
 *
 * @{
 */

/**
 * @brief  Software initialization of VirtualSpeedSensor component
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @retval none
 */
__weak void vss_init(virtual_speed_sensor_t* phandle)
{
#ifdef FASTDIV
    fd_init(&(phandle->fd));
#endif

    vss_clear(phandle);
}

/**
 * @brief  Software initialization of VSS object to be performed at each restart
 *         of the motor.
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @retval none
 */
__weak void vss_clear(virtual_speed_sensor_t* phandle)
{
    phandle->_Super.speed_error_number = 0u;
    phandle->_Super.el_angle = 0;
    phandle->_Super.mec_angle = 0;
    phandle->_Super.avr_mecspeed_unit = 0;
    phandle->_Super.el_speed_dpp = 0;
    phandle->_Super.mec_accel_unit_p = 0;
    phandle->_Super.speed_error_number = 0u;

    phandle->el_acc_dpp_p32 = 0;
    phandle->el_speed_dpp32 = 0;
    phandle->remaining_step = 0u;
    phandle->el_angle_accu = 0;

    phandle->transition_started = false;
    phandle->transition_ended = false;
    phandle->transition_remaining_steps = phandle->transition_steps;
    phandle->transition_locked = false;

    phandle->copy_observer = false;

#ifdef FASTDIV
    /* (Fast division optimization for cortex-M0 micros)*/
    /* Dummy division to speed up next executions */
    fd_fast_div(&(phandle->fd), 1, (int32_t)(phandle->_Super.el_to_mec_ratio));
    fd_fast_div(&(phandle->fd), 1, (int32_t)(phandle->transition_steps));
#endif
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Update the rotor electrical angle integrating the last setled
 *         instantaneous electrical speed express in dpp.
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @retval int16_t Measured electrical angle in s16degree format.
 */
__weak int16_t vss_calc_el_angle(virtual_speed_sensor_t* phandle, void* ptr_input_vars_str)
{
    int16_t ret_angle;
    int16_t angle_diff;
    int16_t angle_corr;
    int32_t aux;
    int16_t sign_corr = 1;

    if (phandle->copy_observer == true) {
        ret_angle = *(int16_t*)ptr_input_vars_str;
    } else {
        phandle->el_angle_accu += phandle->_Super.el_speed_dpp;

#ifdef FASTDIV
        phandle->_Super.mec_angle +=
        (int16_t)(fd_fast_div(&(phandle->fd), (int32_t)phandle->_Super.el_speed_dpp, (int32_t)phandle->_Super.el_to_mec_ratio));
#else
        phandle->_Super.mec_angle += phandle->_Super.el_speed_dpp / (int16_t)phandle->_Super.el_to_mec_ratio;
#endif

        if (phandle->transition_started == true) {
            if (phandle->transition_remaining_steps == 0) {
                ret_angle = *(int16_t*)ptr_input_vars_str;
                phandle->transition_ended = true;
                phandle->_Super.speed_error_number = 0u;
            } else {
                phandle->transition_remaining_steps--;

                if (phandle->_Super.el_speed_dpp >= 0) {
                    angle_diff = *(int16_t*)ptr_input_vars_str - phandle->el_angle_accu;
                } else {
                    angle_diff = phandle->el_angle_accu - *(int16_t*)ptr_input_vars_str;
                    sign_corr = -1;
                }

                aux = (int32_t)angle_diff * phandle->transition_remaining_steps;

#ifdef FASTDIV
                angle_corr = (int16_t)(fd_fast_div(&(phandle->fd), aux, (int32_t)(phandle->transition_steps)));
#else
                angle_corr = (int16_t)(aux / phandle->transition_steps);
#endif

                angle_corr *= sign_corr;

                if (angle_diff >= 0) {
                    phandle->transition_locked = true;
                    ret_angle = *(int16_t*)ptr_input_vars_str - angle_corr;
                } else {
                    if (phandle->transition_locked == false) {
                        ret_angle = phandle->el_angle_accu;
                    } else {
                        ret_angle = *(int16_t*)ptr_input_vars_str + angle_corr;
                    }
                }
            }
        } else {
            ret_angle = phandle->el_angle_accu;
        }
    }

    phandle->_Super.el_angle = ret_angle;
    return ret_angle;
}

/**
 * @brief  This method must be called with the same periodicity
 *         on which speed control is executed.
 *         This method computes and stores rotor instantaneous el speed (express
 *         in dpp considering the measurement frequency) in order to provide it
 *         to SPD_CalcElAngle function and spd_get_el_angle.
 *         Then compute store and return - through parameter
 *         mecspeed_unit - the rotor average mech speed, expressed in the unit
 *         defined by #SPEED_UNIT. Then return the reliability state of the
 *         sensor (always true).
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @param  mecspeed_unit pointer to int16_t, used to return the rotor average
 *         mechanical speed (SPED_UNIT)
 * @retval true = sensor information is reliable
 *         false = sensor information is not reliable
 */
__weak bool vss_calc_avrg_mec_speed_unit(virtual_speed_sensor_t* phandle, int16_t* mecspeed_unit)
{
    bool speed_sensor_reliability = false;

    if (phandle->remaining_step > 1u) {
        phandle->el_speed_dpp32 += phandle->el_acc_dpp_p32;
        phandle->_Super.el_speed_dpp = (int16_t)(phandle->el_speed_dpp32 / 65536);

        /* Convert dpp into MecUnit */
        *mecspeed_unit =
        (int16_t)(((int32_t)phandle->_Super.el_speed_dpp * (int32_t)phandle->_Super.measurement_frequency * SPEED_UNIT)
                  / ((int32_t)phandle->_Super.dpp_conv_factor * (int32_t)phandle->_Super.el_to_mec_ratio));

        phandle->_Super.avr_mecspeed_unit = *mecspeed_unit;

        phandle->remaining_step--;
    } else if (phandle->remaining_step == 1u) {
        *mecspeed_unit = phandle->final_mec_speed_unit;

        phandle->_Super.avr_mecspeed_unit = *mecspeed_unit;

        phandle->_Super.el_speed_dpp = (int16_t)(((int32_t)(*mecspeed_unit) * (int32_t)(phandle->_Super.dpp_conv_factor))
                                                 / ((int32_t)SPEED_UNIT * (int32_t)phandle->_Super.measurement_frequency));

        phandle->_Super.el_speed_dpp *= (int16_t)(phandle->_Super.el_to_mec_ratio);

        phandle->remaining_step = 0u;
    } else {
        *mecspeed_unit = phandle->_Super.avr_mecspeed_unit;
    }
    /* If the transition is not done yet, we already know that speed is not reliable */
    if (phandle->transition_ended == false) {
        phandle->_Super.speed_error_number = phandle->_Super.maximum_speed_errors_number;
        speed_sensor_reliability = false;
    } else {
        speed_sensor_reliability = spd_is_mec_speed_reliable(&phandle->_Super, mecspeed_unit);
    }

    return (speed_sensor_reliability);
}

/**
 * @brief  It is used to set istantaneous information on VSS mechanical and
 *         electrical angle.
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @param  mec_angle istantaneous measure of rotor mechanical angle
 * @retval none
 */
__weak void vss_set_mec_angle(virtual_speed_sensor_t* phandle, int16_t mec_angle)
{
    phandle->el_angle_accu = mec_angle;
    phandle->_Super.mec_angle = phandle->el_angle_accu / (int16_t)phandle->_Super.el_to_mec_ratio;
    phandle->_Super.el_angle = mec_angle;
}

/**
  * @brief  Set the mechanical acceleration of virtual sensor. This acceleration
            is defined starting from current mechanical speed, final mechanical
            speed expressed in 0.1Hz and duration expressed in milliseconds.
  * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
  * @param  final_mec_speed_unit mechanical speed  assumed by
            the virtual sensor at the end of the duration. Expressed in the unit defined
            by #SPEED_UNIT.
  * @param  durationms Duration expressed in ms. It can be 0 to apply
            instantaneous the final speed.
  * @retval none
  */
__weak void vss_set_mec_acceleration(virtual_speed_sensor_t* phandle, int16_t final_mec_speed_unit, uint16_t durationms)
{
    uint16_t nbr_step;
    int16_t current_mec_speed_dpp;
    int32_t mec_acc_dpp_p32;
    int16_t final_mec_speed_dpp;

    if (phandle->transition_started == false) {
        if (durationms == 0u) {
            phandle->_Super.avr_mecspeed_unit = final_mec_speed_unit;

            phandle->_Super.el_speed_dpp =
            (int16_t)(((int32_t)(final_mec_speed_unit) * (int32_t)(phandle->_Super.dpp_conv_factor))
                      / ((int32_t)SPEED_UNIT * (int32_t)phandle->_Super.measurement_frequency));

            phandle->_Super.el_speed_dpp *= (int16_t)(phandle->_Super.el_to_mec_ratio);

            phandle->remaining_step = 0u;

            phandle->final_mec_speed_unit = final_mec_speed_unit;
        } else {
            nbr_step = (uint16_t)(((uint32_t)durationms * (uint32_t)phandle->speed_sampling_freq_hz) / 1000u);

            nbr_step++;

            phandle->remaining_step = nbr_step;

            current_mec_speed_dpp = phandle->_Super.el_speed_dpp / (int16_t)phandle->_Super.el_to_mec_ratio;

            final_mec_speed_dpp = (int16_t)(((int32_t)final_mec_speed_unit * (int32_t)(phandle->_Super.dpp_conv_factor))
                                            / ((int32_t)SPEED_UNIT * (int32_t)phandle->_Super.measurement_frequency));

            mec_acc_dpp_p32 = (((int32_t)final_mec_speed_dpp - (int32_t)current_mec_speed_dpp) * (int32_t)65536) / (int32_t)nbr_step;

            phandle->el_acc_dpp_p32 = mec_acc_dpp_p32 * (int16_t)phandle->_Super.el_to_mec_ratio;

            phandle->final_mec_speed_unit = final_mec_speed_unit;

            phandle->el_speed_dpp32 = (int32_t)phandle->_Super.el_speed_dpp * (int32_t)65536;
        }
    }
}

/**
 * @brief  Checks if the ramp executed after a VSPD_SetMecacceleration command
 *         has been completed.
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @retval bool true if the ramp is completed, otherwise false.
 */
__weak bool vss_ramp_completed(virtual_speed_sensor_t* phandle)
{
    bool ret_val = false;
    if (phandle->remaining_step == 0u) {
        ret_val = true;
    }
    return ret_val;
}

/**
  * @brief  Get the final speed of last setled ramp of virtual sensor expressed
            in 0.1Hz.
  * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
  * @retval none
  */
__weak int16_t vss_get_last_ramp_final_speed(virtual_speed_sensor_t* phandle)
{
    return phandle->final_mec_speed_unit;
}

/**
  * @brief  Set the command to Start the transition phase from VirtualSpeedSensor
            to other SpeedSensor.
            Transition is to be considered ended when Sensor information is
            declared 'Reliable' or if function returned value is false
  * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
  * @param  bool true to Start the transition phase, false has no effect
  * @retval bool true if Transition phase is enabled (started or not), false if
            transition has been triggered but it's actually disabled
            (parameter transition_steps = 0)
  */
__weak bool vss_set_start_transition(virtual_speed_sensor_t* phandle, bool command)
{
    bool aux = true;
    if (command == true) {
        phandle->transition_started = true;

        if (phandle->transition_steps == 0) {
            phandle->transition_ended = true;
            phandle->_Super.speed_error_number = 0u;
            aux = false;
        }
    }
    return aux;
}

/**
 * @brief  Return the status of the transition phase.
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @retval bool true if Transition phase is ongoing, false otherwise.
 */
__weak bool vss_is_transition_ongoing(virtual_speed_sensor_t* phandle)
{
    uint16_t ts = 0u, te = 0u, aux;
    bool ret_val = false;
    if (phandle->transition_started == true) {
        ts = 1u;
    }
    if (phandle->transition_ended == true) {
        te = 1u;
    }
    aux = ts ^ te;
    if (aux != 0u) {
        ret_val = true;
    }
    return (ret_val);
}

__weak bool vss_transition_ended(virtual_speed_sensor_t* phandle)
{
    return phandle->transition_ended;
}

/**
 * @brief  It set istantaneous information on rotor electrical angle cptr_pied by state observer;
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @retval none
 */
__weak void vss_set_copy_observer(virtual_speed_sensor_t* phandle)
{
    phandle->copy_observer = true;
}

/**
 * @brief  It  set istantaneous information on rotor electrical angle.
 * @param  phandle: handler of the current instance of the VirtualSpeedSensor component
 * @param  el_angle istantaneous measure of rotor electrical angle (s16degrees)
 * @retval none
 */
__weak void vss_set_el_angle(virtual_speed_sensor_t* phandle, int16_t el_angle)
{
    phandle->el_angle_accu = el_angle;
    phandle->_Super.el_angle = el_angle;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
