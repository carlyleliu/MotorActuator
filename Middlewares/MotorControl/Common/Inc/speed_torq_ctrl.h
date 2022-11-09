/**
 ******************************************************************************
 * @file    speed_torq_ctrl.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Speed & Torque Control component of the motor Control SDK.
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
 * @ingroup SpeednTorqCtrl
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPEEDNTORQCTRLCLASS_H
#define __SPEEDNTORQCTRLCLASS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
#include "pid_regulator_float.h"
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednTorqCtrl
 * @{
 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  Speed & Torque Control parameters definition
 */
typedef struct {
    stc_modality_t mode;          /*!< Modality of STC. It can be one of these two
                                   settings: STC_TORQUE_MODE to enable the
                                   Torque mode or STC_SPEED_MODE to enable the
                                   Speed mode.*/
    int16_t target_final;         /*!< Backup of htarget_final to be applied in the
                             l     ast step.*/
    int32_t speed_ref_unit_ext;   /*!< Current mechanical rotor speed reference
                                   expressed in tenths of HZ multiplied by
                                   65536.*/
    int32_t torque_ref;           /*!< Current motor torque reference. This value
                                   represents actually the Iq current
                                   expressed in digit multiplied by 65536.*/
    uint32_t ramp_remaining_step; /*!< Number of steps remaining to complete the ramp.*/
    pid_float_t* ptr_pi_speed;    /*!< The regulator used to perform the speed control loop.*/
    speedn_pos_fdbk_t* ptr_spd;   /*!< The speed sensor used to perform the speed regulation.*/
    int32_t inc_dec_amount;       /*!< Increment/decrement amount to be applied to
                                   the reference value at each Calctorque_reference.*/

    uint16_t stc_frequency_hz;                /*!< Frequency on which the user updates
                                               the torque reference calling
                                               stc_calc_torque_reference method
                                               expressed in Hz */
    uint16_t max_app_positive_mec_speed_unit; /*!< Application maximum positive value
                                               of the rotor mechanical speed. Expressed in
                                               the unit defined by #SPEED_UNIT.*/
    uint16_t min_app_positive_mec_speed_unit; /*!< Application minimum positive value
                                               of the rotor mechanical speed. Expressed in
                                               the unit defined by #SPEED_UNIT.*/
    int16_t max_app_negative_mec_speed_unit;  /*!< Application maximum negative value
                                               of the rotor mechanical speed. Expressed in
                                               the unit defined by #SPEED_UNIT.*/
    int16_t min_app_negative_mec_speed_unit;  /*!< Application minimum negative value
                                               of the rotor mechanical speed. Expressed in
                                               the unit defined by #SPEED_UNIT.*/
    uint16_t max_positive_torque;             /*!< Maximum positive value of motor
                                               torque. This value represents
                                               actually the maximum Iq current
                                               expressed in digit.*/
    int16_t min_negative_torque;              /*!< Minimum negative value of motor
                                               torque. This value represents
                                               actually the maximum Iq current
                                               expressed in digit.*/
    stc_modality_t mode_default;              /*!< Default STC modality.*/
    int16_t mec_speed_ref_unit_default;       /*!< Default mechanical rotor speed
                                               reference expressed in the unit
                                               defined by #SPEED_UNIT.*/
    int16_t torque_ref_default;               /*!< Default motor torque reference.
                                               This value represents actually the
                                               Iq current reference expressed in
                                               digit.*/
    int16_t idref_default;                    /*!< Default Id current reference expressed
                                               in digit.*/
} speedn_torq_ctrl_t;

/* It initializes all the object variables */
void stc_init(speedn_torq_ctrl_t* phandle, pid_float_t* ptr_pi, speedn_pos_fdbk_t* ptr_spd);

/* It resets the integral term of speed regulator */
void stc_clear(speedn_torq_ctrl_t* phandle);

/* Get the current mechanical rotor speed reference expressed in tenths of HZ.*/
int16_t stc_get_mec_speed_ref_unit(speedn_torq_ctrl_t* phandle);

/*  Get the current motor torque reference. */
int16_t stc_get_torque_ref(speedn_torq_ctrl_t* phandle);

/* Set the mode of the speed and torque controller (Torque mode or Speed mode)*/
void stc_set_control_mode(speedn_torq_ctrl_t* phandle, stc_modality_t mode);

/* Get the mode of the speed and torque controller. */
stc_modality_t stc_get_control_mode(speedn_torq_ctrl_t* phandle);

/* Starts the execution of a ramp using new target and duration. */
bool stc_exec_ramp(speedn_torq_ctrl_t* phandle, int16_t htarget_final, uint32_t durationms);

/* It interrupts the execution of any previous ramp command.*/
void stc_stop_ramp(speedn_torq_ctrl_t* phandle);

/* It computes the new value of motor torque reference */
int16_t stc_calc_torque_reference(speedn_torq_ctrl_t* phandle);

/* Get the Default mechanical rotor speed reference expressed in tenths of HZ.*/
int16_t stc_get_mec_speed_ref_unit_default(speedn_torq_ctrl_t* phandle);

/* Returns the Application maximum positive rotor mechanical speed. Expressed in the unit defined by SPEED_UNIT.*/
uint16_t stc_get_max_app_positive_mec_speed_unit(speedn_torq_ctrl_t* phandle);

/* Returns the Application minimum negative rotor mechanical speed. Expressed in the unit defined by SPEED_UNIT.*/
int16_t stc_get_min_app_negative_mec_speed_unit(speedn_torq_ctrl_t* phandle);

/* Check if the settled speed or torque ramp has been completed.*/
bool stc_ramp_completed(speedn_torq_ctrl_t* phandle);

/* Stop the execution of speed ramp. */
bool stc_stop_speed_ramp(speedn_torq_ctrl_t* phandle);

/* It sets in real time the speed sensor utilized by the FOC. */
void stc_set_speed_sensor(speedn_torq_ctrl_t* phandle, speedn_pos_fdbk_t* ptr_spd);

/* It returns the speed sensor utilized by the FOC. */
speedn_pos_fdbk_t* stc_get_speed_sensor(speedn_torq_ctrl_t* phandle);

/* It returns the default values of iqd_ref. */
qd_t stc_get_default_iqd_ref(speedn_torq_ctrl_t* phandle);

/* It sets the nominal current */
void stc_set_nominal_current(speedn_torq_ctrl_t* phandle, uint16_t nominal_current);

/* Force the speed reference to the current speed */
void stc_force_speed_reference_to_current_speed(speedn_torq_ctrl_t* phandle);

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SPEEDNTORQCTRLCLASS_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
