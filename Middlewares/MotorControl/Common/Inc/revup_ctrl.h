/**
 ******************************************************************************
 * @file    revup_ctrl.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          RevUpCtrl component of the motor Control SDK.
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
 * @ingroup RevUpCtrl
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REVUP_CTRL_H
#define __REVUP_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pwm_curr_fdbk.h"
#include "speed_torq_ctrl.h"
#include "sto_speed_pos_fdbk.h"
#include "virtual_speed_sensor.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup RevUpCtrl
 * @{
 */

/* Exported constants --------------------------------------------------------*/
/**
 * @brief Maximum number of phases allowed for RevUp process.
 *
 */
#define RUC_MAX_PHASE_NUMBER 5u

/**
 * @brief revup_ctrl_phase_params_t structure used for phases definition
 *
 */
typedef struct {
    uint16_t durationms;          /**< Duration of the RevUp phase.
                                   This parameter is expressed in millisecond.*/
    int16_t final_mec_speed_unit; /**< Mechanical speed assumed by VSS at the end of
                                   the RevUp phase. Expressed in the unit defined
                                   by #SPEED_UNIT */
    int16_t final_torque;         /**< motor torque reference imposed by STC at the
                                   end of RevUp phase. This value represents
                                   actually the Iq current expressed in digit.*/
    void* ptr_next;               /**< Pointer on the next phase section to proceed
                                   This parameter is NULL for the last element.*/
} revup_ctrl_phase_params_t;

/**
 * @brief  Handle structure of the RevUpCtrl.
 *
 */

typedef struct {
    uint16_t ruc_frequency_hz;      /**< Frequency call to main RevUp procedure ruc_exec.
                                     This parameter is equal to speed loop frequency. */
    int16_t starting_mec_angle;     /**< Starting angle of programmed RevUp.*/
    uint16_t phase_remaining_ticks; /**< Number of clock events remaining to complete the phase. */
    int16_t direction;              /**< motor direction.This parameter can be any value -1 or +1 */

    revup_ctrl_phase_params_t* ptr_current_phase_params; /**< Pointer on the current RevUp phase processed. */

    revup_ctrl_phase_params_t params_data[RUC_MAX_PHASE_NUMBER]; /**< Start up Phases sequences used by RevUp controller.
                                                                  Up to five phases can be used for the start up. */

    uint8_t phase_nbr;                /**< Number of phases relative to the programmed RevUp sequence.
                                       This parameter can be any value from 1 to 5 */
    uint8_t first_acceleration_stage; /**< Indicate the phase to start the final acceleration.
                                        At start of this stage sensor-less algorithm cleared.*/
    uint16_t min_startup_valid_speed; /**< Minimum rotor speed required to validate the startup.
                                       This parameter is expressed in SPPED_UNIT */
    uint16_t min_startup_fly_speed;   /**< Minimum rotor speed required to validate the on the fly.
                                       This parameter is expressed in the unit defined by #SPEED_UNIT */
    int16_t otf_final_revup_current;  /**< Final targetted torque for OTF phase. */
    uint16_t otf_section1_duration;   /**< On-the-fly phase duration, millisecond.
                                       This parameter is expressed in millisecond.*/
    bool otf_startup_enabled;         /**< Flag for OTF feature activation.
                                       Feature disabled when set to false */
    uint8_t otf_rel_counter;          /**< Counts the number of reliability of state observer */
    bool otf_sclowside;               /**< Flag to indicate status of low side switchs.
                                       This parameter can be true when Low Sides switch is ON otherwise set to false. */
    bool entered_zone1;               /**< Flag to indicate that the minimum rotor speed has been reached. */
    uint8_t reset_pll_th;             /**< Threshold to reset PLL during OTF */
    uint8_t reset_pll_cnt;            /**< Counter to reset PLL during OTF when the threshold is reached. */
    uint8_t stage_cnt;                /**< Counter of executed phases.
                                       This parameter can be any value from 0 to 5 */

    revup_ctrl_phase_params_t otf_phase_params; /**< RevUp phase parameter of OTF feature.*speedn_torq_ctrl_t* ptr_stc; */
    /**< Speed and torque controller object used by RevUpCtrl.*/
    speedn_torq_ctrl_t* ptr_stc;     /**< Speed and torque controller object used by RevUpCtrl.*/
    virtual_speed_sensor_t* ptr_vss; /**< Virtual speed sensor object used by RevUpCtrl.*/
    sto_t* ptr_snsl;                 /**< STO sensor object used by OTF startup.*/
    pwmc_t* ptr_pwm;                 /**< PWM object used by OTF startup.*/

} rev_up_ctrl_t;

/* Exported functions ------------------------------------------------------- */

/*  Function used to initialize and configure the RevUpCtrl Component */
void ruc_init(rev_up_ctrl_t* phandle, speedn_torq_ctrl_t* ptr_stc, virtual_speed_sensor_t* ptr_vss, sto_t* ptr_snsl, pwmc_t* ptr_pwm);

/*  Function used to reset internal state of the RevUpCtrl Component to its default state */
void ruc_clear(rev_up_ctrl_t* phandle, int16_t motor_direction);

/* Main procedure which clock the RevUp controller. */
bool ruc_exec(rev_up_ctrl_t* phandle);

/* Main procedure which clock the RevUp controller with on-the-fly feature. */
bool ruc_otf_exec(rev_up_ctrl_t* phandle);

/* Return information about current state of programmer RevUp sequence. */
bool ruc_completed(rev_up_ctrl_t* phandle);

/* Function allow to stop the programmed RevUp at the current speed. */
void ruc_stop(rev_up_ctrl_t* phandle);

/* Check that final acceleration during RevUp phase is reached  */
bool ruc_first_acceleration_stage_reached(rev_up_ctrl_t* phandle);

/* Function used to set the duration of a specific phase. */
void ruc_set_phase_durationms(rev_up_ctrl_t* phandle, uint8_t bPhase, uint16_t durationms);

/* Function used to set the targeted motor speed at the end of a specific phase. */
void ruc_set_phase_final_mec_speed_unit(rev_up_ctrl_t* phandle, uint8_t bPhase, int16_t final_mec_speed_unit);

/* Function used to set the final motor torque targeted at the end of a specific phase. */
void ruc_set_phase_final_torque(rev_up_ctrl_t* phandle, uint8_t bPhase, int16_t final_torque);

/* Function used to read the duration of a specific RevUp phase */
uint16_t ruc_get_phase_durationms(rev_up_ctrl_t* phandle, uint8_t bPhase);

/* Function used to read the targeted mechanical speed of a specific RevUp phase */
int16_t ruc_get_phase_final_mec_speed_unit(rev_up_ctrl_t* phandle, uint8_t bPhase);

/* Function used to read the targeted torque of a specific RevUp phase */
int16_t ruc_get_phase_final_torque(rev_up_ctrl_t* phandle, uint8_t bPhase);

/* Function used to read the number of phase(s) used by RevUp procedure  */
uint8_t ruc_get_number_of_phases(rev_up_ctrl_t* phandle);

/* Return the state of low side switches. */
bool ruc_get_sclowside_otf_status(rev_up_ctrl_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __REVUP_CTRL_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
