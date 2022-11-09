/**
 ******************************************************************************
 * @file    virtual_speed_sensor.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Virtual Speed Sensor component of the motor Control SDK.
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
 * @ingroup VirtualSpeedSensor
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _VIRTUALSPEEDSENSOR_H
#define _VIRTUALSPEEDSENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#ifdef FASTDIV
#include "fast_div.h"
#endif
/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/** @addtogroup VirtualSpeedSensor
 * @{
 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  This structure is used to handle an instance of the Virtual Speed
 *         sensor component
 */
typedef struct {
    speedn_pos_fdbk_t _Super;
    int32_t el_acc_dpp_p32;             /*!< Delta electrical speed expressed in dpp per speed
                                         sampling period to be appied each time is called
                                         SPD_CalcAvrgMecspeed_unit multiplied by scaling
                                         factor of 65536.*/
    int32_t el_speed_dpp32;             /*!< Electrical speed expressed in dpp multiplied by
                                         scaling factor 65536.*/
    uint16_t remaining_step;            /*!< Number of steps remaining to reach the final
                                         speed.*/
    int16_t final_mec_speed_unit;       /*!< Backup of final_mec_speed_unit to be applied in
                                         the last step.*/
    bool transition_started;            /*!< Retaining information about Transition status.*/
    bool transition_ended;              /*!< Retaining information about ransition status.*/
    int16_t transition_remaining_steps; /*!< Number of steps remaining to end
                                         transition from CVSS_SPD to other CSPD*/
    int16_t el_angle_accu;              /*!< Electrical angle accumulator*/
    bool transition_locked;             /*!< Transition acceleration started*/
    bool copy_observer;                 /*!< Command to set VSPD output same as state observer*/

    uint16_t speed_sampling_freq_hz; /*!< Frequency (Hz) at which motor speed is to
                                      be computed. It must be equal to the frequency
                                      at which function SPD_CalcAvrgMecspeed_unit
                                      is called.*/
    int16_t transition_steps;        /*< Number of steps to perform the transition phase
                                      from CVSS_SPD to other CSPD; if the Transition PHase
                                      should last TPH milliseconds, and the FOC Execution
                                      Frequency is set to FEF kHz, then
                                      transition_steps = TPH * FEF*/
#ifdef FASTDIV
    /* (Fast division optimization for cortex-M0 micros)*/
    fast_div_t fd; /*!< Fast division obj.*/
#endif

} virtual_speed_sensor_t;

/* It initializes the Virtual Speed Sensor component */
void vss_init(virtual_speed_sensor_t* phandle);

/* It clears Virtual Speed Sensor by re-initializing private variables*/
void vss_clear(virtual_speed_sensor_t* phandle);

/* It compute a theorical speed and update the theorical electrical angle. */
int16_t vss_calc_el_angle(virtual_speed_sensor_t* phandle, void* ptr_input_vars_str);

/* Computes the rotor average theoretical mechanical speed in the unit defined by SPEED_UNIT and returns it in ptr_mecspeed_unit. */
bool vss_calc_avrg_mec_speed_unit(virtual_speed_sensor_t* phandle, int16_t* mecspeed_unit);

/* It set istantaneous information on VSS mechanical and  electrical angle.*/
void vss_set_mec_angle(virtual_speed_sensor_t* phandle, int16_t mec_angle);

/* Set the mechanical acceleration of virtual sensor. */
void vss_set_mec_acceleration(virtual_speed_sensor_t* phandle, int16_t final_mec_speed_unit, uint16_t durationms);
/* Checks if the ramp executed after a VSPD_SetMecacceleration command has been completed*/
bool vss_ramp_completed(virtual_speed_sensor_t* phandle);

/* Get the final speed of last setled ramp of virtual sensor expressed in 0.1Hz*/
int16_t vss_get_last_ramp_final_speed(virtual_speed_sensor_t* phandle);

/* Set the command to Start the transition phase from VirtualSpeedSensor to other SpeedSensor.*/
bool vss_set_start_transition(virtual_speed_sensor_t* phandle, bool command);

/* Return the status of the transition phase.*/
bool vss_is_transition_ongoing(virtual_speed_sensor_t* phandle);

bool vss_transition_ended(virtual_speed_sensor_t* phandle);

/* It set istantaneous information on rotor electrical angle cptr_pied by state observer */
void vss_set_copy_observer(virtual_speed_sensor_t* phandle);

/* It  set istantaneous information on rotor electrical angle */
void vss_set_el_angle(virtual_speed_sensor_t* phandle, int16_t el_angle);

/** @} */
/** @} */
/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _VIRTUALSPEEDSENSOR_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
