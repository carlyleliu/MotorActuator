/**
 ******************************************************************************
 * @file    speed_pos_fdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides all definitions and functions prototypes
 *          of the Speed & Position Feedback component of the motor Control SDK.
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
 * @ingroup SpeednPosFdbk
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPEEDNPOSFDBK_H
#define __SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "stdint.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  SpeednPosFdbk  handle definition
 */
typedef struct {
    uint8_t speed_error_number;

    uint8_t el_to_mec_ratio; /*!< Coefficient used to transform electrical to
                              mechanical quantities and viceversa. It usually
                              coincides with motor pole pairs number*/
    uint8_t speed_unit;      /*!< The speed unit value is defined into mc_stm_types.h*/

    uint8_t maximum_speed_errors_number; /*!< Maximum value of not valid measurements
                                          before an error is reported.*/
    int16_t el_angle;

    int16_t mec_angle;

    int32_t mec_angle_full;

    int16_t avr_mecspeed_unit;

    int16_t el_speed_dpp;
    int16_t instantaneous_el_speed_dpp;

    int16_t mec_accel_unit_p;

    uint16_t max_reliable_mecspeed_unit;    /*!< Maximum value of measured mechanical speed that is
                                             considered to be valid. Expressed
                                             in the unit defined by #SPEED_UNIT.*/
    uint16_t min_reliable_mecspeed_unit;    /*!< Minimum value of measured mechanical speed that is
                                             considered to be valid. Expressed
                                             in the unit defined by #SPEED_UNIT.*/
    uint16_t max_reliable_mec_accel_unit_p; /*!< Maximum value of measured acceleration
                                             that is considered to be valid. Expressed in
                                             the unit defined by #SPEED_UNIT */
    uint16_t measurement_frequency;         /*!< Frequency at which the user will request
                                             a measurement of the rotor electrical angle. Expressed in PWM_FREQ_SCALING*Hz.
                                             It is also used to convert measured speed from the unit
                                             defined by #SPEED_UNIT to dpp and viceversa.*/
    uint32_t dpp_conv_factor;               /* (65536/PWM_FREQ_SCALING) */

} speedn_pos_fdbk_t;

/**
 * @brief input structure type definition for SPD_CalcAngle
 */
typedef struct {
    alphabeta_t valfa_beta;
    alphabeta_t ialfa_beta;
    uint16_t Vbus;
} observer_inputs_t;

int16_t spd_get_el_angle(speedn_pos_fdbk_t* phandle);

int32_t spd_get_mec_angle(speedn_pos_fdbk_t* phandle);

int16_t spd_get_avrg_mecspeed_unit(speedn_pos_fdbk_t* phandle);

int16_t spd_get_el_speed_dpp(speedn_pos_fdbk_t* phandle);

int16_t spd_get_inst_el_speed_dpp(speedn_pos_fdbk_t* phandle);

bool spd_check(speedn_pos_fdbk_t* phandle);

bool spd_is_mec_speed_reliable(speedn_pos_fdbk_t* phandle, int16_t* ptr_mecspeed_unit);

int16_t spd_get_s16_speed(speedn_pos_fdbk_t* phandle);

uint8_t spd_get_el_to_mec_ratio(speedn_pos_fdbk_t* phandle);

void spd_set_el_to_mec_ratio(speedn_pos_fdbk_t* phandle, uint8_t bpp);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __SPEEDNPOSFDBK_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
