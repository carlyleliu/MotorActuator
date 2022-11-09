/**
 ******************************************************************************
 * @file    speed_pos_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the  features
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
 */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup SpeednPosFdbk Speed & Position Feedback
 *
 * @brief Speed & Position Feedback components of the motor Control SDK
 *
 * These components provide the speed and the angular position of the rotor of a motor (both
 * electrical and mechanical).
 *
 * Several implementations of the Speed and Position Feedback feature are provided by the motor
 * to account for the specificities of the motor used on the application:
 *
 * - @ref hall_speed_pos_fdbk "Hall Speed & Position Feedback" for motors with Hall effect sensors
 * - @ref Encoder  "Encoder Speed & Position Feedback" for motors with a quadrature encoder
 * - two general purpose sensorless implementations are provided:
 *   @ref SpeednPosFdbk_STO "State Observer with PLL" and
 *   @ref STO_CORDIC_SpeednPosFdbk "State Observer with CORDIC"
 * - "High Frequency Injection" for anisotrptr_pic I-PMSM motors (Not included in this release).
 *
 * @{
 */

/**
 * @brief  It returns the last computed rotor electrical angle, expressed in
 *         s16degrees. 1 s16degree = 360ï¿½/65536
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @retval int16_t rotor electrical angle (s16degrees)
 */
__weak int16_t spd_get_el_angle(speedn_pos_fdbk_t* phandle)
{
    return (phandle->el_angle);
}

/**
 * @brief  It returns the last computed rotor mechanical angle, expressed in
 *         s16degrees. Mechanical angle frame is based on parameter el_to_mec_ratio
 *         and, if occasionally provided - through function SPD_SetMecAngle -
 *         of a measured mechanical angle, on information computed thereof.
 * @note   both Hall sensor and Sensor-less do not implement either
 *         mechanical angle computation or acceleration computation.
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @retval int16_t rotor mechanical angle (s16degrees)
 */
__weak int32_t spd_get_mec_angle(speedn_pos_fdbk_t* phandle)
{
    return (phandle->mec_angle_full);
}

/**
 * @brief  Returns the last computed average mechanical speed, expressed in
 *         the unit defined by #SPEED_UNIT.
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 */
__weak int16_t spd_get_avrg_mecspeed_unit(speedn_pos_fdbk_t* phandle)
{
    return (phandle->avr_mecspeed_unit);
}

/**
 * @brief  It returns the last computed electrical speed, expressed in Dpp.
 *         1 Dpp = 1 s16Degree/control Period. The control period is the period
 *         on which the rotor electrical angle is computed (through function
 *         SPD_CalcElectricalAngle).
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @retval int16_t rotor electrical speed (Dpp)
 */
__weak int16_t spd_get_el_speed_dpp(speedn_pos_fdbk_t* phandle)
{
    return (phandle->el_speed_dpp);
}

/**
 * @brief  It returns the last instantaneous computed electrical speed, expressed in Dpp.
 *         1 Dpp = 1 s16Degree/control Period. The control period is the period
 *         on which the rotor electrical angle is computed (through function
 *         SPD_CalcElectricalAngle).
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @retval int16_t rotor instantaneous electrical speed (Dpp)
 */
__weak int16_t spd_get_inst_el_speed_dpp(speedn_pos_fdbk_t* phandle)
{
    return (phandle->instantaneous_el_speed_dpp);
}

/**
 * @brief  It returns the result of the last reliability check performed.
 *         Reliability is measured with reference to parameters
 *         hMaxReliableElspeed_unit, hMinReliableElspeed_unit,
 *         maximum_speed_errors_number and/or specific parameters of the derived
 *         true = sensor information is reliable
 *         false = sensor information is not reliable
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @retval bool sensor reliability state
 */
__weak bool spd_check(speedn_pos_fdbk_t* phandle)
{
    bool speed_sensor_reliability = true;
    if (phandle->speed_error_number == phandle->maximum_speed_errors_number) {
        speed_sensor_reliability = false;
    }
    return (speed_sensor_reliability);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif

/**
 * @brief  This method must be called - at least - with the same periodicity
 *         on which speed control is executed. It computes and returns - through
 *         parameter ptr_mecspeed_unit - the rotor average mechanical speed,
 *         expressed in the unit defined by #SPEED_UNIT. It computes and returns
 *         the reliability state of the sensor; reliability is measured with
 *         reference to parameters hMaxReliableElspeed_unit, hMinReliableElspeed_unit,
 *         maximum_speed_errors_number and/or specific parameters of the derived
 *         true = sensor information is reliable
 *         false = sensor information is not reliable
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @param  ptr_mecspeed_unit pointer to int16_t, used to return the rotor average
 *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
 * @retval none
 */
__weak bool spd_is_mec_speed_reliable(speedn_pos_fdbk_t* phandle, int16_t* ptr_mecspeed_unit)
{
    bool speed_sensor_reliability = true;
    uint8_t speed_error_number;
    uint8_t maximum_speed_errors_number = phandle->maximum_speed_errors_number;

    bool speed_error = false;
    uint16_t abs_mecspeed_unit, abs_mec_accel_unit_p;
    int16_t aux;

    speed_error_number = phandle->speed_error_number;

    /* Compute absoulte value of mechanical speed */
    if (*ptr_mecspeed_unit < 0) {
        aux = -(*ptr_mecspeed_unit);
        abs_mecspeed_unit = (uint16_t)(aux);
    } else {
        abs_mecspeed_unit = (uint16_t)(*ptr_mecspeed_unit);
    }

    if (abs_mecspeed_unit > phandle->max_reliable_mecspeed_unit) {
        speed_error = true;
    }

    if (abs_mecspeed_unit < phandle->min_reliable_mecspeed_unit) {
        speed_error = true;
    }

    /* Compute absoulte value of mechanical acceleration */
    if (phandle->mec_accel_unit_p < 0) {
        aux = -(phandle->mec_accel_unit_p);
        abs_mec_accel_unit_p = (uint16_t)(aux);
    } else {
        abs_mec_accel_unit_p = (uint16_t)(phandle->mec_accel_unit_p);
    }

    if (abs_mec_accel_unit_p > phandle->max_reliable_mec_accel_unit_p) {
        speed_error = true;
    }

    if (speed_error == true) {
        if (speed_error_number < maximum_speed_errors_number) {
            speed_error_number++;
        }
    } else {
        if (speed_error_number < maximum_speed_errors_number) {
            speed_error_number = 0u;
        }
    }

    if (speed_error_number == maximum_speed_errors_number) {
        speed_sensor_reliability = false;
    }

    phandle->speed_error_number = speed_error_number;

    return (speed_sensor_reliability);
}

/**
 * @brief  This method returns the average mechanical rotor speed expressed in
 *         "S16Speed". It means that:\n
 *         - it is zero for zero speed,\n
 *         - it become INT16_MAX when the average mechanical speed is equal to
 *           max_reliable_mecspeed_unit,\n
 *         - it becomes -INT16_MAX when the average mechanical speed is equal to
 *         -max_reliable_mecspeed_unit.
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @retval int16_t The average mechanical rotor speed expressed in "S16Speed".
 */
__weak int16_t spd_get_s16_speed(speedn_pos_fdbk_t* phandle)
{
    int32_t aux = (int32_t)phandle->avr_mecspeed_unit;
    aux *= INT16_MAX;
    aux /= (int16_t)phandle->max_reliable_mecspeed_unit;
    return (int16_t)aux;
}

/**
 * @brief  This method returns the coefficient used to transform electrical to
 *         mechanical quantities and viceversa. It usually coincides with motor
 *         pole pairs number.
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @retval uint8_t The motor pole pairs number.
 */
__weak uint8_t spd_get_el_to_mec_ratio(speedn_pos_fdbk_t* phandle)
{
    return (phandle->el_to_mec_ratio);
}

/**
 * @brief  This method sets the coefficient used to transform electrical to
 *         mechanical quantities and viceversa. It usually coincides with motor
 *         pole pairs number.
 * @param  phandle: handler of the current instance of the SpeednPosFdbk component
 * @param  bPP The motor pole pairs number to be set.
 */
__weak void spd_set_el_to_mec_ratio(speedn_pos_fdbk_t* phandle, uint8_t bpp)
{
    phandle->el_to_mec_ratio = bpp;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
