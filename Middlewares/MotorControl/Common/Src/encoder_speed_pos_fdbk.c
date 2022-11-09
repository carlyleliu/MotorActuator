/**
 ******************************************************************************
 * @file    encoder_speed_pos_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the Encoder component of the motor Control SDK:
 *
 *           - computes & stores average mechanical speed
 *           - computes & stores average mechanical acceleration
 *           - computes & stores  the instantaneous electrical speed
 *           - calculates the rotor electrical and mechanical angle
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
#include "encoder_speed_pos_fdbk.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/** @defgroup Encoder Encoder Speed & Position Feedback
 * @brief Quadrature Encoder based Speed & Position Feedback implementation
 *
 * This component is used in applications controlling a motor equipped with a quadrature encoder.
 *
 * This component uses the output of a quadrature encoder to provide a measure of the speed and
 * the position of the rotor of the motor.
 *
 * @todo Document the Encoder Speed & Position Feedback "module".
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVic)
            required for the speed position sensor management using ENCODER
            sensors.
  * @param  phandle: handler of the current instance of the encoder component
  * @retval none
  */
__weak void enc_init(encoder_t* phandle)
{
    uint8_t buffer_size;
    uint8_t index;

#define ENC_MAX_OVERFLOW_NB (1)

    if (NULL == phandle->timer_init) {
        return;
    }

    /* Init counter */
    phandle->timer_init();

    /*Calculations of convenience*/
    phandle->u32_maxdiv_pulse_number = UINT32_MAX / (uint32_t)(phandle->pulse_number);
    phandle->speed_sampling_freq_unit = phandle->speed_sampling_freq_hz * SPEED_UNIT;

    /* Erase speed buffer */
    buffer_size = phandle->speed_buffer_size;

    for (index = 0u; index < buffer_size; index++) {
        phandle->delta_captures_buffer[index] = 0;
    }
}

/**
 * @brief  Clear software FIFO where are "pushed" rotor angle variations captured
 *         This function must be called before starting the motor to initialize
 *         the speed measurement process.
 * @param  phandle: handler of the current instance of the encoder component
 * @retval none
 */
__weak void enc_clear(encoder_t* phandle)
{
    uint8_t index;
    for (index = 0u; index < phandle->speed_buffer_size; index++) {
        phandle->delta_captures_buffer[index] = 0;
    }
    phandle->sensor_reliable = true;
}

/**
 * @brief  It calculates the rotor electrical and mechanical angle, on the basis
 *         of the instantaneous value of the timer counter.
 * @param  phandle: handler of the current instance of the encoder component
 * @retval int16_t Measured electrical angle in s16degree format.
 */
__weak int16_t enc_calc_angle(encoder_t* phandle)
{
    int32_t temp1;
    int16_t el_angle;  /* s16degree format */
    int16_t mec_angle; /* s16degree format */

    if (NULL == phandle->timer_get_counter || NULL == phandle->timer_get_direction) {
        return -1;
    }
    /* PR 52926 We need to keep only the 16 LSB, bit 31 could be at 1
        if the overflow occurs just after the entry in the High frequency task */
    temp1 = (int32_t)(phandle->timer_get_counter(phandle->TIMx) & 0xffff) * (int32_t)(phandle->u32_maxdiv_pulse_number);

    /*Computes and stores the rotor mechanical angle*/
    mec_angle = (int16_t)(temp1 / 65536);

    int16_t mec_anglePrev = phandle->_Super.mec_angle;

    phandle->_Super.mec_angle = mec_angle;

    /*Computes and stores the rotor electrical angle*/
    el_angle = mec_angle * phandle->_Super.el_to_mec_ratio;

    phandle->_Super.el_angle = el_angle;

    int16_t hMecSpeedDpp = mec_angle - mec_anglePrev;
    phandle->_Super.mec_angle += (int32_t)(hMecSpeedDpp);

    /*Returns rotor electrical angle*/
    return (el_angle);
}

/**
 * @brief  This method must be called with the periodicity defined by parameter
 *         hSpeedSamplingFreqUnit. The method generates a capture event on
 *         a channel, computes & stores average mechanical speed [expressed in the unit
 *         defined by #SPEED_UNIT] (on the basis of the buffer filled by CCx IRQ),
 *         computes & stores average mechanical acceleration [#SPEED_UNIT/SpeedSamplingFreq],
 *         computes & stores the instantaneous electrical speed [dpp], updates the index of the
 *         speed buffer, then checks & stores & returns the reliability state
 *         of the sensor.
 * @param  phandle: handler of the current instance of the encoder component
 * @param  ptr_mecspeed_unit pointer used to return the rotor average mechanical speed
 *         (expressed in the unit defined by #SPEED_UNIT)
 * @retval true = sensor information is reliable
 *         false = sensor information is not reliable
 */
__weak bool enc_calc_avrg_mec_speed_unit(encoder_t* phandle, int16_t* ptr_mecspeed_unit)
{
    TIM_TypeDef* TIMx = phandle->TIMx;
    int32_t overall_angle_variation = 0;
    int32_t temp1;
    int32_t temp2;
    uint8_t buffer_index = 0u;
    bool reliability = true;
    uint8_t buffer_size = phandle->speed_buffer_size;
    uint32_t overflow_cnt_sample;
    uint32_t cnt_capture;
    uint32_t direction_sample;
    uint8_t of_bit = 0;

    if (NULL == phandle->timer_get_counter || NULL == phandle->timer_get_direction) {
        return -1;
    }

    cnt_capture = phandle->timer_get_counter(TIMx);
    overflow_cnt_sample = phandle->timer_overflow_nb;
    phandle->timer_overflow_nb = 0;
    direction_sample = phandle->timer_get_direction(TIMx);

    /* If UIFCPY is not present, overflow_cnt_sample can not be used safely for
     speed computation, but we still use it to check that we do not exceed one overflow
     (sample frequency not less than mechanical motor speed */
    if ((overflow_cnt_sample + of_bit) > ENC_MAX_OVERFLOW_NB) {
        phandle->timer_overflow_error = true;
    }

    /*Calculation of delta angle*/
    if (direction_sample == LL_TIM_COUNTERDIRECTION_DOWN) {
        /* encoder timer down-counting*/
        /* if UIFCPY not present Overflow counter can not be safely used -> limitation to 1 OF. */
        phandle->delta_captures_buffer[phandle->delta_captures_index] =
        (int32_t)(cnt_capture) - (int32_t)(phandle->previous_capture)
        - ((int32_t)(overflow_cnt_sample) + of_bit) * (int32_t)(phandle->pulse_number);
    } else {
        /* encoder timer up-counting*/
        /* if UIFCPY not present Overflow counter can not be safely used -> limitation to 1 OF. */
        phandle->delta_captures_buffer[phandle->delta_captures_index] =
        (int32_t)(cnt_capture) - (int32_t)(phandle->previous_capture)
        + ((int32_t)(overflow_cnt_sample) + of_bit) * (int32_t)(phandle->pulse_number);
    }

    /*Computes & returns average mechanical speed */
    for (buffer_index = 0u; buffer_index < buffer_size; buffer_index++) {
        overall_angle_variation += phandle->delta_captures_buffer[buffer_index];
    }
    temp1 = overall_angle_variation * (int32_t)(phandle->speed_sampling_freq_unit);
    temp2 = (int32_t)(phandle->pulse_number) * (int32_t)(phandle->speed_buffer_size);
    temp1 /= temp2;
    *ptr_mecspeed_unit = (int16_t)(temp1);

    /*Computes & stores average mechanical acceleration */
    phandle->_Super.mec_accel_unit_p = (int16_t)(temp1 - phandle->_Super.avr_mecspeed_unit);

    /*Stores average mechanical speed */
    phandle->_Super.avr_mecspeed_unit = (int16_t)temp1;

    /*Computes & stores the instantaneous electrical speed [dpp], var temp1*/
    temp1 = phandle->delta_captures_buffer[phandle->delta_captures_index] * (int32_t)(phandle->speed_sampling_freq_hz)
            * (int32_t)phandle->_Super.el_to_mec_ratio;
    temp1 /= (int32_t)(phandle->pulse_number);
    temp1 *= (int32_t)(phandle->_Super.dpp_conv_factor);
    temp1 /= (int32_t)(phandle->_Super.measurement_frequency);

    phandle->_Super.el_speed_dpp = (int16_t)temp1;

    /*last captured value update*/
    phandle->previous_capture = cnt_capture;
    /*Buffer index update*/
    phandle->delta_captures_index++;

    if (phandle->delta_captures_index == phandle->speed_buffer_size) {
        phandle->delta_captures_index = 0u;
    }

    /*Checks the reliability status, then stores and returns it*/
    if (phandle->timer_overflow_error) {
        reliability = false;
        phandle->sensor_reliable = false;
        phandle->_Super.speed_error_number = phandle->_Super.maximum_speed_errors_number;

    } else {
        reliability = spd_is_mec_speed_reliable(&phandle->_Super, ptr_mecspeed_unit);
    }

    return (reliability);
}

/**
 * @brief  It set instantaneous rotor mechanical angle.
 *         As a consequence, timer counted is computed and updated.
 * @param  phandle: handler of the current instance of the encoder component
 * @param  mec_angle new value of rotor mechanical angle (s16degrees)
 * @retval none
 */
__weak void enc_set_mec_angle(encoder_t* phandle, int16_t mec_angle)
{
    TIM_TypeDef* TIMx = phandle->TIMx;

    uint16_t angle_counts;
    uint16_t mec_angle_uint;

    phandle->_Super.mec_angle = mec_angle;
    phandle->_Super.el_angle = mec_angle * phandle->_Super.el_to_mec_ratio;
    if (mec_angle < 0) {
        mec_angle *= -1;
        mec_angle_uint = 65535u - (uint16_t)mec_angle;
    } else {
        mec_angle_uint = (uint16_t)mec_angle;
    }

    angle_counts = (uint16_t)(((uint32_t)mec_angle_uint * (uint32_t)phandle->pulse_number) / 65535u);

    TIMx->CNT = (uint16_t)(angle_counts);
}

/**
 * @brief  IRQ implementation of the TIMER ENCODER
 * @param  phandle: handler of the current instance of the encoder component
 * @param  flag used to distinguish between various IRQ sources
 * @retval none
 */
__weak void* enc_irq_handler(void* phandle_void)
{
    encoder_t* phandle = (encoder_t*)phandle_void;

    /*Updates the number of overflows occurred*/
    /* the handling of overflow error is done in enc_calc_avrg_mec_speed_unit */
    phandle->timer_overflow_nb += 1u;

    return MC_NULL;
}
/**
 * @}
 */

/**
 * @}
 */

/** @} */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
