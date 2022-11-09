/**
 ******************************************************************************
 * @file    encoder_speed_pos_fdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Encoder Speed & Position Feedback component of the motor Control SDK.
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
#ifndef __ENCODER_SPEEDNPOSFDBK_H
#define __ENCODER_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/** @addtogroup Encoder
 * @{
 */

/* Exported constants --------------------------------------------------------*/

#define GPIO_NoRemap_TIMx    ((uint32_t)(0))
#define ENC_DMA_PRIORITY     DMA_Priority_High
#define ENC_SPEED_ARRAY_SIZE ((uint8_t)16) /* 2^4 */

/**
 * @brief  ENCODER class parameters definition
 */
typedef struct {
    speedn_pos_fdbk_t _Super;

    TIM_TypeDef* TIMx; /*!< Timer used for ENCODER sensor management.*/

    uint32_t speed_sampling_freq_unit;                   /*!< Frequency at which motor speed is to be
                                                          computed. It must be equal to the frequency
                                                          at which function SPD_CalcAvrgMecspeed_unit
                                                          is called.*/
    int32_t delta_captures_buffer[ENC_SPEED_ARRAY_SIZE]; /*!< Buffer used to store
                                                          captured variations of timer counter*/

    uint32_t u32_maxdiv_pulse_number; /*! <It stores U32MAX/hPulseNumber*/

    uint16_t speed_sampling_freq_hz; /*! <Frequency (Hz) at which motor speed
                                      is to be computed. */

    /* SW Settings */
    uint16_t pulse_number; /*!< Number of pulses per revolution, provided by each
                            of the two encoder signals, multiplied by 4 */

    volatile uint16_t timer_overflow_nb; /*!< Number of overflows occurred since
                                          last speed measurement event*/
    uint16_t previous_capture;           /*!< Timer counter value captured during
                                          previous speed measurement event*/

    FunctionalState revert_signal; /*!< To be enabled if measured speed is opposite
                                    to real one (ENABLE/DISABLE)*/

    uint8_t speed_buffer_size; /*!< Size of the buffer used to calculate the average
                                speed. It must be <= 16.*/

    bool sensor_reliable; /*!< Flag to indicate sensor/decoding is not
                           properly working.*/

    uint8_t icx_filter; /*!< Input Capture filter selection */

    volatile uint8_t delta_captures_index; /*! <Buffer index*/

    bool timer_overflow_error; /*!< true if the number of overflow
                                occurred is greater than 'define'
                                ENC_MAX_OVERFLOW_NB*/
    void (*timer_init)(void);
    uint32_t (*timer_get_counter)(TIM_TypeDef* TIMx);
    uint32_t (*timer_get_direction)(TIM_TypeDef* TIMx);
} encoder_t;

void* enc_irq_handler(void* phandle_void);
void enc_init(encoder_t* phandle);
void enc_clear(encoder_t* phandle);
int16_t enc_calc_angle(encoder_t* phandle);
bool enc_calc_avrg_mec_speed_unit(encoder_t* phandle, int16_t* ptr_mecspeed_unit);
void enc_set_mec_angle(encoder_t* phandle, int16_t mec_angle);

/**
 * @}
 */

/**
 * @}
 */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ENCODER_SPEEDNPOSFDBK_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
