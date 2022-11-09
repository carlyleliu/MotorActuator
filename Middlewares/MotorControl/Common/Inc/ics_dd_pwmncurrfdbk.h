/**
 ******************************************************************************
 * @file    ics_dd_pwmncurrfdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          icS DD PWM current feedback component of the motor Control SDK.
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
 * @ingroup PWMnCurrFdbk_icS_DD
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __icS_DD_PWMNCURRFDBK_H
#define __icS_DD_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup pwm_curr_fdbk
 * @{
 */

/** @defgroup PWMnCurrFdbk_icS_DD Insulated Current Sensing Parameters
 *
 *  @brief Common definitions for Insulated Current Sensors based PWM & Current Feedback components
 * @{
 */

#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t)1)
#define NO_SHIFTED_TIMs   ((uint8_t)0)

/**
 * @brief  icS_DD parameters definition
 */
typedef struct {
    /* Dual MC parameters --------------------------------------------------------*/
    uint8_t instance_nbr;       /**< Instance number with reference to PWMC
                                 base class. It is necessary to properly
                                 synchronize TIM8 with TIM1 at
                                 peripheral initializations */
    uint16_t tw;                /**< It is used for switching the context
                                 in dual MC. It contains biggest delay
                                 (expressed in counter ticks) between
                                 the counter crest and ADC latest
                                 trigger  */
    uint8_t freq_ratio;         /**< It is used in case of dual MC to
                                 synchronize TIM1 and TIM8. It has
                                 effect only on the second instanced
                                 object and must be equal to the
                                 ratio between the two PWM frequencies
                                 (higher/lower). Supported values are
                                 1, 2 or 3 */
    uint8_t is_higher_freq_tim; /**< When bfreq_ratio is greather than 1
                                 this param is used to indicate if this
                                 instance is the one with the highest
                                 frequency. Allowed value are: HIGHER_FREQ
                                 or LOWER_FREQ */
    /* Current reading A/D Conversions initialization -----------------------------*/
    uint8_t ia_channel; /**< ADC channel used for conversion of
                         current ia. It must be equal to
                         ADC_CHANNEL_x x= 0, ..., 15*/
    uint8_t ib_channel; /**< ADC channel used for conversion of
                         current ib. It must be equal to
                         ADC_CHANNEL_x x= 0, ..., 15*/

    /* PWM generation parameters --------------------------------------------------*/
    uint8_t repetition_counter; /**< It expresses the number of PWM
                                 periods to be elapsed before compare
                                 registers are updated again. In
                                 particular:
                                 repetition_counter= (2* #PWM periods)-1
                                 */
    TIM_TypeDef* TIMx;          /**< It contains the pointer to the timer
                                 used for PWM generation. It must
                                 equal to TIM1 if binstance_nbr is
                                 equal to 1, to TIM8 otherwise */
    /* PWM Driving signals initialization ----------------------------------------*/
    low_side_outputs_function_t low_side_outputs; /**< Low side or enabling signals
                                                   generation method are defined
                                                   here.*/
    GPIO_TypeDef* pwm_en_u_port;                  /**< enable signal phase U port.*/
    uint32_t pwm_en_u_pin;                        /**< enable signal phase U pin.*/
    GPIO_TypeDef* pwm_en_v_port;                  /**< enable signal phase V port.*/
    uint32_t pwm_en_v_pin;                        /**< enable signal phase V pin.*/
    GPIO_TypeDef* pwm_en_w_port;                  /**< enable signal phase W port.*/
    uint32_t pwm_en_w_pin;                        /**< enable signal phase W pin.*/

    /* Emergency input signal initialization -------------------------------------*/
    FunctionalState emergency_stop; /**< It enable/disable the management of
                                     an emergency input instantaneously
                                     stopping PWM generation. It must be
                                     either equal to ENABLE or DISABLE */
} ics_params_t;
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__icS_DD_PWMNCURRFDBK_H*/
/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
