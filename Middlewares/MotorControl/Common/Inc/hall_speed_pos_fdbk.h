/**
 ******************************************************************************
 * @file    hall_speed_pos_fdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Hall Speed & Position Feedback component of the motor Control SDK.
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
 * @ingroup hall_speed_pos_fdbk
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HALL_SPEEDNPOSFDBK_H
#define __HALL_SPEEDNPOSFDBK_H

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

/** @addtogroup hall_speed_pos_fdbk
 * @{
 */

#define HALL_SPEED_FIFO_SIZE ((uint8_t)18)

/* HALL SENSORS PLACEMENT ----------------------------------------------------*/
#define DEGREES_120 0u
#define DEGREES_60  1u

/* Exported types ------------------------------------------------------------*/

/**
 * @brief HALL component parameters definition
 *
 *  <Type @p type represents a thing that needs to be detailed more. Additional details
 * are provided in the detailed section of the doxygen comment block.
 *
 * The brief line should be brief and light. It should avoid useless repetitions and expression such as
 * "the CCC_Type_t type...". Expressions like "This type ..." are tolerated especially for key types
 * (usually structures) where we may want ot be more formal.
 *
 * In general: be direct, avoid the obvious, tell the hidden.>
 */

typedef struct {
    speedn_pos_fdbk_t _Super;
    /* SW Settings */
    uint8_t sensor_placement; /*!< Define here the mechanical position of the sensors
                               with reference to an electrical cycle.
                               Allowed values are: DEGREES_120 or DEGREES_60.*/

    int16_t phase_shift; /*!< Define here in s16degree the electrical phase shift
                          between the low to high transition of signal H1 and
                          the maximum of the Bemf induced on phase A.*/

    uint16_t speed_sampling_freq_hz; /*!< Frequency (Hz) at which motor speed is to
                                      be computed. It must be equal to the frequency
                                      at which function SPD_CalcAvrgMecspeed_unit
                                      is called.*/

    uint8_t speed_buffer_size; /*!< Size of the buffer used to calculate the average
                                speed. It must be less than 18.*/

    /* hw Settings */
    uint32_t tim_clock_freq;        /*!< Timer clock frequency express in Hz.*/
    TIM_TypeDef* TIMx;              /*!< Timer used for HALL sensor management.*/
    GPIO_TypeDef* h1_port;          /*!< HALL sensor H1 channel GPIO input port (if used,
                                     after re-mapping). It must be GPIOx x= A, B, ...*/
    uint32_t h1_pin;                /*!< HALL sensor H1 channel GPIO output pin (if used,
                                     after re-mapping). It must be GPIO_Pin_x x= 0, 1,...*/
    GPIO_TypeDef* h2_port;          /*!< HALL sensor H2 channel GPIO input port (if used,
                                     after re-mapping). It must be GPIOx x= A, B, ...*/
    uint32_t h2_pin;                /*!< HALL sensor H2 channel GPIO output pin (if used,
                                            after re-mapping). It must be GPIO_Pin_x x= 0, 1,...*/
    GPIO_TypeDef* h3_port;          /*!< HALL sensor H3 channel GPIO input port (if used,
                                     after re-mapping). It must be GPIOx x= A, B, ...*/
    uint32_t h3_pin;                /*!< HALL sensor H3 channel GPIO output pin (if used,
                                     after re-mapping). It must be GPIO_Pin_x x= 0, 1,...*/
    uint8_t icx_filter;             /*!< Input Capture filter selection */
    bool sensor_reliable;           /*!< Flag to indicate a wrong configuration
                                     of the Hall sensor signanls.*/
    volatile bool ratio_dec;        /*!< Flag to avoid consecutive prescaler
                                     decrement.*/
    volatile bool ratio_inc;        /*!< Flag to avoid consecutive prescaler
                                     increment.*/
    volatile uint8_t first_capt;    /*!< Flag used to discard first capture for
                                     the speed measurement*/
    volatile uint8_t buffer_filled; /*!< Indicate the number of speed measuremt
                                     present in the buffer from the start.
                                     It will be max bSpeedBufferSize and it
                                     is used to validate the start of speed
                                     averaging. If bBufferFilled is below
                                     bSpeedBufferSize the instantaneous
                                     measured speed is returned as average
                                     speed.*/
    volatile uint8_t ovf_counter;   /*!< Count overflows if prescaler is too low*/

    int32_t sensor_period[HALL_SPEED_FIFO_SIZE]; /*!< Holding the last
                                                    period captures */

    uint8_t speed_fifo_idx; /*!< Pointer of next element to be stored in
                             the speed sensor buffer*/
    int32_t el_period_sum;  /* Period accumulator used to speed up the average speed computation*/

    int16_t prev_rotor_freq;         /*!< Used to store the last valid rotor electrical
                                      speed in dpp used when HALL_MAX_PSEUDO_SPEED
                                      is detected */
    int8_t direction;                /*!< Instantaneous direction of rotor between two
                                      captures*/
    int16_t avr_el_speed_dpp;        /*!< It is the averaged rotor electrical speed express
                                      in s16degree per current control period.*/
    uint8_t hall_state;              /*!< Current HALL state configuration */
    int16_t delta_angle;             /*!< Delta angle at the Hall sensor signal edge between
                                      current electrical rotor angle of synchronism.
                                      It is in s16degrees.*/
    int16_t measured_el_angle;       /*!< This is the electrical angle  measured at each
                                      Hall sensor signal edge. It is considered the
                                      best measurement of electrical rotor angle.*/
    int16_t comp_speed;              /*!< Speed compensation factor used to syncronize
                                      the current electrical angle with the target
                                      electrical angle. */
    uint16_t hall_max_ratio;         /*!< Max TIM prescaler ratio defining the lowest
                                      expected speed feedback.*/
    uint16_t sat_speed;              /*!< Returned value if the measured speed is above the
                                      maximum realistic.*/
    uint32_t pseudo_freq_conv;       /*!< Conversion factor between time interval Delta T
                                      between HALL sensors captures, express in timer
                                      counts, and electrical rotor speed express in dpp.
                                      Ex. Rotor speed (dpp) = wpseudo_freq_conv / Delta T
                                      It will be ((CKTIM / 6) / (SAMPLING_FREQ)) * 65536.*/
    uint32_t max_period;             /*!< Time delay between two sensor edges when the speed
                                      of the rotor is the minimum realistic in the
                                      application: this allows to discriminate too low
                                      freq for instance.
                                      This period shoud be expressed in timer counts and
                                      it will be:
                                      wmax_period = ((10 * CKTIM) / 6) / MinElFreq(0.1Hz).*/
    uint32_t min_period;             /*!< Time delay between two sensor edges when the speed
                                      of the rotor is the maximum realistic in the
                                      application: this allows discriminating glitches
                                      for instance. This period shoud be expressed in timer counts and
                                      it will be: wSpeedOverflow = ((10 * CKTIM) / 6) / MaxElFreq(0.1Hz).*/
    uint16_t hall_timeout;           /*!< Max delay between two Hall sensor signal to assert
                                      zero speed express in milliseconds.*/
    uint16_t ovf_freq;               /*!< Frequency of timer overflow (from 0 to 0x10000)
                                      it will be: hovf_freq = CKTIM /65536.*/
    uint16_t pwm_nbrp_sampling_freq; /*!< Number of current control periods inside
                                      each speed control periods it will be:
                                      (measurement_frequency / speed_sampling_freq_hz) - 1.*/
    uint8_t pwm_freq_scaling;        /*!< Scaling factor to allow to store a PWMFrequency greater than 16 bits */
    bool hall_mtpa;                  /* if true at each sensor toggling, the true angle is set without ramp*/

    void (*timer_init)(TIM_TypeDef* TIMx);
    void (*timer_deinit)(TIM_TypeDef* TIMx);
    uint32_t (*timer_get_prescaler)(TIM_TypeDef* TIMx);
    uint32_t (*timer_set_prescaler)(TIM_TypeDef* TIMx, uint32_t Prescaler);
    uint32_t (*timer_ic_get_capture_ch1)(TIM_TypeDef* TIMx);
    uint32_t (*gpio_input_pin_set)(GPIO_TypeDef* GPIOx, uint32_t PinMask);
} switch_hall_sensor_t;
/**
 * @}
 */

void* hall_timx_up_irq_handler(void* phandle_void);
void* hall_timx_cc_irq_handler(void* phandle_void);
void hall_init(switch_hall_sensor_t* phandle);
void hall_clear(switch_hall_sensor_t* phandle);
int16_t hall_calc_el_angle(switch_hall_sensor_t* phandle);
bool hall_calc_avrg_mec_speed_unit(switch_hall_sensor_t* phandle, int16_t* mecspeed_unit);
void hall_set_mec_angle(switch_hall_sensor_t* phandle, int16_t mec_angle);

/**
 * @}
 */

/**
 * @}
 */

/** @} */

#endif /*__HALL_SPEEDNPOSFDBK_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
