/**
 ******************************************************************************
 * @file    hall_speed_pos_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features of
 *          the Hall Speed & Position Feedback component of the motor Control SDK.
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
#include "hall_speed_pos_fdbk.h"

#include "mc_type.h"
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/**
 * @defgroup hall_speed_pos_fdbk Hall Speed & Position Feedback
 *
 * @brief Hall Sensor based Speed & Position Feedback implementation
 *
 * This component is used in applications controlling a motor equipped with Hall effect sensors.
 *
 * This component uses the output of two Hall effects sensors to provide a measure of the speed
 * and the position of the rotor of the motor.
 *
 * @todo Document the Hall Speed & Position Feedback "module".
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/* Lower threshold to reques a decrease of clock prescaler */
#define LOW_RES_THRESHOLD ((uint16_t)0x5500u)

#define HALL_COUNTER_RESET ((uint16_t)0u)

#define S16_120_PHASE_SHIFT (int16_t)(65536 / 3)
#define S16_60_PHASE_SHIFT  (int16_t)(65536 / 6)

#define STATE_0 (uint8_t)0
#define STATE_1 (uint8_t)1
#define STATE_2 (uint8_t)2
#define STATE_3 (uint8_t)3
#define STATE_4 (uint8_t)4
#define STATE_5 (uint8_t)5
#define STATE_6 (uint8_t)6
#define STATE_7 (uint8_t)7

#define NEGATIVE (int8_t) - 1
#define POSITIVE (int8_t)1

/* With digit-per-PWM unit (here 2*PI rad = 0xFFFF): */
#define HALL_MAX_PSEUDO_SPEED ((int16_t)0x7FFF)

#define CCER_CC1E_Set   ((uint16_t)0x0001)
#define CCER_CC1E_Reset ((uint16_t)0xFFFE)

static void hall_init_electrical_angle(switch_hall_sensor_t* phandle);

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVic)
            required for the speed position sensor management using HALL
            sensors.
  * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @retval none
  */
__weak void hall_init(switch_hall_sensor_t* phandle)
{
    TIM_TypeDef* TIMx = phandle->TIMx;

    uint16_t min_reliable_elspeed_unit = phandle->_Super.min_reliable_mecspeed_unit * phandle->_Super.el_to_mec_ratio;
    uint16_t max_reliable_elspeed_unit = phandle->_Super.max_reliable_mecspeed_unit * phandle->_Super.el_to_mec_ratio;
    uint8_t speed_buffer_size;
    uint8_t index;

    /* Adjustment factor: minimum measurable speed is x time less than the minimum
    reliable speed */
    min_reliable_elspeed_unit /= 4u;

    /* Adjustment factor: maximum measurable speed is x time greater than the
    maximum reliable speed */
    max_reliable_elspeed_unit *= 2u;

    phandle->ovf_freq = (uint16_t)(phandle->tim_clock_freq / 65536u);

    /* SW Init */
    if (min_reliable_elspeed_unit == 0u) {
        /* Set fixed to 150 ms */
        phandle->hall_timeout = 150u;
    } else {
        /* Set accordingly the min reliable speed */
        /* 1000 comes from mS
         * 6 comes from the fact that sensors are toggling each 60 deg = 360/6 deg */
        phandle->hall_timeout = 1000 * SPEED_UNIT / (6u * min_reliable_elspeed_unit);
    }

    /* Compute the prescaler to the closet value of the TimeOut (in mS )*/
    phandle->hall_max_ratio = (phandle->hall_timeout * phandle->ovf_freq) / 1000;

    /* Align max_period to a multiple of Overflow.*/
    phandle->max_period = (phandle->hall_max_ratio) * 65536uL;

    phandle->sat_speed = max_reliable_elspeed_unit;

    phandle->pseudo_freq_conv =
    ((phandle->tim_clock_freq / 6u) / (phandle->_Super.measurement_frequency)) * (phandle->_Super.dpp_conv_factor);

    phandle->min_period = ((SPEED_UNIT * (phandle->tim_clock_freq / 6uL)) / max_reliable_elspeed_unit);

    phandle->pwm_nbrp_sampling_freq =
    ((phandle->_Super.measurement_frequency * phandle->pwm_freq_scaling) / phandle->speed_sampling_freq_hz) - 1u;

    /* Reset speed reliability */
    phandle->sensor_reliable = true;

    phandle->timer_init(TIMx);

    /* Erase speed buffer */
    speed_buffer_size = phandle->speed_buffer_size;

    for (index = 0u; index < speed_buffer_size; index++) {
        phandle->sensor_period[index] = phandle->max_period;
    }
}

/**
 * @brief  Clear software FIFO where are "pushed" latest speed information
 *         This function must be called before starting the motor to initialize
 *         the speed measurement process.
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component*
 * @retval none
 */
__weak void hall_clear(switch_hall_sensor_t* phandle)
{
    TIM_TypeDef* TIMx = phandle->TIMx;

    phandle->ratio_dec = false;
    phandle->ratio_inc = false;

    /* Reset speed reliability */
    phandle->sensor_reliable = true;

    /* acceleration measurement not implemented.*/
    phandle->_Super.mec_accel_unit_p = 0;

    phandle->first_capt = 0u;
    phandle->buffer_filled = 0u;
    phandle->ovf_counter = 0u;

    phandle->comp_speed = 0;

    phandle->direction = POSITIVE;

    /* Initialize speed buffer index */
    phandle->speed_fifo_idx = 0u;

    /* Clear speed error counter */
    phandle->_Super.speed_error_number = 0;

    phandle->timer_deinit(TIMx);

    hall_init_electrical_angle(phandle);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Update the rotor electrical angle integrating the last measured
 *         instantaneous electrical speed express in dpp.
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @retval int16_t Measured electrical angle in s16degree format.
 */
__weak int16_t hall_calc_el_angle(switch_hall_sensor_t* phandle)
{
    if (phandle->_Super.el_speed_dpp != HALL_MAX_PSEUDO_SPEED) {
        phandle->measured_el_angle += phandle->_Super.el_speed_dpp;
        phandle->_Super.el_angle += phandle->_Super.el_speed_dpp + phandle->comp_speed;
        phandle->prev_rotor_freq = phandle->_Super.el_speed_dpp;
    } else {
        phandle->_Super.el_angle += phandle->prev_rotor_freq;
    }

    return phandle->_Super.el_angle;
}

/**
 * @brief  This method must be called - at least - with the same periodicity
 *         on which speed control is executed.
 *         This method compute and store rotor istantaneous el speed (express
 *         in dpp considering the measurement frequency) in order to provide it
 *         to hall_calc_el_angle function and spd_get_el_angle.
 *         Then compute rotor average el speed (express in dpp considering the
 *         measurement frequency) based on the buffer filled by IRQ, then - as
 *         a consequence - compute, store and return - through parameter
 *         mecspeed_unit - the rotor average mech speed, expressed in Unit.
 *         Then check, store and return the reliability state of
 *         the sensor; in this function the reliability is measured with
 *         reference to specific parameters of the derived
 *         sensor (HALL) through internal variables managed by IRQ.
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @param  mecspeed_unit pointer to int16_t, used to return the rotor average
 *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
 * @retval true = sensor information is reliable
 *         false = sensor information is not reliable
 */
__weak bool hall_calc_avrg_mec_speed_unit(switch_hall_sensor_t* phandle, int16_t* mecspeed_unit)
{
    TIM_TypeDef* TIMx = phandle->TIMx;
    bool bReliability;

    if (phandle->sensor_reliable) {
        /* No errors have been detected during rotor speed information
         extrapolation */
        if (phandle->timer_get_prescaler(TIMx) >= phandle->hall_max_ratio) {
            /* At start-up or very low freq */
            /* Based on current prescaler value only */
            phandle->_Super.el_speed_dpp = 0;
            *mecspeed_unit = 0;
        } else {
            phandle->_Super.el_speed_dpp = phandle->avr_el_speed_dpp;
            if (phandle->avr_el_speed_dpp == 0) {
                /* Speed is too low */
                *mecspeed_unit = 0;
            } else {
                /* Check if speed is not to fast */
                if (phandle->avr_el_speed_dpp != HALL_MAX_PSEUDO_SPEED) {
                    if (phandle->hall_mtpa == true) {
                        phandle->comp_speed = 0;
                    } else {
                        phandle->delta_angle = phandle->measured_el_angle - phandle->_Super.el_angle;
                        phandle->comp_speed =
                        (int16_t)((int32_t)(phandle->delta_angle) / (int32_t)(phandle->pwm_nbrp_sampling_freq));
                    }
                    /* Convert el_dpp to MecUnit */
                    *mecspeed_unit =
                    (int16_t)((phandle->avr_el_speed_dpp * (int32_t)phandle->_Super.measurement_frequency * (int32_t)SPEED_UNIT)
                              / ((int32_t)phandle->_Super.dpp_conv_factor * (int32_t)phandle->_Super.el_to_mec_ratio));
                } else {
                    *mecspeed_unit = (int16_t)phandle->sat_speed;
                }
            }
        }
        bReliability = spd_is_mec_speed_reliable(&phandle->_Super, mecspeed_unit);
    } else {
        bReliability = false;
        phandle->_Super.speed_error_number = phandle->_Super.maximum_speed_errors_number;
        /* If speed is not reliable the El and Mec speed is set to 0 */
        phandle->_Super.el_speed_dpp = 0;
        *mecspeed_unit = 0;
    }

    phandle->_Super.avr_mecspeed_unit = *mecspeed_unit;

    return (bReliability);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Example of private method of the class HALL to implement an MC IRQ function
 *         to be called when TIMx capture event occurs
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @retval none
 */
__weak void* hall_timx_cc_irq_handler(void* phandle_void)
{
    switch_hall_sensor_t* phandle = (switch_hall_sensor_t*)phandle_void;
    TIM_TypeDef* TIMx = phandle->TIMx;
    uint8_t prev_hall_state;
    int8_t prev_direction;
    uint32_t capt_buf;
    uint16_t prsc_buf;
    uint16_t high_speed_capture;

    if (phandle->sensor_reliable) {
        /* A capture event generated this interrupt */
        prev_hall_state = phandle->hall_state;
        prev_direction = phandle->direction;

        if (phandle->sensor_placement == DEGREES_120) {
            phandle->hall_state = (uint8_t)((phandle->gpio_input_pin_set(phandle->h3_port, phandle->h3_pin) << 2)
                                            | (phandle->gpio_input_pin_set(phandle->h2_port, phandle->h2_pin) << 1)
                                            | phandle->gpio_input_pin_set(phandle->h1_port, phandle->h1_pin));
        } else {
            phandle->hall_state = (uint8_t)(((phandle->gpio_input_pin_set(phandle->h2_port, phandle->h2_pin) ^ 1) << 2)
                                            | (phandle->gpio_input_pin_set(phandle->h3_port, phandle->h3_pin) << 1)
                                            | phandle->gpio_input_pin_set(phandle->h1_port, phandle->h1_pin));
        }

        switch (phandle->hall_state) {
            case STATE_5:
                if (prev_hall_state == STATE_4) {
                    phandle->direction = POSITIVE;
                    phandle->measured_el_angle = phandle->phase_shift;
                } else if (prev_hall_state == STATE_1) {
                    phandle->direction = NEGATIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift + S16_60_PHASE_SHIFT);
                } else {
                }
                break;

            case STATE_1:
                if (prev_hall_state == STATE_5) {
                    phandle->direction = POSITIVE;
                    phandle->measured_el_angle = phandle->phase_shift + S16_60_PHASE_SHIFT;
                } else if (prev_hall_state == STATE_3) {
                    phandle->direction = NEGATIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift + S16_120_PHASE_SHIFT);
                } else {
                }
                break;

            case STATE_3:
                if (prev_hall_state == STATE_1) {
                    phandle->direction = POSITIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift + S16_120_PHASE_SHIFT);
                } else if (prev_hall_state == STATE_2) {
                    phandle->direction = NEGATIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT);
                } else {
                }

                break;

            case STATE_2:
                if (prev_hall_state == STATE_3) {
                    phandle->direction = POSITIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT);
                } else if (prev_hall_state == STATE_6) {
                    phandle->direction = NEGATIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift - S16_120_PHASE_SHIFT);
                } else {
                }
                break;

            case STATE_6:
                if (prev_hall_state == STATE_2) {
                    phandle->direction = POSITIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift - S16_120_PHASE_SHIFT);
                } else if (prev_hall_state == STATE_4) {
                    phandle->direction = NEGATIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift - S16_60_PHASE_SHIFT);
                } else {
                }
                break;

            case STATE_4:
                if (prev_hall_state == STATE_6) {
                    phandle->direction = POSITIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift - S16_60_PHASE_SHIFT);
                } else if (prev_hall_state == STATE_5) {
                    phandle->direction = NEGATIVE;
                    phandle->measured_el_angle = (int16_t)(phandle->phase_shift);
                } else {
                }
                break;

            default:
                /* Bad hall sensor configutarion so update the speed reliability */
                phandle->sensor_reliable = false;

                break;
        }
        /* We need to check that the direction has not changed.
         If it is the case, the sign of the current speed can be the opposite of the
         average speed, and the average time can be close to 0 which lead to a
         computed speed close to the infinite, and bring instability. */
        if (phandle->direction != prev_direction) {
            /* Setting buffer_filled to 0 will prevent to compute the average speed based
             on the SpeedPeriod buffer values */
            phandle->buffer_filled = 0;
            phandle->speed_fifo_idx = 0;
        }

        if (phandle->hall_mtpa == true) {
            phandle->_Super.el_angle = phandle->measured_el_angle;
        } else {
            /* Nothing to do */
        }

        /* Discard first capture */
        if (phandle->first_capt == 0u) {
            phandle->first_capt++;
            phandle->timer_ic_get_capture_ch1(TIMx);
        } else {
            /* used to validate the average speed measurement */
            if (phandle->buffer_filled < phandle->speed_buffer_size) {
                phandle->buffer_filled++;
            }

            /* Store the latest speed acquisition */
            high_speed_capture = phandle->timer_ic_get_capture_ch1(TIMx);
            capt_buf = (uint32_t)high_speed_capture;
            prsc_buf = phandle->timer_get_prescaler(TIMx);

            /* Add the numbers of overflow to the counter */
            capt_buf += (uint32_t)phandle->ovf_counter * 0x10000uL;

            if (phandle->ovf_counter != 0u) {
                /* Adjust the capture using prescaler */
                uint16_t hAux;
                hAux = prsc_buf + 1u;
                capt_buf *= hAux;

                if (phandle->ratio_inc) {
                    phandle->ratio_inc = false; /* Previous capture caused overflow */
                                                /* Don't change prescaler (delay due to preload/update mechanism) */
                } else {
                    if (phandle->timer_get_prescaler(TIMx) < phandle->hall_max_ratio) /* Avoid OVF w/ very low freq */
                    {
                        phandle->timer_set_prescaler(TIMx, phandle->timer_get_prescaler(TIMx) + 1); /* To avoid OVF during speed decrease */
                        phandle->ratio_inc = true; /* new prsc value updated at next capture only */
                    }
                }
            } else {
                /* If prsc preload reduced in last capture, store current register + 1 */
                if (phandle->ratio_dec) /* and don't decrease it again */
                {
                    /* Adjust the capture using prescaler */
                    uint16_t hAux;
                    hAux = prsc_buf + 2u;
                    capt_buf *= hAux;

                    phandle->ratio_dec = false;
                } else /* If prescaler was not modified on previous capture */
                {
                    /* Adjust the capture using prescaler */
                    uint16_t hAux = prsc_buf + 1u;
                    capt_buf *= hAux;

                    if (high_speed_capture < LOW_RES_THRESHOLD) /* If capture range correct */
                    {
                        if (phandle->timer_get_prescaler(TIMx) > 0u) /* or prescaler cannot be further reduced */
                        {
                            phandle->timer_set_prescaler(TIMx, phandle->timer_get_prescaler(TIMx) - 1); /* Increase accuracy by decreasing prsc */
                            /* Avoid decrementing again in next capt.(register preload delay) */
                            phandle->ratio_dec = true;
                        }
                    }
                }
            }

            /* Filtering to fast speed... could be a glitch  ? */
            /* the HALL_MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation*/
            if (capt_buf < phandle->min_period) {
                /* phandle->avr_el_speed_dpp = HALL_MAX_PSEUDO_SPEED; */
            } else {
                phandle->el_period_sum -=
                phandle->sensor_period[phandle->speed_fifo_idx]; /* value we gonna removed from the accumulator */
                if (capt_buf >= phandle->max_period) {
                    phandle->sensor_period[phandle->speed_fifo_idx] = phandle->max_period * phandle->direction;
                } else {
                    phandle->sensor_period[phandle->speed_fifo_idx] = capt_buf;
                    phandle->sensor_period[phandle->speed_fifo_idx] *= phandle->direction;
                    phandle->el_period_sum += phandle->sensor_period[phandle->speed_fifo_idx];
                }
                /* Update pointers to speed buffer */
                phandle->speed_fifo_idx++;
                if (phandle->speed_fifo_idx == phandle->speed_buffer_size) {
                    phandle->speed_fifo_idx = 0u;
                }
                if (phandle->sensor_reliable) {
                    if (phandle->buffer_filled < phandle->speed_buffer_size) {
                        phandle->avr_el_speed_dpp = (int16_t)((phandle->pseudo_freq_conv / capt_buf) * phandle->direction);
                    } else { /* Average speed allow to smooth the mechanical sensors misalignement */
                        phandle->avr_el_speed_dpp =
                        (int16_t)((int32_t)phandle->pseudo_freq_conv / (phandle->el_period_sum / phandle->speed_buffer_size)); /* Average value */
                    }
                } else /* Sensor is not reliable */
                {
                    phandle->avr_el_speed_dpp = 0;
                }
            }
            /* Reset the number of overflow occurred */
            phandle->ovf_counter = 0u;
        }
    }
    return MC_NULL;
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Example of private method of the class HALL to implement an MC IRQ function
 *         to be called when TIMx update event occurs
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @retval none
 */
__weak void* hall_timx_up_irq_handler(void* phandle_void)
{
    switch_hall_sensor_t* phandle = (switch_hall_sensor_t*)phandle_void;
    TIM_TypeDef* TIMx = phandle->TIMx;

    if (phandle->sensor_reliable) {
        uint16_t hMaxTimerOverflow;
        /* an update event occured for this interrupt request generation */
        phandle->ovf_counter++;

        hMaxTimerOverflow =
        (uint16_t)(((uint32_t)phandle->hall_timeout * phandle->ovf_freq) / ((phandle->timer_get_prescaler(TIMx) + 1) * 1000u));
        if (phandle->ovf_counter >= hMaxTimerOverflow) {
            /* Set rotor speed to zero */
            phandle->_Super.el_speed_dpp = 0;

            /* Reset the electrical angle according the hall sensor configuration */
            hall_init_electrical_angle(phandle);

            /* Reset the overflow counter */
            phandle->ovf_counter = 0u;

            /* Reset first capture flag */
            phandle->first_capt = 0u;

            /* Reset the SensorSpeed buffer*/
            uint8_t index;
            for (index = 0u; index < phandle->speed_buffer_size; index++) {
                phandle->sensor_period[index] = phandle->max_period;
            }
            phandle->buffer_filled = 0;
            phandle->avr_el_speed_dpp = 0;
            phandle->speed_fifo_idx = 0;
            phandle->el_period_sum = phandle->max_period * phandle->speed_buffer_size;
        }
    }
    return MC_NULL;
}

/**
 * @brief  Read the logic level of the three Hall sensor and individuates in this
 *         way the position of the rotor (+/- 30ï¿½). Electrical angle is then
 *         initialized.
 * @param  phandle: handler of the current instance of the hall_speed_pos_fdbk component
 * @retval none
 */
static void hall_init_electrical_angle(switch_hall_sensor_t* phandle)
{
    if (phandle->sensor_placement == DEGREES_120) {
        phandle->hall_state = phandle->gpio_input_pin_set(phandle->h3_port, phandle->h3_pin) << 2
                              | phandle->gpio_input_pin_set(phandle->h2_port, phandle->h2_pin) << 1
                              | phandle->gpio_input_pin_set(phandle->h1_port, phandle->h1_pin);
    } else {
        phandle->hall_state = (phandle->gpio_input_pin_set(phandle->h2_port, phandle->h2_pin) ^ 1) << 2
                              | phandle->gpio_input_pin_set(phandle->h3_port, phandle->h3_pin) << 1
                              | phandle->gpio_input_pin_set(phandle->h1_port, phandle->h1_pin);
    }

    switch (phandle->hall_state) {
        case STATE_5:
            phandle->_Super.el_angle = (int16_t)(phandle->phase_shift + S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_1:
            phandle->_Super.el_angle = (int16_t)(phandle->phase_shift + S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_3:
            phandle->_Super.el_angle = (int16_t)(phandle->phase_shift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_2:
            phandle->_Super.el_angle = (int16_t)(phandle->phase_shift - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_6:
            phandle->_Super.el_angle = (int16_t)(phandle->phase_shift - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_4:
            phandle->_Super.el_angle = (int16_t)(phandle->phase_shift - S16_60_PHASE_SHIFT / 2);
            break;
        default:
            /* Bad hall sensor configutarion so update the speed reliability */
            phandle->sensor_reliable = false;
            break;
    }

    /* Initialize the measured angle */
    phandle->measured_el_angle = phandle->_Super.el_angle;
}

/**
 * @brief  It could be used to set istantaneous information on rotor mechanical
 *         angle.
 *         Note: Mechanical angle management is not implemented in this
 *         version of Hall sensor class.
 * @param  phandle pointer on related component instance
 * @param  mec_angle istantaneous measure of rotor mechanical angle
 * @retval none
 */
__weak void hall_set_mec_angle(switch_hall_sensor_t* phandle, int16_t mec_angle) {}

/**
 * @}
 */

/**
 * @}
 */

/** @} */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
