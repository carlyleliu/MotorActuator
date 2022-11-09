/**
 ******************************************************************************
 * @file    pwm_curr_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the following features
 *          of the PWM & Current Feedback component of the motor Control SDK:
 *
 *           * current sensing
 *           * regular ADC conversion execution
 *           * space vector modulation
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
#include "pwm_curr_fdbk.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup pwm_curr_fdbk PWM & Current Feedback
 *
 * @brief PWM & Current Feedback components of the motor Control SDK
 *
 * These components fulfill two functions in a motor Control subsystem:
 *
 * - The generation of the Space Vector Pulse Width Modulation on the motor's phases
 * - The sampling of the actual motor's phases current
 *
 * Both these features are closely related as the instants when the values of the phase currents
 * should be sampled by the ADC channels are basically triggered by the timers used to generate
 * the duty cycles for the PWM.
 *
 * Several implementation of PWM and Current Feedback components are provided by the motor Control
 * SDK to account for the specificities of the application:
 *
 * - The selected MCU: the number of ADCs available on a given MCU, the presence of internal
 * comparators or OpAmps, for instance, lead to different implementation of this feature
 * - The Current sensing topology also has an impact on the firmware: implementations are provided
 * for Insulated Current Sensors, Single Shunt and Three Shunt resistors current sensing topologies
 *
 * The choice of the implementation mostly depend on these two factors and is performed by the
 * motor Control Workbench tool.
 *
 * All these implementations are built on a base PWM & Current Feedback component that they extend
 * and that provides the functions and data that are common to all of them. This base component is
 * never used directly as it does not provide a complete implementation of the features. Rather,
 * its handle structure (pwmc) is reused by all the PWM & Current Feedback specific
 * implementations and the functions it provides form the API of the PWM and Current feedback feature.
 * Calling them results in calling functions of the component that actually implement the feature.
 * See pwmc for more details on this mechanism.
 * @{
 */

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief Returns the phase current of the motor as read by the ADC (in s16A unit)
 *
 * The function actually returns the current values of phase A & B. Phase C current
 * can be deduced thanks to the formula:
 *
 * @f[
 * I_{C} = -I_{A} - I_{C}
 * @f]
 *
 * @param  phandle handle on the target PWMC component
 * @param  pStator_Currents Pointer to the structure that will receive motor current
 *         of phase A and B in ElectricalValue format.
 */
__weak void pwmc_get_phase_currents(pwmc_t* phandle, ab_t* iab)
{
    phandle->fct_get_phase_currents(phandle, iab);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief Sets the PWM duty cycles
 *
 *
 */

/**
 * @brief  Converts input voltages @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into PWM duty cycles
 *         and feed them to the inverter.
 * @param  phandle handler on the target PWMC component.
 * @param  valfa_beta Voltage Components expressed in the @f$(\alpha, \beta)@f$ reference frame
 *
 * This function computes the the time during wich the transistors of each phase are to be switched on in
 * a PWM cycle in order to achieve the reference phase voltage set by @p valfa_beta. Then, the function
 * programs the resulting duty cycles in the related timer channels. It also sets the phase current
 * sampling point for the next PWM cycle accordingly.
 *
 * This function is used in the FOC frequency loop and needs to complete before the next PWM cycle starts
 * so that the duty cycles it computes can be taken into account. Failing to do so (for instance because
 * the PWM Frequency is too high) results in the functions returning #MC_FOC_DURATION wich entails a
 * motor Control Fault that stops the motor.
 *
 * @retval Returns #MC_NO_ERROR if no error occurred or #MC_FOC_DURATION if the duty cycles were
 *         set too late for being taken into account in the next PWM cycle.
 */
__weak uint16_t pwmc_set_phase_voltage(pwmc_t* phandle, alphabeta_t valfa_beta)
{
    int32_t x, y, z, u_alpha, u_beta, time_pha, time_phb, time_phc;

    u_alpha = valfa_beta.alpha * (int32_t)phandle->t_sqrt3;
    u_beta = -(valfa_beta.beta * (int32_t)(phandle->pwm_period)) * 2;

    x = u_beta;
    y = (u_beta + u_alpha) / 2;
    z = (u_beta - u_alpha) / 2;

    /* Sector calculation from x, y, z */
    if (y < 0) {
        if (z < 0) {
            phandle->Sector = SECTOR_5;
            time_pha = (int32_t)(phandle->pwm_period) / 4 + ((y - z) / (int32_t)262144);
            time_phb = time_pha + z / 131072;
            time_phc = time_pha - y / 131072;
            phandle->low_duty = time_phc;
            phandle->mid_duty = time_pha;
            phandle->high_duty = time_phb;
        } else /* z >= 0 */
        if (x <= 0) {
            phandle->Sector = SECTOR_4;
            time_pha = (int32_t)(phandle->pwm_period) / 4 + ((x - z) / (int32_t)262144);
            time_phb = time_pha + z / 131072;
            time_phc = time_phb - x / 131072;
            phandle->low_duty = time_phc;
            phandle->mid_duty = time_phb;
            phandle->high_duty = time_pha;
        } else /* x > 0 */
        {
            phandle->Sector = SECTOR_3;
            time_pha = (int32_t)(phandle->pwm_period) / 4 + ((y - x) / (int32_t)262144);
            time_phc = time_pha - y / 131072;
            time_phb = time_phc + x / 131072;
            phandle->low_duty = time_phb;
            phandle->mid_duty = time_phc;
            phandle->high_duty = time_pha;
        }
    } else /* y > 0 */
    {
        if (z >= 0) {
            phandle->Sector = SECTOR_2;
            time_pha = (int32_t)(phandle->pwm_period) / 4 + ((y - z) / (int32_t)262144);
            time_phb = time_pha + z / 131072;
            time_phc = time_pha - y / 131072;
            phandle->low_duty = time_phb;
            phandle->mid_duty = time_pha;
            phandle->high_duty = time_phc;
        } else /* z < 0 */
        if (x <= 0) {
            phandle->Sector = SECTOR_6;
            time_pha = (int32_t)(phandle->pwm_period) / 4 + ((y - x) / (int32_t)262144);
            time_phc = time_pha - y / 131072;
            time_phb = time_phc + x / 131072;
            phandle->low_duty = time_pha;
            phandle->mid_duty = time_phc;
            phandle->high_duty = time_phb;
        } else /* x > 0 */
        {
            phandle->Sector = SECTOR_1;
            time_pha = (int32_t)(phandle->pwm_period) / 4 + ((x - z) / (int32_t)262144);
            time_phb = time_pha + z / 131072;
            time_phc = time_phb - x / 131072;
            phandle->low_duty = time_pha;
            phandle->mid_duty = time_phb;
            phandle->high_duty = time_phc;
        }
    }

    phandle->cnt_pha = (uint16_t)time_pha;
    phandle->cnt_phb = (uint16_t)time_phb;
    phandle->cnt_phc = (uint16_t)time_phc;

    if (phandle->dt_test == 1u) {
        /* Dead time compensation */
        if (phandle->ia > 0) {
            phandle->cnt_pha += phandle->dt_comp_cnt;
        } else {
            phandle->cnt_pha -= phandle->dt_comp_cnt;
        }

        if (phandle->ib > 0) {
            phandle->cnt_phb += phandle->dt_comp_cnt;
        } else {
            phandle->cnt_phb -= phandle->dt_comp_cnt;
        }

        if (phandle->ic > 0) {
            phandle->cnt_phc += phandle->dt_comp_cnt;
        } else {
            phandle->cnt_phc -= phandle->dt_comp_cnt;
        }
    }

    return (phandle->fct_set_adc_samp_point_sectx(phandle));
}

/**
 * @brief  Switches PWM generation off, inactivating the outputs.
 * @param  phandle Handle on the target instance of the PWMC component
 */
__weak void pwmc_switch_off_pwm(pwmc_t* phandle)
{
    if (NULL != phandle->fct_switchoff_pwm)
        phandle->fct_switchoff_pwm(phandle);
}

/**
 * @brief  Switches PWM generation on
 * @param  phandle Handle on the target instance of the PWMC component
 */
__weak void pwmc_switch_on_pwm(pwmc_t* phandle)
{
    if (NULL != phandle->fct_switchon_pwm)
        phandle->fct_switchon_pwm(phandle);
}

/**
 * @brief  Calibrates ADC current conversions by reading the offset voltage
 *         present on ADC pins when no motor current is flowing in.
 *
 * This function should be called before each motor start-up.
 *
 * @param  phandle Handle on the target instance of the PWMC component
 * @param  action Can be #CRC_START to initialize the offset calibration or
 *         #CRC_EXEC to execute the offset calibration.
 * @retval true if the current calibration has been completed, false if it is
 *         still ongoing.
 */
__weak bool pwmc_current_reading_calibr(pwmc_t* phandle, crc_action_t action)
{
    bool ret_val = false;
    if (action == CRC_START) {
        pwmc_switch_off_pwm(phandle);
        phandle->off_calibr_wait_time_counter = phandle->off_calibr_wait_ticks;
        if (phandle->off_calibr_wait_ticks == 0u) {
            phandle->fct_curr_reading_calib(phandle);
            ret_val = true;
        }
    } else if (action == CRC_EXEC) {
        if (phandle->off_calibr_wait_time_counter > 0u) {
            phandle->off_calibr_wait_time_counter--;
            if (phandle->off_calibr_wait_time_counter == 0u) {
                phandle->fct_curr_reading_calib(phandle);
                ret_val = true;
            }
        } else {
            ret_val = true;
        }
    } else {
    }
    return ret_val;
}

/**
 * @brief  Switches power stage Low Sides transistors on.
 *
 * This function is meant for charging boot capacitors of the driving
 * section. It has to be called on each motor start-up when using high
 * voltage drivers.
 *
 * @param  phandle: handle on the target instance of the PWMC component
 */
__weak void PWMC_TurnOnLowSides(pwmc_t* phandle)
{
    phandle->fct_turnon_lowsides(phandle);
}

/** @brief Returns #MC_BREAK_IN if an over current condition was detected on the power stage
 *         controlled by the PWMC component pointed by  @p phandle, since the last call to this function;
 *         returns #MC_NO_FAULTS otherwise. */
__weak uint16_t pwmc_check_over_current(pwmc_t* phandle)
{
    return phandle->fct_is_over_current_occurred(phandle);
}

/**
 * @brief  Sets the over current threshold to be used
 *
 * The value to be set is relative to the VDD_DAC DAC reference voltage with
 * 0 standing for 0 V and 65536 standing for VDD_DAC.
 *
 * @param  phandle handle on the target instance of the PWMC component
 * @param  dac_vref Value of DAC reference expressed as 16bit unsigned integer
 */
__weak void pwmc_ocp_set_reference_voltage(pwmc_t* phandle, uint16_t dac_vref)
{
    if (phandle->fct_ocp_set_reference_voltage) {
        phandle->fct_ocp_set_reference_voltage(phandle, dac_vref);
    }
}

/**
 * @brief  It is used to retrieve the satus of TurnOnLowSides action.
 * @param  phandle: handler of the current instance of the PWMC component
 * @retval bool It returns the state of TurnOnLowSides action:
 *         true if TurnOnLowSides action is active, false otherwise.
 */
/** @brief Returns the status of the "TurnOnLowSide" action on the power stage
 *         controlled by the @p phandle PWMC component: true if it
 *         is active, false otherwise*/
__weak bool pwmc_getturn_on_low_sides_action(pwmc_t* phandle)
{
    return phandle->turn_on_low_sides_action;
}

/** @brief Enables the RL detection mode on the power stage controlled by the @p phandle PWMC component. */
__weak void pwmc_rl_detection_mode_enable(pwmc_t* phandle)
{
    if (phandle->fct_rl_detection_mode_enable) {
        phandle->fct_rl_detection_mode_enable(phandle);
    }
}

/** @brief Disables the RL detection mode on the power stage controlled by the @p phandle PWMC component. */
__weak void pwmc_rl_detection_mode_disable(pwmc_t* phandle)
{
    if (phandle->fct_rl_detection_mode_disable) {
        phandle->fct_rl_detection_mode_disable(phandle);
    }
}

/**
 * @brief  Sets the PWM duty cycle to apply in the RL Detection mode.
 * @param  phandle: handle on the target instance of the PWMC component
 * @param  duty Duty cycle to apply
 *
 * @todo TODO: Describe the unit of the duty variable.
 *
 * @retval If the Duty Cycle could be applied on time for the next PWM period,
 *         #MC_NO_ERROR is returned. Otherwise, #MC_FOC_DURATION is returned.
 */
__weak uint16_t pwmc_rl_detection_mode_set_duty(pwmc_t* phandle, uint16_t duty)
{
    uint16_t ret_val = MC_FOC_DURATION;
    if (phandle->fct_rl_detection_mode_set_duty) {
        ret_val = phandle->fct_rl_detection_mode_set_duty(phandle, duty);
    }
    return ret_val;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to get phases current.
 * @param ptr_callback pointer on the callback
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_get_phase_currents_callback(pwmc_get_phase_curr_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_get_phase_currents = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation off.
 * @param ptr_callback pointer on the callback
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_switch_off_pwm_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_switchoff_pwm = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation on.
 * @param ptr_callback pointer on the callback
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_switch_on_pwm_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_switchon_pwm = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to execute a calibration
 *        of the current sensing system.
 * @param ptr_callback pointer on the callback
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_reading_calibration_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_curr_reading_calib = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to turn low sides on.
 * @param ptr_callback pointer on the callback
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_turn_on_low_sides_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_turnon_lowsides = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to compute ADC sampling point
 * @param ptr_callback pointer on the callback
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_samp_point_sectx_callback(pwmc_set_samp_point_sectx_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_set_adc_samp_point_sectx = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to the overcurrent status
 * @param ptr_callback pointer on the callback
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_registerIs_over_current_occurred_callback(pwmc_over_curr_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_is_over_current_occurred = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the reference
 *        voltage for the over current protection
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_ocp_set_ref_voltage_callback(pwmc_set_ocp_ref_volt_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_ocp_set_reference_voltage = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to enable the R/L detection mode
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_rl_detection_mode_enable_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_rl_detection_mode_enable = ptr_callback;
}

/**
 * @brief Sets the Callback wich PWMC shall invoke to disable the R/L detection mode
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_rl_detection_mode_disable_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_rl_detection_mode_disable = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the duty cycle
 *        for the R/L detection mode
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void pwmc_register_rl_detection_mode_set_duty_callback(pwmc_rl_detect_set_duty_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_rl_detection_mode_set_duty = ptr_callback;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to call PWMC instance IRQ handler
 * @param phandle pointer on the handle structure of the PWMC instance
 *
 * @note this function is deprecated.
 */
__weak void pwmc_register_irq_handler_callback(pwmc_irq_handler_cb_t ptr_callback, pwmc_t* phandle)
{
    phandle->fct_irq_handler = ptr_callback;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
