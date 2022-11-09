/**
 ******************************************************************************
 * @file    pwm_curr_fdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          PWM & Current Feedback component of the motor Control SDK.
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
 * @ingroup pwm_curr_fdbk
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWMNCURRFDBK_H
#define PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup pwm_curr_fdbk
 * @{
 */

/* Exported defines ------------------------------------------------------------*/

#define SECTOR_1    0u
#define SECTOR_2    1u
#define SECTOR_3    2u
#define SECTOR_4    3u
#define SECTOR_5    4u
#define SECTOR_6    5u
#define SQRT3FACTOR (uint16_t)0xDDB4 /* = (16384 * 1.732051 * 2)*/

/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct pwmc pwmc_t;

/**
 * @brief Pointer on callback functions used by PWMC components
 *
 * This type is needed because the actual functions to use can change at run-time.
 *
 * See the following items:
 * - pwmc::fct_switchoff_pwm
 * - pwmc::fct_switchon_pwm
 * - pwmc::fct_curr_reading_calib
 * - pwmc::fct_turnon_lowsides
 * - pwmc::fct_rl_detection_mode_enable
 * - pwmc::fct_rl_detection_mode_disable
 *
 *
 */
typedef void (*pwmc_generic_cb_t)(pwmc_t* phandle);

/**
 * @brief Pointer on the interrupt handling function of the PWMC component instance.
 *
 * This type is needed because the actual function to use can change at run-time
 * (See pwmc::fct_irq_handler).
 *
 */
typedef void* (*pwmc_irq_handler_cb_t)(pwmc_t* phandle, unsigned char flag);

/**
 * @brief Pointer on the function provided by the PMWC component instance to get the phase current.
 *
 * This type is needed because the actual function to use can change at run-time
 * (See pwmc::fct_get_phase_currents).
 *
 */
typedef void (*pwmc_get_phase_curr_cb_t)(pwmc_t* phandle, ab_t* iab);

/**
 * @brief Pointer on the function provided by the PMWC component instance to set the reference
 *        voltage for the over current protection.
 *
 * This type is needed because the actual function to use can change at run-time
 * (See pwmc::fct_ocp_set_reference_voltage).
 *
 */
typedef void (*pwmc_set_ocp_ref_volt_cb_t)(pwmc_t* phandle, uint16_t dac_vref);

/**
 * @brief Pointer on the functions provided by the PMWC component instance to set the ADC sampling
 *        point for each sectors.
 *
 * This type is needed because the actual function to use can change at run-time. See:
 * - pwmc::pFctSetADCSampPointSect1
 * - pwmc::pFctSetADCSampPointSect2
 * - pwmc::pFctSetADCSampPointSect3
 * - pwmc::pFctSetADCSampPointSect4
 * - pwmc::pFctSetADCSampPointSect5
 * - pwmc::pFctSetADCSampPointSect6
 *
 */
typedef uint16_t (*pwmc_set_samp_point_sectx_cb_t)(pwmc_t* phandle);

/**
 * @brief Pointer on the function provided by the PMWC component instance to check if an over current
 *        condition has occured.
 *
 * This type is needed because the actual function to use can change at run-time
 * (See pwmc::fct_is_over_current_occurred).
 *
 */
typedef uint16_t (*pwmc_over_curr_cb_t)(pwmc_t* phandle);

/**
 * @brief Pointer on the function provided by the PMWC component instance to set the PWM duty cycle
 *        in RL detection mode.
 *
 * This type is needed because the actual function to use can change at run-time
 * (See pwmc::fct_rl_detection_mode_set_duty).
 *
 */
typedef uint16_t (*pwmc_rl_detect_set_duty_cb_t)(pwmc_t* phandle, uint16_t duty);

/**
 * @brief This structure is used to handle the data of an instance of the PWM & Current Feedback component
 *
 */
struct pwmc {
    /** @{ */
    pwmc_irq_handler_cb_t fct_irq_handler; /**< pointer on the interrupt handling function. */
    pwmc_get_phase_curr_cb_t fct_get_phase_currents; /**< pointer on the function the component instance uses to retrieve pahse currents */
    pwmc_generic_cb_t fct_switchoff_pwm; /**< pointer on the function the component instance uses to switch PWM off */
    pwmc_generic_cb_t fct_switchon_pwm;  /**< pointer on the function the component instance uses to switch PWM on */
    pwmc_generic_cb_t fct_curr_reading_calib; /**< pointer on the function the component instance uses to calibrate the current reading ADC(s) */
    pwmc_generic_cb_t fct_turnon_lowsides; /**< pointer on the function the component instance uses to turn low sides on */
    pwmc_set_samp_point_sectx_cb_t fct_set_adc_samp_point_sectx; /**< pointer on the function the component instance uses to set the ADC sampling point  */
    pwmc_over_curr_cb_t fct_is_over_current_occurred; /**< pointer on the function the component instance uses to return the over current status */
    pwmc_set_ocp_ref_volt_cb_t fct_ocp_set_reference_voltage; /**< pointer on the function the component instance uses to set the over current reference voltage */
    pwmc_generic_cb_t fct_rl_detection_mode_enable; /**< pointer on the function the component instance uses to enable RL detection mode */
    pwmc_generic_cb_t fct_rl_detection_mode_disable; /**< pointer on the function the component instance uses to disable RL detection mode */
    pwmc_rl_detect_set_duty_cb_t fct_rl_detection_mode_set_duty; /**< pointer on the function the component instance uses to set the PWM duty cycle in RL detection mode */
    /** @} */
    uint16_t t_sqrt3;  /**< a constant utilized by PWM algorithm (@f$\sqrt{3}@f$) */
    uint16_t cnt_pha;  /**< PWM Duty cycle for phase A */
    uint16_t cnt_phb;  /**< PWM Duty cycle for phase B */
    uint16_t cnt_phc;  /**< PWM Duty cycle for phase C */
    uint16_t sw_error; /**< Contains status about SW error */
    uint8_t Sector;    /**< the space vector sector number */
    uint16_t low_duty;
    uint16_t mid_duty;
    uint16_t high_duty;
    bool turn_on_low_sides_action;         /**< true if TurnOnLowSides action is active,
                                                              false otherwise. */
    uint16_t off_calibr_wait_time_counter; /**< Counter to wait fixed time before motor
                                                              current measurement offset calibration. */
    uint8_t motor;                         /**< motor reference number */
    bool rl_detection_mode;                /**< true if enabled, false if disabled. */
    int16_t ia;                            /**< Last @f$I_{A}@f$ measurement. */
    int16_t ib;                            /**< Last @f$I_{B}@f$ measurement. */
    int16_t ic;                            /**< Last @f$I_{C}@f$ measurement. */
    uint16_t dt_test;                      /**< Reserved */

    /* former  PWMnCurrFdbkParams_t */
    uint16_t pwm_period;            /**< PWM period expressed in timer clock cycles unit:
                                     *  @f$hpwm_period = TimerFreq_{CLK} / F_{PWM}@f$    */
    uint16_t off_calibr_wait_ticks; /**< Wait time duration before current reading
                                     *  calibration expressed in number of calls
                                     *  of pwmc_current_reading_calibr() with action
                                     *  #CRC_EXEC */
    uint16_t dt_comp_cnt;           /**< Half of Dead time expressed
                                     *  in timer clock cycles unit:
                                     *  @f$hdt_comp_cnt = (DT_s \cdot TimerFreq_{CLK})/2@f$ */
    uint16_t ton;                   /**< Reserved */
    uint16_t toff;                  /**< Reserved */
};

/**
 * @brief  Current reading calibration definition
 */
typedef enum crc_action {
    CRC_START, /**< Initialize the current reading calibration.*/
    CRC_EXEC   /**< Execute the current reading calibration.*/
} crc_action_t;

/* Used to get the motor phase current in ElectricalValue format as read by AD converter */
void pwmc_get_phase_currents(pwmc_t* phandle, ab_t* iab);

/*  Converts input voltage components Valfa, beta into duty cycles and feed it to the inverter */
uint16_t pwmc_set_phase_voltage(pwmc_t* phandle, alphabeta_t valfa_beta);

/* Switches the PWM generation off, setting the outputs to inactive */
void pwmc_switch_off_pwm(pwmc_t* phandle);

/* Switches the PWM generation on */
void pwmc_switch_on_pwm(pwmc_t* phandle);

/**
 * Calibrates ADC current conversions by reading the offset voltage
 * present on ADC pins when no motor current is flowing.
 * This function should be called before each motor start-up */
bool pwmc_current_reading_calibr(pwmc_t* phandle, crc_action_t action);

/**
 *  Turns low sides on. This function is intended to be used for
 *  charging boot capacitors of driving section. It has to be called on each
 *  motor start-up when using high voltage drivers.
 * */
void PWMC_TurnOnLowSides(pwmc_t* phandle);

/**
 *  Executes a regular conversion using the first ADC used for current reading.
 * The function is not re-entrant (it cannot executed twice at the same time).
 * Returns 0xFFFF in case of conversion error.
 * */
uint16_t pwmc_exec_regular_conv(pwmc_t* phandle, uint8_t channel);

/**
 *  Sets the specified sampling time for the specified ADC channel on the first ADC used for current
 *  reading. Must be called once for each channel utilized by user */
void pwmc_adc_set_sampling_time(pwmc_t* phandle, ad_conv_t ad_conv_struct);

/* Checks if an over current occurred since last call. */
uint16_t pwmc_check_over_current(pwmc_t* phandle);

/* Sets the over current threshold through the DAC reference voltage. */
void pwmc_ocp_set_reference_voltage(pwmc_t* phandle, uint16_t dac_vref);

/* Retrieves the status of the "TurnOnLowSides" action. */
bool pwmc_getturn_on_low_sides_action(pwmc_t* phandle);

/* Enables the RL Detection mode. */
void pwmc_rl_detection_mode_enable(pwmc_t* phandle);

/* Disables the RL Detection mode and sets the standard PWM. */
void pwmc_rl_detection_mode_disable(pwmc_t* phandle);

/* Sets the PWM duty cycle in the RL Detection mode. */
uint16_t pwmc_rl_detection_mode_set_duty(pwmc_t* phandle, uint16_t duty);

/* Sets the Callback that the PWMC component shall invoke to get phases current. */
void pwmc_register_get_phase_currents_callback(pwmc_get_phase_curr_cb_t ptr_callback, pwmc_t* phandle);

/**
 *  Sets the Callback that the PWMC component shall invoke to switch off PWM
 *  generation.
 * */
void pwmc_register_switch_off_pwm_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle);

/**
 *  Sets the Callback that the PWMC component shall invoke to switch on PWM
 *  generation.
 * */
void pwmc_register_switch_on_pwm_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle);

/**
 * Sets the Callback that the PWMC component shall invoke to execute a calibration
 * of the current sensing system.
 * */
void pwmc_register_reading_calibration_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle);

/* Sets the Callback that the PWMC component shall invoke to turn on low sides. */
void pwmc_register_turn_on_low_sides_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle);

/* Sets the Callback that the PWMC component shall invoke to ADC sampling point for sector 1. */
void pwmc_register_samp_point_sectx_callback(pwmc_set_samp_point_sectx_cb_t ptr_callback, pwmc_t* phandle);

/* Sets the Callback that the PWMC component shall invoke to the over current status. */
void pwmc_registerIs_over_current_occurred_callback(pwmc_over_curr_cb_t ptr_callback, pwmc_t* phandle);
/* Sets the Callback that the PWMC component shall invoke to set the reference voltage for the over current protection */
void pwmc_register_ocp_set_ref_voltage_callback(pwmc_set_ocp_ref_volt_cb_t ptr_callback, pwmc_t* phandle);

/* Sets the Callback that the PWMC component shall invoke to set the R/L detection mode */
void pwmc_register_rl_detection_mode_enable_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle);

/* Sets the Callback that the PWMC component shall invoke to disable the R/L detection mode */
void pwmc_register_rl_detection_mode_disable_callback(pwmc_generic_cb_t ptr_callback, pwmc_t* phandle);

/* Sets the Callback that the PWMC component shall invoke to set the duty cycle for the R/L detection mode */
void pwmc_register_rl_detection_mode_set_duty_callback(pwmc_rl_detect_set_duty_cb_t ptr_callback, pwmc_t* phandle);

/* Sets the Callback that the PWMC component shall invoke to call PWMC instance IRQ handler */
void pwmc_register_irq_handler_callback(pwmc_irq_handler_cb_t ptr_callback, pwmc_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PWMNCURRFDBK_H */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
