/**
 ******************************************************************************
 * @file    R3_2_g4xx_pwm_curr_fdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          R3_2_G4XX_pwm_curr_fdbk component of the motor Control SDK.
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
 * @ingroup R3_2_G4XX_pwm_curr_fdbk
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef R3_2_G4XX_PWMNCURRFDBK_H
#define R3_2_G4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

#define NB_CONVERSIONS 16u

/* Exported defines --------------------------------------------------------*/
#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t)1)
#define NO_SHIFTED_TIMs   ((uint8_t)0)

#define NONE     ((uint8_t)(0x00))
#define EXT_MODE ((uint8_t)(0x01))
#define INT_MODE ((uint8_t)(0x02))

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup pwm_curr_fdbk
 * @{
 */

/** @addtogroup R3_2_G4XX_pwm_curr_fdbk
 * @{
 */

/* Exported types ------------------------------------------------------- */

/**
 * @brief  R3_2_G4XX_pwm_curr_fdbk component OPAMP parameters definition
 */
typedef const struct {
    /* First OPAMP settings ------------------------------------------------------*/
    OPAMP_TypeDef* OPAMPx_1;         /* OPAMP dedicated to phase A */
    OPAMP_TypeDef* OPAMPx_2;         /* OPAMP dedicated to phase B */
    OPAMP_TypeDef* OPAMPx_3;         /* OPAMP dedicated to phase C */
    OPAMP_TypeDef* OPAMPSelect_1[6]; /*!< define for each sector first conversion which OPAMP is involved - Null otherwise */
    OPAMP_TypeDef* OPAMPSelect_2[6]; /*!< define for each sector second conversion which OPAMP is involved - Null otherwise */
    uint32_t opamp_config1[6]; /*!< Define the OPAMP_CSR_OPAMPINTEN and the OPAMP_CSR_VPSEL config for each ADC conversions*/
    uint32_t opamp_config2[6]; /*!< Define the OPAMP_CSR_OPAMPINTEN and the OPAMP_CSR_VPSEL config for each ADC conversions*/
} r3_3_opamp_params_t;

/**
 * @brief  r3_4_f30X_pwm_curr_fdbk component parameters definition
 */
typedef struct {
    /* hw IP involved -----------------------------*/
    ADC_TypeDef* ADCx_1;                 /*!< First ADC peripheral to be used.*/
    ADC_TypeDef* ADCx_2;                 /*!< Second ADC peripheral to be used.*/
    TIM_TypeDef* TIMx;                   /*!< timer used for PWM generation.*/
    r3_3_opamp_params_t* opamp_params;   /*!< Pointer to the OPAMP params struct.
                                          It must be MC_NULL if internal OPAMP are not used.*/
    COMP_TypeDef* comp_ocp_a_selection;  /*!< Internal comparator used for Phase A protection.*/
    COMP_TypeDef* comp_ocp_b_selection;  /*!< Internal comparator used for Phase B protection.*/
    COMP_TypeDef* comp_ocp_c_selection;  /*!< Internal comparator used for Phase C protection.*/
    COMP_TypeDef* comp_ovp_selection;    /*!< Internal comparator used for Over Voltage protection.*/
    GPIO_TypeDef* pwm_en_u_port;         /*!< Channel 1N (low side) GPIO output */
    GPIO_TypeDef* pwm_en_v_port;         /*!< Channel 2N (low side) GPIO output*/
    GPIO_TypeDef* pwm_en_w_port;         /*!< Channel 3N (low side)  GPIO output */
    DAC_TypeDef* dac_ocp_a_selection;    /*!< DAC used for Phase A protection.*/
    DAC_TypeDef* dac_ocp_b_selection;    /*!< DAC used for Phase B protection.*/
    DAC_TypeDef* dac_ocp_c_selection;    /*!< DAC used for Phase C protection.*/
    DAC_TypeDef* dac_ovp_selection;      /*!< DAC used for Over Voltage protection.*/
    uint32_t dac_channel_ocp_a;          /*!< DAC channel used for Phase A current protection.*/
    uint32_t dac_channel_ocp_b;          /*!< DAC channel used for Phase B current protection.*/
    uint32_t dac_channel_ocp_c;          /*!< DAC channel used for Phase C current protection.*/
    uint32_t dac_channel_ovp;            /*!< DAC channel used for Over Voltage protection.*/
    uint32_t volatile* adc_data_reg1[6]; /*!< Contains the Address of ADC read value for one phase
                                          and all the 6 sectors */
    uint32_t volatile* adc_data_reg2[6]; /*!< Contains the Address of ADC read value for one phase
                                          and all the 6 sectors */
    uint32_t adc_config1[6];             /*!< values of JSQR for first ADC for 6 sectors */
    uint32_t adc_config2[6];             /*!< values of JSQR for Second ADC for 6 sectors */

    uint16_t pwm_en_u_pin; /*!< Channel 1N (low side) GPIO output pin */
    uint16_t pwm_en_v_pin; /*!< Channel 2N (low side) GPIO output pin */
    uint16_t pwm_en_w_pin; /*!< Channel 3N (low side)  GPIO output pin */

    /* PWM generation parameters --------------------------------------------------*/

    uint16_t t_after;  /*!< It is the sum of dead time plus max
                        value between rise time and noise time
                        express in number of TIM clocks.*/
    uint16_t t_before; /*!< It is the sampling time express in
                        number of TIM clocks.*/

    /* DAC settings --------------------------------------------------------------*/
    uint16_t dac_ocp_threshold; /*!< Value of analog reference expressed
                                 as 16bit unsigned integer.
                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
    uint16_t dac_ovp_threshold; /*!< Value of analog reference expressed
                                 as 16bit unsigned integer.
                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
    /* PWM Driving signals initialization ----------------------------------------*/
    low_side_outputs_function_t low_side_outputs; /*!< Low side or enabling signals
                                                   generation method are defined
                                                   here.*/

    uint8_t repetition_counter; /*!< It expresses the number of PWM
                                 periods to be elapsed before compare
                                 registers are updated again. In
                                 particular:
                                 repetition_counter= (2* #PWM periods)-1*/
    /* Emergency input (BKIN2) signal initialization -----------------------------*/
    uint8_t bkin2_mode; /*!< It defines the modality of emergency
                         input 2. It must be any of the
                         the following:
                         NONE - feature disabled.
                         INT_MODE - Internal comparator used
                         as source of emergency event.
                         EXT_MODE - External comparator used
                         as source of emergency event.*/

    /* Internal COMP settings ----------------------------------------------------*/
    uint8_t comp_ocp_a_invInput_mode; /*!< COMPx inverting input mode. It must be either
                                       equal to EXT_MODE or INT_MODE. */
    uint8_t comp_ocp_b_invInput_mode; /*!< COMPx inverting input mode. It must be either
                                       equal to EXT_MODE or INT_MODE. */
    uint8_t comp_ocp_c_invInput_mode; /*!< COMPx inverting input mode. It must be either
                                       equal to EXT_MODE or INT_MODE. */
    uint8_t comp_ovp_invInput_mode;   /*!< COMPx inverting input mode. It must be either
                                       equal to EXT_MODE or INT_MODE. */

    /* Dual MC parameters --------------------------------------------------------*/
    uint8_t freq_ratio;         /*!< It is used in case of dual MC to
                                 synchronize TIM1 and TIM8. It has
                                 effect only on the second instanced
                                 object and must be equal to the
                                 ratio between the two PWM frequencies
                                 (higher/lower). Supported values are
                                 1, 2 or 3 */
    uint8_t is_higher_freq_tim; /*!< When freq_ratio is greather than 1
                                 this param is used to indicate if this
                                 instance is the one with the highest
                                 frequency. Allowed value are: HIGHER_FREQ
                                 or LOWER_FREQ */

} r3_2_params_t, *ptr_r3_2_params_t;

/**
 * @brief  This structure is used to handle an instance of the
 *         r3_4_f30X_pwm_curr_fdbk component.
 */
typedef struct {
    pwmc_t _Super;            /*!<   */
    uint32_t phase_a_offset;  /*!< Offset of Phase A current sensing network  */
    uint32_t phase_b_offset;  /*!< Offset of Phase B current sensing network  */
    uint32_t phase_c_offset;  /*!< Offset of Phase C current sensing network  */
    uint16_t half_pwm_period; /*!< Half PWM Period in timer clock counts */
    uint16_t adc_external_polarity_injected;
    uint8_t polarization_counter;
    uint8_t polarization_sector; /*!< Sector selected during calibration phase */
    /*!< Trigger selection for ADC peripheral.*/
    bool over_current_flag; /*!< This flag is set when an overcurrent occurs.*/
    bool over_voltage_flag; /*!< This flag is set when an overvoltage occurs.*/
    bool brake_action_lock; /*!< This flag is set to avoid that brake action is
                             interrupted.*/
    ptr_r3_2_params_t ptr_params_str;
    bool adc_regular_locked; /* Cut 2.2 patch*/
} pwmc_r3_2_t;

/* Exported functions ------------------------------------------------------- */

/**
 * It initializes TIMx, ADC, GPIO, DMA1 and NVic for current reading
 * in three shunt topology using STM32F30X and shared ADC
 */
void r3_2_init(pwmc_r3_2_t* phandle);

/**
 * It stores into the handle the voltage present on ia and
 * ib current feedback analog channels when no current is flowin into the
 * motor
 */
void r3_2_current_reading_polarization(pwmc_t* pHdl);

/**
 * It computes and return latest converted motor phase currents motor
 *
 */
void r3_2_get_phase_currents(pwmc_t* pHdl, ab_t* iab);

/**
 * It turns on low sides switches. This function is intended to be
 * used for charging boot capacitors of driving section. It has to be
 * called each motor start-up when using high voltage drivers
 */
void r3_2_turn_on_low_sides(pwmc_t* pHdl);

/**
 * It enables PWM generation on the proper Timer peripheral acting on MOE
 * bit
 */
void r3_2_switch_on_pwm(pwmc_t* pHdl);

/**
 * It disables PWM generation on the proper Timer peripheral acting on
 * MOE bit
 */
void r3_2_switch_off_pwm(pwmc_t* pHdl);

/**
 * Configure the ADC for the current sampling
 * It means set the sampling point via TIMx_Ch4 value and polarity
 * ADC sequence length and channels.
 * And call the WriteTIMRegisters method.
 */
uint16_t r3_2_set_adc_samp_point_sectx(pwmc_t* pHdl);

/**
 *  It contains the TIMx Update event interrupt
 */
void* r3_2_timx_up_irq_handler(pwmc_r3_2_t* pHdl);

/**
 *  It contains the TIMx Break2 event interrupt
 */
void* r3_2_brk2_irq_handler(pwmc_r3_2_t* pHdl);

/**
 *  It contains the TIMx Break1 event interrupt
 */
void* r3_2_brk_irq_handler(pwmc_r3_2_t* pHdl);

/**
 * It is used to check if an overcurrent occurred since last call.
 */
uint16_t r3_2_is_over_current_occurred(pwmc_t* pHdl);

/**
 * It is used to set the PWM mode for R/L detection.
 */
void r3_2_rl_detection_mode_enable(pwmc_t* pHdl);

/**
 * It is used to disable the PWM mode for R/L detection.
 */
void r3_2_rl_detection_mode_disable(pwmc_t* pHdl);

/**
 * It is used to set the PWM dutycycle for R/L detection.
 */
uint16_t r3_2_rl_detection_mode_set_duty(pwmc_t* pHdl, uint16_t duty);

/**
 * @brief  It turns on low sides switches and start ADC triggering.
 *         This function is specific for MP phase.
 */
void rl_turn_on_low_sides_and_start(pwmc_t* pHdl);

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

#endif /*R3_2_G4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
