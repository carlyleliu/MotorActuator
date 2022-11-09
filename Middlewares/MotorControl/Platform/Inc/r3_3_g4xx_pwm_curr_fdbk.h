/**
 ******************************************************************************
 * @file    r3_3_g4xx_pwm_curr_fdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          R3_3_G4XX_pwm_curr_fdbk component of the motor Control SDK.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMicROELECTRONicS AND CONTRibUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTicULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMicROELECTRONicS OR CONTRibUTORS BE LiaBLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECiaL, EXEMPLARY, OR CONSEQUENTiaL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVicES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LiaBILITY, WHETHER IN CONTRACT, STRicT LiaBILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSibILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 * @ingroup R3_3_G4XX_pwm_curr_fdbk
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_3_G4XX_PWMNCURRFDBK_H
#define __R3_3_G4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

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

/** @addtogroup R3_3_G4XX_pwm_curr_fdbk
 * @{
 */

/* Exported types ------------------------------------------------------- */

/**
 * @brief  R3_3_G4XX_pwm_curr_fdbk component OPAMP parameters definition
 */
typedef const struct {
    /* First OPAMP settings ------------------------------------------------------*/
    OPAMP_TypeDef* opamp_pha; /* OPAMP dedicated to phase A */
    OPAMP_TypeDef* opamp_phb; /* OPAMP dedicated to phase B */
    OPAMP_TypeDef* opamp_phc; /* OPAMP dedicated to phase C */

    uint32_t opamp_inver_ting_input; /*!< First OPAMP inverting input pin.
                                      It must be one of the following:
                                      OPAMP1_InvertingInput_PC5 or
                                      OPAMP1_InvertingInput_PA3 if the
                                      bOPAMP_InvertingInput_MODE is
                                      EXT_MODE or
                                      OPAMP1_InvertingInput_PGA or
                                      OPAMP1_InvertingInput_FOLLOWER if the
                                      bOPAMP_InvertingInput_MODE is
                                      INT_MODE.*/

    uint32_t opamp_non_inver_ting_input_pha;
    uint32_t opamp_non_inver_ting_input_phb;
    uint32_t opamp_non_inver_ting_input_phc;

} r3_3_g4xx_opamp_params_t, *ptr_r3_3_g4xx_opamp_params_t;

/**
 * @brief  r3_4_f30X_pwm_curr_fdbk component parameters definition
 */
typedef const struct {
    /* Dual MC parameters --------------------------------------------------------*/
    uint8_t freq_ratio;          /*!< It is used in case of dual MC to
                                  synchronize TIM1 and TIM8. It has
                                  effect only on the second instanced
                                  object and must be equal to the
                                  ratio between the two PWM frequencies
                                  (higher/lower). Supported values are
                                  1, 2 or 3 */
    uint8_t bis_higher_freq_tim; /*!< When freq_ratio is greather than 1
                                  this param is used to indicate if this
                                  instance is the one with the highest
                                  frequency. Allowed value are: HIGHER_FREQ
                                  or LOWER_FREQ */
    /* Current reading A/D Conversions initialization -----------------------------*/
    ADC_TypeDef* adcx_a; /*!< First ADC peripheral to be used.*/
    ADC_TypeDef* adcx_b; /*!< Second ADC peripheral to be used.*/
    ADC_TypeDef* adcx_c; /*!< Second ADC peripheral to be used.*/

    /* PWM generation parameters --------------------------------------------------*/
    uint8_t repetition_counter; /*!< It expresses the number of PWM
                                 periods to be elapsed before compare
                                 registers are updated again. In
                                 particular:
                                 repetition_counter= (2* #PWM periods)-1*/
    uint16_t ht_after;          /*!< It is the sum of dead time plus max
                                 value between rise time and noise time
                                 express in number of TIM clocks.*/
    uint16_t ht_before;         /*!< It is the sampling time express in
                                 number of TIM clocks.*/
    TIM_TypeDef* TIMx;          /*!< It contains the pointer to the timer
                                 used for PWM generation. It must
                                 equal to TIM1 if binstance_nbr is
                                 equal to 1, to TIM8 otherwise */
    /* PWM Driving signals initialization ----------------------------------------*/
    low_side_outputs_function_t low_side_outputs; /*!< Low side or enabling signals
                                                   generation method are defined
                                                   here.*/

    GPIO_TypeDef* pwm_en_u_port; /*!< Channel 1N (low side) GPIO output
                                  port (if used, after re-mapping).
                                  It must be GPIOx x= A, B, ...*/
    uint16_t pwm_en_u_pin;       /*!< Channel 1N (low side) GPIO output pin
                                  (if used, after re-mapping). It must be
                                  GPIO_Pin_x x= 0, 1, ...*/

    GPIO_TypeDef* pwm_en_v_port; /*!< Channel 2N (low side) GPIO output
                                  port (if used, after re-mapping).
                                  It must be GPIOx x= A, B, ...*/
    uint16_t pwm_en_v_pin;       /*!< Channel 2N (low side) GPIO output pin
                                  (if used, after re-mapping). It must be
                                  GPIO_Pin_x x= 0, 1, ...*/

    GPIO_TypeDef* pwm_en_w_port; /*!< Channel 3N (low side)  GPIO output
                                  port (if used, after re-mapping).
                                  It must be GPIOx x= A, B, ...*/
    uint16_t pwm_en_w_pin;       /*!< Channel 3N (low side)  GPIO output pin
                                  (if used, after re-mapping). It must be
                                  GPIO_Pin_x x= 0, 1, ...*/

    /* Emergency input (BKIN2) signal initialization -----------------------------*/
    uint8_t bbkin2_mode; /*!< It defines the modality of emergency
                          input 2. It must be any of the
                          the following:
                          NONE - feature disabled.
                          INT_MODE - Internal comparator used
                          as source of emergency event.
                          EXT_MODE - External comparator used
                          as source of emergency event.*/

    /* Internal OPAMP common settings --------------------------------------------*/
    ptr_r3_3_g4xx_opamp_params_t ptr_opamp_params; /*!< Pointer to the OPAMP params struct.
                                                    It must be MC_NULL if internal
                                                    OPAMP are not used.*/
    /* Internal COMP settings ----------------------------------------------------*/
    COMP_TypeDef* comp_ocp_a_selection; /*!< Internal comparator used for protection.
                                         It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
    uint8_t comp_ocp_a_invInput_mode;   /*!< COMPx inverting input mode. It must be either
                                         equal to EXT_MODE or INT_MODE. */
    COMP_TypeDef* comp_ocp_b_selection; /*!< Internal comparator used for protection.
                                         It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
    uint8_t comp_ocp_b_invInput_mode;   /*!< COMPx inverting input mode. It must be either
                                         equal to EXT_MODE or INT_MODE. */
    COMP_TypeDef* comp_ocp_c_selection; /*!< Internal comparator used for protection.
                                         It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
    uint8_t comp_ocp_c_invInput_mode;   /*!< COMPx inverting input mode. It must be either
                                         equal to EXT_MODE or INT_MODE. */
    COMP_TypeDef* comp_ovp_selection;   /*!< Internal comparator used for protection.
                                         It must be COMP_Selection_COMPx x = 1,2,3,4,5,6,7.*/
    uint8_t comp_ovp_invInput_mode;     /*!< COMPx inverting input mode. It must be either
                                         equal to EXT_MODE or INT_MODE. */
    /* DAC settings --------------------------------------------------------------*/
    uint16_t dac_ocp_threshold; /*!< Value of analog reference expressed
                                 as 16bit unsigned integer.
                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
    uint16_t dac_ovp_threshold; /*!< Value of analog reference expressed
                                 as 16bit unsigned integer.
                                 Ex. 0 = 0V 65536 = VDD_DAC.*/
    /* Regular conversion --------------------------------------------------------*/
    ADC_TypeDef* regconv_adcx; /*!< ADC peripheral used for regular
                                conversion.*/
} r3_3_g4xx_params_t, *ptr_r3_3_g4xx_params_t;

/**
 * @brief  This structure is used to handle an instance of the
 *         r3_4_f30X_pwm_curr_fdbk component.
 */
typedef struct {
    pwmc_t _Super;           /*!<   */
    uint32_t phase_a_offset; /*!< Offset of Phase A current sensing network  */
    uint32_t phase_b_offset; /*!< Offset of Phase B current sensing network  */
    uint32_t phase_c_offset; /*!< Offset of Phase C current sensing network  */
    uint32_t adc_jsqr_ph_a;
    uint32_t adc_jsqr_ph_b;
    uint32_t adc_jsqr_ph_c;
    uint32_t oamp1_cr; /*!< OAMP1 control register to select channel current
                        sampling */
    uint32_t oamp2_cr; /*!< OAMP2 control register to select channel current
                        sampling */

    uint16_t half_pwm_period; /*!< Half PWM Period in timer clock counts */
    uint16_t reg_conv;        /*!< Variable used to store regular conversions
                               result*/
    volatile uint8_t so_foc;  /*!< This flag is reset at the beginning of FOC
                               and it is set in the TIM UP IRQ. If at the end of
                               FOC this flag is set, it means that FOC rate is too
                               high and thus an error is generated */
    uint8_t index;
    uint16_t adc_external_trigger_injected;
    uint16_t adc_external_polarity_injected;
    /*!< Trigger selection for ADC peripheral.*/
    bool over_current_flag; /*!< This flag is set when an overcurrent occurs.*/
    bool over_voltage_flag; /*!< This flag is set when an overvoltage occurs.*/
    bool brake_action_lock; /*!< This flag is set to avoid that brake action is
                             interrupted.*/
    ptr_r3_3_g4xx_params_t ptr_params_str;
} pwmc_r3_3_g4_t;

/* Exported functions ------------------------------------------------------- */

/**
 * It initializes TIMx, ADC, GPIO, DMA1 and NVic for current reading
 * in three shunt topology using STM32F30X and shared ADC
 */
void r3_3_g4xx_init(pwmc_r3_3_g4_t* phandle);

/**
 * It stores into the handle the voltage present on ia and
 * ib current feedback analog channels when no current is flowin into the
 * motor
 */
void r3_3_g4xx_current_reading_calibration(pwmc_t* pHdl);

/**
 * It computes and return latest converted motor phase currents motor
 *
 */
void r3_3_g4xx_get_phase_currents(pwmc_t* pHdl, Curr_Components* pStator_Currents);

/**
 * It turns on low sides switches. This function is intended to be
 * used for charging boot capacitors of driving section. It has to be
 * called each motor start-up when using high voltage drivers
 */
void r3_3_g4xx_turn_on_low_sides(pwmc_t* pHdl);

/**
 * It enables PWM generation on the proper Timer peripheral acting on MOE
 * bit
 */
void r3_3_g4xx_switch_on_pwm(pwmc_t* pHdl);

/**
 * It disables PWM generation on the proper Timer peripheral acting on
 * MOE bit
 */
void r3_3_g4xx_switch_off_pwm(pwmc_t* pHdl);

/**
 * Configure the ADC for the current sampling related to sector 1.
 * It means set the sampling point via TIMx_Ch4 value and polarity
 * ADC sequence length and channels.
 * And call the WriteTIMRegisters method.
 */
uint16_t r3_3_g4xx_set_adc_samp_point_sect1(pwmc_t* pHdl);

/**
 * Configure the ADC for the current sampling related to sector 2.
 * It means set the sampling point via TIMx_Ch4 value and polarity
 * ADC sequence length and channels.
 * And call the WriteTIMRegisters method.
 */
uint16_t r3_3_g4xx_set_adc_samp_point_sect2(pwmc_t* pHdl);

/**
 * Configure the ADC for the current sampling related to sector 3.
 * It means set the sampling point via TIMx_Ch4 value and polarity
 * ADC sequence length and channels.
 * And call the WriteTIMRegisters method.
 */
uint16_t r3_3_g4xx_set_adc_samp_point_sect3(pwmc_t* pHdl);

/**
 * Configure the ADC for the current sampling related to sector 4.
 * It means set the sampling point via TIMx_Ch4 value and polarity
 * ADC sequence length and channels.
 * And call the WriteTIMRegisters method.
 */
uint16_t r3_3_g4xx_set_adc_samp_point_sect4(pwmc_t* pHdl);

/**
 * Configure the ADC for the current sampling related to sector 5.
 * It means set the sampling point via TIMx_Ch4 value and polarity
 * ADC sequence length and channels.
 * And call the WriteTIMRegisters method.
 */
uint16_t r3_3_g4xx_set_adc_samp_point_sect5(pwmc_t* pHdl);

/**
 * Configure the ADC for the current sampling related to sector 6.
 * It means set the sampling point via TIMx_Ch4 value and polarity
 * ADC sequence length and channels.
 * And call the WriteTIMRegisters method.
 */
uint16_t r3_3_g4xx_set_adc_samp_point_sect6(pwmc_t* pHdl);

/**
 *  It contains the TIMx Update event interrupt
 */
void* r3_3_g4xx_timx_up_irq_handler(pwmc_r3_3_g4_t* pHdl);

/**
 *  It contains the TIMx Break2 event interrupt
 */
void* r3_3_g4xx_brk2_irq_handler(pwmc_r3_3_g4_t* pHdl);

/**
 *  It contains the TIMx Break1 event interrupt
 */
void* r3_3_g4xx_brk_irq_handler(pwmc_r3_3_g4_t* pHdl);

/**
 * Execute a regular conversion using ADCx.
 * The function is not re-entrant (can't executed twice at the same time)
 */
uint16_t r3_3_g4xx_exec_regular_conv(pwmc_t* pHdl, uint8_t channel);

/**
 * It sets the specified sampling time for the specified ADC channel
 * on ADCx. It must be called once for each channel utilized by user
 */
void r3_3_g4xx_adc_set_sampling_time(pwmc_t* pHdl, ad_conv_t ad_conv_struct);

/**
 * It is used to check if an overcurrent occurred since last call.
 */
uint16_t r3_3_g4xx_is_over_current_occurred(pwmc_t* pHdl);

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

#endif /*__R3_3_G4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
