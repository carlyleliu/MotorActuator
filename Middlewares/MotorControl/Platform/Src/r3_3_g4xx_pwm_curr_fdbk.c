/**
 ******************************************************************************
 * @file    r3_3_g4xx_pwm_curr_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement current sensor
 *          class to be stantiated when the three shunts current sensing
 *          topology is used. It is specifically designed for STM32F30X
 *          microcontrollers and implements the successive sampling of two motor
 *          current using shared ADC.
 *           + MCU peripheral and handle initialization function
 *           + three shunt current sesnsing
 *           + space vector modulation function
 *           + ADC sampling function
 *
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
 */

/* Includes ------------------------------------------------------------------*/
#include "r3_3_g4xx_pwm_curr_fdbk.h"

#include "mc_type.h"
#include "pwm_common.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup pwm_curr_fdbk
 * @{
 */

/**
 * @defgroup R3_3_G4XX_pwm_curr_fdbk R3 2 ADCs F30x PWM & Current Feedback
 *
 * @brief STM32F3, Shared Resources, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F3 MCU, using a three
 * shunt resistors current sensing topology and 2 ADC peripherals to acquire the current
 * values.
 *
 * It is designed to be used in applications that drive two motors in which case two instances of
 * the component are used that share the 2 ADCs.
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123 \
    (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N)

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* These function overloads the TIM_BDTRConfig and TIM_BDTRStructInit
   of the standard library */
void r3_3_g4xx_timx_init(TIM_TypeDef* TIMx, pwmc_t* ptr_hdl);
uint16_t r3_3_g4xx_write_tim_registers(pwmc_t* ptr_hdl, uint16_t hCCR4Reg);
void r3_3_g4xx_hfcurrents_calibration_abc(pwmc_t* ptr_hdl, Curr_Components* pStator_Currents);
void r3_3_g4xx_set_aoreference_voltage(uint32_t DAC_Channel, uint16_t dac_vref);

/* Private functions ---------------------------------------------------------*/

/* Local redefinition of both LL_TIM_OC_EnablePreload & LL_TIM_OC_DisablePreload */
__STATIC_INLINE void __LL_TIM_OC_EnablePreload(TIM_TypeDef* TIMx, uint32_t Channel)
{
    register uint8_t i_channel = TIM_GET_CHANNEL_INDEX(Channel);
    register volatile uint32_t* ptr_reg = (uint32_t*)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[i_channel]));
    SET_BIT(*ptr_reg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[i_channel]));
}

__STATIC_INLINE void __LL_TIM_OC_DisablePreload(TIM_TypeDef* TIMx, uint32_t Channel)
{
    register uint8_t i_channel = TIM_GET_CHANNEL_INDEX(Channel);
    register volatile uint32_t* ptr_reg = (uint32_t*)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[i_channel]));
    CLEAR_BIT(*ptr_reg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[i_channel]));
}

/**
 * @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVic for current reading
 *         in three shunt topology using STM32F30X and shared ADC
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_3_g4xx_init(pwmc_r3_3_g4_t* phandle)
{
    ptr_r3_3_g4xx_opamp_params_t ptr_dopam_ptr_params_str = phandle->ptr_params_str->ptr_opamp_params;
    COMP_TypeDef* COMP_OCPAx = phandle->ptr_params_str->comp_ocp_a_selection;
    COMP_TypeDef* COMP_OCPBx = phandle->ptr_params_str->comp_ocp_b_selection;
    COMP_TypeDef* COMP_OCPCx = phandle->ptr_params_str->comp_ocp_c_selection;
    COMP_TypeDef* COMP_OVPx = phandle->ptr_params_str->comp_ovp_selection;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    if ((uint32_t)phandle == (uint32_t)&phandle->_Super) {
        /* disable IT and flags in case of LL driver usage
         * workaround for unwanted interrupt enabling done by LL driver */
        LL_ADC_DisableIT_EOC(phandle->ptr_params_str->adcx_a);
        LL_ADC_ClearFlag_EOC(phandle->ptr_params_str->adcx_a);
        LL_ADC_DisableIT_JEOC(phandle->ptr_params_str->adcx_a);
        LL_ADC_ClearFlag_JEOC(phandle->ptr_params_str->adcx_a);
        LL_ADC_DisableIT_EOC(phandle->ptr_params_str->adcx_b);
        LL_ADC_ClearFlag_EOC(phandle->ptr_params_str->adcx_b);
        LL_ADC_DisableIT_JEOC(phandle->ptr_params_str->adcx_b);
        LL_ADC_ClearFlag_JEOC(phandle->ptr_params_str->adcx_b);
        LL_ADC_DisableIT_EOC(phandle->ptr_params_str->adcx_c);
        LL_ADC_ClearFlag_EOC(phandle->ptr_params_str->adcx_c);
        LL_ADC_DisableIT_JEOC(phandle->ptr_params_str->adcx_c);
        LL_ADC_ClearFlag_JEOC(phandle->ptr_params_str->adcx_c);

        r3_3_g4xx_timx_init(TIMx, &phandle->_Super);

        if (TIMx == TIM1) {
            /* TIM1 Counter Clock stopped when the core is halted */
            LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
        } else {
            /* TIM8 Counter Clock stopped when the core is halted */
            LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
        }

        if (ptr_dopam_ptr_params_str) {
            LL_OPAMP_Enable(ptr_dopam_ptr_params_str->opamp_pha);
            LL_OPAMP_Enable(ptr_dopam_ptr_params_str->opamp_phb);
            LL_OPAMP_Enable(ptr_dopam_ptr_params_str->opamp_phc);
        }

        /* Over current protection phase A */
        if (COMP_OCPAx) {
            /* Inverting input*/
            if (phandle->ptr_params_str->comp_ocp_a_invInput_mode != EXT_MODE) {
                if (LL_COMP_GetInput_minus(COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH1) {
                    r3_3_g4xx_set_aoreference_voltage(LL_DAC_CHANNEL_1, (uint16_t)(phandle->ptr_params_str->dac_ocp_threshold));
                }
#if defined(DAC_CHANNEL2_SUPPORT)
                else if (LL_COMP_GetInput_minus(COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH2) {
                    r3_3_g4xx_set_aoreference_voltage(LL_DAC_CHANNEL_2, (uint16_t)(phandle->ptr_params_str->dac_ocp_threshold));
                }
#endif
                else {
                }
            }

            /* Wait to stabilize DAC voltage */
            volatile uint16_t waittime = 0u;
            for (waittime = 0u; waittime < 1000u; waittime++) {
            }

            /* Output */
            LL_COMP_Enable(COMP_OCPAx);
            LL_COMP_Lock(COMP_OCPAx);
        }

        /* Over current protection phase B */
        if (COMP_OCPBx) {
            LL_COMP_Enable(COMP_OCPBx);
            LL_COMP_Lock(COMP_OCPBx);
        }

        /* Over current protection phase C */
        if (COMP_OCPCx) {
            LL_COMP_Enable(COMP_OCPCx);
            LL_COMP_Lock(COMP_OCPCx);
        }

        /* Over voltage protection */
        if (COMP_OVPx) {
            /* Inverting input*/
            if (phandle->ptr_params_str->comp_ovp_invInput_mode != EXT_MODE) {
                if (LL_COMP_GetInput_minus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH1) {
                    r3_3_g4xx_set_aoreference_voltage(LL_DAC_CHANNEL_1, (uint16_t)(phandle->ptr_params_str->dac_ovp_threshold));
                }
#if defined(DAC_CHANNEL2_SUPPORT)
                else if (LL_COMP_GetInput_minus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH2) {
                    r3_3_g4xx_set_aoreference_voltage(LL_DAC_CHANNEL_2, (uint16_t)(phandle->ptr_params_str->dac_ovp_threshold));
                }
#endif
                else {
                }
            }

            /* Wait to stabilize DAC voltage */
            volatile uint16_t waittime = 0u;
            for (waittime = 0u; waittime < 1000u; waittime++) {
            }

            /* Output */
            LL_COMP_Enable(COMP_OVPx);
            LL_COMP_Lock(COMP_OVPx);
        }

        if (phandle->_Super.bmotor == M1) {
            /* - Exit from deep-power-down mode */
            LL_ADC_DisableDeepPowerDown(phandle->ptr_params_str->adcx_a);
            LL_ADC_DisableDeepPowerDown(phandle->ptr_params_str->adcx_b);
            LL_ADC_DisableDeepPowerDown(phandle->ptr_params_str->adcx_c);

            if (LL_ADC_IsInternalRegulatorEnabled(phandle->ptr_params_str->adcx_a) == 0) {
                /* Enable ADC internal voltage regulator */
                LL_ADC_EnableInternalRegulator(phandle->ptr_params_str->adcx_a);
                LL_ADC_EnableInternalRegulator(phandle->ptr_params_str->adcx_b);
                LL_ADC_EnableInternalRegulator(phandle->ptr_params_str->adcx_c);

                /* Wait for Regulator Startup time */
                /* Note: Variable divided by 2 to compensate partially              */
                /*       CPU processing cycles, scaling in us split to not          */
                /*       exceed 32 bits register capacity and handle low frequency. */
                uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
                while (wait_loop_index != 0UL) {
                    wait_loop_index--;
                }
            }

            LL_ADC_StartCalibration(phandle->ptr_params_str->adcx_a, LL_ADC_SINGLE_ENDED);
            while (LL_ADC_IsCalibrationOnGoing(phandle->ptr_params_str->adcx_a)) {
            }

            LL_ADC_StartCalibration(phandle->ptr_params_str->adcx_b, LL_ADC_SINGLE_ENDED);
            while (LL_ADC_IsCalibrationOnGoing(phandle->ptr_params_str->adcx_b)) {
            }

            LL_ADC_StartCalibration(phandle->ptr_params_str->adcx_c, LL_ADC_SINGLE_ENDED);
            while (LL_ADC_IsCalibrationOnGoing(phandle->ptr_params_str->adcx_c)) {
            }

            if ((phandle->ptr_params_str->regconv_adcx != phandle->ptr_params_str->adcx_a)
                && (phandle->ptr_params_str->regconv_adcx != phandle->ptr_params_str->adcx_b)
                && (phandle->ptr_params_str->regconv_adcx != phandle->ptr_params_str->adcx_c)) {
                {
                    LL_ADC_EnableInternalRegulator(phandle->ptr_params_str->regconv_adcx);

                    /* Wait for Regulator Startup time, once for both */
                    uint16_t waittime = 0u;
                    for (waittime = 0u; waittime < 65000u; waittime++) {
                    }

                    LL_ADC_StartCalibration(phandle->ptr_params_str->regconv_adcx, LL_ADC_SINGLE_ENDED);
                    while (LL_ADC_IsCalibrationOnGoing(phandle->ptr_params_str->regconv_adcx)) {
                    }
                }
            }

            /* Enable adcx_a, adcx_b and adcx_c*/
            LL_ADC_Enable(phandle->ptr_params_str->adcx_a);
            LL_ADC_Enable(phandle->ptr_params_str->adcx_b);
            LL_ADC_Enable(phandle->ptr_params_str->adcx_c);
        } else {
            /* already done by the first motor */
        }

        if ((phandle->ptr_params_str->regconv_adcx != phandle->ptr_params_str->adcx_a)
            && (phandle->ptr_params_str->regconv_adcx != phandle->ptr_params_str->adcx_b)
            && (phandle->ptr_params_str->regconv_adcx != phandle->ptr_params_str->adcx_c)) {
            LL_ADC_Enable(phandle->ptr_params_str->regconv_adcx);
        }

        /* reset regular conversion sequencer length set by cubeMX */
        LL_ADC_REG_SetSequencerLength(phandle->ptr_params_str->regconv_adcx, LL_ADC_REG_SEQ_SCAN_DISABLE);

        phandle->adc_jsqr_ph_a = phandle->ptr_params_str->adcx_a->JSQR;
        phandle->adc_jsqr_ph_b = phandle->ptr_params_str->adcx_b->JSQR;
        phandle->adc_jsqr_ph_c = phandle->ptr_params_str->adcx_c->JSQR;
        CLEAR_BIT(phandle->adc_jsqr_ph_a, ADC_JSQR_JEXTEN);
        CLEAR_BIT(phandle->adc_jsqr_ph_b, ADC_JSQR_JEXTEN);
        CLEAR_BIT(phandle->adc_jsqr_ph_c, ADC_JSQR_JEXTEN);

        LL_ADC_INJ_SetQueueMode(phandle->ptr_params_str->adcx_a, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
        LL_ADC_INJ_SetQueueMode(phandle->ptr_params_str->adcx_b, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
        LL_ADC_INJ_SetQueueMode(phandle->ptr_params_str->adcx_c, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);

        // phandle->ptr_params_str->adcx_a->JSQR = phandle->adc_jsqr_ph_a + LL_ADC_INJ_TRIG_EXT_RISING;
        // phandle->ptr_params_str->adcx_b->JSQR = phandle->adc_jsqr_ph_b + LL_ADC_INJ_TRIG_EXT_RISING;
        // phandle->ptr_params_str->adcx_c->JSQR = phandle->adc_jsqr_ph_c + LL_ADC_INJ_TRIG_EXT_RISING;

        LL_ADC_INJ_StartConversion(phandle->ptr_params_str->adcx_a);
        LL_ADC_INJ_StartConversion(phandle->ptr_params_str->adcx_b);
        LL_ADC_INJ_StartConversion(phandle->ptr_params_str->adcx_c);

        // LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH4 );
        // LL_TIM_OC_SetCompareCH4( TIMx, 0xFFFFu );
        // LL_TIM_OC_SetCompareCH4( TIMx, 0x0u );
        // LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH4 );

        // while ( LL_ADC_IsActiveFlag_JEOS( phandle->ptr_params_str->adcx_a ) == RESET )
        // {}
        // while ( LL_ADC_IsActiveFlag_JEOS( phandle->ptr_params_str->adcx_b ) == RESET )
        // {}
        // while ( LL_ADC_IsActiveFlag_JEOS( phandle->ptr_params_str->adcx_c ) == RESET )
        // {}

        // /* ADCx_ Injected conversions end interrupt enabling */
        // LL_ADC_ClearFlag_JEOS( phandle->ptr_params_str->adcx_a );
        // LL_ADC_ClearFlag_JEOS( phandle->ptr_params_str->adcx_b );
        // LL_ADC_ClearFlag_JEOS( phandle->ptr_params_str->adcx_c );

        /* TODO: check this It pending */
        NVic_ClearPendingIRQ(ADC3_IRQn);
        LL_ADC_EnableIT_JEOS(phandle->ptr_params_str->adcx_c);

        /* Clear the flags */
        phandle->over_voltage_flag = false;
        phandle->over_current_flag = false;

        phandle->_Super.dt_test = 0u;
        phandle->_Super.dt_comp_cnt = phandle->_Super.hdt_comp_cnt;
    }
}

/**
 * @brief  It initializes TIMx peripheral for PWM generation
 * @param TIMx: Timer to be initialized
 * @param phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_3_g4xx_timx_init(TIM_TypeDef* TIMx, pwmc_t* ptr_hdl)
{
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;

    /* disable main TIM counter to ensure
     * a synchronous start by TIM2 trigger */
    LL_TIM_DisableCounter(TIMx);

    /* Enables the TIMx Preload on CC1 Register */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
    /* Enables the TIMx Preload on CC2 Register */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
    /* Enables the TIMx Preload on CC3 Register */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
    /* Enables the TIMx Preload on CC4 Register */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

    LL_TIM_ClearFlag_BRK(TIMx);

    if ((phandle->ptr_params_str->bbkin2_mode) != NONE) {
        LL_TIM_ClearFlag_BRK2(TIMx);
    }
    LL_TIM_EnableIT_BRK(TIMx);

    /* Prepare timer for synchronization */
    LL_TIM_GenerateEvent_UPDATE(TIMx);
    if (phandle->ptr_params_str->freq_ratio == 2u) {
        if (phandle->ptr_params_str->bis_higher_freq_tim == HIGHER_FREQ) {
            if (phandle->ptr_params_str->repetition_counter == 3u) {
                /* Set TIMx repetition counter to 1 */
                LL_TIM_SetRepetitionCounter(TIMx, 1);
                LL_TIM_GenerateEvent_UPDATE(TIMx);
                /* Repetition counter will be set to 3 at next Update */
                LL_TIM_SetRepetitionCounter(TIMx, 3);
            }
        }
        LL_TIM_SetCounter(TIMx, (uint32_t)(phandle->half_pwm_period) - 1u);
    } else /* freq_ratio equal to 1 or 3 */
    {
        if (phandle->_Super.bmotor == M1) {
            if (phandle->ptr_params_str->repetition_counter == 1u) {
                LL_TIM_SetCounter(TIMx, (uint32_t)(phandle->half_pwm_period) - 1u);
            } else if (phandle->ptr_params_str->repetition_counter == 3u) {
                /* Set TIMx repetition counter to 1 */
                LL_TIM_SetRepetitionCounter(TIMx, 1);
                LL_TIM_GenerateEvent_UPDATE(TIMx);
                /* Repetition counter will be set to 3 at next Update */
                LL_TIM_SetRepetitionCounter(TIMx, 3);
            }
        }
    }
    /* Enable PWM channel */
    LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);
}

/**
 * @brief  It stores into the component the voltage present on ia and
 *         ib current feedback analog channels when no current is flowin into the
 *         motor
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_3_g4xx_current_reading_calibration(pwmc_t* ptr_hdl)
{
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    phandle->phase_a_offset = 0u;
    phandle->phase_b_offset = 0u;
    phandle->phase_c_offset = 0u;

    phandle->index = 0u;

    /* It forces inactive level on TIMx CHy and CHyN */
    TIMx->CCER &= (uint16_t)(~TIMxCCER_MASK_CH123);

    /* Offset calibration for all phases */
    /* Change function to be executed in ADCx_ISR */
    phandle->_Super.pFctGetphase_currents = &r3_3_g4xx_hfcurrents_calibration_abc;

    r3_3_g4xx_switch_on_pwm(&phandle->_Super);

    /* Wait for NB_CONVERSIONS to be executed */
    // wait_for_polarization_end(TIMx, &phandle->_Super.sw_error, phandle->ptr_params_str->repetition_counter, &phandle->index);

    r3_3_g4xx_switch_off_pwm(&phandle->_Super);

    phandle->phase_a_offset >>= 4;
    phandle->phase_b_offset >>= 4;
    phandle->phase_c_offset >>= 4;

    /* Change back function to be executed in ADCx_ISR */
    phandle->_Super.pFctGetphase_currents = &r3_3_g4xx_get_phase_currents;

    /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverer legs */
    /* Disable TIMx preload */
    LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_SetCompareCH1(TIMx, phandle->half_pwm_period);
    LL_TIM_OC_SetCompareCH2(TIMx, phandle->half_pwm_period);
    LL_TIM_OC_SetCompareCH3(TIMx, phandle->half_pwm_period);
    /* Enable TIMx preload */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

    /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
    TIMx->CCER |= 0x555u;

    phandle->brake_action_lock = false;
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It computes and return latest converted motor phase currents motor
 * @param  phandle: handler of the current instance of the PWM component
 * @retval ia and ib current in Curr_Components format
 */
__weak void r3_3_g4xx_get_phase_currents(pwmc_t* ptr_hdl, Curr_Components* pStator_Currents)
{
    uint8_t Sector;
    int32_t aux;
    uint16_t phase_a, phase_b, phase_c;
    static uint16_t i;

    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;

    /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
    phandle->so_foc = 0u;

    phase_a = (uint16_t)(phandle->ptr_params_str->adcx_a->JDR1);
    phase_b = (uint16_t)(phandle->ptr_params_str->adcx_b->JDR1);
    phase_c = (uint16_t)(phandle->ptr_params_str->adcx_c->JDR1);

    Sector = (uint8_t)phandle->_Super.hSector;

    switch (Sector) {
        case SECTOR_4:
        case SECTOR_5:
            /* Current on Phase C is not accessible     */
            /* ia = phase_a_offset - ADC converted value) */
            aux = (int32_t)(phandle->phase_a_offset) - (int32_t)(phase_a);

            /* Saturation of ia */
            if (aux < -INT16_MAX) {
                pStator_Currents->qI_Component1 = -INT16_MAX;
            } else if (aux > INT16_MAX) {
                pStator_Currents->qI_Component1 = INT16_MAX;
            } else {
                pStator_Currents->qI_Component1 = (int16_t)aux;
            }

            /* ib = phase_b_offset - ADC converted value) */
            aux = (int32_t)(phandle->phase_b_offset) - (int32_t)(phase_b);

            /* Saturation of ib */
            if (aux < -INT16_MAX) {
                pStator_Currents->qI_Component2 = -INT16_MAX;
            } else if (aux > INT16_MAX) {
                pStator_Currents->qI_Component2 = INT16_MAX;
            } else {
                pStator_Currents->qI_Component2 = (int16_t)aux;
            }
            break;

        case SECTOR_6:
        case SECTOR_1:
            /* Current on Phase A is not accessible     */
            /* ib = phase_b_offset - ADC converted value) */
            aux = (int32_t)(phandle->phase_b_offset) - (int32_t)(phase_b);

            /* Saturation of ib */
            if (aux < -INT16_MAX) {
                pStator_Currents->qI_Component2 = -INT16_MAX;
            } else if (aux > INT16_MAX) {
                pStator_Currents->qI_Component2 = INT16_MAX;
            } else {
                pStator_Currents->qI_Component2 = (int16_t)aux;
            }

            /* ia = -ic -ib */
            aux = (int32_t)(phase_c) - (int32_t)(phandle->phase_c_offset); /* -ic */
            aux -= (int32_t)pStator_Currents->qI_Component2;               /* ia  */

            /* Saturation of ia */
            if (aux > INT16_MAX) {
                pStator_Currents->qI_Component1 = INT16_MAX;
            } else if (aux < -INT16_MAX) {
                pStator_Currents->qI_Component1 = -INT16_MAX;
            } else {
                pStator_Currents->qI_Component1 = (int16_t)aux;
            }
            break;

        case SECTOR_2:
        case SECTOR_3:
            /* Current on Phase B is not accessible     */
            /* ia = phase_a_offset - ADC converted value) */
            aux = (int32_t)(phandle->phase_a_offset) - (int32_t)(phase_a);

            /* Saturation of ia */
            if (aux < -INT16_MAX) {
                pStator_Currents->qI_Component1 = -INT16_MAX;
            } else if (aux > INT16_MAX) {
                pStator_Currents->qI_Component1 = INT16_MAX;
            } else {
                pStator_Currents->qI_Component1 = (int16_t)aux;
            }

            /* ib = -ic -ia */
            aux = (int32_t)(phase_c) - (int32_t)(phandle->phase_c_offset); /* -ic */
            aux -= (int32_t)pStator_Currents->qI_Component1;               /* ib */

            /* Saturation of ib */
            if (aux > INT16_MAX) {
                pStator_Currents->qI_Component2 = INT16_MAX;
            } else if (aux < -INT16_MAX) {
                pStator_Currents->qI_Component2 = -INT16_MAX;
            } else {
                pStator_Currents->qI_Component2 = (int16_t)aux;
            }
            break;

        default:
            break;
    }

    phandle->_Super.hia = pStator_Currents->qI_Component1;
    phandle->_Super.hib = pStator_Currents->qI_Component2;
    phandle->_Super.hic = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;
}

/**
 * @brief  Implementaion of pwmc_get_phase_currents to be performed during
 *         calibration. It sum up injected conversion data into phase_a_offset and
 *         phase_b_offset to compute the offset introduced in the current feedback
 *         network. It is requied to proper configure ADC inputs before to enable
 *         the offset computation.
 * @param  phandle Pointer on the target component instance
 * @retval It always returns {0,0} in Curr_Components format
 */
__weak void r3_3_g4xx_hfcurrents_calibration_abc(pwmc_t* ptr_hdl, Curr_Components* pStator_Currents)
{
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;

    /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
    phandle->so_foc = 0u;

    if (phandle->index < NB_CONVERSIONS) {
        phandle->phase_a_offset += phandle->ptr_params_str->adcx_a->JDR1;
        phandle->phase_b_offset += phandle->ptr_params_str->adcx_b->JDR1;
        phandle->phase_c_offset += phandle->ptr_params_str->adcx_c->JDR1;
        phandle->index++;
    }

    /* during offset calibration no current is flowing in the phases */
    pStator_Currents->qI_Component1 = 0;
    pStator_Currents->qI_Component2 = 0;
}

/**
 * @brief  It turns on low sides switches. This function is intended to be
 *         used for charging boot capacitors of driving section. It has to be
 *         called each motor start-up when using high voltage drivers
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_3_g4xx_turn_on_low_sides(pwmc_t* ptr_hdl)
{
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    phandle->_Super.bturn_on_low_sides_action = true;

    /* Clear Update Flag */
    LL_TIM_ClearFlag_UPDATE(phandle->ptr_params_str->TIMx);

    /*Turn on the three low side switches */
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);
    LL_TIM_OC_SetCompareCH2(TIMx, 0u);
    LL_TIM_OC_SetCompareCH3(TIMx, 0u);

    /* Wait until next update */
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET) {
    }

    /* Main PWM Output Enable */
    LL_TIM_EnableAllOutputs(TIMx);

    if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
        LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
        LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
        LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
    }
    return;
}

/**
 * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
 *         bit
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_3_g4xx_switch_on_pwm(pwmc_t* ptr_hdl)
{
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    phandle->_Super.bturn_on_low_sides_action = false;

    /* Set all duty to 50% */
    /* Set ch4 for triggering */
    /* Clear Update Flag */
    LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(phandle->half_pwm_period) >> 1);
    LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(phandle->half_pwm_period) >> 1);
    LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(phandle->half_pwm_period) >> 1);
    LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(phandle->half_pwm_period) - 5u);

    /* wait for a new PWM period */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET) {
    }
    LL_TIM_ClearFlag_UPDATE(TIMx);

    /* Main PWM Output Enable */
    TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    LL_TIM_EnableAllOutputs(TIMx);

    if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
        if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u) {
            LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
            LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
            LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
        } else {
            /* It is executed during calibration phase the EN signal shall stay off */
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
        }
    }

    phandle->ptr_params_str->adcx_a->JSQR = phandle->adc_jsqr_ph_a + LL_ADC_INJ_TRIG_EXT_RISING;
    phandle->ptr_params_str->adcx_b->JSQR = phandle->adc_jsqr_ph_b + LL_ADC_INJ_TRIG_EXT_RISING;
    phandle->ptr_params_str->adcx_c->JSQR = phandle->adc_jsqr_ph_c + LL_ADC_INJ_TRIG_EXT_RISING;

    /* Clear Update Flag */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    /* Enable Update IRQ */
    LL_TIM_EnableIT_UPDATE(TIMx);
}

/**
 * @brief  It disables PWM generation on the proper Timer peripheral acting on
 *         MOE bit
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_3_g4xx_switch_off_pwm(pwmc_t* ptr_hdl)
{
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    /* Disable UPDATE ISR */
    LL_TIM_DisableIT_UPDATE(TIMx);

    phandle->_Super.bturn_on_low_sides_action = false;

    /* Main PWM Output Disable */
    LL_TIM_DisableAllOutputs(TIMx);
    if (phandle->brake_action_lock == true) {
    } else {
        if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
        }
    }

    /* Clear the JSAQR register queue */
    LL_ADC_INJ_StopConversion(phandle->ptr_params_str->adcx_a);
    LL_ADC_INJ_StopConversion(phandle->ptr_params_str->adcx_b);
    LL_ADC_INJ_StopConversion(phandle->ptr_params_str->adcx_c);

    LL_ADC_INJ_StartConversion(phandle->ptr_params_str->adcx_a);
    LL_ADC_INJ_StartConversion(phandle->ptr_params_str->adcx_b);
    LL_ADC_INJ_StartConversion(phandle->ptr_params_str->adcx_c);

    /* wait for a new PWM period to flush last HF task */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET) {
    }
    LL_TIM_ClearFlag_UPDATE(TIMx);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  writes into peripheral registers the new duty cycles and
 *        sampling point
 * @param  phandle handler of the current instance of the PWM component
 * @param hCCR4Reg: new capture/compare register value.
 * @retval none
 */
__weak uint16_t r3_3_g4xx_write_tim_registers(pwmc_t* ptr_hdl, uint16_t hCCR4Reg)
{
    uint16_t hAux;
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    LL_TIM_OC_SetCompareCH1(TIMx, phandle->_Super.hcnt_pha);
    LL_TIM_OC_SetCompareCH2(TIMx, phandle->_Super.hcnt_phb);
    LL_TIM_OC_SetCompareCH3(TIMx, phandle->_Super.hcnt_phc);

    __LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_SetCompareCH4(TIMx, 0xFFFFu);
    __LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_SetCompareCH4(TIMx, hCCR4Reg);

    phandle->ptr_params_str->adcx_a->JSQR = phandle->adc_jsqr_ph_a + phandle->adc_external_polarity_injected;
    phandle->ptr_params_str->adcx_b->JSQR = phandle->adc_jsqr_ph_b + phandle->adc_external_polarity_injected;
    phandle->ptr_params_str->adcx_c->JSQR = phandle->adc_jsqr_ph_c + phandle->adc_external_polarity_injected;

    /* Limit for update event */
    /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */
    if (phandle->so_foc != 0u) {
        hAux = MC_FOC_DURATION;
    } else {
        hAux = MC_NO_ERROR;
    }
    if (phandle->_Super.sw_error == 1u) {
        hAux = MC_FOC_DURATION;
        phandle->_Super.sw_error = 0u;
    }
    return hAux;
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Configure the ADC for the current sampling related to sector 1.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 * @param  phandle Pointer on the target component instance
 * @retval none
 */
__weak uint16_t r3_3_g4xx_set_adc_samp_point_sect1(pwmc_t* ptr_hdl)
{
    uint16_t cnt_smp;
    uint16_t delta_duty;
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_RISING;

    /** Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
     * in the sector 1 (i.e phase A duty cycle) */
    if ((uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_pha) > phandle->ptr_params_str->ht_after) {
        /** When it is possible to sample in the middle of the PWM period, always sample the same phases
         * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
         * between offsets */

        /** sector number needed by Getphase_current, phase A and B are sampled which corresponds
         * to sector 4  */
        phandle->_Super.hSector = SECTOR_4;

        /* set sampling  point trigger in the middle of PWM period */
        cnt_smp = (uint32_t)(phandle->half_pwm_period) - 1u;

    } else {
        /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

        /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
        delta_duty = (uint16_t)(phandle->_Super.hcnt_pha - phandle->_Super.hcnt_phb);

        /* Definition of crossing point */
        if (delta_duty > (uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_pha) * 2u) {
            cnt_smp = phandle->_Super.hcnt_pha - phandle->ptr_params_str->ht_before;
        } else {
            cnt_smp = phandle->_Super.hcnt_pha + phandle->ptr_params_str->ht_after;

            if (cnt_smp >= phandle->half_pwm_period) {
                /* Set CC4 as PWM mode 1 */
                phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_FALLING;
                cnt_smp = (2u * phandle->half_pwm_period) - cnt_smp - 1u;
            }
        }
    }

    return r3_3_g4xx_write_tim_registers(&phandle->_Super, cnt_smp);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Configure the ADC for the current sampling related to sector 2.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 * @param  phandle Pointer on the target component instance
 * @retval none
 */
__weak uint16_t r3_3_g4xx_set_adc_samp_point_sect2(pwmc_t* ptr_hdl)
{
    uint16_t cnt_smp;
    uint16_t delta_duty;
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_RISING;

    /** Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
     * in the sector 2 (i.e phase B duty cycle) */
    if ((uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_phb) > phandle->ptr_params_str->ht_after) {
        /** When it is possible to sample in the middle of the PWM period, always sample the same phases
         * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
         * between offsets */

        /** sector number needed by Getphase_current, phase A and B are sampled which corresponds
         * to sector 4  */
        phandle->_Super.hSector = SECTOR_4;

        /* set sampling  point trigger in the middle of PWM period */
        cnt_smp = (uint32_t)(phandle->half_pwm_period) - 1u;

    } else {
        /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

        /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
        delta_duty = (uint16_t)(phandle->_Super.hcnt_phb - phandle->_Super.hcnt_pha);

        /* Definition of crossing point */
        if (delta_duty > (uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_phb) * 2u) {
            cnt_smp = phandle->_Super.hcnt_phb - phandle->ptr_params_str->ht_before;
        } else {
            cnt_smp = phandle->_Super.hcnt_phb + phandle->ptr_params_str->ht_after;

            if (cnt_smp >= phandle->half_pwm_period) {
                /* Set CC4 as PWM mode 1 */
                phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_FALLING;
                cnt_smp = (2u * phandle->half_pwm_period) - cnt_smp - 1u;
            }
        }
    }

    return r3_3_g4xx_write_tim_registers(&phandle->_Super, cnt_smp);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Configure the ADC for the current sampling related to sector 3.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 * @param  phandle Pointer on the target component instance
 * @retval none
 */
__weak uint16_t r3_3_g4xx_set_adc_samp_point_sect3(pwmc_t* ptr_hdl)
{
    uint16_t cnt_smp;
    uint16_t delta_duty;
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_RISING;

    /** Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
     * in the sector 3 (i.e phase B duty cycle) */
    if ((uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_phb) > phandle->ptr_params_str->ht_after) {
        /** When it is possible to sample in the middle of the PWM period, always sample the same phases
         * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
         * between offsets */

        /** sector number needed by Getphase_current, phase A and B are sampled which corresponds
         * to sector 4  */
        phandle->_Super.hSector = SECTOR_4;

        /* set sampling  point trigger in the middle of PWM period */
        cnt_smp = (uint32_t)(phandle->half_pwm_period) - 1u;
    } else {
        /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

        /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
        delta_duty = (uint16_t)(phandle->_Super.hcnt_phb - phandle->_Super.hcnt_phc);

        /* Definition of crossing point */
        if (delta_duty > (uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_phb) * 2u) {
            cnt_smp = phandle->_Super.hcnt_phb - phandle->ptr_params_str->ht_before;
        } else {
            cnt_smp = phandle->_Super.hcnt_phb + phandle->ptr_params_str->ht_after;

            if (cnt_smp >= phandle->half_pwm_period) {
                /* Set CC4 as PWM mode 1 */
                phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_FALLING;
                cnt_smp = (2u * phandle->half_pwm_period) - cnt_smp - 1u;
            }
        }
    }

    return r3_3_g4xx_write_tim_registers(&phandle->_Super, cnt_smp);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Configure the ADC for the current sampling related to sector 4.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 * @param  phandle Pointer on the target component instance
 * @retval none
 */
__weak uint16_t r3_3_g4xx_set_adc_samp_point_sect4(pwmc_t* ptr_hdl)
{
    uint16_t cnt_smp;
    uint16_t delta_duty;
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_RISING;

    /** Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
     * in the sector 4 (i.e phase C duty cycle) */
    if ((uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_phc) > phandle->ptr_params_str->ht_after) {
        /** When it is possible to sample in the middle of the PWM period, always sample the same phases
         * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
         * between offsets */

        /** sector number needed by Getphase_current, phase A and B are sampled which corresponds
         * to sector 4  */
        phandle->_Super.hSector = SECTOR_4;

        /* set sampling  point trigger in the middle of PWM period */
        cnt_smp = (uint32_t)(phandle->half_pwm_period) - 1u;

    } else {
        /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

        /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
        delta_duty = (uint16_t)(phandle->_Super.hcnt_phc - phandle->_Super.hcnt_phb);

        /* Definition of crossing point */
        if (delta_duty > (uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_phc) * 2u) {
            cnt_smp = phandle->_Super.hcnt_phc - phandle->ptr_params_str->ht_before;
        } else {
            cnt_smp = phandle->_Super.hcnt_phc + phandle->ptr_params_str->ht_after;

            if (cnt_smp >= phandle->half_pwm_period) {
                /* Set CC4 as PWM mode 1 */
                phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_FALLING;
                cnt_smp = (2u * phandle->half_pwm_period) - cnt_smp - 1u;
            }
        }
    }

    return r3_3_g4xx_write_tim_registers(&phandle->_Super, cnt_smp);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Configure the ADC for the current sampling related to sector 5.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 * @param  phandle Pointer on the target component instance
 * @retval none
 */
__weak uint16_t r3_3_g4xx_set_adc_samp_point_sect5(pwmc_t* ptr_hdl)
{
    uint16_t cnt_smp;
    uint16_t delta_duty;
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_RISING;

    /** Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
     * in the sector 5 (i.e phase C duty cycle) */
    if ((uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_phc) > phandle->ptr_params_str->ht_after) {
        /** When it is possible to sample in the middle of the PWM period, always sample the same phases
         * (AB, AC  or BC) for all sectors in order to not induce current discontinuities when there are differences
         * between offsets */

        /** sector number needed by Getphase_current, phase A and B are sampled which corresponds
         * to sector 4  */
        phandle->_Super.hSector = SECTOR_4;

        /* set sampling  point trigger in the middle of PWM period */
        cnt_smp = (uint32_t)(phandle->half_pwm_period) - 1u;
    } else {
        /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

        /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
        delta_duty = (uint16_t)(phandle->_Super.hcnt_phc - phandle->_Super.hcnt_pha);

        /* Definition of crossing point */
        if (delta_duty > (uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_phc) * 2u) {
            cnt_smp = phandle->_Super.hcnt_phc - phandle->ptr_params_str->ht_before;
        } else {
            cnt_smp = phandle->_Super.hcnt_phc + phandle->ptr_params_str->ht_after;

            if (cnt_smp >= phandle->half_pwm_period) {
                /* Set CC4 as PWM mode 1 */
                phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_FALLING;
                cnt_smp = (2u * phandle->half_pwm_period) - cnt_smp - 1u;
            }
        }
    }

    return r3_3_g4xx_write_tim_registers(&phandle->_Super, cnt_smp);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Configure the ADC for the current sampling related to sector 6.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 * @param  phandle Pointer on the target component instance
 * @retval none
 */
__weak uint16_t r3_3_g4xx_set_adc_samp_point_sect6(pwmc_t* ptr_hdl)
{
    uint16_t cnt_smp;
    uint16_t delta_duty;
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_RISING;

    /** Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
     * in the sector 6 (i.e phase A duty cycle) */
    if ((uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_pha) > phandle->ptr_params_str->ht_after) {
        /** When it is possible to sample in the middle of the PWM period, always sample the same phases
         * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
         * between offsets */

        /** sector number needed by Getphase_current, phase A and B are sampled which corresponds
         * to sector 4  */
        phandle->_Super.hSector = SECTOR_4;

        /* set sampling  point trigger in the middle of PWM period */
        cnt_smp = (uint32_t)(phandle->half_pwm_period) - 1u;

    } else {
        /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

        /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
        delta_duty = (uint16_t)(phandle->_Super.hcnt_pha - phandle->_Super.hcnt_phc);

        /* Definition of crossing point */
        if (delta_duty > (uint16_t)(phandle->half_pwm_period - phandle->_Super.hcnt_pha) * 2u) {
            cnt_smp = phandle->_Super.hcnt_pha - phandle->ptr_params_str->ht_before;
        } else {
            cnt_smp = phandle->_Super.hcnt_pha + phandle->ptr_params_str->ht_after;

            if (cnt_smp >= phandle->half_pwm_period) {
                /* Set CC4 as PWM mode 1 */
                phandle->adc_external_polarity_injected = LL_ADC_INJ_TRIG_EXT_FALLING;
                cnt_smp = (2u * phandle->half_pwm_period) - cnt_smp - 1u;
            }
        }
    }

    return r3_3_g4xx_write_tim_registers(&phandle->_Super, cnt_smp);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It contains the TIMx Update event interrupt
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void* r3_3_g4xx_timx_up_irq_handler(pwmc_r3_3_g4_t* phandle)
{
    /* Set the SOFOC flag to indicate the execution of Update IRQ*/
    phandle->so_foc = 1u;
    return &(phandle->_Super.bmotor);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It contains the TIMx Break2 event interrupt
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void* r3_3_g4xx_brk2_irq_handler(pwmc_r3_3_g4_t* phandle)
{
    if (phandle->brake_action_lock == false) {
        if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
        }
    }
    phandle->over_current_flag = true;

    return &(phandle->_Super.bmotor);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It contains the TIMx Break1 event interrupt
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void* r3_3_g4xx_brk_irq_handler(pwmc_r3_3_g4_t* phandle)
{
    phandle->ptr_params_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    phandle->over_voltage_flag = true;
    phandle->brake_action_lock = true;

    return &(phandle->_Super.bmotor);
}

/**
 * @brief  It is used to check if an overcurrent occurred since last call.
 * @param  phandle Pointer on the target component instance
 * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
 *                  detected since last method call, MC_NO_FAULTS otherwise.
 */
__weak uint16_t r3_3_g4xx_is_over_current_occurred(pwmc_t* ptr_hdl)
{
    pwmc_r3_3_g4_t* phandle = (pwmc_r3_3_g4_t*)ptr_hdl;

    uint16_t ret_val = MC_NO_FAULTS;

    if (phandle->over_voltage_flag == true) {
        ret_val = MC_OVER_VOLT;
        phandle->over_voltage_flag = false;
    }

    if (phandle->over_current_flag == true) {
        ret_val |= MC_BREAK_IN;
        phandle->over_current_flag = false;
    }

    return ret_val;
}

/**
 * @brief  It is used to configure the analog output used for protection
 *         thresholds.
 * @param  DAC_Channel: the selected DAC channel.
 *          This parameter can be:
 *            @arg DAC_Channel_1: DAC Channel1 selected
 *            @arg DAC_Channel_2: DAC Channel2 selected
 * @param  dac_vref Value of DAC reference expressed as 16bit unsigned integer.
 *         Ex. 0 = 0V 65536 = VDD_DAC.
 * @retval none
 */
__weak void r3_3_g4xx_set_aoreference_voltage(uint32_t DAC_Channel, uint16_t dac_vref)
{
    if (DAC_Channel == LL_DAC_CHANNEL_2) {
        LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_2, dac_vref);
    } else {
        LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_1, dac_vref);
    }

    /* Enable DAC Channel */
    LL_DAC_TrigSWConversion(DAC1, DAC_Channel);
    LL_DAC_Enable(DAC1, DAC_Channel);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
