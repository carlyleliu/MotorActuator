/**
 ******************************************************************************
 * @file    r3_2_g4xx_pwm_curr_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement current sensor
 *          class to be stantiated when the three shunts current sensing
 *          topology is used. It is specifically designed for STM32F30X
 *          microcontrollers and implements the successive sampling of two motor
 *          current using shared ADC.
 *           + MCU peripheral and handle initialization function
 *           + three shunts current sensing
 *           + space vector modulation function
 *           + ADC sampling function
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
#include "r3_2_g4xx_pwm_curr_fdbk.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup pwm_curr_fdbk
 * @{
 */

/**
 * @defgroup R3_2_G4XX_pwm_curr_fdbk R3 2 ADCs G4 PWM & Current Feedback
 *
 * @brief STM32G4, Shared Resources, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32G4 MCU, using a three
 * shunt resistors current sensing topology and 2 ADC peripherals to acquire the current
 * values.
 *
 *
 * @todo: TODO: complete documentation.
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123 \
    ((uint16_t)(LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N))

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void r3_2_timx_init(TIM_TypeDef* TIMx, pwmc_t* ptr_hdl);
static void r3_2_adcx_init(ADC_TypeDef* ADCx);
__STATIC_INLINE uint16_t r3_2_write_tim_registers(pwmc_t* ptr_hdl, uint16_t hCCR4Reg);
static void r3_2_hfcurrents_polarization_ab(pwmc_t* ptr_hdl, ab_t* iab);
static void r3_2_hfcurrents_polarization_c(pwmc_t* ptr_hdl, ab_t* iab);
static void r3_2_set_aoreference_voltage(uint32_t DAC_Channel, DAC_TypeDef* DACx, uint16_t dac_vref);
uint16_t r3_2_set_adc_samp_point_polarization(pwmc_t* ptr_hdl);
static void r3_2_rl_get_phase_currents(pwmc_t* ptr_hdl, ab_t* pStator_Currents);
static void r3_2_rl_turnon_lowsides(pwmc_t* ptr_hdl);
static void r3_2_rl_switchon_pwm(pwmc_t* ptr_hdl);
/**
 * @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVic for current reading
 *         in three shunt topology using STM32F30X and shared ADC
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_2_init(pwmc_r3_2_t* phandle)
{
    r3_3_opamp_params_t* opamp_params = phandle->ptr_params_str->opamp_params;
    COMP_TypeDef* COMP_OCPAx = phandle->ptr_params_str->comp_ocp_a_selection;
    COMP_TypeDef* COMP_OCPBx = phandle->ptr_params_str->comp_ocp_b_selection;
    COMP_TypeDef* COMP_OCPCx = phandle->ptr_params_str->comp_ocp_c_selection;
    COMP_TypeDef* COMP_OVPx = phandle->ptr_params_str->comp_ovp_selection;
    DAC_TypeDef* DAC_OCPAx = phandle->ptr_params_str->dac_ocp_a_selection;
    DAC_TypeDef* DAC_OCPBx = phandle->ptr_params_str->dac_ocp_b_selection;
    DAC_TypeDef* DAC_OCPCx = phandle->ptr_params_str->dac_ocp_c_selection;
    DAC_TypeDef* DAC_OVPx = phandle->ptr_params_str->dac_ovp_selection;
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;

    /*Check that _Super is the first member of the structure pwmc_r3_2_t */
    if ((uint32_t)phandle == (uint32_t)&phandle->_Super) {
        /** disable IT and flags in case of LL driver usage
         * workaround for unwanted interrupt enabling done by LL driver */
        LL_ADC_DisableIT_EOC(ADCx_1);
        LL_ADC_ClearFlag_EOC(ADCx_1);
        LL_ADC_DisableIT_JEOC(ADCx_1);
        LL_ADC_ClearFlag_JEOC(ADCx_1);
        LL_ADC_DisableIT_EOC(ADCx_2);
        LL_ADC_ClearFlag_EOC(ADCx_2);
        LL_ADC_DisableIT_JEOC(ADCx_2);
        LL_ADC_ClearFlag_JEOC(ADCx_2);

        if (opamp_params != NULL) {
            /* Testing of all OPAMP one by one is required as 2 or 3 OPAMPS cfg may exist*/
            if (opamp_params->OPAMPx_1 != NULL) {
                LL_OPAMP_Enable(opamp_params->OPAMPx_1);
            }
            if (opamp_params->OPAMPx_2 != NULL) {
                LL_OPAMP_Enable(opamp_params->OPAMPx_2);
            }
            if (opamp_params->OPAMPx_3 != NULL) {
                LL_OPAMP_Enable(opamp_params->OPAMPx_3);
            }
        }

        /* Over current protection phase A */
        if (COMP_OCPAx != NULL) {
            /* Inverting input*/
            if (phandle->ptr_params_str->comp_ocp_a_invInput_mode != EXT_MODE) {
                r3_2_set_aoreference_voltage(phandle->ptr_params_str->dac_channel_ocp_a, DAC_OCPAx,
                                             (uint16_t)(phandle->ptr_params_str->dac_ocp_threshold));
            }
            /* Output */
            LL_COMP_Enable(COMP_OCPAx);
            LL_COMP_Lock(COMP_OCPAx);
        }

        /* Over current protection phase B */
        if (COMP_OCPBx != NULL) {
            if (phandle->ptr_params_str->comp_ocp_b_invInput_mode != EXT_MODE) {
                r3_2_set_aoreference_voltage(phandle->ptr_params_str->dac_channel_ocp_b, DAC_OCPBx,
                                             (uint16_t)(phandle->ptr_params_str->dac_ocp_threshold));
            }
            LL_COMP_Enable(COMP_OCPBx);
            LL_COMP_Lock(COMP_OCPBx);
        }

        /* Over current protection phase C */
        if (COMP_OCPCx != NULL) {
            if (phandle->ptr_params_str->comp_ocp_c_invInput_mode != EXT_MODE) {
                r3_2_set_aoreference_voltage(phandle->ptr_params_str->dac_channel_ocp_c, DAC_OCPCx,
                                             (uint16_t)(phandle->ptr_params_str->dac_ocp_threshold));
            }
            LL_COMP_Enable(COMP_OCPCx);
            LL_COMP_Lock(COMP_OCPCx);
        }

        /* Over voltage protection */
        if (COMP_OVPx != NULL) {
            /* Inverting input*/
            if (phandle->ptr_params_str->comp_ovp_invInput_mode != EXT_MODE) {
                r3_2_set_aoreference_voltage(phandle->ptr_params_str->dac_channel_ovp, DAC_OVPx,
                                             (uint16_t)(phandle->ptr_params_str->dac_ovp_threshold));
            }
            /* Output */
            LL_COMP_Enable(COMP_OVPx);
            LL_COMP_Lock(COMP_OVPx);
        }

        if (LL_ADC_IsEnabled(ADCx_1) == 0) {
            r3_2_adcx_init(ADCx_1);
            /** Only the Interrupt of the first ADC is enabled.
             * As Both ADCs are fired by hw at the same moment
             * It is safe to consider that both conversion are ready at the same time*/
            LL_ADC_ClearFlag_JEOS(ADCx_1);
            LL_ADC_EnableIT_JEOS(ADCx_1);
        } else {
            /* Nothing to do ADCx_1 already configured */
        }
        if (LL_ADC_IsEnabled(ADCx_2) == 0) {
            r3_2_adcx_init(ADCx_2);
        } else {
            /* Nothing to do ADCx_2 already configured */
        }
        r3_2_timx_init(TIMx, &phandle->_Super);
    }
    phandle->_Super.fct_set_adc_samp_point_sectx = &r3_2_set_adc_samp_point_polarization;
}

static void r3_2_adcx_init(ADC_TypeDef* ADCx)
{
    /* - Exit from deep-power-down mode */
    LL_ADC_DisableDeepPowerDown(ADCx);

    if (LL_ADC_IsInternalRegulatorEnabled(ADCx) == 0u) {
        /* Enable ADC internal voltage regulator */
        LL_ADC_EnableInternalRegulator(ADCx);

        /* Wait for Regulator Startup time, once for both */
        /* Note: Variable divided by 2 to compensate partially              */
        /*       CPU processing cycles, scaling in us split to not          */
        /*       exceed 32 bits register capacity and handle low frequency. */
        volatile uint32_t wait_loop_index =
        ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
        while (wait_loop_index != 0UL) {
            wait_loop_index--;
        }
    }

    LL_ADC_StartCalibration(ADCx, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADCx) == 1u) {
    }
    /* ADC Enable (must be done after calibration) */
    /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0
     * following a calibration phase, could have no effect on ADC
     * within certain AHB/ADC clock ratio.
     */
    while (LL_ADC_IsActiveFlag_ADRDY(ADCx) == 0u) {
        LL_ADC_Enable(ADCx);
    }
    /* Clear JSQR from CubeMX setting to avoid not wanting conversion*/
    LL_ADC_INJ_StartConversion(ADCx);
    LL_ADC_INJ_StopConversion(ADCx);
    /* TODO: check if not already done by MX */
    LL_ADC_INJ_SetQueueMode(ADCx, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
}

/**
 * @brief  It initializes TIMx peripheral for PWM generation
 * @param TIMx: Timer to be initialized
 * @param phandle: handler of the current instance of the PWM component
 * @retval none
 */
static void r3_2_timx_init(TIM_TypeDef* TIMx, pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
#if 0
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#endif
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

#if 0
    uint32_t brk2_timeout = 1000;
    /** disable main TIM counter to ensure
     * a synchronous start by TIM2 trigger */
    LL_TIM_DisableCounter(TIMx);

    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

    /* Enables the TIMx Preload on CC1 Register */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
    /* Enables the TIMx Preload on CC2 Register */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
    /* Enables the TIMx Preload on CC3 Register */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
    /* Enables the TIMx Preload on CC4 Register */
    LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
#endif
    /* Prepare timer for synchronization */
    LL_TIM_GenerateEvent_UPDATE(TIMx);
#if 0
    if (phandle->ptr_params_str->freq_ratio == 2u) {
        if (phandle->ptr_params_str->is_higher_freq_tim == HIGHER_FREQ) {
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
        if (phandle->_Super.motor == M1) {
            if (phandle->ptr_params_str->repetition_counter == 1u) {
                LL_TIM_SetCounter(TIMx, (uint32_t)(phandle->half_pwm_period) - 1u);
            } else if (phandle->ptr_params_str->repetition_counter == 3u) {
                /* Set TIMx repetition counter to 1 */
                LL_TIM_SetRepetitionCounter(TIMx, 1);
                LL_TIM_GenerateEvent_UPDATE(TIMx);
                /* Repetition counter will be set to 3 at next Update */
                LL_TIM_SetRepetitionCounter(TIMx, 3);
            } else {
            }
        } else {
        }
    }

    LL_TIM_ClearFlag_BRK(TIMx);

    if ((phandle->ptr_params_str->bkin2_mode) != NONE) {
        while ((LL_TIM_IsActiveFlag_BRK2(TIMx) == 1u) && (brk2_timeout != 0u)) {
            LL_TIM_ClearFlag_BRK2(TIMx);
            brk2_timeout--;
        }
    }
#endif
    LL_TIM_EnableIT_BRK(TIMx);

    /* Enable PWM channel */
    LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);
}

/**
 * @brief  It stores into the component the voltage present on ia and
 *         ib current feedback analog channels when no current is flowin into the
 *         motor
 * @param  ptr_hdl: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_2_current_reading_polarization(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;

    phandle->phase_a_offset = 0u;
    phandle->phase_b_offset = 0u;
    phandle->phase_c_offset = 0u;

    phandle->polarization_counter = 0u;

    /* It forces inactive level on TIMx CHy and CHyN */
    LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);

    /* Offset calibration for all phases */
    /* Change function to be executed in ADCx_ISR */
    phandle->_Super.fct_get_phase_currents = &r3_2_hfcurrents_polarization_ab;
    phandle->_Super.fct_set_adc_samp_point_sectx = &r3_2_set_adc_samp_point_polarization;
    phandle->adc_external_polarity_injected = (uint16_t)LL_ADC_INJ_TRIG_EXT_RISING;

    /* We want to polarize calibration Phase A and Phase B, so we select SECTOR_5 */
    phandle->polarization_sector = SECTOR_5;
    /* Required to force first polarization conversion on SECTOR_5*/
    phandle->_Super.Sector = SECTOR_5;
    r3_2_switch_on_pwm(&phandle->_Super);

    /* IF CH4 is enabled, it means that JSQR is now configured to sample polarization current*/
    // while ( LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4) == 0u )
    //{
    //}
    while (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_OC4REF) {
    }
    /* It is the right time to start the ADC without unwanted conversion */
    /* Start ADC to wait for external trigger. This is series dependant*/
    LL_ADC_INJ_StartConversion(ADCx_1);
    LL_ADC_INJ_StartConversion(ADCx_2);

    /* Wait for NB_CONVERSIONS to be executed */
    // wait_for_polarization_end(TIMx, &phandle->_Super.sw_error, phandle->ptr_params_str->repetition_counter,
    //                           &phandle->polarization_counter);

    r3_2_switch_off_pwm(&phandle->_Super);

    /* Offset calibration for C phase */
    phandle->polarization_counter = 0u;

    /* Change function to be executed in ADCx_ISR */
    phandle->_Super.fct_get_phase_currents = &r3_2_hfcurrents_polarization_c;
    /* We want to polarize Phase C, so we select SECTOR_1 */
    phandle->polarization_sector = SECTOR_1;
    /* Required to force first polarization conversion on SECTOR_1*/
    phandle->_Super.Sector = SECTOR_1;
    r3_2_switch_on_pwm(&phandle->_Super);

    /* Wait for NB_CONVERSIONS to be executed */
    // wait_for_polarization_end(TIMx, &phandle->_Super.sw_error, phandle->ptr_params_str->repetition_counter,
    //                           &phandle->polarization_counter);

    r3_2_switch_off_pwm(&phandle->_Super);
    phandle->phase_a_offset /= NB_CONVERSIONS;
    phandle->phase_b_offset /= NB_CONVERSIONS;
    phandle->phase_c_offset /= NB_CONVERSIONS;

    /* Change back function to be executed in ADCx_ISR */
    phandle->_Super.fct_get_phase_currents = &r3_2_get_phase_currents;
    phandle->_Super.fct_set_adc_samp_point_sectx = &r3_2_set_adc_samp_point_sectx;

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
    LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

    /* At the end of calibration, all phases are at 50% we will sample A&B */
    phandle->_Super.Sector = SECTOR_5;

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
 * @param  ptr_hdl: handler of the current instance of the PWM component
 * @retval ia and ib current in Curr_Components format
 */
__weak void r3_2_get_phase_currents(pwmc_t* ptr_hdl, ab_t* iab)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    uint8_t Sector;
    int32_t aux;
    uint32_t adc_data_reg1;
    uint32_t adc_data_reg2;

    Sector = (uint8_t)phandle->_Super.Sector;
    adc_data_reg1 = *phandle->ptr_params_str->adc_data_reg1[Sector];
    adc_data_reg2 = *phandle->ptr_params_str->adc_data_reg2[Sector];

    /* disable ADC trigger source */
    // LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

    switch (Sector) {
        case SECTOR_4:
        case SECTOR_5:
            /* Current on Phase C is not accessible     */
            /* ia = phase_a_offset - ADC converted value) */
            aux = (int32_t)(phandle->phase_a_offset) - (int32_t)(adc_data_reg1);

            /* Saturation of ia */
            if (aux < -INT16_MAX) {
                iab->a = -INT16_MAX;
            } else if (aux > INT16_MAX) {
                iab->a = INT16_MAX;
            } else {
                iab->a = (int16_t)aux;
            }

            /* ib = phase_b_offset - ADC converted value) */
            aux = (int32_t)(phandle->phase_b_offset) - (int32_t)(adc_data_reg2);

            /* Saturation of ib */
            if (aux < -INT16_MAX) {
                iab->b = -INT16_MAX;
            } else if (aux > INT16_MAX) {
                iab->b = INT16_MAX;
            } else {
                iab->b = (int16_t)aux;
            }
            break;

        case SECTOR_6:
        case SECTOR_1:
            /* Current on Phase A is not accessible     */
            /* ib = phase_b_offset - ADC converted value) */
            aux = (int32_t)(phandle->phase_b_offset) - (int32_t)(adc_data_reg1);

            /* Saturation of ib */
            if (aux < -INT16_MAX) {
                iab->b = -INT16_MAX;
            } else if (aux > INT16_MAX) {
                iab->b = INT16_MAX;
            } else {
                iab->b = (int16_t)aux;
            }

            /* ia = -ic -ib */
            aux = (int32_t)(adc_data_reg2) - (int32_t)(phandle->phase_c_offset); /* -ic */
            aux -= (int32_t)iab->b;                                              /* ia  */

            /* Saturation of ia */
            if (aux > INT16_MAX) {
                iab->a = INT16_MAX;
            } else if (aux < -INT16_MAX) {
                iab->a = -INT16_MAX;
            } else {
                iab->a = (int16_t)aux;
            }
            break;

        case SECTOR_2:
        case SECTOR_3:
            /* Current on Phase B is not accessible     */
            /* ia = phase_a_offset - ADC converted value) */
            aux = (int32_t)(phandle->phase_a_offset) - (int32_t)(adc_data_reg1);

            /* Saturation of ia */
            if (aux < -INT16_MAX) {
                iab->a = -INT16_MAX;
            } else if (aux > INT16_MAX) {
                iab->a = INT16_MAX;
            } else {
                iab->a = (int16_t)aux;
            }

            /* ib = -ic -ia */
            aux = (int32_t)(adc_data_reg2) - (int32_t)(phandle->phase_c_offset); /* -ic */
            aux -= (int32_t)iab->a;                                              /* ib */

            /* Saturation of ib */
            if (aux > INT16_MAX) {
                iab->b = INT16_MAX;
            } else if (aux < -INT16_MAX) {
                iab->b = -INT16_MAX;
            } else {
                iab->b = (int16_t)aux;
            }
            break;

        default:
            break;
    }

    phandle->_Super.ia = iab->a;
    phandle->_Super.ib = iab->b;
    phandle->_Super.ic = -iab->a - iab->b;
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Configure the ADC for the current sampling during calibration.
 *         It means set the sampling point via TIMx_Ch4 value and polarity
 *         ADC sequence length and channels.
 *         And call the WriteTIMRegisters method.
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
uint16_t r3_2_set_adc_samp_point_polarization(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    phandle->_Super.Sector = phandle->polarization_sector;

    return r3_2_write_tim_registers(&phandle->_Super, (phandle->half_pwm_period - (uint16_t)1));
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
uint16_t r3_2_set_adc_samp_point_sectx(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    uint16_t sampling_point;
    uint16_t delta_duty;

    /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle */
    if ((uint16_t)(phandle->half_pwm_period - ptr_hdl->low_duty) > phandle->ptr_params_str->t_after) {
        /* When it is possible to sample in the middle of the PWM period, always sample the same phases
         * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
         * between offsets */

        /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
         * to sector 4 or 5  */
        phandle->_Super.Sector = SECTOR_5;

        /* set sampling  point trigger in the middle of PWM period */
        sampling_point = phandle->half_pwm_period - (uint16_t)1;
    } else {
        /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

        /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
        delta_duty = (uint16_t)(ptr_hdl->low_duty - ptr_hdl->mid_duty);

        /* Definition of crossing point */
        if (delta_duty > (uint16_t)(phandle->half_pwm_period - ptr_hdl->low_duty) * 2u) {
            sampling_point = ptr_hdl->low_duty - phandle->ptr_params_str->t_before;
        } else {
            sampling_point = ptr_hdl->low_duty + phandle->ptr_params_str->t_after;

            if (sampling_point >= phandle->half_pwm_period) {
                /* ADC trigger edge must be changed from positive to negative */
                phandle->adc_external_polarity_injected = (uint16_t)LL_ADC_INJ_TRIG_EXT_FALLING;
                sampling_point = (2u * phandle->half_pwm_period) - sampling_point - (uint16_t)1;
            }
        }
    }
    return r3_2_write_tim_registers(&phandle->_Super, sampling_point);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  Stores into the component's handle the voltage present on ia and
 *         ib current feedback analog channels when no current is flowin into the
 *         motor
 * @param  phandle handler of the current instance of the PWM component
 * @retval none
 */
__STATIC_INLINE uint16_t r3_2_write_tim_registers(pwmc_t* ptr_hdl, uint16_t sampling_point)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    uint16_t aux;

    LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)phandle->_Super.cnt_pha);
    LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)phandle->_Super.cnt_phb);
    LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)phandle->_Super.cnt_phc);
    LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)sampling_point);

    /* Limit for update event */

    //  if ( LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4) == 1u )
    if (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET) {
        aux = MC_FOC_DURATION;
    } else {
        aux = MC_NO_ERROR;
    }
    return aux;
}
/**
 * @brief  Implementaion of pwmc_get_phase_currents to be performed during
 *         calibration. It sum up injected conversion data into phase_a_offset and
 *         phase_b_offset to compute the offset introduced in the current feedback
 *         network. It is requied to proper configure ADC inputs before to enable
 *         the offset computation.
 * @param  ptr_hdl Pointer on the target component instance
 * @retval It always returns {0,0} in Curr_Components format
 */
static void r3_2_hfcurrents_polarization_ab(pwmc_t* ptr_hdl, ab_t* iab)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    uint32_t adc_data_reg1 = *phandle->ptr_params_str->adc_data_reg1[phandle->polarization_sector];
    uint32_t adc_data_reg2 = *phandle->ptr_params_str->adc_data_reg2[phandle->polarization_sector];

    /* disable ADC trigger source */
    // LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

    if (phandle->polarization_counter < NB_CONVERSIONS) {
        phandle->phase_a_offset += adc_data_reg1;
        phandle->phase_b_offset += adc_data_reg2;
        phandle->polarization_counter++;
    }

    /* during offset calibration no current is flowing in the phases */
    iab->a = 0;
    iab->b = 0;
}

/**
 * @brief  Implementaion of pwmc_get_phase_currents to be performed during
 *         calibration. It sum up injected conversion data into phase_a_offset and
 *         phase_b_offset to compute the offset introduced in the current feedback
 *         network. It is requied to proper configure ADC inputs before to enable
 *         the offset computation.
 * @param  ptr_hdl Pointer on the target component instance
 * @retval It always returns {0,0} in Curr_Components format
 */
static void r3_2_hfcurrents_polarization_c(pwmc_t* ptr_hdl, ab_t* iab)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    uint32_t adc_data_reg2 = *phandle->ptr_params_str->adc_data_reg2[phandle->polarization_sector];

    /* disable ADC trigger source */
    // LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

    if (phandle->polarization_counter < NB_CONVERSIONS) {
        /* Phase C is read from SECTOR_1, second value */
        phandle->phase_c_offset += adc_data_reg2;
        phandle->polarization_counter++;
    }

    /* during offset calibration no current is flowing in the phases */
    iab->a = 0;
    iab->b = 0;
}
/**
 * @brief  It turns on low sides switches. This function is intended to be
 *         used for charging boot capacitors of driving section. It has to be
 *         called each motor start-up when using high voltage drivers
 * @param  ptr_hdl: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_2_turn_on_low_sides(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    phandle->_Super.turn_on_low_sides_action = true;

    /* Clear Update Flag */
    LL_TIM_ClearFlag_UPDATE(phandle->ptr_params_str->TIMx);

    /*Turn on the three low side switches */
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);
    LL_TIM_OC_SetCompareCH2(TIMx, 0u);
    LL_TIM_OC_SetCompareCH3(TIMx, 0u);

    /* Wait until next update */
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0u) {
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
 * @param  ptr_hdl: handler of the current instance of the PWM component
 * @retval none
 */
__weak void r3_2_switch_on_pwm(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    /* We forbid ADC usage for regular conversion on Systick*/
    phandle->adc_regular_locked = true;

    phandle->_Super.turn_on_low_sides_action = false;

    /* Set all duty to 50% */
    LL_TIM_OC_SetCompareCH1(TIMx, ((uint32_t)phandle->half_pwm_period / (uint32_t)2));
    LL_TIM_OC_SetCompareCH2(TIMx, ((uint32_t)phandle->half_pwm_period / (uint32_t)2));
    LL_TIM_OC_SetCompareCH3(TIMx, ((uint32_t)phandle->half_pwm_period / (uint32_t)2));
    LL_TIM_OC_SetCompareCH4(TIMx, ((uint32_t)phandle->half_pwm_period - (uint32_t)5));

    /* wait for a new PWM period */
    // LL_TIM_ClearFlag_UPDATE(TIMx);
    // while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0u) {
    // }
    // LL_TIM_ClearFlag_UPDATE(TIMx);

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
    /* Clear Update Flag */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    /* Enable Update IRQ */
    LL_TIM_EnableIT_UPDATE(TIMx);
}

/**
 * @brief  It disables PWM generation on the proper Timer peripheral acting on
 *         MOE bit
 * @param  ptr_hdl: handler of the current instance of the PWM component
 * @retval none
 */
void r3_2_switch_off_pwm(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    /* Disable UPDATE ISR */
    LL_TIM_DisableIT_UPDATE(TIMx);

    phandle->_Super.turn_on_low_sides_action = false;

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

    /* wait for a new PWM period to flush last HF task */
    // LL_TIM_ClearFlag_UPDATE(TIMx);
    // while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0u) {
    // }
    // LL_TIM_ClearFlag_UPDATE(TIMx);

    /* We allow ADC usage for regular conversion on Systick*/
    phandle->adc_regular_locked = false;
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
__weak void* r3_2_timx_up_irq_handler(pwmc_r3_2_t* phandle)
{
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;
    r3_3_opamp_params_t* opamp_params = phandle->ptr_params_str->opamp_params;
    OPAMP_TypeDef* Opamp;
    uint32_t opamp_config;

    if (opamp_params != NULL) {
        /* We can not change OPAMP source if ADC acquisition is ongoing (Dual motor with internal opamp use case)*/
        while (ADCx_1->JSQR != 0x0u) {
        }
        /* We need to manage the Operational amplifier internal output enable - Dedicated to G4 and the VPSEL selection */
        Opamp = opamp_params->OPAMPSelect_1[phandle->_Super.Sector];
        if (Opamp != NULL) {
            opamp_config = opamp_params->opamp_config1[phandle->_Super.Sector];
            MODIFY_REG(Opamp->CSR, (OPAMP_CSR_OPAMPINTEN | OPAMP_CSR_VPSEL), opamp_config);
        }
        Opamp = opamp_params->OPAMPSelect_2[phandle->_Super.Sector];
        if (Opamp != NULL) {
            opamp_config = opamp_params->opamp_config2[phandle->_Super.Sector];
            MODIFY_REG(Opamp->CSR, (OPAMP_CSR_OPAMPINTEN | OPAMP_CSR_VPSEL), opamp_config);
        }
    }

    ADCx_1->JSQR = phandle->ptr_params_str->adc_config1[phandle->_Super.Sector] | (uint32_t)phandle->adc_external_polarity_injected;
    ADCx_2->JSQR = phandle->ptr_params_str->adc_config2[phandle->_Super.Sector] | (uint32_t)phandle->adc_external_polarity_injected;

    /* enable ADC trigger source */

    // LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);

    phandle->adc_external_polarity_injected = (uint16_t)LL_ADC_INJ_TRIG_EXT_RISING;

    return &(phandle->_Super.motor);
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
__weak void* r3_2_brk2_irq_handler(pwmc_r3_2_t* phandle)
{
    if (phandle->brake_action_lock == false) {
        if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
        }
    }
    phandle->over_current_flag = true;

    return &(phandle->_Super.motor);
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
__weak void* r3_2_brk_irq_handler(pwmc_r3_2_t* phandle)
{
    phandle->ptr_params_str->TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    phandle->over_voltage_flag = true;
    phandle->brake_action_lock = true;

    return &(phandle->_Super.motor);
}

/**
 * @brief  It is used to check if an overcurrent occurred since last call.
 * @param  ptr_hdl Pointer on the target component instance
 * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
 *                  detected since last method call, MC_NO_FAULTS otherwise.
 */
__weak uint16_t r3_2_is_over_current_occurred(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
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
static void r3_2_set_aoreference_voltage(uint32_t DAC_Channel, DAC_TypeDef* DACx, uint16_t dac_vref)
{
    LL_DAC_ConvertData12LeftAligned(DACx, DAC_Channel, dac_vref);

    /* Enable DAC Channel */
    LL_DAC_TrigSWConversion(DACx, DAC_Channel);

    if (LL_DAC_IsEnabled(DACx, DAC_Channel) == 1u) { /* If DAC is already enable, we wait LL_DAC_DELAY_VOLTAGE_SETTLING_US*/
        uint32_t wait_loop_index = ((LL_DAC_DELAY_VOLTAGE_SETTLING_US) * (SystemCoreClock / (1000000UL * 2UL)));
        while (wait_loop_index != 0UL) {
            wait_loop_index--;
        }
    } else {
        /* If DAC is not enabled, we must wait LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US*/
        LL_DAC_Enable(DACx, DAC_Channel);
        uint32_t wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US) * (SystemCoreClock / (1000000UL * 2UL)));
        while (wait_loop_index != 0UL) {
            wait_loop_index--;
        }
    }
}

/**
 * @brief  It is used to set the PWM mode for R/L detection.
 * @param  phandle: handler of the current instance of the PWM component
 * @param  duty to be applied in uint16_t
 * @retval none
 */
void r3_2_rl_detection_mode_enable(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    if (phandle->_Super.rl_detection_mode == false) {
        /*  Channel1 configuration */
        LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
        LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
        LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
        LL_TIM_OC_SetCompareCH1(TIMx, 0u);

        /*  Channel2 configuration */
        if ((phandle->ptr_params_str->low_side_outputs) == LS_PWM_TIMER) {
            LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);
            LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2);
            LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
        } else if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
            LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE);
            LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);
            LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
        } else {
        }

        /*  Channel3 configuration */
        LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2);
        LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3);
        LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);

        phandle->phase_a_offset = phandle->phase_b_offset; /* Use only the offset of phB */
    }

    phandle->_Super.fct_get_phase_currents = &r3_2_rl_get_phase_currents;
    phandle->_Super.fct_turnon_lowsides = &r3_2_rl_turnon_lowsides;
    phandle->_Super.fct_switchon_pwm = &r3_2_rl_switchon_pwm;
    phandle->_Super.fct_switchoff_pwm = &r3_2_switch_off_pwm;

    phandle->_Super.rl_detection_mode = true;
}

/**
 * @brief  It is used to disable the PWM mode for R/L detection.
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
void r3_2_rl_detection_mode_disable(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    if (phandle->_Super.rl_detection_mode == true) {
        /*  Channel1 configuration */
        LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
        LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);

        if ((phandle->ptr_params_str->low_side_outputs) == LS_PWM_TIMER) {
            LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
        } else if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
            LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1N);
        } else {
        }

        LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(phandle->half_pwm_period) >> 1);

        /*  Channel2 configuration */
        LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
        LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2);

        if ((phandle->ptr_params_str->low_side_outputs) == LS_PWM_TIMER) {
            LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
        } else if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
            LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2N);
        } else {
        }

        LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(phandle->half_pwm_period) >> 1);

        /*  Channel3 configuration */
        LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
        LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3);

        if ((phandle->ptr_params_str->low_side_outputs) == LS_PWM_TIMER) {
            LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
        } else if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
            LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3N);
        } else {
        }

        LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(phandle->half_pwm_period) >> 1);

        phandle->_Super.fct_get_phase_currents = &r3_2_get_phase_currents;
        phandle->_Super.fct_turnon_lowsides = &r3_2_turn_on_low_sides;
        phandle->_Super.fct_switchon_pwm = &r3_2_switch_on_pwm;
        phandle->_Super.fct_switchoff_pwm = &r3_2_switch_off_pwm;

        phandle->_Super.rl_detection_mode = false;
    }
}

/**
 * @brief  It is used to set the PWM dutycycle for R/L detection.
 * @param  phandle: handler of the current instance of the PWM component
 * @param  duty to be applied in uint16_t
 * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
 *         otherwise. These error codes are defined in mc_type.h
 */
uint16_t r3_2_rl_detection_mode_set_duty(pwmc_t* ptr_hdl, uint16_t duty)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    uint32_t val;
    uint16_t haux;

    val = ((uint32_t)(phandle->half_pwm_period) * (uint32_t)(duty)) >> 16;
    phandle->_Super.cnt_pha = (uint16_t)(val);

    /* Set CC4 as PWM mode 2 (default) */
    LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2);

    LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(phandle->half_pwm_period - phandle->_Super.ton));
    LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)phandle->_Super.toff);
    LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)phandle->_Super.cnt_pha);

    /* Enabling next Trigger */
    // LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
    /* set the sector that correspond to Phase A and B sampling */
    ptr_hdl->Sector = SECTOR_4;

    /* Limit for update event */
    /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
    if (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET) {
        haux = MC_FOC_DURATION;
    } else {
        haux = MC_NO_ERROR;
    }
    if (phandle->_Super.sw_error == 1u) {
        haux = MC_FOC_DURATION;
        phandle->_Super.sw_error = 0u;
    }
    return haux;
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It computes and return latest converted motor phase currents motor
 *         during RL detection phase
 * @param  phandle: handler of the current instance of the PWM component
 * @retval ia and ib current in ab_t format
 */
static void r3_2_rl_get_phase_currents(pwmc_t* ptr_hdl, ab_t* pStator_Currents)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    int32_t waux;

    /* disable ADC trigger source */
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

    waux = (int32_t)(phandle->phase_b_offset) - (int32_t)*phandle->ptr_params_str->adc_data_reg2[phandle->_Super.Sector];

    /* Check saturation */
    if (waux > -INT16_MAX) {
        if (waux < INT16_MAX) {
        } else {
            waux = INT16_MAX;
        }
    } else {
        waux = -INT16_MAX;
    }

    pStator_Currents->a = (int16_t)waux;
    pStator_Currents->b = (int16_t)waux;
}

/**
 * @brief  It turns on low sides switches. This function is intended to be
 *         used for charging boot capacitors of driving section. It has to be
 *         called each motor start-up when using high voltage drivers.
 *         This function is specific for RL detection phase.
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
static void r3_2_rl_turnon_lowsides(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    /*Turn on the phase A low side switch */
    LL_TIM_OC_SetCompareCH1(TIMx, 0u);

    /* Clear Update Flag */
    LL_TIM_ClearFlag_UPDATE(TIMx);

    /* Wait until next update */
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {
    }

    /* Main PWM Output Enable */
    LL_TIM_EnableAllOutputs(TIMx);

    if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
        LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
        LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
        LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
    }
    return;
}

/**
 * @brief  It enables PWM generation on the proper Timer peripheral
 *         This function is specific for RL detection phase.
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
static void r3_2_rl_switchon_pwm(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;

    /* wait for a new PWM period */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {
    }
    /* Clear Update Flag */
    LL_TIM_ClearFlag_UPDATE(TIMx);

    LL_TIM_OC_SetCompareCH1(TIMx, 1u);
    LL_TIM_OC_SetCompareCH4(TIMx, (phandle->half_pwm_period) - 5u);

    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {
    }

    /* enable TIMx update interrupt*/
    LL_TIM_EnableIT_UPDATE(TIMx);

    /* Main PWM Output Enable */
    TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    LL_TIM_EnableAllOutputs(TIMx);

    if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
        if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u) {
            LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
            LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
        } else {
            /* It is executed during calibration phase the EN signal shall stay off */
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
        }
    }

    /* set the sector that correspond to Phase B and C sampling
     * B will be sampled by ADCx_1 */
    ptr_hdl->Sector = SECTOR_4;

    LL_ADC_INJ_StartConversion(ADCx_1);
    LL_ADC_INJ_StartConversion(ADCx_2);

    return;
}

/**
 * @brief  It turns on low sides switches and start ADC triggering.
 *         This function is specific for MP phase.
 * @param  phandle Pointer on the target component instance
 * @retval none
 */
void rl_turn_on_low_sides_and_start(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_r3_2_t* phandle = (pwmc_r3_2_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    LL_TIM_ClearFlag_UPDATE(TIMx);
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {
    }
    /* Clear Update Flag */
    LL_TIM_ClearFlag_UPDATE(TIMx);

    LL_TIM_OC_SetCompareCH1(TIMx, 0x0u);
    LL_TIM_OC_SetCompareCH2(TIMx, 0x0u);
    LL_TIM_OC_SetCompareCH3(TIMx, 0x0u);
    LL_TIM_OC_SetCompareCH4(TIMx, (phandle->half_pwm_period - 5u));

    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0) {
    }

    /* Main PWM Output Enable */
    TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
    LL_TIM_EnableAllOutputs(TIMx);

    if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
        /* It is executed during calibration phase the EN signal shall stay off */
        LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
        LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
        LL_GPIO_SetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
    }

    ptr_hdl->Sector = SECTOR_4;

    LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

    LL_TIM_EnableIT_UPDATE(TIMx);

    return;
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
