/**
 ******************************************************************************
 * @file    ics_g4xx_pwm_curr_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement current sensor
 *          class to be stantiated when the three shunts current sensing
 *          topology is used. It is specifically designed for STM32G4X
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
#include "ics_g4xx_pwm_curr_fdbk.h"

#include "mc_type.h"
#include "pwm_common.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup pwm_curr_fdbk
 * @{
 */

/**
 * @defgroup icS_G4XX_pwm_curr_fdbk icS 2 ADCs G4 PWM & Current Feedback
 *
 * @brief STM32G4, Shared Resources, icS PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32G4 MCU, using an icS
 * current sensing topology and 2 ADC peripherals to acquire the current values.
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
static void ics_timx_init(TIM_TypeDef* TIMx, pwmc_t* ptr_hdl);
static void ics_adcx_init(ADC_TypeDef* ADCx);
static void ics_hf_currents_polarization(pwmc_t* ptr_hdl, ab_t* iab);

/**
 * @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVic for current reading
 *         in icS topology using STM32G4X and shared ADC
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void ics_init(pwmc_ics_t* phandle)
{
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;

    /*Check that _Super is the first member of the structure pwmc_ics_t */
    if ((uint32_t)phandle == (uint32_t)&phandle->_Super) {
        /* disable IT and flags in case of LL driver usage
         * workaround for unwanted interrupt enabling done by LL driver */
        LL_ADC_DisableIT_EOC(ADCx_1);
        LL_ADC_ClearFlag_EOC(ADCx_1);
        LL_ADC_DisableIT_JEOC(ADCx_1);
        LL_ADC_ClearFlag_JEOC(ADCx_1);
        LL_ADC_DisableIT_EOC(ADCx_2);
        LL_ADC_ClearFlag_EOC(ADCx_2);
        LL_ADC_DisableIT_JEOC(ADCx_2);
        LL_ADC_ClearFlag_JEOC(ADCx_2);

        if (TIMx == TIM1) {
            /* TIM1 Counter Clock stopped when the core is halted */
            LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
        } else {
            /* TIM8 Counter Clock stopped when the core is halted */
            LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
        }

        if (LL_ADC_IsEnabled(ADCx_1) == 0) {
            ics_adcx_init(ADCx_1);
            /* Only the Interrupt of the first ADC is enabled.
             * As Both ADCs are fired by hw at the same moment
             * It is safe to consider that both conversion are ready at the same time*/
            LL_ADC_ClearFlag_JEOS(ADCx_1);
            LL_ADC_EnableIT_JEOS(ADCx_1);
        } else {
            /* Nothing to do ADCx_1 already configured */
        }
        if (LL_ADC_IsEnabled(ADCx_2) == 0) {
            ics_adcx_init(ADCx_2);
        } else {
            /* Nothing to do ADCx_2 already configured */
        }
        ics_timx_init(TIMx, &phandle->_Super);
    }
}

static void ics_adcx_init(ADC_TypeDef* ADCx)
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
static void ics_timx_init(TIM_TypeDef* TIMx, pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    uint32_t brk2_timeout = 1000;

    /* disable main TIM counter to ensure
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
    /* Prepare timer for synchronization */
    LL_TIM_GenerateEvent_UPDATE(TIMx);
    if (phandle->ptr_params_str->freq_ratio == 2u) {
        if (phandle->ptr_params_str->is_higher_freq_tim == HIGHER_FREQ) {
            if (phandle->ptr_params_str->RepetitionCounter == 3u) {
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
            if (phandle->ptr_params_str->RepetitionCounter == 1u) {
                LL_TIM_SetCounter(TIMx, (uint32_t)(phandle->half_pwm_period) - 1u);
            } else if (phandle->ptr_params_str->RepetitionCounter == 3u) {
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
    LL_TIM_EnableIT_BRK(TIMx);

    /* Enable PWM channel */
    LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);
}

/**
 * @brief  It stores into the component the voltage present on ia and
 *         ib current feedback analog channels when no current is flowing into the
 *         motor
 * @param  ptr_hdl: handler of the current instance of the PWM component
 * @retval none
 */
__weak void ics_current_reading_polarization(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;

    phandle->phase_a_offset = 0u;
    phandle->phase_b_offset = 0u;

    phandle->polarization_counter = 0u;

    /* It forces inactive level on TIMx CHy and CHyN */
    LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);

    /* Offset calibration for all phases */
    /* Change function to be executed in ADCx_ISR */
    phandle->_Super.fct_get_phase_currents = &ics_hf_currents_polarization;
    ics_switch_on_pwm(&phandle->_Super);

    /* IF CH4 is enabled, it means that JSQR is now configured to sample polarization current*/
    while (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_OC4REF) {
    }
    /* It is the right time to start the ADC without unwanted conversion */
    /* Start ADC to wait for external trigger. This is series dependent*/
    LL_ADC_INJ_StartConversion(ADCx_1);
    LL_ADC_INJ_StartConversion(ADCx_2);

    /* Wait for NB_CONVERSIONS to be executed */
    // wait_for_polarization_end(TIMx, &phandle->_Super.sw_error, phandle->ptr_params_str->RepetitionCounter, &phandle->polarization_counter);

    ics_switch_off_pwm(&phandle->_Super);

    phandle->phase_a_offset /= NB_CONVERSIONS;
    phandle->phase_b_offset /= NB_CONVERSIONS;

    /* Change back function to be executed in ADCx_ISR */
    phandle->_Super.fct_get_phase_currents = &ics_get_phase_currents;
    phandle->_Super.fct_set_adc_samp_point_sectx = &ics_write_tim_registers;

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
__weak void ics_get_phase_currents(pwmc_t* ptr_hdl, ab_t* iab)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;
    int32_t aux;
    uint16_t reg;

    /* disable ADC trigger source */
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

    /* ia = (hphase_a_offset)-(PHASE_A_ADC_CHANNEL value)  */
    reg = (uint16_t)(ADCx_1->JDR1);
    aux = (int32_t)(reg) - (int32_t)(phandle->phase_a_offset);

    /* Saturation of ia */
    if (aux < -INT16_MAX) {
        iab->a = -INT16_MAX;
    } else if (aux > INT16_MAX) {
        iab->a = INT16_MAX;
    } else {
        iab->a = (int16_t)aux;
    }

    /* ib = (hphase_b_offset)-(PHASE_B_ADC_CHANNEL value) */
    reg = (uint16_t)(ADCx_2->JDR1);
    aux = (int32_t)(reg) - (int32_t)(phandle->phase_b_offset);

    /* Saturation of ib */
    if (aux < -INT16_MAX) {
        iab->b = -INT16_MAX;
    } else if (aux > INT16_MAX) {
        iab->b = INT16_MAX;
    } else {
        iab->b = (int16_t)aux;
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
 * @brief  Stores into the component's handle the voltage present on ia and
 *         ib current feedback analog channels when no current is flowing into the
 *         motor
 * @param  phandle handler of the current instance of the PWM component
 * @retval none
 */
__weak uint16_t ics_write_tim_registers(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    uint16_t aux;

    LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)phandle->_Super.cnt_pha);
    LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)phandle->_Super.cnt_phb);
    LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)phandle->_Super.cnt_phc);

    /* Limit for update event */
    if (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET) {
        aux = MC_FOC_DURATION;
    } else {
        aux = MC_NO_ERROR;
    }
    return aux;
}
/**
 * @brief  Implementation of pwmc_get_phase_currents to be performed during
 *         calibration. It sum up injected conversion data into phase_a_offset and
 *         phase_b_offset to compute the offset introduced in the current feedback
 *         network. It is required to proper configure ADC inputs before to enable
 *         the offset computation.
 * @param  ptr_hdl Pointer on the target component instance
 * @retval It always returns {0,0} in Curr_Components format
 */
static void ics_hf_currents_polarization(pwmc_t* ptr_hdl, ab_t* iab)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;

    /* disable ADC trigger source */
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

    if (phandle->polarization_counter < NB_CONVERSIONS) {
        phandle->phase_a_offset += ADCx_1->JDR1;
        phandle->phase_b_offset += ADCx_2->JDR1;
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
__weak void ics_turn_on_low_sides(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
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
__weak void ics_switch_on_pwm(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    phandle->_Super.turn_on_low_sides_action = false;

    /* Set all duty to 50% */
    LL_TIM_OC_SetCompareCH1(TIMx, ((uint32_t)phandle->half_pwm_period / (uint32_t)2));
    LL_TIM_OC_SetCompareCH2(TIMx, ((uint32_t)phandle->half_pwm_period / (uint32_t)2));
    LL_TIM_OC_SetCompareCH3(TIMx, ((uint32_t)phandle->half_pwm_period / (uint32_t)2));
    LL_TIM_OC_SetCompareCH4(TIMx, ((uint32_t)phandle->half_pwm_period - (uint32_t)5));

    /* wait for a new PWM period */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0u) {
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
__weak void ics_switch_off_pwm(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;

    /* Disable UPDATE ISR */
    LL_TIM_DisableIT_UPDATE(TIMx);

    phandle->_Super.turn_on_low_sides_action = false;

    /* Main PWM Output Disable */
    if (phandle->brake_action_lock == true) {
    } else {
        TIMx->BDTR &= ~((uint32_t)(LL_TIM_OSSI_ENABLE));
        LL_TIM_DisableAllOutputs(TIMx);

        if ((phandle->ptr_params_str->low_side_outputs) == ES_GPIO) {
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_u_port, phandle->ptr_params_str->pwm_en_u_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_v_port, phandle->ptr_params_str->pwm_en_v_pin);
            LL_GPIO_ResetOutputPin(phandle->ptr_params_str->pwm_en_w_port, phandle->ptr_params_str->pwm_en_w_pin);
        }
    }

    /* wait for a new PWM period to flush last HF task */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 0u) {
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
 * @brief  It contains the TIMx Update event interrupt
 * @param  phandle: handler of the current instance of the PWM component
 * @retval none
 */
__weak void* ics_timx_up_irq_handler(pwmc_ics_t* phandle)
{
    TIM_TypeDef* TIMx = phandle->ptr_params_str->TIMx;
    ADC_TypeDef* ADCx_1 = phandle->ptr_params_str->ADCx_1;
    ADC_TypeDef* ADCx_2 = phandle->ptr_params_str->ADCx_2;

    ADCx_1->JSQR = (uint32_t)phandle->ptr_params_str->adc_config1;
    ADCx_2->JSQR = (uint32_t)phandle->ptr_params_str->adc_config2;

    /* enable ADC trigger source */
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);

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
__weak void* ics_brk2_irq_handler(pwmc_ics_t* phandle)
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
__weak void* ics_brk_irq_handler(pwmc_ics_t* phandle)
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
__weak uint16_t ics_is_over_current_occurred(pwmc_t* ptr_hdl)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    pwmc_ics_t* phandle = (pwmc_ics_t*)ptr_hdl;
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
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
