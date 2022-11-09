#ifndef FRAMEWORK_PLATFORM_H
#define FRAMEWORK_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "device.h"
#include "mc_tuning.h"
#include "mc_type.h"
#include "pwm_curr_fdbk.h"
#include "r3_2_g4xx_pwm_curr_fdbk.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ       170000000uL
#define TIM_CLOCK_DIVIDER 1
#define ADV_TIM_CLK_MHz   170
#define ADC_CLK_MHz       42
#define HALL_TIM_CLK      170000000uL
#define APB1TIM_FREQ      170000000uL

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define ADV_TIM_CLK_MHz 170
#define PWM_FREQUENCY   30000
#define SYS_FREQ        2000

#define SAMPLING_CYCLE_CORRECTION      0.5 /* Add half cycle required by STM32G431RBTx ADC */
#define LL_ADC_SAMPLING_TIME_1CYCLES_5 LL_ADC_SAMPLINGTIME_1CYCLE_5
#define LL_ADC_SAMPLING_CYCLE(CYCLE)   LL_ADC_SAMPLINGTIME_##CYCLE##CYCLES_5

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_PERIOD_CYCLES (uint16_t)(ADV_TIM_CLK_MHz * (uint32_t)1000000u / ((uint32_t)(PWM_FREQUENCY)))

#define PLATFORM_INFO(cpu)                  \
    static platform_t cpu##_info = {        \
    .sysclk_freq = SYSCLK_FREQ,             \
    .tim_clock_divider = TIM_CLOCK_DIVIDER, \
    .adv_tim_clk_mhz = ADV_TIM_CLK_MHz,     \
    .pwm_frequency = PWM_FREQUENCY,         \
    .sys_frequency = SYS_FREQ,              \
    }

typedef struct platform {
    uint32_t sysclk_freq;
    uint16_t tim_clock_divider;
    uint16_t adv_tim_clk_mhz;
    uint16_t adc_clk_mhz;
    uint16_t pwm_frequency;
    uint16_t sys_frequency;
} platform_t;

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif
