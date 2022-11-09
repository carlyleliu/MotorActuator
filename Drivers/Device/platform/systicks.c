#include "systicks.h"

static uint32_t us_ticks = 0; /* system clock is 72Mhz */

/**
 * @brief init systicks
 * @param SYSCLK systick source
 * @retval None
 */
void sys_ticks_init(void)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // SysTick freq HCLK 72Mhz
    us_ticks = HAL_RCC_GetHCLKFreq() / 1000000;          // usTicks = 72Mhz/1000000 = 72 usTicks is Microseconds
}

/**
 * @brief delay nus Microseconds
 * @param nus Microseconds will be delay
 * @retval None
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD; // LOAD value
    ticks = nus * us_ticks;          // systicks nums
    told = SysTick->VAL;             // the value of now
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) // the timer is decrease
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; // after timeout then break
        }
    };
}

/**
 * @brief delay nus millisecond or you can use HAL_Delay()
 * @param nus millisecond will be delay
 * @retval None
 */
void delay_ms(uint16_t nms)
{
    uint32_t i;
    for (i = 0; i < nms; i++)
        delay_us(1000);
}

/**
 * @brief get system Microseconds ticks
 * @param None
 * @retval system ticks with Microseconds
 */
uint32_t micros(void)
{
// TODO
#define SYSTICK_ADJUST 1
    register uint32_t ms, cycle_cnt, us_ticks_t;
    do {
        ms = HAL_GetTick();
        cycle_cnt = SysTick->VAL;
    } while (ms != HAL_GetTick());
    us_ticks_t = us_ticks;
    return (ms * 1000) + (us_ticks_t * SYSTICK_ADJUST - cycle_cnt) / us_ticks_t;
}
