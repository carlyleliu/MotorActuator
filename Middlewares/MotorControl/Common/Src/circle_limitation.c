/**
 ******************************************************************************
 * @file    circle_limitation.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides the functions that implement the circle
 *          limitation feature of the STM32 motor Control SDK.
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
#include "circle_limitation.h"

#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup CircleLimitation Circle Limitation
 * @brief Circle Limitation component of the motor Control SDK
 *
 * @todo Document the Circle Limitation "module".
 *
 * @{
 */

#if defined(CIRCLE_LIMITATION_VD)
__weak qd_t circle_limitation(circle_limitation_t* phandle, qd_t vqd)
{
    int32_t max_module;
    int32_t square_q;
    int32_t square_temp;
    int32_t square_d;
    int32_t square_sum;
    int32_t square_limit;
    int32_t vd_square_limit;
    int32_t new_q;
    int32_t new_d;
    qd_t local_vqd = vqd;

    max_module = phandle->max_module;

    square_q = (int32_t)(vqd.q) * vqd.q;
    square_d = (int32_t)(vqd.d) * vqd.d;
    square_limit = max_module * max_module;
    vd_square_limit = phandle->max_vd * phandle->max_vd;
    square_sum = square_q + square_d;

    if (square_sum > square_limit) {
        if (square_d <= vd_square_limit) {
            square_temp = square_limit - square_d;
            new_q = mcm_sqrt(square_temp);
            if (vqd.q < 0) {
                new_q = -new_q;
            }
            new_d = vqd.d;
        } else {
            new_d = phandle->max_vd;
            if (vqd.d < 0) {
                new_d = -new_d;
            }

            square_temp = square_limit - vd_square_limit;
            new_q = mcm_sqrt(square_temp);
            if (vqd.q < 0) {
                new_q = -new_q;
            }
        }
        local_vqd.q = new_q;
        local_vqd.d = new_d;
    }
    return (local_vqd);
}
#else
/**
 * @brief Check whether vqd.q^2 + vqd.d^2 <= 32767^2
 *        and if not it applies a limitation keeping constant ratio
 *        vqd.q / vqd.d
 * @param  phandle pointer on the related component instance
 * @param  vqd Voltage in qd reference frame
 * @retval qd_t Limited vqd vector
 */
__weak qd_t circle_limitation(circle_limitation_t* phandle, qd_t vqd)
{
    uint16_t table_element;
    uint32_t uw_temp;
    int32_t sw_temp;
    qd_t local_vqd = vqd;

    sw_temp = (int32_t)(vqd.q) * vqd.q + (int32_t)(vqd.d) * vqd.d;

    uw_temp = (uint32_t)sw_temp;

    /* uw_temp min value 0, max value 32767*32767 */
    if (uw_temp > (uint32_t)(phandle->max_module) * phandle->max_module) {
        uw_temp /= (uint32_t)(16777216);

        /* wtemp min value phandle->start_index, max value 127 */
        uw_temp -= phandle->start_index;

        /* uw_temp min value 0, max value 127 - phandle->start_index */
        table_element = phandle->circle_limit_table[(uint8_t)uw_temp];

        sw_temp = vqd.q * (int32_t)table_element;
        local_vqd.q = (int16_t)(sw_temp / 32768);

        sw_temp = vqd.d * (int32_t)(table_element);
        local_vqd.d = (int16_t)(sw_temp / 32768);
    }

    return (local_vqd);
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
