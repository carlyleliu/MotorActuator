/**
 ******************************************************************************
 * @file    feed_forward_ctrl.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the Feed Forward
 *          Control component of the motor Control SDK.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
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
#include "feed_forward_ctrl.h"

#include <stddef.h>

#include "bus_voltage_sensor.h"
#include "mc_type.h"
#include "r_divider_bus_voltage_sensor.h"
#include "speed_pos_fdbk.h"
#include "speed_torq_ctrl.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup FeedForwardCtrl Feed Forward Control
 * @brief Feed Forward Control component of the motor Control SDK
 *
 * @todo Document the Feed Forward Control "module".
 *
 * @{
 */

/* Private macros ------------------------------------------------------------*/
#define SEGMNUM (uint8_t)7 /* coeff no. -1 */
#define SATURATION_TO_S16(a)   \
    if ((a) > 32767) {         \
        (a) = 32767;           \
    } else if ((a) < -32767) { \
        (a) = -32767;          \
    } else {                   \
    }

/**
 * @brief  Initializes all the component variables
 * @param  phandle Feed forward init strutcture.
 * @param  ptr_bus_sensor VBus Sensor.
 * @param  ptr_pid_id Id PID.
 * @param  ptr_pid_iq Iq PID.
 * @retval none
 */
__weak void feed_forward_init(feed_forward_t* phandle, bus_voltage_sensor_t* ptr_bus_sensor, pid_integer_t* ptr_pid_id, pid_integer_t* ptr_pid_iq)
{
    phandle->constant_1d = phandle->def_constant_1d;
    phandle->constant_1q = phandle->def_constant_1q;
    phandle->constant_2 = phandle->def_constant_2;

    phandle->ptr_bus_sensor = ptr_bus_sensor;

    phandle->ptr_pid_d = ptr_pid_id;

    phandle->ptr_pid_q = ptr_pid_iq;
}

/**
 * @brief  It should be called before each motor restart and clears the Flux
 *         weakening internal variables.
 * @param  phandle Feed forward  strutcture.
 * @retval none
 */
__weak void feed_forward_clear(feed_forward_t* phandle)
{
    phandle->vqd_ff.q = (int16_t)0;
    phandle->vqd_ff.d = (int16_t)0;

    return;
}

/**
 * @brief  It implements feed-forward controller by computing new vqd_ff value.
 *         This will be then summed up to PI output in IMFF_vqdConditioning
 *         method.
 * @param  phandle Feed forward  strutcture.
 * @param  iqd_ref Idq reference componets used to calcupate the feed forward
 *         action.
 * @param  ptr_stc  Speed sensor.
 * @retval none
 */
__weak void feed_forward_vqdff_computation(feed_forward_t* phandle, qd_t iqd_ref, speedn_torq_ctrl_t* ptr_stc)
{
    int32_t temp1, temp2;
    int16_t speed_dpp;
    uint16_t avbus_voltage_d;
    speedn_pos_fdbk_t* speed_sensor;

    speed_sensor = stc_get_speed_sensor(ptr_stc);
    speed_dpp = spd_get_el_speed_dpp(speed_sensor);
    avbus_voltage_d = vbs_get_avbus_voltage_d(phandle->ptr_bus_sensor) / 2u;

    if (avbus_voltage_d != (uint16_t)0) {
        /*q-axes ff voltage calculation */
        temp1 = (((int32_t)(speed_dpp)*iqd_ref.d) / (int32_t)32768);
        temp2 = (temp1 * phandle->constant_1d) / (int32_t)(avbus_voltage_d);
        temp2 *= (int32_t)2;

        temp1 = ((phandle->constant_2 * speed_dpp) / (int32_t)avbus_voltage_d) * (int32_t)16;

        temp2 = temp1 + temp2 + phandle->vqd_av_pi_out.q;

        SATURATION_TO_S16(temp2)

        phandle->vqd_ff.q = (int16_t)(temp2);

        /* d-axes ff voltage calculation */
        temp1 = (((int32_t)(speed_dpp)*iqd_ref.q) / (int32_t)32768);
        temp2 = (temp1 * phandle->constant_1q) / (int32_t)(avbus_voltage_d);
        temp2 *= (int32_t)2;

        temp2 = (int32_t)phandle->vqd_av_pi_out.d - temp2;

        SATURATION_TO_S16(temp2)

        phandle->vqd_ff.d = (int16_t)(temp2);
    } else {
        phandle->vqd_ff.q = (int16_t)0;
        phandle->vqd_ff.d = (int16_t)0;
    }
}

#if defined(__ICCARM__)
// false positive
#pragma cstat_disable = "MISRAC2012-Rule-2.2_b"
#endif /* __ICCARM__ */

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It return the vqd componets fed in input plus the feed forward
 *         action and store the last vqd values in the internal variable.
 * @param  phandle Feed forward  strutcture.
 * @param  vqd Initial value of vqd to be manipulated by FF .
 * @retval qd_t vqd conditioned values.
 */
__weak qd_t feed_forward_vqd_conditioning(feed_forward_t* phandle, qd_t vqd)
{
    int32_t temp;
    qd_t lvqd;

    phandle->vqd_pi_out = vqd;

    temp = (int32_t)(vqd.q) + phandle->vqd_ff.q;

    SATURATION_TO_S16(temp)

    lvqd.q = (int16_t)temp;

    temp = (int32_t)(vqd.d) + phandle->vqd_ff.d;

    SATURATION_TO_S16(temp)

    lvqd.d = (int16_t)temp;

    return (lvqd);
}
#if defined(__ICCARM__)
// false positive
#pragma cstat_restore = "MISRAC2012-Rule-2.2_b"
#endif /* __ICCARM__ */

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It low-pass filters the vqd voltage coming from the speed PI. Filter
 *         bandwidth depends on vqd_low_pass_filter_bw parameter.
 * @param  phandle Feed forward  strutcture.
 * @retval none
 */
__weak void feed_forward_data_process(feed_forward_t* phandle)
{
    int32_t aux;
    int32_t low_pass_filter_bw = (int32_t)phandle->vqd_low_pass_filter_bw - (int32_t)1;

#ifdef FULL_MISRA_C_COMPLIANCY
    /* Computation of average vqd as output by PI(D) current controllers, used by
     feed-forward controller algorithm */
    aux = (int32_t)(phandle->vqd_av_pi_out.q) * low_pass_filter_bw;
    aux += phandle->vqd_pi_out.q;

    phandle->vqd_av_pi_out.q = (int16_t)(aux / (int32_t)(phandle->vqd_low_pass_filter_bw));

    aux = (int32_t)(phandle->vqd_av_pi_out.d) * low_pass_filter_bw;
    aux += phandle->vqd_pi_out.d;

    phandle->vqd_av_pi_out.d = (int16_t)(aux / (int32_t)(phandle->vqd_low_pass_filter_bw));
#else
    /* Computation of average vqd as output by PI(D) current controllers, used by
     feed-forward controller algorithm */
    aux = (int32_t)(phandle->vqd_av_pi_out.q) * low_pass_filter_bw;
    aux += phandle->vqd_pi_out.q;
    phandle->vqd_av_pi_out.q = (int16_t)(aux >> phandle->vqd_low_pass_filter_bw_log);

    aux = (int32_t)(phandle->vqd_av_pi_out.d) * low_pass_filter_bw;
    aux += phandle->vqd_pi_out.d;
    phandle->vqd_av_pi_out.d = (int16_t)(aux >> phandle->vqd_low_pass_filter_bw_log);

#endif
}

/**
 * @brief  Use this method to initialize FF vars in START_TO_RUN state.
 * @param  phandle Feed forward  strutcture.
 * @retval none
 */
__weak void feed_forward_init_foc_additional_methods(feed_forward_t* phandle)
{
    phandle->vqd_av_pi_out.q = 0;
    phandle->vqd_av_pi_out.d = 0;
    pid_set_integral_term(phandle->ptr_pid_q, 0);
    pid_set_integral_term(phandle->ptr_pid_d, 0);
}

/**
 * @brief  Use this method to set new values for the constants utilized by
 *         feed-forward algorithm.
 * @param  phandle Feed forward  strutcture.
 * @param  new_constants The ff_tuning_struct_t containing constants utilized by
 *         feed-forward algorithm.
 * @retval none
 */
__weak void feed_forward_setff_constants(feed_forward_t* phandle, ff_tuning_struct_t new_constants)
{
    phandle->constant_1d = new_constants.const_1d;
    phandle->constant_1q = new_constants.const_1q;
    phandle->constant_2 = new_constants.const_2;
}

#if defined(__ICCARM__)
// false positive
#pragma cstat_disable = "MISRAC2012-Rule-2.2_b"
#endif /* __ICCARM__ */

/**
 * @brief  Use this method to get present values for the constants utilized by
 *         feed-forward algorithm.
 * @param  phandle Feed forward  strutcture.
 * @retval ff_tuning_struct_t Values of the constants utilized by
 *         feed-forward algorithm.
 */
__weak ff_tuning_struct_t feed_forward_getff_constants(feed_forward_t* phandle)
{
    ff_tuning_struct_t local_constants;

    local_constants.const_1d = phandle->constant_1d;
    local_constants.const_1q = phandle->constant_1q;
    local_constants.const_2 = phandle->constant_2;

    return (local_constants);
}
#if defined(__ICCARM__)
// false positive
#pragma cstat_restore = "MISRAC2012-Rule-2.2_b"
#endif /* __ICCARM__ */

/**
 * @brief  Use this method to get present values for the vqd feed-forward
 *         components.
 * @param  phandle Feed forward  strutcture.
 * @retval qd_t vqd feed-forward components.
 */
__weak qd_t feed_forward_get_vqdff(const feed_forward_t* phandle)
{
    return (phandle->vqd_ff);
}

/**
 * @brief  Use this method to get values of the averaged output of qd axes
 *         currents PI regulators.
 * @param  phandle Feed forward  strutcture.
 * @retval qd_t Averaged output of qd axes currents PI regulators.
 */
__weak qd_t feed_forward_get_vqd_av_pi_out(const feed_forward_t* phandle)
{
    return (phandle->vqd_ff);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
