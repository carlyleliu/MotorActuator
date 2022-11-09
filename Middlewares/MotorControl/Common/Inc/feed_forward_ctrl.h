/**
 ******************************************************************************
 * @file    feed_forward_ctrl.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Feed Forward Control component of the motor Control SDK.
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
 * @ingroup FeedForwardCtrl
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FEEDFORWARDCTRL_H
#define __FEEDFORWARDCTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"
#include "mc_type.h"
#include "speed_pos_fdbk.h"
#include "speed_torq_ctrl.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup FeedForwardCtrl
 * @{
 */

/**
 * @brief Handle structure of the Feed Forward Component
 */
typedef struct {
    qd_t vqd_ff;                          /**< Feed Forward controller @f$I_{qd}@f$ contribution to @f$V_{qd}@f$ */
    qd_t vqd_pi_out;                      /**< @f$V_{qd}@f$ as output by PID controller */
    qd_t vqd_av_pi_out;                   /**< Averaged @f$V_{qd}@f$ as output by PID controller */
    int32_t constant_1d;                  /**< Feed forward default constant for the @f$d@f$ axis */
    int32_t constant_1q;                  /**< Feed forward default constant for the @f$q@f$ axis */
    int32_t constant_2;                   /**< Default constant value used by Feed-Forward algorithm */
    bus_voltage_sensor_t* ptr_bus_sensor; /**< Related bus voltage sensor */
    pid_integer_t* ptr_pid_q;             /*!< Related PI for @f$I_{q}@f$ regulator */
    pid_integer_t* ptr_pid_d;             /*!< Related PI for @f$I_{d}@f$ regulator */
    uint16_t vqd_low_pass_filter_bw;
    int32_t def_constant_1d;             /**< Feed forward default constant for d axes */
    int32_t def_constant_1q;             /**< Feed forward default constant for q axes */
    int32_t def_constant_2;              /**< Default constant value used by
                                          Feed-Forward algorithm*/
    uint16_t vqd_low_pass_filter_bw_log; /**< vqd_low_pass_filter_bw expressed as power of 2.
                                          E.g. if gain divisor is 512 the value
                                          must be 9 because 2^9 = 512 */

} feed_forward_t;

/**
 * @brief  Initializes all the component variables
 * @param  phandle Feed forward init strutcture.
 * @param  ptr_bus_sensor VBus Sensor.
 * @param  ptr_pid_id Id PID.
 * @param  ptr_pid_iq Iq PID.
 * @retval none
 */
void feed_forward_init(feed_forward_t* phandle, bus_voltage_sensor_t* ptr_bus_sensor, pid_integer_t* ptr_pid_id, pid_integer_t* ptr_pid_iq);

/**
 * @brief  It should be called before each motor restart and clears the Flux
 *         weakening internal variables.
 * @param  phandle Feed forward init strutcture.
 * @retval none
 */
void feed_forward_clear(feed_forward_t* phandle);

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
void feed_forward_vqdff_computation(feed_forward_t* phandle, qd_t iqd_ref, speedn_torq_ctrl_t* ptr_stc);

/**
 * @brief  It return the vqd componets fed in input plus the feed forward
 *         action and store the last vqd values in the internal variable.
 * @param  phandle Feed forward  strutcture.
 * @param  vqd Initial value of vqd to be manipulated by FF.
 * @retval none
 */
qd_t feed_forward_vqd_conditioning(feed_forward_t* phandle, qd_t vqd);

/**
 * @brief  It low-pass filters the vqd voltage coming from the speed PI. Filter
 *         bandwidth depends on vqd_low_pass_filter_bw parameter.
 * @param  phandle Feed forward  strutcture.
 * @retval none
 */
void feed_forward_data_process(feed_forward_t* phandle);

/**
 * @brief  Use this method to initialize FF vars in START_TO_RUN state.
 * @param  phandle Feed forward  strutcture.
 * @retval none
 */
void feed_forward_init_foc_additional_methods(feed_forward_t* phandle);

/**
 * @brief  Use this method to set new values for the constants utilized by
 *         feed-forward algorithm.
 * @param  phandle Feed forward  strutcture.
 * @param  new_constants The ff_tuning_struct_t containing constants utilized by
 *         feed-forward algorithm.
 * @retval none
 */
void feed_forward_setff_constants(feed_forward_t* phandle, ff_tuning_struct_t new_constants);

/**
 * @brief  Use this method to get present values for the constants utilized by
 *         feed-forward algorithm.
 * @param  phandle Feed forward  strutcture.
 * @retval ff_tuning_struct_t Values of the constants utilized by
 *         feed-forward algorithm.
 */
ff_tuning_struct_t feed_forward_getff_constants(feed_forward_t* phandle);

/**
 * @brief  Use this method to get present values for the vqd feed-forward
 *         components.
 * @param  phandle Feed forward  strutcture.
 * @retval qd_t vqd feed-forward components.
 */
qd_t feed_forward_get_vqdff(const feed_forward_t* phandle);

/**
 * @brief  Use this method to get values of the averaged output of qd axes
 *         currents PI regulators.
 * @param  phandle Feed forward  strutcture.
 * @retval qd_t Averaged output of qd axes currents PI regulators.
 */
qd_t feed_forward_get_vqd_av_pi_out(const feed_forward_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __FEEDFORWARDCTRL_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
