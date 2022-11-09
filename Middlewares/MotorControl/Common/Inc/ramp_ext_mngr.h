/**
 ******************************************************************************
 * @file    ramp_ext_mngr.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Ramp Extended Manager component of the motor Control SDK.
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
 * @ingroup RampExtMngr
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RAMPEXTMNGR_H
#define __RAMPEXTMNGR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

#ifdef FASTDIV
#include "fast_div.h"
#endif

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup RampExtMngr
 * @{
 */

/**
 * @brief  RampExtMngr Handle Definition.
 */
typedef struct {
    uint32_t frequency_hz;        /*!< Execution frequency expressed in Hz */
    int32_t target_final;         /*!< Backup of htarget_final to be applied in the
                                   last step.*/
    int32_t ext;                  /*!< Current state variable multiplied by 32768.*/
    uint32_t ramp_remaining_step; /*!< Number of steps remaining to complete the
                                   ramp.*/
    int32_t inc_dec_amount;       /*!< Increment/decrement amount to be applied to
                                   the reference value at each
                                   CalcTorqueReference.*/
    uint32_t scaling_factor;      /*!< Scaling factor between output value and
                                   its internal representation.*/
#ifdef FASTDIV
    /* (Fast division optimization for cortex-M0 micros)*/
    fast_div_t fd; /*!< Fast division obj.*/
#endif
} ramp_ext_mngr_t;

/* Exported functions ------------------------------------------------------- */
void remng_init(ramp_ext_mngr_t* phandle);
int32_t remng_calc(ramp_ext_mngr_t* phandle);
bool remng_exec_ramp(ramp_ext_mngr_t* phandle, int32_t target_final, uint32_t durationms);
int32_t remng_get_value(ramp_ext_mngr_t* phandle);
bool remng_ramp_completed(ramp_ext_mngr_t* phandle);
void remng_stop_ramp(ramp_ext_mngr_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __RAMPEXTMNGR_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
