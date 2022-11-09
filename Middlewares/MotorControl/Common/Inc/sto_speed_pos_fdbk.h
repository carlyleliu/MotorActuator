/**
 ******************************************************************************
 * @file    sto_speed_pos_fdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains definitions and functions prototypes common to all
 *          State Observer based Speed & Position Feedback components of the motor
 *          Control SDK (the CORDIC and PLL implementations).
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
 * @ingroup SpeednPosFdbk_STO
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STO_SPEEDNPOSFDBK_H
#define __STO_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/** @addtogroup SpeednPosFdbk_STO
 * @{
 */

/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct sto sto_t;

typedef void (*sto_force_convergency1_cb_t)(sto_t* phandle);
typedef void (*sto_force_convergency2_cb_t)(sto_t* phandle);
typedef void (*sto_otf_reset_pll_cb_t)(sto_t* phandle);
typedef bool (*sto_speed_reliability_check_cb_t)(const sto_t* phandle);

/**
 * @brief  SpeednPosFdbk  handle definition
 */
struct sto {
    speedn_pos_fdbk_t* _Super;
    sto_force_convergency1_cb_t fct_force_convergency1;
    sto_force_convergency2_cb_t fct_force_convergency2;
    sto_otf_reset_pll_cb_t fct_sto_otf_reset_pll;
    sto_speed_reliability_check_cb_t fct_sto_speed_reliability_check;
};

/**
 * @}
 */

/**
 * @}
 */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__STO_SPEEDNPOSFDBK_H*/

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
