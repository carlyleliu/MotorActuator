
/**
 ******************************************************************************
 * @file    regular_conversion_manager.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          regular_conversion_manager component of the motor Control SDK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __regular_conversion_manager_h
#define __regular_conversion_manager_h

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_stm_types.h"
#include "r3_2_g4xx_pwm_curr_fdbk.h"
#include "stdbool.h"
#include "stdint.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup RCM
 * @{
 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief reg_conv_t contains all the parameters required to execute a regular conversion
 *
 * it is used by all regular_conversion_manager's client
 *
 */
typedef struct {
    ADC_TypeDef* reg_adc;
    uint8_t channel;
    uint32_t sampling_time;
} reg_conv_t;

typedef enum { RCM_USERCONV_IDLE, RCM_USERCONV_REQUESTED, RCM_USERCONV_EOC } rcm_user_conv_state_t;

typedef void (*rcm_exec_cb_t)(uint8_t handle, uint16_t data, void* UserData);

/* Exported functions ------------------------------------------------------- */

/*  Function used to register a regular conversion */
uint8_t rcm_register_reg_conv(reg_conv_t* regConv);

/*  Function used to register a regular conversion with a callback attached*/
uint8_t rcm_register_reg_conv_with_cb(reg_conv_t* regConv, rcm_exec_cb_t fctCB, void* data);

/*  Function used to execute an already registered regular conversion */
uint16_t rcm_exec_regular_conv(uint8_t handle);

/* select the handle conversion to be executed during the next call to rcm_exec_user_conv */
bool rcm_request_user_conv(uint8_t handle);

/* return the latest user conversion value*/
uint16_t rcm_get_user_conv(void);

/* Must be called by MC_TASK only to grantee proper scheduling*/
void rcm_exec_user_conv(void);

/* return the state of the user conversion state machine*/
rcm_user_conv_state_t rcm_get_user_conv_state(void);

/* Function used to un-schedule a regular conversion exectuted after current sampling in HF task */
bool rcm_pause_regular_conv(uint8_t handle);

/* non blocking function to start conversion inside HF task */
void rcm_exec_next_conv(void);

/* non blocking function used to read back already started regular conversion*/
void rcm_read_ongoing_conv(void);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __regular_conversion_manager_h */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
