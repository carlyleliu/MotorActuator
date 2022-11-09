/**
 ******************************************************************************
 * @file    inrush_current_limiter.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          InrushCurrentLimiter component featuring the motor Control SDK
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
 * @ingroup icL
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INRUSHCURRENTLIMITER_H
#define __INRUSHCURRENTLIMITER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"
#include "digital_output.h"
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup icL
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/*
 * @brief icl_state_t defines all the existing icL states of the state machine
 */
typedef enum {
    ICL_IDLE,         /* stable state */
    ICL_ACTIVATION,   /* transition state */
    ICL_ACTIVE,       /* stable state */
    ICL_DEACTIVATION, /* transition state */
    ICL_INACTIVE      /* stable state */
} icl_state_t;

/**
 * @brief  icl_t is used to handle an instance of the InrushCurrentLimiter component
 */
typedef struct {
    bus_voltage_sensor_t* ptr_vbs; /*!< CVBS object used for the automatic icL component activation/deactivation */
    dout_t* p_dout;                /*!< DOUT object used to physically activate/deactivate the icL component */

    icl_state_t icl_state;      /*!< Current state of the icL state machine */
    uint16_t icl_ticks_counter; /*!< Number of clock events remaining to complete the icL activation/deactivation */
    uint16_t icl_total_ticks;   /*!< Total number of clock events to complete the icL activation/deactivation */
    uint16_t icl_frequency_hz;  /*!< Clock frequency used (Hz) to trigger the icl_exec() method */
    uint16_t icl_durationms;    /*!< icL activation/deactivation duration (ms)*/
} icl_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void icl_init(icl_t* phandle, bus_voltage_sensor_t* ptr_vbs, dout_t* p_dout);
icl_state_t icl_exec(icl_t* phandle);
icl_state_t icl_get_state(icl_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __INRUSHCURRENTLIMITER_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
