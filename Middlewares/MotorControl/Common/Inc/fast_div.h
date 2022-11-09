/**
 ******************************************************************************
 * @file    fast_div.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          Fast Division component of the motor Control SDK.
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
 * @ingroup FD
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FAST_DIV_H
#define __FAST_DIV_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup FD
 * @{
 */

/* Exported constants --------------------------------------------------------*/
/// Number of divisors that can be stored for fast division
#define FD_MAX_FDIV 5

/* Exported types ------------------------------------------------------------*/

/// Magic number structure (used for software fast division)
typedef struct {
    int32_t m;
    int32_t s;
    int32_t d;
} fd_fast_div_magic_number_t;

/// Handle structure of the Fast Division component
typedef struct {
    int8_t fd_div_element;                        /*@< Pointer to the stored dividers buffer.*/
    fd_fast_div_magic_number_t fd_m[FD_MAX_FDIV]; /*@< Struct of stored magic numbers.*/
} fast_div_t;

/* Exported functions ------------------------------------------------------- */

void fd_init(fast_div_t* phandle);
int32_t fd_fast_div(fast_div_t* phandle, int32_t n, int32_t d);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __FAST_DIV_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
