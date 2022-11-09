/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

#define _IN
#define _OUT
#define EXPORT

#define Start_Stop_Pin           GPIO_PIN_13
#define Start_Stop_GPIO_Port     GPIOC
#define Start_Stop_EXTI_IRQn     EXTI15_10_IRQn
#define M1_CURR_AMPL_W_Pin       GPIO_PIN_0
#define M1_CURR_AMPL_W_GPIO_Port GPIOC
#define M1_CURR_AMPL_U_Pin       GPIO_PIN_2
#define M1_CURR_AMPL_U_GPIO_Port GPIOC
#define M1_CURR_AMPL_V_Pin       GPIO_PIN_3
#define M1_CURR_AMPL_V_GPIO_Port GPIOC
#define DBG_DAC_CH1_Pin          GPIO_PIN_4
#define DBG_DAC_CH1_GPIO_Port    GPIOA
#define DBG_DAC_CH2_Pin          GPIO_PIN_5
#define DBG_DAC_CH2_GPIO_Port    GPIOA
#define M2_TEMPERATURE_Pin       GPIO_PIN_7
#define M2_TEMPERATURE_GPIO_Port GPIOE
#define M1_PWM_UL_Pin            GPIO_PIN_8
#define M1_PWM_UL_GPIO_Port      GPIOE
#define M1_PWM_UH_Pin            GPIO_PIN_9
#define M1_PWM_UH_GPIO_Port      GPIOE
#define M1_PWM_VL_Pin            GPIO_PIN_10
#define M1_PWM_VL_GPIO_Port      GPIOE
#define M1_PWM_VH_Pin            GPIO_PIN_11
#define M1_PWM_VH_GPIO_Port      GPIOE
#define M1_PWM_WL_Pin            GPIO_PIN_12
#define M1_PWM_WL_GPIO_Port      GPIOE
#define M1_PWM_WH_Pin            GPIO_PIN_13
#define M1_PWM_WH_GPIO_Port      GPIOE
#define M2_BUS_VOLTAGE_Pin       GPIO_PIN_14
#define M2_BUS_VOLTAGE_GPIO_Port GPIOE
#define M2_CURR_AMPL_U_Pin       GPIO_PIN_10
#define M2_CURR_AMPL_U_GPIO_Port GPIOD
#define M2_CURR_AMPL_V_Pin       GPIO_PIN_12
#define M2_CURR_AMPL_V_GPIO_Port GPIOD
#define M2_CURR_AMPL_W_Pin       GPIO_PIN_13
#define M2_CURR_AMPL_W_GPIO_Port GPIOD
#define M2_PWM_UH_Pin            GPIO_PIN_6
#define M2_PWM_UH_GPIO_Port      GPIOC
#define M2_PWM_VH_Pin            GPIO_PIN_7
#define M2_PWM_VH_GPIO_Port      GPIOC
#define M2_PWM_WH_Pin            GPIO_PIN_8
#define M2_PWM_WH_GPIO_Port      GPIOC
#define M2_OCP_Pin               GPIO_PIN_9
#define M2_OCP_GPIO_Port         GPIOC
#define UART_TX_Pin              GPIO_PIN_9
#define UART_TX_GPIO_Port        GPIOA
#define UART_RX_Pin              GPIO_PIN_10
#define UART_RX_GPIO_Port        GPIOA
#define M1_OCP_Pin               GPIO_PIN_11
#define M1_OCP_GPIO_Port         GPIOA
#define TMS_Pin                  GPIO_PIN_13
#define TMS_GPIO_Port            GPIOA
#define TCK_Pin                  GPIO_PIN_14
#define TCK_GPIO_Port            GPIOA
#define M2_PWM_UL_Pin            GPIO_PIN_10
#define M2_PWM_UL_GPIO_Port      GPIOC
#define M2_PWM_VL_Pin            GPIO_PIN_11
#define M2_PWM_VL_GPIO_Port      GPIOC
#define M2_PWM_WL_Pin            GPIO_PIN_12
#define M2_PWM_WL_GPIO_Port      GPIOC

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

/* Exported functions prototypes ---------------------------------------------*/
void error_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
