
/**
 ******************************************************************************
 * @file    mc_math.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides mathematics functions useful for and specific to
 *          motor Control.
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
 * @ingroup MC_Math
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_MATH_H
#define MC_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup MC_Math
 * @{
 */
#define SQRT_2 1.4142
#define SQRT_3 1.732

/* CORDIC coprocessor configuration register settings */

/* CORDIC FUNCTION: PHASE q1.31 (Electrical Angle computation) */
#define CORDIC_CONFIG_PHASE                                                                           \
    (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_2CYCLES | LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_2 \
     | LL_CORDIC_NBREAD_1 | LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/* CORDIC FUNCTION: SQUAREROOT q1.31 */
#define CORDIC_CONFIG_SQRT                                                                                 \
    (LL_CORDIC_FUNCTION_SQUAREROOT | LL_CORDIC_PRECISION_2CYCLES | LL_CORDIC_SCALE_1 | LL_CORDIC_NBWRITE_1 \
     | LL_CORDIC_NBREAD_1 | LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/* CORDIC FUNCTION: COSINE q1.15 */
#define CORDIC_CONFIG_COSINE                                                                           \
    (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_4CYCLES | LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_1 \
     | LL_CORDIC_NBREAD_1 | LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)

/**
 * @brief  Macro to compute logarithm of two
 */
#define LOG2(x)                                                          \
    ((x) == 65535 ?                                                      \
     16 :                                                                \
     ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 ? \
      15 :                                                               \
      ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 ?    \
       14 :                                                              \
       ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 ?       \
        13 :                                                             \
        ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 ?          \
         12 :                                                            \
         ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 ?             \
          11 :                                                           \
          ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 ?                \
           10 :                                                          \
           ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 ?                   \
            9 :                                                          \
            ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 * 2 ?                      \
             8 :                                                         \
             ((x) == 2 * 2 * 2 * 2 * 2 * 2 * 2 ?                         \
              7 :                                                        \
              ((x) == 2 * 2 * 2 * 2 * 2 * 2 ?                            \
               6 :                                                       \
               ((x) == 2 * 2 * 2 * 2 * 2 ?                               \
                5 :                                                      \
                ((x) == 2 * 2 * 2 * 2 ? 4 : ((x) == 2 * 2 * 2 ? 3 : ((x) == 2 * 2 ? 2 : ((x) == 2 ? 1 : ((x) == 1 ? 0 : -1)))))))))))))))))

/**
 * @brief  Trigonometrical functions type definition
 */
typedef struct {
    int16_t cos;
    int16_t sin;
} trig_components;

/**
 * @brief  This function transforms stator currents ia and qib (which are
 *         directed along axes each displaced by 120 degrees) into currents
 *         ialpha and ibeta in a stationary qd reference frame.
 *                               ialpha = ia
 *                       ibeta = -(2*ib+ia)/sqrt(3)
 * @param  Curr_Input: stator current ia and ib in ab_t format
 * @retval Stator current ialpha and ibeta in alphabeta_t format
 */
alphabeta_t mcm_clarke(ab_t Input);

/**
 * @brief  This function transforms stator values alpha and beta, which
 *         belong to a stationary qd reference frame, to a rotor flux
 *         synchronous reference frame (properly oriented), so as Iq and Id.
 *                   Id= ialpha *sin(theta)+qibeta *cos(theta)
 *                   Iq=qialpha *cos(theta)-qibeta *sin(theta)
 * @param  Curr_Input: stator values alpha and beta in alphabeta_t format
 * @param  theta: rotating frame angular position in q1.15 format
 * @retval Stator current q and d in qd_t format
 */
qd_t mcm_park(alphabeta_t Input, int16_t theta);

/**
 * @brief  This function transforms stator voltage qVq and qVd, that belong to
 *         a rotor flux synchronous rotating frame, to a stationary reference
 *         frame, so as to obtain qValpha and qVbeta:
 *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
 *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
 * @param  Curr_Input: stator voltage Vq and Vd in qd_t format
 * @param  theta: rotating frame angular position in q1.15 format
 * @retval Stator values alpha and beta in alphabeta_t format
 */
alphabeta_t mcm_rev_park(qd_t Input, int16_t theta);

/**
 * @brief  This function returns cosine and sine functions of the angle fed in
 *         input
 * @param  angle: angle in q1.15 format
 * @retval trig_components Cos(angle) and Sin(angle) in trig_components format
 */
trig_components mcm_trig_functions(int16_t angle);

/**
 * @brief  It calculates the square root of a non-negative s32. It returns 0
 *         for negative s32.
 * @param  Input int32_t number
 * @retval int32_t Square root of Input (0 if Input<0)
 */
int32_t mcm_sqrt(int32_t input);

int16_t mcm_phase_computation(int32_t bemf_alfa_est, int32_t bemf_beta_est);

/**
 * @brief  This function codify a floting point number into the relative
 *         32bit integer.
 * @param  float Floting point number to be coded.
 * @retval uint32_t Coded 32bit integer.
 */
uint32_t mcm_float_to_int_bit(float x);

/**
 * @}
 */

/**
 * @}
 */
#endif /* MC_MATH_H*/
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
