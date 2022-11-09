/**
 ******************************************************************************
 * @file    mc_math.c
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
 */
/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"

#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup MC_Math motor Control Math functions
 * @brief motor Control Mathematic functions of the motor Control SDK
 *
 * @todo Document the motor Control Math "module".
 *
 * @{
 */

/* Private macro -------------------------------------------------------------*/

#define divSQRT_3 (int32_t)0x49E6 /* 1/sqrt(3) in q1.15 format=0.5773315*/

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  This function transforms stator values a and b (which are
 *         directed along axes each displaced by 120 degrees) into values
 *         alpha and beta in a stationary qd reference frame.
 *                               alpha = a
 *                       beta = -(2*b+a)/sqrt(3)
 * @param  Input: stator values a and b in ab_t format
 * @retval Stator values alpha and beta in alphabeta_t format
 */
__weak alphabeta_t mcm_clarke(ab_t Input)
{
    alphabeta_t out_put;

    int32_t a_div_sqrt3_tmp, b_div_sqrt3_tmp;
    int32_t beta_tmp;
    int16_t hbeta_tmp;

    /* qialpha = qias*/
    out_put.alpha = Input.a;

    a_div_sqrt3_tmp = divSQRT_3 * (int32_t)Input.a;

    b_div_sqrt3_tmp = divSQRT_3 * (int32_t)Input.b;

    /*qibeta = -(2*qibs+qias)/sqrt(3)*/
#ifdef FULL_MISRA_C_COMPLIANCY
    beta_tmp = (-(a_div_sqrt3_tmp) - (b_div_sqrt3_tmp) - (b_div_sqrt3_tmp)) / 32768;
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */

    beta_tmp = (-(a_div_sqrt3_tmp) - (b_div_sqrt3_tmp) - (b_div_sqrt3_tmp)) >> 15;
#endif

    /* Check saturation of ibeta */
    if (beta_tmp > INT16_MAX) {
        hbeta_tmp = INT16_MAX;
    } else if (beta_tmp < (-32768)) {
        hbeta_tmp = (-32768);
    } else {
        hbeta_tmp = (int16_t)(beta_tmp);
    }

    out_put.beta = hbeta_tmp;

    if (out_put.beta == (int16_t)(-32768)) {
        out_put.beta = -32767;
    }

    return (out_put);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  This function transforms stator values alpha and beta, which
 *         belong to a stationary qd reference frame, to a rotor flux
 *         synchronous reference frame (properly oriented), so as q and d.
 *                   d= alpha *sin(theta)+ beta *cos(theta)
 *                   q= alpha *cos(theta)- beta *sin(theta)
 * @param  Input: stator values alpha and beta in alphabeta_t format
 * @param  theta: rotating frame angular position in q1.15 format
 * @retval Stator values q and d in qd_t format
 */
__weak qd_t mcm_park(alphabeta_t Input, int16_t theta)
{
    qd_t out_put;
    int32_t d_tmp_1, d_tmp_2, q_tmp_1, q_tmp_2;
    trig_components local_vector_components;
    int32_t wqd_tmp;
    int16_t hqd_tmp;

    local_vector_components = mcm_trig_functions(theta);

    /*No overflow guaranteed*/
    q_tmp_1 = Input.alpha * (int32_t)local_vector_components.cos;

    /*No overflow guaranteed*/
    q_tmp_2 = Input.beta * (int32_t)local_vector_components.sin;

    /*Iq component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
    wqd_tmp = (q_tmp_1 - q_tmp_2) / 32768;
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
    wqd_tmp = (q_tmp_1 - q_tmp_2) >> 15;
#endif

    /* Check saturation of Iq */
    if (wqd_tmp > INT16_MAX) {
        hqd_tmp = INT16_MAX;
    } else if (wqd_tmp < (-32768)) {
        hqd_tmp = (-32768);
    } else {
        hqd_tmp = (int16_t)(wqd_tmp);
    }

    out_put.q = hqd_tmp;

    if (out_put.q == (int16_t)(-32768)) {
        out_put.q = -32767;
    }

    /*No overflow guaranteed*/
    d_tmp_1 = Input.alpha * (int32_t)local_vector_components.sin;

    /*No overflow guaranteed*/
    d_tmp_2 = Input.beta * (int32_t)local_vector_components.cos;

    /*Id component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
    wqd_tmp = (d_tmp_1 + d_tmp_2) / 32768;
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
    wqd_tmp = (d_tmp_1 + d_tmp_2) >> 15;
#endif

    /* Check saturation of Id */
    if (wqd_tmp > INT16_MAX) {
        hqd_tmp = INT16_MAX;
    } else if (wqd_tmp < (-32768)) {
        hqd_tmp = (-32768);
    } else {
        hqd_tmp = (int16_t)(wqd_tmp);
    }

    out_put.d = hqd_tmp;

    if (out_put.d == (int16_t)(-32768)) {
        out_put.d = -32767;
    }

    return (out_put);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  This function transforms stator voltage qVq and qVd, that belong to
 *         a rotor flux synchronous rotating frame, to a stationary reference
 *         frame, so as to obtain qValpha and qVbeta:
 *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
 *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
 * @param  Input: stator voltage Vq and Vd in qd_t format
 * @param  theta: rotating frame angular position in q1.15 format
 * @retval Stator voltage Valpha and Vbeta in qd_t format
 */
__weak alphabeta_t mcm_rev_park(qd_t Input, int16_t theta)
{
    int32_t alpha_tmp1, alpha_tmp2, beta_tmp1, beta_tmp2;
    trig_components local_vector_components;
    alphabeta_t out_put;

    local_vector_components = mcm_trig_functions(theta);

    /*No overflow guaranteed*/
    alpha_tmp1 = Input.q * (int32_t)local_vector_components.cos;
    alpha_tmp2 = Input.d * (int32_t)local_vector_components.sin;

#ifdef FULL_MISRA_C_COMPLIANCY
    out_put.alpha = (int16_t)(((alpha_tmp1) + (alpha_tmp2)) / 32768);
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
    out_put.alpha = (int16_t)(((alpha_tmp1) + (alpha_tmp2)) >> 15);
#endif

    beta_tmp1 = Input.q * (int32_t)local_vector_components.sin;
    beta_tmp2 = Input.d * (int32_t)local_vector_components.cos;

#ifdef FULL_MISRA_C_COMPLIANCY
    out_put.beta = (int16_t)((beta_tmp2 - beta_tmp1) / 32768);
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
    out_put.beta = (int16_t)((beta_tmp2 - beta_tmp1) >> 15);
#endif

    return (out_put);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  This function returns cosine and sine functions of the angle fed in
 *         input
 * @param  angle: angle in q1.15 format
 * @retval Sin(angle) and Cos(angle) in trig_components format
 */

__weak trig_components mcm_trig_functions(int16_t angle)
{
    union u32toi16x2 {
        uint32_t CORDICRdata;
        trig_components Components;
    } CosSin;

    /* Configure CORDIC */
    WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
    LL_CORDIC_WriteData(CORDIC, 0x7FFF0000 + (uint32_t)angle);
    /* Read angle */
    CosSin.CORDICRdata = LL_CORDIC_ReadData(CORDIC);
    return (CosSin.Components);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It calculates the square root of a non-negative int32_t. It returns 0
 *         for negative int32_t.
 * @param  Input int32_t number
 * @retval int32_t Square root of Input (0 if Input<0)
 */
__weak int32_t mcm_sqrt(int32_t input)
{
    int32_t temp_root_new;

    if (input > 0) {
        uint8_t biter = 0u;
        int32_t wtemproot;

        if (input <= (int32_t)2097152) {
            wtemproot = (int32_t)128;
        } else {
            wtemproot = (int32_t)8192;
        }

        do {
            temp_root_new = (wtemproot + input / wtemproot) / (int32_t)2;
            if (temp_root_new == wtemproot) {
                biter = 6u;
            } else {
                biter++;
                wtemproot = temp_root_new;
            }
        } while (biter < 6u);

    } else {
        temp_root_new = (int32_t)0;
    }

    return (temp_root_new);
}

/**
 * @brief  It executes CORDIC algorithm for rotor position extraction from B-emf
 *         alpha and beta
 * @param  bemf_alfa_est estimated Bemf alpha on the stator reference frame
 *         bemf_beta_est estimated Bemf beta on the stator reference frame
 * @retval int16_t rotor electrical angle (s16degrees)
 */
inline int16_t mcm_phase_computation(int32_t bemf_alfa_est, int32_t bemf_beta_est)
{
    /* Configure and call to CORDIC */
    WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_PHASE);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)bemf_alfa_est);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)bemf_beta_est);

    /* Read computed angle */
    return (int16_t)(LL_CORDIC_ReadData(CORDIC) >> 16);
}

/**
 * @brief  This function codify a floating point number into the relative
 *         32bit integer.
 * @param  float Floating point number to be coded.
 * @retval uint32_t Coded 32bit integer.
 */
__weak uint32_t mcm_float_to_int_bit(float x)
{
    uint32_t* ptr_int;
    ptr_int = (uint32_t*)(&x);
    return *ptr_int;
}

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
