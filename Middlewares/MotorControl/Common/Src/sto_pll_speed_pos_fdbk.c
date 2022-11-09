/**
 ******************************************************************************
 * @file    sto_speed_pos_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the State Observer + PLL Speed & Position Feedback component of the
 *          motor Control SDK.
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
#include "sto_pll_speed_pos_fdbk.h"

#include "mc_math.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/** @defgroup SpeednPosFdbk_STO State Observer Speed & Position Feedback
 * @brief State Observer with PLL Speed & Position Feedback implementation
 *
 * This component uses a State Observer coupled with a software PLL to provide an estimation of
 * the speed and the position of the rotor of the motor.
 *
 * @todo Document the State Observer + PLL Speed & Position Feedback "module".
 * @{
 */

/* Private defines -----------------------------------------------------------*/
#define C6_COMP_CONST1 (int32_t)1043038
#define C6_COMP_CONST2 (int32_t)10430

/* Private function prototypes -----------------------------------------------*/
static void sto_store_rotor_speed(sto_pll_t* phandle, int16_t rotor_speed);
static int16_t sto_execute_pll(sto_pll_t* phandle, int16_t bemf_alfa_est, int16_t bemf_beta_est);
static void sto_init_speed_buffer(sto_pll_t* phandle);

/**
 * @brief  It initializes the state observer component
 * @param  phandle: handler of the current instance of the STO component
 * @retval none
 */
__weak void sto_pll_init(sto_pll_t* phandle)
{
    int16_t htempk;
    int32_t aux;

    phandle->consistency_counter = phandle->startup_consist_threshold;
    phandle->enable_dual_check = true;

    aux = (int32_t)1;
    phandle->f3_pow2 = 0u;

    htempk = (int16_t)(C6_COMP_CONST1 / (phandle->f2));

    while (htempk != 0) {
        htempk /= (int16_t)2;
        aux *= (int32_t)2;
        phandle->f3_pow2++;
    }

    phandle->f3 = (int16_t)aux;
    aux = (int32_t)(phandle->f2) * phandle->f3;
    phandle->c6 = (int16_t)(aux / C6_COMP_CONST2);

    sto_pll_clear(phandle);

    pid_init(&phandle->pi_regulator);

    /* acceleration measurement set to zero */
    phandle->_Super.mec_accel_unit_p = 0;

    return;
}

/**
 * @brief  It only returns, necessary to implement fictitious IRQ_Handler
 * @param  phandle: handler of the current instance of the STO component
 * @param  uint8_t Fictitious interrupt flag
 * @retval none
 */

__weak void sto_pll_return(sto_pll_t* phandle, uint8_t flag)
{
    return;
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  This method executes Luenberger state observer equations and calls
 *         PLL with the purpose of computing a new speed estimation and
 *         updating the estimated electrical angle.
 * @param  phandle: handler of the current instance of the STO component
 * @param  ptr_input_vars_str pointer to the observer inputs structure
 * @retval int16_t rotor electrical angle (s16Degrees)
 */
__weak int16_t sto_pll_calc_el_angle(sto_pll_t* phandle, observer_inputs_t* pInputs)
{
    int32_t aux, direction;
    int32_t ialfa_est_next, ibeta_est_next;
    int32_t bemf_alfa_est_next, bemf_beta_est_next;
    int16_t haux, haux_alfa, haux_beta, ialfa_err, ibeta_err, rotor_speed, valfa, vbeta;

    if (phandle->bemf_alfa_est_full > (int32_t)(phandle->f2) * INT16_MAX) {
        phandle->bemf_alfa_est_full = INT16_MAX * (int32_t)(phandle->f2);
    } else if (phandle->bemf_alfa_est_full <= -INT16_MAX * (int32_t)(phandle->f2)) {
        phandle->bemf_alfa_est_full = -INT16_MAX * (int32_t)(phandle->f2);
    } else {
    }
#ifdef FULL_MISRA_C_COMPLIANCY
    haux_alfa = (int16_t)(phandle->bemf_alfa_est_full / phandle->f2);
#else
    haux_alfa = (int16_t)(phandle->bemf_alfa_est_full >> phandle->f2_log);
#endif

    if (phandle->bemf_beta_est_full > INT16_MAX * (int32_t)(phandle->f2)) {
        phandle->bemf_beta_est_full = INT16_MAX * (int32_t)(phandle->f2);
    } else if (phandle->bemf_beta_est_full <= -INT16_MAX * (int32_t)(phandle->f2)) {
        phandle->bemf_beta_est_full = -INT16_MAX * (int32_t)(phandle->f2);
    } else {
    }
#ifdef FULL_MISRA_C_COMPLIANCY
    haux_beta = (int16_t)(phandle->bemf_beta_est_full / phandle->f2);
#else
    haux_beta = (int16_t)(phandle->bemf_beta_est_full >> phandle->f2_log);
#endif

    if (phandle->ialfa_est > INT16_MAX * (int32_t)(phandle->f1)) {
        phandle->ialfa_est = INT16_MAX * (int32_t)(phandle->f1);
    } else if (phandle->ialfa_est <= -INT16_MAX * (int32_t)(phandle->f1)) {
        phandle->ialfa_est = -INT16_MAX * (int32_t)(phandle->f1);
    } else {
    }

    if (phandle->ibeta_est > INT16_MAX * (int32_t)(phandle->f1)) {
        phandle->ibeta_est = INT16_MAX * (int32_t)(phandle->f1);
    } else if (phandle->ibeta_est <= -INT16_MAX * (int32_t)(phandle->f1)) {
        phandle->ibeta_est = -INT16_MAX * (int32_t)(phandle->f1);
    } else {
    }

#ifdef FULL_MISRA_C_COMPLIANCY
    ialfa_err = (int16_t)(phandle->ialfa_est / phandle->f1);
#else
    ialfa_err = (int16_t)(phandle->ialfa_est >> phandle->f1_log);
#endif

    ialfa_err = ialfa_err - pInputs->ialfa_beta.alpha;

#ifdef FULL_MISRA_C_COMPLIANCY
    ibeta_err = (int16_t)(phandle->ibeta_est / phandle->f1);
#else
    ibeta_err = (int16_t)(phandle->ibeta_est >> phandle->f1_log);
#endif

    ibeta_err = ibeta_err - pInputs->ialfa_beta.beta;

    aux = (int32_t)(pInputs->Vbus) * pInputs->valfa_beta.alpha;
#ifdef FULL_MISRA_C_COMPLIANCY
    valfa = (int16_t)(aux / 65536);
#else
    valfa = (int16_t)(aux >> 16);
#endif

    aux = (int32_t)(pInputs->Vbus) * pInputs->valfa_beta.beta;
#ifdef FULL_MISRA_C_COMPLIANCY
    vbeta = (int16_t)(aux / 65536);
#else
    vbeta = (int16_t)(aux >> 16);
#endif

    /*alfa axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
    haux = (int16_t)(phandle->ialfa_est / phandle->f1);
#else
    haux = (int16_t)(phandle->ialfa_est >> phandle->f1_log);
#endif

    aux = (int32_t)(phandle->c1) * haux;
    ialfa_est_next = phandle->ialfa_est - aux;

    aux = (int32_t)(phandle->c2) * ialfa_err;
    ialfa_est_next += aux;

    aux = (int32_t)(phandle->c5) * valfa;
    ialfa_est_next += aux;

    aux = (int32_t)(phandle->c3) * haux_alfa;
    ialfa_est_next -= aux;

    aux = (int32_t)(phandle->c4) * ialfa_err;
    bemf_alfa_est_next = phandle->bemf_alfa_est_full + aux;

#ifdef FULL_MISRA_C_COMPLIANCY
    aux = (int32_t)haux_beta / phandle->f3;
#else
    aux = (int32_t)haux_beta >> phandle->f3_pow2;
#endif

    aux = aux * phandle->c6;
    aux = phandle->_Super.el_speed_dpp * aux;
    bemf_alfa_est_next += aux;

    /*beta axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
    haux = (int16_t)(phandle->ibeta_est / phandle->f1);
#else
    haux = (int16_t)(phandle->ibeta_est >> phandle->f1_log);
#endif

    aux = (int32_t)(phandle->c1) * haux;
    ibeta_est_next = phandle->ibeta_est - aux;

    aux = (int32_t)(phandle->c2) * ibeta_err;
    ibeta_est_next += aux;

    aux = (int32_t)(phandle->c5) * vbeta;
    ibeta_est_next += aux;

    aux = (int32_t)(phandle->c3) * haux_beta;
    ibeta_est_next -= aux;

    aux = (int32_t)(phandle->c4) * ibeta_err;
    bemf_beta_est_next = phandle->bemf_beta_est_full + aux;

#ifdef FULL_MISRA_C_COMPLIANCY
    aux = (int32_t)haux_alfa / phandle->f3;
#else
    aux = (int32_t)haux_alfa >> phandle->f3_pow2;
#endif

    aux = aux * phandle->c6;
    aux = phandle->_Super.el_speed_dpp * aux;
    bemf_beta_est_next -= aux;

    if (phandle->forced_avr_speed_vss >= 0) {
        direction = 1;
    } else {
        direction = -1;
    }

    /*Calls the PLL blockset*/
    phandle->bemf_alfa_est = haux_alfa;
    phandle->bemf_beta_est = haux_beta;

    haux_alfa = (int16_t)(haux_alfa * direction);
    haux_beta = (int16_t)(haux_beta * direction);

    rotor_speed = sto_execute_pll(phandle, haux_alfa, -haux_beta);
    phandle->_Super.instantaneous_el_speed_dpp = rotor_speed;

    sto_store_rotor_speed(phandle, rotor_speed);

    phandle->_Super.el_angle += rotor_speed;

    /*storing previous values of currents and bemfs*/
    phandle->ialfa_est = ialfa_est_next;
    phandle->bemf_alfa_est_full = bemf_alfa_est_next;

    phandle->ibeta_est = ibeta_est_next;
    phandle->bemf_beta_est_full = bemf_beta_est_next;

    return (phandle->_Super.el_angle);
}

/**
 * @brief  This method must be called - at least - with the same periodicity
 *         on which speed control is executed. It computes and returns - through
 *         parameter mecspeed_unit - the rotor average mechanical speed,
 *         expressed in Unit. Average is computed considering a FIFO depth
 *         equal to speed_buffer_size_unit. Moreover it also computes and returns
 *         the reliability state of the sensor.
 * @param  phandle: handler of the current instance of the STO component
 * @param  ptr_mecspeed_unit pointer to int16_t, used to return the rotor average
 *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
 * @retval bool speed sensor reliability, measured with reference to parameters
 *         breliability_hysteresys, hvariance_percentage and speed_buffer_size
 *         true = sensor information is reliable
 *         false = sensor information is not reliable
 */

__weak bool sto_pll_calc_avrg_mecspeed_unit(sto_pll_t* phandle, int16_t* ptr_mecspeed_unit)
{
    int32_t avr_speed_dpp = (int32_t)0;
    int32_t error, aux, avr_square_speed, avr_quadratic_error = 0;
    uint8_t i, speed_buffer_size_unit = phandle->speed_buffer_size_unit;
    int32_t obs_bemf, est_bemf;
    int32_t obs_bemf_sq = 0, est_bemf_sq = 0;
    int32_t est_bemf_sqlo;

    bool is_speed_reliable = false, baux = false;
    bool is_bemf_consistent = false;

    for (i = 0u; i < speed_buffer_size_unit; i++) {
        avr_speed_dpp += (int32_t)(phandle->speed_buffer[i]);
    }

    avr_speed_dpp = avr_speed_dpp / (int16_t)speed_buffer_size_unit;

    for (i = 0u; i < speed_buffer_size_unit; i++) {
        error = (int32_t)(phandle->speed_buffer[i]) - avr_speed_dpp;
        error = (error * error);
        avr_quadratic_error += error;
    }

    /*It computes the measurement variance   */
    avr_quadratic_error = avr_quadratic_error / (int16_t)speed_buffer_size_unit;

    /* The maximum variance acceptable is here calculated as a function of average
     speed                                                                    */
    avr_square_speed = avr_speed_dpp * avr_speed_dpp;
    avr_square_speed = (avr_square_speed * (int32_t)(phandle->variance_percentage)) / (int16_t)128;

    if (avr_quadratic_error < avr_square_speed) {
        is_speed_reliable = true;
    }

    /*Computation of Mechanical speed Unit*/
    aux = avr_speed_dpp * (int32_t)(phandle->_Super.measurement_frequency);
    aux = aux * (int32_t)(phandle->_Super.speed_unit);
    aux = aux / (int32_t)(phandle->_Super.dpp_conv_factor);
    aux = aux / (int16_t)(phandle->_Super.el_to_mec_ratio);

    *ptr_mecspeed_unit = (int16_t)aux;
    phandle->_Super.avr_mecspeed_unit = (int16_t)aux;

    phandle->is_speed_reliable = is_speed_reliable;

    /*Bemf Consistency Check algorithm*/
    if (phandle->enable_dual_check == true) /*do algorithm if it's enabled*/
    {
        aux = (aux < 0 ? (-aux) : (aux)); /* aux abs value   */
        if (aux < (int32_t)(phandle->max_app_positive_mec_speed_unit)) {
            /*Computation of Observed back-emf*/
            obs_bemf = (int32_t)(phandle->bemf_alfa_est);
            obs_bemf_sq = obs_bemf * obs_bemf;
            obs_bemf = (int32_t)(phandle->bemf_beta_est);
            obs_bemf_sq += obs_bemf * obs_bemf;

            /*Computation of Estimated back-emf*/
            est_bemf = (aux * 32767) / (int16_t)(phandle->_Super.max_reliable_mecspeed_unit);
            est_bemf_sq = (est_bemf * (int32_t)(phandle->bemf_consistency_gain)) / 64;
            est_bemf_sq *= est_bemf;

            /*Computation of threshold*/
            est_bemf_sqlo = est_bemf_sq - (est_bemf_sq / 64) * (int32_t)(phandle->bemf_consistency_check);

            /*Check*/
            if (obs_bemf_sq > est_bemf_sqlo) {
                is_bemf_consistent = true;
            }
        }

        phandle->is_bemf_consistent = is_bemf_consistent;
        phandle->obs_bemf_level = obs_bemf_sq;
        phandle->est_bemf_level = est_bemf_sq;
    } else {
        is_bemf_consistent = true;
    }

    /*Decision making*/
    if (phandle->is_algorithm_converged == false) {
        baux = spd_is_mec_speed_reliable(&phandle->_Super, ptr_mecspeed_unit);
    } else {
        if ((phandle->is_speed_reliable == false) || (is_bemf_consistent == false)) {
            phandle->reliability_counter++;
            if (phandle->reliability_counter >= phandle->reliability_hysteresys) {
                phandle->reliability_counter = 0u;
                phandle->_Super.speed_error_number = phandle->_Super.maximum_speed_errors_number;
                baux = false;
            } else {
                baux = spd_is_mec_speed_reliable(&phandle->_Super, ptr_mecspeed_unit);
            }
        } else {
            phandle->reliability_counter = 0u;
            baux = spd_is_mec_speed_reliable(&phandle->_Super, ptr_mecspeed_unit);
        }
    }
    return (baux);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  This method must be called - at least - with the same periodicity
 *         on which speed control is executed. It computes and update component
 *         variable el_speed_dpp that is estimated average electrical speed
 *         expressed in dpp used for instance in observer equations.
 *         Average is computed considering a FIFO depth equal to
 *         bspeed_buffer_size_dpp.
 * @param  phandle: handler of the current instance of the STO component
 * @retval none
 */
__weak void sto_pll_calc_avrg_el_speed_dpp(sto_pll_t* phandle)
{
    int16_t index_new = (int16_t)phandle->speed_buffer_index;
    int16_t index_old;
    int16_t index_old_temp;
    int32_t sum = phandle->dpp_buffer_sum;
    int32_t avr_speed_dpp;
    int16_t speed_buffer_size_dpp = (int16_t)(phandle->speed_buffer_size_dpp);
    int16_t speed_buffer_size_unit = (int16_t)(phandle->speed_buffer_size_unit);
    int16_t buffer_size_diff;

    buffer_size_diff = speed_buffer_size_unit - speed_buffer_size_dpp;

    if (buffer_size_diff == 0) {
        sum = sum + phandle->speed_buffer[index_new] - phandle->speed_buffer_oldest_el;
    } else {
        index_old_temp = index_new + buffer_size_diff;

        if (index_old_temp >= speed_buffer_size_unit) {
            index_old = index_old_temp - speed_buffer_size_unit;
        } else {
            index_old = index_old_temp;
        }

        sum = sum + phandle->speed_buffer[index_new] - phandle->speed_buffer[index_old];
    }

#ifdef FULL_MISRA_C_COMPLIANCY
    avr_speed_dpp = sum / speed_buffer_size_dpp;
#else
    avr_speed_dpp = sum >> phandle->speed_buffer_size_dpp_log;
#endif

    phandle->_Super.el_speed_dpp = (int16_t)avr_speed_dpp;
    phandle->dpp_buffer_sum = sum;
}

/**
 * @brief  It clears state observer component by re-initializing private variables
 * @param  phandle related object of class CSTO_SPD
 * @retval none
 */
__weak void sto_pll_clear(sto_pll_t* phandle)
{
    phandle->ialfa_est = (int32_t)0;
    phandle->ibeta_est = (int32_t)0;
    phandle->bemf_alfa_est_full = (int32_t)0;
    phandle->bemf_beta_est_full = (int32_t)0;
    phandle->_Super.el_angle = (int16_t)0;
    phandle->_Super.el_speed_dpp = (int16_t)0;
    phandle->consistency_counter = 0u;
    phandle->reliability_counter = 0u;
    phandle->is_algorithm_converged = false;
    phandle->is_bemf_consistent = false;
    phandle->obs_bemf_level = (int32_t)0;
    phandle->est_bemf_level = (int32_t)0;
    phandle->dpp_buffer_sum = (int32_t)0;
    phandle->force_convergency = false;
    phandle->force_convergency2 = false;

    sto_init_speed_buffer(phandle);
    pid_set_integral_term(&phandle->pi_regulator, (int32_t)0);
}

/**
 * @brief  It stores in estimated speed FIFO latest calculated value of motor
 *         speed
 * @param  phandle: handler of the current instance of the STO component
 * @retval none
 */
inline static void sto_store_rotor_speed(sto_pll_t* phandle, int16_t rotor_speed)
{
    uint8_t buffer_index = phandle->speed_buffer_index;

    buffer_index++;
    if (buffer_index == phandle->speed_buffer_size_unit) {
        buffer_index = 0u;
    }

    phandle->speed_buffer_oldest_el = phandle->speed_buffer[buffer_index];

    phandle->speed_buffer[buffer_index] = rotor_speed;
    phandle->speed_buffer_index = buffer_index;
}

/**
 * @brief  It executes PLL algorithm for rotor position extraction from B-emf
 *         alpha and beta
 * @param  phandle: handler of the current instance of the STO component
 *         bemf_alfa_est estimated Bemf alpha on the stator reference frame
 *         bemf_beta_est estimated Bemf beta on the stator reference frame
 * @retval none
 */
inline static int16_t sto_execute_pll(sto_pll_t* phandle, int16_t bemf_alfa_est, int16_t bemf_beta_est)
{
    int32_t alfa_sin_tmp, beta_cos_tmp;
    int16_t output;
    trig_components local_components;
    int16_t haux1, haux2;

    local_components = mcm_trig_functions(phandle->_Super.el_angle);

    /* Alfa & Beta BEMF multiplied by Cos & Sin*/
    alfa_sin_tmp = (int32_t)(bemf_alfa_est) * (int32_t)local_components.sin;
    beta_cos_tmp = (int32_t)(bemf_beta_est) * (int32_t)local_components.cos;

#ifdef FULL_MISRA_C_COMPLIANCY
    haux1 = (int16_t)(beta_cos_tmp / 32768);
#else
    haux1 = (int16_t)(beta_cos_tmp >> 15);
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
    haux2 = (int16_t)(alfa_sin_tmp / 32768);
#else
    haux2 = (int16_t)(alfa_sin_tmp >> 15);
#endif

    /* Speed PI regulator */
    output = pi_controller(&phandle->pi_regulator, (int32_t)(haux1)-haux2);

    return (output);
}

/**
 * @brief  It clears the estimated speed buffer
 * @param  phandle: handler of the current instance of the STO component
 * @retval none
 */
static void sto_init_speed_buffer(sto_pll_t* phandle)
{
    uint8_t b_i;
    uint8_t speed_buffer_size = phandle->speed_buffer_size_unit;

    /*init speed buffer*/
    for (b_i = 0u; b_i < speed_buffer_size; b_i++) {
        phandle->speed_buffer[b_i] = (int16_t)0;
    }
    phandle->speed_buffer_index = 0u;
    phandle->speed_buffer_oldest_el = (int16_t)0;

    return;
}

/**
 * @brief  It internally performs a set of checks necessary to state whether
 *         the state observer algorithm converged. To be periodically called
 *         during motor open-loop ramp-up (e.g. at the same frequency of
 *         SPD_CalcElAngle), it returns true if the estimated angle and speed
 *         can be considered reliable, false otherwise
 * @param  phandle: handler of the current instance of the STO component
 * @param  forced_mec_speed_unit Mechanical speed in 0.1Hz unit as forced by VSS
 * @retval bool sensor reliability state
 */
__weak bool sto_pll_is_observer_converged(sto_pll_t* phandle, int16_t forced_mec_speed_unit)
{
    int16_t estimated_speed_unit, upper_threshold, lower_threshold;
    int32_t aux;
    bool baux = false;
    int32_t temp;

    phandle->forced_avr_speed_vss = forced_mec_speed_unit;

    if (phandle->force_convergency2 == true) {
        forced_mec_speed_unit = phandle->_Super.avr_mecspeed_unit;
    }

    if (phandle->force_convergency == true) {
        baux = true;
        phandle->is_algorithm_converged = true;
        phandle->_Super.speed_error_number = 0u;
    } else {
        estimated_speed_unit = phandle->_Super.avr_mecspeed_unit;

        temp = (int32_t)estimated_speed_unit * (int32_t)forced_mec_speed_unit;

        if (temp > 0) {
            if (estimated_speed_unit < 0) {
                estimated_speed_unit = -estimated_speed_unit;
            }

            if (forced_mec_speed_unit < 0) {
                forced_mec_speed_unit = -forced_mec_speed_unit;
            }
            aux = (int32_t)(forced_mec_speed_unit) * (int16_t)phandle->speed_validation_band_h;
            upper_threshold = (int16_t)(aux / (int32_t)16);

            aux = (int32_t)(forced_mec_speed_unit) * (int16_t)phandle->speed_validation_band_l;
            lower_threshold = (int16_t)(aux / (int32_t)16);

            /* If the variance of the estimated speed is low enough...*/
            if (phandle->is_speed_reliable == true) {
                if ((uint16_t)estimated_speed_unit > phandle->min_startup_valid_speed) {
                    /*...and the estimated value is quite close to the expected value... */
                    if (estimated_speed_unit >= lower_threshold) {
                        if (estimated_speed_unit <= upper_threshold) {
                            phandle->consistency_counter++;

                            /*... for hConsistencyThreshold consecutive times... */
                            if (phandle->consistency_counter >= phandle->startup_consist_threshold) {
                                /* the algorithm converged.*/
                                baux = true;
                                phandle->is_algorithm_converged = true;
                                phandle->_Super.speed_error_number = 0u;
                            }
                        } else {
                            phandle->consistency_counter = 0u;
                        }
                    } else {
                        phandle->consistency_counter = 0u;
                    }
                } else {
                    phandle->consistency_counter = 0u;
                }
            } else {
                phandle->consistency_counter = 0u;
            }
        }
    }

    return (baux);
}

/**
 * @brief  It exports estimated Bemf alpha-beta in alphabeta_t format
 * @param  phandle: handler of the current instance of the STO component
 * @retval alphabeta_t Bemf alpha-beta
 */
__weak alphabeta_t sto_pll_get_estimated_bemf(sto_pll_t* phandle)
{
    alphabeta_t vaux;
    vaux.alpha = phandle->bemf_alfa_est;
    vaux.beta = phandle->bemf_beta_est;
    return (vaux);
}

/**
 * @brief  It exports the stator current alpha-beta as estimated by state
 *         observer
 * @param  phandle: handler of the current instance of the STO component
 * @retval alphabeta_t State observer estimated stator current ialpha-beta
 */
__weak alphabeta_t sto_pll_get_estimated_current(sto_pll_t* phandle)
{
    alphabeta_t iaux;

#ifdef FULL_MISRA_C_COMPLIANCY
    iaux.alpha = (int16_t)(phandle->ialfa_est / (phandle->f1));
#else
    iaux.alpha = (int16_t)(phandle->ialfa_est >> phandle->f1_log);
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
    iaux.beta = (int16_t)(phandle->ibeta_est / (phandle->f1));
#else
    iaux.beta = (int16_t)(phandle->ibeta_est >> phandle->f1_log);
#endif

    return (iaux);
}

/**
 * @brief  It exports current observer gains through parameters hc2 and hc4
 * @param  phandle: handler of the current instance of the STO component
 * @param  pc2 pointer to int16_t used to return parameters hc2
 * @param  pc4 pointer to int16_t used to return parameters hc4
 * @retval none
 */
__weak void sto_pll_get_observer_gains(sto_pll_t* phandle, int16_t* pc2, int16_t* pc4)
{
    *pc2 = phandle->c2;
    *pc4 = phandle->c4;
}

/**
 * @brief  It allows setting new values for observer gains
 * @param  phandle: handler of the current instance of the STO component
 * @param  wK1 new value for observer gain hc1
 * @param  wK2 new value for observer gain hc2
 * @retval none
 */
__weak void sto_pll_set_observer_gains(sto_pll_t* phandle, int16_t hc1, int16_t hc2)
{
    phandle->c2 = hc1;
    phandle->c4 = hc2;
}

/**
 * @brief  It exports current PLL gains through parameters pPgain and pIgain
 * @param  phandle: handler of the current instance of the STO component
 * @param  ptr_pgain pointer to int16_t used to return PLL proportional gain
 * @param  ptr_igain pointer to int16_t used to return PLL integral gain
 * @retval none
 */
__weak void sto_get_pll_gains(sto_pll_t* phandle, int16_t* ptr_pgain, int16_t* ptr_igain)
{
    *ptr_pgain = pid_get_kp(&phandle->pi_regulator);
    *ptr_igain = pid_get_ki(&phandle->pi_regulator);
}

/**
 * @brief  It allows setting new values for PLL gains
 * @param  phandle: handler of the current instance of the STO component
 * @param  pgain new value for PLL proportional gain
 * @param  igain new value for PLL integral gain
 * @retval none
 */
__weak void sto_set_pll_gains(sto_pll_t* phandle, int16_t pgain, int16_t igain)
{
    pid_set_kp(&phandle->pi_regulator, pgain);
    pid_set_ki(&phandle->pi_regulator, igain);
}

/**
 * @brief  It could be used to set istantaneous information on rotor mechanical
 *         angle.
 *         Note: Mechanical angle management is not implemented in this
 *         version of State observer sensor class.
 * @param  phandle: handler of the current instance of the STO component
 * @param  mec_angle istantaneous measure of rotor mechanical angle
 * @retval none
 */
__weak void sto_pll_set_mec_angle(sto_pll_t* phandle, int16_t mec_angle) {}

/**
 * @brief  It resets integral term of PLL during on-the-fly startup
 * @param  phandle: handler of the current instance of the STO component
 * @retval none
 */
__weak void sto_otf_reset_pll(sto_t* phandle)
{
    sto_pll_t* pHdl = (sto_pll_t*)phandle->_Super;
    pid_set_integral_term(&pHdl->pi_regulator, (int32_t)0);
}

#if defined(CCMRAM)
#if defined(__ICCARM__)
#pragma location = ".ccmram"
#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
 * @brief  It resets integral term of PLL
 * @param  phandle: handler of the current instance of the STO component
 * @retval none
 */
__weak void sto_reset_pll(sto_pll_t* phandle)
{
    pid_set_integral_term(&phandle->pi_regulator, (int32_t)0);
}

/**
 * @brief  It sends locking info for PLL
 * @param  phandle: handler of the current instance of the STO component
 * @param  el_speed_dpp:
 * @param  el_angle:
 * @retval none
 */
__weak void sto_set_pll(sto_pll_t* phandle, int16_t el_speed_dpp, int16_t el_angle)
{
    pid_set_integral_term(&phandle->pi_regulator, (int32_t)el_speed_dpp * (int32_t)pid_get_ki_divisor(&phandle->pi_regulator));
    phandle->_Super.el_angle = el_angle;
}

/**
 * @brief  It exports estimated Bemf squared level
 * @param  phandle: handler of the current instance of the STO component
 * @retval int32_t
 */
__weak int32_t sto_pll_get_estimated_bemf_level(sto_pll_t* phandle)
{
    return (phandle->est_bemf_level);
}

/**
 * @brief  It exports observed Bemf squared level
 * @param  phandle: handler of the current instance of the STO component
 * @retval int32_t
 */
__weak int32_t sto_pll_get_observed_bemf_level(sto_pll_t* phandle)
{
    return (phandle->obs_bemf_level);
}

/**
 * @brief  It enables/disables the bemf consistency check
 * @param  phandle: handler of the current instance of the STO component
 * @param  bSel boolean; true enables check; false disables check
 */
__weak void sto_pll_bemf_consistency_check_switch(sto_pll_t* phandle, bool bSel)
{
    phandle->enable_dual_check = bSel;
}

/**
 * @brief  It returns the result of the Bemf consistency check
 * @param  phandle: handler of the current instance of the STO component
 * @retval bool Bemf consistency state
 */
__weak bool sto_pll_is_bemf_consistent(sto_pll_t* phandle)
{
    return (phandle->is_bemf_consistent);
}

/**
 * @brief  It returns the result of the last variance check
 * @param  phandle: handler of the current instance of the STO component
 * @retval bool Variance state
 */
__weak bool sto_pll_is_variance_tight(const sto_t* phandle)
{
    sto_pll_t* pHdl = (sto_pll_t*)phandle->_Super;
    return (pHdl->is_speed_reliable);
}

/**
 * @brief  It forces the state-observer to declare convergency
 * @param  phandle: handler of the current instance of the STO component
 */
__weak void sto_pll_force_convergency1(sto_t* phandle)
{
    sto_pll_t* pHdl = (sto_pll_t*)phandle->_Super;
    pHdl->force_convergency = true;
}

/**
 * @brief  It forces the state-observer to declare convergency
 * @param  phandle: handler of the current instance of the STO component
 */
__weak void sto_pll_force_convergency2(sto_t* phandle)
{
    sto_pll_t* pHdl = (sto_pll_t*)phandle->_Super;
    pHdl->force_convergency2 = true;
}

/**
 * @brief  Set the Absolute value of minimum mechanical speed (expressed in
 *         the unit defined by #SPEED_UNIT) required to validate the start-up.
 * @param  phandle: handler of the current instance of the STO component
 * @param  min_startup_valid_speed: Absolute value of minimum mechanical speed
 */
__weak void sto_set_min_startup_valid_speed_unit(sto_pll_t* phandle, uint16_t min_startup_valid_speed)
{
    phandle->min_startup_valid_speed = min_startup_valid_speed;
}

/**
 * @}
 */

/**
 * @}
 */

/** @} */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
