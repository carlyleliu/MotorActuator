/**
 ******************************************************************************
 * @file    sto_CORDIC_speed_pos_fdbk.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the State Observer + CORDIC Speed & Position Feedback component of the
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
#include "sto_cordic_speed_pos_fdbk.h"

#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SpeednPosFdbk
 * @{
 */

/** @defgroup STO_CORDIC_SpeednPosFdbk State Observer CORDIC Speed & Position Feedback
 * @brief State Observer with CORDIC Speed & Position Feedback implementation
 *
 * This component uses a State Observer coupled with a CORDIC (COordinate Rotation DIgital
 * Computer) to provide an estimation of the speed and the position of the rotor of the motor.
 *
 * @todo Document the State Observer + CORDIC Speed & Position Feedback "module".
 *
 * @{
 */

/* Private defines -----------------------------------------------------------*/

#define C6_COMP_CONST1 (int32_t)1043038
#define C6_COMP_CONST2 (int32_t)10430

/* Private functions prototypes ----------------------------------------------*/
static void sto_cr_store_rotor_speed(sto_cr_t* phandle, int16_t rotor_speed, int16_t orrotor_speed);

static void stc_cr_init_speed_buffer(sto_cr_t* phandle);

/**
 * @brief  It initializes the state observer object
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval none
 */
__weak void stc_cr_init(sto_cr_t* phandle)
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

    sto_cr_clear(phandle);

    /* acceleration measurement set to zero */
    phandle->_Super.mec_accel_unit_p = 0;

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
 *         CORDIC with the purpose of computing a new speed estimation and
 *         updating the estimated electrical angle.
 * @param  phandle: handler of the current instance of the STO CORDIC component
 *         pInputs pointer to the observer inputs structure
 * @retval int16_t rotor electrical angle (s16Degrees)
 */
__weak int16_t sto_cr_calc_el_angle(sto_cr_t* phandle, observer_inputs_t* pInputs)
{
    int32_t aux, direction;
    int32_t aux_alpha, aux_beta;
    int32_t ialfa_est_next, ibeta_est_next;
    int32_t bemf_alfa_est_next, bemf_beta_est_next;
    int16_t haux, haux_alfa, haux_beta, hialfa_err, hibeta_err, rotor_speed, orrotor_speed, hRotor_acceleration,
    rotor_angle, valfa, vbeta;

    int16_t prev_rotor_angle = phandle->_Super.el_angle;
    int16_t prev_rotor_speed = phandle->_Super.el_speed_dpp;
    int16_t max_instant_accel = phandle->max_instant_el_acceleration;

    if (phandle->bemf_alfa_est_full > ((int32_t)phandle->f2 * INT16_MAX)) {
        phandle->bemf_alfa_est_full = INT16_MAX * (int32_t)(phandle->f2);
    } else if (phandle->bemf_alfa_est_full <= (-INT16_MAX * (int32_t)(phandle->f2))) {
        phandle->bemf_alfa_est_full = -INT16_MAX * (int32_t)(phandle->f2);
    } else {
    }
#ifdef FULL_MISRA_C_COMPLIANCY
    haux_alfa = (int16_t)(phandle->bemf_alfa_est_full / phandle->f2);
#else
    haux_alfa = (int16_t)(phandle->bemf_alfa_est_full >> phandle->f2_log);
#endif

    if (phandle->bemf_beta_est_full > (INT16_MAX * (int32_t)(phandle->f2))) {
        phandle->bemf_beta_est_full = INT16_MAX * (int32_t)(phandle->f2);
    } else if (phandle->bemf_beta_est_full <= (-INT16_MAX * (int32_t)(phandle->f2))) {
        phandle->bemf_beta_est_full = -INT16_MAX * (int32_t)(phandle->f2);
    } else {
    }
#ifdef FULL_MISRA_C_COMPLIANCY
    haux_beta = (int16_t)(phandle->bemf_beta_est_full / phandle->f2);
#else
    haux_beta = (int16_t)(phandle->bemf_beta_est_full >> phandle->f2_log);
#endif

    if (phandle->ialfa_est > (INT16_MAX * (int32_t)(phandle->f1))) {
        phandle->ialfa_est = INT16_MAX * (int32_t)(phandle->f1);
    } else if (phandle->ialfa_est <= (-INT16_MAX * (int32_t)(phandle->f1))) {
        phandle->ialfa_est = -INT16_MAX * (int32_t)(phandle->f1);
    } else {
    }

    if (phandle->ibeta_est > (INT16_MAX * (int32_t)(phandle->f1))) {
        phandle->ibeta_est = INT16_MAX * (int32_t)(phandle->f1);
    } else if (phandle->ibeta_est <= (-INT16_MAX * (int32_t)(phandle->f1))) {
        phandle->ibeta_est = -INT16_MAX * (int32_t)(phandle->f1);
    } else {
    }

#ifdef FULL_MISRA_C_COMPLIANCY
    hialfa_err = (int16_t)(phandle->ialfa_est / phandle->f1);
#else
    hialfa_err = (int16_t)(phandle->ialfa_est >> phandle->f1_log);
#endif

    hialfa_err = hialfa_err - pInputs->ialfa_beta.alpha;

#ifdef FULL_MISRA_C_COMPLIANCY
    hibeta_err = (int16_t)(phandle->ibeta_est / phandle->f1);
#else
    hibeta_err = (int16_t)(phandle->ibeta_est >> phandle->f1_log);
#endif

    hibeta_err = hibeta_err - pInputs->ialfa_beta.beta;

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

    aux = (int32_t)(phandle->c2) * hialfa_err;
    ialfa_est_next += aux;

    aux = (int32_t)(phandle->c5) * valfa;
    ialfa_est_next += aux;

    aux = (int32_t)(phandle->c3) * haux_alfa;
    ialfa_est_next -= aux;

    aux = (int32_t)(phandle->c4) * hialfa_err;
    bemf_alfa_est_next = phandle->bemf_alfa_est_full + aux;

#ifdef FULL_MISRA_C_COMPLIANCY
    aux = (int32_t)haux_beta / phandle->f3;
#else
    aux = (int32_t)haux_beta >> phandle->f3_pow2;
#endif

    aux = aux * phandle->c6;
    aux = prev_rotor_speed * aux;
    bemf_alfa_est_next += aux;

    /*beta axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
    haux = (int16_t)(phandle->ibeta_est / phandle->f1);
#else
    haux = (int16_t)(phandle->ibeta_est >> phandle->f1_log);
#endif

    aux = (int32_t)(phandle->c1) * haux;
    ibeta_est_next = phandle->ibeta_est - aux;

    aux = (int32_t)(phandle->c2) * hibeta_err;
    ibeta_est_next += aux;

    aux = (int32_t)(phandle->c5) * vbeta;
    ibeta_est_next += aux;

    aux = (int32_t)(phandle->c3) * haux_beta;
    ibeta_est_next -= aux;

    aux = (int32_t)(phandle->c4) * hibeta_err;
    bemf_beta_est_next = phandle->bemf_beta_est_full + aux;

#ifdef FULL_MISRA_C_COMPLIANCY
    aux = (int32_t)haux_alfa / phandle->f3;
#else
    aux = (int32_t)haux_alfa >> phandle->f3_pow2;
#endif

    aux = aux * phandle->c6;
    aux = prev_rotor_speed * aux;
    bemf_beta_est_next -= aux;

    if (phandle->orig_el_speed_dpp >= 0) {
        direction = 1;
    } else {
        direction = -1;
    }

    /*Stores observed b-emfs */
    phandle->bemf_alfa_est = haux_alfa;
    phandle->bemf_beta_est = haux_beta;

    /*Calls the CORDIC blockset*/
    aux_alpha = phandle->bemf_alfa_est_full * direction;
    aux_beta = phandle->bemf_beta_est_full * direction;

    rotor_angle = mcm_phase_computation(aux_alpha, -aux_beta);

    orrotor_speed = (int16_t)(rotor_angle - prev_rotor_angle);
    hRotor_acceleration = orrotor_speed - prev_rotor_speed;

    rotor_speed = orrotor_speed;

    if (direction == 1) {
        if (rotor_speed < 0) {
            rotor_speed = 0;
        } else {
            if (hRotor_acceleration > max_instant_accel) {
                rotor_speed = prev_rotor_speed + max_instant_accel;

                phandle->_Super.el_angle = prev_rotor_angle + rotor_speed;
            } else {
                phandle->_Super.el_angle = rotor_angle;
            }
        }
    } else {
        if (rotor_speed > 0) {
            rotor_speed = 0;
        } else {
            if (hRotor_acceleration < (-max_instant_accel)) {
                rotor_speed = prev_rotor_speed - max_instant_accel;

                phandle->_Super.el_angle = prev_rotor_angle + rotor_speed;
            } else {
                phandle->_Super.el_angle = rotor_angle;
            }
        }
    }

    if (hRotor_acceleration > max_instant_accel) {
        orrotor_speed = prev_rotor_speed + max_instant_accel;
    } else if (hRotor_acceleration < (-max_instant_accel)) {
        orrotor_speed = prev_rotor_speed - max_instant_accel;
    } else {
        /* nothing to do */
    }

    sto_cr_store_rotor_speed(phandle, rotor_speed, orrotor_speed);

    /*storing previous values of currents and bemfs*/
    phandle->ialfa_est = ialfa_est_next;
    phandle->bemf_alfa_est_full = bemf_alfa_est_next;

    phandle->ibeta_est = ibeta_est_next;
    phandle->bemf_beta_est_full = bemf_beta_est_next;

    return (phandle->_Super.el_angle);
}

/**
 * @brief Computes the rotor average mechanical speed in the unit defined by
 *        #SPEED_UNIT and writes it in ptr_mecspeed_unit
 *
 *  This method must be called - at least - with the same periodicity
 * on which the speed control is executed. It computes and returns - through
 * parameter ptr_mecspeed_unit - the rotor average mechanical speed, expressed in
 * the unit defined by #SPEED_UNIT. Average is computed considering a FIFO depth
 * equal to sto_cr_t::speed_buffer_size_unit.
 *
 * Moreover it also computes and returns the reliability state of the sensor,
 * measured with reference to parameters sto_cr_t::reliability_hysteresys,
 * sto_cr_t::variance_percentage and sto_cr_t::speed_buffer_size_unit of
 * the sto_cr_t handle.
 *
 * @param  phandle handler of the current instance of the STO CORDIC component
 * @param  ptr_mecspeed_unit pointer to int16_t, used to return the rotor average
 *         mechanical speed
 * @retval true if the sensor information is reliable, false otherwise
 */

__weak bool sto_cr_calc_avrg_mec_speed_unit(sto_cr_t* phandle, int16_t* ptr_mecspeed_unit)
{
    int32_t avr_speed_dpp = (int32_t)0;
    int32_t error, aux, avr_square_speed, avr_quadratic_error = 0;
    int32_t obs_bemf, est_bemf;
    int32_t obs_bemf_sq = 0, est_bemf_sq = 0;
    int32_t est_bemf_sqlo;
    uint8_t i, speed_buffer_size_unit = phandle->speed_buffer_size_unit;
    bool is_speed_reliable = false;
#if defined(__ICCARM__)
/* false positive */
#pragma cstat_disable = "MISRAC2012-Rule-2.2_c"
#endif /* __ICCARM__ */
    bool baux = false;
#if defined(__ICCARM__)
/* false positive */
#pragma cstat_restore = "MISRAC2012-Rule-2.2_c"
#endif /* __ICCARM__ */
    bool is_bemf_consistent = false;

    for (i = 0u; i < speed_buffer_size_unit; i++) {
        avr_speed_dpp += (int32_t)(phandle->speed_buffer[i]);
    }

    avr_speed_dpp = avr_speed_dpp / (int16_t)speed_buffer_size_unit;

#if defined(__ICCARM__)
/* value is written during init and computed
     by MC Workbench and always >= 1 */
#pragma cstat_disable = "MISRAC2012-Rule-1.3_d"
#endif /* __ICCARM__ */

    for (i = 0u; i < speed_buffer_size_unit; i++) {
        error = (int32_t)(phandle->speed_buffer[i]) - avr_speed_dpp;
        error = (error * error);
        avr_quadratic_error += error;
    }

#if defined(__ICCARM__)
/* value is written during init and computed
     by MC Workbench and always >= 1 */
#pragma cstat_restore = "MISRAC2012-Rule-1.3_d"
#endif /* __ICCARM__ */

    /*It computes the measurement variance   */
    avr_quadratic_error = avr_quadratic_error / (int16_t)speed_buffer_size_unit;

    /* The maximum variance acceptable is here calculated as a function of average
     speed                                                                    */
    avr_square_speed = avr_speed_dpp * avr_speed_dpp;
    avr_square_speed = (avr_square_speed * (int32_t)(phandle->variance_percentage)) / (int16_t)128;

    if (avr_quadratic_error < avr_square_speed) {
        is_speed_reliable = true;
    }

    /*Computation of Mechanical speed unit */
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
#if defined(__ICCARM__)
/* false positive */
#pragma cstat_disable = "MISRAC2012-Rule-14.3_b"
#endif /* __ICCARM__ */

        aux = ((aux < 0) ? (-aux) : (aux)); /* aux abs value   */

#if defined(__ICCARM__)
/* false positive */
#pragma cstat_restore = "MISRAC2012-Rule-14.3_b"
#endif /* __ICCARM__ */

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
            est_bemf_sqlo = est_bemf_sq - ((est_bemf_sq / 64) * (int32_t)(phandle->bemf_consistency_check));

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
                ;
            }
        } else {
            phandle->reliability_counter = 0u;
            baux = spd_is_mec_speed_reliable(&phandle->_Super, ptr_mecspeed_unit);
        }
    }
    return (baux);
}

/**
 * @brief  It clears state observer object by re-initializing private variables
 * @param  phandle pointer on the component instance to work on.
 */
__weak void sto_cr_clear(sto_cr_t* phandle)
{
    phandle->ialfa_est = (int32_t)0;
    phandle->ibeta_est = (int32_t)0;
    phandle->bemf_alfa_est_full = (int32_t)0;
    phandle->bemf_beta_est_full = (int32_t)0;
    phandle->_Super.el_angle = (int16_t)0;
    phandle->_Super.el_speed_dpp = (int16_t)0;
    phandle->orig_el_speed_dpp = (int16_t)0;
    phandle->consistency_counter = 0u;
    phandle->reliability_counter = 0u;
    phandle->is_algorithm_converged = false;
    phandle->is_bemf_consistent = false;
    phandle->obs_bemf_level = (int32_t)0;
    phandle->est_bemf_level = (int32_t)0;
    phandle->dpp_buffer_sum = (int32_t)0;
    phandle->dpp_orig_buffer_sum = (int32_t)0;
    phandle->force_convergency = false;
    phandle->force_convergency2 = false;

    stc_cr_init_speed_buffer(phandle);
}

/**
 * @brief  It stores in estimated speed FIFO latest calculated value of motor
 *         speed
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval none
 */
inline static void sto_cr_store_rotor_speed(sto_cr_t* phandle, int16_t rotor_speed, int16_t orrotor_speed)
{
    uint8_t buffer_index = phandle->speed_buffer_index;

    buffer_index++;
    if (buffer_index == phandle->speed_buffer_size_unit) {
        buffer_index = 0u;
    }

    phandle->speed_buffer_oldest_el = phandle->speed_buffer[buffer_index];
    phandle->orig_speed_buffer_oldest_el = phandle->orig_speed_buffer[buffer_index];

    phandle->speed_buffer[buffer_index] = rotor_speed;
    phandle->orig_speed_buffer[buffer_index] = orrotor_speed;
    phandle->speed_buffer_index = buffer_index;
}

/**
 * @brief  It clears the estimated speed buffer
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval none
 */
static void stc_cr_init_speed_buffer(sto_cr_t* phandle)
{
    uint8_t b_i;
    uint8_t speed_buffer_size_unit = phandle->speed_buffer_size_unit;

    /*init speed buffer*/
    for (b_i = 0u; b_i < speed_buffer_size_unit; b_i++) {
        phandle->speed_buffer[b_i] = (int16_t)0;
        phandle->orig_speed_buffer[b_i] = (int16_t)0;
    }

    phandle->speed_buffer_index = 0u;
    phandle->speed_buffer_oldest_el = (int16_t)0;
    phandle->orig_speed_buffer_oldest_el = (int16_t)0;

    return;
}

/**
 * @brief Returns true if the Observer has converged or false otherwise.
 *
 *  Internally performs a set of checks necessary to state whether
 * the state observer algorithm has converged or not.
 *
 * This function is to be periodically called during the motor rev-up procedure
 * at the same frequency as the *_CalcElAngle() functions.
 *
 * It returns true if the estimated angle and speed can be considered reliable,
 * false otherwise.
 *
 * @param  phandle pointer on the component instance
 * @param  forced_mec_speed_unit Mechanical speed as forced by VSS, in the unit defined by #SPEED_UNIT
 */
__weak bool sto_cr_is_observer_converged(sto_cr_t* phandle, int16_t forced_mec_speed_unit)
{
    int32_t aux;
    int32_t wtemp;
    int16_t estimatedspeed_unit;
    int16_t upper_threshold;
    int16_t lower_threshold;
    int16_t forced_mecspeed_unit;
    bool baux = false;

    if (phandle->force_convergency2 == true) {
        forced_mecspeed_unit = phandle->_Super.avr_mecspeed_unit;
    } else {
        forced_mecspeed_unit = forced_mec_speed_unit;
    }

    if (phandle->force_convergency == true) {
        baux = true;
        phandle->is_algorithm_converged = true;
        phandle->_Super.speed_error_number = 0u;
    } else {
        estimatedspeed_unit = phandle->_Super.avr_mecspeed_unit;
        wtemp = (int32_t)estimatedspeed_unit * (int32_t)forced_mecspeed_unit;

        if (wtemp > 0) {
            if (estimatedspeed_unit < 0) {
                estimatedspeed_unit = -estimatedspeed_unit;
            }

            if (forced_mecspeed_unit < 0) {
                forced_mecspeed_unit = -forced_mecspeed_unit;
            }
            aux = (int32_t)(forced_mecspeed_unit) * (int16_t)phandle->speed_validation_band_h;
            upper_threshold = (int16_t)(aux / (int32_t)16);

            aux = (int32_t)(forced_mecspeed_unit) * (int16_t)phandle->speed_validation_band_l;
            lower_threshold = (int16_t)(aux / (int32_t)16);

            /* If the variance of the estimated speed is low enough...*/
            if (phandle->is_speed_reliable == true) {
                if ((uint16_t)estimatedspeed_unit > phandle->min_startup_valid_speed) {
                    /*...and the estimated value is quite close to the expected value... */
                    if (estimatedspeed_unit >= lower_threshold) {
                        if (estimatedspeed_unit <= upper_threshold) {
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

#if defined(__ICCARM__)
/* false positive */
#pragma cstat_disable = "MISRAC2012-Rule-2.2_b"
#endif /* __ICCARM__ */
/**
 * @brief  It exports estimated Bemf alpha-beta in qd_t format
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval alphabeta_t Bemf alpha-beta
 */
__weak alphabeta_t sto_cr_get_estimated_bemf(sto_cr_t* phandle)
{
    alphabeta_t vaux;
    vaux.alpha = phandle->bemf_alfa_est;
    vaux.beta = phandle->bemf_beta_est;
    return (vaux);
}

/**
 * @brief  It exports the stator current alpha-beta as estimated by state
 *         observer
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval alphabeta_t State observer estimated stator current ialpha-beta
 */
__weak alphabeta_t sto_cr_get_estimated_current(sto_cr_t* phandle)
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
#if defined(__ICCARM__)
/* false positive */
#pragma cstat_restore = "MISRAC2012-Rule-2.2_b"
#endif /* __ICCARM__ */

/**
 * @brief  It exports current observer gains through parameters hc2 and hc4
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @param  pc2 pointer to int16_t used to return parameters hc2
 * @param  pc4 pointer to int16_t used to return parameters hc4
 * @retval none
 */
__weak void sto_cr_get_observer_gains(sto_cr_t* phandle, int16_t* pc2, int16_t* pc4)
{
    *pc2 = phandle->c2;
    *pc4 = phandle->c4;
}

/**
 * @brief  It allows setting new values for observer gains
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @param  wK1 new value for observer gain hc1
 * @param  wK2 new value for observer gain hc2
 * @retval none
 */
__weak void sto_cr_set_observer_gains(sto_cr_t* phandle, int16_t hc1, int16_t hc2)
{
    phandle->c2 = hc1;
    phandle->c4 = hc2;
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
 *         on which speed control is executed. It computes and update object
 *         variable el_speed_dpp that is estimated average electrical speed
 *         expressed in dpp used for instance in observer equations.
 *         Average is computed considering a FIFO depth equal to
 *         bspeed_buffer_size_dpp.
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval none
 */
__weak void sto_cr_calc_avrg_el_speed_dpp(sto_cr_t* phandle)
{
    int16_t index_new = (int16_t)phandle->speed_buffer_index;
    int16_t index_old;
    int16_t index_old_temp;
    int32_t sum = phandle->dpp_buffer_sum;
    int32_t sum_orig = phandle->dpp_orig_buffer_sum;
    int32_t avr_speed_dpp;
    int16_t speed_buffer_size_dpp = (int16_t)(phandle->speed_buffer_size_dpp);
    int16_t speed_buffer_size_unit = (int16_t)(phandle->speed_buffer_size_unit);
    int16_t buffer_size_diff;

    buffer_size_diff = speed_buffer_size_unit - speed_buffer_size_dpp;

    if (buffer_size_diff == 0) {
        sum = sum + phandle->speed_buffer[index_new] - phandle->speed_buffer_oldest_el;

        sum_orig = sum_orig + phandle->orig_speed_buffer[index_new] - phandle->orig_speed_buffer_oldest_el;
    } else {
        index_old_temp = index_new + buffer_size_diff;

        if (index_old_temp >= speed_buffer_size_unit) {
            index_old = index_old_temp - speed_buffer_size_unit;
        } else {
            index_old = index_old_temp;
        }

        sum = sum + phandle->speed_buffer[index_new] - phandle->speed_buffer[index_old];

        sum_orig = sum_orig + phandle->orig_speed_buffer[index_new] - phandle->orig_speed_buffer[index_old];
    }

#ifdef FULL_MISRA_C_COMPLIANCY
    if (speed_buffer_size_dpp != 0) {
        avr_speed_dpp = sum / speed_buffer_size_dpp;
        phandle->_Super.el_speed_dpp = (int16_t)avr_speed_dpp;
        avr_speed_dpp = sum_orig / speed_buffer_size_dpp;
    } else {
        avr_speed_dpp = (int32_t)0;
    }
#else
    avr_speed_dpp = (int32_t)(sum >> phandle->speed_buffer_size_dpp_log);
    phandle->_Super.el_speed_dpp = (int16_t)avr_speed_dpp;
    avr_speed_dpp = (int32_t)(sum_orig >> phandle->speed_buffer_size_dpp_log);
#endif

    phandle->orig_el_speed_dpp = (int16_t)avr_speed_dpp;

    phandle->dpp_buffer_sum = sum;

    phandle->dpp_orig_buffer_sum = sum_orig;
}

/**
 * @brief  It exports estimated Bemf squared level
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval int32_t
 */
__weak int32_t sto_cr_get_estimated_bemf_level(const sto_cr_t* phandle)
{
    return (phandle->est_bemf_level);
}

/**
 * @brief  It exports observed Bemf squared level
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval int32_t
 */
__weak int32_t sto_cr_get_observed_bemf_level(const sto_cr_t* phandle)
{
    return (phandle->obs_bemf_level);
}

/**
 * @brief  It enables/disables the bemf consistency check
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @param  bSel boolean; true enables check; false disables check
 */
__weak void sto_cr_bemf_consistency_check_switch(sto_cr_t* phandle, bool bSel)
{
    phandle->enable_dual_check = bSel;
}

/**
 * @brief  It returns the result of the Bemf consistency check
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval bool Bemf consistency state
 */
__weak bool sto_cr_is_bemf_consistent(const sto_cr_t* phandle)
{
    return (phandle->is_bemf_consistent);
}

/**
 * @brief  This method returns the reliability of the speed sensor
 * @param  phandle: handler of the current instance of the STO CORDIC component
 * @retval bool speed sensor reliability, measured with reference to parameters
 *         breliability_hysteresys, hvariance_percentage and bSpeedBufferSize
 *         true = sensor information is reliable
 *         false = sensor information is not reliable
 */
__weak bool sto_cr_is_speed_reliable(const sto_t* phandle)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    const sto_cr_t* ptr_hdl = (sto_cr_t*)phandle->_Super;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

    return (ptr_hdl->is_speed_reliable);
}

/* @brief  It forces the state-observer to declare converged
 * @param  phandle: handler of the current instance of the STO CORDIC component
 */
__weak void sto_cr_force_convergency1(sto_t* phandle)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    sto_cr_t* ptr_hdl = (sto_cr_t*)phandle->_Super;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

    ptr_hdl->force_convergency = true;
}

/* @brief  It forces the state-observer to declare converged
 * @param  phandle: handler of the current instance of the STO CORDIC component
 */
__weak void sto_cr_force_convergency2(sto_t* phandle)
{
#if defined(__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
    sto_cr_t* ptr_hdl = (sto_cr_t*)phandle->_Super;
#if defined(__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

    ptr_hdl->force_convergency2 = true;
}

/**
 * @}
 */

/**
 * @}
 */

/** @} */
/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
