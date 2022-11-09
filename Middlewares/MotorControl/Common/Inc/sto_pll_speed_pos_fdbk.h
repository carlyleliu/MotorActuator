/**
 ******************************************************************************
 * @file    sto_pll_speed_pos_fdbk.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          State Observer + PLL Speed & Position Feedback component of the motor
 *          Control SDK.
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
#ifndef __STO_PLL_SPEEDNPOSFDBK_H
#define __STO_PLL_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pid_regulator.h"
#include "speed_pos_fdbk.h"
#include "sto_speed_pos_fdbk.h"

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

/**
 * @brief This structure is used to handle an instance of the STO_SpeednPosFdbk component
 *
 */

typedef struct {
    speedn_pos_fdbk_t _Super;

    int16_t c1;                  /*!< Variable containing state observer constant
                                  C1 to speed-up computations */
    int16_t c2;                  /*!< Variable containing state observer constant
                                  C2, it can be computed as F1 * K1/ State
                                  observer execution rate [Hz] being K1 one
                                  of the two observer gains   */
    int16_t c3;                  /*!< Variable containing state observer constant
                                  C3 */
    int16_t c4;                  /*!< State Observer constant C4, it can
                                  be computed as K2 * max measurable
                                  current (A) / (Max application speed
                                  [#SPEED_UNIT] * motor B-emf constant
                                  [Vllrms/krpm] * sqrt(2) * F2 * State
                                  observer execution rate [Hz]) being
                                  K2 one of the two observer gains  */
    int16_t c5;                  /*!< Variable containing state observer constant
                                  C5 */
    int16_t c6;                  /*!< State observer constant C6, computed with a
                                  specific procedure starting from the other
                                  constants */
    int16_t f1;                  /*!< Variable containing state observer scaling
                                  factor F1 */
    int16_t f2;                  /*!< Variable containing state observer scaling factor F2*/
    int16_t f3;                  /*!< State observer scaling factor F3 */
    uint16_t f3_pow2;            /*!< State observer scaling factor F3 expressed as power of 2.
                                  E.g. if gain divisor is 512 the value
                                  must be 9 because 2^9 = 512 */
    pid_integer_t pi_regulator;  /*!< PI regulator component handle, used for
                                  PLL implementation */
    int32_t ialfa_est;           /*!< Estimated ialfa current in int32 format */
    int32_t ibeta_est;           /*!< Estimated ibeta current in int32 format */
    int32_t bemf_alfa_est_full;  /*!< Estimated B-emf alfa in int32_t format */
    int32_t bemf_beta_est_full;  /*!< Estimated B-emf beta in int32_t format */
    int16_t bemf_alfa_est;       /*!< Estimated B-emf alfa in int16_t format */
    int16_t bemf_beta_est;       /*!< Estimated B-emf beta in int16_t format */
    int16_t speed_buffer[64];    /*!< Estimated DPP speed FIFO, it contains latest
                                  speed_buffer_size_dpp speed measurements*/
    uint8_t speed_buffer_index;  /*!< Position of latest speed estimation in
                                  estimated speed FIFO */
    bool is_speed_reliable;      /*!< Latest private speed reliability information,
                                  updated by SPD_CalcAvrgMecspeed_unit, it is
                                  true if the speed measurement variance is
                                  lower then threshold corresponding to
                                  hvariance_percentage */
    uint8_t consistency_counter; /*!< Counter of passed tests for start-up
                                  validation */
    uint8_t reliability_counter; /*!< Counter for reliability check */
    bool is_algorithm_converged; /*!< Boolean variable containing observer
                                  convergence information */
    bool is_bemf_consistent;     /*!< Sensor reliability information, updated by
                                  SPD_CalcAvrgMecspeed_unit, it is true if the
                                  observed back-emfs are consistent with
                                  expectation*/

    int32_t obs_bemf_level;         /*!< Observed back-emf Level*/
    int32_t est_bemf_level;         /*!< Estimated back-emf Level*/
    bool enable_dual_check;         /*!< Consistency check enabler*/
    int32_t dpp_buffer_sum;         /*!< summation of speed buffer elements [dpp]*/
    int16_t speed_buffer_oldest_el; /*!< Oldest element of the speed buffer*/

    uint8_t speed_buffer_size_unit;           /*!< Depth of FIFO used to average
                                               estimated speed exported by
                                               spd_get_avrg_mecspeed_unit. It
                                               must be an integer number between 1
                                               and 64 */
    uint8_t speed_buffer_size_dpp;            /*!< Depth of FIFO used for both averaging
                                               estimated speed exported by
                                               spd_get_el_speed_dpp and state
                                               observer equations. It must be an
                                               integer number between 1 and
                                               bspeed_buffer_size_unit */
    uint16_t variance_percentage;             /*!< Parameter expressing the maximum
                                               allowed variance of speed estimation
                                               */
    uint8_t speed_validation_band_h;          /*!< It expresses how much estimated speed
                                               can exceed forced stator electrical
                                               frequency during start-up without
                                               being considered wrong. The
                                               measurement unit is 1/16 of forced
                                               speed */
    uint8_t speed_validation_band_l;          /*!< It expresses how much estimated speed
                                               can be below forced stator electrical
                                               frequency during start-up without
                                               being considered wrong. The
                                               measurement unit is 1/16 of forced
                                               speed */
    uint16_t min_startup_valid_speed;         /*!< Absolute value of minimum mechanical
                                               speed  required to validate the start-up.
                                               Expressed in the unit defined by #SPEED_UNIT. */
    uint8_t startup_consist_threshold;        /*!< Number of consecutive tests on speed
                                               consistency to be passed before
                                               validating the start-up */
    uint8_t reliability_hysteresys;           /*!< Number of reliability failed
                                               consecutive tests before a speed
                                               check fault is returned to _Super.speed_error_number
                                               */
    uint8_t bemf_consistency_check;           /*!< Degree of consistency of the observed
                                               back-emfs, it must be an integer
                                               number ranging from 1 (very high
                                               consistency) down to 64 (very poor
                                               consistency) */
    uint8_t bemf_consistency_gain;            /*!< Gain to be applied when checking
                                               back-emfs consistency; default value
                                               is 64 (neutral), max value 105
                                               (x1.64 amplification), min value 1
                                               (/64 attenuation) */
    uint16_t max_app_positive_mec_speed_unit; /*!< Maximum positive value of rotor speed. Expressed in
                                               the unit defined by #SPEED_UNIT. It can be
                                               x1.1 greater than max application speed*/
    uint16_t f1_log;                          /*!< F1 gain divisor expressed as power of 2.
                                               E.g. if gain divisor is 512 the value
                                               must be 9 because 2^9 = 512 */
    uint16_t f2_log;                          /*!< F2 gain divisor expressed as power of 2.
                                               E.g. if gain divisor is 512 the value
                                               must be 9 because 2^9 = 512 */
    uint16_t speed_buffer_size_dpp_log;       /*!< bspeed_buffer_size_dpp expressed as power of 2.
                                               E.g. if gain divisor is 512 the value
                                               must be 9 because 2^9 = 512 */
    bool force_convergency;                   /*!< Variable to force observer convergence.*/
    bool force_convergency2;                  /*!< Variable to force observer convergence.*/

    int16_t forced_avr_speed_vss;

} sto_pll_t;

/* Exported functions ------------------------------------------------------- */

/* It initializes the state observer object */
void sto_pll_init(sto_pll_t* phandle);

/* It only returns, necessary to implement fictitious IRQ_Handler */
void sto_pll_return(sto_pll_t* phandle, uint8_t flag);

/* It clears state observer object by re-initializing private variables*/
void sto_pll_clear(sto_pll_t* phandle);

/* It executes Luenberger state observer and calls PLL to compute a new speed
 *  estimation and update the estimated electrical angle.
 */
int16_t sto_pll_calc_el_angle(sto_pll_t* phandle, observer_inputs_t* ptr_input_vars_str);

/* Computes the rotor average mechanical speed in the unit defined by #SPEED_UNIT and returns it in ptr_mecspeed_unit. */
bool sto_pll_calc_avrg_mecspeed_unit(sto_pll_t* phandle, int16_t* ptr_mecspeed_unit);

/* It resets integral term of PLL during on-the-fly startup */
void sto_otf_reset_pll(sto_t* phandle);

/* It resets integral term of PLL*/
void sto_reset_pll(sto_pll_t* phandle);

/* It checks whether the state observer algorithm converged.*/
bool sto_pll_is_observer_converged(sto_pll_t* phandle, int16_t forced_mec_speed_unit);

/* It computes the estimated average electrical speed ElSpeedDpp expressed in dpp */
void sto_pll_calc_avrg_el_speed_dpp(sto_pll_t* phandle);

/* It exports estimated Bemf alpha-beta in alphabeta_t format */
alphabeta_t sto_pll_get_estimated_bemf(sto_pll_t* phandle);

/* It exports the stator current alpha-beta as estimated by state  observer */
alphabeta_t sto_pll_get_estimated_current(sto_pll_t* phandle);

/* It set new values for observer gains*/
void sto_pll_set_observer_gains(sto_pll_t* phandle, int16_t c1, int16_t c2);

/* It exports current observer gains through parameters pC2 and pC4 */
void sto_pll_get_observer_gains(sto_pll_t* phandle, int16_t* pC2, int16_t* pC4);

/* It exports current PLL gains through parameters pPgain and pIgain */
void sto_get_pll_gains(sto_pll_t* phandle, int16_t* ptr_pgain, int16_t* ptr_igain);

/* It set new values for PLL gains */
void sto_set_pll_gains(sto_pll_t* phandle, int16_t pgain, int16_t igain);

void sto_pll_set_mec_angle(sto_pll_t* phandle, int16_t mec_angle);

void sto_set_pll(sto_pll_t* phandle, int16_t el_speed_dpp, int16_t el_angle);

/* It exports estimated Bemf squared level*/
int32_t sto_pll_get_estimated_bemf_level(sto_pll_t* phandle);

/* It exports observed Bemf squared level*/
int32_t sto_pll_get_observed_bemf_level(sto_pll_t* phandle);

/* It enables / disables the Bemf consistency check */
void sto_pll_bemf_consistency_check_switch(sto_pll_t* phandle, bool sel);

/* It exports Bemf consistency */
bool sto_pll_is_bemf_consistent(sto_pll_t* phandle);

/* It returns the result of the last variance check*/
bool sto_pll_is_variance_tight(const sto_t* phandle);

/* It set internal force_convergency1 to true*/
void sto_pll_force_convergency1(sto_t* phandle);

/* It set internal force_convergency2 to true*/
void sto_pll_force_convergency2(sto_t* phandle);

/* It set absolute value of minimum mechanical speed required to validate the start-up. */
void sto_set_min_startup_valid_speed_unit(sto_pll_t* phandle, uint16_t min_startup_valid_speed);

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

#endif /*__STO_PLL_SPEEDNPOSFDBK_H*/

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
