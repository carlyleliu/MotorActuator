/**
 ******************************************************************************
 * @file    mptr_one_touch_tuning.h
 * @author  STMicroelectronics - System Lab - MC Team
 * @version 4.3.0
 * @date    22-Sep-2016 15:29
 * @brief   This file contains interface of OneTouchTuning class
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 * @ingroup OneTouchTuning
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Mptr_one_touch_tuning_H
#define __Mptr_one_touch_tuning_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
#include "ramp_ext_mngr.h"
#include "speed_torq_ctrl.h"
#include "sto_pll_speed_pos_fdbk.h"

/** @addtogroup STM32_PMSM_MC_Library
 * @{
 */

/** @addtogroup OneTouchTuning
 * @{
 */

/** @defgroup OneTouchTuning_class_exported_types OneTouchTuning class exported types
 * @{
 */

/**
 * @brief  Public OneTouchTuning class definition
 */

/**
 * @brief  OneTouchTuning class parameters definition
 */
typedef const struct {
    ramp_ext_mngr_t ramp_ext_mngr_params; /*!< Ramp manager used by SCC.*/
    float bw_def;                         /*!< Default bandwidth of speed regulator.*/
    float meas_win;                       /*!< Duration of measurement window for speed and
                                           current Iq, expressed in seconds.*/
    uint8_t poles_pairs;                  /*!< Number of motor poles pairs.*/
    uint16_t max_positive_torque;         /*!< Maximum positive value of motor
                                           torque. This value represents
                                           actually the maximum Iq current
                                           expressed in digit.*/
    float currt_reg_stab_time_sec;        /*!< Current regulation stabilization time in seconds.*/
    float ott_low_speed_perc;             /*!< OTT lower speed percentage.*/
    float ott_high_speed_perc;            /*!< OTT higher speed percentage.*/
    float speed_stab_time_sec;            /*!< Speed stabilization time in seconds.*/
    float time_out_sec;                   /*!< Timeout for speed stabilization.*/
    float speed_margin;                   /*!< Speed margin percentage to validate speed ctrl.*/
    int32_t nominal_speed;                /*!< Nominal speed set by the user expressed in RPM.*/
    float spd_kp;                         /*!< Initial KP factor of the speed regulator to be tuned.*/
    float spd_ki;                         /*!< Initial KI factor of the speed regulator to be tuned.*/
    float spd_ks;                         /*!< Initial antiwindup factor of the speed regulator to be tuned.*/
    float rshunt;                         /*!< Value of shunt resistor.*/
    float amplification_gain;             /*!< Current sensing amplification gain.*/
} ott_params_t, *ptr_one_touch_tuning_params_t;

/**
* @brief  ott_state_t enum type definition, it lists all the possible OTT state
          machine states.
*/
typedef enum {
    OTT_IDLE,
    OTT_NOMINAL_SPEED_DET,
    OTT_DYNAMicS_DET_RAMP_DOWN,
    OTT_DYNAMicS_DET_SET_TORQUE,
    OTT_DYNAMicS_DETECTION,
    OTT_RAMP_DOWN_H_SPEED,
    OTT_H_SPEED_TEST,
    OTT_RAMP_DOWN_L_SPEED,
    OTT_L_SPEED_TEST,
    OTT_TORQUE_STEP,
    OTT_END
} ott_state_t;

/**
 * @brief  OneTouchTuning class members definition
 */
typedef struct {
    speedn_pos_fdbk_t* ptr_speed_sensor; /*!< Related speed sensor used. */
    ptr_foc_vars_t foc_vars;             /*!< Related structure of FOC vars.*/
    pid_integer_t* ptr_pid_peed;         /*!< Related speed controller used. */
    speedn_torq_ctrl_t* ptr_stc;         /*!< Speed and torque controller used.*/

    ramp_ext_mngr_t* ptr_remng;   /*!< Ramp manager used.*/
    int16_t hf_det_iq[2];         /*!< Array used to store Iq measurements done during F estimation.*/
    float f_det_omega[2];         /*!< Array used to store omega values during F estimation.*/
    float fF;                     /*!< Stores the last F estimation.*/
    float omega_th;               /*!< Stores the last omega threshold.*/
    float tau;                    /*!< Stores the last computed mechanical time constant.*/
    float fJ;                     /*!< Stores the last J estimation.*/
    float bw;                     /*!< Bandwidth of speed regulator.*/
    ott_state_t state;            /*!< State macchine state.*/
    int32_t iq_sum;               /*!< Sum of measured Iq.*/
    int32_t speed01hz_sum;        /*!< Sum of average mechanical speed.*/
    uint16_t iq_cnt;              /*!< Counter for Iq acquisitions.*/
    int32_t cnt;                  /*!< 32bit counter.*/
    int16_t speed01hz_mean;       /*!< Mean value of mechanical speed.*/
    int16_t speed01hz_delta;      /*!< Delta speed between mechanical speed.*/
    uint16_t cur_reg_stab_cnt;    /*!< Stabilization counter.*/
    uint16_t j_det_cnt;           /*!< Counter to measure the mechanical time constant.*/
    float est_nominal_spd_rpm;    /*!< Estimated nominal speed.*/
    int16_t iq_nominal;           /*!< Current measured at nominal speed steady state.*/
    int16_t iq_acc;               /*!< Current used to accelerate the motor.*/
    int16_t target_lrpm;          /*!< Lower speed used for OTT.*/
    int16_t target_hrpm;          /*!< Higher speed used for OTT.*/
    uint16_t meas_win_ticks;      /*!< Number of ticks of the measurement window.*/
    uint16_t cur_reg_stab_tks;    /*!< Number of ticks for current regulation stabilization time.*/
    uint16_t speed_stab_tks;      /*!< Number of ticks for speed stabilization time.*/
    bool pi_tuned;                /*!< True is PI is tuned, false otherwise.*/
    float kp;                     /*!< Computed Kp.*/
    float ki;                     /*!< Computed Ki.*/
    int8_t stab_cnt;              /*!< Stabilization counter.*/
    float speed;                  /*!< Internal target reference.*/
    uint16_t time_out_tks;        /*!< Number of tick for timeout.*/
    uint8_t poles_pairs;          /*!< motor poles pairs.*/
    uint16_t max_positive_torque; /*!< Maximum positive value of motor
                                  torque. This value represents
                                  actually the maximum Iq current
                                  expressed in digit.*/
    float rpm_th;                 /*!< Speed threshold for mecchanical constant
                                time estimation.*/
    int32_t nominal_speed;        /*!< Nominal speed set by the user expressed in RPM.*/
    float spd_kp;                 /*!< KP factor of the speed regulator to be tuned.*/
    float spd_ki;                 /*!< KI factor of the speed regulator to be tuned.*/
    float spd_ks;                 /*!< Antiwindup factor of the speed regulator to be tuned.*/
    float spd_int_term;           /*!< Integral term of the speed regulator to be tuned.*/
    float spd_anti_wind_term;     /*!< Antiwindup term of the speed regulator to be tuned.*/
    float ke;                     /*!< Stores the last Ke estimation.*/

    ptr_one_touch_tuning_params_t ptr_one_touch_tuning_Params_str; /**< OTT parameters */

} one_touch_tuning_t;

/**
 * @}
 */

/**
 * @brief  Initializes all the object variables, usually it has to be called
 *         once right after object creation.
 * @param  this related object of class COTT.
 * @param  ptr_one_touch_tuning_Init pointer to the OTT init structure.
 * @retval none.
 */
void ott_init(one_touch_tuning_t* phandle);

/**
 * @brief  It should be called before each motor restart. It initialize
 *         internal state of OTT.
 * @param  this related object of class COTT.
 * @retval none.
 */
void ott_clear(one_touch_tuning_t* phandle);

/**
 * @brief  It should be called at MF and execute the OTT algorithm.
 * @param  this related object of class COTT.
 * @retval none.
 */
void ott_mf(one_touch_tuning_t* phandle);

/**
 * @brief  It should be called in START_RUN state. It begins the OTT procedure.
 * @param  this related object of class COTT.
 * @retval none.
 */
void ott_sr(one_touch_tuning_t* phandle);

/**
 * @brief  Call this method before start motor to force new OTT procedure.
 * @param  this related object of class COTT.
 * @retval none.
 */
void ott_force_tuning(one_touch_tuning_t* phandle);

/**
 * @brief  It returns the nominal speed estimated by OTT.
 * @param  this related object of class COTT.
 * @retval uint32_t It returns the nominal speed estimated by OTT, it is a
 *         floating point number codified into a 32bit integer.
 */
uint32_t ott_get_nominal_speed(one_touch_tuning_t* phandle);

/**
 * @brief  It returns the number of states of OTT.
 * @param  this related object of class COTT.
 * @retval uint8_t It returns the number of states of Selfcommissioning procedure.
 */
uint8_t ott_get_steps(one_touch_tuning_t* phandle);

/**
 * @brief  It returns the state of OTT.
 * @param  this related object of class COTT.
 * @retval uint8_t It returns the state of OTT.
 */
uint8_t ott_get_state(one_touch_tuning_t* phandle);

/**
 * @brief  It returns true if OTT procedure has been completed, false otherwise.
 * @param  this related object of class COTT.
 * @retval bool It returns true if OTT procedure has been completed, false otherwise.
 */
bool ott_is_speed_pi_tuned(one_touch_tuning_t* phandle);

/**
 * @brief  It returns the nominal speed estimated by OTT in RPM.
 * @param  this related object of class COTT.
 * @retval float It returns the nominal speed in RPM estimated by OTT.
 */
float ott_f_get_nominal_speed_rpm(one_touch_tuning_t* phandle);

/**
 * @brief  Sets the number of motor poles pairs.
 * @param  this related object of class COTT.
 * @param  bPP Number of motor poles pairs to be set.
 * @retval none
 */
void ott_set_poles_pairs(one_touch_tuning_t* phandle, uint8_t bPP);

/**
  * @brief  Change the nominal current.
  * @param  this related object of class COTT.
  * @param  nominal_current This value represents actually the maximum Iq current
            expressed in digit.
  * @retval none
  */
void ott_set_nominal_current(one_touch_tuning_t* phandle, uint16_t nominal_current);

/**
 * @brief  Change the speed regulator bandwidth.
 * @param  this related object of class COTT.
 * @param  bw Current regulator bandwidth espressed in rad/s.
 * @retval none
 */
void ott_set_speed_regulator_bandwidth(one_touch_tuning_t* phandle, float bw);

/**
 * @brief  Get the speed regulator bandwidth.
 * @param  this related object of class COTT.
 * @retval float Current regulator bandwidth espressed in rad/s.
 */
float ott_get_speed_regulator_bandwidth(one_touch_tuning_t* phandle);

/**
 * @brief  Get the measured inertia of the motor.
 * @param  this related object of class COTT.
 * @retval float Measured inertia of the motor expressed in Kgm^2.
 */
float ott_get_j(one_touch_tuning_t* phandle);

/**
 * @brief  Get the measured friction of the motor.
 * @param  this related object of class COTT.
 * @retval float Measured friction of the motor expressed in Nms.
 */
float ott_get_f(one_touch_tuning_t* phandle);

/**
 * @brief  Set the nominal speed according motor datasheet.
 *         This function shall be called before the start
 *         of the MP procedure.
 * @param  this related object of class COTT.
 * @param  nominal_speed Nominal speed expressed in RPM.
 * @retval none
 */
void ott_set_nominal_speed(one_touch_tuning_t* phandle, int32_t nominal_speed);

/**
 * @brief  Store the Ke measured by the SCC for the OTT purpouses.
 * @param  this related object of class COTT.
 * @param  ke Last measured Ke.
 * @retval none
 */
void ott_set_ke(one_touch_tuning_t* phandle, float ke);

/**
 * @brief  It should be called before each motor stop.
 * @param  this related object of class COTT.
 * @retval none.
 */
void ott_stop(one_touch_tuning_t* phandle);

/**
 * @brief  Return true if the motor has been already profiled.
 * @param  this related object of class COTT.
 * @retval bool true if the if the motor has been already profiled,
 *         false otherwise.
 */
bool ott_is_motor_already_profiled(one_touch_tuning_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif /* __ONETOUCHTUNINGCLASS_H */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
