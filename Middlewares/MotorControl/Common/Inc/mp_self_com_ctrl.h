/**
 ******************************************************************************
 * @file    mp_self_com_ctrl.h
 * @author  STMicroelectronics - System Lab - MC Team
 * @version 4.2.0
 * @date    20-Aug-2015 18:06
 * @brief   This file contains private definition of SelfComCtrl component
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MP_SELF_COM_CTRL_H
#define __MP_SELF_COM_CTRL_H

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"
#include "circle_limitation.h"
#include "mc_type.h"
#include "mp_one_touch_tuning.h"
#include "open_loop.h"
#include "pid_regulator.h"
#include "pwm_curr_fdbk.h"
#include "r_divider_bus_voltage_sensor.h"
#include "ramp_ext_mngr.h"
#include "revup_ctrl.h"
#include "speed_pos_fdbk.h"
#include "speed_torq_ctrl.h"
#include "state_machine.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "virtual_speed_sensor.h"

/** @addtogroup STM32_PMSM_MC_Library
 * @{
 */

/** @addtogroup SelfComCtrl
 * @{
 */

#define RSCURRLEVELNUM 4u
#define EMF_BUFF_VAL   5u

/** @defgroup SelfComCtrl_class_private_types SelfComCtrl class private types
 * @{
 */

/**
 * @brief  LS detection states
 */
typedef enum { LSDET_DECAY, LSDET_HOLD, LSDET_RISE } ls_det_state_t;

/**
 * @brief  KE detection states
 */
typedef enum {
    KEDET_REVUP,
    KEDET_DETECTION,
    KEDET_SET_OBS_PARAMS,
    KEDET_STABILIZEPLL,
    KEDET_RUN,
    KEDET_RESTART
} ke_det_state_t;

/**
* @brief  scc_state_t enum type definition, it lists all the possible SCC state
          machine states.
*/
typedef enum {
    SCC_IDLE,
    SCC_DUTY_DETECTING_PHASE,
    SCC_ALIGN_PHASE,
    SCC_RS_DETECTING_PHASE_RAMP,
    SCC_RS_DETECTING_PHASE,
    SCC_LS_DETECTING_PHASE,
    SCC_WAIT_RESTART,
    SCC_RESTART_SCC,
    SCC_KE_DETECTING_PHASE,
    SCC_STOP,
    SCC_CALibRATION_END
} scc_state_t;

/**
 * @brief  KE detection speed ramp status
 */
typedef enum {
    RampIdle,    /* Ramp not started yet */
    RampOngoing, /* Ramp is ongoing */
    RampSucces,  /* The motor has been accelerated up to the target speed */
    motorStill,  /* motor didn't move at all */
    LoseControl, /* motor start to follow acceleration but didn't reach the target speed */
} acc_result_t;

/**
 * @brief  SelfComCtrl parameters definition
 */
typedef const struct {
    ramp_ext_mngr_t ramp_ext_mngr_params; /*!< Ramp manager used by SCC.*/
    float rshunt;                         /*!< Value of shunt resistor.*/
    float amplification_gain;             /*!< Current sensing amplification gain.*/
    float vbus_conv_factor;               /*!< Bus voltage conversion factor.*/
    float vbus_partitioning_factor;       /*!< Bus voltage partitioning factor.
                                           (Vmcu / Bus voltage conversion factor).*/
    float rvnk;                           /*!< Power stage calibration factor.
                                           (Measured experimentally).*/
    float rs_meas_curr_level_max;         /*!< Maximum level of DC current used
                                           for RS measurement.*/
    uint16_t duty_ramp_duration;          /*!< Duration of voltage ramp executed
                                           for the duty cycle determination
                                           stage.*/
    uint16_t alignment_duration;          /*!< Duration of the alignment stage.*/
    uint16_t rs_detection_duration;       /*!< Duration of R detection stage.*/
    float ld_lq_ratio;                    /*!< Ld vs Lq ratio.*/
    float current_bw;                     /*!< Bandwidth of current regulator.*/
    bool pb_characterization;             /*!< Set to true for characterization
                                           of power board, otherwise false.*/
    int32_t nominal_speed;                /*!< Nominal speed set by the user expressed in RPM.*/
    uint16_t pwm_freq_hz;                 /*!< PWM frequency used for the test.*/
    uint8_t foc_rep_rate;                 /*!< FOC repetition rate used for the test.*/
    float mcu_power_supply;               /*!< MCU Power Supply */
    float i_threshold;
} scc_params_t, *ptr_self_com_ctrl_params_t;

/**
 * * @brief  Handle structure of the SelfComCtrl.
 */
typedef struct {
    pwmc_t* ptr_pwmc;                /*!< Current feedback and PWM object used.*/
    r_divider_t* ptr_vbs;            /*!< Bus voltage sensor used.*/
    ptr_foc_vars_t foc_vars;         /*!< Related structure of FOC vars.*/
    state_machine_t* ptr_stm;        /*!< State machine of related MC.*/
    virtual_speed_sensor_t* ptr_vss; /*!< VSS used.*/
    circle_limitation_t* ptr_clm;    /*!< Circle limitation used.*/
    pid_integer_t* ptr_pid_iq;       /*!< Iq PID used.*/
    pid_integer_t* ptr_pid_id;       /*!< Id PID used.*/
    rev_up_ctrl_t* ptr_revup_ctrl;   /*!< RUC used.*/
    sto_pll_t* ptr_sto;              /*!< State Observer used.*/
    speedn_torq_ctrl_t* ptr_stc;     /*!< Speed and torque controller used.*/
    one_touch_tuning_t* ptr_one_touch_tuning;

    scc_state_t sm_state;        /*!< SCC state machine state.*/
    ramp_ext_mngr_t* ptr_remng;  /*!< Ramp manager used.*/
    uint16_t duty_max;           /*!< Duty cycle to be applied to reach the target current.*/
    ls_det_state_t ls_det_state; /*!< Internal state during LS detection. */
    ke_det_state_t ke_det_state; /*!< Internal state during KE detection. */

    float tpwm;                                /*!< PWM period in second. */
    float foc_rate;                            /*!< FOC execution rate. */
    float pp;                                  /*!< motor poles pairs. */
    int16_t max_voltage;                       /*!< Maximum readable voltage. */
    float max_current;                         /*!< Maximum readable current. */
    float target_curr;                         /*!< Instantaneous value of target current used for R
                                                estimation.*/
    float last_target_curr;                    /*!< Last value of set nominal current used for R
                                                estimation.*/
    float rs_curr_level_step;                  /*!< Step of current used for R estimation.*/
    float bus_v;                               /*!< Stores the last Vbus measurement.*/
    float rs;                                  /*!< Stores the last R estimation.*/
    float ls;                                  /*!< Stores the last L estimation.*/
    float ke;                                  /*!< Stores the last Ke estimation.*/
    float i_max_array[RSCURRLEVELNUM];         /*!< Array to store current measurement
                                                values for R estimation.*/
    float v_max_array[RSCURRLEVELNUM];         /*!< Array to store voltage measurement
                                                values for R estimation.*/
    uint8_t rs_curr_level_tests;               /*!< Counter to store I and V values for R
                                                estimation.*/
    float i_max;                               /*!< Stores the last current measurement done, it
                                                will be used to compute the 63% for electrical
                                                time constant measurement.*/
    float i_tau;                               /*!< Stores the last computed electrical time constant.*/
    uint8_t duty_det_test;                     /*!< Number of test done in duty determination phase.*/
    uint32_t ls_time_cnt;                      /*!< Time counter for LS determination.*/
    uint32_t ls_test_cnt;                      /*!< Counter for LS tests. */
    float ls_sum;                              /*!< Sum of estimated LS for mean.*/
    float i_sum;                               /*!< Sum of measured current for mean.*/
    float v_sum;                               /*!< Sum of estimated voltage for mean.*/
    uint32_t i_cnt;                            /*!< Counter of I and V acquired.*/
    uint32_t index;                            /*!< Counter of I and V acquired.*/
    float iq_sum;                              /*!< Sum of measured Iq for mean.*/
    float vq_sum;                              /*!< Sum of measured Vq for mean.*/
    float vd_sum;                              /*!< Sum of measured Vd for mean.*/
    float fe_sum;                              /*!< Sum of electrical frequency for mean.*/
    uint32_t ke_acq_cnt;                       /*!< Counter of Iq, Vq and Vd acquired.*/
    float est_nominal_spd_rpm;                 /*!< Estimated nominal speed.*/
    float k1, k2;                              /*!< Coefficient computed for state observer tuning.*/
    int8_t stab_cnt;                           /*!< Stabilization counter.*/
    float ld_lq_ratio;                         /*!< Ld vs Lq ratio.*/
    int32_t nominal_speed;                     /*!< Nominal speed set by the user expressed in RPM.*/
    int32_t max_ol_speed;                      /*!< Maximum speed that can be sustained in the startup.*/
    uint32_t acc_rpms;                         /*!< acceleration ramp for the motor expressed in RMP/s.*/
    bool acc_ramp_lock;                        /*!< It become true if the motor follow the acceleration. */
    float em_val[EMF_BUFF_VAL];                /*!< Buffer used for linear regression (BEMF amplitude).*/
    float fw_val[EMF_BUFF_VAL];                /*!< Buffer used for linear regression (Angular velocity).*/
    uint16_t val_ctn;                          /*!< Counter for the buffer used for linear regression.*/
    bool start_computation;                    /*!< It becomes true if the buffer used for linear
                                                regression has been filled.*/
    uint16_t time_out_cnt;                     /*!< Time out counter to assert the motor still
                                                condition during acceleration ramp.*/
    uint32_t lose_control_at_rpm;              /*!< Last speed forced before loosing control during
                                                acceleration ramp.*/
    acc_result_t res;                          /*!< Result state of the last acceleration ramp.*/
    float last_valid_i;                        /*!< Last valid current measurement during SCC_DUTY_DETECTING_PHASE */
    uint16_t mf_count;                         /*!< Counter of MF to be wait after OC or loose control.*/
    uint16_t mf_timeout;                       /*!< Counter of MF to be wait after OC or loose control.*/
    float current_bw;                          /*!< Bandwidth of speed regulator.*/
    uint8_t mp_ongoing;                        /*!< It is 1 if MP is ongoing, 0 otherwise.*/
    uint32_t speed_th_to_validate_startup_rpm; /*!< Speed threshold to validate the startup.*/
    float ia_buff[256];
    bool detect_bemf_state;

    ptr_self_com_ctrl_params_t p_self_com_ctrl_Params_str; /**< SelfComCtrl parameters */

} self_com_ctrl_t;

/**
 * @brief  Initializes all the object variables, usually it has to be called
 *         once right after object creation.
 * @param  this related object of class CSCC.
 * @retval none.
 */
void scc_init(self_com_ctrl_t* phandle);
/**
 * @brief  It should be called before each motor restart.
 * @param this related object of class CSCC.
 * @retval bool It return false if function fails to start the SCC.
 */
bool scc_start(self_com_ctrl_t* phandle);

/**
 * @brief  It should be called before each motor stop.
 * @param this related object of class Cscc.
 * @retval none.
 */
void scc_stop(self_com_ctrl_t* phandle);

/**
 * @brief  It feed the required phase voltage to the inverter.
 * @param  this related object of class Cscc.
 * @retval It returns the code error 'MC_FOC_DURATION' if any, 'MC_NO_ERROR'
 *         otherwise. These error codes are defined in mc_type.h
 */
uint16_t scc_set_phase_voltage(self_com_ctrl_t* phandle);

/**
 * @brief  Medium frequency task.
 * @param  this related object of class Cscc.
 * @retval none
 */
void scc_mf(self_com_ctrl_t* phandle);

/**
 * @brief  It returns the state of Selfcommissioning procedure.
 * @param  this related object of class Cscc.
 * @retval uint8_t It returns the state of Selfcommissioning procedure.
 */
uint8_t scc_get_state(self_com_ctrl_t* phandle);

/**
 * @brief  It returns the number of states of Selfcommissioning procedure.
 * @param  this related object of class CSCC.
 * @retval uint8_t It returns the number of states of Selfcommissioning procedure.
 */
uint8_t scc_get_steps(self_com_ctrl_t* phandle);

/**
 * @brief  It returns the measured Rs.
 * @param  this related object of class CSCC.
 * @retval uint32_t It returns the measured Rs, it is a floating point number
 *         codified into a 32bit integer.
 */
uint32_t scc_get_rs(self_com_ctrl_t* phandle);

/**
 * @brief  It returns the measured Ls.
 * @param  this related object of class CSCC.
 * @retval uint32_t It returns the measured Ls, it is a floating point number
 *         codified into a 32bit integer.
 */
uint32_t scc_get_ls(self_com_ctrl_t* phandle);

/**
 * @brief  It returns the measured Ke.
 * @param  this related object of class CSCC.
 * @retval uint32_t It returns the measured Ke, it is a floating point number
 *         codified into a 32bit integer.
 */
uint32_t scc_get_ke(self_com_ctrl_t* phandle);

/**
 * @brief  It returns the measured VBus.
 * @param  this related object of class CSCC.
 * @retval uint32_t It returns the measured Vbus, it is a floating point number
 *         codified into a 32bit integer.
 */
uint32_t scc_get_vbus(self_com_ctrl_t* phandle);

/**
 * @brief  It returns the nominal speed estimated from Ke.
 * @param  this related object of class CSCC.
 * @retval uint32_t It returns the nominal speed estimated from Ke, it is a
 *         floating point number codified into a 32bit integer.
 */
uint32_t scc_get_est_nominal_speed(self_com_ctrl_t* phandle);

/**
 * @brief  Call this method before start motor to force new motor profile.
 * @param  this related object of class CSCC.
 * @retval none
 */
void scc_force_profile(self_com_ctrl_t* phandle);

/**
 * @brief  Call this method to force end of motor profile.
 * @param  this related object of class CSCC.
 * @retval none
 */
void scc_stop_profile(self_com_ctrl_t* phandle);

/**
 * @brief  Sets the number of motor poles pairs.
 *         This function shall be called before the start
 *         of the MP procedure.
 * @param  this related object of class CSCC.
 * @param  bPP Number of motor poles pairs to be set.
 * @retval none
 */
void scc_set_poles_pairs(self_com_ctrl_t* phandle, uint8_t bPP);

/**
  * @brief  Change the current used for RL determination.
            Usually is the nominal current of the motor.
  *         This function shall be called before the start
  *         of the MP procedure.
  * @param  this related object of class CSCC.
  * @param   Current used for RL determination.
  * @retval none
  */
void scc_set_nominal_current(self_com_ctrl_t* phandle, float);

/**
 * @brief  Get the nominal current used for RL determination.
 * @param  this related object of class CSCC.
 * @retval float Nominal current used for RL determination.
 */
float scc_get_nominal_current(self_com_ctrl_t* phandle);

/**
  * @brief  Set the Ld/Lq ratio.
  *         This function shall be called before the start
  *         of the MP procedure.
  * @param  this related object of class CSCC.
  * @param  ld_lq_ratio New value of Lq/Lq ratio used by MP for tuning of
            current regulators.
  * @retval none
  */
void scc_set_ld_lq_ratio(self_com_ctrl_t* phandle, float ld_lq_ratio);

/**
  * @brief  Get the Ld/Lq ratio.
  * @param  this related object of class CSCC.
  * @retval float New value of Lq/Lq ratio used by MP for tuning of
            current regulators.
  */
float scc_ge_ld_lq_ratio(self_com_ctrl_t* phandle);

/**
 * @brief  Set the nominal speed according motor datasheet.
 *         This function shall be called before the start
 *         of the MP procedure.
 * @param  this related object of class CSCC.
 * @param  nominal_speed Nominal speed expressed in RPM.
 * @retval none
 */
void scc_set_nominal_speed(self_com_ctrl_t* phandle, int32_t nominal_speed);

/**
 * @brief  Get the last nominal speed set by scc_set_nominal_speed.
 *         Note that this is not the estimated one.
 * @param  this related object of class CSCC.
 * @retval int32_t Nominal speed expressed in RPM.
 */
int32_t scc_get_nominal_speed(self_com_ctrl_t* phandle);

/**
 * @brief  Get the estimated maximum speed that can be
 *         sustatined in the startup open loop acceleration.
 *         This function must be called only after that the
 *         MP procedure is completed succesfully.
 * @param  this related object of class CSCC.
 * @retval int32_t Estimated maximum open loop speed expressed in RPM.
 */
int32_t scc_get_est_max_ol_speed(self_com_ctrl_t* phandle);

/**
 * @brief  Get the estimated maximum acceleration that can be
 *         sustatined in the startup using the estimated
 *         startup current. You can retireve the max startup
 *         current using the SCC_GetStartupCurrentX function.
 *         This function must be called only after that the
 *         MP procedure is completed succesfully.
 * @param  this related object of class CSCC.
 * @retval int32_t Estimated maximum open loop acceleration
 *         espressed in RPM/s.
 */
int32_t scc_get_est_max_acceleration(self_com_ctrl_t* phandle);

/**
 * @brief  Get the estimated maximum statup current that
 *         can be applied to the selected motor.
 *         This function must be called only after that the
 *         MP procedure is completed succesfully.
 * @param  this related object of class CSCC.
 * @retval int16_t Estimated maximum open loop current
 *         espressed in s16int.
 */
int16_t scc_get_startup_current_s16(self_com_ctrl_t* phandle);

/**
 * @brief  Get the estimated maximum statup current that
 *         can be applied to the selected motor.
 *         This function must be called only after that the
 *         MP procedure is completed succesfully.
 * @param  this related object of class CSCC.
 * @retval int16_t Estimated maximum open loop current
 *         espressed in Ampere.
 */
float scc_get_startup_current_amp(self_com_ctrl_t* phandle);

/**
 * @brief  Set the bandwidth used to tune the current regulators.
 *         This function shall be called before the start
 *         of the MP procedure.
 * @param  this related object of class CSCC.
 * @param  current_bw Bandwidth used to tune the current regulators expressed in rad/s.
 * @retval none
 */
void scc_set_current_bandwidth(self_com_ctrl_t* phandle, float current_bw);

/**
 * @brief  Get the bandwidth used to tune the current regulators.
 * @param  this related object of class CSCC.
 *         This function must be called only after that the
 *         MP procedure is completed succesfully.
 * @retval float Bandwidth used to tune the current regulators expressed in
 *         rad/s.
 */
float scc_get_current_bandwidth(self_com_ctrl_t* phandle);

/**
 * @brief  Get the PWM frequency used by the test.
 * @param  this related object of class CSCC.
 * @retval uint16_t PWM frequency used by the test expressed in Hz.
 */
uint16_t scc_get_pwm_frequency_hz(self_com_ctrl_t* phandle);

/**
 * @brief  Get the FOC repetition rate. It is the number of PWM
 *         periods elapsed before executing one FOC control cycle.
 * @param  this related object of class CSCC.
 * @retval uint8_t FOC repetition used by the test.
 */
uint8_t scc_get_foc_rep_rate(self_com_ctrl_t* phandle);

/**
 * @brief  Check overcurrent during RL detetction and restart the procedure
 *         with less current.
 * @param  this related object of class CSCC.
 * @retval none.
 */
void scc_check_oc_rl(self_com_ctrl_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif /*__MP_SELF_COM_CTRL_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
