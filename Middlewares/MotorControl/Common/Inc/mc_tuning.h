/**
 ******************************************************************************
 * @file    mc_tuning.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          motor control tuning component of the motor Control SDK.
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
 * @ingroup MCTuning
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_TUNING_H
#define __MC_TUNING_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "digital_output.h"
#include "mc_type.h"
#include "pid_regulator.h"
#include "pid_regulator_float.h"
#include "revup_ctrl.h"
#include "speed_torq_ctrl.h"
#include "sto_cordic_speed_pos_fdbk.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "virtual_speed_sensor.h"
#ifdef HFINJECTION
#include "hifreqinj_fpu_ctrl.h"
#endif /* HFINJECTION */
#include "bus_voltage_sensor.h"
#include "feed_forward_ctrl.h"
#include "flux_weakening_ctrl.h"
#include "motor_power_measurement.h"
#include "mp_self_com_ctrl.h"
#include "ntc_temperature_sensor.h"
#include "open_loop.h"
#include "pqd_motor_power_measurement.h"
#include "state_machine.h"
#include "trajectory_ctrl.h"

/**
 * @addtogroup MCSDK
 * @{
 */

/**
 * @defgroup MCTuning motor Control Fine Tuning interface
 *
 * @brief This interface provides access to the internals of the motor Control Subsystem
 *
 * @todo Complete documentation
 * @{
 */

/**
 * @internal
 * @brief  Public DigitalOutput class definition
 */
#ifndef __DIGITALOUTPUTCLASS_H
typedef struct CDOUT_t* CDOUT;
#endif

/**
 * @brief  MC tuning internal objects initialization structure type;
 */
typedef struct {
    pid_float_t* ptr_pid_speed;
    pid_float_t* ptr_pid_iq;
    pid_float_t* ptr_pid_id;
    pid_float_t* ptr_pid_flux_weakening;
    pwmc_t* ptr_pwmn_curr_fdbk;
    rev_up_ctrl_t* ptr_revup_ctrl;
    speedn_pos_fdbk_t* ptr_speed_sensor_main;
    speedn_pos_fdbk_t* ptr_speed_sensor_aux;
    virtual_speed_sensor_t* ptr_speed_sensor_virtual;
    speedn_torq_ctrl_t* ptr_speedn_torque_ctrl;
    state_machine_t* ptr_state_machine;
    ntc_t* ptr_temperature_sensor;
    bus_voltage_sensor_t* ptr_bus_voltage_sensor;
    dout_t* ptr_brake_digital_output;
    dout_t* ptr_ntc_relay;
    motor_pow_meas_t* ptr_mpm;
    flux_weakening_t* ptr_flux_weakening;
    feed_forward_t* ptr_feed_forward;
    pos_ctrl_t* ptr_pos_ctrl;
#ifdef HFINJECTION
    hfi_fp_ctrl_t* ptr_hfi;
#endif /* HFINJECTION */
    self_com_ctrl_t* ptr_self_com_ctrl;
    one_touch_tuning_t* ptr_one_touch_tuning;
} mct_t;

/**
 * @brief  Use this method to set a new value for the voltage reference used by
 *         flux weakening algorithm.
 * @param  phandle Flux weakening init strutcture.
 * @param  uint16_t New target voltage value, expressend in tenth of percentage
 *         points of available voltage.
 * @retval none
 */
void flux_weakening_set_vref(flux_weakening_t* phandle, uint16_t new_vref);

/**
 * @brief  It returns the present value of target voltage used by flux
 *         weakening algorihtm.
 * @param  phandle Flux weakening init strutcture.
 * @retval int16_t Present target voltage value expressed in tenth of
 *         percentage points of available voltage.
 */
uint16_t flux_weakening_get_vref(flux_weakening_t* phandle);

/**
 * @brief  It returns the present value of voltage actually used by flux
 *         weakening algorihtm.
 * @param  phandle Flux weakening init strutcture.
 * @retval int16_t Present averaged phase stator voltage value, expressed
 *         in s16V (0-to-peak), where
 *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767].
 */
int16_t flux_weakening_get_av_v_amplitude(flux_weakening_t* phandle);

/**
 * @brief  It returns the measure of present voltage actually used by flux
 *         weakening algorihtm as percentage of available voltage.
 * @param  phandle Flux weakening init strutcture.
 * @retval uint16_t Present averaged phase stator voltage value, expressed in
 *         tenth of percentage points of available voltage.
 */
uint16_t flux_weakening_get_av_v_percentage(flux_weakening_t* phandle);

#if 0
/**
  * @brief  Use this method to set new values for the constants utilized by
  *         feed-forward algorithm.
  * @param  phandle Feed forward  strutcture.
  * @param  new_constants The ff_tuning_struct_t containing constants utilized by
  *         feed-forward algorithm.
  * @retval none
  */
void feed_forward_setff_constants( feed_forward_t * phandle, ff_tuning_struct_t new_constants );

/**
  * @brief  Use this method to get present values for the constants utilized by
  *         feed-forward algorithm.
  * @param  phandle Feed forward  strutcture.
  * @retval ff_tuning_struct_t Values of the constants utilized by
  *         feed-forward algorithm.
  */
ff_tuning_struct_t feed_forward_getff_constants( feed_forward_t * phandle );

/**
  * @brief  Use this method to get present values for the vqd feed-forward
  *         components.
  * @param  phandle Feed forward  strutcture.
  * @retval qd_t vqd feed-forward components.
  */
qd_t feed_forward_get_vqdff( feed_forward_t * phandle );

/**
  * @brief  Use this method to get values of the averaged output of qd axes
  *         currents PI regulators.
  * @param  phandle Feed forward  strutcture.
  * @retval qd_t Averaged output of qd axes currents PI regulators.
  */
qd_t feed_forward_get_vqd_av_pi_out( feed_forward_t * phandle );
#endif
#ifdef HFINJECTION
/**
 * @brief  It returns the rotor angle lock value
 * @param  phandle related HFI_FP_Ctrl_Handle
 * @retval int16_t Rotor angle lock value
 */
int16_t hfi_fp_get_rotor_angle_lock(hfi_fp_ctrl_t* phandle);

/**
 * @brief  It returns the saturation difference measured during the last
 *         north/south identification stage.
 * @param  phandle related HFI_FP_Ctrl_Handle
 * @retval int16_t Saturation difference measured during the last north/south
 *         identification stage.
 */
int16_t hfi_fp_get_saturation_difference(hfi_fp_ctrl_t* phandle);

/**
 * @brief  It return the quantity that shall be put in the DAC to tune the HFI
 * @param  phandle related HFI_FP_Ctrl_Handle
 * @retval int16_t HFI current
 */
int16_t hfi_fp_get_current(hfi_fp_ctrl_t* phandle);

/**
 * @brief  It returns the Track PI
 * @param  phandle related HFI_FP_Ctrl_Handle
 * @retval CPI Track PI
 */
pid_integer_t* hfi_fp_get_pi_track(hfi_fp_ctrl_t* phandle);

/**
 * @brief  It set the min saturation difference used to validate the
 *         north/south identification stage.
 * @param  phandle related hfi_fp_Ctrl_Handle
 * @param  hMinSaturationDifference Min Saturation difference used to validate
 *         the north/south identification stage.
 *         identification stage.
 * @retval none
 */
void hfi_fp_set_min_saturation_difference(hfi_fp_ctrl_t* phandle, int16_t MinSaturationDifference);
#endif /* HFINJECTION */

/**
 * @brief  It allows changing applied open loop phase voltage.
 * @param  phandle related object of class COL
 * @param  new_voltage New voltage value to be applied by the open loop.
 * @retval None
 */
void ol_update_voltage(open_loop_t* phandle, int16_t new_voltage);

/**
 * @brief  It updates the Kp gain
 * @param  CPI PI object
 * @param  int16_t New Kp gain
 * @retval None
 */
void pid_set_kp(pid_integer_t* phandle, int16_t kp_gain);

/**
 * @brief  It updates the Ki gain
 * @param  CPI PI object
 * @param  int16_t New Ki gain
 * @retval None
 */
void pid_set_ki(pid_integer_t* phandle, int16_t ki_gain);

/**
 * @brief  It returns the Kp gain of the passed PI object
 * @param  CPI PI regulator object
 * @retval int16_t Kp gain
 */
int16_t pid_get_kp(pid_integer_t* phandle);

/**
 * @brief  It returns the Kp gain divisor of the passed PI object
 * @param  CPI PI regulator object
 * @retval int16_t Kp gain
 */
uint16_t pid_get_kp_divisor(pid_integer_t* phandle);

/**
 * @brief  It returns the Ki gain of the passed PI object
 * @param  CPI PI regulator object
 * @retval int16_t Ki gain
 */
int16_t pid_get_ki(pid_integer_t* phandle);

/**
 * @brief  It returns the Ki gain divisor of the passed PI object
 * @param  CPI PI regulator object
 * @retval int16_t Ki gain
 */
uint16_t pid_get_ki_divisor(pid_integer_t* phandle);

/**
 * @brief  It returns the Default Kp gain of the passed PI object
 * @param  CPI PI regulator object
 * @retval int16_t Kp gain
 */
int16_t pid_get_default_kp(pid_integer_t* phandle);

/**
 * @brief  It returns the Default Ki gain of the passed PI object
 * @param  CPI PI regulator object
 * @retval int16_t Ki gain
 */
int16_t pid_get_default_ki(pid_integer_t* phandle);

/**
 * @brief  It set a new value into the PI integral term
 * @param  CPI PI regulator object
 * @param  int32_t New integral term value
 * @retval None
 */
void pid_set_integral_term(pid_integer_t* phandle, int32_t integral_term_value);

/**
 * @brief  It set a new value into the PID Previous error variable required to
 *         compute derivative term
 * @param  phandle regulator object
 * @param  prev_process_var_error New integral term value
 * @retval None
 */
void pid_set_prev_error(pid_integer_t* phandle, int32_t prev_process_var_error);

/**
 * @brief  It updates the Kd gain
 * @param  phandle PID regulator object
 * @param  kd_gain New Kd gain
 * @retval None
 */
void pid_set_kd(pid_integer_t* phandle, int16_t kd_gain);

/**
 * @brief  It returns the Kd gain of the PID object passed
 * @param  phandle PID regulator object
 * @retval int16_t Kd gain
 */
int16_t pid_get_kd(pid_integer_t* phandle);

/**
 * @brief  Execute a regular conversion using ADC1.
 *         The function is not re-entrant (can't executed twice at the same time)
 *         It returns 0xFFFF in case of conversion error.
 * @param  phandle PWM component handler
 * @param  channel ADC channel used for the regular conversion
 * @retval It returns converted value or oxFFFF for conversion error
 */
uint16_t pwmc_exec_regular_conv(pwmc_t* phandle, uint8_t channel);

/**
 * @brief  It sets the specified sampling time for the specified ADC channel
 *         on ADC1. It must be called once for each channel utilized by user
 * @param  phandle PWM component handler
 * @param  ad_conv_struct struct containing ADC channel and sampling time
 * @retval none
 */
void pwmc_adc_set_sampling_time(pwmc_t* phandle, ad_conv_t ad_conv_struct);

/**
 * @brief  It is used to modify the default value of duration of a specific
 *         rev up phase.
 *         Note: The module can be also compiled commenting the
 *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
 *         and the RAM usage if the tuning is not required in this case this
 *         function has no effect.
 * @param  phandle related object of class CRUC.
 * @param  bPhase is the rev up phase, zero based, to be modified.
 * @param  durationms is the new value of duration for that phase.
 * @retval none.
 */
void ruc_set_phase_durationms(rev_up_ctrl_t* phandle, uint8_t bPhase, uint16_t durationms);

/* Function used to set the targeted motor speed at the end of a specific phase. */
void ruc_set_phase_final_mec_speed_unit(rev_up_ctrl_t* phandle, uint8_t bPhase, int16_t final_mec_speed_unit);

/**
 * @brief  It is used to modify the default value of motor torque at the end of
 *         a specific rev up phase.
 *         Note: The module can be also compiled commenting the
 *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
 *         and the RAM usage if the tuning is not required in this case this
 *         function has no effect.
 * @param  phandle related object of class CRUC.
 * @param  bPhase is the rev up phase, zero based, to be modified.
 * @param  final_torque is the new value of motor torque at the end of that
 *         phase. This value represents actually the Iq current expressed in
 *         digit.
 * @retval none.
 */
void ruc_set_phase_final_torque(rev_up_ctrl_t* phandle, uint8_t bPhase, int16_t final_torque);

/**
 * @brief  It is used to read the current value of duration of a specific rev
 *         up phase.
 *         Note: The module can be also compiled commenting the
 *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
 *         and the RAM usage if the tuning is not required in this case this
 *         function has no effect.
 * @param  phandle related object of class CRUC.
 * @param  bPhase is the rev up phase, zero based, to be read.
 * @retval uint16_t The current value of duration for that phase expressed in
 *         milliseconds.
 */
uint16_t ruc_get_phase_durationms(rev_up_ctrl_t* phandle, uint8_t bPhase);

/* Function used to read the targeted mechanical speed of a specific RevUp phase */
int16_t ruc_get_phase_final_mec_speed_unit(rev_up_ctrl_t* phandle, uint8_t bPhase);

/**
 * @brief  It is used to read the current value of motor torque at the end of a
 *         specific rev up phase.
 *         Note: The module can be also compiled commenting the
 *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
 *         and the RAM usage if the tuning is not required in this case this
 *         function has no effect.
 * @param  phandle related object of class CRUC.
 * @param  bPhase is the rev up phase, zero based, to be read.
 * @retval int16_t The current value of motor torque at the end of that phase.
 *         This value represents actually the Iq current expressed in digit.
 */
int16_t ruc_get_phase_final_torque(rev_up_ctrl_t* phandle, uint8_t bPhase);

/**
 * @brief  It is used to get information about the number of phases relative to
 *         the programmed rev up.
 *         Note: The module can be also compiled commenting the
 *         define RUC_ALLOWS_TUNING to optimize the flash memory occupation
 *         and the RAM usage if the tuning is not required in this case this
 *         function has no effect.
 * @param  phandle related object of class CRUC.
 * @retval uint8_t The number of phases relative to the programmed rev up.
 */
uint8_t ruc_get_number_of_phases(rev_up_ctrl_t* phandle);

/**
 * @brief  Returns latest averaged temperature measurement expressed in Celsius degrees
 *
 * @param  phandle: Pointer on Handle structure of TemperatureSensor component
 *
 * @retval Latest averaged temperature measurement in Celsius degrees
 */
int16_t ntc_get_av_temp_c(ntc_t* phandle);

/**
 * @brief  Returns Temperature mesurement fault status
 * Fault can be either MC_OVER_TEMP or MC_NO_ERROR according on protection threshold values set
 *
 * @param  phandle: Pointer on Handle structure of TemperatureSensor component.
 *
 * @retval Fault code error
 */
uint16_t ntc_check_temp(ntc_t* phandle);

/**
 * @brief It returns the state of the digital output
 * @param this object of class DOUT
 * @retval OutputState_t Digital output state (ACTIVE or INACTIVE)
 */
doutput_state_t dout_get_output_state(dout_t* phandle);

/**
 * @brief  It returns the state of Selfcommissioning procedure.
 * @param  this related object of class CSCC.
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
 **/
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
  * @param  current Current used for RL determination.
  * @retval none
  */
void scc_set_nominal_current(self_com_ctrl_t* phandle, float current);

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
 * @retval float It returns the nominal speed estimated by OTT in RPM.
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
  * @brief  Change the nominal current .
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

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MC_TUNING_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
