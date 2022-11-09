#ifndef FRAMEWORK_MOTOR_CONTROL_INTERFACE_H
#define FRAMEWORK_MOTOR_CONTROL_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "mc_math.h"
#include "mc_tuning.h"
#include "mc_type.h"
#include "speed_torq_ctrl.h"
#include "state_machine.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MCI_BUFFER_EMPTY,                  /*!< If no buffered command has been
                                        called.*/
    MCI_COMMAND_NOT_ALREADY_EXECUTED,  /*!< If the buffered command condition
                                        hasn't already occurred.*/
    MCI_COMMAND_EXECUTED_SUCCESFULLY,  /*!< If the buffered command has been
                                        executed successfully.*/
    MCI_COMMAND_EXECUTED_UNSUCCESFULLY /*!< If the buffered command has been
                                        executed unsuccessfully.*/
} mci_command_state_t;

typedef enum {
    MCI_NOCOMMANDSYET,        /*!< No command has been set by the user.*/
    MCI_EXECSPEEDRAMP,        /*!< ExecSpeedRamp command coming from the user.*/
    MCI_EXECTORQUERAMP,       /*!< ExecTorqueRamp command coming from the user.*/
    MCI_SETCURRENTREFERENCES, /*!< SetCurrentReferences command coming from the
                               user.*/
} mci_user_commands_t;

typedef struct {
    state_machine_t* ptr_stm;         /*!< State machine object used by MCI.*/
    speedn_torq_ctrl_t* ptr_stc;      /*!< Speed and torque controller object used by MCI.*/
    ptr_foc_vars_t ptr_foc_vars;      /*!< Pointer to FOC vars used by MCI.*/
    mci_user_commands_t last_command; /*!< Last command coming from the user.*/
    int16_t final_speed;              /*!< Final speed of last ExecSpeedRamp command.*/
    int16_t final_torque;             /*!< Final torque of last ExecTorqueRamp
                                       command.*/
    qd_t iqd_ref;                     /*!< Current component of last
                                       SetCurrentReferences command.*/
    uint16_t durationms;              /*!< Duration in ms of last ExecSpeedRamp or
                                       ExecTorqueRamp command.*/

    mci_command_state_t command_state;        /*!< The status of the buffered command.*/
    stc_modality_t last_modality_set_by_user; /*!< The last stc_modality_t set by the
                                               user. */
} mci_t;

typedef struct motor_control_interface {
    mci_t motor_control_interface;
} motor_control_interface_t;

void mci_init(motor_control_interface_t* phandle, state_machine_t* ptr_stm, speedn_torq_ctrl_t* ptr_stc, ptr_foc_vars_t ptr_foc_vars);
void mci_exec_speed_ramp(motor_control_interface_t* phandle, int16_t final_speed, uint16_t durationms);
void mci_exec_torque_ramp(motor_control_interface_t* phandle, int16_t final_torque, uint16_t durationms);
void mci_set_current_references(motor_control_interface_t* phandle, qd_t iqd_ref);
bool mci_start_motor(motor_control_interface_t* phandle);
bool mci_stop_motor(motor_control_interface_t* phandle);
bool mci_fault_acknowledged(motor_control_interface_t* phandle);
bool mci_encoder_align(motor_control_interface_t* phandle);
void mci_exec_buffered_commands(motor_control_interface_t* phandle);
mci_command_state_t mci_is_command_acknowledged(motor_control_interface_t* phandle);
State_t mci_get_stm_state(motor_control_interface_t* phandle);
uint16_t mci_get_occurred_faults(motor_control_interface_t* phandle);
uint16_t mci_get_current_faults(motor_control_interface_t* phandle);
stc_modality_t mci_get_control_mode(motor_control_interface_t* phandle);
int16_t mci_get_imposed_motor_direction(motor_control_interface_t* phandle);
int16_t mci_get_last_ramp_final_speed(motor_control_interface_t* phandle);
bool mci_ramp_completed(motor_control_interface_t* phandle);
bool mci_stop_speed_ramp(motor_control_interface_t* phandle);
void mci_stop_ramp(motor_control_interface_t* phandle);
bool mci_get_spd_sensor_reliability(motor_control_interface_t* phandle);
int16_t mci_get_avrg_mec_speed_unit(motor_control_interface_t* phandle);
int16_t mci_get_mec_speed_ref_unit(motor_control_interface_t* phandle);
ab_t mci_get_iab(motor_control_interface_t* phandle);
alphabeta_t mci_get_ialphabeta(motor_control_interface_t* phandle);
qd_t mci_get_iqd(motor_control_interface_t* phandle);
qd_t mci_get_iqd_hf(motor_control_interface_t* phandle);
qd_t mci_get_iqd_ref(motor_control_interface_t* phandle);
qd_t mci_get_vqd(motor_control_interface_t* phandle);
alphabeta_t mci_get_valphabeta(motor_control_interface_t* phandle);
int16_t mci_get_el_angle_dpp(motor_control_interface_t* phandle);
int16_t mci_get_teref(motor_control_interface_t* phandle);
int16_t mci_get_phase_current_amplitude(motor_control_interface_t* phandle);
int16_t mci_get_phase_voltage_amplitude(motor_control_interface_t* phandle);
void mci_set_id_ref(motor_control_interface_t* phandle, int16_t hNewIdref);
void mci_clear_iqd_ref(motor_control_interface_t* phandle);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif
