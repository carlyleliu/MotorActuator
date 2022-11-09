/**
 ******************************************************************************
 * @file    mc_interface.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of the MC Interface component of the Motor Control SDK:
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

#include "motor_control_interface.h"

/** @addtogroup MCSDK
 * @{
 */

/** @defgroup MCInterface Motor Control Interface
 * @brief MC Interface component of the Motor Control SDK
 *
 * @todo Document the MC Interface "module".
 *
 * @{
 */

/* Private macros ------------------------------------------------------------*/
/**
 * @brief This macro converts the exported enum from the state machine to the corresponding bit field.
 */
#define BC(state) (1u << ((uint16_t)((uint8_t)(state))))

/* Functions -----------------------------------------------*/

/**
 * @brief  Initializes all the object variables, usually it has to be called
 *         once right after object creation. It is also used to assign the
 *         state machine object, the speed and torque controller, and the FOC
 *         drive object to be used by MC Interface.
 * @param  pHandle pointer on the component instance to initialize.
 * @param  ptr_stm the state machine object used by the MCI.
 * @param  ptr_stc the speed and torque controller used by the MCI.
 * @param  ptr_foc_vars pointer to FOC vars to be used by MCI.
 * @retval none.
 */
void mci_init(motor_control_interface_t* phandle, state_machine_t* ptr_stm, speedn_torq_ctrl_t* ptr_stc, ptr_foc_vars_t ptr_foc_vars)
{
    phandle->motor_control_interface.ptr_stm = ptr_stm;
    phandle->motor_control_interface.ptr_stc = ptr_stc;
    phandle->motor_control_interface.ptr_foc_vars = ptr_foc_vars;

    /* Buffer related initialization */
    phandle->motor_control_interface.last_command = MCI_NOCOMMANDSYET;
    phandle->motor_control_interface.final_speed = 0;
    phandle->motor_control_interface.final_torque = 0;
    phandle->motor_control_interface.durationms = 0;
    phandle->motor_control_interface.command_state = MCI_BUFFER_EMPTY;
}

/**
 * @brief  This is a buffered command to set a motor speed ramp. This commands
 *         don't become active as soon as it is called but it will be executed
 *         when the ptr_stm state is START_RUN or RUN. User can check the status
 *         of the command calling the MCI_IsCommandAcknowledged method.
 * @param  pHandle Pointer on the component instance to operate on.
 * @param  final_speed is the value of mechanical rotor speed reference at the
 *         end of the ramp expressed in tenths of HZ.
 * @param  durationms the duration of the ramp expressed in milliseconds. It
 *         is possible to set 0 to perform an instantaneous change in the
 *         value.
 * @retval none.
 */
void mci_exec_speed_ramp(motor_control_interface_t* phandle, int16_t final_speed, uint16_t durationms)
{
    phandle->motor_control_interface.last_command = MCI_EXECSPEEDRAMP;
    phandle->motor_control_interface.final_speed = final_speed;
    phandle->motor_control_interface.durationms = durationms;
    phandle->motor_control_interface.command_state = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    phandle->motor_control_interface.last_modality_set_by_user = STC_SPEED_MODE;
}

/**
 * @brief  This is a buffered command to set a motor torque ramp. This commands
 *         don't become active as soon as it is called but it will be executed
 *         when the ptr_stm state is START_RUN or RUN. User can check the status
 *         of the command calling the MCI_IsCommandAcknowledged method.
 * @param  final_torque is the value of motor torque reference at the end of
 *         the ramp. This value represents actually the Iq current expressed in
 *         digit.
 *         To convert current expressed in Amps to current expressed in digit
 *         is possible to use the formula:
 *         Current (digit) = [Current(Amp) * 65536 * Rshunt * Aop] / Vdd micro.
 * @param  durationms the duration of the ramp expressed in milliseconds. It
 *         is possible to set 0 to perform an instantaneous change in the
 *         value.
 * @retval none.
 */
void mci_exec_torque_ramp(motor_control_interface_t* phandle, int16_t final_torque, uint16_t durationms)
{
    phandle->motor_control_interface.last_command = MCI_EXECTORQUERAMP;
    phandle->motor_control_interface.final_torque = final_torque;
    phandle->motor_control_interface.durationms = durationms;
    phandle->motor_control_interface.command_state = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    phandle->motor_control_interface.last_modality_set_by_user = STC_TORQUE_MODE;
}

/**
 * @brief  This is a buffered command to set directly the motor current
 *         references Iq and Id. This commands don't become active as soon as
 *         it is called but it will be executed when the ptr_stm state is
 *         START_RUN or RUN. User can check the status of the command calling
 *         the MCI_IsCommandAcknowledged method.
 * @param  iqd_ref current references on qd reference frame in qd_t
 *         format.
 * @retval none.
 */
void mci_set_current_references(motor_control_interface_t* phandle, qd_t iqd_ref)
{
    phandle->motor_control_interface.last_command = MCI_SETCURRENTREFERENCES;
    phandle->motor_control_interface.iqd_ref.q = iqd_ref.q;
    phandle->motor_control_interface.iqd_ref.d = iqd_ref.d;
    phandle->motor_control_interface.command_state = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    phandle->motor_control_interface.last_modality_set_by_user = STC_TORQUE_MODE;
}

/**
 * @brief  This is a user command used to begin the start-up procedure.
 *         If the state machine is in IDLE state the command is executed
 *         instantaneously otherwise the command is discarded. User must take
 *         care of this possibility by checking the return value.
 *         Before calling MCI_StartMotor it is mandatory to execute one of
 *         these commands:\n
 *         MCI_ExecSpeedRamp\n
 *         MCI_ExecTorqueRamp\n
 *         MCI_SetCurrentReferences\n
 *         Otherwise the behaviour in run state will be unpredictable.\n
 *         <B>Note:</B> The MCI_StartMotor command is used just to begin the
 *         start-up procedure moving the state machine from IDLE state to
 *         IDLE_START. The command MCI_StartMotor is not blocking the execution
 *         of project until the motor is really running; to do this, the user
 *         have to check the state machine and verify that the RUN state (or
 *         any other state) has been reached.
 * @retval bool It returns true if the command is successfully executed
 *         otherwise it return false.
 */
bool mci_start_motor(motor_control_interface_t* phandle)
{
    bool RetVal = stm_next_state(phandle->motor_control_interface.ptr_stm, IDLE_START);

    if (RetVal == true) {
        phandle->motor_control_interface.command_state = MCI_COMMAND_NOT_ALREADY_EXECUTED;
    }

    return RetVal;
}

/**
 * @brief  This is a user command used to begin the stop motor procedure.
 *         If the state machine is in RUN or START states the command is
 *         executed instantaneously otherwise the command is discarded. User
 *         must take care of this possibility by checking the return value.\n
 *         <B>Note:</B> The MCI_StopMotor command is used just to begin the
 *         stop motor procedure moving the state machine to ANY_STOP.
 *         The command MCI_StopMotor is not blocking the execution of project
 *         until the motor is really stopped; to do this, the user have to
 *         check the state machine and verify that the IDLE state has been
 *         reached again.
 * @retval bool It returns true if the command is successfully executed
 *         otherwise it return false.
 */
bool mci_stop_motor(motor_control_interface_t* phandle)
{
    return stm_next_state(phandle->motor_control_interface.ptr_stm, ANY_STOP);
}

/**
 * @brief  This is a user command used to indicate that the user has seen the
 *         error condition. If is possible, the command is executed
 *         instantaneously otherwise the command is discarded. User must take
 *         care of this possibility by checking the return value.
 * @retval bool It returns true if the command is successfully executed
 *         otherwise it return false.
 */
bool mci_fault_acknowledged(motor_control_interface_t* phandle)
{
    return stm_fault_acknowledged(phandle->motor_control_interface.ptr_stm);
}

/**
 * @brief  This is a user command used to begin the encoder alignment procedure.
 *         If the state machine is in IDLE state the command is executed
 *         instantaneously otherwise the command is discarded. User must take
 *         care of this possibility by checking the return value.\n
 *         <B>Note:</B> The MCI_EncoderAlign command is used just to begin the
 *         encoder alignment procedure moving the state machine from IDLE state
 *         to IDLE_ALIGNMENT. The command MCI_EncoderAlign is not blocking the
 *         execution of project until the encoder is really calibrated; to do
 *         this, the user have to check the state machine and verify that the
 *         IDLE state has been reached again.
 * @retval bool It returns true if the command is successfully executed
 *         otherwise it return false.
 */
bool mci_encoder_align(motor_control_interface_t* phandle)
{
    return stm_next_state(phandle->motor_control_interface.ptr_stm, IDLE_ALIGNMENT);
}

/**
 * @brief  This is usually a method managed by task. It must be called
 *         periodically in order to check the status of the related ptr_stm object
 *         and eventually to execute the buffered command if the condition
 *         occurs.
 * @retval none.
 */
void mci_exec_buffered_commands(motor_control_interface_t* phandle)
{
    if (phandle->motor_control_interface.command_state == MCI_COMMAND_NOT_ALREADY_EXECUTED) {
        bool commandHasBeenExecuted = false;
        switch (phandle->motor_control_interface.last_command) {
            case MCI_EXECSPEEDRAMP: {
                phandle->motor_control_interface.ptr_foc_vars->drive_input = INTERNAL;
                stc_set_control_mode(phandle->motor_control_interface.ptr_stc, STC_SPEED_MODE);
                commandHasBeenExecuted =
                stc_exec_ramp(phandle->motor_control_interface.ptr_stc, phandle->motor_control_interface.final_speed,
                              phandle->motor_control_interface.durationms);
            } break;
            case MCI_EXECTORQUERAMP: {
                phandle->motor_control_interface.ptr_foc_vars->drive_input = INTERNAL;
                stc_set_control_mode(phandle->motor_control_interface.ptr_stc, STC_TORQUE_MODE);
                commandHasBeenExecuted =
                stc_exec_ramp(phandle->motor_control_interface.ptr_stc, phandle->motor_control_interface.final_torque,
                              phandle->motor_control_interface.durationms);
            } break;
            case MCI_SETCURRENTREFERENCES: {
                phandle->motor_control_interface.ptr_foc_vars->drive_input = EXTERNAL;
                phandle->motor_control_interface.ptr_foc_vars->iqd_ref = phandle->motor_control_interface.iqd_ref;
                commandHasBeenExecuted = true;
            } break;
            default:
                break;
        }

        if (commandHasBeenExecuted) {
            phandle->motor_control_interface.command_state = MCI_COMMAND_EXECUTED_SUCCESFULLY;
        } else {
            phandle->motor_control_interface.command_state = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
        }
    }
}

/**
 * @brief  It returns information about the state of the last buffered command.
 * @retval command_state_t  It can be one of the following codes:
 *         - MCI_BUFFER_EMPTY if no buffered command has been called.
 *         - MCI_COMMAND_NOT_ALREADY_EXECUTED if the buffered command
 *         condition hasn't already occurred.
 *         - MCI_COMMAND_EXECUTED_SUCCESFULLY if the buffered command has
 *         been executed successfully. In this case calling this function reset
 *         the command state to BC_BUFFER_EMPTY.
 *         - MCI_COMMAND_EXECUTED_UNSUCCESFULLY if the buffered command has
 *         been executed unsuccessfully. In this case calling this function
 *         reset the command state to BC_BUFFER_EMPTY.
 */
mci_command_state_t mci_is_command_acknowledged(motor_control_interface_t* phandle)
{
    mci_command_state_t retVal = phandle->motor_control_interface.command_state;

    if ((retVal == MCI_COMMAND_EXECUTED_SUCCESFULLY) | (retVal == MCI_COMMAND_EXECUTED_UNSUCCESFULLY)) {
        phandle->motor_control_interface.command_state = MCI_BUFFER_EMPTY;
    }
    return retVal;
}

/**
 * @brief  It returns information about the state of the related ptr_stm object.
 * @retval state_t It returns the current state of the related ptr_stm object.
 */
State_t mci_get_stm_state(motor_control_interface_t* phandle)
{
    return stm_get_state(phandle->motor_control_interface.ptr_stm);
}

/**
 * @brief It returns a 16 bit fields containing information about faults
 *        historically occurred since the state machine has been moved into
 *        FAULT_NOW state.
 * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
 * @param pHandle Pointer on the component instance to work on.
 * @retval uint16_t  16 bit fields with information about the faults
 *         historically occurred since the state machine has been moved into
 *         FAULT_NOW state.
 * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
 */
uint16_t mci_get_occurred_faults(motor_control_interface_t* phandle)
{
    return (uint16_t)(stm_get_fault_state(phandle->motor_control_interface.ptr_stm));
}

/**
 * @brief It returns a 16 bit fields containing information about faults
 *        currently present.
 * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
 * @param pHandle Pointer on the component instance to work on.
 * @retval uint16_t  16 bit fields with information about about currently
 *         present faults.
 * \n\link Fault_generation_error_codes Returned error codes are listed here \endlink
 */
uint16_t mci_get_current_faults(motor_control_interface_t* phandle)
{
    return (uint16_t)(stm_get_fault_state(phandle->motor_control_interface.ptr_stm) >> 16);
}

/**
 * @brief  It returns the modality of the speed and torque controller.
 * @retval stc_modality_t It returns the modality of STC. It can be one of
 *         these two values: STC_TORQUE_MODE or STC_SPEED_MODE.
 */
stc_modality_t mci_get_control_mode(motor_control_interface_t* phandle)
{
    return phandle->motor_control_interface.last_modality_set_by_user;
}

/**
 * @brief  It returns the motor direction imposed by the last command
 *         (MCI_ExecSpeedRamp, MCI_ExecTorqueRamp or MCI_SetCurrentReferences).
 * @retval int16_t It returns 1 or -1 according the sign of final_speed,
 *         final_torque or iqd_ref.q of the last command.
 */
int16_t mci_get_imposed_motor_direction(motor_control_interface_t* phandle)
{
    int16_t retVal = 1;

    switch (phandle->motor_control_interface.last_command) {
        case MCI_EXECSPEEDRAMP:
            if (phandle->motor_control_interface.final_speed < 0) {
                retVal = -1;
            }
            break;
        case MCI_EXECTORQUERAMP:
            if (phandle->motor_control_interface.final_torque < 0) {
                retVal = -1;
            }
            break;
        case MCI_SETCURRENTREFERENCES:
            if (phandle->motor_control_interface.iqd_ref.q < 0) {
                retVal = -1;
            }
            break;
        default:
            break;
    }
    return retVal;
}

/**
 * @brief  It returns information about the last ramp final speed sent by the
 *         user expressed in tenths of HZ.
 * @retval int16_t last ramp final speed sent by the user expressed in tehts
 *         of HZ.
 */
int16_t mci_get_last_ramp_final_speed(motor_control_interface_t* phandle)
{
    int16_t hRetVal = 0;

    /* Examine the last buffered commands */
    if (phandle->motor_control_interface.last_command == MCI_EXECSPEEDRAMP) {
        hRetVal = phandle->motor_control_interface.final_speed;
    }
    return hRetVal;
}

/**
 * @brief  Check if the settled speed or torque ramp has been completed.
 * @retval bool It returns true if the ramp is completed, false otherwise.
 */
bool mci_ramp_completed(motor_control_interface_t* phandle)
{
    bool retVal = false;

    if ((stm_get_state(phandle->motor_control_interface.ptr_stm)) == RUN) {
        retVal = stc_ramp_completed(phandle->motor_control_interface.ptr_stc);
    }

    return retVal;
}

/**
 * @brief  Stop the execution of speed ramp.
 * @retval bool It returns true if the command is executed, false otherwise.
 */
bool mci_stop_speed_ramp(motor_control_interface_t* phandle)
{
    return stc_stop_speed_ramp(phandle->motor_control_interface.ptr_stc);
}

/**
 * @brief  Stop the execution of ongoing ramp.
 */
void mci_stop_ramp(motor_control_interface_t* phandle)
{
    stc_stop_ramp(phandle->motor_control_interface.ptr_stc);
}

/**
 * @brief  It returns speed sensor reliability with reference to the sensor
 *         actually used for reference frame transformation
 * @retval bool It returns true if the speed sensor utilized for reference
 *         frame transformation and (in speed control mode) for speed
 *         regulation is reliable, false otherwise
 */
bool mci_get_spd_sensor_reliability(motor_control_interface_t* phandle)
{
    speedn_pos_fdbk_t* SpeedSensor = stc_get_speed_sensor(phandle->motor_control_interface.ptr_stc);

    return (spd_check(SpeedSensor));
}

/**
 * @brief  Returns the last computed average mechanical speed, expressed in
 *         the unit defined by #SPEED_UNIT and related to the sensor actually
 *         used by FOC algorithm
 */
int16_t mci_get_avrg_mec_speed_unit(motor_control_interface_t* phandle)
{
    speedn_pos_fdbk_t* SpeedSensor = stc_get_speed_sensor(phandle->motor_control_interface.ptr_stc);

    return (spd_get_avrg_mecspeed_unit(SpeedSensor));
}

/**
 * @brief  Returns the current mechanical rotor speed reference expressed in the unit defined by #SPEED_UNIT
 *
 *
 */
int16_t mci_get_mec_speed_ref_unit(motor_control_interface_t* phandle)
{
    return (stc_get_mec_speed_ref_unit(phandle->motor_control_interface.ptr_stc));
}

/**
 * @brief  It returns stator current iab in ab_t format
 * @retval ab_t Stator current iab
 */
ab_t mci_get_iab(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->iab);
}

/**
 * @brief  It returns stator current ialphabeta in alphabeta_t format
 * @retval alphabeta_t Stator current ialphabeta
 */
alphabeta_t mci_get_ialphabeta(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->ialphabeta);
}

/**
 * @brief  It returns stator current iqd in qd_t format
 * @retval qd_t Stator current iqd
 */
qd_t mci_get_iqd(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->iqd);
}

/**
 * @brief  It returns stator current iqdHF in qd_t format
 * @retval qd_t Stator current iqdHF if HFI is selected as main
 *         sensor. Otherwise it returns { 0, 0}.
 */
qd_t mci_get_iqd_hf(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->iqd_hf);
}

/**
 * @brief  It returns stator current iqd_ref in qd_t format
 * @retval qd_t Stator current iqd_ref
 */
qd_t mci_get_iqd_ref(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->iqd_ref);
}

/**
 * @brief  It returns stator current vqd in qd_t format
 * @retval qd_t Stator current vqd
 */
qd_t mci_get_vqd(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->vqd);
}

/**
 * @brief  It returns stator current valphabeta in alphabeta_t format
 * @retval alphabeta_t Stator current valphabeta
 */
alphabeta_t mci_get_valphabeta(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->valphabeta);
}

/**
 * @brief  It returns the rotor electrical angle actually used for reference
 *         frame transformation
 * @retval int16_t Rotor electrical angle in dpp format
 */
int16_t mci_get_el_angle_dpp(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->el_angle);
}

/**
 * @brief  It returns the reference eletrical torque, fed to derived class for
 *         Iqref and Idref computation
 * @retval int16_t Teref
 */
int16_t mci_get_teref(motor_control_interface_t* phandle)
{
    return (phandle->motor_control_interface.ptr_foc_vars->teref);
}

/**
 * @brief  It returns the motor phase current amplitude (0-to-peak) in s16A
 *         To convert s16A into Ampere following formula must be used:
 *         Current(Amp) = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
 * @retval int16_t Motor phase current (0-to-peak) in s16A
 */
int16_t mci_get_phase_current_amplitude(motor_control_interface_t* phandle)
{
    alphabeta_t Local_Curr;
    int32_t wAux1, wAux2;

    Local_Curr = phandle->motor_control_interface.ptr_foc_vars->ialphabeta;
    wAux1 = (int32_t)(Local_Curr.alpha) * Local_Curr.alpha;
    wAux2 = (int32_t)(Local_Curr.beta) * Local_Curr.beta;

    wAux1 += wAux2;
    wAux1 = mcm_sqrt(wAux1);

    if (wAux1 > INT16_MAX) {
        wAux1 = (int32_t)INT16_MAX;
    }

    return ((int16_t)wAux1);
}

/**
 * @brief  It returns the applied motor phase voltage amplitude (0-to-peak) in
 *         s16V. To convert s16V into Volts following formula must be used:
 *         PhaseVoltage(V) = [PhaseVoltage(s16A) * Vbus(V)] /[sqrt(3) *32767]
 * @retval int16_t Motor phase voltage (0-to-peak) in s16V
 */
int16_t mci_get_phase_voltage_amplitude(motor_control_interface_t* phandle)
{
    alphabeta_t Local_Voltage;
    int32_t wAux1, wAux2;

    Local_Voltage = phandle->motor_control_interface.ptr_foc_vars->valphabeta;
    wAux1 = (int32_t)(Local_Voltage.alpha) * Local_Voltage.alpha;
    wAux2 = (int32_t)(Local_Voltage.beta) * Local_Voltage.beta;

    wAux1 += wAux2;
    wAux1 = mcm_sqrt(wAux1);

    if (wAux1 > INT16_MAX) {
        wAux1 = (int32_t)INT16_MAX;
    }

    return ((int16_t)wAux1);
}

/**
 * @brief  When drive_input is set to INTERNAL, Idref should is normally managed
 *         by FOC_CalcCurrRef. Neverthless, this method allows forcing changing
 *         Idref value. Method call has no effect when either flux weakening
 *         region is entered or MTPA is enabled
 * @param  int16_t New target Id value
 * @retval none
 */
void mci_set_id_ref(motor_control_interface_t* phandle, int16_t hNewIdref)
{
    phandle->motor_control_interface.ptr_foc_vars->iqd_ref.d = hNewIdref;
    phandle->motor_control_interface.ptr_foc_vars->user_id_ref = hNewIdref;
}

/**
 * @brief  It re-initializes iqd_ref variables with their default values.
 * @retval none
 */
void mci_clear_iqd_ref(motor_control_interface_t* phandle)
{
    phandle->motor_control_interface.ptr_foc_vars->iqd_ref = stc_get_default_iqd_ref(phandle->motor_control_interface.ptr_stc);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
