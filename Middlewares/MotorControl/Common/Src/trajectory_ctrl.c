/**
 ******************************************************************************
 * @file    trajectory_ctrl.c
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions that implement the features
 *          of Position Control Mode component of the motor Control SDK.
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
#include "trajectory_ctrl.h"

#include "speed_pos_fdbk.h"

/**
 * @brief  It initializes the static variables used by the position control modes.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @param  ptr_pid_pos_reg pointer on the handler of the current instance of PID used for the position regulation.
 * @param  ptr_stc pointer on the handler of the current instance of the SpeednTorqCtrl component.
 * @param  ptr_enc handler of the current instance of the EncAlignCtrl component.
 * @retval none
 */
void tc_init(pos_ctrl_t* phandle, pid_integer_t* ptr_pid_pos_reg, speedn_torq_ctrl_t* ptr_stc, encoder_t* ptr_enc)
{
    phandle->movement_duration = 0.0f;
    phandle->angle_step = 0.0f;
    phandle->sub_step[0] = 0.0f;
    phandle->sub_step[1] = 0.0f;
    phandle->sub_step[2] = 0.0f;
    phandle->sub_step[3] = 0.0f;
    phandle->sub_step[4] = 0.0f;
    phandle->sub_step[5] = 0.0f;

    phandle->sub_step_duration = 0;

    phandle->jerk = 0.0f;
    phandle->cruise_speed = 0.0f;
    phandle->acceleration = 0.0f;
    phandle->omega = 0.0f;
    phandle->omega_prev = 0.0f;
    phandle->theta = 0.0f;
    phandle->theta_prev = 0.0f;
    phandle->received_th = 0.0f;
    phandle->tc_tick = 0;
    phandle->elapse_time = 0.0f;

    phandle->position_control_regulation = DISABLE;
    phandle->position_ctrl_status = TC_READY_FOR_COMMAND;

    phandle->ptr_enc = ptr_enc;
    phandle->ptr_stc = ptr_stc;
    phandle->pid_pos_regulator = ptr_pid_pos_reg;

    phandle->mec_angle_offset = 0;
}

/**
 * @brief  It configures the trapezoidal speed trajectory.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @param  starting_angle Current mechanical position.
 * @param  angle_step Target mechanical position.
 * @param  movement_duration Duration to reach the final position.
 * @retval true  = Trajectory command programmed
 *         false = Not ready for a new trajectory configuration.
 */
bool tc_move_command(pos_ctrl_t* phandle, float starting_angle, float angle_step, float movement_duration)
{
    bool ret_config_status = false;
    float minimum_step_duration;

    if ((phandle->position_ctrl_status == TC_FOLLOWING_ON_GOING) && (movement_duration > 0)) {
        // Back to Move command as the movement duration is different from 0
        phandle->position_ctrl_status = TC_READY_FOR_COMMAND;
    }

    if ((phandle->position_ctrl_status == TC_READY_FOR_COMMAND) && (movement_duration > 0)) {
        phandle->position_control_regulation = ENABLE;

        minimum_step_duration = (9.0f * phandle->sampling_time);

        // WARNING: Movement duration value is rounded to the nearest valid value
        //          [(DeltaT/9) / sampling_time]:  shall be an integer value
        phandle->movement_duration = (float)((int)(movement_duration / minimum_step_duration)) * minimum_step_duration;

        phandle->starting_angle = starting_angle;
        phandle->angle_step = angle_step;
        phandle->final_angle = starting_angle + angle_step;

        // sub_step duration = DeltaT/9  (DeltaT represents the total duration of the programmed movement)
        phandle->sub_step_duration = phandle->movement_duration / 9.0f;

        // Sub step of acceleration phase
        phandle->sub_step[0] = 1 * phandle->sub_step_duration; /* Sub-step 1 of acceleration phase */
        phandle->sub_step[1] = 2 * phandle->sub_step_duration; /* Sub-step 2 of acceleration phase */
        phandle->sub_step[2] = 3 * phandle->sub_step_duration; /* Sub-step 3 of acceleration phase */

        // Sub step of  deceleration Phase
        phandle->sub_step[3] = 6 * phandle->sub_step_duration; /* Sub-step 1 of deceleration phase */
        phandle->sub_step[4] = 7 * phandle->sub_step_duration; /* Sub-step 2 of deceleration phase */
        phandle->sub_step[5] = 8 * phandle->sub_step_duration; /* Sub-step 3 of deceleration phase */

        // jerk (J) to be used by the trajectory calculator to integrate (step by step) the target position.
        // J = Deltatheta/(12 * A * A * A)  => Deltatheta = final position and A = Sub-Step duration
        phandle->jerk =
        phandle->angle_step / (12 * phandle->sub_step_duration * phandle->sub_step_duration * phandle->sub_step_duration);

        // Speed cruiser = 2*J*A*A)
        phandle->cruise_speed = 2 * phandle->jerk * phandle->sub_step_duration * phandle->sub_step_duration;

        phandle->elapse_time = 0.0f;

        phandle->omega = 0.0f;
        phandle->acceleration = 0.0f;
        phandle->theta = starting_angle;

        phandle->position_ctrl_status = TC_MOVEMENT_ON_GOING; /* new trajectory has been programmed */

        ret_config_status = true;
    }
    return (ret_config_status);
}

/**
 * @brief  Used to follow an angular position command.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @param  Angle Target mechanical position.
 * @retval none
 */
void tc_follow_command(pos_ctrl_t* phandle, float Angle)
{
    float omega = 0, acceleration = 0, dt = 0;

    // Estimate speed
    if (phandle->received_th > 0) {
        // Calculate dt
        dt = phandle->tc_tick * phandle->sys_tick_period;
        phandle->tc_tick = 0;
        if (dt > 0) {
            omega = (Angle - phandle->theta_prev) / dt;
        }
    }

    // Estimated acceleration
    if (phandle->received_th > 1) {
        if (dt > 0) {
            acceleration = (omega - phandle->omega_prev) / dt;
        }
    }

    // Update state variable
    phandle->theta_prev = Angle;
    phandle->omega_prev = omega;
    if (phandle->received_th < 2) {
        phandle->received_th++;
    }

    phandle->acceleration = acceleration;
    phandle->omega = omega;
    phandle->theta = Angle;

    phandle->position_ctrl_status = TC_FOLLOWING_ON_GOING; /* follow mode has been programmed */

    return;
}

/**
 * @brief  It proceeds on the position control loop.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval none
 */
void tc_position_regulation(pos_ctrl_t* phandle)
{
    int32_t mec_angle_ref;
    int32_t mec_angle;
    int32_t error;
    int32_t torque_ref_pos;

    if (phandle->position_ctrl_status == TC_MOVEMENT_ON_GOING) {
        tc_move_execution(phandle);
    }

    if (phandle->position_ctrl_status == TC_FOLLOWING_ON_GOING) {
        tc_follow_execution(phandle);
    }

    if (phandle->position_control_regulation == ENABLE) {
        mec_angle_ref = (int32_t)(phandle->theta * RADTOS16);

        mec_angle = spd_get_mec_angle(stc_get_speed_sensor(phandle->ptr_stc));
        error = mec_angle_ref - mec_angle;
        torque_ref_pos = pid_controller(phandle->pid_pos_regulator, error);

        stc_set_control_mode(phandle->ptr_stc, STC_TORQUE_MODE);
        stc_exec_ramp(phandle->ptr_stc, torque_ref_pos, 0);
    }
}

/**
 * @brief  It executes the programmed trajectory movement.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval none
 */
void tc_move_execution(pos_ctrl_t* phandle)
{
    float jerk_applied = 0;

    if (phandle->elapse_time < phandle->sub_step[0]) // 1st Sub-Step interval time of acceleration phase
    {
        jerk_applied = phandle->jerk;
    } else if (phandle->elapse_time < phandle->sub_step[1]) // 2nd Sub-Step interval time of acceleration phase
    {
    } else if (phandle->elapse_time < phandle->sub_step[2]) // 3rd Sub-Step interval time of acceleration phase
    {
        jerk_applied = -(phandle->jerk);
    } else if (phandle->elapse_time < phandle->sub_step[3]) // Speed Cruise phase (after acceleration and before deceleration phases)
    {
        phandle->acceleration = 0.0f;
        phandle->omega = phandle->cruise_speed;
    } else if (phandle->elapse_time < phandle->sub_step[4]) // 1st Sub-Step interval time of deceleration phase
    {
        jerk_applied = -(phandle->jerk);
    } else if (phandle->elapse_time < phandle->sub_step[5]) // 2nd Sub-Step interval time of deceleration phase
    {
    } else if (phandle->elapse_time < phandle->movement_duration) // 3rd Sub-Step interval time of deceleration phase
    {
        jerk_applied = phandle->jerk;
    } else {
        phandle->theta = phandle->final_angle;
        phandle->position_ctrl_status = TC_TARGET_POSITION_REACHED;
    }

    if (phandle->position_ctrl_status == TC_MOVEMENT_ON_GOING) {
        phandle->acceleration += jerk_applied * phandle->sampling_time;
        phandle->omega += phandle->acceleration * phandle->sampling_time;
        phandle->theta += phandle->omega * phandle->sampling_time;
    }

    phandle->elapse_time += phandle->sampling_time;

    if (tc_ramp_completed(phandle)) {
        if (phandle->alignment_status == TC_ZERO_ALIGNMENT_START) {
            // Ramp is used to search the zero index, if completed there is no z signal
            phandle->alignment_status = TC_ALIGNMENT_ERROR;
        }
        phandle->elapse_time = 0;
        phandle->position_ctrl_status = TC_READY_FOR_COMMAND;
    }
}

/**
 * @brief  It updates the angular position.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval none
 */
void tc_follow_execution(pos_ctrl_t* phandle)
{
    phandle->omega += phandle->acceleration * phandle->sampling_time;
    phandle->theta += phandle->omega * phandle->sampling_time;
}

/**
 * @brief  It handles the alignment phase at starting before any position commands.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval none
 */
void tc_enc_alignment_command(pos_ctrl_t* phandle)
{
    int32_t mec_angle_ref;

    if (phandle->alignment_status == TC_ALIGNMENT_COMPLETED) {
        phandle->position_ctrl_status = TC_READY_FOR_COMMAND;
        // Do nothing - EncAlignment must be done only one time after the power on
    } else {
        if (phandle->alignment_cfg == TC_ABSOLUTE_ALIGNMENT_SUPPORTED) {
            // If index is supported start the search of the zero
            phandle->encoder_absolute_aligned = false;
            mec_angle_ref = spd_get_mec_angle(stc_get_speed_sensor(phandle->ptr_stc));
            tc_move_command(phandle, (float)(mec_angle_ref) / RADTOS16, Z_ALIGNMENT_NB_ROTATION, Z_ALIGNMENT_DURATION);
            phandle->alignment_status = TC_ZERO_ALIGNMENT_START;
        } else {
            // If index is not supprted set the alignment angle as zero reference
            phandle->ptr_enc->_Super.mec_angle = 0;
            phandle->alignment_status = TC_ALIGNMENT_COMPLETED;
            phandle->position_ctrl_status = TC_READY_FOR_COMMAND;
            phandle->position_control_regulation = ENABLE;
        }
    }
}

/**
 * @brief  It controls if time allowed for movement is completed.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval true  = the programmed trajectory movement is completed
 *         false = the trajectory movement execution is still ongoing.
 */
bool tc_ramp_completed(pos_ctrl_t* phandle)
{
    bool ret_val = false;

    // Check that entire sequence (acceleration - Cruise - Deceleration) is completed.
    if (phandle->elapse_time > phandle->movement_duration + phandle->sampling_time) {
        ret_val = true;
    }
    return (ret_val);
}

/**
 * @brief  Set the absolute zero mechanical position.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval none
 */
void tc_encoder_reset(pos_ctrl_t* phandle)
{
    if ((!phandle->encoder_absolute_aligned) && (phandle->alignment_status == TC_ZERO_ALIGNMENT_START)) {
        phandle->mec_angle_offset = phandle->ptr_enc->_Super.mec_angle;
        phandle->ptr_enc->_Super.mec_angle = 0;
        phandle->encoder_absolute_aligned = true;
        phandle->alignment_status = TC_ALIGNMENT_COMPLETED;
        phandle->position_ctrl_status = TC_READY_FOR_COMMAND;
        phandle->theta = 0.0f;
        enc_set_mec_angle(phandle->ptr_enc, phandle->mec_angle_offset);
    }
}

/**
 * @brief  Returns the current rotor mechanical angle, expressed in radiant.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval current mechanical position
 */
float tc_get_current_position(pos_ctrl_t* phandle)
{
    return ((float)((spd_get_mec_angle(stc_get_speed_sensor(phandle->ptr_stc))) / RADTOS16));
}

/**
 * @brief  Returns the target rotor mechanical angle, expressed in radiant.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval Target mechanical position
 */
float tc_get_target_position(pos_ctrl_t* phandle)
{
    return (phandle->final_angle);
}

/**
 * @brief  Returns the duration used to execute the movement, expressed in seconds.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval Duration of programmed movement
 */
float tc_get_move_duration(pos_ctrl_t* phandle)
{
    return (phandle->movement_duration);
}

/**
 * @brief  Returns the status of the position control execution.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval Position Control Status
 */
pos_ctrl_status_t tc_get_control_position_status(pos_ctrl_t* phandle)
{
    return (phandle->position_ctrl_status);
}

/**
 * @brief  Returns the status after the rotor alignment phase.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval Alignment Status
 */
align_status_t tc_get_alignment_status(pos_ctrl_t* phandle)
{
    return (phandle->alignment_status);
}

/**
 * @brief  It increments Tick counter used in follow mode.
 * @param  phandle: handler of the current instance of the Position Control component.
 * @retval none
 */
void tc_inc_tick(pos_ctrl_t* phandle)
{
    phandle->tc_tick++;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
