/**
 ******************************************************************************
 * @file    trajectory_ctrl.h
 * @author  motor Control SDK Team, ST Microelectronics
 * @brief   This file provides all definitions and functions prototypes
 *          of the Position Control component of the motor Control SDK.
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
 * @ingroup TrajectoryCtrl
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __TRAJCTRL_H
#define __TRAJCTRL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "enc_align_ctrl.h"
#include "mc_type.h"
#include "speed_torq_ctrl.h"

#define RADTOS16 10430.378350470452725f /* 2^15/Pi */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define Z_ALIGNMENT_DURATION    2             /* 2 seconds */
#define Z_ALIGNMENT_NB_ROTATION (2.0f * M_PI) /* 1 turn in 2 seconds allowed to find the "Z" signal  */

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup PosCtrl
 * @{
 */

typedef enum {
    TC_READY_FOR_COMMAND = 0,
    TC_MOVEMENT_ON_GOING = 1,
    TC_TARGET_POSITION_REACHED = 2,
    TC_FOLLOWING_ON_GOING = 3
} pos_ctrl_status_t;

typedef enum {
    TC_AWAITING_FOR_ALIGNMENT = 0,
    TC_ZERO_ALIGNMENT_START = 1,
    TC_ALIGNMENT_COMPLETED = 2,
    TC_ABSOLUTE_ALIGNMENT_NOT_SUPPORTED = 3, /* Encoder sensor without "Z" output signal */
    TC_ABSOLUTE_ALIGNMENT_SUPPORTED = 4,
    TC_ALIGNMENT_ERROR = 5,
} align_status_t;

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  Position Control handle definition
 */
typedef struct {
    float movement_duration;
    float starting_angle;
    float final_angle;
    float angle_step;
    float sub_step[6];
    float sub_step_duration; /* Sub step time duration of sequence : acceleration / Cruise / Deceleration */
    float elapse_time;
    float sampling_time;
    float jerk;
    float cruise_speed;
    float acceleration;
    float omega;
    float omega_prev;
    float theta;
    float theta_prev;
    uint8_t received_th;
    bool position_control_regulation;
    bool encoder_absolute_aligned;
    int16_t mec_angle_offset;
    uint32_t tc_tick;
    float sys_tick_period;

    pos_ctrl_status_t position_ctrl_status;
    align_status_t alignment_cfg;
    align_status_t alignment_status;

    encoder_t* ptr_enc;
    speedn_torq_ctrl_t* ptr_stc;
    pid_integer_t* pid_pos_regulator;
} pos_ctrl_t;

void tc_init(pos_ctrl_t* phandle, pid_integer_t* ptr_pid_pos_reg, speedn_torq_ctrl_t* ptr_stc, encoder_t* ptr_enc);
bool tc_move_command(pos_ctrl_t* phandle, float starting_angle, float angle_step, float movement_duration);
void tc_follow_command(pos_ctrl_t* phandle, float Angle);
void tc_position_regulation(pos_ctrl_t* phandle);
void tc_move_execution(pos_ctrl_t* phandle);
void tc_follow_execution(pos_ctrl_t* phandle);
void tc_enc_alignment_command(pos_ctrl_t* phandle);
bool tc_ramp_completed(pos_ctrl_t* phandle);
void tc_encoder_reset(pos_ctrl_t* phandle);
float tc_get_current_position(pos_ctrl_t* phandle);
float tc_get_target_position(pos_ctrl_t* phandle);
float tc_get_move_duration(pos_ctrl_t* phandle);
pos_ctrl_status_t tc_get_control_position_status(pos_ctrl_t* phandle);
align_status_t tc_get_alignment_status(pos_ctrl_t* phandle);
void tc_inc_tick(pos_ctrl_t* phandle);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __TRAJCTRL_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
