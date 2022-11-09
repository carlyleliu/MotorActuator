
/* Includes ------------------------------------------------------------------*/
#include "mc_position.h"

#include "stdlib.h"

/**
 * @brief  Position init function
 * @param  angle_ref: Target angle
 * @retval error angle
 */
void position_init(position_t* pHandl, abs_encoder_t* abs_enc)
{
    pHandl->ptr_abs_encoder = abs_enc;
    pHandl->target_pos = 0;
    pHandl->error_pos = 0;
    pHandl->target_rpm = 0;
    pHandl->current_rpm = 0;
    pHandl->mode_flag = 0;
    pHandl->last_pos_ticks = 0;
}

/**
 * @brief  Return error angle unit in rad*10000
 * @param  angle_ref: Target angle
 * @retval error angle
 */
inline int32_t position_get_error_angle(position_t* phandle)
{
    int32_t angle;

    /* Measure angle = circle_counter * 2PI * 10000 + mec_angle_dpp / 65536 * 2PI * 10000 */
    angle = (int32_t)(phandle->ptr_abs_encoder->circle_counter) * PI_MUL
            + (int32_t)(phandle->ptr_abs_encoder->mec_angle_dpp) / 65536.0f * PI_MUL;

    phandle->current_pos = angle;

    /* Compute angle error */
    phandle->error_pos = phandle->target_pos - phandle->current_pos;

    /* If enter target range of position, then change to Torque mode */
    if (abs(phandle->error_pos) < CHANGE_LIMIT_LOW) {
        phandle->mode_flag = P_SPEED_MODE; // P_TORQUE_MODE;
    }
    /* Else if motor run in speed mode */
    else if (abs(phandle->error_pos) > CHANGE_LIMIT_HIGH) {
        phandle->mode_flag = P_SPEED_MODE; // P_SPEED_MODE

        phandle->torque_first_flag = 0;
    }

    return phandle->error_pos;
}

/**
 * @brief  Return error angle unit in rad*10000
 * @param  angle_ref: Target angle
 * @retval error angle
 */
inline void position_get_error_angle_by_imu(position_t* phandle)
{
    /* Compute angle error */

    phandle->target_pos = 0;

    phandle->error_pos = phandle->target_pos - phandle->current_pos_imu;

    /* If enter target range of position, then change to Torque mode */
    if (abs(phandle->error_pos) < CHANGE_LIMIT_LOW) {
        phandle->mode_flag = P_SPEED_MODE; // P_TORQUE_MODE;
    }
    /* Else if motor run in speed mode */
    else if (abs(phandle->error_pos) > CHANGE_LIMIT_HIGH) {
        phandle->mode_flag = P_SPEED_MODE; // P_SPEED_MODE

        phandle->torque_first_flag = 0;
    }
}

/**
 * @brief  set imu pos
 * @param  angle_ref: Target angle
 * @retval error angle
 */
void position_set_imu_angle(position_t* phandle, int32_t current_angle)
{
    phandle->current_pos_imu = current_angle;

    if (phandle->current_pos_imu > MAX_POSITION / 2) {
        phandle->current_pos_imu = MAX_POSITION / 2;
    }
    if (phandle->current_pos_imu < -MAX_POSITION / 2) {
        phandle->current_pos_imu = -MAX_POSITION / 2;
    }
}

/**
 * @brief  It computes the new values for max speed
 * @param  None
 * @retval Speed referrence
 */
inline int16_t position_calc_speed_referrence(position_t* phandle)
{
    int32_t speed_reference = 0;

    phandle->ptr_abs_encoder->mec_pos = phandle->ptr_abs_encoder->circle_counter * 65536 + phandle->ptr_abs_encoder->mec_angle_dpp;

    phandle->current_rpm =
    (int32_t)((phandle->ptr_abs_encoder->mec_pos - phandle->ptr_abs_encoder->last_mec_pos) / 65536.0f * 120000.0f);

    phandle->ptr_abs_encoder->_Super.avr_mecspeed_unit = phandle->current_rpm_filter;

    phandle->ptr_abs_encoder->last_mec_pos = phandle->ptr_abs_encoder->mec_pos;

    speed_reference = phandle->current_rpm_filter;

    /* Limit max referrence speed */
    if (speed_reference > motor_MAX_SPEED_RPM) {
        speed_reference = motor_MAX_SPEED_RPM;
    } else if (speed_reference < -motor_MAX_SPEED_RPM) {
        speed_reference = -motor_MAX_SPEED_RPM;
    }

    return (int16_t)speed_reference;
}

/**
 * @brief  Set target position
 * @param  None
 * @retval None
 */
void position_set_target_pos(position_t* phandle, int32_t angle_ref)
{
    phandle->target_pos = angle_ref;

    if (phandle->target_pos > MAX_POSITION) {
        phandle->target_pos = MAX_POSITION;
    }
    if (phandle->target_pos < MIN_POSITION) {
        phandle->target_pos = MIN_POSITION;
    }
}

/**
 * @brief  Set target speed
 * @param  None
 * @retval None
 */
void position_set_target_speed(position_t* phandle, int32_t speed_ref)
{
    phandle->target_rpm = speed_ref;

    if (phandle->target_rpm > MAX_SPEED_RPM) {
        phandle->target_rpm = MAX_SPEED_RPM;
    }
    if (phandle->target_rpm < MIN_SPEED_RPM) {
        phandle->target_rpm = MIN_SPEED_RPM;
    }
}
