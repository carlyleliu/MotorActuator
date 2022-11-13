#include <angle_encoder_abstract.hpp>

/**
* @brief  Init sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Init(void)
{
    return ImplInit();
}

/**
* @brief  DeInit sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::DeInit(void)
{
    return ImplDeInit();
}

/**
* @brief  Calibration sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Calibration(void)
{
    ImplCalibration();

    return 0;
}

/**
* @brief  align sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Align(void)
{
    return ImplGetAbsoluteAngle();
}

/**
* @brief  Update the rotor electrical angle integrating the last measured
*         instantaneous electrical speed express in dpp.
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Update(void)
{
    if (!inited_) {
        return -1;
    }

    float current_time = time();

    if (0 == time_) {
        time_ = current_time;
    }

    float dt = current_time - time_;

    if (1 == mechanical_to_phase_direction_)
        angle_measure_ = (int16_t)(ImplGetAbsoluteAngle() - angle_offset_);
    else
        angle_measure_ = (int16_t)(angle_offset_ - ImplGetAbsoluteAngle());

    int16_t delta_mechanical_angle = angle_measure_ - angle_measure_prev_;

	if (delta_mechanical_angle < -32768) {
		circle_counter_++;
	} else if ( delta_mechanical_angle > 32767 ) {
		circle_counter_--;
	}

    rotate_direction_ = (delta_mechanical_angle > 0) ? 1 : -1;

    time_ = current_time;

    normalize_angle_measure_ = (angle_measure_ / 65536.0) * 2 * kPI_;
    velocity_measure_ = (delta_mechanical_angle / 65536.0) * 2 * kPI_ / dt;
    sensor_update_time_ = current_time;

    angle_measure_prev_ = angle_measure_;

    LOG_INF("ori_angle = %d angle = %f velocity = %f time = %f\n", angle_measure_, (angle_measure_ / 65536.0) * 2 * kPI_, (delta_mechanical_angle / 65536.0) * 2 * kPI_ / dt, current_time);

    return 0;
}

/**
* @brief  sensor update notify to app
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Notify(void)
{
    return 0;
}
