#include <angle_encoder_abstract.hpp>

/**
* @brief  Init sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Init(void)
{
    pll_kp_ = 2.0f * bandwidth_;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped
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
    float current_time = time();

    if (0 == sensor_update_time_) {
        sensor_update_time_ = current_time;
    }

    float dt = current_time - sensor_update_time_;

    /* get raw angle form abs sensor */
    if (1 == mechanical_to_phase_direction_)
        cpr_angle_measure_ = (uint16_t)(ImplGetAbsoluteAngle() - cpr_angle_offset_);
    else
        cpr_angle_measure_ = (uint16_t)(cpr_angle_offset_ - ImplGetAbsoluteAngle());

    /* cal circle counter */
    int32_t delta_raw_angle = (int32_t)cpr_angle_measure_ - cpr_angle_measure_prev_;

    if (delta_raw_angle > (int32_t)sensor_range_ / 2) {
        circle_counts_--;
        delta_raw_angle -= sensor_range_;
    } else if (delta_raw_angle < -(int32_t)(sensor_range_ / 2)) {
        circle_counts_++;
        delta_raw_angle += sensor_range_;
    }

    /* cal position from abs sensor */
    cpr_position_measure_ = (int32_t)(circle_counts_ * sensor_range_) + cpr_angle_measure_;

    /* Predict current angle and position */
    cpr_position_estimate_ += cpr_velocity_estimate_ * dt;
    cpr_angle_estimate_ += cpr_velocity_estimate_ * dt;
    cpr_angle_estimate_ = Mod(cpr_angle_estimate_, sensor_range_);

    /* cal delat angle with measure and estimate */
    auto encoder_floor = [this](float internal_position)->int32_t {
        return (int32_t)std::floor(internal_position);
    };

    /* discrete phase detector */
    float delta_cpr_position = (float)(cpr_position_measure_ - encoder_floor(cpr_position_estimate_));
    float delta_cpr_angle = (float)(cpr_angle_measure_ - encoder_floor(cpr_angle_estimate_));
    delta_cpr_angle = WrapPm(delta_cpr_angle, sensor_range_);

    /* pll feedback */
    cpr_position_estimate_ += dt * pll_kp_ * delta_cpr_position;
    cpr_angle_estimate_ += dt * pll_kp_ * delta_cpr_angle;
    cpr_angle_estimate_ = FmodfPos(cpr_angle_estimate_, sensor_range_);

    cpr_velocity_estimate_ += dt * pll_ki_ * delta_cpr_angle;
    bool snap_to_zero_vel = false;
    if (std::abs(cpr_velocity_estimate_) < 0.5f * dt * pll_ki_) {
        cpr_velocity_estimate_ = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    /* update angle velocity and position estimate */
    angle_estimate_ = cpr_angle_estimate_ / sensor_range_ * kTwoPI_;
    velocity_estimate_ = cpr_velocity_estimate_ / sensor_range_ * kTwoPI_;
    position_estimate_ = cpr_position_estimate_ / sensor_range_ * kTwoPI_;

    /* run encoder count interpolation */
    if (snap_to_zero_vel) {
        interpolation_ = 0.5f;
    } else if (delta_cpr_angle > 0) {
        interpolation_ = 0.0f;
    } else if (delta_cpr_angle < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += dt * cpr_velocity_estimate_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    //float interpolated_angle = cpr_angle_measure_ + interpolation_;

    /* output measure angle velocity and position */
    angle_measure_ = ((int16_t)cpr_angle_measure_ / sensor_range_) * kTwoPI_;
    float vel = -(delta_raw_angle / sensor_range_) * kTwoPI_ * 10000;
    velocity_measure_ = vel;
    position_measure_ = (cpr_position_measure_ / sensor_range_) * kTwoPI_;

    /* update time */
    sensor_update_time_ = current_time;

    /* update cpr angle measure prev */
    cpr_angle_measure_prev_ = cpr_angle_measure_;

    //LOG_ERR("%d\n", (int)velocity_measure_);

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
