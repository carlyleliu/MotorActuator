#include <bldc_motor.hpp>
#include <algorithm_utils.hpp>

BldcMotor::BldcMotor() :
    MotorAbstract(),
    bldc_type_(BLDC_GIMBAL),
    enable_current_control_(false),
    v_dq_target_({0.0f, 0.0f}),
    i_dq_target_({0.0f, 0.0f}),
    r_wl_ff_enable(false),
    bemf_ff_enable_(false)
{
}

BldcMotor::~BldcMotor()
{
    MotorStop();
}

void BldcMotor::MotorStart(void)
{

}

void BldcMotor::MotorStop(void)
{
    pwm_phase_u_ = 0;
    pwm_phase_v_ = 0;
    pwm_phase_w_ = 0;
}

void BldcMotor::MotorRun(void)
{
    FocControl();
}

void BldcMotor::MotorTask(void)
{
    if (motor_control_type_ >= MOTOR_CONTROL_TYPE_POSITION) {
        PositionControl();
    }
    if (motor_control_type_ >= MOTOR_CONTROL_TYPE_SPEES) {
        SpeedControl();
    }
    if (motor_control_type_ >= MOTOR_CONTROL_TYPE_TORQUE && bldc_type_ != BLDC_GIMBAL) {
        TorqueControl();
    }

    MotorRun();
}


/**
 * @brief torque close loop control
 *
 */
void BldcMotor::PositionControl(void)
{
    float position_err = motor_controller_conf_.target_position_ - position_measure_;

    position_err = std::clamp(position_err, \
                -motor_controller_conf_.max_position_ramp_, motor_controller_conf_.max_position_ramp_);

    motor_controller_conf_.target_velocity_ = position_pid_.PIController(position_err);

    motor_controller_conf_.target_velocity_ = std::clamp(motor_controller_conf_.target_velocity_, \
                                                -motor_conf_.max_velocity_, motor_conf_.max_velocity_);
}

/**
 * @brief torque close loop control
 *
 */
void BldcMotor::SpeedControl(void)
{
    velocity_estimate_ = velocity_measure_;

    if (velocity_measure_ > motor_controller_conf_.velocity_limit_tolerance_ * motor_conf_.max_velocity_) {
        motor_status_.over_velocity_ = true;
        return;
    }

    float velocity_err = motor_controller_conf_.target_velocity_ - velocity_estimate_;

    velocity_err = std::clamp(velocity_err, \
                    -motor_controller_conf_.max_velocity_ramp_, motor_controller_conf_.max_velocity_ramp_);

    if (bldc_type_ == BLDC_GIMBAL) {
        float vd = 0.0f;
        float vq = 0.0f;
        vq = velocity_pid_.PIDController(velocity_err);
        vq = std::clamp(vq, -motor_conf_.nominal_voltage_ / 2, motor_conf_.nominal_voltage_ / 2);
        v_dq_target_ = {vd, vq};
    } else {
        motor_controller_conf_.target_torque_ = velocity_pid_.PIController(velocity_err);
        motor_controller_conf_.target_torque_ = std::clamp(motor_controller_conf_.target_torque_,\
                                             -motor_conf_.stal_torque_, motor_conf_.stal_torque_);
    }

    LOG_INF("velocity = %f %f, torque = %f velocity_err = %f\n", \
             velocity_measure_, velocity_estimate_, motor_controller_conf_.target_torque_, velocity_err);
}

/**
 * @brief torque close loop control
 *
 */
void BldcMotor::TorqueControl(void)
{
    float torque = motor_controller_conf_.target_torque_;

    // Load target from previous iteration.
    float id = i_dq_target_[0];
    float iq = i_dq_target_[1];

    // 1% space reserved for Iq to avoid numerical issues
    id = std::clamp(id, -motor_conf_.stal_current_ * 0.99f, motor_conf_.stal_current_ * 0.99f);

    // Convert requested torque to current
    iq = torque / motor_conf_.torque_constant_;

    // 2-norm clamping where Id takes priority
    float iq_lim_sqr = SQ(motor_conf_.stal_current_) - SQ(id);
    float iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr);
    iq = std::clamp(iq, -iq_lim, iq_lim);

    if (bldc_type_ != BLDC_GIMBAL) {
        i_dq_target_ = {id, iq};
    }

    float vd = 0.0f;
    float vq = 0.0f;

    if (r_wl_ff_enable) {

        vd -= velocity_measure_ * motor_conf_.phase_inductance_ * iq;
        vq += velocity_measure_ * motor_conf_.phase_inductance_ * id;
        vd += motor_conf_.phase_resistance_ * id;
        vq += motor_conf_.phase_resistance_ * iq;
    }

    if (bemf_ff_enable_) {
        vq += velocity_measure_ * (2.0f / 3.0f) * (motor_conf_.torque_constant_ / motor_conf_.pole_pairs_);
    }

    if (bldc_type_ == BLDC_GIMBAL) {
        // reinterpret current as voltage
        v_dq_target_ = {vd + id, vq + iq};
    } else {
        v_dq_target_ = {vd, vq};
    }

    LOG_INF("vd = %f vq = %f\n", vd + id, vq + iq);
}

/**
 * @brief current controller update
 *
 */
int BldcMotor::FocControl(void)
{
    std::array<float, 2> v_dq;

    control_time_ = time();
    //float predict_theta = phase + velocity_measure_ * (control_time_ - sensor_update_time_);
    float predict_theta = normalize_angle_measure_ * motor_conf_.pole_pairs_;

    float mod_to_v = (2.0f / 3.0f) * motor_controller_conf_.vbus_measured_;
    float v_to_mod = 1.0f / mod_to_v;

    float mod_d, mod_q;

    float v_d = v_dq_target_[0];
    float v_q = v_dq_target_[1];

    if (IsEnableCurrentControl()) {
        foc_.FocClark(current_measure_);
        foc_.FocPark(predict_theta);

        std::array<float, 2>& i_dq_measure = foc_.GetIqdMeasure();

        float id_target = i_dq_target_[0];
        float iq_target = i_dq_target_[1];

        float id_measure = i_dq_measure[0];
        float iq_measure = i_dq_measure[1];

        float i_err_d = id_target - id_measure;
        float i_err_q = iq_target - iq_measure;

        mod_d = v_to_mod * (v_d + current_d_axis_pid_controller_.PIController(i_err_d));
        mod_q = v_to_mod * (v_q + current_q_axis_pid_controller_.PIController(i_err_q));

        float mod_scalefactor = 0.80f * kSqrt3Div2_ * 1.0f / std::sqrt(mod_d * mod_d + mod_q * mod_q);

        if (mod_scalefactor < 1.0f) {
            mod_d *= mod_scalefactor;
            mod_q *= mod_scalefactor;
        }
    } else {
        mod_d = v_to_mod * v_d;
        mod_q = v_to_mod * v_q;
    }

    v_dq = {
        mod_d,
        mod_q
    };

    LOG_DBG("mod_to_v = %f v_dq[%f %f]\n", mod_to_v, mod_d, mod_q);

    foc_.FocRevPark(v_dq, predict_theta);

    bool success = foc_.FocSVM(&pwm_phase_u_, &pwm_phase_v_, &pwm_phase_w_);

    if (!success) {
        LOG_ERR("foc exec failed pwm[%f %f %f]\n", pwm_phase_u_, pwm_phase_v_, pwm_phase_w_);
        pwm_phase_u_ = 0;
        pwm_phase_v_ = 0;
        pwm_phase_w_ = 0;
    }

    return 0;
}
