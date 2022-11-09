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
    /* enter or exit state action */
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_OL, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_OL, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_SPEES, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_SPEES, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_TORQUE, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_TORQUE, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_POSITION, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_POSITION, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    /* idle state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_OL) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_SPEES) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_TORQUE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_POSITION) = [&](const Fsm::fsm_args &args) {

    };

    /* speed state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_IDLE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_OL) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_TORQUE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_POSITION) = [&](const Fsm::fsm_args &args) {

    };

    /* torque state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_TORQUE, MOTOR_CONTROL_EVENT_SET_IDLE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_TORQUE, MOTOR_CONTROL_EVENT_SET_OL) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_TORQUE, MOTOR_CONTROL_EVENT_SET_SPEES) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_TORQUE, MOTOR_CONTROL_EVENT_SET_POSITION) = [&](const Fsm::fsm_args &args) {

    };

    /* position state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_POSITION, MOTOR_CONTROL_EVENT_SET_IDLE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_POSITION, MOTOR_CONTROL_EVENT_SET_OL) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_POSITION, MOTOR_CONTROL_EVENT_SET_SPEES) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_POSITION, MOTOR_CONTROL_EVENT_SET_TORQUE) = [&](const Fsm::fsm_args &args) {

    };
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
    std::optional<float> position_measure = position_measure_.GetPresent();
    //std::optional<float> velocity_measure = velocity_measure_.GetPresent();

    float position_err = motor_controller_conf_.target_position_ - *position_measure;

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
    std::optional<float> velocity_measure = velocity_measure_.GetPresent().value_or(0);
    velocity_estimate_ = *velocity_measure;

    if (velocity_measure > motor_controller_conf_.velocity_limit_tolerance_ * motor_conf_.max_velocity_) {
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
        vq = std::clamp(vq, -motor_conf_.nominal_voltage_/2, motor_conf_.nominal_voltage_/2);
        v_dq_target_ = {vd, vq};
    } else {
        motor_controller_conf_.target_torque_ = velocity_pid_.PIController(velocity_err);
        motor_controller_conf_.target_torque_ = std::clamp(motor_controller_conf_.target_torque_,\
                                             -motor_conf_.stal_torque_, motor_conf_.stal_torque_);
    }

    LOG_INF("velocity = %f %f, torque = %f velocity_err = %f\n", \
             *velocity_measure, velocity_estimate_, motor_controller_conf_.target_torque_, velocity_err);
}

/**
 * @brief torque close loop control
 *
 */
void BldcMotor::TorqueControl(void)
{
    float torque = motor_controller_conf_.target_torque_;

    // Load target from previous iteration.
    auto [id, iq] = *i_dq_target_;

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

    std::optional<float> phase_vel = velocity_measure_.GetPresent().value_or(0) * motor_conf_.pole_pairs_;

    if (r_wl_ff_enable) {
        if (!phase_vel.has_value()) {
            return;
        }

        vd -= *phase_vel * motor_conf_.phase_inductance_ * iq;
        vq += *phase_vel * motor_conf_.phase_inductance_ * id;
        vd += motor_conf_.phase_resistance_ * id;
        vq += motor_conf_.phase_resistance_ * iq;
    }

    if (bemf_ff_enable_) {
        if (!phase_vel.has_value()) {
            return;
        }

        vq += *phase_vel * (2.0f / 3.0f) * (motor_conf_.torque_constant_ / motor_conf_.pole_pairs_);
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
    std::optional<float2D> v_dq;
    float phase = normalize_angle_measure_.GetPresent().value_or(0) * motor_conf_.pole_pairs_;
    //float phase_velocity = velocity_measure_.GetPresent().value_or(0) * motor_conf_.pole_pairs_;

    control_time_ = time();
    //float predict_theta = phase + phase_velocity * (control_time_ - *sensor_update_time_.GetPresent());
    float predict_theta = phase;

    float mod_to_v = (2.0f / 3.0f) * motor_controller_conf_.vbus_measured_;
    float v_to_mod = 1.0f / mod_to_v;

    float mod_d, mod_q;

    auto [v_d, v_q] = *v_dq_target_;

    if (IsEnableCurrentControl()) {
        foc_.FocClark(current_measure_.GetPresent());
        foc_.FocPark(predict_theta);

        std::optional<float2D> i_dq_measure = foc_.GetIqdMeasure();

        auto [id_target, iq_target] = *i_dq_target_;
        auto [id_measure, iq_measure] = *i_dq_measure;

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

    auto [tA, tB, tC, success] = foc_.FocSVM();

    if (success) {
        pwm_phase_u_ = tA;
        pwm_phase_v_ = tB;
        pwm_phase_w_ = tC;
    } else {
        LOG_ERR("foc exec failed pwm[%f %f %f]\n", tA, tB, tC);
    }

    return 0;
}
