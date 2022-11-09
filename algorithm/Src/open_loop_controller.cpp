#include <open_loop_controller.hpp>


/**
 * @brief  This function is open loop foc control task
 * @param  timestamp current time
 * @retval None
 */
void OpenLoopController::Update(void)
{
    auto [prev_id, prev_iq] = *i_dq_target_;
    auto [prev_vd, prev_vq] = *v_dq_target_;

    (void)prev_iq; // unused
    (void)prev_vq; // unused

    float dt = 0.001;

    i_dq_target_ = {
        std::clamp(current_target_, prev_id - max_current_ramp_ * dt, prev_id + max_current_ramp_ * dt),
        0.0f
    };
    v_dq_target_ = {
        0.0f,
        std::clamp(voltage_target_, prev_vq - max_voltage_ramp_ * dt, prev_vq + max_voltage_ramp_ * dt)
    };

    if (!align_mode_) {
        phase_velocity_target_ = std::clamp(velocity_target_, phase_velocity_target_ - max_phase_vel_ramp_ * dt, \
                                            phase_velocity_target_ + max_phase_vel_ramp_ * dt);
        phase_target_ = WrapPmPi(phase_target_ + phase_velocity_target_ * dt);
    } else {
        phase_velocity_target_ = 0;
        phase_target_ = -1.5707963f;
    }

    LOG_DBG("iqd[%f %f] vqd[%f %f] phase_target_[%f] phase_vel[%f]\n", \
            prev_id, prev_iq, prev_vd, prev_vq, phase_target_, phase_velocity_target_);
}
