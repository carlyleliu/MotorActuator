#ifndef __OPEN_LOOP_CONTROLLER_HPP__
#define __OPEN_LOOP_CONTROLLER_HPP__

#include <component_port.hpp>
#include <algorithm_utils.hpp>
#include <foc_controller.hpp>
#include <absolute_encoder.hpp>

#include <cmath>
#include <utility>

#include <time_util.h>

#include <driver.h>
#include <syslog.h>

using float2D = std::pair<float, float>;

class OpenLoopController
{
  public:
    OpenLoopController() :
        align_mode_(false),
        phase_target_(0.0f),
        phase_velocity_target_(0.0f),
        i_dq_target_({0.0f, 0.0f}),
        v_dq_target_({0.0f, 0.0f})
        {};
    ~OpenLoopController();

    /* set OpenLoopController config param */
    void SetMaxCurrentRamp(float max_current) {
        max_current_ramp_ = max_current;
    };
    void SetMaxVoltageRamp(float max_voltage) {
        max_voltage_ramp_ = max_voltage;
    };
    void SetMaxPhaseVelRamp(float max_phase_vel) {
        max_phase_vel_ramp_ = max_phase_vel;
    };

    void SetVelocityTarget(float vel) {
        velocity_target_ = vel;
    };
    void SetCurrentTarget(float current) {
        current_target_ = current;
    };
    void SetVoltageTarget(float voltage) {
        voltage_target_ = voltage;
    };
    void SetInitialPhase(float phase) {
        initial_phase_ = phase;
    };
    void SetAlignMode(bool enable) {
        align_mode_ = enable;
    };

    /* get OpenLoopController config param */
    float GetMaxCurrentRamp(void) {
        return max_current_ramp_;
    };
    float GetMaxVoltageRamp(void) {
        return max_voltage_ramp_;
    };
    float GetMaxPhaseVelRamp(void) {
        return max_phase_vel_ramp_;
    };
    float GetVelocityTarget(void) {
        return velocity_target_;
    };
    float GetCurrentTarget(void) {
        return current_target_;
    };
    float GetVoltageTarget(void) {
        return voltage_target_;
    };
    float GetInitialPhase(void) {
        return initial_phase_;
    };
    bool GetAlignMode(void) {
        return align_mode_;
    };

    /* export function interface */
    void Update(void);
    float GetTargetPhase(void) {
        return phase_target_;
    };
    float GetTargetPhaseVelocity(void) {
        return phase_velocity_target_;
    };
    std::optional<float2D> GetIdqTarget(void) {
        return i_dq_target_;
    };
    std::optional<float2D> GetVdqTarget(void) {
        return v_dq_target_;
    };

  private:
    /* Config */
    bool align_mode_;
    float max_current_ramp_;    // [A/s]
    float max_voltage_ramp_;    // [V/s]
    float max_phase_vel_ramp_;  // [rad/s]

    /* Inputs */
    float velocity_target_;
    float current_target_;
    float voltage_target_;
    float initial_phase_;

    /* Output */
    float phase_target_;
    float phase_velocity_target_;
    std::optional<float2D> i_dq_target_;
    std::optional<float2D> v_dq_target_;
};

#endif // ! __OPEN_LOOP_CONTROLLER_HPP__
