#ifndef __MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__

#include <driver.h>

#include <algorithm>

#include <component_port.hpp>
#include <pid_controller.hpp>
#include <foc_controller.hpp>
#include <absolute_encoder.hpp>

#include <fsm.hpp>

class PidController;

enum MotorType
{
    MOTOR_TYPE_DC = 0,
    MOTOR_TYPE_BLDC,
    MOTOR_TYPE_SERVO,
    MOTOR_TYPE_STEP,
    MOTOR_TYPE_NUM,
};

enum MotorControlType
{
    MOTOR_CONTROL_TYPE_IDLE = 0,
    MOTOR_CONTROL_TYPE_OL,
    MOTOR_CONTROL_TYPE_TORQUE,
    MOTOR_CONTROL_TYPE_SPEES,
    MOTOR_CONTROL_TYPE_POSITION,
    MOTOR_CONTROL_TYPE_NUM,
};

enum MotorControlEvent
{
    MOTOR_CONTROL_EVENT_SET_IDLE = MOTOR_CONTROL_TYPE_NUM + 1,
    MOTOR_CONTROL_EVENT_SET_OL,
    MOTOR_CONTROL_EVENT_SET_SPEES,
    MOTOR_CONTROL_EVENT_SET_TORQUE,
    MOTOR_CONTROL_EVENT_SET_POSITION,
    MOTOR_CONTROL_EVENT_NUM,
};

/* motor const parameter */
typedef struct MotorConfig
{
    uint8_t pole_pairs_;

    /* torque param */
    float torque_constant_;     // N.M/A
    float nominal_torque_;      // N.M
    float stal_torque_;         // N.M

    /* velocity param */
    float velocity_constant_;   //rpm/V
    float nominal_velocity_;    //rpm
    float max_velocity_;        //rpm

    /* voltage resistance inductance */
    float phase_inductance_;    //mH
    float phase_resistance_;    //o
    float nominal_voltage_;     //V

    /* current */
    float nominal_current_;     //A
    float stal_current_;        //A

    /* rotor and tempersture */
    float rotor_inertia_;       //fcm^2
    float max_demagnetize_tempersture_;
} MotorConfig_t;

/* motor status */
typedef struct MotorStatus
{
    uint32_t fault_code_ : 16;

    uint32_t over_velocity_ : 1;
    uint32_t over_current_ : 1;
    uint32_t over_temperature_ : 1;
} MotorStatus_t;

/* motor control config */
typedef struct MotorControllerConfig
{
    float max_angle_ramp_;
    float max_torque_ramp_;
    float max_velocity_ramp_;
    float max_position_ramp_;
    float velocity_limit_tolerance_;

    float target_angle_;
    float target_torque_;
    float target_velocity_;
    float target_position_;

    float actual_angle_;
    float actual_torque_;
    float actual_velocity_;
    float actual_position_;
    float vbus_measured_;
} MotorControllerConfig_t;

class MotorAbstract
{
  public:
    MotorAbstract():
        control_time_(0.0f),
        position_estimate_(0.0f),
        velocity_estimate_(0.1f),
        normalize_angle_estimate_(0.0f),
        pwm_phase_u_(0.0f),
        pwm_phase_v_(0.0f),
        pwm_phase_w_(0.0f)
        {};
    virtual ~MotorAbstract() {};

  public:
    enum MotorType GetType(void) {
        return motor_type_;
    };
    enum MotorControlType GetControlType(void) {
        return motor_control_type_;
    };

    const float& GetMeasureAngle(void) {
        return motor_controller_conf_.actual_angle_;
    };
    const float& GetMeasureTorque(void) {
        return motor_controller_conf_.actual_torque_;
    };
    const float& GetMeasureVelocity(void) {
        return motor_controller_conf_.actual_velocity_;
    };
    const float& GetMeasurePosition(void) {
        return motor_controller_conf_.actual_position_;
    };

    const float& GetExpectAngle(void) {
        return motor_controller_conf_.target_angle_;
    };
    const float& GetExpectTorque(void) {
        return motor_controller_conf_.target_torque_;
    };
    const float& GetExpectVelocity(void) {
        return motor_controller_conf_.target_velocity_;
    };
    const float& GetExpectPosition(void) {
        return motor_controller_conf_.target_position_;
    };

    const uint32_t GetFaultCode(void) {
        return motor_status_.fault_code_;
    };

    void SetControlType(enum MotorControlType type) {
        motor_control_type_ = type;
    };

    void SetAngle(float angle) {
        motor_controller_conf_.target_angle_ = angle;
    };
    void SetTorque(float torque) {
        motor_controller_conf_.target_torque_ = \
        std::clamp(torque, -motor_conf_.stal_torque_, motor_conf_.stal_torque_);
    };
    void SetVelocity(float velocity) {
        motor_controller_conf_.target_velocity_ = \
        std::clamp(velocity, -motor_conf_.max_velocity_, motor_conf_.max_velocity_);
    };
    void SetPosition(float position) {
        motor_controller_conf_.target_position_ = position;
    };

    float GetVBusMeasured(void) {
        return motor_controller_conf_.vbus_measured_;
    };
    void SetVBusMeasured(float vbus) {
        motor_controller_conf_.vbus_measured_ = vbus;
    };

    MotorConfig_t& GetConfig(void) {
        return motor_conf_;
    };
    const MotorStatus_t& GetStatus(void) const {
        return motor_status_;
    };
    MotorControllerConfig_t& GetControllerConfig(void) {
        return motor_controller_conf_;
    };

    void SendMotorControlEvent(enum MotorControlEvent event) {
        fsm_.Event(event);
    };

  public:
    virtual void MotorStart(void) = 0;
    virtual void MotorStop(void) = 0;
    virtual void MotorRun(void) = 0;
    virtual void MotorTask(void) = 0;

  protected:
    enum MotorType motor_type_;
    enum MotorControlType motor_control_type_;

    float control_time_;

    MotorConfig_t motor_conf_;
    MotorStatus_t motor_status_;
    MotorControllerConfig_t motor_controller_conf_;

    float position_estimate_;
    float velocity_estimate_;
    float normalize_angle_estimate_;

    Fsm::StateMachine fsm_;

  public:
    /* Inputs */
    InputPort<float> position_measure_;
    InputPort<float> velocity_measure_;
    InputPort<float> normalize_angle_measure_;

    InputPort<float> sensor_update_time_;
    InputPort<float2D> current_measure_;

    /* Output */
    OutputPort<float> pwm_phase_u_;
    OutputPort<float> pwm_phase_v_;
    OutputPort<float> pwm_phase_w_;
};

#endif // !__MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__
