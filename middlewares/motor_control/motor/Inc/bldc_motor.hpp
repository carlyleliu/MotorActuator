#ifndef __MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__

#include <motor_abstract.hpp>
#include <foc_controller.hpp>
#include <open_loop_controller.hpp>

enum BldcMotorType
{
    BLDC_ACIM = 0,
    BLDC_GIMBAL,
    BLDC_HIGH_CURRENT,
};

class BldcMotor : public MotorAbstract
{
  public:
    BldcMotor();
    ~BldcMotor();

    void MotorStart(void) final;
    void MotorStop(void) final;
    void MotorTask(void) final;
    void MotorRun(void) final;

    PidController& GetVelocityPidHandler(void) {
        return velocity_pid_;
    };
    PidController& GetPositionPidHandler(void) {
        return position_pid_;
    };

    PidController& GetDAxisCurrentPidHandler(void) {
        return current_d_axis_pid_controller_;
    };
    PidController& GetQAxisCurrentPidHandler(void) {
        return current_q_axis_pid_controller_;
    };

  public:
    bool IsEnableCurrentControl(void) {
        return enable_current_control_;
    };

  private:
    int FocControl(void);
    void TorqueControl(void);
    void SpeedControl(void);
    void PositionControl(void);

  private:
    enum BldcMotorType bldc_type_;
    bool enable_current_control_;

    std::optional<float2D> v_dq_target_;
    std::optional<float2D> i_dq_target_;

    FieldOrientedController foc_;

    /* s */
    PidController current_d_axis_pid_controller_;
    PidController current_q_axis_pid_controller_;

    PidController velocity_pid_;
    PidController position_pid_;

    bool r_wl_ff_enable;
    bool bemf_ff_enable_;
};


#endif // !__MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
