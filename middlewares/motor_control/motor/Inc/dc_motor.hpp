#ifndef __MIDDLEWARE_MOTOR_CONTROL_DC_MOTOR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_DC_MOTOR_HPP__

#include <motor_abstract.hpp>

class WheeledChassisAbstract;

class DcMotor : public MotorAbstract
{
  public:
    DcMotor();
    ~DcMotor();
    void MotorStart(void) final;
    void MotorStop(void) final;
    void MotorTask(void) final;
    void MotorRun(void) final;

  private:
    void ExecuteVelocityControl(void);

  private:
    PidController pid_controller_;
  public:
    float positive_pwm_;
    float negative_pwm_;
};

#endif // ! __MIDDLEWARE_MOTOR_CONTROL_DC_MOTOR_HPP__
