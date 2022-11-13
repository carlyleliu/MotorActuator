#ifndef __FRAMEWORK_MOTOR_CONTROL_FACTORY_HPP__
#define __FRAMEWORK_MOTOR_CONTROL_FACTORY_HPP__

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <dc_motor.hpp>
#include <pid_controller.hpp>

class MotorFactory
{
  public:
    virtual BldcMotor& Create(void) = 0;

  protected:
    bool created;
    BldcMotor motor_;
};

class PM3505 : public MotorFactory
{
  public:
    PM3505();
    ~PM3505();
    BldcMotor& Create(void);
};

#endif
