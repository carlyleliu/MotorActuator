#ifndef __DEVICE_PWM_DEVICE_HPP__
#define __DEVICE_PWM_DEVICE_HPP__

#include <driver.h>

#include <observer.hpp>
#include <component_port.hpp>

using namespace DesignedPatterns;

class ImplPwm : public Observer
{
  public:
    ImplPwm() {};
    ~ImplPwm() {};
    int Init(uint8_t idx);
    int Update(void);
    void Test(void);

  public:
    InputPort<float> pwm_input_port_;
};

#endif // ! __DEVICE_PWM_DEVICE_HPP__
