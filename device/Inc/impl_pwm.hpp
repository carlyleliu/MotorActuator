#ifndef __DEVICE_PWM_DEVICE_HPP__
#define __DEVICE_PWM_DEVICE_HPP__

#include <driver.h>
class ImplPwm
{
  public:
    ImplPwm() {};
    ~ImplPwm() {};
    int Init(uint8_t idx);
    int Update(void);
    void Test(void);

  public:
    void SetPwmPlus(float pwm) {
        pwm_input_ = pwm;
    };

  private:
    TIM_HandleTypeDef* tim_handle_;
    float pwm_input_;
    uint16_t peripord_;
    uint16_t channel_;
};

#endif // ! __DEVICE_PWM_DEVICE_HPP__
