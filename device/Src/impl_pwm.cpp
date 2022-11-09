#include <impl_pwm.hpp>

/* define pwm channel num */
constexpr uint8_t kPwmDeviceNum = 4;

/**
 * @brief init pwm device struct
 * @param idx channel num
 */
int ImplPwm::Init(uint8_t idx)
{
    return -1;
}

/** @brief Update pwm device
 *  @param None
 *  @return None
 */
int ImplPwm::Update(void)
{
    uint32_t pwm_pulse = 0;

    std::optional<float> normalize_pwm = pwm_input_port_.GetAlways();

    if (normalize_pwm.has_value()) {
        LOG_DBG("pwm_pulse[%d]\n", pwm_pulse);
    }

    return 0;
}

/** @brief Update pwm device
 *  @param None
 *  @return None
 */
void ImplPwm::Test(void)
{
}

