#include <impl_pwm.hpp>

/* tim1 & tim8 */
static uint16_t kTimxChannelMap[2][6] = {
    {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_5, TIM_CHANNEL_6},
    {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_5, TIM_CHANNEL_6}
};

/**
 * @brief init pwm device struct
 * @param idx channel num
 */
int ImplPwm::Init(uint8_t idx)
{
    tim_handle_ = &htim1;
    channel_ = kTimxChannelMap[0][idx];
    peripord_ = 4250; //(170000000 / 20000 / 2)

    return -1;
}

/** @brief Update pwm device
 *  @param None
 *  @return None
 */
int ImplPwm::Update(void)
{
    uint32_t pwm_pulse = 0;

    pwm_pulse = pwm_input_ * peripord_;

    __HAL_TIM_SetCompare(tim_handle_, channel_, pwm_pulse);

    LOG_DBG("pwm_pulse[%d]\n", pwm_pulse);

    return 0;
}

/** @brief Update pwm device
 *  @param None
 *  @return None
 */
void ImplPwm::Test(void)
{
}

