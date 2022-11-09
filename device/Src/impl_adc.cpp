#include <impl_adc.hpp>

/**
 * @brief Init adc value
 *
 */
int ImplAdc::Init(void)
{
    int err = 0;

    return err;
}

/**
 * @brief Update adc value
 *
 */
int ImplAdc::Update(void)
{
    int err = 0;

    time_ = time();

    value_ = raw_value_.adc_16bit_value_[0] / GeRresolution() * \
                    GetReferenceValue() * GetDividerRatio();

    LOG_INF("ADC value = %d\n", raw_value_.adc_16bit_value_[0]);

    return err;
}
