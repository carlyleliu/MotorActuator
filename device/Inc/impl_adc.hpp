
#ifndef __DEVICE_IMPL_ADC_HPP__
#define __DEVICE_IMPL_ADC_HPP__

#include <driver.h>

#include <time_util.h>

union AdcValue
{
    uint16_t adc_16bit_value_[2];
    uint32_t adc_32bit_value_;
};

/* vbus temp current */
class ImplAdc
{
  public:
    ImplAdc() :
        time_(0.0f),
        adc_resolution_(12) {};
    ~ImplAdc();

    void SetDividerRatio(float ratio) { divider_ratio_ = ratio; };
    float GetDividerRatio(void) { return divider_ratio_; };

    void SetReferenceValue(float value) { adc_reference_value_ = value; };
    float GetReferenceValue(void) { return adc_reference_value_; };

    void SetResolution(uint16_t resolution) { adc_resolution_ = resolution; };
    uint16_t GeRresolution(void) { return adc_resolution_; };

    void SetAdcChannel(uint16_t channel) { adc_channel_ = channel; }

    float GetChannelValue(void) { return value_; };

    float GetTime(void) { return time_; };

    int Update(void);
    int Init(void);

  private:
    float time_;
    uint16_t adc_channel_;
    uint16_t adc_resolution_;
    float divider_ratio_;
    float adc_reference_value_;

    AdcValue raw_value_;
    float value_;
};

#endif // ! __DEVICE_IMPL_ADC_HPP__
