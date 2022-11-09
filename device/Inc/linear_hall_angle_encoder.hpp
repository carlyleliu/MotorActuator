#ifndef __MIDDLEWARE_SENSOR_LINEAR_HALL_ANGLE_ENCODER_HPP__
#define __MIDDLEWARE_SENSOR_LINEAR_HALL_ANGLE_ENCODER_HPP__

#include <angle_encoder_abstract.hpp>

constexpr int16_t kCalibrationTimes_ = 5000;
constexpr int16_t kHallAdcFilterDepth_ = 1;

typedef struct LinearHallSensorCalibration
{
    uint16_t cal_step_;
    uint16_t cal_mode_;

    float min_hall_a_;
    float min_hall_b_;

    float max_hall_a_;
    float max_hall_b_;

    float hall_middle_a_;
    float hall_middle_b_;

    float hall_amplitude_a_;
    float hall_amplitude_b_;

} LinearHallSensorCalibration_t;

union DualAdcValue
{
    uint16_t single_hall_adc_value_[2];
    uint32_t dual_hall_adc_value_;
};

class LinearHallAngleEncoder : public AngleEncoderAbstract
{
  public:
    LinearHallAngleEncoder(){};
    ~LinearHallAngleEncoder();

  public:
    int ImplInit(void) final;
    int ImplDeInit(void) final;
    int ImplCalibration(void) final;
    uint16_t ImplGetAbsoluteAngle(void) final;

  private:
    void SearchMinMax(float* max, float* min, const float value);
    void LinearHallSensorReadValue(void);
    uint16_t LinearHallSensorGetAngle(void);
    void LinearHallSensorCalibrationPeak(void);
    void LinearHallSensorCalibration(void);

  private:
    volatile float hall_raw_a_;
    volatile float hall_raw_b_;
    union DualAdcValue hall_adc_value_[kHallAdcFilterDepth_];
    LinearHallSensorCalibration_t hall_sensor_calibration_;
};

#endif // ! __MIDDLEWARE_SENSOR_LINEAR_HALL_ANGLE_ENCODER_HPP__
