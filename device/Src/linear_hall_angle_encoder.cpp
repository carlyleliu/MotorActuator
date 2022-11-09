#include <linear_hall_angle_encoder.hpp>

/**
  * @brief Hall Sensor ImplInit
  * @param None
  * @retval None
  */
int LinearHallAngleEncoder::ImplInit(void)
{
    hall_sensor_calibration_.min_hall_a_ = 161;
    hall_sensor_calibration_.min_hall_b_ = 161.2;
    hall_sensor_calibration_.max_hall_a_ = 4092;
    hall_sensor_calibration_.max_hall_b_ = 4094.2;
    hall_sensor_calibration_.hall_middle_a_ = 2126.5;
    hall_sensor_calibration_.hall_middle_b_ = 2127.8;
    hall_sensor_calibration_.hall_amplitude_a_ = 1965.5;
    hall_sensor_calibration_.hall_amplitude_b_ = 1966.7;

    inited_ = 1;

    return 0;
}

/**
  * @brief Hall Sensor ImplDeInit
  * @param None
  * @retval None
  */
int LinearHallAngleEncoder::ImplDeInit(void)
{
    hall_sensor_calibration_.min_hall_a_ = 0;
    hall_sensor_calibration_.min_hall_b_ = 0;
    hall_sensor_calibration_.max_hall_a_ = 0;
    hall_sensor_calibration_.max_hall_b_ = 0;
    hall_sensor_calibration_.hall_middle_a_ = 0;
    hall_sensor_calibration_.hall_middle_b_ = 0;
    hall_sensor_calibration_.hall_amplitude_a_ = 10;
    hall_sensor_calibration_.hall_amplitude_b_ = 0;

    inited_ = 0;

    return 0;
}

/**
  * @brief Hall Sensor read absolute angle
  * @param None
  * @retval None
  */
uint16_t LinearHallAngleEncoder::ImplGetAbsoluteAngle(void)
{
    return LinearHallSensorGetAngle();
}

/**
  * @brief Hall Sensor ImplCalibration
  * @param None
  * @retval None
  */
int LinearHallAngleEncoder::ImplCalibration(void)
{
    LinearHallSensorCalibration();

    return 0;
}

/**
  * @brief search max and min value
  * @param None
  * @retval None
  */
void LinearHallAngleEncoder::SearchMinMax(float* max, float* min, const float value)
{
	if(value < *min) {
		*min = value;
	}
	if(value > *max) {
		*max = value;
	}
}

/**
  * @brief Hall Sensor read adc value
  * @param None
  * @retval None
  */
void LinearHallAngleEncoder::LinearHallSensorReadValue(void)
{
	int32_t sum[2] = {0, 0};

    for (int8_t idx = 0; idx < kHallAdcFilterDepth_; idx++) {
        sum[0] += hall_adc_value_[idx].single_hall_adc_value_[0];
        sum[1] += hall_adc_value_[idx].single_hall_adc_value_[1];
    }

    hall_raw_a_ = (float)sum[0] / kHallAdcFilterDepth_;
    hall_raw_b_ = (float)sum[1] / kHallAdcFilterDepth_;
}

/**
  * @brief Hall Sensor read angle
  * @param None
  * @retval None
  */
uint16_t LinearHallAngleEncoder::LinearHallSensorGetAngle(void)
{
    float hall_sin = 0.0f, hall_cos = 0.0f;

    LinearHallSensorReadValue();

    hall_sin = (hall_raw_a_ - hall_sensor_calibration_.hall_middle_a_) / \
              hall_sensor_calibration_.hall_amplitude_a_;
    hall_cos = (hall_raw_b_ - hall_sensor_calibration_.hall_middle_b_) / \
              hall_sensor_calibration_.hall_amplitude_b_;

    std::clamp(hall_sin, -1.0f, 1.0f);
    std::clamp(hall_cos, -1.0f, 1.0f);

    return atan2f(hall_sin, hall_cos);
}

/**
  * @brief Hall Sensor calibration scale
  * @param None
  * @retval None
  */
void LinearHallAngleEncoder::LinearHallSensorCalibrationPeak(void)
{
    LinearHallSensorReadValue();

    SearchMinMax(&hall_sensor_calibration_.max_hall_a_, \
                 &hall_sensor_calibration_.min_hall_a_, \
                 hall_raw_a_);
    SearchMinMax(&hall_sensor_calibration_.max_hall_b_, \
                 &hall_sensor_calibration_.min_hall_b_, \
                 hall_raw_b_);

    hall_sensor_calibration_.hall_middle_a_ = \
        (hall_sensor_calibration_.max_hall_a_ + hall_sensor_calibration_.min_hall_a_) / 2.0f;
    hall_sensor_calibration_.hall_middle_b_ = \
        (hall_sensor_calibration_.max_hall_b_ + hall_sensor_calibration_.min_hall_b_) / 2.0f;

    hall_sensor_calibration_.hall_amplitude_a_ = \
        (hall_sensor_calibration_.max_hall_a_ - hall_sensor_calibration_.min_hall_a_) / 2.0f;
    hall_sensor_calibration_.hall_amplitude_b_ = \
        (hall_sensor_calibration_.max_hall_b_ - hall_sensor_calibration_.min_hall_b_) / 2.0f;
}

/**
  * @brief Hall Sensor calibration
  * @param None
  * @retval None
  */
void LinearHallAngleEncoder::LinearHallSensorCalibration(void)
{
    for (int idx = 0; idx < kCalibrationTimes_; idx++) {
        LinearHallSensorCalibrationPeak();
    }
}
