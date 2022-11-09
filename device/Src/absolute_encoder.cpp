#include <absolute_encoder.hpp>

/**
  * @brief absolute angle encoder Sensor ImplInit
  * @param None
  * @retval None
  */
int AbsoluteAngleEncoder::ImplInit(void)
{
    inited_ = 1;

    return 0;
}

/**
  * @brief absolute angle encoder Sensor ImplDeInit
  * @param None
  * @retval None
  */
int AbsoluteAngleEncoder::ImplDeInit(void)
{
    inited_ = 0;

    return 0;
}

/**
  * @brief absolute angle encoder Sensor ImplCalibration
  * @param None
  * @retval None
  */
int AbsoluteAngleEncoder::ImplCalibration(void)
{

    return 0;
}

/**
  * @brief get true angle from sensor
  * @param None
  * @retval None
  */
uint16_t AbsoluteAngleEncoder::ImplGetAbsoluteAngle(void)
{
    angle_ = Tle5012bReadAngle();

    return (uint16_t)angle_;
}


