#ifndef __DEVICE_ABSOLUTE_ENCODER_HPP__
#define __DEVICE_ABSOLUTE_ENCODER_HPP__

#include <angle_encoder_abstract.hpp>

class AbsoluteAngleEncoder : public AngleEncoderAbstract
{
  public:
    AbsoluteAngleEncoder() {};
    ~AbsoluteAngleEncoder() {};

    int ImplInit(void) final;
    int ImplDeInit(void) final;
    int ImplCalibration(void) final;
    uint16_t ImplGetAbsoluteAngle(void) final;

  private:
    uint16_t angle_;
};

#endif // ! __DEVICE_ABSOLUTE_ENCODER_HPP__
