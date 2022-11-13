#ifndef __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__
#define __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__

#include <driver.h>
#include <syslog.h>

#include <cmath>
#include <algorithm_utils.hpp>
#include <algorithm>

class AngleEncoderAbstract {
  public:
    AngleEncoderAbstract() :
        time_(0.0f),
        angle_measure_prev_(0),
        angle_measure_(0),
        angle_offset_(1900), //1180
        rotate_direction_(1),
        mechanical_to_phase_direction_(-1),
        circle_counter_(0),
        calibrationed_(0),
        inited_(0),
        aligned_(0),
        normalize_angle_measure_(0.0f),
        velocity_measure_(0.0f),
        sensor_update_time_(0.0f)
        {};
    virtual ~AngleEncoderAbstract() {};
    virtual int ImplInit(void) = 0;
    virtual int ImplDeInit(void) = 0;
    virtual int ImplCalibration(void) = 0;
    virtual uint16_t ImplGetAbsoluteAngle(void) = 0;

    int Init(void);
    int DeInit(void);
    int Align(void);
    int Calibration(void);
    int Update(void);
    int Notify(void);
    uint16_t GetOriginAngle(void) {
        return ImplGetAbsoluteAngle();
    };
    float GetTime(void) {
        return time_;
    };
    float GetNormalizeAngleMeasure(void) {
        return normalize_angle_measure_;
    };
    float GetVelocityMeasure(void) {
        return velocity_measure_;
    };
    float GetSensorUpdateTime(void) {
        return sensor_update_time_;
    };

  protected:
    float time_;

    int16_t angle_measure_prev_;
    int16_t angle_measure_;
    int16_t angle_offset_;
    int8_t rotate_direction_;
    int8_t mechanical_to_phase_direction_;

    int32_t circle_counter_;

    uint8_t calibrationed_ : 1;
    uint8_t inited_ : 1;
    uint8_t aligned_ : 1;

    float normalize_angle_measure_; // [rad]
    float velocity_measure_;        // [rad/s]
    float sensor_update_time_;      // [s]
};

#endif  // ! __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__
