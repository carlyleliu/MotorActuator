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
        resolution_(15),
        sensor_range_(65536),
        bandwidth_(1000.0f),
        calibrationed_(0),
        inited_(0),
        aligned_(0),
        mechanical_to_phase_direction_(1),
        cpr_angle_offset_(1300), //1180
        circle_counts_(0),
        cpr_angle_measure_prev_(0),
        cpr_angle_measure_(0),
        angle_measure_(0.0f),
        velocity_measure_(0.0f),
        pll_kp_(0.0f),
        pll_ki_(0.0f),
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
    float GetNormalizeAngleMeasure(void) {
        return angle_measure_;
    };
    float GetVelocityMeasure(void) {
        return velocity_measure_;
    };
    float GetTotalAngleMeasure(void) {
        return position_measure_;
    };
    float GetSensorUpdateTime(void) {
        return sensor_update_time_;
    };
    void SetOffset(int16_t offset) {
        cpr_angle_offset_ = offset;
    };

  protected:
    float time_;

    uint16_t resolution_;       // 15 bit for tle5012b
    float sensor_range_;        // 65536  for tle5012b
    float bandwidth_;

    uint8_t calibrationed_ : 1;
    uint8_t inited_ : 1;
    uint8_t aligned_ : 1;
    int8_t mechanical_to_phase_direction_;

    uint16_t cpr_angle_offset_;
    int32_t circle_counts_;

    /* raw angle velocity and position */
    uint16_t cpr_angle_measure_prev_;        // [count]
    uint16_t cpr_angle_measure_;             // [count]
    int32_t cpr_position_measure_;          // [count]

    /* interpolation */
    float interpolation_;

    /* estimate cpr angle */
    float cpr_angle_estimate_;       // [count]
    float cpr_velocity_estimate_;    // [count/s]
    float cpr_position_estimate_;    // [count]

    /* output raw angle velocity and pos */
    float angle_measure_;           // [rad]
    float velocity_measure_;        // [rad/s]
    float position_measure_;        // [rad]

    /* output estimate angle velocity and pos */
    float angle_estimate_;          // [rad]
    float velocity_estimate_;       // [rad/s]
    float position_estimate_;       // [rad]

    float pll_kp_;           // [count/s / count]
    float pll_ki_;           // [(count/s^2) / count]

    float sensor_update_time_;      // [s]
};

#endif  // ! __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__
