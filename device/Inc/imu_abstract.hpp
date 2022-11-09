#ifndef __DEVICE_IMU_ABSTRACT_HPP__
#define __DEVICE_IMU_ABSTRACT_HPP__

#include <array>
#include <driver.h>
#include <time_util.h>

class ImuAbstract
{
  public:
    ImuAbstract();
    ~ImuAbstract();
    virtual int Init(void) = 0;
    virtual int DeInit(void) = 0;
    virtual int Read(void) = 0;

    int Update(void);
    int Filter(void);
    int Notify(void);
    int Calibration(void);

  protected:
    std::array<double, 3> accel_normalized_;          /* 3 axis acc normalized data */
    std::array<double, 3> gyro_normalized_;           /* 3 axis gyro normalized data */
    std::array<double, 3> magnetometer_normalized_;   /* 3 axis magnetometer normalized data */

    double accel_scale_;                               /* accel scale data */
    double gyro_scale_;                                /* gyro scale data */
    double magnetometer_scale_;                        /* magnetometer scale data */

    std::array<double, 3> accel_cal_offset_;          /* accel calibration data. static offset correction */
    std::array<double, 3> accel_cal_scale_;           /* accel calibration data. scale error correction */
    std::array<double, 3> gyro_cal_offset_;           /* gyro calibration data. static offset correct */

    double temperature_;                                /* sensor temperature value */
    float time_;                                        /* this data time stamp */

    uint8_t gyro_has_calibration_ : 1;   /* flag gyro has calibration */
    uint8_t accel_has_calibration_ : 1;  /* flag accel has calibration */
    uint8_t accel_err_ : 1;              /* flag of accel read error */
    uint8_t gyro_err_ : 1;               /* flag of gyro read error */
    uint8_t inited_ : 1;
};

#endif // ! __DEVICE_IMU_ABSTRACT_HPP__
