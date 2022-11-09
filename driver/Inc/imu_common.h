#ifndef __DRIVERS_IMU_COMMON_H__
#define __DRIVERS_IMU_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "util.h"

/* imu raw data */
typedef struct imu_raw {
    float acc_raw[3];                 /* 3 axis acc raw data */
    float gyro_raw[3];                /* 3 axis gyro raw data */
    float acc_offset[3];              /* accel calibration data. static offset correction */
    float acc_scale[3];               /* accel calibration data. scale error correction */
    float gyro_offset[3];             /* gyro calibration data. static offset correct */
    float temperature;                /* sensor temperature value */
    volatile uint32_t time_stamp;     /* this data time stamp */
    uint8_t gyro_has_calibration : 1; /* flag gyro has calibration */
    uint8_t acc_has_calibration : 1;  /* flag accel has calibration */
    uint8_t accel_err : 1;            /* flag of accel read error */
    uint8_t gyro_err : 1;             /* flag of gyro read error */
} imu_raw_t;

#ifdef __cplusplus
}
#endif

#endif // ! __DRIVERS_IMU_COMMON_H__
