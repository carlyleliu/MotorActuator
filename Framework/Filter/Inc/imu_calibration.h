#ifndef DEVicE_IMU_calibration_H_
#define DEVicE_IMU_calibration_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <math.h>
#include <stdbool.h>

#include "ahrs.h"
#include "main.h"

#define calibration_GYRO_COUNT     5000
#define TRY_calibration_GYRO_COUNT 10
#define GYRO_MAX_ERR               5

typedef enum {
    DetectOrentationTailDown,
    DetectOrentationNoseDown,
    DetectOrentationLeft,
    DetectOrentationRight,
    DetectOrentationUpsideDown,
    DetectOrentationRightsideUp,
    DetectOrentationError,
    DetectOrentationNull,
} detect_orientation_t;

/* imu calibration data */
typedef struct imu_calibration {
    int8_t gyro_step;
    int8_t acc_step;
    uint8_t is_calibrationing;
    uint8_t gyro_finished;
    uint8_t accel_finished;
    imu_raw_t* imu_raw;
} imu_calibration_t;

/* calibration data struct */
typedef struct calibrationHandle {
    imu_calibration_t* imu_cal;
} calibrationHandle_t;

void calibration(void);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* DEVicE_IMU_calibration_H_ */
