#include "imu_calibration.h"

/* imu calibration instance */
static imu_calibration_t imu_calibration_instance = {
.gyro_step = 0,
.acc_step = 0,
.is_calibrationing = 0,
.gyro_finished = 0,
.accel_finished = 0,
.imu_raw = &kImuInstance,
};

/* all calibration instance */
__attribute__((unused)) static calibrationHandle_t calibration_instance = {
.imu_cal = &imu_calibration_instance,
};

/**
 * @brief calibration gyro data
 * @param imu_cal calibration struct
 * @retval None
 */
static void calibrationGyro(imu_calibration_t* imu_cal)
{
    float gyro_offset_sum[3] = {0.0f, 0.0f, 0.0f};
    static float offset_x[3] = {0.0f, 0.0f, 0.0f};
    static uint16_t calibration_count = 0;
    static uint8_t try_calibration_count = 0;
    float x_err, y_err, z_err;

    if (1 == imu_cal->imu_raw->gyro_has_calibration) {
        return;
    }

    switch (imu_cal->gyro_step) {
        case 0:
            calibration_count = 0;
            try_calibration_count++;
            gyro_offset_sum[0] = 0.0f;
            gyro_offset_sum[1] = 0.0f;
            gyro_offset_sum[2] = 0.0f;
            imu_cal->imu_raw->gyro_offset[0] = 0.0f;
            imu_cal->imu_raw->gyro_offset[1] = 0.0f;
            imu_cal->imu_raw->gyro_offset[2] = 0.0f;
            imu_cal->gyro_step++;
            imu_cal->is_calibrationing = 1;
            break;
        case 1:
            gyro_offset_sum[0] += imu_cal->imu_raw->gyro_raw[0];
            gyro_offset_sum[1] += imu_cal->imu_raw->gyro_raw[1];
            gyro_offset_sum[2] += imu_cal->imu_raw->gyro_raw[2];
            calibration_count++;
            if (calibration_count == calibration_GYRO_COUNT) {
                offset_x[0] = gyro_offset_sum[0] / (float)calibration_count;
                offset_x[1] = gyro_offset_sum[1] / (float)calibration_count;
                offset_x[2] = gyro_offset_sum[2] / (float)calibration_count;
                imu_cal->gyro_step++;
            }
            break;
        case 2:
            x_err = imu_cal->imu_raw->gyro_raw[0] - offset_x[0];
            y_err = imu_cal->imu_raw->gyro_raw[1] - offset_x[1];
            z_err = imu_cal->imu_raw->gyro_raw[2] - offset_x[2];
            if (!isfinite(offset_x[0]) || !isfinite(offset_x[1]) || !isfinite(offset_x[2])
                || fabsf(x_err) > GYRO_MAX_ERR || fabsf(y_err) > GYRO_MAX_ERR || fabsf(z_err) > GYRO_MAX_ERR) {
                imu_cal->gyro_step = 0;
                if (try_calibration_count > TRY_calibration_GYRO_COUNT) {
                    imu_cal->gyro_step = -1;
                    // ERR
                }
            } else {
                imu_cal->gyro_step++;
            }
            break;
        case 3:
            imu_cal->imu_raw->gyro_offset[0] = offset_x[0];
            imu_cal->imu_raw->gyro_offset[1] = offset_x[1];
            imu_cal->imu_raw->gyro_offset[2] = offset_x[2];
            imu_cal->gyro_finished = 1;
            imu_cal->imu_raw->gyro_has_calibration = 1;
            break;
        default:
            break;
    }

    if (-1 == imu_cal->gyro_step) {
        // ERR
        imu_cal->gyro_step = 0;
    }
}

/**
 * @brief calibration data
 * @param imu_cal calibration struct
 * @retval None
 */
void calibration(void)
{
    calibrationGyro(&imu_calibration_instance);
}
