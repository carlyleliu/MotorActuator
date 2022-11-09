#ifndef DEVicE_ahrs_H_
#define DEVicE_ahrs_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <math.h>

#include "common.h"
#include "icm20602.h"
#include "imu_calibration.h"
#include "imu_common.h"
#include "systicks.h"

/* kalman param */
typedef struct kalman_param {
    /* self param */
    const unsigned int kC_0;
    float q_bias;
    float angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    float P[2][2];

    /* adjust parameter */
    float Q_angle;
    float Q_gyro;
    float R_angle;
} kalman_param_t;

/**
 * @brief handle of a mahony attitude estimation component
 */
typedef struct mahony_param {
    float euler_angle[3];     /**< Roll-Pitch -Yaw angle in rad */
    float euler_angle_deg[3]; /**< Roll-Pitch -Yaw angle in degress */
    float tbn[3][3];          /**< body coordinates to navigation coordinates rotation matrix */
    float sample_freq;        /**< sample frequency in Hz */
    float sample_t;           /**< the sample period in s*/
    float half_t;             /**< half the sample period in s*/
    float q[4];               /**< quaternion elements representing orientation */
    float ex_int;             /**< scaled integral error in x-axis*/
    float ey_int;             /**< scaled integral error in y-axis*/
    float ez_int;             /**< scaled integral error in z-axis*/
    float kp;                 /**< proportional gain */
    float ki;                 /**< integral gain */
} mahony_param_t;

/**
 * @brief ahrs algorithm struct
 */
typedef struct ahrs {
    imu_raw_t* imu_raw;            /* pointer of imu raw data */
    mahony_param_t* mahony_handle; /* pointer of mahony param */
    kalman_param_t* kalman_handle; /* pointer of kalman param */
    float euler_angle[3];          /* euler angle with radian */
    float euler_by_acc[3];         /* erler angle with accel  */
    float euler_angle_deg[3];      /* euler angle with degree */
    float dt;                      /* time difference between two operations */
    uint32_t current_ticks;
    uint32_t last_ticks;
} ahrs_t;

/* export variable */
extern imu_raw_t kImuInstance;

void kalman_filter(kalman_param_t* phandle, float acc_angle, float gyro_rate, float* angle, float* gyro_rate_filter, float dt);
int mahony_init(mahony_param_t* phandle, float ax, float ay, float az);
void mahony_update(mahony_param_t* phandle, float ax, float ay, float az, float gx, float gy, float gz);
void ahrs_update(void);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* DEVicE_ahrs_H_ */
