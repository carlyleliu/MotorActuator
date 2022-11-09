/**
 * @file ahrs_deprecated.hpp
 * @author carlyleliu (yyliushuai@gmail.com)
 * @brief this file is old c program for imu filter(for backup)
 * @version 0.1
 * @date 2022-10-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __ALGORITHM_AHRS_DEPRECATED_HPP__
#define __ALGORITHM_AHRS_DEPRECATED_HPP__

#include <algorithm_utils.hpp>

#include <driver.h>
#include <cmath>

#define CALIBRATION_GYRO_COUNT			5000
#define TRY_CALIBRATION_GYRO_COUNT	    10
#define GYRO_MAX_ERR					5

typedef enum  {
	DetectOrentationTailDown,
	DetectOrentationNoseDown,
	DetectOrentationLeft,
	DetectOrentationRight,
	DetectOrentationUpsideDown,
	DetectOrentationRightsideUp,
	DetectOrentationError,
	DetectOrentationNull,
} DetectOrientationTypeDef;

/* imu raw data */
typedef struct ImuRaw {
	float acc_raw[3];  		        /* 3 axis acc raw data */
	float gyro_raw[3]; 		        /* 3 axis gyro raw data */
    float unified_acc_raw[3];       /* 3 axis acc raw data with unified */
    float unified_gyro_raw[3];      /* 3 axis gyro raw data with unified */
	float acc_offset[3];	        /* accel calibration data. static offset correction */
	float acc_scale[3];		        /* accel calibration data. scale error correction */
	float gyro_offset[3];	        /* gyro calibration data. static offset correct */
	float temperature;		        /* sensor temperature value */
	volatile uint32_t time_stamp;	/* this data time stamp */
	uint8_t gyro_has_calibration:1;	/* flag gyro has calibration */
	uint8_t acc_has_calibration:1;	/* flag accel has calibration */
	uint8_t accel_err:1;		    /* flag of accel read error */
	uint8_t gyro_err:1;		        /* flag of gyro read error */
	int8_t axis_rotation[3][3];
} ImuRaw_t;

/* function pointer for imu data update */
typedef void (*Update)(ImuRaw_t * imu);

/* imu calibration data */
typedef struct ImuCalibration
{
	int8_t gyro_step;
	int8_t acc_step;
	uint8_t is_calibrationing;
	uint8_t gyro_finished;
	uint8_t accel_finished;
	ImuRaw_t* imu_raw;
}ImuCalibration_t;

/* calibration data struct */
typedef struct CalibrationHandle
{
	ImuCalibration_t *imu_cal;
} Calibration_t;

/* kalman param */
typedef struct KalmanParam
{
	/* self param */
	unsigned int kC_0;
	float q_bias;
	float angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	float P[2][2];
	/* adjust parameter */
	float Q_angle;
	float Q_gyro;
	float R_angle;
} KalmanParam_t;

/**
  * @brief handle of a mahony attitude estimation component
  */
typedef struct MahonyParam
{
	float euler_angle[3];        /**< Roll-Pitch -Yaw angle in rad */
	float euler_angle_deg[3];    /**< Roll-Pitch -Yaw angle in degress */
	float tbn[3][3];             /**< body coordinates to navigation coordinates rotation matrix */
	float sample_freq;           /**< sample frequency in Hz */
	float sample_t;              /**< the sample period in s*/
	float half_t;                /**< half the sample period in s*/
	float q[4];                  /**< quaternion elements representing orientation */
	float ex_int;                /**< scaled integral error in x-axis*/
	float ey_int;                /**< scaled integral error in y-axis*/
	float ez_int;                /**< scaled integral error in z-axis*/
	float kp;                    /**< proportional gain */
	float ki;                    /**< integral gain */
} MahonyParam_t;

/**
  * @brief ahrs algorithm struct
  */
typedef struct AHRS
{
	ImuRaw_t *imu_raw;			        /* pointer of imu raw data */
	MahonyParam_t *mahony_handle;		/* pointer of mahony param */
	KalmanParam_t *kalman_handle;		/* pointer of kalman param */
	float euler_angle[3];				/* euler angle with radian */
	float euler_angle_deg[3];		    /* euler angle with degree */
	float dt;							/* time difference between two operations */
	uint32_t current_ticks;
	uint32_t last_ticks;
    Update ImuUpdate;
} AHRS_t;

/* export variable */
extern ImuRaw_t kImuInstance;
extern AHRS_t ahrs_instance;

void KalmanFilter(KalmanParam_t *phandle, float acc_angle, float gyro_rate, float* angle, float* gyro_rate_filter, float dt);
int MahonyInit(MahonyParam_t *phandle,float ax,float ay,float az);
void MahonyUpdate(MahonyParam_t *phandle,float ax,float ay,float az,float gx,float gy,float gz);
void AHRSUpdate(void);
void AHRSCalibration(void);
void Calibration(void);

#endif // ! __ALGORITHM_AHRS_DEPRECATED_HPP__

