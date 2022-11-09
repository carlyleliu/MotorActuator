/**
 * @file fusion_ahrs_t.h
 * @author Seb Madgwick
 * @brief The AHRS sensor fusion algorithm to combines gyroscope, accelerometer,
 * and magnetometer measurements into a single measurement of orientation
 * relative to the Earth (NWU convention).
 *
 * The algorithm behaviour is governed by a gain.  A low gain will decrease the
 * influence of the accelerometer and magnetometer so that the algorithm will
 * better reject disturbances causes by translational motion and temporary
 * magnetic distortions.  However, a low gain will also increase the risk of
 * drift due to gyroscope calibration errors.  A typical gain value suitable for
 * most applications is 0.5.
 *
 * The algorithm allows the application to define a minimum and maximum valid
 * magnetic field magnitude.  The algorithm will ignore magnetic measurements
 * that fall outside of this range.  This allows the algorithm to reject
 * magnetic measurements that do not represent the direction of magnetic North.
 * The typical magnitude of the Earth's magnetic field is between 20 uT and
 * 70 uT.
 *
 * The algorithm can be used without a magnetometer.  Measurements of
 * orientation obtained using only gyroscope and accelerometer measurements
 * can be expected to drift in the yaw component of orientation only.  The
 * application can reset the drift in yaw by setting the yaw to a specified
 * angle at any time.
 *
 * The algorithm provides the measurement of orientation as a quaternion.  The
 * library includes functions for converting this quaternion to a rotation
 * matrix and Euler angles.
 *
 * The algorithm also provides a measurement of linear acceleration and Earth
 * acceleration.  Linear acceleration is equal to the accelerometer  measurement
 * with the 1 g of gravity removed.  Earth acceleration is a measurement of
 * linear acceleration in the Earth coordinate frame.
 */

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

//------------------------------------------------------------------------------
// Includes

#include <stdbool.h>

#include "fusion_types.h"

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief AHRS algorithm structure.  Structure members are used internally and
 * should not be used by the user application.
 */
typedef struct {
    float gain;
    float minimum_magnetic_field_squared;
    float maximum_magnetic_field_squared;
    fusion_quaternion_t quaternion; // describes the Earth relative to the sensor
    fusion_vector3_t linear_acceleration;
    float ramped_gain;
    bool zero_yaw_pending;
} fusion_ahrs_t;

//------------------------------------------------------------------------------
// Function prototypes

void fusion_ahrs_initialise(fusion_ahrs_t* const fusion_ahrs_t, const float gain);
void fusion_ahrs_set_gain(fusion_ahrs_t* const fusion_ahrs_t, const float gain);
void fusion_ahrs_set_magnetic_field(fusion_ahrs_t* const fusion_ahrs_t, const float minimumMagneticField, const float maximumMagneticField);
void fusion_ahrs_update(fusion_ahrs_t* const fusion_ahrs_t,
const fusion_vector3_t gyroscope,
const fusion_vector3_t accelerometer,
const fusion_vector3_t magnetometer,
const float sample_period);
void fusion_ahrs_update_without_magnetometer(fusion_ahrs_t* const fusion_ahrs_t,
const fusion_vector3_t gyroscope,
const fusion_vector3_t accelerometer,
const float sample_period);
fusion_quaternion_t fusion_ahrs_get_quaternion(const fusion_ahrs_t* const fusion_ahrs_t);
fusion_vector3_t fusion_ahrs_getlinear_acceleration(const fusion_ahrs_t* const fusion_ahrs_t);
fusion_vector3_t fusion_ahrs_get_earth_acceleration(const fusion_ahrs_t* const fusion_ahrs_t);
void fusion_ahrs_reinitialise(fusion_ahrs_t* const fusion_ahrs_t);
bool fusion_ahrs_is_initialising(const fusion_ahrs_t* const fusion_ahrs_t);
void fusion_ahrs_set_yaw(fusion_ahrs_t* const fusion_ahrs_t, const float yaw);

#endif

//------------------------------------------------------------------------------
// End of file
