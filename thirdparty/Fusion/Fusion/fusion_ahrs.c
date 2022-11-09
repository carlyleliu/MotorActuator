/**
 * @file fusion_ahrs_t.c
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

//------------------------------------------------------------------------------
// Includes

#include "fusion_ahrs.h"

#include <float.h> // FLT_MAX
#include <math.h>  // atan2f, cosf, sinf

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Initial gain used during the initialisation period.  The gain used by
 * each algorithm iteration will ramp down from this initial gain to the
 * specified algorithm gain over the initialisation period.
 */
#define INITIAL_GAIN (10.0f)

/**
 * @brief Initialisation period (in seconds).
 */
#define INITIALISATION_PERIOD (3.0f)

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Initialises the AHRS algorithm structure.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @param gain AHRS algorithm gain.
 */
void fusion_ahrs_initialise(fusion_ahrs_t* const fusion_ahrs_t, const float gain)
{
    fusion_ahrs_t->gain = gain;
    fusion_ahrs_t->minimum_magnetic_field_squared = 0.0f;
    fusion_ahrs_t->maximum_magnetic_field_squared = FLT_MAX;
    fusion_ahrs_t->quaternion = FUSION_QUATERNION_IDENTITY;
    fusion_ahrs_t->linear_acceleration = FUSION_VECTOR3_ZERO;
    fusion_ahrs_t->ramped_gain = INITIAL_GAIN;
    fusion_ahrs_t->zero_yaw_pending = false;
}

/**
 * @brief Sets the AHRS algorithm gain.  The gain must be equal or greater than
 * zero.
 * @param gain AHRS algorithm gain.
 */
void fusion_ahrs_set_gain(fusion_ahrs_t* const fusion_ahrs_t, const float gain)
{
    fusion_ahrs_t->gain = gain;
}

/**
 * @brief Sets the minimum and maximum valid magnetic field magnitudes in uT.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @param minimumMagneticField Minimum valid magnetic field magnitude.
 * @param maximumMagneticField Maximum valid magnetic field magnitude.
 */
void fusion_ahrs_set_magnetic_field(fusion_ahrs_t* const fusion_ahrs_t, const float minimumMagneticField, const float maximumMagneticField)
{
    fusion_ahrs_t->minimum_magnetic_field_squared = minimumMagneticField * minimumMagneticField;
    fusion_ahrs_t->maximum_magnetic_field_squared = maximumMagneticField * maximumMagneticField;
}

/**
 * @brief Updates the AHRS algorithm.  This function should be called for each
 * new gyroscope measurement.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param magnetometer Magnetometer measurement in uT.
 * @param sample_period Sample period in seconds.  This is the difference in time
 * between the current and previous gyroscope measurements.
 */
void fusion_ahrs_update(fusion_ahrs_t* const fusion_ahrs_t,
const fusion_vector3_t gyroscope,
const fusion_vector3_t accelerometer,
const fusion_vector3_t magnetometer,
const float sample_period)
{
#define Q fusion_ahrs_t->quaternion.element // define shorthand label for more readable code

    // Calculate feedback error
    fusion_vector3_t halfFeedbackError = FUSION_VECTOR3_ZERO; // scaled by 0.5 to avoid repeated multiplications by 2
    do {
        // Abandon feedback calculation if accelerometer measurement invalid
        if ((accelerometer.axis.x == 0.0f) && (accelerometer.axis.y == 0.0f) && (accelerometer.axis.z == 0.0f)) {
            break;
        }

        // Calculate direction of gravity assumed by quaternion
        const fusion_vector3_t halfGravity = {
        .axis.x = Q.x * Q.z - Q.w * Q.y,
        .axis.y = Q.w * Q.x + Q.y * Q.z,
        .axis.z = Q.w * Q.w - 0.5f + Q.z * Q.z,
        }; // equal to 3rd column of rotation matrix representation scaled by 0.5

        // Calculate accelerometer feedback error
        halfFeedbackError = fusion_vector_cross_product(fusion_vector_fast_normalise(accelerometer), halfGravity);

        // Abandon magnetometer feedback calculation if magnetometer measurement invalid
        const float magnetometerMagnitudeSquared = fusion_vector_magnitude_squared(magnetometer);
        if ((magnetometerMagnitudeSquared < fusion_ahrs_t->minimum_magnetic_field_squared)
            || (magnetometerMagnitudeSquared > fusion_ahrs_t->maximum_magnetic_field_squared)) {
            break;
        }

        // Compute direction of 'magnetic west' assumed by quaternion
        const fusion_vector3_t halfWest = {.axis.x = Q.x * Q.y + Q.w * Q.z,
        .axis.y = Q.w * Q.w - 0.5f + Q.y * Q.y,
        .axis.z = Q.y * Q.z - Q.w * Q.x}; // equal to 2nd column of rotation matrix representation scaled by 0.5

        // Calculate magnetometer feedback error
        halfFeedbackError = fusion_vector_add(halfFeedbackError,
        fusion_vector_cross_product(fusion_vector_fast_normalise(fusion_vector_cross_product(accelerometer, magnetometer)), halfWest));

    } while (false);

    // Ramp down gain until initialisation complete
    if (fusion_ahrs_t->gain == 0) {
        fusion_ahrs_t->ramped_gain = 0; // skip initialisation if gain is zero
    }
    float feedbackGain = fusion_ahrs_t->gain;
    if (fusion_ahrs_t->ramped_gain > fusion_ahrs_t->gain) {
        fusion_ahrs_t->ramped_gain -= (INITIAL_GAIN - fusion_ahrs_t->gain) * sample_period / INITIALISATION_PERIOD;
        feedbackGain = fusion_ahrs_t->ramped_gain;
    }

    // Convert gyroscope to radians per second scaled by 0.5
    fusion_vector3_t halfGyroscope = fusion_vector_multiply_scalar(gyroscope, 0.5f * fusion_degrees_to_radians(1.0f));

    // Apply feedback to gyroscope
    halfGyroscope = fusion_vector_add(halfGyroscope, fusion_vector_multiply_scalar(halfFeedbackError, feedbackGain));

    // Integrate rate of change of quaternion
    fusion_ahrs_t->quaternion = fusion_quaternion_add(fusion_ahrs_t->quaternion,
    fusion_quaternion_multiply_vector(fusion_ahrs_t->quaternion, fusion_vector_multiply_scalar(halfGyroscope, sample_period)));

    // Normalise quaternion
    fusion_ahrs_t->quaternion = fusion_quaternion_fast_normalise(fusion_ahrs_t->quaternion);

    // Calculate linear acceleration
    const fusion_vector3_t gravity = {
    .axis.x = 2.0f * (Q.x * Q.z - Q.w * Q.y),
    .axis.y = 2.0f * (Q.w * Q.x + Q.y * Q.z),
    .axis.z = 2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),
    }; // equal to 3rd column of rotation matrix representation
    fusion_ahrs_t->linear_acceleration = fusion_vector_subtract(accelerometer, gravity);

#undef Q // undefine shorthand label
}

/**
 * @brief Updates the AHRS algorithm.  This function should be called for each
 * new gyroscope measurement.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param sample_period Sample period in seconds.  This is the difference in time
 * between the current and previous gyroscope measurements.
 */
void fusion_ahrs_update_without_magnetometer(fusion_ahrs_t* const fusion_ahrs_t,
const fusion_vector3_t gyroscope,
const fusion_vector3_t accelerometer,
const float sample_period)
{
    // Update AHRS algorithm
    fusion_ahrs_update(fusion_ahrs_t, gyroscope, accelerometer, FUSION_VECTOR3_ZERO, sample_period);

    // Zero yaw once initialisation complete
    if (fusion_ahrs_is_initialising(fusion_ahrs_t) == true) {
        fusion_ahrs_t->zero_yaw_pending = true;
    } else {
        if (fusion_ahrs_t->zero_yaw_pending == true) {
            fusion_ahrs_set_yaw(fusion_ahrs_t, 0.0f);
            fusion_ahrs_t->zero_yaw_pending = false;
        }
    }
}

/**
 * @brief Gets the quaternion describing the sensor relative to the Earth.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @return Quaternion describing the sensor relative to the Earth.
 */
fusion_quaternion_t fusion_ahrs_get_quaternion(const fusion_ahrs_t* const fusion_ahrs_t)
{
    return fusion_quaternion_conjugate(fusion_ahrs_t->quaternion);
}

/**
 * @brief Gets the linear acceleration measurement equal to the accelerometer
 * measurement with the 1 g of gravity removed.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @return Linear acceleration measurement.
 */
fusion_vector3_t fusion_ahrs_getlinear_acceleration(const fusion_ahrs_t* const fusion_ahrs_t)
{
    return fusion_ahrs_t->linear_acceleration;
}

/**
 * @brief Gets the Earth acceleration measurement equal to linear acceleration
 * in the Earth coordinate frame.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @return Earth acceleration measurement.
 */
fusion_vector3_t fusion_ahrs_get_earth_acceleration(const fusion_ahrs_t* const fusion_ahrs_t)
{
#define Q fusion_ahrs_t->quaternion.element // define shorthand labels for more readable code
#define A fusion_ahrs_t->linear_acceleration.axis
    const float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    const fusion_vector3_t earthAcceleration = {
    .axis.x = 2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
    .axis.y = 2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
    .axis.z = 2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
    }; // transpose of a rotation matrix representation of the quaternion multiplied with the linear acceleration
    return earthAcceleration;
#undef Q // undefine shorthand label
#undef A
}

/**
 * @brief Reinitialise the AHRS algorithm.
 * @param fusion_ahrs_t AHRS algorithm structure.
 */
void fusion_ahrs_reinitialise(fusion_ahrs_t* const fusion_ahrs_t)
{
    fusion_ahrs_t->quaternion = FUSION_QUATERNION_IDENTITY;
    fusion_ahrs_t->linear_acceleration = FUSION_VECTOR3_ZERO;
    fusion_ahrs_t->ramped_gain = INITIAL_GAIN;
}

/**
 * @brief Returns true while the AHRS algorithm is initialising.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @return True while the AHRS algorithm is initialising.
 */
bool fusion_ahrs_is_initialising(const fusion_ahrs_t* const fusion_ahrs_t)
{
    return fusion_ahrs_t->ramped_gain > fusion_ahrs_t->gain;
}

/**
 * @brief Sets the yaw component of the orientation measurement provided by the
 * AHRS algorithm.  This function can be used to reset drift in yaw when the
 * AHRS algorithm is being used without a magnetometer.
 * @param fusion_ahrs_t AHRS algorithm structure.
 * @param yaw Yaw angle in degrees.
 */
void fusion_ahrs_set_yaw(fusion_ahrs_t* const fusion_ahrs_t, const float yaw)
{
#define Q fusion_ahrs_t->quaternion.element // define shorthand label for more readable code
    fusion_ahrs_t->quaternion = fusion_quaternion_normalise(
    fusion_ahrs_t->quaternion); // quaternion must be normalised accurately (approximation not sufficient)
    const float inverseYaw = atan2f(Q.x * Q.y + Q.w * Q.z, Q.w * Q.w - 0.5f + Q.x * Q.x); // Euler angle of conjugate
    const float halfInverseYawMinusOffset = 0.5f * (inverseYaw - fusion_degrees_to_radians(yaw));
    const fusion_quaternion_t inverseYawQuaternion = {
    .element.w = cosf(halfInverseYawMinusOffset),
    .element.x = 0.0f,
    .element.y = 0.0f,
    .element.z = -1.0f * sinf(halfInverseYawMinusOffset),
    };
    fusion_ahrs_t->quaternion = fusion_quaternion_multiply(inverseYawQuaternion, fusion_ahrs_t->quaternion);
#undef Q // undefine shorthand label
}

//------------------------------------------------------------------------------
// End of file
