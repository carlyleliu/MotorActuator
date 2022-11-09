/**
 * @file fusion_bias_t.c
 * @author Seb Madgwick
 * @brief The gyroscope bias correction algorithm achieves run-time calibration
 * of the gyroscope bias.  The algorithm will detect when the gyroscope is
 * stationary for a set period of time and then begin to sample gyroscope
 * measurements to calculate the bias as an average.
 */

//------------------------------------------------------------------------------
// Includes

#include "fusion_bias.h"

#include "math.h" // fabs

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Minimum stationary period (in seconds) after which the the algorithm
 * becomes active and begins sampling gyroscope measurements.
 */
#define STATIONARY_PERIOD (5.0f)

/**
 * @brief Corner frequency (in Hz) of the high-pass filter used to sample the
 * gyroscope bias.
 */
#define CORNER_FREQUENCY (0.02f)

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Initialises the gyroscope bias correction algorithm.
 * @param fusion_bias_t fusion_bias_t structure.
 * @param threshold Gyroscope threshold (in degrees per second) below which the
 * gyroscope is detected stationary.
 * @param sample_period Nominal sample period (in seconds) corresponding the rate
 * at which the application will update the algorithm.
 */
void fusion_bias_initialise(fusion_bias_t* const fusion_bias, const float threshold, const float sample_period)
{
    fusion_bias->threshold = threshold;
    fusion_bias->sample_period = sample_period;
    fusion_bias->filter_coefficient = (2.0f * M_PI * CORNER_FREQUENCY) * fusion_bias->sample_period;
    fusion_bias->stationary_timer = 0.0f;
    fusion_bias->gyroscope_bias = FUSION_VECTOR3_ZERO;
}

/**
 * @brief Updates the gyroscope bias correction algorithm and returns the
 * corrected gyroscope measurement.
 * @param fusion_bias fusion_bias_t structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @return Corrected gyroscope measurement in degrees per second.
 */
fusion_vector3_t fusion_bias_update(fusion_bias_t* const fusion_bias, fusion_vector3_t gyroscope)
{
    // Subtract bias from gyroscope measurement
    gyroscope = fusion_vector_subtract(gyroscope, fusion_bias->gyroscope_bias);

    // Reset stationary timer if gyroscope not stationary
    if ((fabs(gyroscope.axis.x) > fusion_bias->threshold) || (fabs(gyroscope.axis.y) > fusion_bias->threshold)
        || (fabs(gyroscope.axis.z) > fusion_bias->threshold)) {
        fusion_bias->stationary_timer = 0.0f;
        return gyroscope;
    }

    // Increment stationary timer while gyroscope stationary
    if (fusion_bias->stationary_timer < STATIONARY_PERIOD) {
        fusion_bias->stationary_timer += fusion_bias->sample_period;
        return gyroscope;
    }

    // Adjust bias if stationary timer has elapsed
    fusion_bias->gyroscope_bias =
    fusion_vector_add(fusion_bias->gyroscope_bias, fusion_vector_multiply_scalar(gyroscope, fusion_bias->filter_coefficient));
    return gyroscope;
}

/**
 * @brief Returns true if the gyroscope bias correction algorithm is active.
 * @param fusion_bias fusion_bias_t structure.
 * @return True if the gyroscope bias correction algorithm is active.
 */
bool fusion_bias_is_active(fusion_bias_t* const fusion_bias)
{
    return fusion_bias->stationary_timer >= STATIONARY_PERIOD;
}

//------------------------------------------------------------------------------
// End of file
