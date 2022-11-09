/**
 * @file fusion_bias_t.h
 * @author Seb Madgwick
 * @brief The gyroscope bias correction algorithm achieves run-time calibration
 * of the gyroscope bias.  The algorithm will detect when the gyroscope is
 * stationary for a set period of time and then begin to sample gyroscope
 * measurements to calculate the bias as an average.
 */

#ifndef FUSION_BIAS_H
#define FUSION_BIAS_H

//------------------------------------------------------------------------------
// Includes

#include <stdbool.h>

#include "fusion_types.h"

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Gyroscope bias correction algorithm structure.  Structure members are
 * used internally and should not be used by the user application.
 */
typedef struct {
    float threshold;
    float sample_period;
    float filter_coefficient;
    float stationary_timer;
    fusion_vector3_t gyroscope_bias;
} fusion_bias_t;

//------------------------------------------------------------------------------
// Function prototypes

void fusion_bias_initialise(fusion_bias_t* const fusion_bias, const float threshold, const float sample_period);
fusion_vector3_t fusion_bias_update(fusion_bias_t* const fusion_bias, fusion_vector3_t gyroscope);
bool fusion_bias_is_active(fusion_bias_t* const fusion_bias);

#endif

//------------------------------------------------------------------------------
// End of file
