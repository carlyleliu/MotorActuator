/**
 * @file FusionCompass.c
 * @author Seb Madgwick
 * @brief The tilt-compensated compass calculates an angular heading relative to
 * magnetic north using accelerometer and magnetometer measurements (NWU
 * convention).
 */

//------------------------------------------------------------------------------
// Includes

#include "fusion_compass.h"

#include <math.h> // atan2f

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Calculates the tilt-compensated heading relative to magnetic north.
 * @param accelerometer Accelerometer measurement in any calibrated units.
 * @param magnetometer Magnetometer measurement in any calibrated units.
 * @return Heading angle in degrees.
 */
float fusion_compass_calculate_heading(const fusion_vector3_t accelerometer, const fusion_vector3_t magnetometer)
{
    // Compute direction of 'magnetic west' (Earth's y axis)
    const fusion_vector3_t magneticWest = fusion_vector_fast_normalise(fusion_vector_cross_product(accelerometer, magnetometer));

    // Compute direction of magnetic north (Earth's x axis)
    const fusion_vector3_t magneticNorth = fusion_vector_fast_normalise(fusion_vector_cross_product(magneticWest, accelerometer));

    // Calculate angular heading relative to magnetic north
    return fusion_radians_to_degrees(atan2f(magneticWest.axis.x, magneticNorth.axis.x));
}

//------------------------------------------------------------------------------
// End of file
