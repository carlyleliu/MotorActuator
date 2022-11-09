/**
 * @file FusionCompass.h
 * @author Seb Madgwick
 * @brief The tilt-compensated compass calculates an angular heading relative to
 * magnetic north using accelerometer and magnetometer measurements (NWU
 * convention).
 */

#ifndef FUSION_COMPASS_H
#define FUSION_COMPASS_H

//------------------------------------------------------------------------------
// Includes

#include "fusion_types.h"

//------------------------------------------------------------------------------
// Function prototypes

float fusion_compass_calculate_heading(const fusion_vector3_t accelerometer, const fusion_vector3_t magnetometer);

#endif

//------------------------------------------------------------------------------
// End of file
