/**
 * @file Fusion.h
 * @author Seb Madgwick
 * @brief Main header file for the library.  This is the only file that needs to
 * be included when using the library.
 *
 * Fusion is an ANSI C99 compliment sensor fusion library for sensor arrays of
 * gyroscopes, accelerometers, and magnetometers.  Fusion was specifically
 * developed for use with embedded systems and has been optimised for execution
 * speed.  The library includes modules for: attitude and heading reference
 * system (AHRS) sensor fusion, gyroscope bias correction, and a tilt-
 * compensated compass.
 */

#ifndef FUSION_H
#define FUSION_H

//------------------------------------------------------------------------------
// Includes

#ifdef __cplusplus
extern "C" {
#endif

#include "fusion_ahrs.h"
#include "fusion_bias.h"
#include "fusion_calibration.h"
#include "fusion_compass.h"
#include "fusion_types.h"

#ifdef __cplusplus
}
#endif

#endif
//------------------------------------------------------------------------------
// End of file
