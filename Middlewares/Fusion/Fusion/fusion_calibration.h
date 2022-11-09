/**
 * @file FusionCalibration.h
 * @author Seb Madgwick
 * @brief Gyroscope, accelerometer, and magnetometer calibration model.
 *
 * Static inline implementations are used to optimise for increased execution
 * speed.
 */

#ifndef FUSION_CALibRATION_H
#define FUSION_CALibRATION_H

//------------------------------------------------------------------------------
// Includes

#include "fusion_types.h"

//------------------------------------------------------------------------------
// Inline functions

/**
 * @brief Gyroscope and accelerometer calibration model.
 * @param uncalibrated Uncalibrated gyroscope or accelerometer measurement in
 * lsb.
 * @param misalignment Misalignment matrix (may not be a true rotation matrix).
 * @param sensitivity Sensitivity in g per lsb for an accelerometer and degrees
 * per second per lsb for a gyroscope.
 * @param bias Bias in lsb.
 * @return Calibrated gyroscope or accelerometer measurement.
 */
static inline __attribute__((always_inline)) fusion_vector3_t fusion_calibration_inertial(const fusion_vector3_t uncalibrated,
const fusion_rotation_matrix_t misalignment,
const fusion_vector3_t sensitivity,
const fusion_vector3_t bias)
{
    return fusion_rotation_matrix_multiply_vector(
    misalignment, fusion_vector_hadamard_product(fusion_vector_subtract(uncalibrated, bias), sensitivity));
}

/**
 * @brief Magnetometer calibration model.
 * @param magnetometer Uncalibrated magnetometer measurement in uT.
 * @param softIronMatrix Soft-iron matrix (may not be a true rotation matrix).
 * @param hardIronBias Hard-iron bias in uT.
 * @return Calibrated magnetometer measurement.
 */
static inline __attribute__((always_inline)) fusion_vector3_t fusion_calibration_magnetic(const fusion_vector3_t uncalibrated,
const fusion_rotation_matrix_t softIronMatrix,
const fusion_vector3_t hardIronBias)
{
    return fusion_vector_subtract(fusion_rotation_matrix_multiply_vector(softIronMatrix, uncalibrated), hardIronBias);
}

#endif

//------------------------------------------------------------------------------
// End of file
