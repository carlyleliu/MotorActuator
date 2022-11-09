#include <stdio.h>

#include "..\fusion\fusion.h"

fusion_vector3_t accelerometerSensitivity = {
.axis.x = 1.0f,
.axis.y = 1.0f,
.axis.z = 1.0f,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

fusion_vector3_t hardIronBias = {
.axis.x = 0.0f,
.axis.y = 0.0f,
.axis.z = 0.0f,
}; // replace these values with actual hard-iron bias in uT if known

int main()
{
    // The contents of this do while loop should be called for each time new sensor measurements are available
    do {
        // Calibrate accelerometer
        fusion_vector3_t uncalibratedAccelerometer = {
        .axis.x = 0.0f, /* replace this value with actual accelerometer x axis measurement in lsb */
        .axis.y = 0.0f, /* replace this value with actual accelerometer y axis measurement in lsb */
        .axis.z = 1.0f, /* replace this value with actual accelerometer z axis measurement in lsb */
        };
        fusion_vector3_t calibratedAccelerometer = fusion_calibration_inertial(
        uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate magnetometer
        fusion_vector3_t uncalibratedMagnetometer = {
        .axis.x = 0.5f, /* replace this value with actual magnetometer x axis measurement in uT */
        .axis.y = 0.0f, /* replace this value with actual magnetometer y axis measurement in uT */
        .axis.z = 0.0f, /* replace this value with actual magnetometer z axis measurement in uT */
        };
        fusion_vector3_t calibratedMagnetometer =
        fusion_calibration_magnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

        // Calculate heading
        float heading = fusion_compass_calculate_heading(calibratedAccelerometer, calibratedMagnetometer);

        // Print heading
        printf("Heading = %0.1f\r\n", heading);

    } while (false);
}
