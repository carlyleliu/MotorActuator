#include <stdio.h>

#include "..\fusion\fusion.h"

fusion_bias_t fusion_bias_t;
fusion_ahrs_t fusion_ahrs_t;

float sample_period = 0.01f; // replace this value with actual sample period in seconds

fusion_vector3_t gyroscopeSensitivity = {
.axis.x = 1.0f,
.axis.y = 1.0f,
.axis.z = 1.0f,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

fusion_vector3_t accelerometerSensitivity = {
.axis.x = 1.0f,
.axis.y = 1.0f,
.axis.z = 1.0f,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

int main()
{
    // Initialise gyroscope bias correction algorithm
    fusion_bias_initialise(&fusion_bias_t, 0.5f, sample_period); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    fusion_ahrs_initialise(&fusion_ahrs_t, 0.5f); // gain = 0.5

    // The contents of this do while loop should be called for each time new sensor measurements are available
    do {
        // Calibrate gyroscope
        fusion_vector3_t uncalibratedGyroscope = {
        .axis.x = 0.0f, /* replace this value with actual gyroscope x axis measurement in lsb */
        .axis.y = 0.0f, /* replace this value with actual gyroscope y axis measurement in lsb */
        .axis.z = 0.0f, /* replace this value with actual gyroscope z axis measurement in lsb */
        };
        fusion_vector3_t calibratedGyroscope = fusion_calibration_inertial(
        uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

        // Calibrate accelerometer
        fusion_vector3_t uncalibratedAccelerometer = {
        .axis.x = 0.0f, /* replace this value with actual accelerometer x axis measurement in lsb */
        .axis.y = 0.0f, /* replace this value with actual accelerometer y axis measurement in lsb */
        .axis.z = 1.0f, /* replace this value with actual accelerometer z axis measurement in lsb */
        };
        fusion_vector3_t calibratedAccelerometer = fusion_calibration_inertial(
        uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

        // Update gyroscope bias correction algorithm
        calibratedGyroscope = fusion_bias_update(&fusion_bias_t, calibratedGyroscope);

        // Update AHRS algorithm
        fusion_ahrs_update_without_magnetometer(&fusion_ahrs_t, calibratedGyroscope, calibratedAccelerometer, sample_period);

        // Print Euler angles
        fusion_euler_angles_t eulerAngles = fusion_quaternion_to_euler_angles(fusion_ahrs_get_quaternion(&fusion_ahrs_t));
        printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch,
        eulerAngles.angle.yaw);

    } while (false);
}
