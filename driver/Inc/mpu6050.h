#ifndef __DRIVERS_INC_MPU6050_H__
#define __DRIVERS_INC_MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "imu_common.h"
#include "peripherals.h"
#include "systicks.h"

#define RAD_TO_DEG        57.295779513082320876798154814105
#define I2C_TIMEOUT       100
#define ACCEL_Z_CORRECTOR 14418.0

/* define register address */
#define WHO_AM_I_REG     0x75
#define PWR_MGMT_1_REG   0x6B
#define SMPLRT_DIV_REG   0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG   0x41
#define GYRO_CONFIG_REG  0x1B
#define GYRO_XOUT_H_REG  0x43
#define MPU6050_ADDR     0xD0

uint8_t Mpu6050Init(void);
void Mpu6050ReadAccel(struct imu_raw* imu);
void Mpu6050ReadGyro(struct imu_raw* imu);
void Mpu6050ReadTemp(struct imu_raw* imu);
void Mpu6050Update(struct imu_raw* imu);

#ifdef __cplusplus
}
#endif

#endif // ! __DRIVERS_INC_MPU6050_H__
