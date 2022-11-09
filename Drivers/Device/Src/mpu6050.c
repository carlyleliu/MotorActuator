#include "mpu6050.h"

static I2C_HandleTypeDef* i2c_handle = &hi2c1;

/**
 * @brief  mpu6050 initialize function
 * @param  None
 * @retval None
 */
uint8_t Mpu6050Init(void)
{
    uint8_t check, data;

    /* check device ID WHO_AM_I */
    HAL_I2C_Mem_Read(i2c_handle, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, I2C_TIMEOUT);

    if (check == 104) { /* 0x68 will be returned by the sensor if everything goes well  */
        /* power management register 0X6B we should write all 0's to wake the sensor up */
        data = 0;
        HAL_I2C_Mem_Write(i2c_handle, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, I2C_TIMEOUT);
        /* Set data RATE of 1KHz by writing SMPLRT_DIV register */
        data = 0x07;
        HAL_I2C_Mem_Write(i2c_handle, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, I2C_TIMEOUT);
        /* Set accelerometer configuration in ACCEL_CONFIG Register
         * XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
         * */
        data = 0x00;
        HAL_I2C_Mem_Write(i2c_handle, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, I2C_TIMEOUT);
        /* Set Gyroscopic configuration in GYRO_CONFIG Register
         * XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
         * */
        data = 0x00;
        HAL_I2C_Mem_Write(i2c_handle, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, I2C_TIMEOUT);
        return 0;
    }
    return 1;
}

/**
 * @brief 	mpu6050 read accel data
 * @param  None
 * @retval None
 */
void Mpu6050ReadAccel(struct imu_raw* imu)
{
    uint8_t rec_data[6];
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;

    /* Read 6 BYTES of data starting from ACCEL_XOUT_H register */
    HAL_I2C_Mem_Read(i2c_handle, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 6, I2C_TIMEOUT);
    accel_x_raw = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    accel_y_raw = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    accel_z_raw = (int16_t)(rec_data[4] << 8 | rec_data[5]);
    imu->acc_raw[0] = accel_x_raw / 16384.0;
    imu->acc_raw[1] = accel_y_raw / 16384.0;
    imu->acc_raw[2] = accel_z_raw / ACCEL_Z_CORRECTOR;
}

/**
 * @brief  mpu6050 read gyro data
 * @param  None
 * @retval None
 */
void Mpu6050ReadGyro(struct imu_raw* imu)
{
    uint8_t rec_data[6];
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

    /* Read 6 BYTES of data starting from GYRO_XOUT_H register */
    HAL_I2C_Mem_Read(i2c_handle, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data, 6, I2C_TIMEOUT);
    gyro_x_raw = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    gyro_y_raw = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    gyro_z_raw = (int16_t)(rec_data[4] << 8 | rec_data[5]);
    imu->gyro_raw[0] = gyro_x_raw / 131.0;
    imu->gyro_raw[1] = gyro_y_raw / 131.0;
    imu->gyro_raw[2] = gyro_z_raw / 131.0;
}

/**
 * @brief  mpu6050 read temp data
 * @param  None
 * @retval None
 */
void Mpu6050ReadTemp(struct imu_raw* imu)
{
    uint8_t rec_data[2];
    int16_t temp;
    /* Read 2 BYTES of data starting from TEMP_OUT_H_REG register */
    HAL_I2C_Mem_Read(i2c_handle, MPU6050_ADDR, TEMP_OUT_H_REG, 1, rec_data, 2, I2C_TIMEOUT);
    temp = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    imu->temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

/**
 * @brief  mpu6050 read all data
 * @param  None
 * @retval None
 */
void mpu6050Update(struct imu_raw* imu)
{
    uint8_t rec_data[14];
    int16_t temp;
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

    /* Read 14 BYTES of data starting from ACCEL_XOUT_H register */
    HAL_I2C_Mem_Read(i2c_handle, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 14, I2C_TIMEOUT);
    accel_x_raw = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    accel_y_raw = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    accel_z_raw = (int16_t)(rec_data[4] << 8 | rec_data[5]);
    temp = (int16_t)(rec_data[6] << 8 | rec_data[7]);
    gyro_x_raw = (int16_t)(rec_data[8] << 8 | rec_data[9]);
    gyro_y_raw = (int16_t)(rec_data[10] << 8 | rec_data[11]);
    gyro_z_raw = (int16_t)(rec_data[12] << 8 | rec_data[13]);
    imu->acc_raw[0] = accel_x_raw / 16384.0;
    imu->acc_raw[1] = accel_y_raw / 16384.0;
    imu->acc_raw[2] = accel_z_raw / ACCEL_Z_CORRECTOR;
    imu->temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    imu->gyro_raw[0] = gyro_x_raw / 131.0;
    imu->gyro_raw[1] = gyro_y_raw / 131.0;
    imu->gyro_raw[2] = gyro_z_raw / 131.0;

    if (1 == imu->gyro_has_calibration) {
        imu->gyro_raw[0] -= imu->gyro_offset[0];
        imu->gyro_raw[1] -= imu->gyro_offset[1];
        imu->gyro_raw[2] -= imu->gyro_offset[2];
    }

    imu->time_stamp = micros();
}
