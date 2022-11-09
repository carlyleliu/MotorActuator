/**
 * @Autor: yyliushuai
 * @Version: 1.0
 * @Date: 2021-05-23 11:05:03
 * @LastEditors: yyliushuai
 * @LastEditTime: 2021-05-30 00:17:17
 * @Description:
 * Copyright (C) 2021 yyliushuai. All rights reserved.
 */
#include "icm20602.h"

static SPI_HandleTypeDef* spi_handle = &hspi1;

static float accel_scale;
static float gyro_scale;

/**
 * @brief  icm20602 read buffer data
 * @param  None
 * @retval None
 */
static uint8_t Icm20602ReadBuffer(uint8_t const reg_addr, uint8_t* p_data, uint8_t len)
{
    uint8_t tx = 0, rx = 0;
    uint8_t tx_buff[14];

    ICM20602_ENABLE
    tx = reg_addr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(spi_handle, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(spi_handle, tx_buff, p_data, len, 55);
    ICM20602_DISABLE

    return 0;
}

/**
 * @brief  icm20602 write register data
 * @param  None
 * @retval None
 */
static uint8_t Icm20602WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx = 0, rx = 0;

    ICM20602_ENABLE
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(spi_handle, &tx, &rx, 1, 55);
    tx = val;
    HAL_SPI_TransmitReceive(spi_handle, &tx, &rx, 1, 55);
    ICM20602_DISABLE

    return 0;
}

/**
 * @brief  icm20602 read register data
 * @param  None
 * @retval None
 */
static uint8_t Icm20602ReadReg(uint8_t reg)
{
    uint8_t tx = 0, rx = 0;

    ICM20602_ENABLE
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(spi_handle, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(spi_handle, &tx, &rx, 1, 55);
    ICM20602_DISABLE

    return rx;
}

/**
 * @brief  icm20602 set gyro scale
 * @param  None
 * @retval None
 */
static uint8_t Icm20602SetGyroFullscale(uint8_t fs)
{
    switch (fs) {
        case ICM20_GYRO_FS_250:
            gyro_scale = 1.0f / 131.068f; // 32767/250
            break;
        case ICM20_GYRO_FS_500:
            gyro_scale = 1.0f / 65.534f;
            break;
        case ICM20_GYRO_FS_1000:
            gyro_scale = 1.0f / 32.767f;
            break;
        case ICM20_GYRO_FS_2000:
            gyro_scale = 1.0f / 16.4f;
            break;
        default:
            fs = ICM20_GYRO_FS_2000;
            gyro_scale = 1.0f / 16.3835f;
            break;
    }

    return Icm20602WriteReg(ICM20_GYRO_CONFIG, fs);
}

/**
 * @brief  icm20602 set accel scale
 * @param  None
 * @retval None
 */
static uint8_t Icm20602SetAccelFullscale(uint8_t fs)
{
    switch (fs) {
        case ICM20_ACCEL_FS_2G:
            accel_scale = 1.0f / 16384.0f;
            break;
        case ICM20_ACCEL_FS_4G:
            accel_scale = 1.0f / 8192.0f;
            break;
        case ICM20_ACCEL_FS_8G:
            accel_scale = 1.0f / 4096.0f;
            break;
        case ICM20_ACCEL_FS_16G:
            accel_scale = 1.0f / 2048.0f;
            break;
        default:
            fs = ICM20_ACCEL_FS_8G;
            accel_scale = 1.0f / 4096.0f;
            break;
    }
    return Icm20602WriteReg(ICM20_ACCEL_CONFIG, fs);
}

/**
 * @brief  icm20602 get accel adc data
 * @param  None
 * @retval None
 */
static uint8_t Icm20602GetAccelAdc(int16_t* accel)
{
    uint8_t buf[6];

    if (Icm20602ReadBuffer(ICM20_ACCEL_XOUT_H, buf, 6))
        return 1;

    accel[0] = ((int16_t)buf[0] << 8) + buf[1];
    accel[1] = ((int16_t)buf[2] << 8) + buf[3];
    accel[2] = ((int16_t)buf[4] << 8) + buf[5];

    return 0;
}

/**
 * @brief  icm20602 fet gyro adc data
 * @param  None
 * @retval None
 */
static uint8_t Icm20602GetGyroAdc(int16_t* gyro)
{
    uint8_t buf[6];

    if (Icm20602ReadBuffer(ICM20_GYRO_XOUT_H, buf, 6))
        return 1;
    gyro[0] = (buf[0] << 8) + buf[1];
    gyro[1] = (buf[2] << 8) + buf[3];
    gyro[2] = (buf[4] << 8) + buf[5];

    return 0;
}

/**
 * @brief  icm20602 read gyro data
 * @param  None
 * @retval None
 */
static uint8_t Icm20602GetGyro(float* gyro)
{
    int16_t gyro_adc[3];

    if (Icm20602GetGyroAdc(gyro_adc) == 1)
        return 1;
    gyro[0] = gyro_scale * gyro_adc[0];
    gyro[1] = gyro_scale * gyro_adc[1];
    gyro[2] = gyro_scale * gyro_adc[2];

    return 0;
}

/**
 * @brief  icm20602 read accel data
 * @param  None
 * @retval None
 */
static uint8_t Icm20602GetAccel(float* accel)
{
    int16_t accel_adc[3];

    if (Icm20602GetAccelAdc(accel_adc) == 1) {
        return 1;
    }
    accel[0] = accel_scale * accel_adc[0];
    accel[1] = accel_scale * accel_adc[1];
    accel[2] = accel_scale * accel_adc[2];

    return 0;
}

/**
 * @brief  icm20602 read temp data
 * @param  None
 * @retval None
 */
static float Icm20602GetTemp(void)
{
    int16_t temp_adc;
    uint8_t buf[2];

    if (Icm20602ReadBuffer(ICM20_TEMP_OUT_H, buf, 2))
        return 0.0f;

    temp_adc = (buf[0] << 8) + buf[1];

    return (25.0f + (float)temp_adc / 326.8f);
}

/**
 * @brief  icm20602 initialize function
 * @param  None
 * @retval None
 */
uint8_t Icm20602Init(void)
{
    if (Icm20602WriteReg(ICM20_PWR_MGMT_1, 0x80)) {
        // LOG_ERR("icm_20602 reset fail\r\n");
        return 1;
    }
    HAL_Delay(50);

    Icm20602WriteReg(ICM20_PWR_MGMT_1, 0x01);
    HAL_Delay(100);

    if (Icm20602ReadReg(ICM20_WHO_AM_I) != 0x12) {
        // LOG_ERR("icm_20602 init failed [id] = %x\r\n", Icm20602ReadReg(ICM20_WHO_AM_I));
        return 1;
    }

    Icm20602WriteReg(ICM20_SMPLRT_DIV, 0);
    Icm20602WriteReg(ICM20_INT_PIN_CFG, 0x10);

    Icm20602WriteReg(ICM20_CONFIG, DLPF_BW_41);
    Icm20602WriteReg(ICM20_ACCEL_CONFIG2, ACCEL_AVER_8 | ACCEL_DLPF_BW_44);

    Icm20602SetAccelFullscale(ICM20_ACCEL_FS_8G);
    Icm20602SetGyroFullscale(ICM20_GYRO_FS_2000);
    HAL_Delay(100);

    return 0;
}

/**
 * @brief  icm20602 read all data
 * @param  None
 * @retval None
 */
void Icm20602Update(struct imu_raw* imu)
{
    if (1 == Icm20602GetGyro(imu->gyro_raw)) {
        imu->gyro_err = 1; // gyro read error
    } else {
        imu->gyro_err = 0;
    }
    if (1 == Icm20602GetAccel(imu->acc_raw)) {
        imu->accel_err = 1; // accel read error
    } else {
        imu->accel_err = 0;
    }
    imu->temperature = Icm20602GetTemp();

    if (1 == imu->gyro_has_calibration) {
        imu->gyro_raw[0] -= imu->gyro_offset[0];
        imu->gyro_raw[1] -= imu->gyro_offset[1];
        imu->gyro_raw[2] -= imu->gyro_offset[2];
    }

    imu->time_stamp = Micros();
}
