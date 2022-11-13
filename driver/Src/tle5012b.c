#include "tle5012b.h"

static SPI_HandleTypeDef* spi_handle = &hspi1;

/**
 * @brief spi read value
 * @param u16RegValue write data
 * @retval read value
 */
static uint16_t SpiReadValue(uint16_t reg_value)
{
    uint16_t u16_data;

    SPI_CS_ENABLE;

    HAL_SPI_Transmit(spi_handle, (uint8_t*)(&reg_value), sizeof(reg_value) / sizeof(uint16_t), 0);
    HAL_SPI_Receive(spi_handle, (uint8_t*)(&u16_data), sizeof(u16_data) / sizeof(uint16_t), 0);

    SPI_CS_DISABLE;

    return ((u16_data & 0x7FFF) << 1);
}

/**
 * @brief read angle
 * @param None
 * @retval angle raw value
 */
uint16_t Tle5012bReadAngle(void)
{
    return SpiReadValue(READ_ANGLE_VALUE);
}

/**
 * @brief read speed
 * @param None
 * @retval speed raw value
 */
uint16_t Tle5012bReadSpeed(void)
{
    return SpiReadValue(READ_SPEED_VALUE);
}
