#include "as5600.h"

/* chose one i2c instance */
static I2C_HandleTypeDef* i2c_handle = &hi2c1;

/**
 * @brief detect as5600 device address
 * @param None
 * @retval None
 */
__attribute__((unused)) static uint8_t AS5600DetectAddr(void)
{
    uint8_t addr = 0x00;
    uint8_t value = 0x55;

    for (int i = 0; i < 255; i++) {
        if (HAL_I2C_Master_Transmit(i2c_handle, addr, &value, 1, 100) == HAL_OK)
            break;
        addr++;
    }

    return addr;
}

/**
 * @brief read one byte value by address
 * @param None
 * @retval None
 */
static uint8_t AS5600ReadOneByte(uint8_t in_adr)
{
    uint8_t value = 0;

    HAL_I2C_Master_Transmit(i2c_handle, ams5600_address, &in_adr, 1, 100);
    HAL_I2C_Master_Receive(i2c_handle, ams5600_address, &value, 1, 100);

    return value;
}

/**
 * @brief read two byte from address
 * @param None
 * @retval None
 */
static uint16_t AS5600ReadTwoByte(uint8_t h_adr, uint8_t l_adr)
{
    uint8_t high = 0;
    uint8_t low = 0;
    uint16_t ret_val = 0;

    high = AS5600ReadOneByte(h_adr);
    low = AS5600ReadOneByte(l_adr);
    high &= 0x0f; // hight 4 bit is 0
    ret_val = high << 8;
    ret_val = ret_val | low;

    return ret_val;
}

/**
 * @brief get as5600 address
 * @retval None
 */
__attribute__((unused)) static int16_t AS5600GetAddress(void)
{
    return ams5600_address;
}

/**
 * @brief get value of as5600
 * @retval None
 */
uint16_t AS5600GetAngle(void)
{
    return AS5600ReadTwoByte(raw_ang_hi, raw_ang_lo);
}
