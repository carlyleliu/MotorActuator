#include "util.h"

/**
 * @brief get value of hight byte
 * @retval None
 */
uint8_t HighByte(uint16_t value)
{
    uint8_t ret;

    value = value >> 8;
    ret = (uint8_t)value;

    return ret;
}

/**
 * @brief get value of low byte
 * @retval None
 */
uint8_t LowByte(uint16_t value)
{
    uint8_t ret;

    value = value & 0x00ff;
    ret = (uint8_t)value;

    return ret;
}