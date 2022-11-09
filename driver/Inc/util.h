#ifndef __DRIVER_UTIL_H__
#define __DRIVER_UTIL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32g4xx_hal.h>

#define PI       3.1415926
#define Deg2Rad  (1 / 57.2957795)
#define NORMAL_G 0.974197667

uint8_t HighByte(uint16_t value);
uint8_t LowByte(uint16_t value);

#ifdef __cplusplus
}
#endif

#endif // ! __DRIVER_UTIL_H__