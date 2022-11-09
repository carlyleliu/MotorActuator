#ifndef DRIVE_DEVICE_USART_H_
#define DRIVE_DEVICE_USART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdio.h>

#include "main.h"
#include "peripherals.h"

/* export function */
void UsartPrintf(const char* fmt, ...);
void UsartPuts(uint8_t* p_data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_AS5600_H_ */
