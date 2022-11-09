#ifndef __DRIVE_USART_H__
#define __DRIVE_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdio.h>

#include "peripherals.h"

/* export function */
void UsartPrintf(const char* fmt, ...);
void UsartPuts(uint8_t* p_data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif // ! __DRIVE_USART_H__
