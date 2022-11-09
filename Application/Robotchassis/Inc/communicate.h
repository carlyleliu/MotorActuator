#ifndef APPLICATION_COMMUNICATE_H_
#define APPLICATION_COMMUNICATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "peripherals.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

void UsartPrintf(const char *fmt, ...);
void UsartPuts(uint8_t *p_data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_COMMUNICATE_H_ */