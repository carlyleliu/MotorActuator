#ifndef __SYSTICKS_H__
#define __SYSTICKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32g4xx_hal.h>
#include "util.h"

void SysTicksInit(void);
void DelayUs(uint32_t nus);
void DelayMs(uint16_t nms);
uint32_t Micros(void);
float time(void);

#ifdef __cplusplus
}
#endif

#endif // ! __SYSTICKS_H__
