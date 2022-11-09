#ifndef APPLICATION_SYSTICKS_H_
#define APPLICATION_SYSTICKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void SysTicksInit(void);
void DelayUs(uint32_t nus);
void DelayMs(uint16_t nms);
uint32_t Micros(void);
	
#ifdef __cplusplus
}
#endif

#endif /*  APPLICATION_SYSTICKS_H_  */
