#ifndef APPLICATION_SYSTICKS_H_
#define APPLICATION_SYSTICKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void sys_ticks_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
uint32_t micros(void);

#ifdef __cplusplus
}
#endif

#endif /*  APPLICATION_SYSTICKS_H_  */
