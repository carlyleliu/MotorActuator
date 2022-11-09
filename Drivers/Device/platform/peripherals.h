#ifndef PLATFORM_PERIPHERALS
#define PLATFORM_PERIPHERALS

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_hal.h"

extern ADC_HandleTypeDef hadc2;
extern CORDIC_HandleTypeDef hcordic;
extern FMAC_HandleTypeDef hfmac;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

void system_clock_config(void);

void peripherals_init(void);

#ifdef __cplusplus
}
#endif

#endif
