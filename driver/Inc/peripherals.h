#ifndef __DRIVERS_PERIPHERALS_H__
#define __DRIVERS_PERIPHERALS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stm32g4xx_hal.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern CORDIC_HandleTypeDef hcordic;
extern CRC_HandleTypeDef hcrc;
extern FDCAN_HandleTypeDef hfdcan1;
extern FMAC_HandleTypeDef hfmac;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

extern I2C_HandleTypeDef hi2c1;

void PeripheralsInit(void);
void Error_Handler(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#endif // ! __DRIVERS_PERIPHERALS_H__
