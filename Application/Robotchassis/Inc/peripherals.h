#ifndef APPLICATION_PERIPHERALS_H_
#define APPLICATION_PERIPHERALS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* export struct data */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern CORDIC_HandleTypeDef hcordic;

extern CRC_HandleTypeDef hcrc;

extern FDCAN_HandleTypeDef hfdcan1;

extern FMAC_HandleTypeDef hfmac;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* export function */
void SystemClockConfig(void);
void PeripheralsInit(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_PERIPHERALS_H_ */