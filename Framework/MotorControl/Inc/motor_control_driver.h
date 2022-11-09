#ifndef FRAMEWORK_MOTOR_CONTROL_DRIVER_H
#define FRAMEWORK_MOTOR_CONTROL_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "motor_control_param.h"
#include "platform.h"

typedef struct motor_control_driver {
    r3_2_params_t r32_param;
    pwmc_r3_2_t pwm_control_r32;
    pwmc_t* ptr_pwm_control;
    r_divider_t bus_voltage_sensor;
    platform_t* ptr_platform_info;

    int driver_type;

} motor_control_driver_t;

void motor_control_driver_init(motor_control_driver_t* phandle, TIM_TypeDef* tim);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif
