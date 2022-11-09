#ifndef FRAMEWORK_FOC_H
#define FRAMEWORK_FOC_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "abs_encoder.h"
#include "digital_output.h"
#include "mc_math.h"
#include "mc_position.h"
#include "mc_tuning.h"
#include "mc_type.h"
#include "regular_conversion_manager.h"
#include "state_machine.h"

#include "motor_control_base.h"
#include "motor_control_driver.h"
#include "motor_control_interface.h"
#include "motor_control_param.h"

typedef enum foc_run_mode {
    OPEN_LOOP_MODE,
    POSITION_LOOP_MODE,
    SPEED_LOOP_MODE,
    TORQ_LOOP_MODE,
} foc_run_mode_t;

typedef struct foc {
    motor_control_driver_t motor_control_driver;
    motor_control_interface_t motor_control_interface;

    foc_vars_t foc_vars;
    mct_t motor_control_tuning;
    state_machine_t state_machine;
    abs_encoder_t abs_encoder;
    position_t position;
    speedn_torq_ctrl_t speedn_torq_ctrl;
    pid_float_t pid_position;
    pid_float_t pid_speed;
    pid_float_t pid_integer_torq;
    pid_float_t pid_iq;
    pid_float_t pid_id;
    virtual_speed_sensor_t virtual_speed_sensor;

    pqd_motor_pow_meas_t motor_power_meas;
    circle_limitation_t circle_limit;
    ramp_ext_mngr_t ramp_ext_mngr;
    rev_up_ctrl_t revup_control;
    ntc_t ntc_sensor;

    foc_run_mode_t mode;

#ifdef D_OUT
    dout_t r_break;
    dout_t ocp_disabling;
#endif

    uint16_t boot_cap_delay_counter;
    uint16_t stop_permanency_counter;
    uint16_t task_counter;

    uint8_t boot_completed : 1;
    uint8_t use_current_loop : 1;
    int direction;

    TIM_TypeDef* tim_x;

    int medium_freq_ticks;
    int medium_freq;
    int transition_duration;
    int park_angle_compensation_factor;
    int rev_park_angle_compensation_factor;
} foc_t;

void foc_init(foc_t* phandle, TIM_TypeDef* tim);
void foc_config_pid(pid_float_t* src_pid, pid_float_t* target_pid);
void foc_run_task(foc_t* phandle);
void foc_scheduler(foc_t* phandle);
void foc_medium_frequency_task(foc_t* phandle);
void foc_clear(foc_t* phandle);
void foc_calc_curr_ref(foc_t* phandle);
void foc_high_frequency_task(foc_t* phandle);
uint16_t foc_svpwm_controller(foc_t* phandle, qd_t vqd, int16_t el_angle);
uint16_t foc_curr_controller(foc_t* phandle);
void foc_open_loop_controller(foc_t* phandle);
void foc_speed_controller(foc_t* phandle);
void foc_torq_controller(foc_t* phandle);
void foc_position_controller(foc_t* phandle);

foc_vars_t* foc_get_varis(foc_t* phandle);

motor_control_driver_t* foc_get_motor_control_driver(foc_t* phandle);
motor_control_interface_t* foc_get_motor_control_interface(foc_t* phandle);
void foc_set_charge_boot_cap_delay(foc_t* phandle, uint16_t tick_count);
bool foc_charge_boot_cap_delay_has_elapsed(foc_t* phandle);
void foc_set_stop_permanency_time(foc_t* phandle, uint16_t tick_count);
bool foc_stop_permanency_time_has_elapsed(foc_t* phandle);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif
