#include "foc.h"

#include "device.h"

static uint16_t mini_table[87] = MMITABLE;

/**
 * @brief  foc initralize
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_init(foc_t* phandle, TIM_TypeDef* tim)
{
    LOG_DBG("####  FOC Start Init!  ####\n");

    phandle->tim_x = tim;
    motor_control_driver_init(&phandle->motor_control_driver, phandle->tim_x);

    phandle->mode = OPEN_LOOP_MODE;
    phandle->park_angle_compensation_factor = 0;
    phandle->direction = 1;

    phandle->motor_power_meas.conv_fact = PQD_CONVERSION_FACTOR;

    stm_init(&phandle->state_machine);
    /* config speed pid */
    phandle->pid_speed.def_kp_gain = 0;
    phandle->pid_speed.def_ki_gain = 0;
    pid_float_init(&phandle->pid_speed);
    /* config position pid */
    phandle->pid_position.def_kp_gain = 0;
    phandle->pid_position.def_ki_gain = 0;
    pid_float_init(&phandle->pid_position);
    /* config troq pid */
    phandle->pid_integer_torq.def_kp_gain = 0;
    phandle->pid_integer_torq.def_ki_gain = 0;
    pid_float_init(&phandle->pid_integer_torq);

    phandle->circle_limit.max_module = MAX_MODULE;
    phandle->circle_limit.max_vd = (uint16_t)(MAX_MODULE * 950 / 1000);
    phandle->circle_limit.circle_limit_table = mini_table;
    phandle->circle_limit.start_index = START_INDEX;

    /* config abs encoder */
    phandle->abs_encoder.sensor_reliable = 1;
    phandle->abs_encoder.abs_encoder_device_init = NULL;
    phandle->abs_encoder.abs_encoder_read_angle = &Tle5012bReadAngle;
    phandle->abs_encoder.abs_encoder_read_speed = &Tle5012bReadSpeed;
    abs_encoder_init(&phandle->abs_encoder);

    /* config vss */
    phandle->virtual_speed_sensor._Super.el_to_mec_ratio = POLE_PAIR_NUM;
    phandle->virtual_speed_sensor._Super.dpp_conv_factor = 65536;
    phandle->virtual_speed_sensor.speed_sampling_freq_hz = phandle->medium_freq;
    vss_init(&phandle->virtual_speed_sensor);

    /* config stc */
    phandle->speedn_torq_ctrl.stc_frequency_hz = phandle->medium_freq;
    phandle->speedn_torq_ctrl.mode_default = STC_SPEED_MODE;
    phandle->speedn_torq_ctrl.idref_default = 0;
    phandle->speedn_torq_ctrl.torque_ref_default = 0;
    stc_init(&phandle->speedn_torq_ctrl, &phandle->pid_position, &phandle->abs_encoder._Super);

    /* config REMNG */
    phandle->ramp_ext_mngr.frequency_hz = phandle->motor_control_driver.ptr_platform_info->pwm_frequency;
    remng_init(&phandle->ramp_ext_mngr);

    position_init(&phandle->position, &phandle->abs_encoder);

    phandle->foc_vars.drive_input = EXTERNAL;
    phandle->foc_vars.iqd_ref = stc_get_default_iqd_ref(&phandle->speedn_torq_ctrl);
    phandle->foc_vars.user_id_ref = stc_get_default_iqd_ref(&phandle->speedn_torq_ctrl).d;

    mci_init(&phandle->motor_control_interface, &phandle->state_machine, &phandle->speedn_torq_ctrl, &phandle->foc_vars);
    mci_exec_speed_ramp(&phandle->motor_control_interface, stc_get_mec_speed_ref_unit_default(&phandle->speedn_torq_ctrl), 0); /*First command to STC*/

    /* config mct */
    phandle->motor_control_tuning.ptr_pid_speed = &phandle->pid_speed;
    phandle->motor_control_tuning.ptr_pid_iq = &phandle->pid_iq;
    phandle->motor_control_tuning.ptr_pid_id = &phandle->pid_id;
    phandle->motor_control_tuning.ptr_pid_flux_weakening = NULL;
    phandle->motor_control_tuning.ptr_pwmn_curr_fdbk = phandle->motor_control_driver.ptr_pwm_control;
    phandle->motor_control_tuning.ptr_speed_sensor_main = &phandle->abs_encoder._Super;
    phandle->motor_control_tuning.ptr_speed_sensor_aux = NULL;
    phandle->motor_control_tuning.ptr_speed_sensor_virtual = &phandle->virtual_speed_sensor;
    phandle->motor_control_tuning.ptr_speedn_torque_ctrl = &phandle->speedn_torq_ctrl;
    phandle->motor_control_tuning.ptr_state_machine = &phandle->state_machine;
    phandle->motor_control_tuning.ptr_temperature_sensor = &phandle->ntc_sensor;
    phandle->motor_control_tuning.ptr_brake_digital_output = NULL;
    phandle->motor_control_tuning.ptr_ntc_relay = NULL;
    phandle->motor_control_tuning.ptr_mpm = NULL;
    phandle->motor_control_tuning.ptr_pos_ctrl = NULL;

    phandle->use_current_loop = 0;

    foc_clear(phandle);

    phandle->boot_completed = 1;
    LOG_DBG("CPU Freq %d\n", phandle->motor_control_driver.ptr_platform_info->sysclk_freq);
    LOG_DBG("PWM Freq %d\n", phandle->motor_control_driver.ptr_platform_info->pwm_frequency);
    LOG_DBG("SYS Freq %d\n", phandle->motor_control_driver.ptr_platform_info->sys_frequency);
    LOG_DBG("####  FOC End Init!  ####\n");
}

/**
 * @brief  foc config pid param
 * @param  src_pid: handler of the src pis instance
 * @param  target_pid: handler of the target pis instance
 * @retval None
 */
void foc_config_pid(pid_float_t* src_pid, pid_float_t* target_pid)
{
    src_pid->kp_gain = target_pid->kp_gain;
    src_pid->ki_gain = target_pid->ki_gain;
    src_pid->kd_gain = target_pid->kd_gain;
    src_pid->lower_integral_limit = target_pid->lower_integral_limit;
    src_pid->lower_output_limit = target_pid->lower_output_limit;
    src_pid->upper_integral_limit = target_pid->upper_integral_limit;
    src_pid->upper_output_limit = target_pid->upper_output_limit;
}

/**
 * @brief  foc clear
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_clear(foc_t* phandle)
{
    ab_t null_ab = {(int16_t)0, (int16_t)0};
    qd_t null_qd = {(int16_t)0, (int16_t)0};
    alphabeta_t null_alphabeta = {(int16_t)0, (int16_t)0};

    phandle->foc_vars.iab = null_ab;
    phandle->foc_vars.ialphabeta = null_alphabeta;
    phandle->foc_vars.iqd = null_qd;
    phandle->foc_vars.iqd_ref = null_qd;
    phandle->foc_vars.teref = (int16_t)0;
    phandle->foc_vars.vqd = null_qd;
    phandle->foc_vars.valphabeta = null_alphabeta;
    phandle->foc_vars.el_angle = (int16_t)0;

    phandle->park_angle_compensation_factor = 0;
    phandle->direction = 1;

    pid_float_set_integral_term(&phandle->pid_iq, 0);
    pid_float_set_integral_term(&phandle->pid_id, 0);

    stc_clear(&phandle->speedn_torq_ctrl);

    pwmc_switch_off_pwm(phandle->motor_control_driver.ptr_pwm_control);
}

/**
 * @brief  foc run task
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_run_task(foc_t* phandle)
{
    if (phandle->boot_completed) {
        /* ** Medium Frequency Tasks ** */
        foc_scheduler(phandle);
    }
}

/**
 * @brief  foc scheduler
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_scheduler(foc_t* phandle)
{
    if (phandle->boot_completed == 1) {
        if (phandle->task_counter > 0u) {
            phandle->task_counter--;
        } else {
            foc_medium_frequency_task(phandle);
            phandle->task_counter = phandle->medium_freq_ticks;
        }
        if (phandle->boot_cap_delay_counter > 0u) {
            phandle->boot_cap_delay_counter--;
        }
        if (phandle->stop_permanency_counter > 0u) {
            phandle->stop_permanency_counter--;
        }
    } else {
    }
}

/**
 * @brief  foc medium frequency task
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_medium_frequency_task(foc_t* phandle)
{
    State_t state;

    bool is_speed_reliable = abs_encoder_reliable(&phandle->abs_encoder);
    pqd_calc_el_motor_power(&phandle->motor_power_meas);

    state = stm_get_state(&phandle->state_machine);

    switch (state) {
        case IDLE_START:
            foc_set_charge_boot_cap_delay(phandle, CHARGE_BOOT_CAP_TICKS);
            stm_next_state(&phandle->state_machine, CHARGE_BOOT_CAP);
            break;

        case CHARGE_BOOT_CAP:
            LOG_DBG("---- CHARGE_BOOT_CAP ----\n");
            if (foc_charge_boot_cap_delay_has_elapsed(phandle)) {
                pwmc_current_reading_calibr(phandle->motor_control_driver.ptr_pwm_control, CRC_START);
                stm_next_state(&phandle->state_machine, OFFSET_CALIB);
            }
            break;

        case OFFSET_CALIB:
            LOG_DBG("---- OFFSET_CALIB ----\n");
            if (pwmc_current_reading_calibr(phandle->motor_control_driver.ptr_pwm_control, CRC_EXEC)) {
                stm_next_state(&phandle->state_machine, CLEAR);
            }
            break;

        case CLEAR:
            LOG_DBG("---- CLEAR ----\n");
            if (stm_next_state(&phandle->state_machine, START) == true) {
                foc_clear(phandle);
                r3_2_switch_on_pwm(phandle->motor_control_driver.ptr_pwm_control);
            }
            break;

        case START:
            stm_next_state(&phandle->state_machine, SWITCH_OVER);
            break;

        case SWITCH_OVER:
            stm_next_state(&phandle->state_machine, START_RUN);
            break;

        case START_RUN:
            stm_next_state(&phandle->state_machine, RUN);
            break;

        case RUN:
            mci_exec_buffered_commands(&phandle->motor_control_interface);
            if (!is_speed_reliable) {
                stm_fault_processing(&phandle->state_machine, MC_SPEED_FDBK, 0);
            }
            break;

        case ANY_STOP:
            r3_2_switch_off_pwm(phandle->motor_control_driver.ptr_pwm_control);
            foc_clear(phandle);
            mpm_clear((motor_pow_meas_t*)&phandle->motor_power_meas);
            foc_set_stop_permanency_time(phandle, STOPPERMANENCY_TICKS);
            stm_next_state(&phandle->state_machine, STOP);
            break;

        case STOP:
            if (foc_stop_permanency_time_has_elapsed(phandle)) {
                stm_next_state(&phandle->state_machine, STOP_IDLE);
            }
            break;

        case STOP_IDLE:
            vss_clear(&phandle->virtual_speed_sensor); /* Reset measured speed in IDLE */
            stm_next_state(&phandle->state_machine, IDLE);
            break;

        default:
            break;
    }
}

/**
 * @brief  foc calc current reference
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_calc_curr_ref(foc_t* phandle)
{
    if (phandle->foc_vars.drive_input == INTERNAL) {
        phandle->foc_vars.teref = stc_calc_torque_reference(&phandle->speedn_torq_ctrl);
        phandle->foc_vars.iqd_ref.q = phandle->foc_vars.teref;
    }
}

/**
 * @brief  foc high frequency task
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_high_frequency_task(foc_t* phandle)
{
    uint16_t foc_return;

    abs_encoder_calc_el_angle(&phandle->abs_encoder);

    switch (phandle->mode) {
        case OPEN_LOOP_MODE:
            foc_open_loop_controller(phandle);
            break;
        case POSITION_LOOP_MODE:
            foc_position_controller(phandle);
            foc_speed_controller(phandle);
            break;
        case SPEED_LOOP_MODE:
            foc_speed_controller(phandle);
            break;
        case TORQ_LOOP_MODE:
            foc_torq_controller(phandle);
            break;
        default:
            LOG_INFO("Not support mode\n");
            break;
    }

    foc_return = foc_curr_controller(phandle);

    if (foc_return == MC_FOC_DURATION) {
        stm_fault_processing(&phandle->state_machine, MC_FOC_DURATION, 0);
    }
}

/**
 * @brief  foc exec svpwm controller
 * @param  phandle: handler of the foc instance
 * @retval None
 */
uint16_t foc_svpwm_controller(foc_t* phandle, qd_t vqd, int16_t el_angle)
{
    uint16_t code_error;
    alphabeta_t valphabeta;

    valphabeta = mcm_rev_park(vqd, el_angle);
    code_error = pwmc_set_phase_voltage(phandle->motor_control_driver.ptr_pwm_control, valphabeta);

    phandle->foc_vars.valphabeta = valphabeta;

    return code_error;
}

/**
 * @brief  foc current controller
 * @param  phandle: handler of the foc instance
 * @retval None
 */
uint16_t foc_curr_controller(foc_t* phandle)
{
    qd_t iqd, vqd;
    ab_t iab;
    alphabeta_t ialphabeta;

    static int16_t el_angle;
    uint16_t code_error;
    speedn_pos_fdbk_t* speed_handle;

    speed_handle = stc_get_speed_sensor(&phandle->speedn_torq_ctrl);
    el_angle = spd_get_el_angle(speed_handle);
    el_angle += spd_get_inst_el_speed_dpp(speed_handle) * phandle->park_angle_compensation_factor;

    if (phandle->use_current_loop) {
        pwmc_get_phase_currents(phandle->motor_control_driver.ptr_pwm_control, &iab);
        rcm_exec_next_conv();
        ialphabeta = mcm_clarke(iab);
        iqd = mcm_park(ialphabeta, el_angle);
        vqd.q = pi_float_controller(&phandle->pid_iq, (phandle->foc_vars.iqd_ref.q) - iqd.q);
        vqd.d = pi_float_controller(&phandle->pid_id, (phandle->foc_vars.iqd_ref.d) - iqd.d);
    } else {
        vqd = phandle->foc_vars.vqd;
    }

    vqd = circle_limitation(&phandle->circle_limit, vqd);
    el_angle += spd_get_inst_el_speed_dpp(speed_handle) * phandle->rev_park_angle_compensation_factor;

    code_error = foc_svpwm_controller(phandle, vqd, el_angle);

    if (phandle->use_current_loop)
        rcm_read_ongoing_conv();
    phandle->foc_vars.vqd = vqd;
    phandle->foc_vars.iab = iab;
    phandle->foc_vars.ialphabeta = ialphabeta;
    phandle->foc_vars.iqd = iqd;
    phandle->foc_vars.el_angle = el_angle;
    return code_error;
}

/**
 * @brief  foc exec open loop controller
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_open_loop_controller(foc_t* phandle)
{
    static qd_t vqd;
    static int16_t el_angle = 0;

    vqd.q = 18000;
    vqd.d = 0;
    el_angle += 10;

    foc_svpwm_controller(phandle, vqd, el_angle);
}

/**
 * @brief It executes the core of FOC drive that is the controllers for iqd
 *        currents regulation. Reference frame transformations are carried out
 *        accordingly to the active speed sensor. It must be called periodically
 *        when new motor currents have been converted
 * @param this related object of class CFOC.
 * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
 *         next PWM Update event, MC_FOC_DURATION otherwise
 */
void foc_speed_controller(foc_t* phandle)
{
    volatile float h_err;
    int16_t h_ret;

    if (phandle->use_current_loop) {
        qd_t* iqd = &phandle->foc_vars.iqd_ref;
        h_err = (float)phandle->position.target_rpm - phandle->position.current_rpm; // current_rpmFilter  current_rpm
        h_ret = pi_float_controller(&phandle->pid_speed, h_err);

        /* adjust direct */
        iqd->q = (int16_t)h_ret * phandle->direction;
        iqd->d = 0;
    } else {
        qd_t* vqd = &phandle->foc_vars.vqd;
        h_err = (float)phandle->position.target_rpm - phandle->position.current_rpm; // current_rpmFilter  current_rpm
        h_ret = pi_float_controller(&phandle->pid_speed, h_err);

        /* adjust direct */
        vqd->q = (int16_t)h_ret * phandle->direction;
        vqd->d = 0;
    }
}

/**
 * @brief It executes the core of FOC drive that is the controllers for torq
 *        currents regulation. Reference frame transformations are carried out
 *        accordingly to the active speed sensor. It must be called periodically
 *        when new motor currents have been converted
 * @param this related object of class CFOC.
 * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
 *         next PWM Update event, MC_FOC_DURATION otherwise
 */
void foc_torq_controller(foc_t* phandle)
{
    int16_t h_ret;

    if (phandle->use_current_loop) {
        qd_t* iqd = &phandle->foc_vars.iqd_ref;
        h_ret = pid_float_controller(&phandle->pid_integer_torq, phandle->position.error_pos);

        /* adjust direct */
        iqd->q = (int16_t)h_ret * phandle->direction;
        iqd->d = 0;
    } else {
        qd_t* vqd = &phandle->foc_vars.vqd;
        h_ret = pid_float_controller(&phandle->pid_integer_torq, phandle->position.error_pos);

        /* adjust direct */
        vqd->q = (int16_t)h_ret * phandle->direction;
        vqd->d = 0;
    }
}

/**
 * @brief It executes the core of FOC drive that is the controllers for iqd
 *        currents regulation. Reference frame transformations are carried out
 *        accordingly to the active speed sensor. It must be called periodically
 *        when new motor currents have been converted
 * @param phandle related object of class CFOC.
 * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
 *         next PWM Update event, MC_FOC_DURATION otherwise
 */
void foc_position_controller(foc_t* phandle)
{
    int32_t target_speed = 0;

    target_speed = pid_float_controller(&phandle->pid_position, phandle->position.error_pos);

    position_set_target_speed(&phandle->position, target_speed);
}

/**
 * @brief  foc get varis
 * @param  phandle: handler of the foc instance
 * @retval None
 */
foc_vars_t* foc_get_varis(foc_t* phandle)
{
    return &phandle->foc_vars;
}

/**
 * @brief  foc get motor control driver
 * @param  phandle: handler of the foc instance
 * @retval None
 */
motor_control_driver_t* foc_get_motor_control_driver(foc_t* phandle)
{
    return &phandle->motor_control_driver;
}

/**
 * @brief  foc get motor control interface
 * @param  phandle: handler of the foc instance
 * @retval None
 */
motor_control_interface_t* foc_get_motor_control_interface(foc_t* phandle)
{
    return &phandle->motor_control_interface;
}

/**
 * @brief  foc set charge boot cap delay
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_set_charge_boot_cap_delay(foc_t* phandle, uint16_t tick_count)
{
    phandle->boot_cap_delay_counter = tick_count;
}

/**
 * @brief  foc charge boot cap delay has elasped
 * @param  phandle: handler of the foc instance
 * @retval None
 */
bool foc_charge_boot_cap_delay_has_elapsed(foc_t* phandle)
{
    return (phandle->boot_cap_delay_counter == 0 ? false : true);
}

/**
 * @brief  foc set stop permanency time
 * @param  phandle: handler of the foc instance
 * @retval None
 */
void foc_set_stop_permanency_time(foc_t* phandle, uint16_t tick_count)
{
    phandle->stop_permanency_counter = tick_count;
}

/**
 * @brief  foc stop permanency time has elasped
 * @param  phandle: handler of the foc instance
 * @retval None
 */
bool foc_stop_permanency_time_has_elapsed(foc_t* phandle)
{
    return (phandle->stop_permanency_counter == 0 ? true : false);
}
