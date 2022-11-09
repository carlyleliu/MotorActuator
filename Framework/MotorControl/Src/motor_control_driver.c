#include "motor_control_driver.h"

PLATFORM_INFO(cpu);

static uint16_t Real_bus_voltage_sensor_filter_buffer[6];

/**
 * @brief  motor control driver init
 * @param  phandle: handler of the motor_control_driver_t instance
 * @retval None
 */
void motor_control_driver_init(motor_control_driver_t* phandle, TIM_TypeDef* tim)
{
    phandle->ptr_platform_info = &cpu_info;
    /* Dual MC parameters --------------------------------------------------------*/
    phandle->r32_param.freq_ratio = 1;
    phandle->r32_param.is_higher_freq_tim = 1;
    /* Current reading A/D Conversions initialization -----------------------------*/
    phandle->r32_param.ADCx_1 = ADC1;
    phandle->r32_param.ADCx_2 = ADC2;

    /* Motor Control Kit config */
    phandle->r32_param.adc_config1[0] =
    MC_ADC_CHANNEL_14 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config1[1] =
    MC_ADC_CHANNEL_2 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config1[2] =
    MC_ADC_CHANNEL_2 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config1[3] =
    MC_ADC_CHANNEL_2 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config1[4] =
    MC_ADC_CHANNEL_2 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config1[5] =
    MC_ADC_CHANNEL_14 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);

    phandle->r32_param.adc_config2[0] =
    MC_ADC_CHANNEL_4 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config2[1] =
    MC_ADC_CHANNEL_4 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config2[2] =
    MC_ADC_CHANNEL_4 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config2[3] =
    MC_ADC_CHANNEL_14 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config2[4] =
    MC_ADC_CHANNEL_14 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);
    phandle->r32_param.adc_config2[5] =
    MC_ADC_CHANNEL_4 << ADC_JSQR_JSQ1_Pos | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT);

    phandle->r32_param.adc_data_reg1[0] = &ADC1->JDR1;
    phandle->r32_param.adc_data_reg1[1] = &ADC1->JDR1;
    phandle->r32_param.adc_data_reg1[2] = &ADC1->JDR1;
    phandle->r32_param.adc_data_reg1[3] = &ADC1->JDR1;
    phandle->r32_param.adc_data_reg1[4] = &ADC1->JDR1;
    phandle->r32_param.adc_data_reg1[5] = &ADC1->JDR1;

    phandle->r32_param.adc_data_reg2[0] = &ADC2->JDR1;
    phandle->r32_param.adc_data_reg2[1] = &ADC2->JDR1;
    phandle->r32_param.adc_data_reg2[2] = &ADC2->JDR1;
    phandle->r32_param.adc_data_reg2[3] = &ADC2->JDR1;
    phandle->r32_param.adc_data_reg2[4] = &ADC2->JDR1;
    phandle->r32_param.adc_data_reg2[5] = &ADC2->JDR1;

    /* PWM generation parameters --------------------------------------------------*/
    phandle->r32_param.repetition_counter = 1;
    phandle->r32_param.t_after = (550 + 500) * 170 / 1000;
    phandle->r32_param.t_before = (ADC_TRIG_CONV_LATENCY_CYCLES + 6.5) * 170 / 42 + 1;
    phandle->r32_param.TIMx = tim;

    /* PWM Driving signals initialization ----------------------------------------*/
    phandle->r32_param.low_side_outputs = (low_side_outputs_function_t)LS_DISABLED;

    /* Emergency input (BKIN2) signal initialization -----------------------------*/
    phandle->r32_param.bkin2_mode = EXT_MODE;

    /* Internal OPAMP common settings --------------------------------------------*/
    phandle->r32_param.opamp_params = NULL;
    /* Internal COMP settings ----------------------------------------------------*/
    phandle->r32_param.comp_ocp_a_selection = NULL;
    phandle->r32_param.comp_ocp_a_invInput_mode = NONE;
    phandle->r32_param.comp_ocp_b_selection = NULL;
    phandle->r32_param.comp_ocp_b_invInput_mode = NONE;
    phandle->r32_param.comp_ocp_c_selection = NULL;
    phandle->r32_param.comp_ocp_c_invInput_mode = NONE;
    phandle->r32_param.dac_ocp_a_selection = NULL;
    phandle->r32_param.dac_ocp_b_selection = NULL;
    phandle->r32_param.dac_ocp_c_selection = NULL;
    phandle->r32_param.dac_channel_ocp_a = (uint32_t)0;
    phandle->r32_param.dac_channel_ocp_b = (uint32_t)0;
    phandle->r32_param.dac_channel_ocp_c = (uint32_t)0;

    phandle->r32_param.comp_ovp_selection = NULL;
    phandle->r32_param.comp_ovp_invInput_mode = NONE;
    phandle->r32_param.dac_ovp_selection = NULL;
    phandle->r32_param.dac_channel_ovp = (uint32_t)0;

    /* DAC settings --------------------------------------------------------------*/
    phandle->r32_param.dac_ocp_threshold = 23830;
    phandle->r32_param.dac_ocp_threshold = 23830;

    phandle->pwm_control_r32._Super.fct_get_phase_currents = &r3_2_get_phase_currents;
    phandle->pwm_control_r32._Super.fct_switchoff_pwm = &r3_2_switch_off_pwm;
    phandle->pwm_control_r32._Super.fct_switchon_pwm = &r3_2_switch_on_pwm;
    phandle->pwm_control_r32._Super.fct_curr_reading_calib = &r3_2_current_reading_polarization;
    phandle->pwm_control_r32._Super.fct_turnon_lowsides = &r3_2_turn_on_low_sides;
    phandle->pwm_control_r32._Super.fct_is_over_current_occurred = &r3_2_is_over_current_occurred;
    phandle->pwm_control_r32._Super.fct_ocp_set_reference_voltage = NULL;
    phandle->pwm_control_r32._Super.fct_rl_detection_mode_enable = &r3_2_rl_detection_mode_enable;
    phandle->pwm_control_r32._Super.fct_rl_detection_mode_disable = &r3_2_rl_detection_mode_disable;
    phandle->pwm_control_r32._Super.fct_rl_detection_mode_set_duty = &r3_2_rl_detection_mode_set_duty;
    phandle->pwm_control_r32._Super.t_sqrt3 = (PWM_PERIOD_CYCLES * SQRT3FACTOR) / 16384u,
    phandle->pwm_control_r32._Super.Sector = 0;
    phandle->pwm_control_r32._Super.cnt_pha = 0;
    phandle->pwm_control_r32._Super.cnt_phb = 0;
    phandle->pwm_control_r32._Super.cnt_phc = 0;
    phandle->pwm_control_r32._Super.sw_error = 0;
    phandle->pwm_control_r32._Super.turn_on_low_sides_action = false;
    phandle->pwm_control_r32._Super.off_calibr_wait_time_counter = 0;
    phandle->pwm_control_r32._Super.motor = 0;
    phandle->pwm_control_r32._Super.rl_detection_mode = false;
    phandle->pwm_control_r32._Super.ia = 0;
    phandle->pwm_control_r32._Super.ib = 0;
    phandle->pwm_control_r32._Super.ic = 0;
    phandle->pwm_control_r32._Super.dt_test = 0;
    phandle->pwm_control_r32._Super.pwm_period = PWM_PERIOD_CYCLES;
    phandle->pwm_control_r32._Super.off_calibr_wait_ticks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS) / 1000);
    phandle->pwm_control_r32._Super.dt_comp_cnt = DT_COMP_CNT;
    phandle->pwm_control_r32._Super.ton = T_ON;
    phandle->pwm_control_r32._Super.toff = T_OFF;

    phandle->pwm_control_r32.phase_a_offset = 0;
    phandle->pwm_control_r32.phase_b_offset = 0;
    phandle->pwm_control_r32.phase_c_offset = 0;
    phandle->pwm_control_r32.half_pwm_period = PWM_PERIOD_CYCLES / 2u;
    phandle->pwm_control_r32.over_current_flag = false;
    phandle->pwm_control_r32.over_voltage_flag = false;
    phandle->pwm_control_r32.brake_action_lock = false;
    phandle->pwm_control_r32.ptr_params_str = &phandle->r32_param;

    phandle->bus_voltage_sensor._Super.sensor_type = REAL_SENSOR;
    phandle->bus_voltage_sensor._Super.conversion_factor = (uint16_t)(3.3 * 16);
    phandle->bus_voltage_sensor.vbus_reg_conv.reg_adc = ADC2;
    phandle->bus_voltage_sensor.vbus_reg_conv.channel = MC_ADC_CHANNEL_11;
    phandle->bus_voltage_sensor.vbus_reg_conv.sampling_time = LL_ADC_SAMPLING_CYCLE(47);

    phandle->bus_voltage_sensor.low_pass_filter_bw = 6;
    phandle->bus_voltage_sensor.over_voltage_threshold = OVERVOLTAGE_THRESHOLD_d;
    phandle->bus_voltage_sensor.under_voltage_threshold = UNDERVOLTAGE_THRESHOLD_d;
    phandle->bus_voltage_sensor.ptr_aver_buffer = Real_bus_voltage_sensor_filter_buffer;

    r3_2_init(&phandle->pwm_control_r32);

    phandle->ptr_pwm_control = &phandle->pwm_control_r32._Super;
}
