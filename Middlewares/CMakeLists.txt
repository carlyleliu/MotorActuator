set(MOTOR_CONTROL_SRC
    MotorControl/Common/Src/bus_voltage_sensor.c
    MotorControl/Common/Src/circle_limitation.c
    MotorControl/Common/Src/digital_output.c
    MotorControl/Common/Src/enc_align_ctrl.c
    MotorControl/Common/Src/encoder_speed_pos_fdbk.c
    MotorControl/Common/Src/fast_div.c
    MotorControl/Common/Src/feed_forward_ctrl.c
    MotorControl/Common/Src/flux_weakening_ctrl.c
    MotorControl/Common/Src/hall_speed_pos_fdbk.c
    MotorControl/Common/Src/inrush_current_limiter.c
    MotorControl/Common/Src/max_torque_per_ampere.c
    MotorControl/Common/Src/motor_power_measurement.c
    MotorControl/Common/Src/ntc_temperature_sensor.c
    MotorControl/Common/Src/open_loop.c
    MotorControl/Common/Src/pid_regulator.c
    MotorControl/Common/Src/pid_regulator_float.c
    MotorControl/Common/Src/pqd_motor_power_measurement.c
    MotorControl/Common/Src/pwm_curr_fdbk.c
    MotorControl/Common/Src/ramp_ext_mngr.c
    MotorControl/Common/Src/r_divider_bus_voltage_sensor.c
    MotorControl/Common/Src/revup_ctrl.c
    MotorControl/Common/Src/speed_pos_fdbk.c
    MotorControl/Common/Src/speed_torq_ctrl.c
    MotorControl/Common/Src/state_machine.c
    MotorControl/Common/Src/sto_cordic_speed_pos_fdbk.c
    MotorControl/Common/Src/sto_pll_speed_pos_fdbk.c
    MotorControl/Common/Src/trajectory_ctrl.c
    MotorControl/Common/Src/virtual_bus_voltage_sensor.c
    MotorControl/Common/Src/virtual_speed_sensor.c
    MotorControl/Common/Src/mc_math.c
    MotorControl/Common/Src/abs_encoder.c
    MotorControl/Common/Src/mc_position.c
    MotorControl/Platform/Src/r3_2_g4xx_pwm_curr_fdbk.c
    MotorControl/Platform/Src/regular_conversion_manager.c
    )

set(FUSION_SRC
    Fusion/Fusion/fusion_ahrs.c
    Fusion/Fusion/fusion_bias.c
    Fusion/Fusion/fusion_compass.c
    )

include_directories(
    ${PROJ_DRIVE_PATH}/CMSIS/Device/ST/STM32G4xx/Include
    ${PROJ_DRIVE_PATH}/STM32G4xx_HAL_Driver/Inc
    ${PROJ_DRIVE_PATH}/Device/Inc
    ${PROJ_DRIVE_PATH}/Device/platform
    )

add_library(middlewares STATIC ${MOTOR_CONTROL_SRC} ${FUSION_SRC})

target_link_libraries(middlewares PUBLIC cmsis stm32g4xx_hal)

target_include_directories(middlewares PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Device/Inc>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Fusion/Fusion>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MotorControl/Common/Inc>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MotorControl/Platform/Inc>
    PRIVATE Src
    )
