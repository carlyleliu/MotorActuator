set(MOTOR_CONTROL_SRC
    MotorControl/Src/foc.c
    MotorControl/Src/motor_control_base.c
    MotorControl/Src/motor_control_driver.c
    MotorControl/Src/motor_control_interface.c
)

include_directories(
    ${PROJ_DRIVE_PATH}/CMSIS/Device/ST/STM32G4xx/Include
    ${PROJ_DRIVE_PATH}/STM32G4xx_HAL_Driver/Inc
    ${PROJ_DRIVE_PATH}/Device/Inc
    ${PROJ_DRIVE_PATH}/Device/platform
    )

add_library(framework STATIC ${MOTOR_CONTROL_SRC})

target_link_libraries(framework PUBLIC cmsis stm32g4xx_hal stm32g4xx_device middlewares)

target_include_directories(framework PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MotorControl/Inc>
    PRIVATE Src
)
