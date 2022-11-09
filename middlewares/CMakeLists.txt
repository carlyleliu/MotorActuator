set(MOTOR_CONTROL_SRC
    motor_control/motor/Src/bldc_motor.cpp
    motor_control/motor/Src/dc_motor.cpp
)

include_directories(
    ${PROJ_DRIVER_PATH}/Inc
    ${PROJ_UTIL_PATH}/Inc
    ${PROJ_UTIL_PATH}/design_patterns/Inc
    ${PROJ_ALGORITHM_PATH}/Inc
    ${PROJ_DEVICE_PATH}/Inc
)

add_library(middleware STATIC ${MOTOR_CONTROL_SRC})

target_link_libraries(middleware PUBLIC cmsis hal algorithm driver device)

target_include_directories(middleware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/motor_control/chassis/Inc>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/motor_control/motor/Inc>
    PRIVATE Src
)
