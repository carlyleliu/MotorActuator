set(DEVICE_SRC
    Src/impl_adc.cpp
    Src/impl_pwm.cpp
    Src/absolute_encoder.cpp
    Src/angle_encoder_abstract.cpp
)

set(DEVICE_EXT_SRC
    Src/imu_abstract.cpp
    Src/icm42688.cpp
    Src/linear_hall_angle_encoder.cpp
)

include_directories(${PROJ_DRIVER_PATH}/Inc)
include_directories(${PROJ_UTIL_PATH}/Inc)
include_directories(${PROJ_UTIL_PATH}/design_patterns/Inc)
include_directories(${PROJ_ALGORITHM_PATH}/Inc)

add_library(device STATIC ${DEVICE_SRC})

target_link_libraries(device PUBLIC driver)

target_include_directories(device PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
    PRIVATE Src)
