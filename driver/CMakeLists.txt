set(DRIVER_SRC
    Src/peripherals.c
    Src/peripherals_it.c
    Src/systicks.c
    Src/tle5012b.c
    Src/usart.c
    Src/hal_msp.c
)

set(DRIVER_EXT_SRC
    Src/as5600.c
    Src/icm20602.c
    Src/mpu6050.c
)

include_directories(${PROJ_HAL_PATH}/Inc)

add_library(driver STATIC ${DRIVER_SRC})

target_link_libraries(driver PUBLIC hal)

target_include_directories(driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
    PRIVATE Src)
