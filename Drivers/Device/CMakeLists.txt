set(STM32G4_DEVICE_SRC
    platform/peripherals.c
    platform/systicks.c
    Src/as5600.c
    Src/icm20602.c
    Src/mpu6050.c
    Src/tle5012b.c
    Src/usart.c
)

add_library(stm32g4xx_device STATIC ${STM32G4_DEVICE_SRC})

target_link_libraries(stm32g4xx_device PUBLIC cmsis stm32g4xx_hal)

target_include_directories(stm32g4xx_device PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/platform>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
  PRIVATE Src)
