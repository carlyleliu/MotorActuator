# Add libraries

set(CMSIS_SRC
    DSP/Source/FastMathFunctions/arm_cos_f32.c
    DSP/Source/FastMathFunctions/arm_sin_f32.c
)

add_library(cmsis INTERFACE ${CMSIS_SRC})

target_compile_definitions(cmsis INTERFACE -D${MCU_FAMILY})
target_include_directories(cmsis INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Core/Include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Device/ST/STM32G4xx/Include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/DSP/Include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>)
