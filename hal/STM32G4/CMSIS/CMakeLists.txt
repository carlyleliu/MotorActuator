# Add libraries

set(CMSIS_SRC
    DSP/Source/FastMathFunctions/arm_cos_f32.c
    DSP/Source/FastMathFunctions/arm_sin_f32.c
    DSP/Source/CommonTables/arm_common_tables.c
    DSP/Source/CommonTables/CommonTables.c
)

add_library(cmsis STATIC ${CMSIS_SRC})

target_compile_definitions(cmsis PUBLIC -D${MCU_FAMILY})
target_include_directories(cmsis PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Core/Include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Device/ST/STM32G4xx/Include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/DSP/Include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>)
