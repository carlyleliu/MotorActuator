cmake_minimum_required(VERSION 3.18)
project(Robotchissis C CXX ASM)

# adjust project folder name if necessary
set(PROJ_FOLDER Application/Robotchassis)

message(STATUS "Building project: ${PROJECT_NAME}")
set(STM32_CMAKE_DIR "${CMAKE_SOURCE_DIR}")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# set the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS "Build type: Debug")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "Build type: Release")
endif()

# # Setup cross-compile toolchain
set(CROSS_COMPILE_PREFIX ${STM32_CMAKE_DIR}/toolchain/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi)
include(${STM32_CMAKE_DIR}/cmake/gcc-arm-none-eabi.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_FOLDER}/target-def.cmake)

# # auto-set variables from user input
set(MCU_FLAGS "${CPU} -mthumb ${FPU} ${FLOAT_ABI}")
set(LINK_FLAGS "${MCU_FLAGS}")
set(EXTRA_LINK_FLAGS "-specs=nano.specs -lc -lm -lnosys -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections -Wl,--print-memory-usage")
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32-gcc-flags.cmake)

# print summary of configurations
message(STATUS "MCU_FAMILY: ${MCU_FAMILY}")
message(STATUS "MCU_NAME: ${MCU_NAME}")
message(STATUS "Specified C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Linker script: ${LINKER_SCRIPT}")
message(STATUS "Use LL library: ${USE_LL_LIB}")
message(STATUS "Use HAL library: ${USE_HAL_LIB}")
message(STATUS "Use Segger SystemView library: ${USE_SYSTEM_VIEW}")

# # version information
set(VERSION_MAJOR 0 CACHE STRING "Project major version number.")
set(VERSION_MINOR 1 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_FOLDER}/Inc)

if(USE_SYSTEM_VIEW)
    add_definitions("-DENABLE_SYSTEMVIEW")
endif()

# # add project components
set(ELF_TARGET ${PROJECT_NAME}.elf)
set(PROJ_DRIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Drivers)
set(PROJ_FRAMEWORK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Framework)
set(PROJ_OS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/OS)
set(PROJ_THIRDPART_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)
set(PROJ_MIDDLEWARE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares)

# project-specific modules
# add_subdirectory(driver)

# common libraries
add_subdirectory(${PROJ_DRIVE_PATH}/CMSIS ${CMAKE_BINARY_DIR}/Drivers/CMSIS)
add_subdirectory(${PROJ_DRIVE_PATH}/STM32G4xx_HAL_Driver ${CMAKE_BINARY_DIR}/Drivers/STM32G4xx_HAL_Driver)
add_subdirectory(${PROJ_DRIVE_PATH}/Device ${CMAKE_BINARY_DIR}/Drivers/Device)
add_subdirectory(${PROJ_FRAMEWORK_PATH} ${CMAKE_BINARY_DIR}/Framework)
add_subdirectory(${PROJ_OS_PATH} ${CMAKE_BINARY_DIR}/OS)
add_subdirectory(${PROJ_THIRDPART_PATH} ${CMAKE_BINARY_DIR}/ThirdParty)
add_subdirectory(${PROJ_MIDDLEWARE_PATH} ${CMAKE_BINARY_DIR}/Middlewares)

# APPLICATION_SRC generated source files
set(APPLICATION_SRC
    ${PROJ_FOLDER}/Src/main.c
    ${PROJ_FOLDER}/Src/communicate.c
    ${PROJ_FOLDER}/Src/peripherals.c
    ${PROJ_FOLDER}/Src/systicks.c
    ${PROJ_FOLDER}/Src/stm32g4xx_hal_msp.c
    ${PROJ_FOLDER}/Src/system_stm32g4xx.c
    ${PROJ_FOLDER}/Src/stm32g4xx_it.c
    ${PROJ_FOLDER}/startup_stm32g431xx.s
)

add_executable(${ELF_TARGET} ${APPLICATION_SRC})

link_directories(${CMAKE_BINARY_DIR}/Drivers/STM32G4xx_HAL_Driver)
link_directories(${CMAKE_BINARY_DIR}/Drivers/CMSIS)

target_link_options(${ELF_TARGET} PRIVATE
    -T${LINKER_SCRIPT}
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -lc
    -lm
    -lnosys
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

target_compile_definitions(${ELF_TARGET} PUBLIC
    -D${MCU_FAMILY}
    -DUSE_FULL_LL_DRIVER
    -DUSE_HAL_DRIVER
)

target_link_libraries(${ELF_TARGET} PUBLIC
    cmsis
    stm32g4xx_hal
    stm32g4xx_device
    middlewares
    framework
    threadx
    -lc
    -lm
    -lnosys
)

target_include_directories(${ELF_TARGET} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_FOLDER}/Inc>
    $<BUILD_INTERFACE:${PROJ_DRIVE_PATH}/STM32G4xx_HAL_Driver/Inc>
    $<BUILD_INTERFACE:${PROJ_THIRDPART_PATH}/Eigen>
    $<BUILD_INTERFACE:${PROJ_OS_PATH}>
)

# # add post-build targets
include(${STM32_CMAKE_DIR}/cmake/stm32-gcc-postbuild.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_FOLDER}/stm32-gcc-jlink.cmake)
