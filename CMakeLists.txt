cmake_minimum_required(VERSION 3.18)
project(MotorActuator C CXX ASM)

# adjust project folder name if necessary
set(PROJ_PATH app)

message(STATUS "Building project: ${PROJECT_NAME}")
set(CMAKE_ROOT_DIR "${CMAKE_SOURCE_DIR}")
set(SOC_PLAT STM32G4)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

# set the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS "Build type: Debug")
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "Build type: Release")
endif()

# # add project components
set(ELF_TARGET ${PROJECT_NAME}.elf)
set(PROJ_SOC_PLAT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/hal/${SOC_PLAT})
set(PROJ_CMSIS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/hal/${SOC_PLAT}/CMSIS)
set(PROJ_HAL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/hal/${SOC_PLAT}/STM32G4xx_HAL_Driver)

set(PROJ_DRIVER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/driver)
set(PROJ_DEVICE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/device)
set(PROJ_ALGORITHM_PATH ${CMAKE_CURRENT_SOURCE_DIR}/algorithm)
set(PROJ_FRAMEWORK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/framework)
set(PROJ_OS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/os)
set(PROJ_THIRDPART_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
set(PROJ_MIDDLEWARE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/middlewares)
set(PROJ_UTIL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/util)
set(PROJ_TEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test)

# # Setup cross-compile toolchain
set(CROSS_COMPILE_PREFIX ${CMAKE_ROOT_DIR}/toolchain/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi)
include(${CMAKE_ROOT_DIR}/cmake/gcc-arm-none-eabi.cmake)
include(${CMAKE_ROOT_DIR}/cmake/target-def.cmake)

# # auto-set variables from user input
set(MCU_FLAGS "${CPU} -mthumb ${FPU} ${FLOAT_ABI}")
set(LINK_FLAGS "${MCU_FLAGS}")
set(EXTRA_LINK_FLAGS "-specs=nano.specs -lc -lm -lnosys -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections -Wl,--print-memory-usage")
include(${CMAKE_ROOT_DIR}/cmake/stm32-gcc-flags.cmake)

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

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_PATH}/Inc)
include_directories(${PROJ_SOC_PLAT_PATH}/Inc)

if(USE_SYSTEM_VIEW)
    add_definitions("-DENABLE_SYSTEMVIEW")
endif()

# project-specific modules
# add_subdirectory(driver)

# common libraries
add_subdirectory(${PROJ_CMSIS_PATH} ${CMAKE_BINARY_DIR}/Drivers/CMSIS)
add_subdirectory(${PROJ_HAL_PATH} ${CMAKE_BINARY_DIR}/Drivers/STM32G4xx_HAL_Driver)
add_subdirectory(${PROJ_THIRDPART_PATH} ${CMAKE_BINARY_DIR}/thirdparty)
add_subdirectory(${PROJ_DRIVER_PATH} ${CMAKE_BINARY_DIR}/drivers)
add_subdirectory(${PROJ_DEVICE_PATH} ${CMAKE_BINARY_DIR}/device)
add_subdirectory(${PROJ_UTIL_PATH} ${CMAKE_BINARY_DIR}/util)
add_subdirectory(${PROJ_ALGORITHM_PATH} ${CMAKE_BINARY_DIR}/algorithm)
add_subdirectory(${PROJ_MIDDLEWARE_PATH} ${CMAKE_BINARY_DIR}/middlewares)
add_subdirectory(${PROJ_FRAMEWORK_PATH} ${CMAKE_BINARY_DIR}/framework)

# add_subdirectory(${PROJ_OS_PATH} ${CMAKE_BINARY_DIR}/os)

# APPLICATION_SRC generated source files
set(APPLICATION_SRC
    ${PROJ_PATH}/Src/main.cpp
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
    util
    algorithm
    framework
    -lc
    -lm
    -lnosys
)

target_include_directories(${ELF_TARGET} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_PATH}/Inc>
    $<BUILD_INTERFACE:${PROJ_SOC_PLAT_PATH}/STM32G4xx_HAL_Driver/Inc>
    $<BUILD_INTERFACE:${PROJ_THIRDPART_PATH}/Eigen>
    $<BUILD_INTERFACE:${PROJ_OS_PATH}>
)

# # add post-build targets
include(${CMAKE_ROOT_DIR}/cmake/stm32-gcc-postbuild.cmake)
include(${CMAKE_ROOT_DIR}/cmake/stm32-gcc-jlink.cmake)
