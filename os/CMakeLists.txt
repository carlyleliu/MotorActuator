# set the build type
if(NOT RTOS)
    set(RTOS threadx)
endif(NOT RTOS)

if(RTOS MATCHES threadx)
    message(STATUS "Build os: threadx")
    set(THREADX_ARCH "cortex_m4")
    set(THREADX_TOOLCHAIN "gnu")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/threadx)
endif()


