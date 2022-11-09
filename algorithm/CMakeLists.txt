set(ALGORITHM_SRC
    Src/ahrs_deprecated.cpp
    Src/foc_controller.cpp
    Src/open_loop_controller.cpp
    Src/pid_controller.cpp
    Src/trap_traj.cpp
)

include_directories(${PROJ_DRIVER_PATH}/Inc)
include_directories(${PROJ_DEVICE_PATH}/Inc)
include_directories(${PROJ_UTIL_PATH}/Inc)
include_directories(Inc)

add_library(algorithm STATIC ${ALGORITHM_SRC})

target_link_libraries(algorithm PUBLIC driver hal cmsis)

target_include_directories(algorithm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
    PRIVATE Src)
