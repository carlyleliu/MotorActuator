set(ALGORITHM_SRC
    Src/foc_controller.cpp
    Src/open_loop_controller.cpp
    Src/pid_controller.cpp
    Src/trap_traj.cpp
)

set(ALGORITHM_EXT_SRC
    Src/ahrs_deprecated.cpp
)

include_directories(${PROJ_DRIVER_PATH}/Inc)
include_directories(${PROJ_DEVICE_PATH}/Inc)
include_directories(${PROJ_UTIL_PATH}/Inc)
include_directories(Inc)

add_library(algorithm STATIC ${ALGORITHM_SRC})

target_link_libraries(algorithm PUBLIC device)

target_include_directories(algorithm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
    PRIVATE Src)
