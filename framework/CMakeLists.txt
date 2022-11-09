set(FRAMEWORK_SRC
    Src/motor_control_factory.cpp
)

include_directories(
    ${PROJ_DRIVER_PATH}/Inc
    ${PROJ_UTIL_PATH}/Inc
    ${PROJ_UTIL_PATH}/design_patterns/Inc
    ${PROJ_ALGORITHM_PATH}/Inc
    ${PROJ_DEVICE_PATH}/Inc
)

add_library(framework STATIC ${FRAMEWORK_SRC})

target_link_libraries(framework PUBLIC middleware)

target_include_directories(framework PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
    PRIVATE Src
)
