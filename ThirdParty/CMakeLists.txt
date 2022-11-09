# set the build type
if(NOT EIGEN)
    set(EIGEN "OFF")
endif(NOT EIGEN)

if(EIGEN MATCHES "ON")
    message(STATUS "Build ThirdParty: Eigen")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Eigen)
endif()


