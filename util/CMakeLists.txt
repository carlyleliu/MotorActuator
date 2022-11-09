# Add libraries
add_library(util INTERFACE)

target_include_directories(util INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/design_patterns/Inc>
)
