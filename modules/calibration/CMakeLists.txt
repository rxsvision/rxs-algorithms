cmake_minimum_required(VERSION 3.16)
project(rxs_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: PCL, Eigen
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS common io filters segmentation registration kdtree search)

# Optional: PCL visualization (VTK-based) — only when RXS_ENABLE_VISUALIZATION=ON
if(RXS_ENABLE_VISUALIZATION)
    find_package(PCL COMPONENTS visualization)
    if(PCL_visualization_FOUND)
        target_compile_definitions(rxs_calibration PUBLIC RXS_HAS_VISUALIZATION)
    endif()
endif()

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_calibration STATIC
    calibration.cpp
)

target_include_directories(rxs_calibration PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(rxs_calibration PUBLIC
    ${PCL_LIBRARIES}
)

target_compile_definitions(rxs_calibration PUBLIC
    ${PCL_DEFINITIONS}
)

# ---------------------------------------------------------------------------
# Test executable (optional)
# ---------------------------------------------------------------------------
option(RXS_CALIBRATION_BUILD_TESTS "Build calibration test executable" OFF)
if(RXS_CALIBRATION_BUILD_TESTS)
    add_executable(calibration_test test_main.cpp)
    target_link_libraries(calibration_test PRIVATE rxs_calibration)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_calibration
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES calibration.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)
