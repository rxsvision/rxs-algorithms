cmake_minimum_required(VERSION 3.16)
project(rxs_ellipse_fitting LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: PCL + Eigen3
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS common io filters sample_consensus segmentation)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_ellipse_fitting STATIC
    ellipse_fitting.cpp
)

target_include_directories(rxs_ellipse_fitting PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(rxs_ellipse_fitting PUBLIC
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)

target_compile_definitions(rxs_ellipse_fitting PUBLIC
    ${PCL_DEFINITIONS}
)

# ---------------------------------------------------------------------------
# Test executable (optional)
# ---------------------------------------------------------------------------
option(RXS_ELLIPSE_FITTING_BUILD_TESTS "Build ellipse_fitting test executable" OFF)
if(RXS_ELLIPSE_FITTING_BUILD_TESTS)
    add_executable(ellipse_fitting_test test_main.cpp)
    target_link_libraries(ellipse_fitting_test PRIVATE rxs_ellipse_fitting)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_ellipse_fitting
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES ellipse_fitting.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)
