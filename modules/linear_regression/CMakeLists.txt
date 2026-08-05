cmake_minimum_required(VERSION 3.16)
project(rxs_linear_regression LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: Eigen3 (header-only, no PCL needed)
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_linear_regression STATIC
    linear_regression.cpp
)

target_include_directories(rxs_linear_regression PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# Test executable (optional)
# ---------------------------------------------------------------------------
option(RXS_LINEAR_REGRESSION_BUILD_TESTS "Build linear regression test executable" OFF)
if(RXS_LINEAR_REGRESSION_BUILD_TESTS)
    add_executable(linear_regression_test test_main.cpp)
    target_link_libraries(linear_regression_test PRIVATE rxs_linear_regression)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_linear_regression
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES linear_regression.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)