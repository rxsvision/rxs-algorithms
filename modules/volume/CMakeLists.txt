cmake_minimum_required(VERSION 3.16)
project(rxs_volume LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: PCL
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS common io surface)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_volume STATIC
    volume.cpp
)

target_include_directories(rxs_volume PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(rxs_volume PUBLIC
    ${PCL_LIBRARIES}
)

target_compile_definitions(rxs_volume PUBLIC
    ${PCL_DEFINITIONS}
)

# ---------------------------------------------------------------------------
# Test executable (optional)
# ---------------------------------------------------------------------------
option(RXS_VOLUME_BUILD_TESTS "Build volume test executable" OFF)
if(RXS_VOLUME_BUILD_TESTS)
    add_executable(volume_test test_main.cpp)
    target_link_libraries(volume_test PRIVATE rxs_volume)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_volume
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES volume.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)
