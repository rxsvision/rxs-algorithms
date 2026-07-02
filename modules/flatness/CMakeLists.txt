cmake_minimum_required(VERSION 3.16)
project(rxs_flatness LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: PCL
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS common io filters segmentation surface kdtree search)

# Optional: PCL visualization (VTK-based) — only when RXS_ENABLE_VISUALIZATION=ON
if(RXS_ENABLE_VISUALIZATION)
    find_package(PCL COMPONENTS visualization)
    if(PCL_visualization_FOUND)
        target_compile_definitions(rxs_flatness PUBLIC RXS_HAS_VISUALIZATION)
    endif()
endif()

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_flatness STATIC
    flatness.cpp
)

target_include_directories(rxs_flatness PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(rxs_flatness PUBLIC
    ${PCL_LIBRARIES}
)

target_compile_definitions(rxs_flatness PUBLIC
    ${PCL_DEFINITIONS}
)

# ---------------------------------------------------------------------------
# Test executable (optional)
# ---------------------------------------------------------------------------
option(RXS_FLATNESS_BUILD_TESTS "Build flatness test executable" OFF)
if(RXS_FLATNESS_BUILD_TESTS)
    add_executable(flatness_test test_main.cpp)
    target_link_libraries(flatness_test PRIVATE rxs_flatness)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_flatness
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES flatness.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)
