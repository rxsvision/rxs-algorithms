cmake_minimum_required(VERSION 3.16)
project(rxs_hole_depth LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: PCL only
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS common io filters kdtree segmentation)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_hole_depth STATIC
    hole_depth.cpp
    hole_depth.h
)

target_include_directories(rxs_hole_depth PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(rxs_hole_depth PUBLIC
    ${PCL_LIBRARIES}
)

target_compile_definitions(rxs_hole_depth PUBLIC
    ${PCL_DEFINITIONS}
)

# ---------------------------------------------------------------------------
# Optional: Build as DLL
# ---------------------------------------------------------------------------
option(HOLE_DEPTH_BUILD_SHARED "Build as shared library (DLL)" OFF)
if(HOLE_DEPTH_BUILD_SHARED)
    set_target_properties(rxs_hole_depth PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
    target_compile_definitions(rxs_hole_depth PRIVATE HOLE_DEPTH_BUILD_DLL)
endif()

# ---------------------------------------------------------------------------
# Optional: Test executable
# ---------------------------------------------------------------------------
option(HOLE_DEPTH_BUILD_TESTS "Build test program" OFF)
if(HOLE_DEPTH_BUILD_TESTS)
    add_executable(hole_depth_test test_main.cpp)
    target_link_libraries(hole_depth_test PRIVATE rxs_hole_depth)
endif()

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_hole_depth
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES hole_depth.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)
