cmake_minimum_required(VERSION 3.16)
project(rxs_coplanarity LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: PCL
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS common io filters)

# Optional: PCL visualization (VTK-based) - only when RXS_ENABLE_VISUALIZATION=ON

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_coplanarity STATIC
    coplanarity.cpp
)

# Optional: PCL visualization (VTK-based) - only when RXS_ENABLE_VISUALIZATION=ON
# NOTE: target_compile_definitions must come AFTER add_library
if(RXS_ENABLE_VISUALIZATION)
    find_package(PCL COMPONENTS visualization)
    if(PCL_visualization_FOUND)
        target_compile_definitions(rxs_coplanarity PUBLIC RXS_HAS_VISUALIZATION)
    endif()
endif()
target_include_directories(rxs_coplanarity PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(rxs_coplanarity PUBLIC
    ${PCL_LIBRARIES}
)

target_compile_definitions(rxs_coplanarity PUBLIC
    ${PCL_DEFINITIONS}
)

# ---------------------------------------------------------------------------
# Test executable (optional)
# ---------------------------------------------------------------------------
option(RXS_COPLANARITY_BUILD_TESTS "Build coplanarity test executable" OFF)
if(RXS_COPLANARITY_BUILD_TESTS)
    add_executable(coplanarity_test test_main.cpp)
    target_link_libraries(coplanarity_test PRIVATE rxs_coplanarity)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_coplanarity
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES coplanarity.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)
