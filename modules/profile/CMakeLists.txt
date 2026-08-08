cmake_minimum_required(VERSION 3.16)
project(rxs_profile LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: PCL, Eigen
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS common io filters segmentation surface kdtree search)

# Optional: PCL visualization (VTK-based) — only when RXS_ENABLE_VISUALIZATION=ON
if(RXS_ENABLE_VISUALIZATION)
    find_package(PCL COMPONENTS visualization)
    if(PCL_visualization_FOUND)
        target_compile_definitions(rxs_profile PUBLIC RXS_HAS_VISUALIZATION)
    endif()
endif()

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_profile STATIC
    profile.cpp
)

target_include_directories(rxs_profile PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(rxs_profile PUBLIC
    ${PCL_LIBRARIES}
)

target_compile_definitions(rxs_profile PUBLIC
    ${PCL_DEFINITIONS}
)

# ---------------------------------------------------------------------------
# Test executable (optional)
# ---------------------------------------------------------------------------
option(RXS_PROFILE_BUILD_TESTS "Build profile test executable" OFF)
if(RXS_PROFILE_BUILD_TESTS)
    add_executable(profile_test test_main.cpp)
    target_link_libraries(profile_test PRIVATE rxs_profile)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_profile
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES profile.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)
