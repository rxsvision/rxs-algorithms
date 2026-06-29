cmake_minimum_required(VERSION 3.16)
project(rxs_volume_glue LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies: PCL + OpenCV
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS common io segmentation kdtree search)
find_package(OpenCV REQUIRED)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(rxs_volume_glue STATIC
    volume_glue.cpp
)

target_include_directories(rxs_volume_glue PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rxs_volume_glue PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

target_compile_definitions(rxs_volume_glue PUBLIC
    ${PCL_DEFINITIONS}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS rxs_volume_glue
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES volume_glue.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs
)
