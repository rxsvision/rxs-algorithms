cmake_minimum_required(VERSION 3.16)
project(rxs_surface LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(PCL REQUIRED COMPONENTS common io filters segmentation surface kdtree search)

# Optional: PCL visualization (VTK-based) — only when RXS_ENABLE_VISUALIZATION=ON
if(RXS_ENABLE_VISUALIZATION)
    find_package(PCL COMPONENTS visualization)
    if(PCL_visualization_FOUND)
        target_compile_definitions(${PROJECT_NAME} PRIVATE RXS_HAS_VISUALIZATION)
    endif()
endif()
find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)

# --- Sources ---
set(SOURCES
    "main.cpp"
    "czxTool.cpp"
    "NurbsFit.cpp"
    "NurbsSolve.cpp"
    "NurbsSurface.cpp"
    "DerivedFittingSurface.cpp"
    "OMPSurface.cpp"
)

set(HEADERS
    "czxTool.h"
    "NurbsFit.h"
    "NurbsSolve.h"
    "NurbsSurface.h"
    "DerivedFittingSurface.h"
    "OMPSurface.h"
    "NurbsCommon.hpp"
    "nanoflann.hpp"
)

# --- Executable ---
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)

# MSVC settings
if(MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    target_compile_options(${PROJECT_NAME} PRIVATE /W3 /utf-8)
endif()
