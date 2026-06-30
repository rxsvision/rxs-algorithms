cmake_minimum_required(VERSION 3.16)
project(rxs_regis LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(PCL REQUIRED COMPONENTS common io filters segmentation surface kdtree search visualization)
find_package(Eigen3 REQUIRED)

# --- Sources ---
set(SOURCES
    "main.cpp"
    "CzxRansac.cpp"
    "czxTool.cpp"
    "Hole.cpp"
)

set(HEADERS
    "CzxRansac.h"
    "czxTool.h"
    "Hole.h"
)

# --- Executable ---
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})

# MSVC settings
if(MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    target_compile_options(${PROJECT_NAME} PRIVATE /W3 /utf-8)
endif()
