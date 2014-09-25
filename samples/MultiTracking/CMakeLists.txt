cmake_minimum_required(VERSION 2.8)

project(MultiTracking)
set(EXECUTABLE_OUTPUT_PATH bin)

# ARAM include
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
set(ARAM_DIR CACHE PATH "ARAM install path")
find_package(ARAM REQUIRED)
include_directories(${ARAM_INCLUDE_DIR})

# OpenCV include
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sources files
file(GLOB_RECURSE source_files src/* include/*)

add_executable(${CMAKE_PROJECT_NAME} ${source_files})


# linker
target_link_libraries(${CMAKE_PROJECT_NAME} ${ARAM_LIBRARY} ${OpenCV_LIBS})