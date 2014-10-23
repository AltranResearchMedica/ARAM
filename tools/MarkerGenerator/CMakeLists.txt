cmake_minimum_required(VERSION 2.8)

# tag dictonnary generator
project(ARAMMarkTagGenerator)
set(EXECUTABLE_OUTPUT_PATH bin)

# OpenCV include
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sources files
file(GLOB_RECURSE source_files src/* include/*)

add_executable(${CMAKE_PROJECT_NAME} ${source_files})

# linker
target_link_libraries(${CMAKE_PROJECT_NAME} ${OpenCV_LIBS})