cmake_minimum_required(VERSION 2.8)

project(FirstApp)
set(EXECUTABLE_OUTPUT_PATH bin)

# ARAM include
include_directories("C:/Tracking/ARAM/include")
link_directories("C:/Tracking/ARAM/build/lib/Debug")

# OpenCV includes
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sources files
file(GLOB_RECURSE source_files src/* include/*)

add_executable(${CMAKE_PROJECT_NAME} ${source_files})


# linker
target_link_libraries(${CMAKE_PROJECT_NAME} ARAM)
target_link_libraries(${CMAKE_PROJECT_NAME} ${OpenCV_LIBS})