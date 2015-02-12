cmake_minimum_required(VERSION 2.8)

project(TrackingOpenGL)
set(EXECUTABLE_OUTPUT_PATH bin)

# ARAM include
#include_directories("C:/Users/bscheurer/Documents/Projets/TrackingTag/ARAM/include")
#link_directories("C:/Users/bscheurer/Documents/Projets/TrackingTag/build32/lib/Release")
# ARAM include
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
set(ARAM_DIR CACHE PATH "ARAM install path")
find_package(ARAM REQUIRED)
include_directories(${ARAM_INCLUDE_DIR})


# OpenGL include
#include_directories("C:/Users/bscheurer/Documents/Projets/GLUT/include")
#link_directories("C:/Users/bscheurer/Documents/Projets/GLUT/lib")
#include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

# OpenCV include
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sources files
file(GLOB_RECURSE source_files src/* include/*)

add_executable(${CMAKE_PROJECT_NAME} ${source_files})

# linker
target_link_libraries(${CMAKE_PROJECT_NAME} ${ARAM_LIBRARY} ${OpenCV_LIBS})
#target_link_libraries(${CMAKE_PROJECT_NAME} ${OpenCV_LIBS})
#target_link_libraries(${CMAKE_PROJECT_NAME} ARAM)
#target_link_libraries(${CMAKE_PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
