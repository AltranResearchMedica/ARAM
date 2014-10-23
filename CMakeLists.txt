cmake_minimum_required(VERSION 2.8)
# Augmented Reality Altran Medica
project(ARAM)
set(LIBRARY_OUTPUT_PATH lib)

# OpenCV include
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)


# Sources files
file(GLOB_RECURSE source_files src/* include/*)
add_library(${CMAKE_PROJECT_NAME} SHARED ${source_files})

# linker
target_link_libraries(${CMAKE_PROJECT_NAME} ${OpenCV_LIBS})

# make install
install(TARGETS ${CMAKE_PROJECT_NAME} RUNTIME DESTINATION bin
                                      LIBRARY DESTINATION lib
                                      ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)