cmake_minimum_required(VERSION 3.1)
project(untitled)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES main.cpp Frame.h  grid3D.h)
add_executable(opencv ${SOURCE_FILES})

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv ${OpenCV_LIBS})