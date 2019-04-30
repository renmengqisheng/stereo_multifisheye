cmake_minimum_required(VERSION 2.8)

PROJECT(main)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV 3.0 REQUIRED)	
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(ocam SHARED fisheye_param.cc)
add_library(orb SHARED ORBextractor.cc)

add_executable(main main.cpp)
target_link_libraries(main ocam orb ${OpenCV_LIBS})
