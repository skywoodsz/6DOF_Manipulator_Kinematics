cmake_minimum_required(VERSION 3.20)
project(src_cpp)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(src_cpp main.cpp kuka_control.cpp kuka_control.h robotics_math.cpp robotics_math.h music.h)
