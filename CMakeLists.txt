cmake_minimum_required(VERSION 2.8)
project(fisheye_calibrate)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(fisheye_calibrate fisheye_calibrate.cpp)
target_link_libraries(fisheye_calibrate ${OpenCV_LIBS})